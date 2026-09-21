#include "dg_build_profile.h"
/* dg_bridge.c - F2: in-process anchors, a detour, and the FPS state machine.
 *
 * Read dg_bridge.h first for the shape. What follows is, in order:
 *
 *   1. the pure state machine - no Windows, no game, no clock;
 *   2. a conservative x64 length decoder and a minimal detour built on it;
 *   3. the in-process anchor resolver, sharing shared\dg_anchors.h with the
 *      external scanner;
 *   4. the drive point installed at Action()'s PlayerPad merge seam;
 *   5. lifecycle, telemetry and the desk tests.
 *
 * The ordering is deliberate: everything above the game-touching part can be
 * compiled and tested on a machine with no MGS2 on it at all, which is the only
 * kind of evidence this phase is allowed to produce.
 */

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <float.h>
#include <math.h>
#include <tlhelp32.h>
#include <stdio.h>
#include <string.h>

#include "dg_bridge.h"
#include "dg_interact_adapter.inl"
#include "dg_radial_inventory.h"
#include "dg_arm_map.h"
#include "dg_ik.h"
#include "dg_pose.h"
#include "dg_aim_capture.h"
#include "dg_fire.h"
#include "dg_recoil.h"
#include "dg_move.h"
#include "dg_menu.h"      /* DG_MENU_PAD_ALLOWED, the only bits we may name */
#include "dg_codec_menu.h"
static volatile LONG g_codec_exit_requested, g_codec_exit_written, g_codec_exit_refused;
/* Only for DG_REC_PAIRSTATE: the bridge fills the recorder's game-side
   half in place, and the hook's camera-seam tap embeds it. No recorder
   ring, no file I/O, no ownership - just the shape of the evidence. */
#include "dg_rec.h"
#include "../shared/dg_anchors.h"

/* Last, and with the redirect on: every policy call this file makes -
   pose, IK, arm map, fire, recoil, move - dispatches through the
   hot-reload table. With no DLL staged that IS the statically linked
   builtin, so the bridge behaves byte-for-byte as before. */
#define DG_POLICY_REDIRECT
#include "dg_policy.h"

/* ======================================================= state machine === */

/* Ticks the native state gets to confirm a requested transition. CheckWatch
   runs once per player update, so half a second at 60 Hz is generous without
   being an invitation to re-request forever. */
#define DG_FPS_CONFIRM_TICKS 30
/* How stale the camera-seam status latch may be before bridge_tick stops
   trusting it. The two seams ran at roughly comparable rates in the run of
   2026-08-17 (3960 camera samples against a few thousand ticks), so a handful
   of ticks is generous; the point is only to notice if the camera hook stops
   firing entirely, not to police normal jitter. */
#define DG_LATE_STATUS_TICKS 16
/* A hand solve is produced after the hierarchy/camera pass and consumed at
   the following Action() tick.  Permit one extra tick for normal scheduling,
   but never keep replaying a command after the camera seam has disappeared. */
#define DG_HAND_COMMAND_TICKS 2
#define DG_HAND_PULL_DIVISOR 4

#define DG_HAND_FORE_TWIST_RAD (90.0 * 3.14159265358979323846 / 180.0)
#define DG_HAND_WRIST_SWING_RAD (70.0 * 3.14159265358979323846 / 180.0)
#define DG_HAND_WRIST_TWIST_RAD (55.0 * 3.14159265358979323846 / 180.0)
#define DG_WEAPON_COUNT 22
#define DG_WEAPON_MIC 12
#define DG_WEAPON_DEMO_MIC 20
/* DG_ADJ_SETTLE_TICKS lives in dg_bridge.h: the log prints it, so the number
   in the probe and the number in the report cannot drift apart. */
/* After a timeout, wait this long before offering another edge. Without it a
   state that never confirms would produce one subject edge per tick, which is
   exactly the "per-frame blind write" the plan forbids. */
#define DG_FPS_RETRY_TICKS 120

void dg_fps_init(DG_FPS_STATE *state)
{
    memset(state, 0, sizeof(*state));
    state->state = DG_FPS_OFF;
    state->reason = DG_FPS_REASON_NONE;
}

static void fps_set(DG_FPS_STATE *s, DG_FPS_STEP *o, int state, int reason)
{
    if (s->state == state && s->reason == reason) return;
    o->transitioned = 1;
    s->state = state;
    s->reason = reason;
    s->ticks = 0;
}

static void fps_edge(DG_FPS_STATE *s, DG_FPS_STEP *o, const DG_FPS_INPUT *in)
{
    o->write = DG_FPS_WRITE_SUBJECT_EDGE;
    o->write_value = (int)in->pad_subject_mask;
    s->edges++;
}

typedef struct { ULONGLONG arm; int pending; } DG_FPS_RIG;
static int fps_rig_changed(DG_FPS_RIG *r, ULONGLONG arm, int desired, int active)
{
    if (arm) {
        if (r->arm && r->arm != arm && desired && active) r->pending=1;
        r->arm=arm;
    }
    if (!desired || !active) r->pending=0;
    return r->pending;
}
static int fps_calibration_ready(const DG_FPS_STATE *s,const DG_FPS_INPUT *in,const DG_FPS_RIG *rig)
{
    return s->state==DG_FPS_ACTIVE && in->native_active && in->safe_gameplay &&
        !in->level_load && !in->native_camera_missing && rig->arm && !rig->pending &&
        !s->camera_missing_ticks;
}

void dg_fps_step(DG_FPS_STATE *s, const DG_FPS_INPUT *in, DG_FPS_STEP *out)
{
    int masks_ok;

    memset(out, 0, sizeof(*out));
    out->from_state = s->state;
    out->from_reason = s->reason;
    s->ticks++;
    if (in->safe_gameplay && !in->level_load && in->pad_subject_mask &&
        !in->mgshdfix_owner && in->mode != DG_FPS_MODE_OFF && s->desired &&
        s->state == DG_FPS_ACTIVE && in->native_active && in->native_camera_missing) {
        if (s->camera_missing_ticks < DG_FPS_RETRY_TICKS) s->camera_missing_ticks++;
    } else s->camera_missing_ticks = 0;

    /* `off` must be bit-for-bit baseline behaviour, so it is the first thing
       tested and it returns before any other rule can produce a write. Handing
       the preserved values back is a separate, once-only act - see
       bridge_restore_once() - because it is not a per-tick decision. */
    if (in->mode == DG_FPS_MODE_OFF) {
        s->desired = 0;
        s->owned = 0;
        fps_set(s, out, DG_FPS_OFF, DG_FPS_REASON_MODE_OFF);
        return;
    }

    /* MGSHDFix's own FPS feature owns gBP_1stPersonCamera_* when it is on.
       Two owners of one variable is a crash or a stuck camera, never a merge,
       so this is terminal for the session: observe and report, never write. */
    if (in->mgshdfix_owner) {
        fps_set(s, out, DG_FPS_SUSPENDED, DG_FPS_REASON_MGSHDFIX_OWNER);
        return;
    }

    masks_ok = in->pad_subject_mask != 0;

    if (in->level_load || (!masks_ok && in->subject_move)) {
        fps_set(s, out, DG_FPS_SUSPENDED, DG_FPS_REASON_LEVEL_LOAD);
        return;
    }
    if (!masks_ok) {
        fps_set(s, out, DG_FPS_SUSPENDED, DG_FPS_REASON_MASK_ZERO);
        return;
    }
    if (!in->safe_gameplay) {
        /* Suspension is passive on purpose. The desired state survives, but
           nothing is written and the native camera is left exactly as the game
           put it - menus, codec, cutscenes and death own it, not us. */
        fps_set(s, out, DG_FPS_SUSPENDED, DG_FPS_REASON_UNSAFE);
        return;
    }

    if (in->mode == DG_FPS_MODE_ALWAYS) {
        s->desired = 1;
    } else if (in->toggle_request) {
        s->desired = !s->desired;
        out->request_consumed = 1;
    }

    if (s->desired || s->owned) {
        if (in->native_override != 1) {
            s->held = 1;
            out->write = DG_FPS_WRITE_OVERRIDE;
            out->write_value = 1;
            return;
        }
        if (in->native_toggle != 1) {
            s->held = 1;
            out->write = DG_FPS_WRITE_TOGGLE;
            out->write_value = 1;
            return;
        }

        if (in->move_mode != DG_FPS_MOVE_NATIVE) {
            int want = (in->move_mode == DG_FPS_MOVE_ON) ? 1 : 0;
            if (in->native_move != want) {
                s->held = 1;
                out->write = DG_FPS_WRITE_MOVE;
                out->write_value = want;
                return;
            }
        }
    } else if (s->held) {
        /* Move goes back first of the three. It has no shadow to worry about -
           the engine copies it once on entry rather than every tick - so it is
           the one value here whose order genuinely does not matter, and doing
           it first keeps the Toggle-before-Override argument below intact and
           unqualified. */
        if (in->native_move != in->saved_move) {
            out->write = DG_FPS_WRITE_MOVE;
            out->write_value = in->saved_move;
            return;
        }

        if (in->native_toggle != in->saved_toggle) {
            out->write = DG_FPS_WRITE_TOGGLE;
            out->write_value = in->saved_toggle;
            return;
        }
        if (in->native_override != in->saved_override) {
            out->write = DG_FPS_WRITE_OVERRIDE;
            out->write_value = in->saved_override;
            return;
        }
        s->held = 0;
    }

    switch (s->state) {
    case DG_FPS_REQUEST_ENTER:
        if (in->native_active) {
            /* Our edge landed, so this session is ours and we may undo it. */
            s->owned = 1;
            fps_set(s, out, DG_FPS_ACTIVE, DG_FPS_REASON_NONE);
        }
        else if (s->ticks > DG_FPS_CONFIRM_TICKS)
            fps_set(s, out, DG_FPS_SUSPENDED, DG_FPS_REASON_TIMEOUT);
        break;

    case DG_FPS_REQUEST_LEAVE:
        if (!in->native_active) {
            /* The camera is out. Dropping the claim here, and not a tick
               earlier, is what lets Override and Toggle go back next tick. */
            s->owned = 0;
            fps_set(s, out, DG_FPS_OFF, DG_FPS_REASON_NONE);
        }
        else if (s->ticks > DG_FPS_CONFIRM_TICKS)
            fps_set(s, out, DG_FPS_SUSPENDED, DG_FPS_REASON_TIMEOUT);
        break;

    case DG_FPS_ACTIVE:
        if (s->desired && s->camera_missing_ticks >= DG_FPS_RETRY_TICKS) {
            /* CheckWatch only calls IntoSubject when Active changes. A new
               level can retain Active while losing the actual camera. Ask
               the native leave/enter sequence once; keep the user's choice. */
            s->camera_missing_ticks = 0;
            fps_edge(s, out, in);
            fps_set(s, out, DG_FPS_REQUEST_LEAVE, DG_FPS_REASON_NATIVE_EXIT);
        } else if (!in->native_active) {
            /* Keep the VR toggle preference across native exits. Recovery
               waits for a quiet interval rather than fighting animations. */
            s->owned = 0;
            if (s->desired && in->mode == DG_FPS_MODE_ALWAYS) {
                fps_edge(s, out, in);
                fps_set(s, out, DG_FPS_REQUEST_ENTER, DG_FPS_REASON_NONE);
            } else {
                fps_set(s, out, s->desired ? DG_FPS_SUSPENDED : DG_FPS_OFF,
                    s->desired ? DG_FPS_REASON_NATIVE_EXIT : DG_FPS_REASON_NONE);
            }
        } else if (!s->desired && s->owned) {
            fps_edge(s, out, in);
            fps_set(s, out, DG_FPS_REQUEST_LEAVE, DG_FPS_REASON_NONE);
        }
        break;

    case DG_FPS_OFF:
    case DG_FPS_SUSPENDED:
    default:
        if (s->state == DG_FPS_SUSPENDED &&
            (s->reason == DG_FPS_REASON_TIMEOUT ||
             s->reason == DG_FPS_REASON_NATIVE_EXIT) && s->desired && !in->native_active &&
            s->ticks < DG_FPS_RETRY_TICKS)
            break;
        if (s->desired && !in->native_active) {
            fps_edge(s, out, in);
            fps_set(s, out, DG_FPS_REQUEST_ENTER, DG_FPS_REASON_NONE);
        } else if (in->native_active) {
            /* The player's first person, not ours. Reaching OFF or SUSPENDED
               means we hold no claim on the camera, so adopt the state and
               watch it - owned stays 0, so ACTIVE will not undo what it did
               not do. Forcing a leave here treated the mismatch as an error to
               correct in both directions, and since the native control is a
               hold rather than a toggle, every press of the game's own first
               person button bounced straight back to third person. */
            fps_set(s, out, DG_FPS_ACTIVE, DG_FPS_REASON_NONE);
        } else {
            fps_set(s, out, DG_FPS_OFF, DG_FPS_REASON_NONE);
        }
        break;
    }
}

/* A gap in the hand-pair stream long enough to hide a scene change. The
   fps-state dwell time is the WRONG signal for this: around a cutscene the
   machine flashes SUSPENDED for one tick and then leaves to OFF for the
   scene's duration (measured 2026-08-20 night, auto-recaptured stuck at 0),
   but the hand pairs themselves stop for exactly the stretch that matters,
   whichever states carried it. Cutscenes and codecs gap for seconds; the
   1.1-second area-transition blips sit under two seconds of ticks and must
   NOT re-anchor a hand mid gesture. Strictly greater: a boundary gap is
   still a blip. */
#define DG_HAND_REST_STALE_TICKS 120

/* No actor pointers or cached writes survive. Keep only the player's numeric
   reference, expressed against the base camera so a new room can face elsewhere. */
typedef struct {
    unsigned long id;
    double camera[4],root[4],controller[4],stick;
} DG_ARM_REFERENCE;
static void arm_reference(DG_ARM_REFERENCE *s,const DG_BRIDGE_ARM_TARGET *t,
                           double root[4],double *controller,double *stick)
{
    double camera[4],inv[4],delta[4],rebased[4];
    if(!t->calibration_id)return;
    memcpy(camera,t->calibration_camera,sizeof camera);
    if(!dg_ik_quat_normalize(camera))return;
    if(s->id!=t->calibration_id) {
        s->id=t->calibration_id;
        memcpy(s->camera,camera,sizeof camera);memcpy(s->root,root,sizeof s->root);
        if(controller)memcpy(s->controller,controller,sizeof s->controller);
        s->stick=*stick;
    } else {
        dg_ik_quat_conj(s->camera,inv);dg_ik_quat_mul(camera,inv,delta);
        dg_ik_quat_mul(delta,s->root,rebased);
        memcpy(root,rebased,sizeof rebased);
        if(controller)memcpy(controller,s->controller,sizeof s->controller);
        *stick=s->stick;
    }
}

static int rest_stale_after_gap(LONG last_pair_tick, LONG now_tick)
{
    return now_tick - last_pair_tick > DG_HAND_REST_STALE_TICKS;
}

const char *dg_fps_state_name(int state)
{
    switch (state) {
    case DG_FPS_OFF: return "OFF";
    case DG_FPS_REQUEST_ENTER: return "REQUEST_ENTER";
    case DG_FPS_ACTIVE: return "ACTIVE";
    case DG_FPS_SUSPENDED: return "SUSPENDED";
    case DG_FPS_REQUEST_LEAVE: return "REQUEST_LEAVE";
    default: return "?";
    }
}

const char *dg_fps_reason_name(int reason)
{
    switch (reason) {
    case DG_FPS_REASON_NONE: return "none";
    case DG_FPS_REASON_MODE_OFF: return "MODE_OFF";
    case DG_FPS_REASON_MASK_ZERO: return "MASK_ZERO";
    case DG_FPS_REASON_LEVEL_LOAD: return "LEVEL_LOAD";
    case DG_FPS_REASON_MGSHDFIX_OWNER: return "MGSHDFIX_OWNER";
    case DG_FPS_REASON_UNSAFE: return "UNSAFE";
    case DG_FPS_REASON_TIMEOUT: return "TIMEOUT";
    case DG_FPS_REASON_NATIVE_EXIT: return "NATIVE_EXIT";
    default: return "?";
    }
}

/* ============================================== x64 length decoder ======= */

/* Deliberately partial. Every form this does not recognise returns 0, and the
   caller refuses to patch - which is the correct answer for an unknown byte in
   code we are about to overwrite. The set covered is what MSVC actually emits
   around a mid-function seam: mov/lea/test/cmp/arith with any ModRM form, the
   push/pop and sub rsp prologue shapes, SSE loads and stores, movzx/movsx,
   setcc/cmovcc, and the immediate groups. */

typedef struct {
    unsigned length;
    unsigned rip_relative;      /* ModRM mod=00 rm=101: disp32 is RIP-relative */
    unsigned disp_offset;       /* offset of that disp32 inside the instruction */
    unsigned rel_branch;        /* rel8/rel32 jump, jcc or call */
} DG_INSN;

/* Returns the ModRM+SIB+disp length starting at p[i], or 0 if it cannot be
   determined. addr32 (a 0x67 prefix) is refused by the caller, so 64-bit
   addressing rules are the only ones needed here. */
static unsigned modrm_length(const unsigned char *p, unsigned i, unsigned avail,
                             unsigned *rip_relative, unsigned *disp_offset)
{
    unsigned char modrm;
    unsigned char mod;
    unsigned char rm;
    unsigned n = 1;

    if (i >= avail) return 0;
    modrm = p[i];
    mod = (unsigned char)(modrm >> 6);
    rm = (unsigned char)(modrm & 7);
    *rip_relative = 0;
    *disp_offset = 0;
    if (mod == 3) return 1;

    if (rm == 4) {
        unsigned char sib;
        if (i + 1 >= avail) return 0;
        sib = p[i + 1];
        n = 2;
        if (mod == 0 && (sib & 7) == 5) { *disp_offset = i + n; n += 4; }
        else if (mod == 1) { *disp_offset = i + n; n += 1; }
        else if (mod == 2) { *disp_offset = i + n; n += 4; }
    } else if (mod == 0 && rm == 5) {
        *rip_relative = 1;
        *disp_offset = i + 1;
        n = 5;
    } else if (mod == 1) {
        *disp_offset = i + 1;
        n = 2;
    } else if (mod == 2) {
        *disp_offset = i + 1;
        n = 5;
    }
    if (i + n > avail) return 0;
    return n;
}

static int dg_x64_decode(const unsigned char *p, unsigned avail, DG_INSN *out)
{
    unsigned i = 0;
    unsigned opsize16 = 0;
    unsigned rex_w = 0;
    unsigned modrm_at;
    unsigned n;
    unsigned char op;
    unsigned char op2 = 0;
    int has_modrm = 0;
    int two_byte = 0;
    unsigned immediate = 0;

    memset(out, 0, sizeof(*out));
    if (!avail) return 0;

    for (; i < avail && i < 8; i++) {
        unsigned char b = p[i];
        if (b == 0x66) { opsize16 = 1; continue; }
        if (b == 0x67) return 0;            /* 32-bit addressing: refuse */
        if (b == 0xF0 || b == 0xF2 || b == 0xF3) continue;
        if (b == 0x2E || b == 0x36 || b == 0x3E || b == 0x26 ||
            b == 0x64 || b == 0x65) continue;
        break;
    }
    if (i >= avail) return 0;
    if (p[i] >= 0x40 && p[i] <= 0x4F) {      /* REX must be last */
        rex_w = (p[i] & 8) != 0;
        i++;
    }
    if (i >= avail) return 0;
    op = p[i++];

    if (op == 0x0F) {
        if (i >= avail) return 0;
        two_byte = 1;
        op2 = p[i++];
    }

    if (two_byte) {
        if (op2 >= 0x80 && op2 <= 0x8F) {    /* jcc rel32 */
            out->rel_branch = 1;
            immediate = 4;
        } else if ((op2 >= 0x10 && op2 <= 0x17) ||
                   (op2 >= 0x28 && op2 <= 0x2F) ||
                   (op2 >= 0x40 && op2 <= 0x4F) ||
                   (op2 >= 0x51 && op2 <= 0x6F) ||
                   op2 == 0x1F ||
                   op2 == 0x7E || op2 == 0x7F ||
                   (op2 >= 0x90 && op2 <= 0x9F) ||
                   op2 == 0xA3 || op2 == 0xAB || op2 == 0xAF ||
                   op2 == 0xB0 || op2 == 0xB1 || op2 == 0xB3 ||
                   op2 == 0xB6 || op2 == 0xB7 ||
                   op2 == 0xBB || op2 == 0xBE || op2 == 0xBF ||
                   op2 == 0xC0 || op2 == 0xC1 ||
                   (op2 >= 0xD0 && op2 <= 0xEF) ||
                   (op2 >= 0xF1 && op2 <= 0xFE)) {
            has_modrm = 1;
        } else if (op2 == 0x70 || op2 == 0x71 || op2 == 0x72 || op2 == 0x73 ||
                   op2 == 0xC2 || op2 == 0xC4 || op2 == 0xC5 || op2 == 0xC6 ||
                   op2 == 0xBA) {
            has_modrm = 1;
            immediate = 1;
        } else {
            return 0;
        }
    } else if (op < 0x40 && (op & 7) < 6) {
        /* add/or/adc/sbb/and/sub/xor/cmp in all six encodings */
        switch (op & 7) {
        case 0: case 1: case 2: case 3: has_modrm = 1; break;
        case 4: immediate = 1; break;
        default: immediate = opsize16 ? 2 : 4; break;
        }
    } else if (op >= 0x50 && op <= 0x5F) {
        /* push/pop r64 */
    } else if (op == 0x63) {
        has_modrm = 1;
    } else if (op == 0x68) {
        immediate = opsize16 ? 2 : 4;
    } else if (op == 0x69) {
        has_modrm = 1;
        immediate = opsize16 ? 2 : 4;
    } else if (op == 0x6A) {
        immediate = 1;
    } else if (op == 0x6B) {
        has_modrm = 1;
        immediate = 1;
    } else if (op >= 0x70 && op <= 0x7F) {
        out->rel_branch = 1;
        immediate = 1;
    } else if (op == 0x80 || op == 0x83 || op == 0xC0 || op == 0xC1 ||
               op == 0xC6) {
        has_modrm = 1;
        immediate = 1;
    } else if (op == 0x81 || op == 0xC7) {
        has_modrm = 1;
        immediate = opsize16 ? 2 : 4;
    } else if ((op >= 0x84 && op <= 0x8B) || op == 0x8D || op == 0x8F ||
               (op >= 0xD0 && op <= 0xD3) || op == 0xFE || op == 0xFF) {
        has_modrm = 1;
    } else if (op == 0x90 || (op >= 0x91 && op <= 0x99) ||
               op == 0x9C || op == 0x9D || op == 0xC3 || op == 0xC9 ||
               op == 0xCC || op == 0xF8 || op == 0xF9 || op == 0xFC ||
               op == 0xFD) {
        /* no operands */
    } else if (op == 0xA8) {
        immediate = 1;
    } else if (op == 0xA9) {
        immediate = opsize16 ? 2 : 4;
    } else if (op >= 0xB0 && op <= 0xB7) {
        immediate = 1;
    } else if (op >= 0xB8 && op <= 0xBF) {
        immediate = rex_w ? 8 : (opsize16 ? 2 : 4);
    } else if (op == 0xC2) {
        immediate = 2;
    } else if (op == 0xE8 || op == 0xE9) {
        out->rel_branch = 1;
        immediate = 4;
    } else if (op == 0xEB) {
        out->rel_branch = 1;
        immediate = 1;
    } else if (op == 0xF6 || op == 0xF7) {
        unsigned rip;
        unsigned disp;
        unsigned char reg;
        if (i >= avail) return 0;
        reg = (unsigned char)((p[i] >> 3) & 7);
        n = modrm_length(p, i, avail, &rip, &disp);
        if (!n) return 0;
        out->rip_relative = rip;
        out->disp_offset = disp;
        i += n;
        if (reg == 0 || reg == 1) immediate = (op == 0xF6)
            ? 1 : (opsize16 ? 2 : 4);
        if (i + immediate > avail) return 0;
        out->length = i + immediate;
        return 1;
    } else {
        return 0;
    }

    if (has_modrm) {
        unsigned rip;
        unsigned disp;
        modrm_at = i;
        n = modrm_length(p, modrm_at, avail, &rip, &disp);
        if (!n) return 0;
        out->rip_relative = rip;
        out->disp_offset = disp;
        i += n;
    }
    if (i + immediate > avail) return 0;
    out->length = i + immediate;
    return out->length != 0;
}

/* ==================================================== minimal detour ===== */

#define DG_DETOUR_MAX_STOLEN 32
#define DG_DETOUR_PAGE 0x1000
#define DG_DETOUR_SCAN_MAX 0x10000

typedef struct {
    unsigned char *target;
    unsigned char original[DG_DETOUR_MAX_STOLEN];
    unsigned stolen;
    unsigned char *page;
    unsigned char *stub;
    unsigned char *tramp;
    int installed;
} DG_DETOUR;

/* Save every volatile register and the flags, because a mid-function seam has
   no ABI: the game is between two of its own instructions and anything may be
   live. xmm0-5 are included for the same reason - they are volatile, so the C
   callback is free to destroy them, and the game is free to be holding a float
   in them across our patch point. */
static const unsigned char k_stub_head[] = {
    0x9C,                                       /* pushfq                     */
    0x50, 0x51, 0x52, 0x53, 0x55, 0x56, 0x57,   /* push rax rcx rdx rbx rbp rsi rdi */
    0x41, 0x50, 0x41, 0x51, 0x41, 0x52, 0x41, 0x53,
    0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, /* push r8..r15           */
    0x48, 0x89, 0xE3,                           /* mov rbx, rsp               */
    0x48, 0x83, 0xE4, 0xF0,                     /* and rsp, -16               */
    0x48, 0x81, 0xEC, 0x80, 0x00, 0x00, 0x00,   /* sub rsp, 0x80              */
    0x0F, 0x29, 0x44, 0x24, 0x20,               /* movaps [rsp+0x20], xmm0    */
    0x0F, 0x29, 0x4C, 0x24, 0x30,
    0x0F, 0x29, 0x54, 0x24, 0x40,
    0x0F, 0x29, 0x5C, 0x24, 0x50,
    0x0F, 0x29, 0x64, 0x24, 0x60,
    0x0F, 0x29, 0x6C, 0x24, 0x70,
    0x48, 0xB8                                  /* mov rax, imm64             */
};

static const unsigned char k_stub_tail[] = {
    0xFF, 0xD0,                                 /* call rax                   */
    0x0F, 0x28, 0x44, 0x24, 0x20,               /* movaps xmm0, [rsp+0x20]    */
    0x0F, 0x28, 0x4C, 0x24, 0x30,
    0x0F, 0x28, 0x54, 0x24, 0x40,
    0x0F, 0x28, 0x5C, 0x24, 0x50,
    0x0F, 0x28, 0x64, 0x24, 0x60,
    0x0F, 0x28, 0x6C, 0x24, 0x70,
    0x48, 0x89, 0xDC,                           /* mov rsp, rbx               */
    0x41, 0x5F, 0x41, 0x5E, 0x41, 0x5D, 0x41, 0x5C,
    0x41, 0x5B, 0x41, 0x5A, 0x41, 0x59, 0x41, 0x58, /* pop r15..r8            */
    0x5F, 0x5E, 0x5D, 0x5B, 0x5A, 0x59, 0x58,   /* pop rdi rsi rbp rbx rdx rcx rax */
    0x9D,                                       /* popfq                      */
    0xFF, 0x25, 0x00, 0x00, 0x00, 0x00          /* jmp [rip+0]                */
};

/* The E9 that replaces the seam reaches +/-2 GB, so the stub has to live inside
   that window. 1.5 GB of search either way keeps a comfortable margin. */
static void *alloc_near(ULONG_PTR target, SIZE_T size)
{
    SYSTEM_INFO si;
    ULONG_PTR granularity;
    ULONG_PTR anchor;
    ULONG_PTR at;
    ULONG_PTR span = 0x60000000ULL;

    GetSystemInfo(&si);
    granularity = si.dwAllocationGranularity;
    if (!granularity) granularity = 0x10000;
    anchor = target & ~(granularity - 1);

    for (at = anchor; at > granularity && anchor - at < span; at -= granularity) {
        void *p = VirtualAlloc((void *)at, size, MEM_RESERVE | MEM_COMMIT,
                               PAGE_EXECUTE_READWRITE);
        if (p) return p;
    }
    for (at = anchor + granularity; at - anchor < span; at += granularity) {
        void *p = VirtualAlloc((void *)at, size, MEM_RESERVE | MEM_COMMIT,
                               PAGE_EXECUTE_READWRITE);
        if (p) return p;
    }
    return NULL;
}

/* Any direct branch inside the enclosing function that lands strictly inside
   the bytes we are about to replace makes the patch unsafe: the branch would
   arrive in the middle of our E9. Undecodable bytes are stepped over one at a
   time, which can only invent extra targets, never hide one - so the error is
   always towards refusing. */
static int branch_target_inside(const unsigned char *scan_begin,
                                const unsigned char *scan_end,
                                const unsigned char *lo,
                                const unsigned char *hi)
{
    const unsigned char *at = scan_begin;
    if ((SIZE_T)(scan_end - scan_begin) > DG_DETOUR_SCAN_MAX)
        scan_end = scan_begin + DG_DETOUR_SCAN_MAX;
    while (at < scan_end) {
        DG_INSN insn;
        unsigned avail = (unsigned)(scan_end - at);
        if (!dg_x64_decode(at, avail > 16 ? 16 : avail, &insn)) {
            at++;
            continue;
        }
        if (insn.rel_branch) {
            const unsigned char *dest;
            unsigned imm = (at[0] >= 0x70 && at[0] <= 0x7F) || at[0] == 0xEB
                ? 1u : 4u;
            /* A 0x0F-prefixed jcc always has rel32; the byte test above only
               matches the one-byte forms, so this stays correct for both. */
            if (imm == 1) {
                dest = at + insn.length + (signed char)at[insn.length - 1];
            } else {
                LONG rel;
                memcpy(&rel, at + insn.length - 4, sizeof(rel));
                dest = at + insn.length + rel;
            }
            if (dest > lo && dest < hi) return 1;
        }
        at += insn.length;
    }
    return 0;
}

static int suspend_others(HANDLE *threads, int max)
{
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
    THREADENTRY32 te;
    DWORD self_pid = GetCurrentProcessId();
    DWORD self_tid = GetCurrentThreadId();
    int n = 0;
    if (snap == INVALID_HANDLE_VALUE) return 0;
    memset(&te, 0, sizeof(te));
    te.dwSize = sizeof(te);
    if (Thread32First(snap, &te)) {
        do {
            HANDLE t;
            if (te.th32OwnerProcessID != self_pid) continue;
            if (te.th32ThreadID == self_tid) continue;
            if (n >= max) break;
            t = OpenThread(THREAD_SUSPEND_RESUME, FALSE, te.th32ThreadID);
            if (!t) continue;
            if (SuspendThread(t) == (DWORD)-1) { CloseHandle(t); continue; }
            threads[n++] = t;
        } while (Thread32Next(snap, &te));
    }
    CloseHandle(snap);
    return n;
}

static void resume_others(HANDLE *threads, int n)
{
    int i;
    for (i = 0; i < n; i++) {
        ResumeThread(threads[i]);
        CloseHandle(threads[i]);
    }
}

/* Writes `length` bytes over code, with every other thread in this process
   suspended so none of them can be executing a half-written instruction. */
static int patch_code(unsigned char *at, const unsigned char *bytes,
                      unsigned length)
{
    HANDLE threads[256];
    DWORD old = 0;
    DWORD ignored = 0;
    int n;
    int ok = 0;

    n = suspend_others(threads, 256);
    if (VirtualProtect(at, length, PAGE_EXECUTE_READWRITE, &old)) {
        memcpy(at, bytes, length);
        VirtualProtect(at, length, old, &ignored);
        FlushInstructionCache(GetCurrentProcess(), at, length);
        ok = 1;
    }
    resume_others(threads, n);
    return ok;
}

static void emit_jmp_abs(unsigned char *at, const void *destination)
{
    static const unsigned char jmp_rip[6] = { 0xFF, 0x25, 0, 0, 0, 0 };
    ULONGLONG value = (ULONGLONG)(ULONG_PTR)destination;
    memcpy(at, jmp_rip, sizeof(jmp_rip));
    memcpy(at + sizeof(jmp_rip), &value, sizeof(value));
}

static int dg_detour_install_ex(DG_DETOUR *d, void *target, void *callback,
                             const void *scan_begin, const void *scan_end,
                             const char **why, int original_stack)
{
    unsigned char *code = (unsigned char *)target;
    unsigned char patch[DG_DETOUR_MAX_STOLEN];
    DG_INSN insn[DG_DETOUR_MAX_STOLEN];
    unsigned offset[DG_DETOUR_MAX_STOLEN];
    unsigned count = 0;
    unsigned stolen = 0;
    unsigned i;
    unsigned char *page;
    unsigned char *tramp;
    unsigned char *stub;
    LONGLONG rel;
    ULONGLONG cb = (ULONGLONG)(ULONG_PTR)callback;

    memset(d, 0, sizeof(*d));
    *why = "unknown";

    while (stolen < 5) {
        if (count >= DG_DETOUR_MAX_STOLEN) { *why = "stolen range too long"; return 0; }
        if (!dg_x64_decode(code + stolen, 16, &insn[count])) {
            *why = "instruction at the seam is not confidently decodable";
            return 0;
        }
        if (insn[count].rel_branch) {
            *why = "a relative branch is inside the stolen range";
            return 0;
        }
        offset[count] = stolen;
        stolen += insn[count].length;
        count++;
        if (stolen > DG_DETOUR_MAX_STOLEN) { *why = "stolen range too long"; return 0; }
    }

    if (branch_target_inside((const unsigned char *)scan_begin,
                             (const unsigned char *)scan_end,
                             code, code + stolen)) {
        *why = "a branch target lands inside the patch site";
        return 0;
    }

    page = (unsigned char *)alloc_near((ULONG_PTR)code, DG_DETOUR_PAGE);
    if (!page) { *why = "no executable page within reach of a rel32 jump"; return 0; }

    tramp = page;
    stub = page + 0x80;

    memcpy(tramp, code, stolen);
    for (i = 0; i < count; i++) {
        LONGLONG new_disp;
        LONG disp;
        unsigned char *src_end;
        unsigned char *dst_end;
        if (!insn[i].rip_relative) continue;
        memcpy(&disp, code + insn[i].disp_offset + offset[i], sizeof(disp));
        src_end = code + offset[i] + insn[i].length;
        dst_end = tramp + offset[i] + insn[i].length;
        new_disp = (LONGLONG)(src_end + disp) - (LONGLONG)dst_end;
        if (new_disp > 0x7FFFFFFFLL || new_disp < -0x80000000LL) {
            VirtualFree(page, 0, MEM_RELEASE);
            *why = "RIP-relative operand cannot be relocated into the trampoline";
            return 0;
        }
        disp = (LONG)new_disp;
        memcpy(tramp + offset[i] + insn[i].disp_offset, &disp, sizeof(disp));
    }
    emit_jmp_abs(tramp + stolen, code + stolen);

    if (original_stack==2) {
        /* Function-entry replacement with a callable original trampoline. */
        emit_jmp_abs(stub,callback);
    } else {
        size_t n=sizeof(k_stub_head)-2;
        static const unsigned char stack_arg[]={0x48,0x8d,0x8b,0x80,0,0,0};
        ULONGLONG back=(ULONGLONG)(ULONG_PTR)tramp;
        memcpy(stub,k_stub_head,n);
        if(original_stack==3) {
            static const unsigned char save67[]={
                0x0f,0x29,0xb4,0x24,0x80,0,0,0,
                0x0f,0x29,0xbc,0x24,0x90,0,0,0,
                0x48,0x8d,0x94,0x24,0x80,0,0,0};
            /* k_stub_head's sub rsp,0x80 becomes 0xa0; GPR layout unchanged. */
            stub[34]=0xa0;
            memcpy(stub+n,save67,sizeof save67);n+=sizeof save67;
        }
        /* RBX points at the saved context: 15 GPRs + RFLAGS = 128 bytes.
           Set RCX only after saving its original value. */
        if(original_stack){memcpy(stub+n,stack_arg,sizeof stack_arg);n+=sizeof stack_arg;}
        memcpy(stub+n,k_stub_head+sizeof(k_stub_head)-2,2);n+=2;
        memcpy(stub+n,&cb,sizeof cb);n+=sizeof cb;
        if(original_stack==3) {
            static const unsigned char load67[]={
                0x0f,0x28,0xb4,0x24,0x80,0,0,0,
                0x0f,0x28,0xbc,0x24,0x90,0,0,0};
            memcpy(stub+n,k_stub_tail,2);n+=2; /* call before restoring XMM */
            memcpy(stub+n,load67,sizeof load67);n+=sizeof load67;
            memcpy(stub+n,k_stub_tail+2,sizeof k_stub_tail-2);n+=sizeof k_stub_tail-2;
        } else {memcpy(stub+n,k_stub_tail,sizeof k_stub_tail);n+=sizeof k_stub_tail;}
        memcpy(stub+n,&back,sizeof back);
    }

    rel = (LONGLONG)stub - (LONGLONG)(code + 5);
    if (rel > 0x7FFFFFFFLL || rel < -0x80000000LL) {
        VirtualFree(page, 0, MEM_RELEASE);
        *why = "stub is out of rel32 range after allocation";
        return 0;
    }

    memcpy(d->original, code, stolen);
    memset(patch, 0x90, sizeof(patch));
    patch[0] = 0xE9;
    { LONG r32 = (LONG)rel; memcpy(patch + 1, &r32, sizeof(r32)); }

    /* A replacement wrapper may enter as soon as patch_code resumes peers. */
    if(original_stack==2)d->tramp=tramp;
    if (!patch_code(code, patch, stolen)) {
        d->tramp=NULL;
        VirtualFree(page, 0, MEM_RELEASE);
        *why = "VirtualProtect refused the patch site";
        return 0;
    }

    d->target = code;
    d->stolen = stolen;
    d->page = page;
    d->stub = stub;
    d->tramp = tramp;
    d->installed = 1;
    *why = "ok";
    return 1;
}

static int dg_detour_install(DG_DETOUR *d,void *target,void *callback,
    const void *begin,const void *end,const char **why) {
    return dg_detour_install_ex(d,target,callback,begin,end,why,0);
}

static void dg_detour_remove(DG_DETOUR *d)
{
    if (!d->installed) return;
    patch_code(d->target, d->original, d->stolen);
    d->installed = 0;
    /* The page is leaked on purpose: another thread can still be inside the
       stub at this instant, and one 4 KB reservation for the life of the
       process is a far better trade than a use-after-free in game code. */
    d->page = NULL;
    d->stub = NULL;
    d->tramp = NULL;
}

/* ============================================ in-process anchor resolve == */

#define DG_PAD_STATUS_OFFSET 4
#define DG_PAD_PRESS_OFFSET  8
#define DG_PAD_RELEASE_OFFSET 12
/* pressure[12] at +0x18, indexed by PL_PAD_PRESS_WEAPON - which is a runtime
   index into that array, not a mask, and is read from the anchor rather than
   assumed. Twelve is the array's own length; anything outside it would be a
   write past the pad. */

#define DG_PAD_DIR_OFFSET     0x10   /* short: pad->dir, 0..4095 or -1 */
#define DG_PAD_ANALOG_OFFSET  0x12
#define DG_PAD_RIGHT_DX_OFFSET 0x14
#define DG_PAD_LEFT_DX_OFFSET 0x16
#define DG_PAD_LEFT_DY_OFFSET 0x17
#define DG_PAD_PRESSURE_OFFSET 0x18
#define DG_PAD_PRESSURE_COUNT  12
/* BP_PlayerPad.weaponState / .buttonState, one 40-byte pad above the anchor
   (F7 research SS3). Read only, and read as a witness: they are the Bluepoint
   layer's own answer to what we wrote the tick before. */
#define DG_PAD_WEAPON_STATE_OFFSET 0x28
#define DG_PAD_BUTTON_STATE_OFFSET 0x2C
#define DG_GV_PAD_PRESS_SCN 0x00000020u
#define DG_PAD_START 0x00000800u
/* Retail titlescr Step (RVA E7410) tests direct.press bit 20 at E74DE/E7604. */
#define DG_PAD_TITLE_ENTER 0x00100000u

#define DG_PLAYER_UNSAFE_MASK ( \
      0x0000000000000080ULL /* LOCKER    */ \
    | 0x0000000000000400ULL /* DOWNED    */ \
    | 0x0000000000001000ULL /* BEYOND    */ \
    | 0x0000000000002000ULL /* FORCE     */ \
    | 0x0000000000004000ULL /* CB_BOX    */ \
    | 0x0000000000008000ULL /* DEAD      */ \
    | 0x0000000000010000ULL /* LADDER    */ \
    | 0x0000000008000000ULL /* MENU_OPEN */ \
    | 0x0000000010000000ULL /* STOP      */ \
    | 0x0000020000000000ULL /* PAD_OFF   */ )

/* PLAYER_HOLD (0x800) is deliberately absent. It is the weapon-ready stance,
   not a loss of control; refusing it would stand the tracked hand down at the
   exact moment the player raises a pistol. The user approved this policy on
   2026-08-18. All actual takeover/death/menu bits above remain fail-closed. */

#define DG_GAME_UNSAFE_MASK ( \
      0x00000040UL /* STATE_CUT_IN        */ \
    | 0x00004000UL /* STATE_DISP_GAMEOVER */ \
    | 0x08000000UL /* STATE_SCN_DEMO      */ \
    | 0x10000000UL /* STATE_DEMO          */ \
    | 0x20000000UL /* STATE_PRG_DEMO      */ \
    | 0x40000000UL /* STATE_PAD_DEMO      */ \
    | 0x80000000UL /* STATE_GAMEOVER      */ )

#define DG_MENU_UNSAFE_MASK ( \
      0x00000100UL /* MENU_WEAPON_OPEN */ \
    | 0x00000200UL /* MENU_ITEM_OPEN   */ \
    | 0x00000400UL /* MENU_RADIO_ON    */ )

#define DG_THEATER_GAME_MASK ( \
      0x08000000UL   \
    | 0x10000000UL /* STATE_DEMO - polygon demos AND PSS/MPEG movies      */ \
    | 0x40000000UL /* STATE_PAD_DEMO - attract replay, player only watches */ \
    | 0x80004000UL /* STATE_GAMEOVER / STATE_DISP_GAMEOVER - flat Continue UI */ )
#define DG_THEATER_MENU_MASK ( \
      0x00000400UL /* MENU_RADIO_ON - the only bit that sees a codec */ )

/* UI-U1: the full-menu panel judgment. Separate from the theater mask so the
   marker can enable either without the other; the codec is the theater's. */
#define DG_UI_PANEL_MENU_MASK ( \
      0x00000100UL /* MENU_WEAPON_OPEN */ \
    | 0x00000200UL /* MENU_ITEM_OPEN   */ )

#define DG_HUD_HIDE_BITS ( \
      0x00000001UL /* MENU_WEAPON_OFF */ \
    | 0x00000002UL /* MENU_ITEM_OFF   */ \
    | 0x00000004UL /* MENU_RADAR_OFF  */ \
    | 0x00000008UL /* MENU_GAGE_OFF   */ )

/* Hysteresis, in camera-seam samples (the seam runs about one per frame and
   SPEEDS UP during a demo - measured 2474 seams against 465 ticks in one
   window - so ticks would be the wrong clock here). Enter fast: ~0.1 s costs
   an unnoticeable head-coupled sliver at the start of a scene the game is
   fading anyway. Exit slow: STATE_SCN_DEMO can blink across script
   transitions (pad release -> cancel -> next demo, section 2.5), and a
   theater that reappears for each blink is the flap the hysteresis exists to
   prevent. The menu panel closes faster - its bits are set and cleared
   cleanly by the menu machine, and a camera that stays frozen for a second
   after closing a weapon ring would read as a hang. */
#define DG_THEATER_ENTER_SAMPLES 8
#define DG_THEATER_EXIT_SAMPLES  45
#define DG_UI_ENTER_SAMPLES 4
#define DG_UI_EXIT_SAMPLES  12

/* Consumers refuse a publication older than this. Milliseconds rather than
   ticks, because the tick seam is exactly the clock that stops during the
   states the theater is for; GetTickCount is VEH-safe (it reads shared user
   data). A camera hook that stops firing thus fails back to today's
   behaviour within half a second instead of pinning the theater on. */
#define DG_THEATER_FRESH_MS 500

/* Pure, and separated for the same reason as fold_late_status: this is the
   decision a wrong bit would turn into a frozen screen mid-boss-fight, so it
   has to be exercisable at a desk against the measured words. */
static int dg_theater_masked(unsigned int game_status, unsigned int menu_status)
{
    return (game_status & DG_THEATER_GAME_MASK) != 0 ||
           (menu_status & DG_THEATER_MENU_MASK) != 0;
}

static int dg_ui_panel_masked(unsigned int menu_status)
{
    return (menu_status & DG_UI_PANEL_MENU_MASK) != 0;
}

/* One hysteresis instance. streak counts CONSECUTIVE samples that contradict
   the current state; any agreeing sample resets it, so a single blink can
   never accumulate its way through the threshold. Pure: no globals, no
   clock. Returns the (possibly new) active state. */
typedef struct {
    int active;
    long streak;
} DG_THEATER_HYST;

static int dg_theater_hyst_step(DG_THEATER_HYST *h, int masked,
                                int enter_n, int exit_n)
{
    if (h->active) {
        if (masked) h->streak = 0;
        else if (++h->streak >= exit_n) { h->active = 0; h->streak = 0; }
    } else {
        if (!masked) h->streak = 0;
        else if (++h->streak >= enter_n) { h->active = 1; h->streak = 0; }
    }
    return h->active;
}

static int read_self_image(LiveImage *image)
{
    unsigned char *base = (unsigned char *)GetModuleHandleA(NULL);
    IMAGE_DOS_HEADER *dos;
    IMAGE_NT_HEADERS64 *nt;
    MEMORY_BASIC_INFORMATION mbi;
    ULONG_PTR cursor;
    ULONG_PTR end;
    DWORD size;

    memset(image, 0, sizeof(*image));
    if (!base) return 0;
    dos = (IMAGE_DOS_HEADER *)base;
    if (dos->e_magic != IMAGE_DOS_SIGNATURE || dos->e_lfanew <= 0 ||
        dos->e_lfanew > 0x1000)
        return 0;
    nt = (IMAGE_NT_HEADERS64 *)(base + dos->e_lfanew);
    if (nt->Signature != IMAGE_NT_SIGNATURE ||
        nt->OptionalHeader.Magic != IMAGE_NT_OPTIONAL_HDR64_MAGIC)
        return 0;
    size = nt->OptionalHeader.SizeOfImage;
    if (!size || size > 0x40000000UL) return 0;

    image->base = (ULONGLONG)(ULONG_PTR)base;
    image->size = size;
    image->bytes = (unsigned char *)calloc(size, 1);
    image->valid = (unsigned char *)calloc(size, 1);
    if (!image->bytes || !image->valid) return 0;

    /* A snapshot, not a live view: the resolver walks these bytes for a while
       and the game keeps running. Copying only committed, readable regions is
       also what keeps this from faulting on the packed .bind gaps. */
    cursor = (ULONG_PTR)base;
    end = cursor + size;
    while (cursor < end) {
        ULONG_PTR region_start;
        ULONG_PTR region_end;
        if (!VirtualQuery((void *)cursor, &mbi, sizeof(mbi))) {
            cursor += 0x1000;
            continue;
        }
        region_start = (ULONG_PTR)mbi.BaseAddress;
        region_end = region_start + (ULONG_PTR)mbi.RegionSize;
        if (region_end <= cursor) break;
        if (region_end > end) region_end = end;
        if (region_start < (ULONG_PTR)base) region_start = (ULONG_PTR)base;
        if (mbi.State == MEM_COMMIT && protection_readable(mbi.Protect)) {
            SIZE_T offset = (SIZE_T)(region_start - (ULONG_PTR)base);
            SIZE_T length = (SIZE_T)(region_end - region_start);
            memcpy(image->bytes + offset, (const void *)region_start, length);
            memset(image->valid + offset, 1, length);
        }
        cursor = region_end;
    }
    return validate_image_headers(image);
}

static int self_fingerprint_ok(const LiveImage *image)
{
    char path[MAX_PATH];
    char sha[65];
    const char *leaf;
    DiskPe disk;

    if (!GetModuleFileNameA(NULL, path, sizeof(path))) return 0;
    leaf = strrchr(path, '\\');
    leaf = leaf ? leaf + 1 : path;
    if (_stricmp(leaf, GAME_EXE) != 0) return 0;
    if (!inspect_disk_pe(path, &disk)) return 0;
    if (!sha256_file(path, sha)) return 0;
    return fingerprint_ok(&disk, image, sha);
}

/* ================================================== the bridge itself ==== */

#define DG_RING 32

typedef struct {
    unsigned from_state;
    unsigned to_state;
    unsigned reason;
    unsigned write;
    unsigned long tick;
    /* The three status words as they were at the instant of the transition.
       A suspend line that only says UNSAFE says nothing testable: it cannot
       distinguish a codec call from a cutscene from a bit we misread. The
       heartbeat carries the same values, but it fires up to 30 s later, by
       which time the state it is meant to explain is over. */
    unsigned game_status;
    unsigned menu_status;
    unsigned long long player_status;
} DG_FPS_EVENT;

static struct {
    volatile LONG armed;
    volatile LONG mode;
    volatile LONG move_mode;
    volatile LONG arm_show;
    volatile LONG work_probe;
    volatile LONG c_arm_forced;
    volatile LONG c_arm_forced_late;
    volatile LONG c_subject_move_ticks;
    volatile LONG wrote_move;       /* Move is opt-in, so restore must ask */
    volatile LONG toggle_request;
    volatile LONG toggle_request_ms;
    volatile LONG start_pending;
    volatile LONG c_pad_seam_entries, c_start_queued, c_start_consumed;
    volatile LONG c_pad_context_refused;
    volatile LONG script_menu_only;
    volatile LONG started;

    DG_ANCHORS a;
    DG_DETOUR detour;
    /* U2: the second detour, on GV_UpdatePadSystem. Separate from the tick
       detour in every way that matters - its own install, its own failure,
       its own removal - because the pad anchor is the one OPTIONAL anchor
       and a miss there must cost the menu feature alone. */
    DG_DETOUR pad_detour;
    volatile LONG pad_detour_live;
    volatile LONG menu_mode;             /* vr_menu: 0 off, 1 measure, 2 write */
    volatile LONG menu_status;           /* the bits the seam should arm */
    volatile LONG menu_clear;            /* bits to remove first, see DG_BRIDGE_MENU */
    volatile LONG menu_allow;            /* the caller's front-end judgment */
    volatile LONG menu_stamp;            /* tick the command came from */
    /* The game's own status/press words in GV_PadDataDirect as seen at the
       pad seam, accumulated. Named for the record so it cannot be confused
       with seen_pad_status, which watches the PlayerPad copy instead. The
       only way to learn which bit this build calls PAD_OK. */
    volatile LONG seen_direct_status;
    volatile LONG seen_direct_press;
    /* Distinct press words and their counts - see the filling site for
       why a single snapshot was not enough. */
    volatile LONG press_ring_word[DG_PAD_PRESS_RING];
    volatile LONG press_ring_count[DG_PAD_PRESS_RING];
    /* The same events in order. See DG_PAD_PRESS_SEQ. */
    volatile LONG press_seq_word[DG_PAD_PRESS_SEQ];
    volatile LONG press_seq_ms[DG_PAD_PRESS_SEQ];
    volatile LONG press_seq_n;
    volatile LONG c_menu_writes;
    volatile LONG c_menu_stale;
    volatile LONG c_menu_gate;
    DG_FPS_STATE fps;
    DG_FPS_RIG fps_rig;

    int owner;                  /* latched: someone else holds the values */
    int override_fights;
    LONG always_warned;
    int saved_override;
    int saved_toggle;
    int saved_move;
    int saved_active;
    volatile LONG wrote_any;
    volatile LONG restored;

    void (*log)(const char *fmt, ...);

    volatile LONG c_ticks;
    volatile LONG c_requested;
    volatile LONG c_entered;
    volatile LONG c_explicit_entries;
    volatile LONG64 calibration_ready_arm;
    volatile LONG c_left;
    volatile LONG c_transitions;
    volatile LONG c_edges;
    volatile LONG c_writes;
    volatile LONG c_mask_zero;
    volatile LONG c_level_load;
    volatile LONG c_unsafe;
    volatile LONG c_timeouts;
    volatile LONG c_dropped;

    volatile LONG s_native_active;
    volatile LONG s_override;
    volatile LONG s_toggle;
    volatile LONG s_move;
    volatile LONG s_subject_move;
    volatile LONG s_subject_toggle;
    volatile LONG s_pad_subject;
    volatile LONG s_pad_stop_aim;
    volatile LONG s_status_lo;
    volatile LONG s_status_hi;
    volatile LONG s_game_status;
    volatile LONG s_menu_status;

    /* F4 observation only. The arm object is read, never written, and the two
       dereferences below are the reason this lives in bridge_tick rather than
       in the logger thread: the game thread is the thread that creates and
       destroys the object, so reading it from inside its own tick cannot race
       the free. The logger only ever sees the values. */
    volatile LONG s_arm_body_lo;
    volatile LONG s_arm_body_hi;
    volatile LONG s_arm_objs_lo;
    volatile LONG s_arm_objs_hi;
    volatile LONG s_arm_flag;
    volatile LONG s_arm_cam_rot_xy;
    volatile LONG s_arm_cam_rot_z;
    volatile LONG hand_probe;
    volatile LONG hand_probe_rot[3];
    /* Set the first time the probe writes, so the exit path knows to hand the
       SVECTOR back. Normally the game reclaims it by itself - ArmMove pulls it
       to zero in a dozen frames - but that only happens while the arm actor is
       running, and unloading during a cutscene would otherwise leave our angle
       sitting there for the next time the arm camera comes on. */
    volatile LONG hand_probe_owned;
    /* The wrist channel's release debt. hand_drive_write puts our command into
       ArmCamRotateShift every tick; when a stream ends, the game does not
       remove it - ArmMove only decays it 25% per frame toward the weapon
       table's {0,0,0}. The next stream's rest capture reads the hierarchy
       DG_ADJ_SETTLE_TICKS later, when 0.75^2 = 56% of our last wrist is still
       in it, and bakes that error into the new rest pair. That is a ratchet:
       measured on 2026-08-19 as a session where nine B-press recalibrations
       never unpinned the wrist envelope (200/200, then 93%, then 88% of pairs
       clamped at the full 70 degrees). So a release WRITES the zero the decay
       was drifting toward - one tick, through the same gates as every other
       wrist write, and only if this session ever wrote the channel at all. */
    volatile LONG hand_zero_pending;
    volatile LONG hand_drive_owned;
    volatile LONG c_arm_hand_zeroed;
    volatile LONG c_arm_body_uncompensated;
    volatile LONG s_hand_probe_wrote[3];
    volatile LONG s_hand_probe_read[3];
    volatile LONG s_hand_probe_adjust[4];
    volatile LONG s_hand_probe_pred[4];
    volatile LONG s_hand_probe_worst;
    volatile LONG c_hand_probe_writes;
    volatile LONG c_hand_probe_reads;
    volatile LONG c_hand_probe_no_setpos;
    volatile LONG c_arm_created;    /* null -> non-null transitions */
    volatile LONG c_arm_destroyed;
    ULONGLONG last_arm_body;

    /* The GM_PlayerBody candidate, same shape, opt-in and read-only. */
    volatile LONG s_body_lo;
    volatile LONG s_body_hi;
    volatile LONG s_body_objs_lo;
    volatile LONG s_body_objs_hi;
    volatile LONG s_body_flag;
    volatile LONG seen_body_flag;
    volatile LONG held_body_flag;
    volatile LONG c_body_visible;
    /* The arm actor's Work, and what it reaches. */
    volatile LONG s_work_lo;
    volatile LONG s_work_hi;
    volatile LONG s_pwork_lo;       /* derived: trigger - 0xCF4 */
    volatile LONG s_pwork_hi;
    volatile LONG s_cam_on;         /* the flag SetPos branches on */
    volatile LONG c_cam_on;         /* ...and the ticks it was set */
    volatile LONG s_arm_trigger;
    volatile LONG seen_arm_trigger;
    /* The same visibility word sampled at the CAMERA seam, after every actor,
       because the tick-seam copy reads before the arm actor has run. */
    volatile LONG s_arm_flag_late;
    volatile LONG seen_arm_flag_late;
    volatile LONG held_arm_flag_late;
    volatile LONG c_arm_visible_late;
    /* F5 step 1: the joint bend. */
    volatile LONG bend_deg;
    volatile LONG bend_joint;
    volatile LONG bent_joint;       /* -1 when nothing is bent */
    volatile LONG c_arm_bent;
    volatile LONG c_arm_tracked;        /* ...of which, from a controller */
    volatile LONG c_arm_quat_refused;   /* non-unit quaternions rejected */
    volatile LONG ik_active;            /* joints 4 and 5 currently ours */
    volatile LONG c_arm_ik_refused;
    volatile LONG c_arm_ik_clamped;
    volatile LONG c_arm_ik_implausible;
    /* Calibrated arm mapping is pair-owned, not seam-owned. Core state is
       touched only on the render/game thread; counters/snapshots use the same
       interlocked publication pattern as the older IK telemetry. */
    DG_ARM_MAP_STATE arm_map;
    DG_POSITION_STATE camera_position;
    DG_FREE_WRIST_STATE free_right;
    unsigned long arm_map_last_pair;
    unsigned long arm_map_stream;
    ULONGLONG arm_map_arm;
    ULONGLONG arm_map_objs;
    LONG arm_map_settle_until;
    int arm_map_unarmed; /* reset positional calibration when equip mode changes */
    int arm_map_phase;              /* 0 reset, 1 settling, 2 calibrated */
    /* Only q4 and q5 are ours. SetPos owns q6 and rebuilds it from
       ArmCamRotateShift on every arm-camera pass. */
    float arm_map_cached_adjust[8];
    int arm_map_cache_valid;
    ULONGLONG ik_owned_arm;
    ULONGLONG ik_owned_mctrl;
    ULONGLONG ik_owned_adjust;
    /* Exactly the adjust_flag bits this hook set, so release clears what it
       took and nothing else. Joint 6 comes and goes with hand tracking, and a
       hard-coded mask would either strand a bit or clear one we never owned. */
    ULONGLONG ik_owned_mask;
    /* The hand's animated rotation and the controller's, both in the arm root
       frame, captured on the same calibration pair. Everything the hand does
       afterwards is the controller's movement SINCE that moment applied to
       that pose, so no absolute agreement between the two frames is assumed -
       only that they turn the same way. */
    int    hand_have_rest;
    double hand_rest_view[4];
    double hand_ctrl_rest[4];
    DG_ARM_REFERENCE right_reference,left_reference;
    /* The arm root's rotation on that same calibration pair. The camera this
       build publishes never follows the body, so a body that yaws after
       calibration slides this root frame out from under the player's
       camera-frame input and the arm orbits the character. Every later pair
       measures the root's yaw since this capture and turns the input to
       match. */
    int    arm_root_have_q0;
    double arm_root_q0[4];
    /* What the previous pair asked the hand to become, kept so the next pass
       can measure how close it got. */
    int    hand_have_desired;
    double hand_desired[4];
    /* The adjust-space quaternion that same pair published, kept so the next
       pass can also cut the measurement at the SetPos slot: a clean echo with
       a large residual says the game took our angles and the hierarchy did
       something else with them; a dirty echo says the game never
       reconstructed our angles at all. */
    int    hand_have_last_cmd;
    double hand_last_cmd[4];
    /* The residual's offset quaternion (live times desired-inverse) from the
       previous measured pair. A large residual whose offset barely moves
       between pairs is a constant frame error - a per-weapon hand frame our
       model does not know - while an offset that wanders pair to pair is a
       composition/order error. The distinction picks the fix. */
    int    hand_have_prev_off;
    double hand_prev_off[4];
    /* The previous pair's UNblended joint swings, fed back into
       dg_ik_orient so each pair's solution is the point on its bone circle
       nearest the last one instead of a memoryless shortest arc. This is
       what stops the 175-degrees-per-pair branch flips the walking-turn
       probe measured. Reset with the stream: a new rig must not inherit an
       old rig's swivel. */
    int    arm_orient_have_prev;
    double arm_orient_prev_upper[4];
    double arm_orient_prev_fore[4];
    /* The previous pair's unwrapped raw twist, fed back into the hand
       stabilizer so its clamp branch cannot flip across the 180-degree
       seam - the crouch tumble. Reset with the stream, like the orient
       memory above. */
    int    hand_have_prev_twist;
    double hand_prev_twist_rad;
    /* And the swing's branch memory beside it: the APPLIED signed swing and
       the axis that reading was about, fed back so the clamped swing cannot
       flap between hemispheres as the raw swing crosses 180 - the body-turn
       tumble of 2026-08-21. Reset wherever the twist memory is. */
    int    hand_have_prev_swing;
    double hand_prev_swing_rad;
    double hand_prev_swing_axis[3];
    /* The fold-hysteresis references: the unwrapped RAW angles the last
       stabilize reported, fed back beside the applied ones so the unwrap
       does not flap at the cap+180 fold midpoint (the 2026-08-21 evening
       tumble). Bounded under a full turn by unwrap_branch's own rule;
       armed and reset together with the applied memories above. */
    double hand_prev_twist_raw_rad;
    double hand_prev_swing_raw_rad;
    /* Raised when the hand-pair stream gapped long enough to hide a scene
       change (a cutscene ends with the character re-posed - crouch was the
       one that shipped this), consumed by the next hand pair that can as an
       automatic B-press: the rest trio is recaptured against the CURRENT
       animation base and controller, because the old zero is testimony
       about a world that moved while nobody was watching. */
    volatile LONG hand_rest_stale;
    LONG   arm_last_hand_tick;
    volatile LONG c_arm_rest_recaptured;
    /* The previous pair's stripped animation base, for the base-drift
       telemetry: is the animation we stand on itself coherent, or are we
       reading a model a cutscene left in some other shape. */
    int    hand_have_prev_base;
    double hand_prev_base[4];
    volatile LONG s_arm_base_drift;
    volatile LONG s_arm_base_drift_worst;
    volatile LONG s_arm_cmd_drift;
    volatile LONG s_arm_cmd_drift_worst;
    /* Camera seam publishes one precompensated ArmCamRotateShift command;
       tick seam consumes a coherent snapshot. Odd seq means writer active. */
    volatile LONG hand_command_seq;
    volatile LONG hand_command_requested;
    volatile LONG hand_command_valid;
    volatile LONG hand_command_rot[3];
    volatile LONG hand_command_tick;
    volatile LONG hand_command_pair;
    /* F7. The same crossing as the hand command and for the same reason, but
       carrying sequence numbers rather than a level: the camera seam runs
       several times per tick and both eyes see one frame, so a level would
       manufacture edges that the player never made. */
    volatile LONG fire_seq;
    volatile LONG fire_valid;
    volatile LONG fire_tick;
    volatile LONG fire_press_lo, fire_press_hi;
    volatile LONG fire_release_lo, fire_release_hi;
    volatile LONG fire_value;       /* trigger travel * 1000 */
    volatile LONG fire_click;       /* threshold * 1000 */
    volatile LONG fire_stream;
    DG_FIRE fire;                   /* tick-seam only: no other thread steps it */
    volatile LONG fire_mode;
    volatile LONG c_fire_published;
    volatile LONG c_fire_no_command;
    volatile LONG c_fire_stale;
    volatile LONG c_fire_drawn;
    volatile LONG c_fire_released;
    volatile LONG c_fire_aborted;
    volatile LONG c_fire_forced;
    volatile LONG c_fire_blocked_phys;
    volatile LONG c_fire_blocked_gate;
    volatile LONG c_fire_repeats;
    volatile LONG c_fire_coasting;
    volatile LONG c_fire_auto_ticks;
    volatile LONG c_fire_wrote_press;
    volatile LONG c_fire_wrote_status;
    volatile LONG c_fire_wrote_release;
    volatile LONG c_fire_wrote_pressure;
    volatile LONG c_fire_pressure_kept;
    volatile LONG c_fire_no_index;
    volatile LONG c_fire_yielded;
    volatile LONG s_fire_index;
    volatile LONG seen_weapon_state;
    volatile LONG seen_button_state;
    volatile LONG s_weapon_state;
    volatile LONG s_button_state;
    DG_RECOIL recoil;               /* tick seam only, like DG_FIRE */
    /* F9: the walking channel. A plain level crossing - see DG_BRIDGE_MOVE. */
    volatile LONG move_seq;
    volatile LONG move_valid;
    volatile LONG move_tick_stamp;
    volatile LONG move_x;           /* stick * 1000 */
    volatile LONG move_y;
    volatile LONG move_stream;
    /* walk_, not move_: move_mode two structs up is vr_fps_move's and has
       owned the name since F4. */
    volatile LONG walk_mode;
    volatile LONG move_deadzone_mils;
    volatile LONG turn_mode;
    /* 2026-09-11: third-person walk and prone crawl. */
    volatile LONG move_third, move_prone, move_prone_max;
    volatile LONG move_dir_offset, move_dir_sign, move_dir_org;
    volatile LONG s_cam_dir;            /* seam-published camera yaw, -1 none */
    volatile LONG c_move_third_writes, c_move_not_third;
    volatile LONG c_move_prone_writes, c_move_dir_writes;
    volatile LONG s_move_last_org, s_move_last_dir;
    ULONGLONG tick_status;              /* GM_PlayerStatus at this tick */
    /* Move probe. */
    volatile LONG mp_w_tick, mp_w_dir, mp_w_status, mp_w_bytes;
    volatile LONG mp_c_tick, mp_c_dir, mp_c_status, mp_c_bytes, mp_c_analog;
    volatile LONG mp_c_rot, mp_c_turn, mp_c_seen;
    volatile LONGLONG mp_c_act, mp_c_act2, mp_c_work_pad;
    volatile LONG mp_c_gv_flag, mp_c_wp_status, mp_c_wp_dir, mp_c_wp_bytes;
    volatile LONGLONG mp_c_workl;
    volatile LONG mp_c_padto, mp_c_wallto, mp_c_liable, mp_c_padforce, mp_c_data, mp_c_data2;

    ULONGLONG workl_ptr;                /* address of the pointer variable */
    volatile LONG c_move_padto_writes, c_move_no_workl;
    volatile LONG seam_is_copy;         /* the detour sits on the copy seam */
    /* vr_arm_freeze. See the header. */
    volatile LONG arm_freeze;
    volatile LONG arm_uproll;           /* marker vr_arm_uproll */
    volatile LONG arm_hand_basis;       /* marker vr_arm_hand_basis: 1 world */
    volatile LONG turn_gain_mils;

    /* The roll/swing envelope in milli-degrees, 0 meaning "the built-in
       default" - so no start-up ordering can leave a zero envelope that
       silently pins every hand. See the defines above for why this is a
       knob and not a constant. */
    volatile LONG hand_fore_twist_mdeg;
    volatile LONG hand_wrist_twist_mdeg;
    volatile LONG hand_wrist_swing_mdeg;
    volatile LONG turn_follow_thresh_mdeg;
    volatile LONG turn_follow_full_mdeg;
    volatile LONG c_turn_follow_writes;
    volatile LONG turn_dir_acc;
    /* Follow refusal counters, one per gate: the heartbeat must be able
       to say WHY a session ended with zero follow writes. */
    volatile LONG c_follow_no_sign;
    volatile LONG c_follow_stale;
    volatile LONG c_follow_under;
    volatile LONG c_follow_aim_hold;
    volatile LONG c_turn_write_dead;
    /* The FPS-aim actuator probe: single producer (the pad-write site on
       the game thread), single consumer (the heartbeat's drain). The
       write index publishes each filled slot; a full ring drops new
       samples rather than blocking the seam. */
    volatile LONG turn_probe;
    volatile LONG turn_probe_w;
    volatile LONG turn_probe_r;
    DG_TURN_PROBE_SAMPLE turn_probe_ring[DG_TURN_PROBE_RING];
    volatile LONG turn_last_write_dir;   /* +1 / -1 byte side, 0 = none */
    volatile LONG turn_last_write_tick;
    volatile LONG s_arm_aim_gap;         /* f2l float, degrees */
    volatile LONG arm_aim_gap_tick;
    volatile LONG s_arm_head_yaw;        /* f2l float: the gap's head term */
    volatile LONG s_arm_root_tilt;       /* f2l float: root up vs world up */
    double turn_vote_prev_drift;         /* camera-seam local, like the
                                            hand memories */
    int    turn_vote_have_prev;
    volatile LONG move_turn_x;
    volatile LONG move_turn_valid;
    volatile LONG c_move_not_subject;
    volatile LONG c_turn_writes;
    volatile LONG c_turn_yielded;
    volatile LONG c_turn_idle;
    /* The driver's own analog bytes, snapshotted each move tick BEFORE our
       write, packed [rdx rdy ldx ldy] into one word for a tear-free read. */
    volatile LONG s_pad_analog_raw;
    volatile LONG c_move_published;
    volatile LONG c_move_no_command;
    volatile LONG c_move_stale;
    volatile LONG c_move_yielded;
    volatile LONG c_move_idle;
    volatile LONG c_move_writes;
    volatile LONG c_move_blocked_gate;

    volatile LONG recoil_climb_mdeg;
    volatile LONG recoil_push_um;
    volatile LONG s_recoil_amp;     /* amplitude * 1000, for the camera seam */
    volatile LONG s_recoil_worst;
    volatile LONG c_recoil_kicks;
    volatile LONG c_recoil_climb_writes;
    volatile LONG c_recoil_no_axis;
    volatile LONG c_recoil_push_writes;
    volatile LONG c_recoil_push_refused;
    volatile LONG s_fire_state;
    volatile LONG s_fire_pressure;
    volatile LONG s_fire_wtype;
    /* The recorder's game-side half for the pass just solved. Written only
       by arm_ik_now (camera seam) and read only by the recorder tap on the
       same thread, so it needs no more synchronisation than the rest of
       the seam-local state around it. */
    DG_REC_PAIRSTATE rec_pair;
    volatile LONG c_arm_pairs_seen;
    volatile LONG c_arm_pairs_eligible;
    volatile LONG c_arm_pairs_accepted;
    volatile LONG c_arm_pairs_refused;
    volatile LONG c_arm_pairs_calibration;
    volatile LONG c_arm_pairs_map_clamped;
    volatile LONG c_arm_pairs_map_soft;
    volatile LONG c_arm_pair_replays;
    volatile LONG c_arm_pairs_frozen;
    /* REST freeze: the reference bones in the ROOT frame, captured on the
       calibration pair. Root frame rather than world so a body turn does
       not read as reference drift. Written and read on the game thread
       only (the seam), like the other arm_map state. */
    double rest_ref_upper[3];
    double rest_ref_fore[3];
    int    rest_ref_have;
    volatile LONG s_rest_drift;
    volatile LONG s_rest_drift_max;
    /* The aim direction's horizontal yaw at calibration, degrees. The
       reference the position-path aim gap is measured against - see the
       publisher in arm_ik_now. Game thread only. */
    /* The mapped head yaw at the calibration pair, degrees: the zero of
       the facing gap. The stick's software turn and the player's physical
       turn both move the live head yaw against this; the manual-rebase
       machinery that used to live here died with the position-path
       publisher - a stick turn now moves the head term itself, so there
       is no zero to chase. */
    double head_yaw0_deg;
    int    head_yaw0_have;

    double hand_yaw0_deg;
    int    hand_yaw0_have;
    double stick_yaw0_deg;
    int    stick_yaw0_have;
    volatile LONG follow_src;         /* 0 head, 1 hand, 2 stick - vr_turn_follow_src */
    volatile LONG follow_aim;         /* 0 hold, 1 on - vr_turn_follow_aim */
    volatile LONG arm_comp;           /* 0 full, 1 organic - vr_arm_comp */
    volatile LONG c_arm_comp_no_stick;
    /* The adjust frame (V5.1): the marker's choice, this pair's frame and
       the two health meters of the heartbeat's frame line. */
    volatile LONG adjust_frame;       /* 0 legacy, 1 live - vr_adjust_frame */
    DG_ADJ_FRAME  pair_frame;
    short         pair_rot_vy;        /* rot.vy read with the matrices */
    int           pair_rot_ok;
    volatile LONG c_frame_missing;    /* live pairs with no heading */
    volatile LONG s_frame_skew_worst; /* f2l float degrees */
    /* The wrist-miss meter (V5.1 par. 4.6, replacing the axis-bone meter the
       design asked for - see the note at the commit): where this pair's
       solution says the wrist will be under this pair's frame, against
       where the next pair finds it. A wrong frame moves the wrist by the
       heading error's worth of forearm; the roll about a wrong axis, the
       2026-08-30 failure, is the same thing seen at the wrist. */
    double        pred_wrist[3];
    int           have_pred_wrist;
    volatile LONG s_wrist_miss_worst; /* f2l float mm */
    volatile LONG c_wrist_miss_over;  /* pairs with a miss over 20 mm */
    volatile LONG follow_head_sign;   /* +1/-1, marker vr_follow_head_sign */
    volatile LONG c_arm_release_owner_mismatch;
    volatile LONG c_arm_hand_written;
    volatile LONG c_arm_hand_refused;
    volatile LONG c_arm_hand_measured;
    volatile LONG c_arm_hand_tick_writes;
    volatile LONG c_arm_hand_tick_no_command;
    volatile LONG c_arm_hand_tick_stale;
    volatile LONG c_arm_hand_tick_no_player;
    volatile LONG c_arm_hand_tick_owner_mismatch;
    volatile LONG c_arm_hand_tick_bad_weapon;
    volatile LONG c_arm_hand_tick_mic;
    volatile LONG c_arm_hand_no_setpos;
    volatile LONG c_arm_hand_fore_twist;
    volatile LONG c_arm_hand_limited;
    volatile LONG s_arm_hand_residual;
    volatile LONG s_arm_hand_worst;
    volatile LONG s_arm_hand_slot_echo;
    volatile LONG s_arm_hand_slot_echo_worst;
    volatile LONG c_arm_hand_slot_dirty;
    volatile LONG s_arm_hand_off_drift;
    volatile LONG s_arm_hand_off_drift_worst;
    volatile LONG c_arm_adjust_bits_lost;
    volatile LONG c_arm_hand_alt_named;
    volatile LONG c_arm_hand_scaled;
    volatile LONG s_arm_hand_fit;
    volatile LONG s_arm_hand_shortfall_worst;
    volatile LONG s_arm_body_drift;
    volatile LONG s_arm_hand_raw_twist;
    volatile LONG s_arm_hand_fore_twist;
    volatile LONG s_arm_hand_wrist_swing;
    volatile LONG s_arm_hand_wrist_twist;
    volatile LONG s_arm_map_flags;
    volatile LONG s_arm_map_scale;
    volatile LONG s_arm_map_source_span;
    volatile LONG s_arm_map_reach;
    volatile LONG s_arm_map_delta[3];
    volatile LONG s_arm_map_target[3];
    volatile LONG s_arm_shoulder_view[3];
    volatile LONG s_arm_native_wrist_view[3];
    volatile LONG arm_anchor;
    volatile LONG s_arm_map_pair;
    volatile LONG s_arm_map_stream;
    volatile LONG s_arm_ik_weight;      /* float bits */
    volatile LONG s_arm_ik_view[3];     /* float bits, camera-relative mm */
    volatile LONG s_arm_ik_root[3];     /* float bits, arm matrix space */
    volatile LONG s_arm_ik_target[3];   /* float bits, world millimetres */
    volatile LONG s_arm_ik_root_distance;
    volatile LONG s_arm_ik_root_limit;
    volatile LONG s_arm_pose_flags;
    volatile LONG s_arm_joints;     /* n_joints, 21 for HUMAN21 */
    volatile LONG s_arm_adjust_lo;
    volatile LONG s_arm_adjust_hi;
    volatile LONG s_arm_mctrl_lo;
    volatile LONG s_arm_mctrl_hi;
    volatile LONG s_mctrl_dump[20];     /* 10 qwords from m_ctrl + 0x00 */
    volatile LONG s_obj_dump[16];       /* 8 qwords from the OBJECT + 0x00 */

    /* Every bit each status word has ever had set this session, ORed.
       Twice now a run has been read as "the word is dead" when all it really
       said was "the heartbeat sampled a quiet moment" - the heartbeat shows one
       tick in eighteen hundred, and a status word that flickers for ten frames
       is invisible to it. These accumulate, so a single set bit anywhere in the
       session survives to the log. Bits only ever get set, so a torn read is a
       subset of the truth rather than a wrong answer. */
    volatile LONG seen_player_lo;
    volatile LONG seen_player_hi;
    volatile LONG seen_game;
    volatile LONG seen_menu;
    volatile LONG seen_arm_flag;
    /* The same three words, sampled at the camera seam instead, and counted.
       See the block in dg_bridge_arm_seam_now for why a second sample was
       needed and what disagreement between the two would mean. */
    volatile LONG seen_game_late;
    volatile LONG seen_menu_late;
    volatile LONG seen_player_late_lo;
    volatile LONG seen_player_late_hi;
    volatile LONG c_late_status;
    /* The same three words sampled at the Present seam - the one seam that
       survives GV_PauseLevel, and therefore the only place a menu is visible
       at all (see dg_bridge_screen_seam_now). Kept separate from the pair
       above because "the camera seam never saw it" and "nobody looked while
       it was true" are the two readings that pair could not tell apart. */
    volatile LONG seen_game_screen;
    volatile LONG seen_menu_screen;
    volatile LONG c_screen_status;
    /* Pairs whose upper-arm roll was pinned by the POSE against pairs that
       fell back to continuity. See the counting site in arm_orient_now. */
    volatile LONG c_orient_anchored;
    volatile LONG c_orient_fallback;
    volatile LONG c_orient_uprolled;    /* grip roll actually applied */
    volatile LONG c_uproll_skipped;     /* grip roll skipped fail-closed */
    /* Passive hand-rotation meter, filled by the skeleton probe so it keeps
       measuring with the arm drive switched off entirely. */
    volatile LONG s_skel_hand_turned;    /* f2l float, degrees, accumulated */
    volatile LONG c_skel_hand_samples;
    double skel_hand_prev[4];
    int    skel_hand_have_prev;
    /* The net meter's fixed reference. Captured on the first sample after a
       reset and never moved, which is the whole point: a reference that is
       refreshed from the live pose cannot show winding. */
    double skel_hand_ref[4];
    int    skel_hand_have_ref;
    volatile LONG s_skel_hand_net;
    volatile LONG s_skel_hand_net_max;
    /* The same total for the rotations WE write, so the hierarchy's turn and
       our own demand can be compared instead of guessed between. */
    /* Did the hierarchy keep what we wrote into joints 4 and 5? See the
       measuring site: the strip - and therefore the roll anchor's own
       reference - is only honest while this stays near zero. */
    volatile LONG s_arm_adj_echo;
    volatile LONG s_arm_adj_echo_worst;
    volatile LONG c_arm_adj_echo_dirty;
    volatile LONG s_adj_turned4;
    volatile LONG s_adj_turned5;
    double adj_turn_prev4[4];
    double adj_turn_prev5[4];
    int    adj_turn_have_prev;
    /* The LATEST camera-seam values, as opposed to the accumulators above.
       The accumulators answer "did this ever happen"; these three feed the
       live safety decision in bridge_tick, which needs what is true now. */
    volatile LONG s_late_game;
    volatile LONG s_late_menu;
    volatile LONG s_late_player_lo;
    volatile LONG s_late_player_hi;
    volatile LONG late_tick;        /* c_ticks at the last camera sample */
    volatile LONG c_late_stale;     /* ticks the latch was too old to use */
    /* The safety verdict computed at the CAMERA seam, from the words that seam
       samples itself. It exists because the tick seam, which is the only place
       fps.state is written, nearly stops during a codec or a cutscene while
       this seam speeds up: measured on 2026-08-18, 465 ticks against 2474
       camera samples in the window that contained both. A writer gated on
       fps.state alone is therefore gated on a value that has not been updated
       since before the cutscene began. */
    volatile LONG s_late_unsafe;
    volatile LONG c_seam_refused_unsafe;
    volatile LONG c_seam_active;    /* camera seams that reached the probes */
    volatile LONG c_seam_active_ticks; /* distinct ticks among those seams */
    volatile LONG seam_active_last_tick;

    /* F5 step 3, the adjust-space probe. pending_case is which write the NEXT
       read belongs to, and starts at -1 because the first frame has no write
       behind it yet - reading then would pair a matrix with an input that was
       never applied, which is the one mistake this measurement cannot survive.

       write_tick is the second half of that same guard, and the run of
       2026-08-18 is why it exists. The camera seam runs 1.00 times per tick in
       third person but 5.32 times per tick in first person - measured, not
       assumed - while the hierarchy pass runs once. Alternating per seam call
       therefore wrote five times per pass and read the same pose five times,
       and both cases recorded the identical 48 floats. The case may only turn
       over once the tick counter has moved. */
    volatile LONG adj_probe;
    volatile LONG adj_joint;
    volatile LONG adj_deg;
    volatile LONG adj_axis;
    volatile LONG adj_pending_case;
    volatile LONG adj_write_tick;
    volatile LONG c_adj_held;       /* seams skipped waiting for a new tick */
    volatile LONG adj_quat[DG_ADJ_CASES * 4];           /* float bits */
    volatile LONG adj_world[DG_ADJ_CASES * DG_ADJ_JOINTS * 16];
    volatile LONG adj_indices[DG_ADJ_JOINTS];
    volatile LONG adj_samples[DG_ADJ_CASES];
    /* ROLL V5.1 Meetplan A': the actor's heading words beside each case,
       read on the WRITE pass (phase 0) and again on the READ pass (phase 1,
       DG_ADJ_SETTLE_TICKS later) so the skew between "matrices from the
       previous hierarchy pass" and "rot.vy now" is bracketed, not argued.
       Per case and phase: tick, ok, rot.vy (+0x82), turn.vy (+0x8A),
       camdir.vy (+0xD22) as raw int16. */
    volatile LONG adj_words[DG_ADJ_CASES * 2 * 5];
    /* The per-cycle reduction, single producer (the seam) single consumer
       (the 1 Hz drain), same ring discipline as the turn probe. */
    volatile LONG adj_cycle_w;
    volatile LONG adj_cycle_r;
    DG_ADJ_CYCLE adj_cycle_ring[DG_ADJ_CYCLE_RING];
    /* ...and the same word ANDed, so a bit that was ever CLEAR is provable too.
       Seeded to all-ones by the arm block's first sample. */
    volatile LONG held_arm_flag;
    volatile LONG c_arm_visible;    /* ticks channel 0 was actually shown */
    /* Which pad bits this session has ever seen down, and how often the weapon
       button in particular went down. The game is on keyboard and mouse and
       nobody here knows the binding; rather than guess it, let the pad say so.
       F7 needs this instrument regardless - it has to merge a controller
       trigger into exactly this field. */
    volatile LONG seen_pad_status;
    volatile LONG c_weapon_press;
    /* And the same edge counted only while we hold first person. Six presses
       and a hidden arm is two different findings depending on whether any of
       the six landed inside a first-person session, and the plain counter
       cannot tell them apart - the weapon camera needs PLAYER_WATCH, which
       only holds while we are in. */
    volatile LONG c_weapon_press_fps;
    /* Which physical keys were down at the instant the weapon bit went down.
       Knowing the button is reachable is not the same as knowing how to press
       it, and telling the player "now shoot" without being able to say with
       what is how the last run came back empty. The game is inside this
       process and the keyboard is the user's own, so ask the OS.
       Captured only on the edge - never polled per tick. */
    volatile LONG s_weapon_vk[4];
    LONG last_pad_status;

    /* F5 step 2, the skeleton probe. Measured once per object and then left
       alone: the stride and the parent table do not change while the model is
       loaded, and re-scanning every frame would be pointless work in the middle
       of the render seam. skel_for is the DG_OBJS the measurement belongs to,
       so a model swap re-measures instead of reporting a stale rig. */
    volatile LONG skel_probe;
    volatile LONG skel_base;
    volatile LONG skel_for_lo;
    volatile LONG skel_for_hi;
    volatile LONG skel_stride;
    volatile LONG skel_stride_score;
    volatile LONG skel_stride_tried;
    volatile LONG skel_n_models;
    volatile LONG skel_parents_read;
    volatile LONG skel_parents[DG_SKEL_MAX];
    volatile LONG skel_chain[DG_SKEL_CHAIN];
    volatile LONG skel_chain_len;
    volatile LONG skel_chain_pos[DG_SKEL_CHAIN * 3];   /* float bits */
    volatile LONG skel_window_pos[DG_SKEL_WINDOW * 3]; /* float bits */
    volatile LONG skel_mctrl_trans_lo;
    volatile LONG skel_mctrl_trans_hi;
    volatile LONG skel_objs_lo;
    volatile LONG skel_objs_hi;
    volatile LONG skel_region_end;

    /* ---- theater judgment (camera-seam owned) + UI-U1 HUD hide ---- */
    volatile LONG theater_mode;     /* DG_THEATER_* from the marker */
    volatile LONG theater_ui_mode;  /* vr_ui on/off */
    volatile LONG hud_mode;         /* vr_hud: 1 = hide */
    /* Only the camera seam steps these, which is what keeps the judgment
       per-frame by construction: one seam status sample is one frame's
       camera build, and both eyes of a stereo pair are consecutive frames
       reading the SAME published verdict rather than each judging. */
    DG_THEATER_HYST thea_demo;
    DG_THEATER_HYST thea_ui;
    volatile LONG s_theater;        /* published raw verdict bits */
    volatile LONG theater_ms;       /* GetTickCount() at publication */
    volatile LONG c_thea_samples;
    volatile LONG c_thea_masked;
    volatile LONG c_thea_ui_masked;
    volatile LONG c_thea_enter;
    volatile LONG c_thea_exit;
    volatile LONG c_thea_ui_enter;
    volatile LONG c_thea_ui_exit;
    volatile LONG c_thea_enter_held;
    volatile LONG c_thea_exit_held;
    volatile LONG c_thea_bit_demo;
    volatile LONG c_thea_bit_scn;
    volatile LONG c_thea_bit_pad;
    volatile LONG c_thea_bit_radio;
    volatile LONG c_thea_bit_weapon;
    volatile LONG c_thea_bit_item;
    /* HUD hide: held is the release debt - set the first time the hide bits
       are asserted, cleared by the one release write, exactly the shape of
       hand_probe_owned above. */
    volatile LONG hud_held;
    volatile LONG c_hud_writes;
    volatile LONG c_hud_cleared;
} g_b;

#include "dg_hand_profile_store.inl"

static DG_FPS_EVENT g_ring[DG_RING];
static volatile LONG g_ring_head;
static volatile LONG g_ring_tail;

/* Theater flank events: one per enter/exit of either judgment, carrying the
   three status words AT the flank. This is the phase-0 evidence - every
   switch must be explainable from the words that caused it, in the log, so a
   wrong flank is a replayable finding instead of a discussion. Pushed at the
   camera seam (no formatting there, same as g_ring), drained by the logger.
   Only pushed when vr_theater is measure or on: OFF means today's log. */
typedef struct {
    unsigned kind;              /* DG_THEATER_V_* bit; 0x100 added on exit */
    unsigned game_status;
    unsigned menu_status;
    unsigned long long player_status;
    unsigned long sample;       /* c_thea_samples at the flank */
    unsigned long ms;           /* GetTickCount() at the flank */
} DG_THEA_EVENT;

#define DG_THEA_RING 16
static DG_THEA_EVENT g_thea_ring[DG_THEA_RING];
static volatile LONG g_thea_ring_head;
static volatile LONG g_thea_ring_tail;

static void thea_ring_push(unsigned kind, unsigned game, unsigned menu,
                           unsigned long long player)
{
    LONG slot = InterlockedIncrement(&g_thea_ring_head) - 1;
    DG_THEA_EVENT *e = &g_thea_ring[slot & (DG_THEA_RING - 1)];
    e->kind = kind;
    e->game_status = game;
    e->menu_status = menu;
    e->player_status = player;
    e->sample = (unsigned long)g_b.c_thea_samples;
    e->ms = (unsigned long)GetTickCount();
}

#define RD32(addr) (*(volatile LONG *)(ULONG_PTR)(addr))
#define WR32(addr, v) (*(volatile LONG *)(ULONG_PTR)(addr) = (LONG)(v))

/* Cheap sanity for a pointer we are about to dereference on the game thread.
   Every other address the bridge touches came out of the anchor table and was
   cross-checked before it was believed; the GM_PlayerBody candidate did not, so
   it gets this instead. Canonical user-space, above the null page, and aligned,
   which is all an OBJECT* from this allocator can be. It cannot make a bad
   pointer safe - only a bad pointer obvious. */
static int plausible_ptr(ULONGLONG p)
{
    return p >= 0x10000ULL && p < 0x00007FFFFFFFFFFFULL && (p & 7) == 0;
}

static void ring_push(const DG_FPS_STATE *s, const DG_FPS_STEP *o,
                      const DG_FPS_INPUT *in, unsigned long long player)
{
    LONG slot = InterlockedIncrement(&g_ring_head) - 1;
    DG_FPS_EVENT *e = &g_ring[slot & (DG_RING - 1)];
    e->from_state = (unsigned)o->from_state;
    e->to_state = (unsigned)s->state;
    e->reason = (unsigned)s->reason;
    e->write = (unsigned)o->write;
    e->tick = (unsigned long)g_b.c_ticks;
    e->game_status = in->game_status;
    e->menu_status = in->menu_status;
    e->player_status = player;
}

/* One write per tick, at most. Never gBP_1stPersonCamera_Active. */
static void apply_write(int kind, int value)
{
    switch (kind) {
    case DG_FPS_WRITE_OVERRIDE:
        WR32(g_b.a.gbp_override, value);
        InterlockedExchange(&g_b.wrote_any, 1);
        break;
    case DG_FPS_WRITE_TOGGLE:
        WR32(g_b.a.gbp_toggle, value);
        InterlockedExchange(&g_b.wrote_any, 1);
        break;
    case DG_FPS_WRITE_MOVE:
        WR32(g_b.a.gbp_move, value);
        InterlockedExchange(&g_b.wrote_any, 1);
        InterlockedExchange(&g_b.wrote_move, 1);
        break;
    case DG_FPS_WRITE_SUBJECT_EDGE:
        /* One frame of PL_PAD_SUBJECT in press, offered after Action() has
           copied GV_PadData into PlayerPad.pad and before the Bluepoint weapon
           and button state rewrite it. The next tick's copy clears it, so the
           edge cannot be consumed twice. status is left alone: CheckWatch reads
           press in toggle mode, and setting status would also trip the raw
           GV_PadData consumers documented in the review. */
        *(volatile LONG *)(ULONG_PTR)(g_b.a.player_pad + DG_PAD_PRESS_OFFSET) |=
            (LONG)value;
        break;
    default:
        return;
    }
    InterlockedIncrement(&g_b.c_writes);
}

static void warn_always_is_incomplete(void);

static void bridge_restore_once(void)
{
    if (!InterlockedCompareExchange(&g_b.wrote_any, 0, 0)) return;
    if (InterlockedCompareExchange(&g_b.restored, 1, 0) != 0) return;
    /* Move is conditional here where Override and Toggle are not, and the
       asymmetry is real rather than an oversight: those two are written on
       every claim, so reaching this line means they were ours. Move is opt-in,
       and in the default configuration the bridge never touches it - writing
       our saved copy back would then be a write to a field we never borrowed.
       The desk test caught it as a null-pointer store, which is the polite
       version of the same mistake. */
    if (InterlockedCompareExchange(&g_b.wrote_move, 0, 0))
        WR32(g_b.a.gbp_move, g_b.saved_move);
    WR32(g_b.a.gbp_override, g_b.saved_override);
    WR32(g_b.a.gbp_toggle, g_b.saved_toggle);
}

/* Capture the values this generation may borrow.  A stopped bridge can be
   started again in the same process, so the once-only restore latch and its
   write debt belong to the generation, not to the DLL lifetime. */
static void bridge_capture_ownership_snapshot(void)
{
    g_b.saved_override = (int)RD32(g_b.a.gbp_override);
    g_b.saved_toggle = (int)RD32(g_b.a.gbp_toggle);
    g_b.saved_move = (int)RD32(g_b.a.gbp_move);
    g_b.saved_active = (int)RD32(g_b.a.gbp_active);
    /* Override non-zero before we ever wrote it means MGSHDFix's FPS feature
       already owns these values. Observe only, for the whole session. */
    g_b.owner = g_b.saved_override != 0;
    InterlockedExchange(&g_b.restored, 0);
    InterlockedExchange(&g_b.wrote_any, 0);
    InterlockedExchange(&g_b.wrote_move, 0);
}

/* Pure, and separated out for exactly that reason: this is the one decision in
   bridge_tick that can disable the bridge for a whole session, and it did.

   The premise is "we are holding Override up right now", not "we have ever
   written it". Those were the same claim while the bridge held Override for the
   entire session. Since the claim is handed back when idle, `wrote_any &&
   override != 1` stopped describing a second owner and started describing our
   own released claim - on every single idle tick. Eight ticks after the first
   toggle finished, ownership latched and the bridge went inert for the rest of
   the run.

   `held` is read before dg_fps_step runs, so it is last tick's answer: if we
   were holding Override at the end of that tick, this tick must read it as 1.
   The release costs at most two counts, because held only clears once both
   values are back, and the reset below wipes those before they can accumulate. */
/* Fold the camera-seam status latch into the tick-seam reading. Kept pure and
   separate from bridge_tick so the rule can be exercised at a desk: bridge_tick
   reads globals, this decides what to do with what they said.
   Returns 1 if the latch was used, 0 if it was refused as absent or stale. */
static int fold_late_status(unsigned int *game, unsigned int *menu,
                            ULONGLONG *player,
                            unsigned int late_game, unsigned int late_menu,
                            ULONGLONG late_player, LONG samples, LONG age)
{
    if (samples <= 0) return 0;
    /* A negative age means the two counters disagree about time, which should
       be impossible and is therefore exactly the case not to trust. */
    if (age < 0 || age > DG_LATE_STATUS_TICKS) return 0;
    *game |= late_game;
    *menu |= late_menu;
    *player |= late_player;
    return 1;
}

static int fight_step(int held, int native_override, int *fights)
{
    if (held && native_override != 1)
        return ++(*fights) >= 8;
    /* Consecutive, not cumulative. A second owner clears the value every
       frame; a level load or a script clearing it once is not the same thing,
       and a counter that never decays would eventually latch on eight
       unrelated transients and disable the bridge for the session. */
    *fights = 0;
    return 0;
}

/* The per-tick drive point. Runs on the game thread inside Action(). No
   OpenXR, no allocation, no formatting, no logging - counters and a bounded
   ring only. */
/* Defined with the rest of the ArmCamRotateShift probe further down. They are
   called from here because the tick seam is the only one that runs ahead of
   the arm actor, which is the whole point of that probe. */
static void hand_probe_write(void);
static void hand_probe_release(void);
static void hand_drive_write(void);
static void fire_tick(int safe_gameplay);
static void move_tick(int safe_gameplay);
static void move_resolve_workl(const LiveImage *image);
static void interact_tick(int safe_gameplay);
static DG_INTERACT_ADAPTER g_interact_adapter;
static volatile LONG64 g_interact_codec_pending;
static struct {
    volatile LONG samples,blocked,actor_missing,pad_writes,capture_writes,ladder_writes;
    volatile LONG codec_queued,codec_written,codec_dropped,weapon,context;
    volatile LONG64 actor;
    DWORD last_log;
} g_interact_stats;
#include "dg_interact_player.inl"

static SRWLOCK g_controls_lock = SRWLOCK_INIT;
static DG_BRIDGE_CONTROLS_PROVIDER g_controls_provider;
static DG_BRIDGE_CONTROLS_STOP g_controls_stop;
static void *g_controls_user;
static DG_BRIDGE_CONTROLS_FRAME g_controls_frame;
static uint64_t g_controls_lease;
static int g_controls_allowed, g_controls_radial_allowed, g_controls_active, g_controls_ladder, g_controls_special;
static volatile LONG g_controls_fire_retired;
#include "dg_radial_game.inl"
#include "dg_health_bridge.inl"
#include "dg_action_bridge.inl"
#include "dg_m9_bridge.inl"
static void stinger_resolve(const LiveImage *im);
static void stinger_install(void);
static void stinger_stop(void);
static void unarmed_prone_resolve(const LiveImage *im);
static void unarmed_prone_install(int enabled);
static void unarmed_prone_stop(void);
static void blade_resolve(const LiveImage *im);
static void blade_install(int requested);
static void blade_stop(void);
static void blade_tick(int safe);
static int blade_claim(void);
#include "dg_native_hud.inl"
static void coolant_resolve(const LiveImage *im);
static void coolant_install(void);
static void coolant_stop(void);
#include "dg_pistol_reload.inl"
#include "dg_mod_menu_bridge.inl"
static void controls_clear_legacy(void) {
    InterlockedExchange(&g_b.fire_valid,0);
    InterlockedExchange(&g_b.move_valid,0);
    InterlockedExchange(&g_b.move_turn_valid,0);
}
void dg_bridge_controls_register(DG_BRIDGE_CONTROLS_PROVIDER provider,
    DG_BRIDGE_CONTROLS_STOP stop, void *user) {
    if(!provider)dg_bridge_action_register(NULL);
    AcquireSRWLockExclusive(&g_controls_lock);
    if (g_controls_stop) g_controls_stop(g_controls_user);
    dg_bridge_radial_cancel();
    if (provider) InterlockedExchange(&g_controls_fire_retired,0);
    else if (g_controls_provider) InterlockedExchange(&g_controls_fire_retired,1);
    g_controls_provider=provider; g_controls_stop=stop; g_controls_user=user;
    g_controls_allowed=g_controls_radial_allowed=0; g_controls_lease=0; g_controls_active=0;
    g_controls_ladder=0;
    g_controls_special=0;
    memset(&g_interact_adapter,0,sizeof g_interact_adapter);
    InterlockedExchange64(&g_interact_codec_pending,0);
    memset(&g_controls_frame,0,sizeof(g_controls_frame));
    controls_clear_legacy();
    ReleaseSRWLockExclusive(&g_controls_lock);
}
void dg_bridge_controls_context(int allowed) {
    dg_bridge_controls_context_all(allowed,0,allowed);
}
void dg_bridge_controls_context_modes(int allowed,int ladder) {
    dg_bridge_controls_context_all(allowed,ladder?DG_CONTROLS_LADDER:0,0);
}
void dg_bridge_controls_context_special(int allowed,int special) {
    dg_bridge_controls_context_all(allowed,special,0);
}
void dg_bridge_controls_context_ex(int allowed,int radial) {
    dg_bridge_controls_context_all(allowed,0,radial);
}
void dg_bridge_controls_context_all(int allowed,int special,int radial) {
    AcquireSRWLockExclusive(&g_controls_lock);
    g_controls_allowed=allowed ? 1 : 0;
    g_controls_special=(special==DG_CONTROLS_LADDER || special==DG_CONTROLS_BEYOND ||
        special==DG_CONTROLS_LOCKER || special==DG_CONTROLS_DOWNED)?special:0;
    g_controls_ladder=g_controls_special==DG_CONTROLS_LADDER;
    g_controls_radial_allowed=radial ? 1 : 0;
    if (!allowed) InterlockedExchange64(&g_interact_codec_pending,0);
    g_controls_lease=(allowed || g_controls_special || radial) ? GetTickCount64() : 0;
    if (!allowed && !g_controls_special && !radial) {
        dg_bridge_radial_cancel();
        memset(&g_controls_frame,0,sizeof(g_controls_frame));
        g_interact_adapter.held=0;
        g_interact_adapter.blocked=DG_IA_ALL;
        InterlockedExchange64(&g_interact_codec_pending,0);
        controls_clear_legacy();
        if (g_controls_stop) g_controls_stop(g_controls_user);
    }
    ReleaseSRWLockExclusive(&g_controls_lock);
}
/* Called only by the single game-tick thread. Hold through BOTH consumers;
   context(false) is a completed-write barrier, not a sampled flag. */
static void controls_begin(int safe, uint64_t now) {
    int allowed;
    AcquireSRWLockShared(&g_controls_lock);
    memset(&g_controls_frame,0,sizeof(g_controls_frame));
    g_controls_active=1;
    if (!g_controls_provider) return;
    allowed=g_controls_allowed && now>=g_controls_lease &&
            now-g_controls_lease<=100u && safe &&
            InterlockedCompareExchange(&g_b.armed,0,0) &&
            !InterlockedCompareExchange(&g_b.script_menu_only,0,0) &&
            !InterlockedCompareExchange(&g_b.s_late_unsafe,0,0);
    if (allowed && (g_b.fps.state!=DG_FPS_ACTIVE ||
        !g_b.a.player_pad || RD32(g_b.a.player_pad-4)==0))
        allowed=DG_CONTROLS_TURN_ONLY;
    if (!allowed && g_controls_special && now>=g_controls_lease &&
        now-g_controls_lease<=100u && !g_b.script_menu_only &&
        dg_bridge_controller_special_now()==g_controls_special) allowed=g_controls_special;
    if (!allowed && g_controls_radial_allowed && now>=g_controls_lease &&
        now-g_controls_lease<=100u && dg_bridge_controller_radial_now())
        allowed=DG_CONTROLS_RADIAL_ONLY;
    if (!allowed && g_controls_stop) g_controls_stop(g_controls_user);
    g_controls_provider(g_controls_user,allowed,g_b.fire.state==DG_FIRE_IDLE,
        now,(uint64_t)(DWORD)g_b.c_ticks,&g_controls_frame);
    if (g_controls_frame.fire_available) InterlockedIncrement(&g_b.c_fire_published);
    if (g_controls_frame.move_available) InterlockedIncrement(&g_b.c_move_published);
}
static void controls_end(void) {
    g_controls_active=0;
    memset(&g_controls_frame,0,sizeof(g_controls_frame));
    ReleaseSRWLockShared(&g_controls_lock);
}

static void bridge_tick_body(void);
/* Declared here because the install site sits thousands of lines above
   the body; see pad_seam_tick for why this seam exists at all. */
static void pad_seam_tick(void);
static SRWLOCK g_script_menu_lock = SRWLOCK_INIT;
static DG_BRIDGE_MENU g_script_menu_pending;
static ULONGLONG g_script_menu_deadline;
static volatile LONG g_script_menu_cancel;
static LONG g_script_menu_pending_epoch;
static int g_xr_menu_context;
static void script_menu_clear(int nonblocking)
{
    InterlockedIncrement(&g_script_menu_cancel);
    if (nonblocking) {
        if (!TryAcquireSRWLockExclusive(&g_script_menu_lock)) return;
    } else AcquireSRWLockExclusive(&g_script_menu_lock);
    memset(&g_script_menu_pending, 0, sizeof g_script_menu_pending);
    g_script_menu_deadline = 0;
    ReleaseSRWLockExclusive(&g_script_menu_lock);
}

/* The detour target. The tick seam is the ONE place a staged policy table
   becomes active - between ticks, never inside one - and the whole tick is
   one policy pass: every dispatched call below sees the same table, even if
   the worker stages a new one mid-tick. The VEH camera seam brackets its
   own pass the same way in dg_hook.c. */
static void bridge_tick(void)
{
    void *pol_pass;
    dg_policy_tick_adopt();
    pol_pass = dg_policy_pass_begin();
    bridge_tick_body();
    dg_policy_pass_end(pol_pass);
}

#ifdef DG_HOOK_TEST
void dg_bridge_test_tick(void) { bridge_tick(); }
#endif

/* UI-U1's HUD hide, on the tick seam because that is the game thread and the
   one seam allowed to write game state. The write is the game's own command
   vocabulary - the MENU_*_OFF bits scenes already use to blank the HUD - OR'd
   in and re-asserted every tick (the game may clear them), and the release
   clears exactly those four bits once. Bounded: four bits, one word, only
   with the anchors resolved and the bridge armed, and MENU_CAPTION_OFF is
   never touched. On release the game's own scene logic re-imposes whatever
   hide state it wants; these are standing commands, not latched history. */
static void hud_tick(void)
{
    LONG want = InterlockedCompareExchange(&g_b.hud_mode, 0, 0);
    LONG cur;
    if (!g_b.a.gm_menu_status) return;
    cur = RD32(g_b.a.gm_menu_status);
    if (want) {
        if ((cur & DG_HUD_HIDE_BITS) != DG_HUD_HIDE_BITS) {
            WR32(g_b.a.gm_menu_status, cur | DG_HUD_HIDE_BITS);
            InterlockedIncrement(&g_b.c_hud_writes);
        }
        InterlockedExchange(&g_b.hud_held, 1);
    } else if (InterlockedCompareExchange(&g_b.hud_held, 0, 0)) {
        WR32(g_b.a.gm_menu_status, cur & ~(LONG)DG_HUD_HIDE_BITS);
        InterlockedExchange(&g_b.hud_held, 0);
        InterlockedIncrement(&g_b.c_hud_cleared);
    }
}

static int controller_toggle_pending(int safe_gameplay, DWORD now_ms)
{
    if (!safe_gameplay || (DWORD)(now_ms-(DWORD)InterlockedCompareExchange(
            &g_b.toggle_request_ms,0,0))>100u) {
        InterlockedExchange(&g_b.toggle_request,0);
        return 0;
    }
    return InterlockedCompareExchange(&g_b.toggle_request,0,0)!=0;
}

static void bridge_tick_body(void)
{
    DG_FPS_INPUT in;
    DG_FPS_STEP out;
    ULONGLONG status;
    unsigned int game_status;
    unsigned int menu_status;

    if (!InterlockedCompareExchange(&g_b.armed, 0, 0)) return;
    InterlockedIncrement(&g_b.c_ticks);

    memset(&in, 0, sizeof(in));
    in.mode = (int)InterlockedCompareExchange(&g_b.mode, 0, 0);
    in.native_override = (int)RD32(g_b.a.gbp_override);
    in.native_toggle = (int)RD32(g_b.a.gbp_toggle);
    in.native_active = (int)RD32(g_b.a.gbp_active);
    in.native_move = (int)RD32(g_b.a.gbp_move);
    in.subject_move = (int)RD32(g_b.a.pl_subject_move);
    in.pad_subject_mask = (unsigned int)RD32(g_b.a.pad_subject);
    in.pad_stop_aim_mask = (unsigned int)RD32(g_b.a.pad_stop_aim);
    in.move_mode = (int)InterlockedCompareExchange(&g_b.move_mode, 0, 0);
    in.saved_override = g_b.saved_override;
    in.saved_toggle = g_b.saved_toggle;
    in.saved_move = g_b.saved_move;
    status = *(volatile ULONGLONG *)(ULONG_PTR)g_b.a.gm_player_status;

    game_status = (unsigned int)RD32(g_b.a.gm_game_status) |
                  (unsigned int)RD32(g_b.a.gm_game_status_scn);
    menu_status = (unsigned int)RD32(g_b.a.gm_menu_status) |
                  (unsigned int)RD32(g_b.a.gm_menu_status_scn);
    /* Fold in what the camera seam last saw, because this seam cannot see a
       cutscene. The run of 2026-08-17 measured both seams side by side in one
       session, against the same anchors:

           tick seam    game 0x00000000  menu 0x00005800
           camera seam  game 0x18000240  menu 0x00345C0F   (3960 samples)

       0x18000240 has bits inside DG_GAME_UNSAFE_MASK and 0x00345C0F has one
       inside DG_MENU_UNSAFE_MASK, so the states the gate exists for did occur -
       and the gate did not fire, reporting `unsafe 0` for the whole session.
       That makes this not a measurement problem but a live safety hole: the
       one decision that stands the bridge down during a codec or a cutscene
       was reading from a seam where neither is visible.

       ORing is deliberate and is the only safe direction: it can make the
       bridge stand down when it otherwise would not have, never the reverse.
       The camera value is at most one frame old, so entering a cutscene costs
       at most one frame of exposure and leaving one costs one frame of extra
       caution.

       Guarded on freshness, because a latched value that stops being updated -
       if the camera hook ever stops firing - would pin the bridge into suspend
       forever, and a mod that quietly stops working is worse than one that
       says so. Beyond the window we fall back to this seam alone. */
    {
        LONG samples = InterlockedCompareExchange(&g_b.c_late_status, 0, 0);
        LONG age = (LONG)((DWORD)g_b.c_ticks -
                          (DWORD)InterlockedCompareExchange(&g_b.late_tick,
                                                            0, 0));
        ULONGLONG late_player =
            ((ULONGLONG)(DWORD)InterlockedCompareExchange(
                 &g_b.s_late_player_hi, 0, 0) << 32)
          | (ULONGLONG)(DWORD)InterlockedCompareExchange(
                 &g_b.s_late_player_lo, 0, 0);
        if (!fold_late_status(&game_status, &menu_status, &status,
                (unsigned int)InterlockedCompareExchange(&g_b.s_late_game, 0, 0),
                (unsigned int)InterlockedCompareExchange(&g_b.s_late_menu, 0, 0),
                late_player, samples, age) && samples)
            InterlockedIncrement(&g_b.c_late_stale);
    }

    health_tick(game_status,menu_status,status);
    in.game_status = game_status;
    in.menu_status = menu_status;
    in.safe_gameplay = (status & DG_PLAYER_UNSAFE_MASK) == 0 &&
                       (game_status & DG_GAME_UNSAFE_MASK) == 0 &&
                       (menu_status & DG_MENU_UNSAFE_MASK) == 0;

    /* Ownership is latched, never negotiated. Either MGSHDFix held Override
       before we ever touched it, or it starts clearing the value we hold - both
       mean the same thing and both are terminal for the session. */
    if (fight_step(g_b.fps.held, in.native_override, &g_b.override_fights))
        g_b.owner = 1;
    in.mgshdfix_owner = g_b.owner;

    /* PL_SetPadTypeSubjectMove() runs on level load whenever PL_SubjectMove is
       set, and the pattern it installs zeroes PL_PAD_SUBJECT. That pair is the
       only level-load evidence the resolved anchors can carry. */
    in.level_load = in.subject_move != 0 && in.pad_subject_mask == 0;
    {
        DG_CAMERA_GATE camera;
        dg_bridge_camera_gate_now(&camera);
        /* Unknown pointers are not evidence of camera loss. Interactions
           own temporary camera changes and cannot start this recovery. */
        int replaced=fps_rig_changed(&g_b.fps_rig,camera.arm_body,
                                    g_b.fps.desired,in.native_active);
        in.native_camera_missing = camera.camera &&
            (!camera.arm_camera_on || replaced) &&
            !(status & 0x30901ULL); /* WATCH, attack, HOLD, ladder, enemy pull */
    }
    m9_context_boundary(in.level_load || (game_status & 0x80004000u));

    /* A controller request belongs to its gameplay context. Never retain it
       through an unsafe/menu interval for delayed execution on return. */
    in.toggle_request = controller_toggle_pending(in.safe_gameplay,GetTickCount()) &&
                        in.mode == DG_FPS_MODE_TOGGLE;

    InterlockedExchange64(&g_b.calibration_ready_arm,0);
    dg_fps_step(&g_b.fps, &in, &out);

    if (out.request_consumed) {
        InterlockedExchange(&g_b.toggle_request, 0);
        /* What the player asked for, which is not what we injected. This used
           to be incremented alongside c_edges, so the two could never differ
           and one of them said nothing. They part company exactly where it
           matters: a press held pending through a menu, or a request the mask
           gate refuses, is a request with no edge behind it. */
        InterlockedIncrement(&g_b.c_requested);
    }

    if (out.write != DG_FPS_WRITE_NONE) {
        apply_write(out.write, out.write_value);
        if (out.write == DG_FPS_WRITE_SUBJECT_EDGE)
            InterlockedIncrement(&g_b.c_edges);
    }

    if (out.transitioned) {
        InterlockedIncrement(&g_b.c_transitions);
        if (g_b.fps.state == DG_FPS_ACTIVE) {
            InterlockedIncrement(&g_b.c_entered);
            if(out.from_state==DG_FPS_REQUEST_ENTER ||
               (out.from_state==DG_FPS_SUSPENDED && out.from_reason==DG_FPS_REASON_LEVEL_LOAD))
                InterlockedIncrement(&g_b.c_explicit_entries);
        }
        if (out.from_state == DG_FPS_REQUEST_LEAVE &&
            g_b.fps.state == DG_FPS_OFF)
            InterlockedIncrement(&g_b.c_left);
        if (g_b.fps.state == DG_FPS_SUSPENDED) {
            switch (g_b.fps.reason) {
            case DG_FPS_REASON_MASK_ZERO:
                InterlockedIncrement(&g_b.c_mask_zero); break;
            case DG_FPS_REASON_LEVEL_LOAD:
                InterlockedIncrement(&g_b.c_level_load); break;
            case DG_FPS_REASON_UNSAFE:
                InterlockedIncrement(&g_b.c_unsafe); break;
            case DG_FPS_REASON_TIMEOUT:
                InterlockedIncrement(&g_b.c_timeouts); break;
            default: break;
            }
        }
        ring_push(&g_b.fps, &out, &in, (unsigned long long)status);
    }

    if(fps_calibration_ready(&g_b.fps,&in,&g_b.fps_rig))
        InterlockedExchange64(&g_b.calibration_ready_arm,(LONG64)g_b.fps_rig.arm);
    InterlockedExchange(&g_b.s_native_active, in.native_active);
    InterlockedExchange(&g_b.s_override, in.native_override);
    InterlockedExchange(&g_b.s_toggle, in.native_toggle);
    InterlockedExchange(&g_b.s_move, in.native_move);
    InterlockedExchange(&g_b.s_subject_move, in.subject_move);
    /* PL_SubjectMove is the whole point of vr_fps_move=on and the snapshot
       above is one tick in thousands, so count the ticks it was actually set.
       Zero here with Move held at 1 means our write did not reach the entry
       edge, which is a completely different failure from "the arm did not
       appear" and must not be mistaken for it. */
    if (in.subject_move) InterlockedIncrement(&g_b.c_subject_move_ticks);
    InterlockedExchange(&g_b.s_subject_toggle,
                        (LONG)RD32(g_b.a.pl_subject_toggle));
    InterlockedExchange(&g_b.s_pad_subject, (LONG)in.pad_subject_mask);
    InterlockedExchange(&g_b.s_pad_stop_aim,
                        (LONG)in.pad_stop_aim_mask);
    /* Ahead of the arm actor, and only while the subjective view is ours -
       the value is meaningless to SetPos otherwise, and writing into a global
       we are not currently entitled to is not something to do for a probe. */
    if (g_b.fps.state == DG_FPS_ACTIVE && in.safe_gameplay) {
        if (InterlockedCompareExchange(&g_b.hand_probe, 0, 0)) {
            hand_probe_write();
        } else {
            hand_probe_release();
            hand_drive_write();
        }
    } else {
        hand_probe_release();
    }
    /* Unconditional, unlike the hand: an aim that has to be stood down needs
       ticks in which to do it, and those are exactly the ticks where the gate
       has just closed. */
    action_log_drain();
    controls_begin(in.safe_gameplay,GetTickCount64());
    blade_tick(in.safe_gameplay);
    fire_tick(in.safe_gameplay);
    g_b.tick_status = status;
    move_tick(in.safe_gameplay);
    interact_tick(in.safe_gameplay);
    mod_menu_pad(in.safe_gameplay);
    controls_end();
    InterlockedExchange(&g_b.s_status_lo, (LONG)(DWORD)status);
    InterlockedExchange(&g_b.s_status_hi, (LONG)(DWORD)(status >> 32));
    InterlockedExchange(&g_b.s_game_status, (LONG)in.game_status);
    InterlockedExchange(&g_b.s_menu_status, (LONG)in.menu_status);
    InterlockedOr(&g_b.seen_player_lo, (LONG)(DWORD)status);
    InterlockedOr(&g_b.seen_player_hi, (LONG)(DWORD)(status >> 32));
    InterlockedOr(&g_b.seen_game, (LONG)in.game_status);
    InterlockedOr(&g_b.seen_menu, (LONG)in.menu_status);
    {
        LONG pad = *(volatile LONG *)(ULONG_PTR)
                       (g_b.a.player_pad + DG_PAD_STATUS_OFFSET);
        LONG wmask = (LONG)RD32(g_b.a.pad_weapon);
        InterlockedOr(&g_b.seen_pad_status, pad);
        /* Rising edge against the mask the game resolved at runtime, not a
           constant: PL_PAD_WEAPON is reassigned per pad pattern. */
        if (wmask && (pad & wmask) && !(g_b.last_pad_status & wmask)) {
            int vk, n = 0;
            InterlockedIncrement(&g_b.c_weapon_press);
            if (g_b.fps.state == DG_FPS_ACTIVE)
                InterlockedIncrement(&g_b.c_weapon_press_fps);
            for (vk = 1; vk <= 0xFE && n < 4; vk++) {
                /* Skip the modifier aliases: VK_SHIFT and friends duplicate
                   their left/right forms and would fill the four slots with
                   the same key twice. */
                if (vk == VK_SHIFT || vk == VK_CONTROL || vk == VK_MENU)
                    continue;
                if (GetAsyncKeyState(vk) & 0x8000)
                    InterlockedExchange(&g_b.s_weapon_vk[n++], vk);
            }
            while (n < 4) InterlockedExchange(&g_b.s_weapon_vk[n++], 0);
        }
        g_b.last_pad_status = pad;
    }

    {
        ULONGLONG arm = 0, objs = 0;
        LONG flag = 0;
        if (g_b.a.gm_player_arm_body)
            arm = *(volatile ULONGLONG *)(ULONG_PTR)g_b.a.gm_player_arm_body;
        if (arm) {
            objs = *(volatile ULONGLONG *)(ULONG_PTR)arm;
            if (objs) flag = RD32(objs + 0x58);
        }
        /* Counted on any change of identity, not only on the passes through
           null. The 2026-08-13 run swapped GM_PlayerArmBody from ...842990 to
           ...842B50 between two heartbeats and both counters stayed at their
           starting values, because a destroy and a create inside one tick
           never shows a null to anybody sampling per tick. A counter that
           misses the event it is named after is worse than no counter. */
        if (arm != g_b.last_arm_body) {
            if (arm) InterlockedIncrement(&g_b.c_arm_created);
            if (g_b.last_arm_body) InterlockedIncrement(&g_b.c_arm_destroyed);
        }
        g_b.last_arm_body = arm;
        InterlockedExchange(&g_b.s_arm_body_lo, (LONG)(DWORD)arm);
        InterlockedExchange(&g_b.s_arm_body_hi, (LONG)(DWORD)(arm >> 32));
        InterlockedExchange(&g_b.s_arm_objs_lo, (LONG)(DWORD)objs);
        InterlockedExchange(&g_b.s_arm_objs_hi, (LONG)(DWORD)(objs >> 32));
        InterlockedExchange(&g_b.s_arm_flag, flag);
        InterlockedOr(&g_b.seen_arm_flag, flag);

        if (g_b.a.arm_cam_rotate_shift) {
            const volatile short *r =
                (const volatile short *)(ULONG_PTR)g_b.a.arm_cam_rotate_shift;
            LONG xy = (LONG)((unsigned short)r[0] |
                             ((unsigned long)(unsigned short)r[1] << 16));
            InterlockedExchange(&g_b.s_arm_cam_rot_xy, xy);
            InterlockedExchange(&g_b.s_arm_cam_rot_z, (LONG)r[2]);
        }
        /* An OR accumulator can only ever prove a bit was SET. It says nothing
           about whether that bit was ever clear, and reading "seen 0x0000F122"
           as "the arm was hidden all session" was exactly that mistake - the
           accumulator was blind to the only transition the run was about.
           So: the same word ANDed, which proves the opposite direction, and a
           plain count of the ticks channel 0 was actually visible. If the arm
           appears only while the trigger is held, three heartbeats will miss it
           and this counter will not. */
        if (objs) {
            InterlockedAnd(&g_b.held_arm_flag, flag);
            if (!(flag & 0x1000)) InterlockedIncrement(&g_b.c_arm_visible);
        }

        if (objs && !(status&0x1000ULL) && InterlockedCompareExchange(&g_b.arm_show, 0, 0) &&
            g_b.fps.state == DG_FPS_ACTIVE) {
            ULONGLONG evm = *(volatile ULONGLONG *)(ULONG_PTR)(arm + 0x30);
            WR32(objs + 0x58, (LONG)(flag & ~0x1000));
            if (evm) {
                LONG ef = RD32(evm + 0x58);
                WR32(evm + 0x58, (LONG)(ef & ~0x100));
            }
            InterlockedIncrement(&g_b.c_arm_forced);
        }
    }

    if (InterlockedCompareExchange(&g_b.work_probe, 0, 0) &&
        g_b.a.gm_player_arm_body) {
        ULONGLONG armbody =
            *(volatile ULONGLONG *)(ULONG_PTR)g_b.a.gm_player_arm_body;
        ULONGLONG work = 0, cam = 0, trig = 0, pwork = 0;
        ULONGLONG pbody = 0, pobjs = 0;
        LONG on = 0, chanl = -1, armtrig = 0, pflag = 0, ok = 0;

        if (plausible_ptr(armbody) && armbody > 0x60) {
            work = armbody - 0x60;
            chanl = RD32(work + 0x238);
            cam = *(volatile ULONGLONG *)(ULONG_PTR)(work + 0x248);
            if (chanl == 0 && plausible_ptr(cam)) {
                ok = 1;
                on = RD32(cam + 0x2C);
                /* trigger is an int*, so it is 4-aligned, not 8 - checking it
                   with the 8-aligned test would reject half the valid values
                   and quietly report nothing. */
                trig = *(volatile ULONGLONG *)(ULONG_PTR)(work + 0x228);
                if (trig >= 0x10000ULL && trig < 0x00007FFFFFFFFFFFULL &&
                    (trig & 3) == 0) {
                    armtrig = RD32(trig);
                    pwork = trig - 0xCF4;
                }
                pbody = *(volatile ULONGLONG *)(ULONG_PTR)(work + 0x1F8);
                if (plausible_ptr(pbody)) {
                    pobjs = *(volatile ULONGLONG *)(ULONG_PTR)pbody;
                    if (plausible_ptr(pobjs)) pflag = RD32(pobjs + 0x58);
                    else pobjs = 0;
                } else {
                    pbody = 0;
                }
            } else {
                work = cam = 0;
            }
        }
        if (ok && on) InterlockedIncrement(&g_b.c_cam_on);
        if (pobjs) {
            InterlockedAnd(&g_b.held_body_flag, pflag);
            if (!(pflag & 0x1000)) InterlockedIncrement(&g_b.c_body_visible);
            InterlockedOr(&g_b.seen_body_flag, pflag);
        }
        InterlockedExchange(&g_b.s_work_lo, (LONG)(DWORD)work);
        InterlockedExchange(&g_b.s_work_hi, (LONG)(DWORD)(work >> 32));
        InterlockedExchange(&g_b.s_pwork_lo, (LONG)(DWORD)pwork);
        InterlockedExchange(&g_b.s_pwork_hi, (LONG)(DWORD)(pwork >> 32));
        InterlockedExchange(&g_b.s_cam_on, on);
        InterlockedExchange(&g_b.s_arm_trigger, armtrig);
        InterlockedOr(&g_b.seen_arm_trigger, armtrig);
        InterlockedExchange(&g_b.s_body_lo, (LONG)(DWORD)pbody);
        InterlockedExchange(&g_b.s_body_hi, (LONG)(DWORD)(pbody >> 32));
        InterlockedExchange(&g_b.s_body_objs_lo, (LONG)(DWORD)pobjs);
        InterlockedExchange(&g_b.s_body_objs_hi, (LONG)(DWORD)(pobjs >> 32));
        InterlockedExchange(&g_b.s_body_flag, pflag);
    }

    /* Last, after every reader above has seen the tick's own values. */
    hud_tick();
}

/* One place decides what a config means for Move, so start and configure can
   never drift apart - an unknown number has to become NATIVE, because NATIVE is
   the only value that writes nothing. */

static int sane_bend_joint(const DG_BRIDGE_CONFIG *cfg)
{
    int j = cfg ? cfg->arm_bend_joint : 5;
    if (j <= 0 || j > 20 || j == 6) j = 5;
    return j;
}

/* Same reasoning as the two beside it: start and configure must not be able to
   drift apart, so the clamp lives in one place. */
static int sane_skel_base(const DG_BRIDGE_CONFIG *cfg)
{
    int b = cfg ? cfg->skel_base : 0;
    if (b < 0 || b >= DG_SKEL_MAX) return 0;
    return b;
}

/* Joint 0 has no parent to measure against and joint 6 is written every frame
   by SetPos, so neither can carry this measurement. Default 5, the forearm,
   which F5 step 1 already proved responds. */
static int sane_adj_joint(const DG_BRIDGE_CONFIG *cfg)
{
    int j = cfg ? cfg->adjust_probe_joint : 5;
    if (j <= 0 || j >= DG_SKEL_MAX || j == 6) return 5;
    return j;
}

static int sane_adj_axis(const DG_BRIDGE_CONFIG *cfg)
{
    int a = cfg ? cfg->adjust_probe_axis : 0;
    if (a < 0 || a > 2) return 0;
    return a;
}

static int sane_move_mode(const DG_BRIDGE_CONFIG *cfg)
{
    int m = cfg ? cfg->fps_move : DG_FPS_MOVE_NATIVE;
    if (m != DG_FPS_MOVE_OFF && m != DG_FPS_MOVE_ON) m = DG_FPS_MOVE_NATIVE;
    return m;
}

/* Same one-place rule for the theater: an unknown number is OFF, never a
   guess, because ON is the value that hands the camera to the director. */
static int sane_theater_mode(const DG_BRIDGE_CONFIG *cfg)
{
    int m = cfg ? cfg->theater_mode : DG_THEATER_OFF;
    if (m != DG_THEATER_MEASURE && m != DG_THEATER_ON) m = DG_THEATER_OFF;
    return m;
}

static int sane_menu_mode(const DG_BRIDGE_CONFIG *cfg)
{
    int mode = cfg ? cfg->menu_mode : 0;
    return mode >= 0 && mode <= 2 ? mode : 0;
}

static void hanging_visibility_reset(void);
int dg_bridge_start(void (*log)(const char *fmt, ...),
                    const DG_BRIDGE_CONFIG *cfg)
{
    LiveImage image;
    const char *why = "";
    int ok;

    if (InterlockedCompareExchange(&g_b.started, 1, 0) != 0) {
        dg_bridge_configure(cfg);
        return 1;
    }
    g_b.log = log;
    hanging_visibility_reset();
    memset(&g_interact_stats,0,sizeof g_interact_stats);
    script_menu_clear(0);
    InterlockedExchange(&g_b.c_pad_seam_entries, 0);
    InterlockedExchange(&g_b.c_start_queued, 0);
    InterlockedExchange(&g_b.c_start_consumed, 0);
    InterlockedExchange(&g_b.c_pad_context_refused, 0);
    InterlockedExchange(&g_b.script_menu_only, cfg && cfg->script_menu_only ? 1 : 0);
    dg_fps_init(&g_b.fps);
    InterlockedExchange(&g_b.mode, cfg ? cfg->fps_mode : DG_FPS_MODE_OFF);
    InterlockedExchange(&g_b.move_mode, sane_move_mode(cfg));
    InterlockedExchange(&g_b.menu_mode, sane_menu_mode(cfg));
    InterlockedExchange(&g_b.arm_show, cfg ? cfg->arm_show : 0);
    InterlockedExchange(&g_b.work_probe, cfg ? cfg->work_probe : 0);
    InterlockedExchange(&g_b.turn_probe, cfg ? cfg->turn_probe : 0);
    InterlockedExchange(&g_b.arm_uproll, cfg ? cfg->arm_uproll : 0);
    InterlockedExchange(&g_b.arm_hand_basis, cfg ? cfg->arm_hand_basis : 1);
    InterlockedExchange(&g_b.bend_deg, cfg ? cfg->arm_bend_deg : 0);
    InterlockedExchange(&g_b.bend_joint, sane_bend_joint(cfg));
    InterlockedExchange(&g_b.skel_probe, (cfg && cfg->skel_probe) ? 1 : 0);
    InterlockedExchange(&g_b.skel_base, sane_skel_base(cfg));
    InterlockedExchange(&g_b.adj_probe, (cfg && cfg->adjust_probe) ? 1 : 0);
    InterlockedExchange(&g_b.adj_joint, sane_adj_joint(cfg));
    InterlockedExchange(&g_b.adj_deg, cfg ? cfg->adjust_probe_deg : 30);
    InterlockedExchange(&g_b.adj_axis, sane_adj_axis(cfg));
    InterlockedExchange(&g_b.theater_mode, sane_theater_mode(cfg));
    InterlockedExchange(&g_b.theater_ui_mode,
                        (cfg && cfg->theater_ui) ? 1 : 0);
    InterlockedExchange(&g_b.hud_mode, (cfg && cfg->hud_mode) ? 1 : 0);
    InterlockedExchange(&g_b.bent_joint, -1);
    InterlockedExchange(&g_b.ik_active, 0);
    dg_arm_map_reset(&g_b.arm_map);
    memset(&g_b.camera_position, 0, sizeof g_b.camera_position);
    memset(&g_b.free_right, 0, sizeof g_b.free_right);
    g_b.arm_map_phase = 0;
    g_b.arm_map_cache_valid = 0;
    g_b.arm_map_last_pair = 0;
    g_b.arm_map_stream = 0;
    g_b.arm_map_arm = g_b.arm_map_objs = 0;
    g_b.ik_owned_arm = g_b.ik_owned_mctrl = g_b.ik_owned_adjust = 0;
    /* -1 means "no write has landed yet", so the first camera seam writes
       without reading. Starting at 0 would publish a matrix produced by
       whatever the joint happened to hold, labelled as the identity case. */
    InterlockedExchange(&g_b.adj_pending_case, -1);
    InterlockedExchange(&g_b.adj_write_tick, 0);
    InterlockedExchange(&g_b.seam_active_last_tick, -1);
    /* AND accumulators start at all-ones or they stay at zero forever, and a
       zero here would read as "every bit was clear at some point" - the exact
       false confidence this instrument exists to remove. */
    InterlockedExchange(&g_b.held_arm_flag, (LONG)0xFFFFFFFFu);
    InterlockedExchange(&g_b.held_arm_flag_late, (LONG)0xFFFFFFFFu);
    InterlockedExchange(&g_b.held_body_flag, (LONG)0xFFFFFFFFu);

    memset(&image, 0, sizeof(image));
    if (!read_self_image(&image)) {
        if (log) log("  bridge: no validated view of the loaded image\r\n");
        free_live_image(&image);
        InterlockedExchange(&g_b.started, 0);
        return 0;
    }
    if (!self_fingerprint_ok(&image)) {
        if (log) log("  bridge: FAIL CLOSED, executable fingerprint mismatch\r\n");
        free_live_image(&image);
        InterlockedExchange(&g_b.started, 0);
        return 0;
    }
    if (!dg_anchors_resolve(&image, &g_b.a)) {
        if (log) log("  bridge: FAIL CLOSED, F1B anchors did not all resolve\r\n");
        free_live_image(&image);
        InterlockedExchange(&g_b.started, 0);
        return 0;
    }
    interact_resolve_player(&image);
    if (log) log("  interactions: general player anchor %s\r\n",
        (g_interact_player.valid_bits&DG_RINV_ACTOR)?"resolved":"unavailable");
    radial_game_resolve(&image);
    health_resolve(&image);
    action_resolve(&image);
    camera_resolve(&image);
    zoom_resolve_image(&image,&g_camera_a,&g_zoom_a);
    move_resolve_workl(&image);
    m9_resolve(&image);
    blade_resolve(&image);
    stinger_resolve(&image);
    unarmed_prone_resolve(&image);
    native_hud_resolve(&image);
    coolant_resolve(&image);
    reload_resolve(&image);
    /* The anchor addresses are image-relative; the copy is about to go away. */
    free_live_image(&image);

    bridge_capture_ownership_snapshot();

    ok = 0;
    InterlockedExchange(&g_b.seam_is_copy, 0);
    if (cfg && cfg->tick_seam == 1) {
        /* The copy seam, opt-in (vr_tick_seam=copy). Refusal or an absent
           anchor falls back to the publication seam below, logged. */
        if (g_b.a.copy_seam) {
            ok = dg_detour_install(&g_b.detour,
                                   (void *)(ULONG_PTR)(g_b.a.copy_seam +
                                                       (ULONG_PTR)GetModuleHandleA(NULL)),
                                   (void *)bridge_tick,
                                   (const void *)(ULONG_PTR)(g_b.a.copy_function_begin +
                                                             (ULONG_PTR)GetModuleHandleA(NULL)),
                                   (const void *)(ULONG_PTR)(g_b.a.copy_function_end +
                                                             (ULONG_PTR)GetModuleHandleA(NULL)),
                                   &why);
            if (ok) InterlockedExchange(&g_b.seam_is_copy, 1);
            else if (log) log("  bridge: copy seam refused (%s) - using the publication seam\r\n", why);
        } else if (log) {
            log("  bridge: copy seam requested but not resolved - using the publication seam\r\n");
        }
    }
    if (!ok)
        ok = dg_detour_install(&g_b.detour,
                           (void *)(ULONG_PTR)(g_b.a.merge_seam +
                                               (ULONG_PTR)GetModuleHandleA(NULL)),
                           (void *)bridge_tick,
                           (const void *)(ULONG_PTR)(g_b.a.merge_function_begin +
                                                     (ULONG_PTR)GetModuleHandleA(NULL)),
                           (const void *)(ULONG_PTR)(g_b.a.merge_function_end +
                                                     (ULONG_PTR)GetModuleHandleA(NULL)),
                           &why);
    if (!ok) {
        if (log) log("  bridge: FAIL CLOSED, detour refused (%s)\r\n", why);
        InterlockedExchange(&g_b.started, 0);
        return 0;
    }

    native_hud_install();
    coolant_install();
    m9_install(1);
    stinger_install();
    unarmed_prone_install(cfg && cfg->unarmed_prone_enabled);
    blade_install(cfg && cfg->hf_blade);
    reload_install(1);
    g_reload.enabled=cfg && cfg->pistol_reload;
    InterlockedExchange(&g_m9_enabled,cfg && cfg->m9_slide ? 1:0);
    dg_bridge_mod_menu_request((cfg && cfg->pistol_reload?2u:0u)|(cfg && cfg->m9_slide?4u:0u));
    dg_bridge_mod_menu_capture(0);g_mod_capture_ticks=0;
    radial_game_install();
    /* The pad detour, installed only when its OPTIONAL anchor resolved and
       only when it has somewhere to write. Its failure is logged and
       dropped: the camera, the arm and walking must not be lost because a
       menu feature could not find its seam. */
    if (g_b.a.pad_press_ok && g_b.a.gv_pad_data_direct &&
        g_b.a.pad_direct_seam) {
        const char *pwhy = "unknown";
        ULONGLONG mod = (ULONGLONG)(ULONG_PTR)GetModuleHandleA(NULL);
        if (dg_detour_install(&g_b.pad_detour,
                              (void *)(ULONG_PTR)(g_b.a.pad_direct_seam + mod),
                              (void *)pad_seam_tick,
                              (const void *)(ULONG_PTR)
                                  (g_b.a.pad_update_begin + mod),
                              (const void *)(ULONG_PTR)
                                  (g_b.a.pad_update_end + mod),
                              &pwhy)) {
            InterlockedExchange(&g_b.pad_detour_live, 1);
        } else if (log) {
            log("  bridge: menu pad detour refused (%s) - front-end input "
                "stays off, everything else is unaffected\r\n", pwhy);
        }
    }

    action_install();
    camera_install();
    zoom_install();
    InterlockedExchange(&g_b.armed, 1);
    if (InterlockedCompareExchange(&g_b.mode, 0, 0) == DG_FPS_MODE_ALWAYS)
        warn_always_is_incomplete();
    /* Three calls, not one. The block is emitted per line-group so that adding
       a line can never push the tail of it off the end of the logger's buffer
       again - which is how the menu-status masks were lost the first time they
       mattered. Splitting is the fix; a bigger buffer only moves the cliff. */
    if (log) {
        log("  bridge: armed, mode %d, seam rva 0x%08lX (%u bytes stolen), "
            "owner %d\r\n"
            "    Override 0x%llX  Toggle 0x%llX  Active 0x%llX  Move 0x%llX\r\n",
            (int)g_b.mode,
            (unsigned long)(g_b.seam_is_copy ? g_b.a.copy_seam : g_b.a.merge_seam),
            g_b.detour.stolen, g_b.owner,
            g_b.a.gbp_override, g_b.a.gbp_toggle, g_b.a.gbp_active,
            g_b.a.gbp_move);
        log("  bridge: tick seam = %s (copy seam %s rva 0x%08lX, publication rva 0x%08lX)\r\n",
            g_b.seam_is_copy ? "COPY (before CheckDirection)" : "publication",
            g_b.a.copy_seam ? "resolved" : "not found",
            (unsigned long)g_b.a.copy_seam, (unsigned long)g_b.a.merge_seam);
        log("    PL_SubjectMove 0x%llX  PL_SubjectToggle 0x%llX\r\n"
            "    PL_PAD_SUBJECT 0x%llX  PL_PAD_STOP_AIM 0x%llX\r\n"
            "    PlayerPad.pad 0x%llX\r\n",
            g_b.a.pl_subject_move, g_b.a.pl_subject_toggle,
            g_b.a.pad_subject, g_b.a.pad_stop_aim, g_b.a.player_pad);
        /* GM_PlayerStatus has never once been logged with its address, and it
           has read 0 in every session so far - including with first person
           live, where PLAYER_WATCH must be set. Print where it thinks it is,
           so the next run can settle whether the word is wrong or the mask is
           simply never exercised. */
        log("    GM_PlayerStatus 0x%llX  (unsafe mask 0x%016llX)\r\n"
            "    GM_GameStatus 0x%llX  GM_GameStatusScn 0x%llX"
            "  (unsafe mask 0x%08lX)\r\n"
            "    GM_MenuStatus 0x%llX  GM_MenuStatusScn 0x%llX"
            "  (unsafe mask 0x%08lX)\r\n",
            g_b.a.gm_player_status,
            (unsigned long long)DG_PLAYER_UNSAFE_MASK,
            g_b.a.gm_game_status, g_b.a.gm_game_status_scn,
            (unsigned long)DG_GAME_UNSAFE_MASK,
            g_b.a.gm_menu_status, g_b.a.gm_menu_status_scn,
            (unsigned long)DG_MENU_UNSAFE_MASK);
    }
    return 1;
}

/* `always` promises suspension during menus, codec, cutscenes, pad demos, load
   and death (plan section 4). All three status words now back that promise:
   player-status for death, lockers, ladders and the menu-open bit, game-status
   for cutscenes, both demo kinds, cut-in and gameover, and menu-status for the
   codec. Each is read the way the game reads it - the Scn half ORed in - and
   each mask is pinned by a desk test in both directions.

   The warning stays, and it is not a formality. What the masks assert is which
   bits mean "not controllable gameplay", and only a live run can settle
   whether a codec call really does raise MENU_RADIO_ON on this build, or
   whether some sequence holds a bit we chose to permit. Until F2.5's live gate
   says otherwise, `always` is plausible rather than proven. */
static void warn_always_is_incomplete(void)
{
    if (InterlockedCompareExchange(&g_b.always_warned, 1, 0) != 0) return;
    if (!g_b.log) return;
    g_b.log("  bridge: NOTE vr_fps_mode=always now reads all three status "
            "words\r\n"
            "    (player 0x%016llX-mask, game 0x%08lX, menu 0x%08lX) - the bit "
            "semantics are not live-verified yet\r\n",
            (unsigned __int64)DG_PLAYER_UNSAFE_MASK,
            (unsigned long)DG_GAME_UNSAFE_MASK,
            (unsigned long)DG_MENU_UNSAFE_MASK);
}

/* Seed the follow's byte-sign accumulator from a marker declaration.
   +1/-1 pin the votes at a committed value; anything else leaves the
   per-session learning alone. Seeded, not latched: later drift responses
   still vote, so live evidence can overrule a stale marker - it just
   starts from knowledge instead of silence. */
static void turn_dir_seed(int td)
{
    if (td == 1 || td == -1)
        InterlockedExchange(&g_b.turn_dir_acc, (LONG)(td * 30));
}

void dg_bridge_configure(const DG_BRIDGE_CONFIG *cfg)
{
    int mode = cfg ? cfg->fps_mode : DG_FPS_MODE_OFF;
    InterlockedExchange(&g_b.script_menu_only, cfg && cfg->script_menu_only ? 1 : 0);
    if (mode < DG_FPS_MODE_OFF || mode > DG_FPS_MODE_ALWAYS)
        mode = DG_FPS_MODE_OFF;
    if (mode == DG_FPS_MODE_ALWAYS) warn_always_is_incomplete();
    InterlockedExchange(&g_b.mode, mode);
    InterlockedExchange(&g_b.move_mode, sane_move_mode(cfg));
    InterlockedExchange(&g_b.arm_show, (cfg && cfg->arm_show) ? 1 : 0);
    InterlockedExchange(&g_b.work_probe, (cfg && cfg->work_probe) ? 1 : 0);
    /* 0/1/2 travel as they are: 2 is the dense mode, not a truth value. */
    InterlockedExchange(&g_b.turn_probe, cfg ? cfg->turn_probe : 0);
    InterlockedExchange(&g_b.arm_uproll, (cfg && cfg->arm_uproll) ? 1 : 0);
    InterlockedExchange(&g_b.arm_hand_basis, cfg ? cfg->arm_hand_basis : 1);
    InterlockedExchange(&g_b.bend_deg, cfg ? cfg->arm_bend_deg : 0);
    InterlockedExchange(&g_b.bend_joint, sane_bend_joint(cfg));
    InterlockedExchange(&g_b.skel_probe, (cfg && cfg->skel_probe) ? 1 : 0);
    InterlockedExchange(&g_b.skel_base, sane_skel_base(cfg));
    InterlockedExchange(&g_b.adj_probe, (cfg && cfg->adjust_probe) ? 1 : 0);
    InterlockedExchange(&g_b.adj_joint, sane_adj_joint(cfg));
    InterlockedExchange(&g_b.adj_deg, cfg ? cfg->adjust_probe_deg : 30);
    InterlockedExchange(&g_b.adj_axis, sane_adj_axis(cfg));
    {   /* Clamped to a half turn either way. The field is a short and the game
           reads it as an angle, so a value that wraps is not a bigger rotation,
           it is a different one - and a probe whose input is not the number in
           the marker file proves nothing about anything. */
        int k;
        for (k = 0; k < 3; k++) {
            int v = cfg ? cfg->hand_probe_rot[k] : 0;
            if (v > 2048) v = 2048;
            if (v < -2048) v = -2048;
            InterlockedExchange(&g_b.hand_probe_rot[k], (LONG)v);
        }
        InterlockedExchange(&g_b.hand_probe, (cfg && cfg->hand_probe) ? 1 : 0);
        if (!(cfg && cfg->hand_probe)) hand_probe_release();
    }
    {   /* Both live modes are honoured here. Anything else becomes OFF -
           never DRY by accident, and never a write by accident. The mode this
           line settles on is the one the heartbeat reports, so a value that is
           refused here is refused visibly rather than by falling silent. */
        int fm = cfg ? cfg->fire_mode : DG_FIRE_MODE_OFF;
        if (fm != DG_FIRE_MODE_DRY && fm != DG_FIRE_MODE_ON)
            fm = DG_FIRE_MODE_OFF;
        InterlockedExchange(&g_b.fire_mode, (LONG)fm);
    }
    /* Theater and HUD, live like every other knob. The judgment machinery
       does not care about the mode (it always measures); the mode gates the
       verdict the consumers see and the flank logging. hud_mode going to 0
       is honoured by the next tick's hud_tick, which owes the clear. */
    InterlockedExchange(&g_b.theater_mode, sane_theater_mode(cfg));
    InterlockedExchange(&g_b.theater_ui_mode,
                        (cfg && cfg->theater_ui) ? 1 : 0);
    InterlockedExchange(&g_b.hud_mode, (cfg && cfg->hud_mode) ? 1 : 0);
    {   /* Negative asks would point the muzzle the wrong way and pull the hand
           away from the shoulder, so they are refused rather than clamped -
           same rule as dg_recoil_fire, and for the same reason. The upper
           bounds are sanity, not taste: past them the wrist envelope in dg_ik
           would be doing the limiting and the log would stop meaning what it
           says. */
        int cd = cfg ? cfg->recoil_climb_mdeg : 0;
        int pu = cfg ? cfg->recoil_push_um : 0;
        if (cd < 0) cd = 0;
        if (cd > 30000) cd = 30000;             /* 30 degrees */
        if (pu < 0) pu = 0;
        if (pu > 120000) pu = 120000;           /* 120 mm */
        InterlockedExchange(&g_b.arm_anchor, (cfg && cfg->arm_anchor) ? 1 : 0);
        {   /* F9. The deadzone is clamped to the range the pure module
               accepts, so the marker cannot configure a stick that is all
               deadzone or none. */
            int dz = cfg ? cfg->move_deadzone_mils : 150;
            if (dz < 0) dz = 0;
            if (dz > 900) dz = 900;
            InterlockedExchange(&g_b.walk_mode,
                                (cfg && cfg->move_mode) ? 1 : 0);
            InterlockedExchange(&g_b.move_deadzone_mils, (LONG)dz);
        }
        {   /* Third person / prone, clamped to what dg_move.c honours. */
            int pm = cfg ? cfg->move_prone_max : DG_MOVE_PRONE_DEFLECT;
            if (pm < DG_MOVE_GAME_MARGIN + 2) pm = DG_MOVE_GAME_MARGIN + 2;
            if (pm > 127) pm = 127;
            InterlockedExchange(&g_b.move_third, (cfg && cfg->move_third) ? 1 : 0);
            InterlockedExchange(&g_b.move_prone, (cfg && cfg->move_prone) ? 1 : 0);
            InterlockedExchange(&g_b.move_prone_max, (LONG)pm);
            InterlockedExchange(&g_b.move_dir_offset,
                                (LONG)((cfg ? cfg->move_dir_offset : 0) & 4095));
            InterlockedExchange(&g_b.move_dir_sign,
                                (cfg && cfg->move_dir_sign < 0) ? -1 : 1);
            InterlockedExchange(&g_b.move_dir_org, (cfg && cfg->move_dir_org) ? 1 : 0);
        }
        {   /* The turn's gain, clamped to the pure module's own bounds so the
               marker cannot configure a turn that the maths will re-clamp
               into something the log did not say. */
            int tg = cfg ? cfg->turn_gain_mils : 1000;
            if (tg < 100) tg = 100;
            if (tg > 3000) tg = 3000;
            InterlockedExchange(&g_b.turn_mode,
                                (cfg && cfg->turn_mode) ? 1 : 0);
            InterlockedExchange(&g_b.turn_gain_mils, (LONG)tg);
        }
        {   /* vr_menu. Clamped here rather than trusted, like every other
               mode word: an unknown number is OFF, never "write". */
            int mm = sane_menu_mode(cfg);
            InterlockedExchange(&g_b.menu_mode, (LONG)mm);
            if (mm < 2) {
                script_menu_clear(0);
                InterlockedExchange(&g_b.menu_status, 0);
                InterlockedExchange(&g_b.menu_allow, 0);
            }
        }
        {   /* vr_arm_freeze. A CHANGE re-zeroes the net meter's reference,
               so the number that follows is measured from the change and
               not from whatever the pose happened to be at load. 0 off,
               1 constant-replay, 2 rest-only; anything else is off. */
            LONG want = (cfg ? (LONG)cfg->arm_freeze : 0);
            if (want < 0 || want > 2) want = 0;
            if (InterlockedExchange(&g_b.arm_freeze, want) != want) {
                g_b.skel_hand_have_ref = 0;
                InterlockedExchange(&g_b.s_rest_drift, f2l(0.0f));
                InterlockedExchange(&g_b.s_rest_drift_max, f2l(0.0f));
            }
        }
        {   /* The hand envelope. Clamped to a half turn per stage: past
               180 a cap stops being a limit at all (the unwrap's branch
               choice owns everything beyond it), and 0 means default. */
            int ft = cfg ? cfg->hand_fore_twist_mdeg : 0;
            int wt = cfg ? cfg->hand_wrist_twist_mdeg : 0;
            int ws = cfg ? cfg->hand_wrist_swing_mdeg : 0;
            if (ft < 0 || ft > 180000) ft = 0;
            if (wt < 0 || wt > 180000) wt = 0;
            if (ws < 0 || ws > 180000) ws = 0;
            InterlockedExchange(&g_b.hand_fore_twist_mdeg, (LONG)ft);
            InterlockedExchange(&g_b.hand_wrist_twist_mdeg, (LONG)wt);
            InterlockedExchange(&g_b.hand_wrist_swing_mdeg, (LONG)ws);
        }
        {   /* Body-follows-aim knobs, clamped so the marker cannot ask for
               a threshold past the fold seams it exists to keep clear of,
               or a ramp that divides by zero. */
            int ft = cfg ? cfg->turn_follow_thresh_mdeg : 0;
            int ff = cfg ? cfg->turn_follow_full_mdeg : 0;
            if (ft < 0) ft = 0;
            if (ft > 120000) ft = 120000;
            if (ft && ff < ft + 5000) ff = ft + 5000;
            if (ff > 180000) ff = 180000;
            InterlockedExchange(&g_b.turn_follow_thresh_mdeg, (LONG)ft);
            InterlockedExchange(&g_b.turn_follow_full_mdeg, (LONG)ff);
            turn_dir_seed(cfg ? cfg->turn_dir_override : 0);
            {
                int hsv = cfg ? cfg->follow_head_sign : 0;
                InterlockedExchange(&g_b.follow_head_sign,
                                    (LONG)(hsv < 0 ? -1 : 1));
                InterlockedExchange(&g_b.follow_src,
                    (cfg && (cfg->follow_src == 1 || cfg->follow_src == 2))
                        ? (LONG)cfg->follow_src : 0);
                InterlockedExchange(&g_b.follow_aim,
                                    (cfg && cfg->follow_aim == 1) ? 1 : 0);
                InterlockedExchange(&g_b.adjust_frame,
                                    (cfg && cfg->adjust_frame == 1) ? 1 : 0);
                InterlockedExchange(&g_b.arm_comp,
                                    (cfg && cfg->arm_comp == 1) ? 1 : 0);
            }
        }
        InterlockedExchange(&g_b.recoil_climb_mdeg, (LONG)cd);
        InterlockedExchange(&g_b.recoil_push_um, (LONG)pu);
        if (!cd && !pu) dg_recoil_reset(&g_b.recoil);
    }
    if (mode == DG_FPS_MODE_OFF && InterlockedCompareExchange(&g_b.started, 0, 0))
        bridge_restore_once();
}

void dg_bridge_rec_pair(struct DG_REC_PAIRSTATE *out)
{
#if DG_ENABLE_DIAGNOSTICS

    if (!out) return;
    *out = g_b.rec_pair;

#else

#endif
}

void dg_bridge_rec_camera(const MAT *eye, const MAT *pers)
{
#if DG_ENABLE_DIAGNOSTICS

    dg_rec_pair_camera(&g_b.rec_pair, eye, pers);

#else

#endif
}

void dg_bridge_request_toggle(void)
{
    InterlockedExchange(&g_b.toggle_request_ms,(LONG)GetTickCount());
    InterlockedExchange(&g_b.toggle_request, 1);
}

void dg_bridge_cancel_toggle(void)
{
    InterlockedExchange(&g_b.toggle_request, 0);
}

int dg_bridge_menu_context_ready(void)
{
    return InterlockedCompareExchange(&g_b.armed, 0, 0) &&
           g_b.a.gm_player_arm_body &&
           *(volatile ULONGLONG *)(ULONG_PTR)g_b.a.gm_player_arm_body == 0;
}

void dg_bridge_start_now(void)
{
    if (InterlockedCompareExchange(&g_b.script_menu_only, 0, 0) &&
        !dg_bridge_menu_context_ready()) return;
    /* Do not queue input before the optional pad seam is live.  A request
       without that seam would otherwise survive until a later session and
       become an unexplained START press. */
    if (!InterlockedCompareExchange(&g_b.armed, 0, 0) ||
        !InterlockedCompareExchange(&g_b.pad_detour_live, 0, 0) ||
        !g_b.a.pad_press_ok)
        return;
    InterlockedExchange(&g_b.start_pending, 1);
    InterlockedIncrement(&g_b.c_start_queued);
}

void dg_bridge_drain_log(void)
{
    static LONG codec_last_requested, codec_last_written, codec_last_refused;
    LONG head;
    LONG tail;
    if (!g_b.log) return;
    if (codec_last_requested != g_codec_exit_requested ||
        codec_last_written != g_codec_exit_written || codec_last_refused != g_codec_exit_refused) {
        codec_last_requested = g_codec_exit_requested;
        codec_last_written = g_codec_exit_written;
        codec_last_refused = g_codec_exit_refused;
        g_b.log("  codec exit: B requests %ld delivered %ld context/chord refused %ld\r\n",
                codec_last_requested, codec_last_written, codec_last_refused);
    }
    if (g_interact_stats.samples &&
        (DWORD)(GetTickCount()-g_interact_stats.last_log)>=2000u) {
        g_interact_stats.last_log=GetTickCount();
        g_b.log("  interactions: samples %ld blocked %ld actor_missing %ld "
            "actor 0x%llX weapon %ld context %ld pad_writes %ld capture_writes %ld ladder_writes %ld "
            "codec queued %ld written %ld seam_refused %ld\r\n",
            g_interact_stats.samples,g_interact_stats.blocked,g_interact_stats.actor_missing,
            (unsigned long long)InterlockedCompareExchange64(&g_interact_stats.actor,0,0),
            g_interact_stats.weapon,g_interact_stats.context,g_interact_stats.pad_writes,g_interact_stats.capture_writes,g_interact_stats.ladder_writes,
            g_interact_stats.codec_queued,g_interact_stats.codec_written,g_interact_stats.codec_dropped);
    }
    head = InterlockedCompareExchange(&g_ring_head, 0, 0);
    tail = g_ring_tail;
    if (head - tail > DG_RING) {
        InterlockedExchange(&g_b.c_dropped,
                            g_b.c_dropped + (head - tail - DG_RING));
        tail = head - DG_RING;
    }
    for (; tail < head; tail++) {
        const DG_FPS_EVENT *e = &g_ring[tail & (DG_RING - 1)];
        g_b.log("  bridge: %s -> %s (%s) write %d at tick %lu\r\n",
                dg_fps_state_name((int)e->from_state),
                dg_fps_state_name((int)e->to_state),
                dg_fps_reason_name((int)e->reason),
                (int)e->write, e->tick);
        /* Only where it settles something. Every other transition is explained
           by its own name, and a status dump on all of them would bury the two
           lines that matter. */
        if (e->reason == DG_FPS_REASON_UNSAFE ||
            e->reason == DG_FPS_REASON_LEVEL_LOAD)
            g_b.log("      status: game 0x%08X  menu 0x%08X  "
                    "player 0x%016llX\r\n",
                    e->game_status, e->menu_status, e->player_status);
    }
    g_ring_tail = tail;

    /* Theater flanks, the phase-0 evidence: each line pairs a switch with the
       exact words that caused it, so a wrong flank names its own bit. Only
       pushed under vr_theater=measure|on (or vr_ui=on for the panel), so a
       default session's log is unchanged. */
    head = InterlockedCompareExchange(&g_thea_ring_head, 0, 0);
    tail = g_thea_ring_tail;
    if (head - tail > DG_THEA_RING) tail = head - DG_THEA_RING;
    for (; tail < head; tail++) {
        const DG_THEA_EVENT *e = &g_thea_ring[tail & (DG_THEA_RING - 1)];
        g_b.log("  theater: %s %s at sample %lu (t=%lums)  game 0x%08X"
                "  menu 0x%08X  player 0x%016llX\r\n",
                (e->kind & 0xFF) == DG_THEATER_V_UI ? "ui-panel" : "demo/codec",
                (e->kind & 0x100) ? "EXIT" : "ENTER",
                e->sample, e->ms, e->game_status, e->menu_status,
                e->player_status);
    }
    g_thea_ring_tail = tail;
}

static void arm_bend_release(void);   /* defined with its writer, below */
static void arm_ik_release(void);
static void arm_map_forget(void);
static void left_arm_release(void);
static void hand_pose_release(void);

void dg_bridge_stop(void)
{
    dg_bridge_controls_context(0);
    script_menu_clear(0);
    if (!InterlockedCompareExchange(&g_b.started, 0, 0)) {
        hanging_visibility_reset();
        InterlockedExchange(&g_b.script_menu_only, 0);
        return;
    }
    InterlockedExchange(&g_b.armed, 0);
    /* Let any in-flight tick finish before the values move under it. */
    Sleep(50);
    hanging_visibility_reset();
    /* The HUD's release debt, paid on the same once-only path as the borrowed
       pad values: clear exactly the four bits this session held, and only if
       it ever held them. From here on the tick seam no longer runs, so this
       is the last writer. */
    if (InterlockedCompareExchange(&g_b.hud_held, 0, 0) &&
        g_b.a.gm_menu_status) {
        WR32(g_b.a.gm_menu_status,
             RD32(g_b.a.gm_menu_status) & ~(LONG)DG_HUD_HIDE_BITS);
        InterlockedExchange(&g_b.hud_held, 0);
        InterlockedIncrement(&g_b.c_hud_cleared);
    }
    arm_bend_release();
    arm_map_forget();
    left_arm_release();
    bridge_restore_once();
    if (g_radial_game.live) {
        dg_detour_remove(&g_radial_game.detour);
        g_radial_game.live=0;
    }
    dg_bridge_action_register(NULL);
    reload_stop();
    stinger_stop();
    unarmed_prone_stop();
    native_hud_stop();
    coolant_stop();
    m9_stop();
    blade_stop();
    dg_detour_remove(&g_action_detour);
    dg_detour_remove(&g_camera_detour);
    dg_detour_remove(&g_b.detour);
    if (InterlockedCompareExchange(&g_b.pad_detour_live, 0, 0)) {
        dg_detour_remove(&g_b.pad_detour);
        InterlockedExchange(&g_b.pad_detour_live, 0);
    }
    InterlockedExchange(&g_b.started, 0);
    InterlockedExchange(&g_b.script_menu_only, 0);
}

static void arm_bend_now(ULONGLONG arm, const double *quat_xyzw)
{
    ULONGLONG mctrl, adjust;
    LONG deg, joint, joints;
    double rad, s, c;
    double q[4];

    deg = InterlockedCompareExchange(&g_b.bend_deg, 0, 0);
    joint = InterlockedCompareExchange(&g_b.bend_joint, 0, 0);
    if (joint < 0 || joint > 20 || joint == 6) return;

    /* The OBJECT itself, dumped before any gate can swallow the reason. The
       run of 2026-08-14 produced no bend line at all, which means m_ctrl was
       never even a plausible pointer - a different failure from the one before
       it, and one the m_ctrl dump could not report because it sat behind the
       very check that was failing. Two known values make this dump
       self-checking: +0x00 must equal the objs already logged and +0x30 the
       evmobj, both fixed by retail at 0x0057D5A2 and 0x0057D5AC. */
    {
        int i;
        for (i = 0; i < 8; i++) {
            ULONGLONG v = *(volatile ULONGLONG *)(ULONG_PTR)(arm + i * 8);
            InterlockedExchange(&g_b.s_obj_dump[i * 2], (LONG)(DWORD)v);
            InterlockedExchange(&g_b.s_obj_dump[i * 2 + 1],
                                (LONG)(DWORD)(v >> 32));
        }
    }

    mctrl = *(volatile ULONGLONG *)(ULONG_PTR)(arm + 0x08);
    if (!plausible_ptr(mctrl)) return;
    /* HUMAN21 means 21 joints, so this is a free and very strong check that
       the whole struct reading is right before anything is written. */
    joints = RD32(mctrl + 0x14);
    InterlockedExchange(&g_b.s_arm_joints, joints);
    adjust = *(volatile ULONGLONG *)(ULONG_PTR)(mctrl + 0x48);
    InterlockedExchange(&g_b.s_arm_adjust_lo, (LONG)(DWORD)adjust);
    InterlockedExchange(&g_b.s_arm_adjust_hi, (LONG)(DWORD)(adjust >> 32));
    InterlockedExchange(&g_b.s_arm_mctrl_lo, (LONG)(DWORD)mctrl);
    InterlockedExchange(&g_b.s_arm_mctrl_hi, (LONG)(DWORD)(mctrl >> 32));
    /* The gate refused on 2026-08-14 with n_joints reading 55 and the adjust
       pointer null, which cannot both be true of a MOTION_CONTROL that SetPos
       writes adjust[6] into every frame. Rather than theorise about which
       offset moved, dump the head of the block and let it say. Torn reads are
       fine here: it is a diagnostic, not a decision. */
    {
        int i;
        for (i = 0; i < 10; i++) {
            ULONGLONG v = *(volatile ULONGLONG *)(ULONG_PTR)(mctrl + i * 8);
            InterlockedExchange(&g_b.s_mctrl_dump[i * 2], (LONG)(DWORD)v);
            InterlockedExchange(&g_b.s_mctrl_dump[i * 2 + 1],
                                (LONG)(DWORD)(v >> 32));
        }
    }

    if (joints < 21 || joints > 255) return;
    if (joint >= joints) return;
    if (!plausible_ptr(adjust)) return;

    /* A tracked controller wins; the fixed angle is the fallback that proved
       the path. With neither, the joint is released rather than left holding
       whatever it held last - F5's tracking-loss rule is that a frozen hand in
       the world is worse than no hand. */
    if (quat_xyzw) {
        q[0] = quat_xyzw[0];
        q[1] = quat_xyzw[1];
        q[2] = quat_xyzw[2];
        q[3] = quat_xyzw[3];
        InterlockedIncrement(&g_b.c_arm_tracked);
    } else if (deg) {

        rad = (double)deg * 3.14159265358979323846 / 180.0;
        s = sin(rad * 0.5);
        c = cos(rad * 0.5);
        q[0] = s; q[1] = 0.0; q[2] = 0.0; q[3] = c;
    } else {
        if (InterlockedCompareExchange(&g_b.bent_joint, 0, 0) >= 0)
            arm_bend_release();
        return;
    }

    {
        double n = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
        if (!(n > 0.9 && n < 1.1)) {
            InterlockedIncrement(&g_b.c_arm_quat_refused);
            return;
        }
    }
    {
        volatile float *w = (volatile float *)(ULONG_PTR)(adjust + joint * 16);
        w[0] = (float)q[0];
        w[1] = (float)q[1];
        w[2] = (float)q[2];
        w[3] = (float)q[3];
    }
    *(volatile ULONGLONG *)(ULONG_PTR)(mctrl + 0x38) |= (1ULL << joint);
    InterlockedExchange(&g_b.bent_joint, joint);
    InterlockedIncrement(&g_b.c_arm_bent);
}

/* Put the joint back the way an untouched joint looks: identity quaternion and
   the bit clear. Nothing else writes joint 5, so this is the exact original
   rather than an approximation of it - and it does not need a saved copy that
   could outlive the object it came from. */
static void arm_bend_release(void)
{
    ULONGLONG arm, mctrl, adjust;
    LONG joint = InterlockedExchange(&g_b.bent_joint, -1);

    if (joint < 0 || joint > 20) return;
    if (!g_b.a.gm_player_arm_body) return;
    arm = *(volatile ULONGLONG *)(ULONG_PTR)g_b.a.gm_player_arm_body;
    if (!plausible_ptr(arm)) return;
    mctrl = *(volatile ULONGLONG *)(ULONG_PTR)(arm + 0x08);
    if (!plausible_ptr(mctrl)) return;
    {
        LONG n = RD32(mctrl + 0x14);
        if (n < 21 || n > 255 || joint >= n) return;
    }
    adjust = *(volatile ULONGLONG *)(ULONG_PTR)(mctrl + 0x48);
    if (!plausible_ptr(adjust)) return;
    {
        volatile float *q = (volatile float *)(ULONG_PTR)(adjust + joint * 16);
        q[0] = q[1] = q[2] = 0.0f;
        q[3] = 1.0f;
    }
    *(volatile ULONGLONG *)(ULONG_PTR)(mctrl + 0x38) &= ~(1ULL << joint);
}

/* The four-axis run of 2026-08-18 measured the fixed conjugation below. Rows
   are the world axes produced by adjust-space X, Y and Z respectively. The
   logged joint-0 world rotation missed the three cases by 0.070..0.099 and is
   therefore NOT folded in here; this closest SO(3) fit reproduced them within
   0.188 degrees. For a world-axis quaternion vector v, adjust coordinates are
   v * R^-1 = v * R^T, i.e. one dot product with each measured row. */
static const double DG_ADJUST_TO_WORLD[3][3] = {
    { 0.001786891,  0.004505413, -0.999988254 },
    { 0.002787253,  0.999985944,  0.004510383 },
    { 0.999994519, -0.002795280,  0.001774308 }
};

/* Angle between two 3-vectors in degrees; degenerate input reads as 0,
   never as drift - a collapsed live bone must not raise the alarm the
   meter exists for. */
static double v_angle_deg(const double a[3], const double b[3])
{
    double la = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    double lb = sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
    double c;
    if (!(la > 1e-9) || !(lb > 1e-9)) return 0.0;
    c = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (la * lb);
    if (c > 1.0) c = 1.0;
    if (c < -1.0) c = -1.0;
    return acos(c) * 180.0 / 3.14159265358979323846;
}

static double vdist3(const double a[3], const double b[3])
{
    double x = a[0] - b[0], y = a[1] - b[1], z = a[2] - b[2];
    return sqrt(x * x + y * y + z * z);
}

/* The arm rig is camera-local but its matrices are expressed in their own
   affine frame. Joint 0 is that live frame. The 2026-08-18 spin video proved
   that substituting the native camera's translation can move the target more
   than a metre away, so this transform deliberately accepts no camera matrix. */
static void arm_view_to_world(const double root[4][4], const double view[3],
                              double out[3])
{
    int j;
    for (j = 0; j < 3; j++)
        out[j] = root[3][j] + view[0] * root[0][j] +
                 view[1] * root[1][j] + view[2] * root[2][j];
}

/* Inverse of arm_view_to_world for a general finite 3x3 root basis. The live
   matrices are normally orthonormal, but using the real inverse keeps a small
   rig scale from turning calibration into a character-dependent units bug. */
static int arm_world_to_view(const double root[4][4], const double world[3],
                             double out[3])
{
    double a = root[0][0], b = root[0][1], c = root[0][2];
    double d = root[1][0], e = root[1][1], f = root[1][2];
    double g = root[2][0], h = root[2][1], i = root[2][2];
    double det = a * (e * i - f * h) - b * (d * i - f * g) +
                 c * (d * h - e * g);
    double x, y, z;
    if (!_finite(det) || fabs(det) <= 1e-9) return 0;
    x = world[0] - root[3][0];
    y = world[1] - root[3][1];
    z = world[2] - root[3][2];
    /* Row-vector v satisfies v*B = world-root. */
    out[0] = (x * (e * i - f * h) + y * (f * g - d * i) +
              z * (d * h - e * g)) / det;
    out[1] = (x * (c * h - b * i) + y * (a * i - c * g) +
              z * (b * g - a * h)) / det;
    out[2] = (x * (b * f - c * e) + y * (c * d - a * f) +
              z * (a * e - b * d)) / det;
    return _finite(out[0]) && _finite(out[1]) && _finite(out[2]);
}

/* A controller may legitimately exceed the character's reach and let the IK
   clamp at the elbow. It may not be farther from the arm root than the entire
   root-to-shoulder offset plus 1.25 arm lengths. That larger envelope keeps
   ordinary human/controller proportions legal while refusing the 1.34..1.59 m
   frame mismatch that made all 3365 writes in the spin run clamp. */
static int arm_target_plausible(const double root[3],
                                const double joint[5][3],
                                const double target[3], double *distance,
                                double *limit)
{
    double upper = vdist3(joint[2], joint[3]);
    double fore = vdist3(joint[3], joint[4]);
    double root_shoulder = vdist3(root, joint[2]);
    double d = vdist3(root, target);
    double lim = root_shoulder + 1.25 * (upper + fore);
    if (distance) *distance = d;
    if (limit) *limit = lim;
    return _finite(d) && _finite(lim) && upper > 0.0 && fore > 0.0 &&
           root_shoulder >= 0.0 && d <= lim;
}

/* ------------------------------------------------ the adjust frame (V5.1) ---
   The constant above is the frozen heading of one measurement session. The
   adjust-probe runs of 2026-08-30 and 2026-09-01 (ROLL_ONTWERP_V5.1 par. 3.1)
   measured what it stands in for: the world image of adjust-X has heading
   atan2(z, x) = DG_FRAME_HEADING_SIGN * rot.vy * (360/4096) + DG_FRAME_ZERO_DEG
   with rot.vy the actor's body yaw (PlayerWork+0x82), rms 0.10 degrees over
   432 cycles and 12 headings, the frame yaw-only (root tilt never reaches
   it). So world = f (x) adjust (x) f* with f that yaw. Which frame a
   conversion uses is one runtime choice, vr_adjust_frame=legacy|live:
   legacy is the constant, byte for byte the behaviour every build so far
   shipped; live is the measured heading. Both helpers take the frame as a
   mandatory parameter and return 0 - output untouched - for a NULL, invalid,
   non-finite or non-unit frame, so a pair that never acquired a heading
   fails at the site's own fail-closed policy instead of converting with
   yesterday's. */
/* DG_ADJ_FRAME itself lives in dg_bridge.h: the bridge state holds one. */
static const DG_ADJ_FRAME ADJ_FRAME_LEGACY = { 0, 1, { 0.0, 0.0, 0.0, 1.0 } };

/* Meetplan A' (2026-09-01): heading(adjust-X in world) = SIGN * rot.vy *
   360/4096 + ZERO, heading = atan2(z, x). Measured sign -1, zero -0.002
   degrees (below the 0.088-degree grid, so 0). */
#define DG_FRAME_HEADING_SIGN (-1.0)
#define DG_FRAME_ZERO_DEG (0.0)

/* The frame quaternion for a body yaw word. dg_ik's yaw quaternion of angle
   +phi sends x to (cos phi, 0, -sin phi), i.e. to heading -phi (pinned by
   the f_yaw73 oracle in t_adjust_frame), so the quaternion's angle is minus
   the measured heading. Yaw-only by construction: the probe showed the
   9-degree root tilt never reaches the mapping. */
static int arm_frame_yaw(short rot_vy, double f[4])
{
    double heading = DG_FRAME_HEADING_SIGN * (double)rot_vy * (360.0 / 4096.0) +
                     DG_FRAME_ZERO_DEG;
    double half = -0.5 * heading * (3.14159265358979323846 / 180.0);
    f[0] = 0.0; f[1] = sin(half); f[2] = 0.0; f[3] = cos(half);
    return 1;
}

static int adj_frame_usable(const DG_ADJ_FRAME *f)
{
    double n;
    if (!f || !f->valid) return 0;
    if (!f->live) return 1;
    n = f->q[0] * f->q[0] + f->q[1] * f->q[1] + f->q[2] * f->q[2] +
        f->q[3] * f->q[3];
    return _finite(n) && fabs(sqrt(n) - 1.0) < 1e-3;
}

static int world_quat_to_adjust(const DG_ADJ_FRAME *frame,
                                const double world[4], double adjust[4])
{
    double n, out[4];
    int r;
    if (!adj_frame_usable(frame)) return 0;
    if (frame->live) {
        /* The way back: a = f* (x) w (x) f. */
        const double *frame_yaw_q = frame->q;
        double f_conj[4], t[4];
        dg_ik_quat_conj(frame_yaw_q, f_conj);
        dg_ik_quat_mul(f_conj, world, t);
        dg_ik_quat_mul(t, frame_yaw_q, out);
    } else {
        for (r = 0; r < 3; r++)
            out[r] = world[0] * DG_ADJUST_TO_WORLD[r][0] +
                     world[1] * DG_ADJUST_TO_WORLD[r][1] +
                     world[2] * DG_ADJUST_TO_WORLD[r][2];
        out[3] = world[3];
    }
    n = sqrt(out[0] * out[0] + out[1] * out[1] + out[2] * out[2] +
             out[3] * out[3]);
    if (!_finite(n) || !(n > 0.0)) return 0;
    for (r = 0; r < 4; r++) adjust[r] = out[r] / n;
    return 1;
}

static int adjust_quat_to_world(const DG_ADJ_FRAME *frame,
                                const double adjust[4], double world[4])
{
    double n, out[4];
    int c, r;

    if (!adj_frame_usable(frame)) return 0;
    if (frame->live) {
        const double *frame_yaw_q = frame->q;
        double f_conj[4], t[4];
        /* The adjust lives in the actor's heading frame: w = f (x) a (x) f*. */
        dg_ik_quat_mul(frame_yaw_q, adjust, t);
        dg_ik_quat_conj(frame_yaw_q, f_conj);
        dg_ik_quat_mul(t, f_conj, out);
    } else {
        for (c = 0; c < 3; c++) {
            out[c] = 0.0;
            for (r = 0; r < 3; r++)
                out[c] += adjust[r] * DG_ADJUST_TO_WORLD[r][c];
        }
        out[3] = adjust[3];
    }
    n = sqrt(out[0] * out[0] + out[1] * out[1] + out[2] * out[2] +
             out[3] * out[3]);
    if (!_finite(n) || !(n > 0.0)) return 0;
    for (c = 0; c < 4; c++) world[c] = out[c] / n;
    return 1;
}

static void arm_quat_rotate(const double q[4], const double v[3],
                            double out[3])
{
    double ux = q[0], uy = q[1], uz = q[2], w = q[3];
    double cx = uy * v[2] - uz * v[1];
    double cy = uz * v[0] - ux * v[2];
    double cz = ux * v[1] - uy * v[0];
    double ccx = uy * cz - uz * cy;
    double ccy = uz * cx - ux * cz;
    double ccz = ux * cy - uy * cx;
    out[0] = v[0] + 2.0 * (w * cx + ccx);
    out[1] = v[1] + 2.0 * (w * cy + ccy);
    out[2] = v[2] + 2.0 * (w * cz + ccz);
}

static void arm_quat_unrotate(const double q[4], const double v[3],
                              double out[3])
{
    double inverse[4] = { -q[0], -q[1], -q[2], q[3] };
    arm_quat_rotate(inverse, v, out);
}

/* ------------------------------- F5 step 3a: the ArmCamRotateShift probe --- */

/* Defined with the skeleton probe further down; used here first. */
static LONG f2l(float f);

static void set_pos_quat(const double rot[3], double q[4])
{
    const double to_rad = 2.0 * 3.14159265358979323846 / 4096.0;
    double x = rot[0] * to_rad, y = rot[1] * to_rad, z = rot[2] * to_rad;

    if (rot[1] != 0.0) {
        double cr = cos(x * 0.5), sr = sin(x * 0.5);
        double cp = cos(y * 0.5), sp = sin(y * 0.5);
        double cy = cos(z * 0.5), sy = sin(z * 0.5);
        double cpcy = cp * cy, spsy = sp * sy;
        q[3] = cr * cpcy + sr * spsy;
        q[0] = sr * cpcy - cr * spsy;
        q[1] = cr * sp * cy + sr * cp * sy;
        q[2] = cr * cp * sy - sr * sp * cy;
    } else {
        double r;
        q[3] = cos(x * 0.5);
        r = 1.0 - q[3] * q[3];
        r = (r > 0.0) ? sqrt(r) : 0.0;
        if (x < 0.0) r = -r;
        q[0] = cos(y) * r;
        q[2] = -sin(y) * r;
        q[1] = 0.0;
    }
    {   /* GM_RotToQuat normalises; XAfterY is unit by construction. Doing it
           for both costs nothing and removes one way for the comparison below
           to differ for an uninteresting reason. */
        double n = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        if (n > 0.0) { q[0] /= n; q[1] /= n; q[2] /= n; q[3] /= n; }
    }
}

/* The tick-seam half. This runs at the Action() merge point, ahead of the arm
   actor, which is the only reason the value can reach SetPos in the same
   frame - the camera seam is behind the hierarchy pass and would always be a
   frame late. Whether it really is early enough is one of the things the probe
   is here to find out, and the read-back half answers it. */
static void hand_probe_write(void)
{
    volatile short *r;
    int k;

    if (!InterlockedCompareExchange(&g_b.hand_probe, 0, 0)) return;
    if (!g_b.a.arm_cam_rotate_shift) return;
    r = (volatile short *)(ULONG_PTR)g_b.a.arm_cam_rotate_shift;
    for (k = 0; k < 3; k++) {
        LONG v = InterlockedCompareExchange(&g_b.hand_probe_rot[k], 0, 0);
        r[k] = (short)v;
        InterlockedExchange(&g_b.s_hand_probe_wrote[k], v);
    }
    InterlockedExchange(&g_b.hand_probe_owned, 1);
    InterlockedIncrement(&g_b.c_hand_probe_writes);
}

/* Give the SVECTOR back. Only ever writes zero, which is what every weapon's
   table entry holds, so this cannot leave the game somewhere it would not
   have been on its own. */
static void hand_probe_release(void)
{
    volatile short *r;
    int k;

    if (!InterlockedExchange(&g_b.hand_probe_owned, 0)) return;
    if (!g_b.a.arm_cam_rotate_shift) return;
    r = (volatile short *)(ULONG_PTR)g_b.a.arm_cam_rotate_shift;
    for (k = 0; k < 3; k++) r[k] = 0;
}

/* A bounded seqlock: there is one normal publisher (the camera seam), but
   configure/stand-down may invalidate from another thread. Contention means
   no command, never a torn angle triplet. */
static int hand_command_lock(LONG *locked)
{
    int tries;
    for (tries = 0; tries < 8; tries++) {
        LONG s = InterlockedCompareExchange(&g_b.hand_command_seq, 0, 0);
        if (s & 1) continue;
        if (InterlockedCompareExchange(&g_b.hand_command_seq, s + 1, s) == s) {
            *locked = s + 1;
            return 1;
        }
    }
    return 0;
}

static void hand_command_unlock(LONG locked)
{
    MemoryBarrier();
    InterlockedExchange(&g_b.hand_command_seq, locked + 1);
}

static void hand_command_clear(void)
{
    LONG locked;
    /* Immediate fail-closed invalidation even if another writer owns seq. */
    InterlockedExchange(&g_b.hand_command_valid, 0);
    if (!hand_command_lock(&locked)) return;
    InterlockedExchange(&g_b.hand_command_valid, 0);
    hand_command_unlock(locked);
}

static int hand_command_publish(const short rot[3], unsigned long pair)
{
    LONG locked;
    int k;
    if (!rot || !hand_command_lock(&locked)) return 0;
    for (k = 0; k < 3; k++)
        InterlockedExchange(&g_b.hand_command_rot[k], (LONG)rot[k]);
    InterlockedExchange(&g_b.hand_command_tick,
                        InterlockedCompareExchange(&g_b.c_ticks, 0, 0));
    InterlockedExchange(&g_b.hand_command_pair, (LONG)pair);
    InterlockedExchange(&g_b.hand_command_valid, 1);
    hand_command_unlock(locked);
    return 1;
}

static int hand_command_read(short rot[3], LONG *tick, LONG *pair)
{
    int tries, k;
    for (tries = 0; tries < 8; tries++) {
        LONG before = InterlockedCompareExchange(&g_b.hand_command_seq, 0, 0);
        LONG valid, t, p, v[3];
        if (before & 1) continue;
        valid = InterlockedCompareExchange(&g_b.hand_command_valid, 0, 0);
        for (k = 0; k < 3; k++)
            v[k] = InterlockedCompareExchange(&g_b.hand_command_rot[k], 0, 0);
        t = InterlockedCompareExchange(&g_b.hand_command_tick, 0, 0);
        p = InterlockedCompareExchange(&g_b.hand_command_pair, 0, 0);
        MemoryBarrier();
        if (before != InterlockedCompareExchange(&g_b.hand_command_seq, 0, 0))
            continue;
        if (!valid) return 0;
        for (k = 0; k < 3; k++) rot[k] = (short)v[k];
        if (tick) *tick = t;
        if (pair) *pair = p;
        return 1;
    }
    return 0;
}

/* The camera-seam half, after SetPos has run. Reading the SVECTOR and the
   adjust slot in the same pass is what makes this a measurement rather than a
   pairing exercise: no cross-frame bookkeeping, so nothing can be lined up
   against the wrong frame. */
static void hand_probe_read(ULONGLONG arm)
{
    ULONGLONG mctrl, adjust;
    LONG joints;
    const volatile short *r;
    const volatile float *a;
    double rot[3], want[4], worst = 0.0;
    int k;

    if (!InterlockedCompareExchange(&g_b.hand_probe, 0, 0)) return;
    if (!g_b.a.arm_cam_rotate_shift) return;
    r = (const volatile short *)(ULONG_PTR)g_b.a.arm_cam_rotate_shift;
    for (k = 0; k < 3; k++) {
        rot[k] = (double)r[k];
        InterlockedExchange(&g_b.s_hand_probe_read[k], (LONG)r[k]);
    }
    set_pos_quat(rot, want);

    mctrl = *(volatile ULONGLONG *)(ULONG_PTR)(arm + 0x08);
    if (!plausible_ptr(mctrl)) return;
    joints = RD32(mctrl + 0x14);
    adjust = *(volatile ULONGLONG *)(ULONG_PTR)(mctrl + 0x48);
    if (joints <= 6 || joints > 255 || !plausible_ptr(adjust)) return;
    a = (const volatile float *)(ULONG_PTR)(adjust + 6 * 16);
    {
        int all_zero = 1;
        for (k = 0; k < 4; k++)
            if (a[k] != 0.0f) all_zero = 0;
        if (all_zero) {
            for (k = 0; k < 4; k++) {
                InterlockedExchange(&g_b.s_hand_probe_adjust[k], f2l(0.0f));
                InterlockedExchange(&g_b.s_hand_probe_pred[k],
                                    f2l((float)want[k]));
            }
            InterlockedIncrement(&g_b.c_hand_probe_no_setpos);
            InterlockedIncrement(&g_b.c_hand_probe_reads);
            return;
        }
    }
    for (k = 0; k < 4; k++) {
        double got = (double)a[k];
        double d = fabs(got - want[k]);
        if (d > worst) worst = d;
        InterlockedExchange(&g_b.s_hand_probe_adjust[k], f2l((float)got));
        InterlockedExchange(&g_b.s_hand_probe_pred[k], f2l((float)want[k]));
    }
    InterlockedExchange(&g_b.s_hand_probe_worst, f2l((float)worst));
    InterlockedIncrement(&g_b.c_hand_probe_reads);
}

/* Re-express a rotation given in the arm root's frame as a world rotation.
   The root basis rows are the world directions of that frame's axes, so a
   quaternion's axis travels with them while its angle does not change.
   Passing the root quaternion converts root-frame to world; passing its
   inverse converts the other way. Which of the two a caller wants is the only
   thing that can be wrong here, so neither direction is hidden in a name. */
static void quat_rebase(const double basis_q[4], const double q[4],
                        double out[4])
{
    double v[3] = { q[0], q[1], q[2] };
    double r[3];
    arm_quat_rotate(basis_q, v, r);
    out[0] = r[0]; out[1] = r[1]; out[2] = r[2]; out[3] = q[3];
}

static void quat_identity(double q[4])
{
    q[0] = 0.0; q[1] = 0.0; q[2] = 0.0; q[3] = 1.0;
}

/* The yaw (twist about world up, +Y - the axis arm_frame_yaw turns about)
   of a rotation, swing-twist style: the rest of q is its swing, q = s * t.
   A rotation lying in the horizontal plane has no yaw and yields identity. */
static void quat_ytwist(const double q[4], double t[4])
{
    double n = sqrt(q[1] * q[1] + q[3] * q[3]);
    if (!(n > 1e-9)) { quat_identity(t); return; }
    t[0] = 0.0; t[1] = q[1] / n; t[2] = 0.0; t[3] = q[3] / n;
}

static int arm_hand_solve(const double root_q[4], const double live_root_q[4],
                          const double live_hand[4],
                          const double q4_adjust[4], const double q5_adjust[4],
                          const double prev_hand_adjust[4],
                          const double ctrl[4], int have_prev, double weight,
                          double hand_adjust[4], double desired_world[4],
                          const double absolute_world[4])
{
    DG_IK_HAND_IN in;
    double ctrl_now[4], delta[4], desired_view[4], world[4], blended[4];
    double prev[3][4], inv[4], base[4], predicted[4];
    static const double identity[4] = { 0.0, 0.0, 0.0, 1.0 };
    int j, k;

    quat_identity(hand_adjust);
    quat_identity(desired_world);
    if (!g_b.hand_have_rest) return 0;
    for (k = 0; k < 4; k++) {
        ctrl_now[k] = ctrl[k];
        if (!_finite(ctrl_now[k])) return 0;
    }
    if (!dg_ik_quat_normalize(ctrl_now)) return 0;

    /* Where the controller has turned to since calibration, applied to the
       hand pose that was animated at that same moment. */
    if (absolute_world) {
        for (k = 0; k < 4; k++) {
            if (!_finite(absolute_world[k])) return 0;
            desired_world[k] = absolute_world[k];
        }
    } else {
    dg_ik_quat_conj(g_b.hand_ctrl_rest, inv);
    dg_ik_quat_mul(ctrl_now, inv, delta);
    if (InterlockedCompareExchange(&g_b.arm_hand_basis, 0, 0) == 1 &&
        g_b.arm_root_have_q0) {
        /* vr_arm_hand_basis=world (run 9 analysis, 2026-09-02). The
           controller's turn since calibration is a ROOM rotation: express
           it in world through the calibration root's YAW only (room up is
           world up; the root's own 10-degree cant would tilt the turn's
           axis) and turn the calibration-time hand by it from the LEFT.
           The body's own rotation since calibration reaches the hand
           through its swing alone - a crouch carries the hand, a yaw does
           not, which is the F10 contract. The `root` basis below
           conjugated the whole hand by the live root instead, which under
           organic comp put the body's yaw on the hand's RIGHT side as
           well: a rotation about the hand's own horizontal axis that read
           as a cant growing 0.85 degrees per degree of body turn. */
        double f0[4], d_world[4], rest_world[4], r0_inv[4], d[4], t[4];
        double t_inv[4], s[4], sd[4];
        quat_ytwist(g_b.arm_root_q0, f0);
        quat_rebase(f0, delta, d_world);
        quat_rebase(g_b.arm_root_q0, g_b.hand_rest_view, rest_world);
        dg_ik_quat_conj(g_b.arm_root_q0, r0_inv);
        dg_ik_quat_mul(live_root_q, r0_inv, d);
        quat_ytwist(d, t);
        dg_ik_quat_conj(t, t_inv);
        dg_ik_quat_mul(d, t_inv, s);
        dg_ik_quat_mul(s, d_world, sd);
        dg_ik_quat_mul(sd, rest_world, desired_world);
    } else {
        dg_ik_quat_mul(delta, g_b.hand_rest_view, desired_view);
        quat_rebase(root_q, desired_view, desired_world);
    }
    }
    if (!dg_ik_quat_normalize(desired_world)) return 0;

    memset(&in, 0, sizeof in);
    for (k = 0; k < 4; k++) in.live[k] = live_hand[k];
    for (j = 0; j < 2; j++) {
        double stored[4];
        if (!have_prev) {
            for (k = 0; k < 4; k++) prev[j][k] = identity[k];
            continue;
        }
        for (k = 0; k < 4; k++)
            stored[k] = (double)g_b.arm_map_cached_adjust[j * 4 + k];
        if (!adjust_quat_to_world(&g_b.pair_frame, stored, prev[j])) return 0;
    }
    if (!adjust_quat_to_world(&g_b.pair_frame, prev_hand_adjust, prev[2]))
        return 0;
    for (k = 0; k < 4; k++) {
        in.prev_upper[k] = prev[0][k];
        in.prev_fore[k] = prev[1][k];
        in.prev_hand[k] = prev[2][k];
        in.desired[k] = desired_world[k];
    }
    if (!adjust_quat_to_world(&g_b.pair_frame, q4_adjust, in.new_upper) ||
        !adjust_quat_to_world(&g_b.pair_frame, q5_adjust, in.new_fore))
        return 0;
    if (!dg_ik_hand_adjust(&in, world)) return 0;

    /* Blend in world space, where the rotation means something, and convert
       once at the end - the same order the shoulder and elbow take. */
    dg_pose_quat_blend(identity, world, weight, blended);
    /* The requested controller pose is only the achieved pose at weight 1.
       Report the pose the blended adjustment will actually produce, so the
       next-pair residual remains meaningful during fade-in/out too. */
    dg_ik_quat_conj(world, inv);
    dg_ik_quat_mul(inv, desired_world, base);
    dg_ik_quat_mul(blended, base, predicted);
    if (!dg_ik_quat_normalize(predicted)) return 0;
    for (k = 0; k < 4; k++) desired_world[k] = predicted[k];
    /* Per-site fail policy (V5.1 par. 4.2): no frame, no hand command for
       this pair - the arm solve carries on. */
    if (!world_quat_to_adjust(&g_b.pair_frame, blended, hand_adjust)) {
        quat_identity(hand_adjust);
        return 0;
    }
    for (k = 0; k < 4; k++)
        if (!_finite(hand_adjust[k])) { quat_identity(hand_adjust); return 0; }
    return 1;
}

/* Move twist from joint 6 into joint 5 without changing the solved wrist
   point. q5 is pre-multiplied by a rotation about the final world forearm
   direction, so that direction is invariant. If either conversion fails the
   caller keeps the position solve and refuses only the hand command. */
/* The amplitude the tick seam left, turned into this frame's angle and
   distance. Reading it here rather than stepping it here is the whole
   discipline: the camera seam runs about five times per game tick, and an
   envelope stepped five times a tick is an envelope five times too fast. */
static double recoil_climb_rad(void)
{
    LONG mdeg = InterlockedCompareExchange(&g_b.recoil_climb_mdeg, 0, 0);
    LONG amp = InterlockedCompareExchange(&g_b.s_recoil_amp, 0, 0);
    if (mdeg <= 0 || amp <= 0) return 0.0;
    return ((double)mdeg / 1000.0) * ((double)amp / 1000.0) *
           (3.14159265358979323846 / 180.0);
}

static double recoil_push_mm(void)
{
    LONG um = InterlockedCompareExchange(&g_b.recoil_push_um, 0, 0);
    LONG amp = InterlockedCompareExchange(&g_b.s_recoil_amp, 0, 0);
    if (um <= 0 || amp <= 0) return 0.0;
    return ((double)um / 1000.0) * ((double)amp / 1000.0);
}

/* One envelope cap, in radians. A zero slot is the built-in default, so a
   configure that never ran cannot pin the hand at zero. */
static double hand_cap_rad(volatile LONG *slot, double def)
{
    LONG v = InterlockedCompareExchange(slot, 0, 0);
    if (v <= 0) return def;
    return (double)v / 1000.0 * 3.14159265358979323846 / 180.0;
}

/* One vote per hand pair on how the turn byte reads in our root-quat yaw:
   if the ticks since the last pair wrote a turn byte, the sign of the body
   drift's movement times the byte's side is one vote. The accumulator
   clamps at +/-30 and the body-follow only speaks from +/-15, so the
   player's own stick turns teach the sign within seconds and a mistaken
   vote (a damage spin under our write) is outvoted, never fatal. */
static void turn_dir_vote(double drift_deg)
{
    LONG wd = InterlockedCompareExchange(&g_b.turn_last_write_dir, 0, 0);
    LONG wt = InterlockedCompareExchange(&g_b.turn_last_write_tick, 0, 0);
    LONG nowt = InterlockedCompareExchange(&g_b.c_ticks, 0, 0);
    if (g_b.turn_vote_have_prev && wd != 0 &&
        (LONG)((DWORD)nowt - (DWORD)wt) < 8) {
        double d = drift_deg - g_b.turn_vote_prev_drift;
        if (d > 180.0) d -= 360.0;
        if (d < -180.0) d += 360.0;
        /* Under 0.2 deg is measurement noise; over 20 in one pair is a
           recalibration snap, not a turn response - neither votes. */
        if (fabs(d) > 0.2 && fabs(d) < 20.0) {
            LONG acc = InterlockedCompareExchange(&g_b.turn_dir_acc, 0, 0);
            acc += (d > 0.0 ? 1 : -1) * (int)wd;
            if (acc > 30) acc = 30;
            if (acc < -30) acc = -30;
            InterlockedExchange(&g_b.turn_dir_acc, acc);
        }
    }
    g_b.turn_vote_prev_drift = drift_deg;
    g_b.turn_vote_have_prev = 1;
}

static int arm_hand_stabilize(const double clean[5][3], double q4[4],
                              double q5[4], double hand[4],
                              double achieved[4], double climb_rad,
                              DG_IK_HAND_STABILIZE_OUT *report)
{
    DG_IK_HAND_STABILIZE_IN in;
    DG_IK_HAND_STABILIZE_OUT out;
    double w4[4], w5[4], raw_hand[4], new_w5[4];
    double rest_fore[3], inherited[3], fore_axis[3];
    double inv[4], base[4], chain[4], bounded[4];
    double new_q5[4], new_hand[4], n;
    int k;

    if (!clean || !q4 || !q5 || !hand || !achieved) return 0;
    if (!adjust_quat_to_world(&g_b.pair_frame, q4, w4) ||
        !adjust_quat_to_world(&g_b.pair_frame, q5, w5) ||
        !adjust_quat_to_world(&g_b.pair_frame, hand, raw_hand)) return 0;
    for (k = 0; k < 3; k++) rest_fore[k] = clean[4][k] - clean[3][k];
    arm_quat_rotate(w4, rest_fore, inherited);
    arm_quat_rotate(w5, inherited, fore_axis);
    n = sqrt(fore_axis[0] * fore_axis[0] +
             fore_axis[1] * fore_axis[1] +
             fore_axis[2] * fore_axis[2]);
    if (!_finite(n) || !(n > 1e-9)) return 0;

    /* The aim gap used to be published from here, off the hand demand's
       yaw - which meant vr_arm_hand=off starved the body-follow entirely
       (a whole live session of "follow writes 0" while the player had to
       recalibrate after every physical turn). It moved to the position
       path in arm_ik_now, which runs on every accepted pair in every hand
       mode; a second publisher here would fight it with a different
       convention on the same slot. */

    memset(&in, 0, sizeof in);
    for (k = 0; k < 4; k++) in.raw_world[k] = raw_hand[k];
    for (k = 0; k < 3; k++) in.fore_axis_world[k] = fore_axis[k] / n;
    for (k = 0; k < 3; k++)                                   /* DGREC4 */
        g_b.rec_pair.fore_axis[k] = (float)in.fore_axis_world[k];

    /* The recoil climb goes in HERE - on the raw hand, before the envelope -
       and the position is the argument for it. The climb is by construction a
       swing about an axis perpendicular to the forearm, which is exactly the
       quantity dg_ik_hand_stabilize already bounds, so a kick cannot put the
       wrist anywhere a wrist does not go: the same limit that holds the
       player's own motion holds ours, with no second mechanism to keep in
       step. `base` below is deliberately still taken from the UNclimbed raw
       hand, which puts the climb into `achieved` as well - so the residual
       telemetry goes on measuring "did the write land" instead of quietly
       reporting our own kick as an error. */
    if (climb_rad > 0.0) {
        double up[3], climb[4], climbed[4];
        /* +Y is up, the same reading solve_arm_adjust takes for its pole and
           the measured torso chain confirms. */
        up[0] = 0.0; up[1] = 1.0; up[2] = 0.0;
        if (dg_recoil_climb_quat(in.fore_axis_world, up, climb_rad, climb)) {
            dg_ik_quat_mul(climb, raw_hand, climbed);
            if (dg_ik_quat_normalize(climbed)) {
                for (k = 0; k < 4; k++) in.raw_world[k] = climbed[k];
                InterlockedIncrement(&g_b.c_recoil_climb_writes);
            } else {
                InterlockedIncrement(&g_b.c_recoil_no_axis);
            }
        } else {
            /* An arm held straight up or straight down has no horizontal axis
               to climb about. No kick that frame, said out loud. */
            InterlockedIncrement(&g_b.c_recoil_no_axis);
        }
    }
    in.max_fore_twist_rad = hand_cap_rad(&g_b.hand_fore_twist_mdeg,
                                         DG_HAND_FORE_TWIST_RAD);
    in.max_wrist_swing_rad = hand_cap_rad(&g_b.hand_wrist_swing_mdeg,
                                          DG_HAND_WRIST_SWING_RAD);
    in.max_wrist_twist_rad = hand_cap_rad(&g_b.hand_wrist_twist_mdeg,
                                          DG_HAND_WRIST_TWIST_RAD);
    if (g_b.hand_have_prev_twist) {
        in.have_prev_twist = 1;
        in.prev_twist_rad = g_b.hand_prev_twist_rad;
        in.have_prev_twist_raw = 1;
        in.prev_twist_raw_rad = g_b.hand_prev_twist_raw_rad;
    }
    if (g_b.hand_have_prev_swing) {
        in.have_prev_swing = 1;
        in.prev_swing_rad = g_b.hand_prev_swing_rad;
        for (k = 0; k < 3; k++)
            in.prev_swing_axis[k] = g_b.hand_prev_swing_axis[k];
        in.have_prev_swing_raw = 1;
        in.prev_swing_raw_rad = g_b.hand_prev_swing_raw_rad;
    }
    if (!dg_ik_hand_stabilize(&in, &out)) return 0;
    /* The APPLIED twist, not the raw one: a raw reference winds up whole
       turns during a bad episode and then pins the clamps forever (the
       2026-08-20 latch). The applied sum is bounded by the caps, so the
       branch memory can never run away. */
    g_b.hand_prev_twist_rad = out.fore_twist_rad + out.wrist_twist_rad;
    g_b.hand_prev_twist_raw_rad = out.raw_twist_rad;
    g_b.hand_have_prev_twist = 1;
    /* Same applied-not-raw discipline for the swing: the reference angle is
       the clamped one, bounded by the cap, so this branch memory cannot run
       away either. The axis only updates when the swing had one - identity
       swings keep the convention they inherited. The RAW angles ride along
       as the fold-hysteresis references - bounded under a full turn by
       unwrap_branch itself, so they cannot wind up the way the 2026-08-20
       raw APPLIED reference did. */
    if (out.wrist_swing_axis[0] * out.wrist_swing_axis[0] +
        out.wrist_swing_axis[1] * out.wrist_swing_axis[1] +
        out.wrist_swing_axis[2] * out.wrist_swing_axis[2] > 0.25) {
        g_b.hand_prev_swing_rad = out.wrist_swing_rad;
        g_b.hand_prev_swing_raw_rad = out.raw_swing_rad;
        for (k = 0; k < 3; k++)
            g_b.hand_prev_swing_axis[k] = out.wrist_swing_axis[k];
        g_b.hand_have_prev_swing = 1;
    }

    dg_ik_quat_mul(out.fore_twist_world, w5, new_w5);
    if (!dg_ik_quat_normalize(new_w5)) return 0;
    if (!world_quat_to_adjust(&g_b.pair_frame, new_w5, new_q5) ||
        !world_quat_to_adjust(&g_b.pair_frame, out.wrist_world, new_hand))
        return 0;

    /* achieved = raw_hand * base. Replace raw_hand by wrist * fore while
       preserving that same animation/upper/old-fore base for telemetry. */
    dg_ik_quat_conj(raw_hand, inv);
    dg_ik_quat_mul(inv, achieved, base);
    dg_ik_quat_mul(out.fore_twist_world, base, chain);
    dg_ik_quat_mul(out.wrist_world, chain, bounded);
    if (!dg_ik_quat_normalize(bounded)) return 0;

    for (k = 0; k < 4; k++) {
        q5[k] = new_q5[k];
        hand[k] = new_hand[k];
        achieved[k] = bounded[k];
    }
    if (report) *report = out;
    return 1;
}

/* The hierarchy matrices at the camera seam already contain the adjustments
   written on the previous pair. Recover the animation's unmodified bone
   directions before solving again, otherwise a fixed controller feeds our
   own rotation back into the next solve and the arm keeps rolling. The live
   forearm inherited q4 and then q5, so their inverses are applied in reverse. */
static int arm_remove_cached_adjust(const DG_ADJ_FRAME *frame,
                                    const double live[5][3],
                                    const float cached[8],
                                    double clean[5][3])
{
    double a4[4], a5[4], q4[4], q5[4];
    double upper[3], fore[3], native_upper[3], after_q5[3], native_fore[3];
    int i, k;

    memcpy(clean, live, 5 * 3 * sizeof(double));
    for (k = 0; k < 4; k++) {
        a4[k] = (double)cached[k];
        a5[k] = (double)cached[4 + k];
        if (!_finite(a4[k]) || !_finite(a5[k])) return 0;
    }
    /* The strip conjugates with THIS pair's heading, the one the matrices
       being stripped were composed under. */
    if (!adjust_quat_to_world(frame, a4, q4) ||
        !adjust_quat_to_world(frame, a5, q5)) return 0;
    for (i = 0; i < 3; i++) {
        upper[i] = live[3][i] - live[2][i];
        fore[i] = live[4][i] - live[3][i];
    }
    arm_quat_unrotate(q4, upper, native_upper);
    arm_quat_unrotate(q5, fore, after_q5);
    arm_quat_unrotate(q4, after_q5, native_fore);
    for (i = 0; i < 3; i++) {
        if (!_finite(native_upper[i]) || !_finite(native_fore[i])) return 0;
        clean[3][i] = clean[2][i] + native_upper[i];
        clean[4][i] = clean[3][i] + native_fore[i];
    }
    return vdist3(clean[2], clean[3]) > 0.0 &&
           vdist3(clean[3], clean[4]) > 0.0;
}

/* ---- The grip roll (vr_arm_uproll). ------------------------------------
   The two-bone solve spends the forearm's twist DOF on nobody: the pose
   anchor serves the ELBOW (roll_to_align on the arm-plane normal) and the
   fore residual is deliberately swing-only, so the twist that reads as
   pistol cant is an emergent leftover. This owner pins it to world up,
   composed AFTER the weight blend as a pure twist about the forearm
   direction the screen will actually show next pass. That direction is the
   blended chain applied to the ANIMATION's forearm - recovered from the
   raw hierarchy with the adjust quaternions READ BACK from the slots, not
   trusted from the cache: the strip's cache is documented to go dishonest
   when slots are lost or altered, and a twist about a mispredicted axis
   would move the wrist. A twist about the displayed bone cannot,
   rest-freeze included. Fail-closed: unreadable or implausible slots skip
   the pair and count it, so an unhealthy read-back path looks like a
   climbing counter instead of like "the fix works".

   Fades, both stateless: w_up takes the correction out over a sine band
   around vertical (perp-of-up degenerates there), the pole fallback's own
   pattern; w_align takes it out as |th| approaches pi, which removes the
   principal-branch jump at +/-pi WITHOUT history - at the branch the
   correction is zero from both sides. Cost, stated: a nearly-180-degree
   carried error is only partially corrected inside the last 30 degrees. */

#define DG_UPROLL_SIN_LO   0.03
#define DG_UPROLL_SIN_HI   0.15
#define DG_UPROLL_PI_BAND  (3.14159265358979323846 / 6.0)

typedef struct {
    int    on;             /* marker vr_arm_uproll */
    int    have_readback;  /* the slot quats below are trustworthy */
    double raw_fore[3];    /* joint 5 -> 6 straight from the hierarchy */
    double slot_q4[4];     /* adjust-space quats read back from the slots */
    double slot_q5[4];
} DG_GRIP_ROLL_IN;

static double uproll_smoothstep01(double x)
{
    if (!(x > 0.0)) return 0.0;
    if (x >= 1.0) return 1.0;
    return x * x * (3.0 - 2.0 * x);
}

static void arm_grip_roll(const DG_ADJ_FRAME *frame, const DG_GRIP_ROLL_IN *gr,
                          double q4[4], double q5[4])
{
    static const double pi = 3.14159265358979323846;
    /* World +Y is up: the same world constant the pole reads as -Y down and
       the recoil climb reads as +Y up. A constant here, not an input, so
       every desk leg exercises the one axis the live path will use. */
    static const double up_w[3] = { 0.0, 1.0, 0.0 };
    double s4w[4], s5w[4], b4w[4], b5w[4], chain_w[4];
    double after5[3], anim_fore[3], axis[3];
    double fn[3], an[3], upn[3], perp_ref[3], carried[3], wanted[3];
    double cx[3], nf, na, nc, nw, dotc, sin_axis, sin_fore, sin_eff;
    double w_up, w_align, th, alpha, half, T[4], new5w[4];
    int k;

    if (!gr || !gr->on) return;
    if (!gr->have_readback) goto skipped;
    /* Per-site fail policy: no frame, no roll - q4/q5 untouched, counted. */
    if (!adjust_quat_to_world(frame, gr->slot_q4, s4w) ||
        !adjust_quat_to_world(frame, gr->slot_q5, s5w)) goto skipped;
    /* The live forearm inherited q4 and then q5; peel them in reverse -
       the strip's exact order, but from slot truth. */
    arm_quat_unrotate(s5w, gr->raw_fore, after5);
    arm_quat_unrotate(s4w, after5, anim_fore);
    if (!adjust_quat_to_world(frame, q4, b4w) ||
        !adjust_quat_to_world(frame, q5, b5w)) goto skipped;
    dg_ik_quat_mul(b5w, b4w, chain_w);
    arm_quat_rotate(chain_w, anim_fore, axis);

    nf = sqrt(anim_fore[0] * anim_fore[0] + anim_fore[1] * anim_fore[1] +
              anim_fore[2] * anim_fore[2]);
    na = sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    if (!_finite(nf) || !_finite(na) || !(nf > 1e-9) || !(na > 1e-9))
        goto skipped;
    for (k = 0; k < 3; k++) {
        fn[k] = anim_fore[k] / nf;
        an[k] = axis[k] / na;
        upn[k] = up_w[k];
    }

    /* Vertical fade: sine of the smaller of the two bone-vs-up angles -
       both the reference projection (on the animation fore) and the wanted
       projection (on the displayed fore) must be healthy. Continuous, and
       the fade formula the design's oracle table is computed from. */
    cx[0] = an[1] * upn[2] - an[2] * upn[1];
    cx[1] = an[2] * upn[0] - an[0] * upn[2];
    cx[2] = an[0] * upn[1] - an[1] * upn[0];
    sin_axis = sqrt(cx[0] * cx[0] + cx[1] * cx[1] + cx[2] * cx[2]);
    cx[0] = fn[1] * upn[2] - fn[2] * upn[1];
    cx[1] = fn[2] * upn[0] - fn[0] * upn[2];
    cx[2] = fn[0] * upn[1] - fn[1] * upn[0];
    sin_fore = sqrt(cx[0] * cx[0] + cx[1] * cx[1] + cx[2] * cx[2]);
    sin_eff = (sin_axis < sin_fore) ? sin_axis : sin_fore;
    w_up = uproll_smoothstep01((sin_eff - DG_UPROLL_SIN_LO) /
                               (DG_UPROLL_SIN_HI - DG_UPROLL_SIN_LO));
    if (!(w_up > 0.0)) return;

    for (k = 0; k < 3; k++)
        perp_ref[k] = upn[k] - (upn[0] * fn[0] + upn[1] * fn[1] +
                                upn[2] * fn[2]) * fn[k];
    arm_quat_rotate(chain_w, perp_ref, carried);
    for (k = 0; k < 3; k++)
        wanted[k] = upn[k] - (upn[0] * an[0] + upn[1] * an[1] +
                              upn[2] * an[2]) * an[k];
    nc = sqrt(carried[0] * carried[0] + carried[1] * carried[1] +
              carried[2] * carried[2]);
    nw = sqrt(wanted[0] * wanted[0] + wanted[1] * wanted[1] +
              wanted[2] * wanted[2]);
    if (!(nc > 1e-9) || !(nw > 1e-9)) return;
    for (k = 0; k < 3; k++) { carried[k] /= nc; wanted[k] /= nw; }

    cx[0] = carried[1] * wanted[2] - carried[2] * wanted[1];
    cx[1] = carried[2] * wanted[0] - carried[0] * wanted[2];
    cx[2] = carried[0] * wanted[1] - carried[1] * wanted[0];
    dotc = carried[0] * wanted[0] + carried[1] * wanted[1] +
           carried[2] * wanted[2];
    th = atan2(cx[0] * an[0] + cx[1] * an[1] + cx[2] * an[2], dotc);
    w_align = uproll_smoothstep01((pi - fabs(th)) / DG_UPROLL_PI_BAND);
    alpha = th * w_up * w_align;
    if (!_finite(alpha) || fabs(alpha) <= 1e-6) return;

    half = 0.5 * alpha;
    T[0] = an[0] * sin(half);
    T[1] = an[1] * sin(half);
    T[2] = an[2] * sin(half);
    T[3] = cos(half);
    dg_ik_quat_mul(T, b5w, new5w);
    {
        double q5_new[4];
        if (!world_quat_to_adjust(frame, new5w, q5_new)) goto skipped;
        for (k = 0; k < 4; k++) q5[k] = q5_new[k];
    }
    InterlockedIncrement(&g_b.c_orient_uprolled);
    return;

skipped:
    InterlockedIncrement(&g_b.c_uproll_skipped);
}

/* Pure glue between the measured skeleton and the already-tested solver. The
   five points are joints 2..6. The pole is down-and-out from the chest branch:
   2->3 supplies the character's rightward direction, projected onto the MGS2
   XZ ground plane, and -Y is down (confirmed by the measured torso chain).
   Neither a character-specific bone length nor a world-facing direction is
   hard-coded. */
static int solve_arm_adjust(const DG_ADJ_FRAME *frame,
                            const double joint[5][3], const double target[3],
                            double weight, const double (*rest_ovr)[3],
                            const DG_GRIP_ROLL_IN *gr,
                            double q4[4], double q5[4], unsigned *clamped)
{
    DG_IK_IN in;
    DG_IK_OUT solved;
    DG_IK_ORIENT_IN oi;
    DG_IK_ORIENT_OUT oo;
    double out[3], n, span;
    double a4[4], a5[4];
    static const double identity[4] = { 0.0, 0.0, 0.0, 1.0 };
    int i;

    if (!_finite(weight) || weight <= 0.0 || weight > 1.0) return 0;
    memset(&in, 0, sizeof in);
    for (i = 0; i < 3; i++) {
        in.shoulder[i] = joint[2][i];                 /* joint 4 */
        in.target[i] = target[i];
    }
    in.upper = vdist3(joint[2], joint[3]);            /* 4 -> 5 */
    in.fore  = vdist3(joint[3], joint[4]);            /* 5 -> 6 */
    in.max_reach_frac = 0.99;
    in.min_elbow_deg = 15.0;
    in.max_elbow_deg = 175.0;

    out[0] = joint[1][0] - joint[0][0];               /* 2 -> 3 */
    out[1] = 0.0;
    out[2] = joint[1][2] - joint[0][2];
    n = sqrt(out[0] * out[0] + out[2] * out[2]);
    if (!(n > 1e-9)) {
        out[0] = joint[2][0] - joint[0][0];           /* 2 -> 4 fallback */
        out[2] = joint[2][2] - joint[0][2];
        n = sqrt(out[0] * out[0] + out[2] * out[2]);
    }
    if (!(n > 1e-9)) { out[0] = 1.0; out[2] = 0.0; n = 1.0; }
    out[0] /= n; out[2] /= n;
    span = in.upper + in.fore;
    in.pole[0] = in.shoulder[0] + span * out[0];
    in.pole[1] = in.shoulder[1] - span;
    in.pole[2] = in.shoulder[2] + span * out[2];

    if (!dg_ik_solve(&in, &solved)) return 0;
    memset(&oi, 0, sizeof oi);
    for (i = 0; i < 3; i++) {
        oi.shoulder[i] = in.shoulder[i];
        oi.elbow[i] = solved.elbow[i];
        oi.wrist[i] = solved.wrist[i];
        g_b.rec_pair.ik_elbow[i] = (float)solved.elbow[i];   /* DGREC4 */
        g_b.rec_pair.ik_wrist[i] = (float)solved.wrist[i];
        /* The orientation reference. rest_ovr is the REST-freeze override:
           a reference captured once on a provably clean pair instead of
           recovered from the live matrices this pair - geometry above stays
           live either way, because positions and bone lengths must track
           the animation and are not what winds. NULL is every ordinary
           call. */
        if (rest_ovr) {
            oi.rest_upper[i] = rest_ovr[0][i];
            oi.rest_fore[i] = rest_ovr[1][i];
        } else {
            oi.rest_upper[i] = joint[3][i] - joint[2][i];
            oi.rest_fore[i] = joint[4][i] - joint[3][i];
        }
    }
    if (g_b.arm_orient_have_prev) {
        oi.have_prev = 1;
        for (i = 0; i < 4; i++) {
            oi.prev_upper_quat[i] = g_b.arm_orient_prev_upper[i];
            oi.prev_fore_quat[i] = g_b.arm_orient_prev_fore[i];
        }
    }
    if (!dg_ik_orient(&oi, &oo)) return 0;
    /* Which branch the roll actually took, live. The desk proof of the
       pose anchor (closed loops return to the same roll, 2026-08-21) says
       nothing about how often the LIVE rig can offer the anchor its
       reference: the rest bones here are read from the animated skeleton,
       and a pose whose arm plane collapses falls back to continuity, which
       is the mechanism that accumulates. A fallback count that is not
       ~zero means the fix is not reaching the frames that matter, and
       without this counter that would look exactly like "the fix did not
       work" - the confusion that cost a whole day. */
    if (oo.flags & DG_IK_ORIENT_ANCHORED)
        InterlockedIncrement(&g_b.c_orient_anchored);
    else
        InterlockedIncrement(&g_b.c_orient_fallback);
    /* The unblended solution is the circle point; the blend below is a
       fade toward it and must not feed back into the continuity. */
    for (i = 0; i < 4; i++) {
        g_b.arm_orient_prev_upper[i] = oo.upper_quat[i];
        g_b.arm_orient_prev_fore[i] = oo.fore_quat[i];
    }
    g_b.arm_orient_have_prev = 1;

    /* The commit: world solution into adjust space under this pair's frame.
       Per-site fail policy: no frame, no pair (the caller refuses). */
    if (!world_quat_to_adjust(frame, oo.upper_quat, a4) ||
        !world_quat_to_adjust(frame, oo.fore_quat, a5)) return 0;
    dg_pose_quat_blend(identity, a4, weight, q4);
    dg_pose_quat_blend(identity, a5, weight, q5);
    arm_grip_roll(frame, gr, q4, q5);
    if (clamped) *clamped = solved.clamped;
    return 1;
}

static void arm_ik_release(void)
{
    ULONGLONG arm, mctrl, adjust;
    ULONGLONG release_mask;
    LONG joints;
    int j;

    if (!InterlockedExchange(&g_b.ik_active, 0)) return;
    g_b.arm_map_cache_valid = 0;
    if (!g_b.a.gm_player_arm_body) goto mismatch;
    arm = *(volatile ULONGLONG *)(ULONG_PTR)g_b.a.gm_player_arm_body;
    if (!plausible_ptr(arm) || arm != g_b.ik_owned_arm) goto mismatch;
    mctrl = *(volatile ULONGLONG *)(ULONG_PTR)(arm + 0x08);
    if (!plausible_ptr(mctrl) || mctrl != g_b.ik_owned_mctrl) goto mismatch;
    joints = RD32(mctrl + 0x14);
    if (joints < 21 || joints > 255) goto mismatch;
    adjust = *(volatile ULONGLONG *)(ULONG_PTR)(mctrl + 0x48);
    if (!plausible_ptr(adjust) || adjust != g_b.ik_owned_adjust) goto mismatch;
    release_mask = g_b.ik_owned_mask & ((1ULL << 4) | (1ULL << 5));
    for (j = 4; j <= 5; j++) {
        volatile float *q;
        if (!((release_mask >> j) & 1)) continue;
        q = (volatile float *)(ULONG_PTR)(adjust + j * 16);
        q[0] = q[1] = q[2] = 0.0f; q[3] = 1.0f;
    }
    *(volatile ULONGLONG *)(ULONG_PTR)(mctrl + 0x38) &= ~release_mask;
    g_b.ik_owned_arm = g_b.ik_owned_mctrl = g_b.ik_owned_adjust = 0;
    g_b.ik_owned_mask = 0;
    return;

mismatch:
    /* The object that owned our bits disappeared or changed. Its pointers may
       already be freed, so the safe release is local bookkeeping only; never
       clear joints on the replacement model. */
    InterlockedIncrement(&g_b.c_arm_release_owner_mismatch);
    g_b.ik_owned_arm = g_b.ik_owned_mctrl = g_b.ik_owned_adjust = 0;
    g_b.ik_owned_mask = 0;
}

static void arm_map_forget(void)
{
    int i;
    arm_ik_release();
    dg_arm_map_reset(&g_b.arm_map);
    memset(&g_b.camera_position, 0, sizeof g_b.camera_position);
    memset(&g_b.free_right, 0, sizeof g_b.free_right);
    g_b.arm_map_phase = 0;
    g_b.arm_map_cache_valid = 0;
    g_b.have_pred_wrist = 0;            /* nothing predicted across a pose loss */
    /* Back to identity, not to whatever the last owner left. The next pair
       subtracts this array from the live matrices, so a stale rotation in here
       would be removed from a hierarchy that never had it. */
    for (i = 0; i < 8; i++)
        g_b.arm_map_cached_adjust[i] = ((i % 4) == 3) ? 1.0f : 0.0f;
    hand_command_clear();
    if (InterlockedCompareExchange(&g_b.hand_drive_owned, 0, 0))
        InterlockedExchange(&g_b.hand_zero_pending, 1);
    g_b.hand_have_rest = 0;
    g_b.arm_root_have_q0 = 0;
    g_b.hand_have_desired = 0;
    g_b.hand_have_last_cmd = 0;
    g_b.hand_have_prev_off = 0;
    g_b.arm_orient_have_prev = 0;
    g_b.hand_have_prev_twist = 0;
    g_b.hand_have_prev_swing = 0;
    InterlockedExchange(&g_b.hand_rest_stale, 0);
    g_b.arm_last_hand_tick = 0;
    g_b.hand_have_prev_base = 0;
    g_b.arm_map_last_pair = 0;
    g_b.arm_map_stream = 0;
    g_b.arm_map_arm = 0;
    g_b.arm_map_objs = 0;
    g_b.arm_map_settle_until = 0;
}

static void arm_map_begin(ULONGLONG arm, ULONGLONG objs,
                          const DG_BRIDGE_ARM_TARGET *target)
{
    arm_ik_release();
    /* The channel is cleared HERE as well as in the forget path, because a
       worker that flips track off and on inside one iteration can bring the
       seam straight to begin - and a command left over from the previous
       stream is both a stale wrist about to be written into a new stream and
       a live-looking publication that would supersede the release zero
       below. */
    hand_command_clear();
    if (InterlockedCompareExchange(&g_b.hand_drive_owned, 0, 0))
        InterlockedExchange(&g_b.hand_zero_pending, 1);
    dg_arm_map_reset(&g_b.arm_map);
    memset(&g_b.camera_position, 0, sizeof g_b.camera_position);
    memset(&g_b.free_right, 0, sizeof g_b.free_right);
    /* A new identity or stream means a new calibration is coming; the old
       rest capture belongs to the old skeleton and must not survive into
       it. The calibration pair that follows re-captures. */
    g_b.rest_ref_have = 0;
    g_b.arm_map_phase = 1;
    g_b.arm_root_have_q0 = 0;
    g_b.hand_have_last_cmd = 0;
    g_b.hand_have_prev_off = 0;
    g_b.arm_orient_have_prev = 0;
    g_b.hand_have_prev_twist = 0;
    g_b.hand_have_prev_swing = 0;
    InterlockedExchange(&g_b.hand_rest_stale, 0);
    g_b.arm_last_hand_tick = 0;
    g_b.hand_have_prev_base = 0;
    g_b.arm_map_cache_valid = 0;
    g_b.arm_map_last_pair = target->pair_id;
    g_b.arm_map_stream = target->stream_id;
    g_b.arm_map_arm = arm;
    g_b.arm_map_objs = objs;
    g_b.arm_map_settle_until = g_b.c_ticks + DG_ADJ_SETTLE_TICKS;
    InterlockedIncrement(&g_b.c_arm_pairs_calibration);
}

/* ------------------------------------------------ F5 step 2: the skeleton --- */

/* Float bits through memory rather than a pointer cast, so nothing here depends
   on how the compiler feels about aliasing a volatile. */
static LONG f2l(float f) { LONG v; memcpy(&v, &f, sizeof v); return v; }

#define DG_OBJS_ARRAY   0x110
#define DG_OBJS_NMODELS 0x64        /* short n_models, short chanl */
#define DG_OBJ_PARENT   0xCA        /* short, ahead of every renderer field */

/* What a hierarchy pass leaves behind: an affine transform whose fourth column
   is (0,0,0,1) and whose three basis rows are roughly unit length. Cheap, and
   strong enough that several consecutive joints passing it at one stride is not
   something random memory does. */
static int looks_like_matrix(ULONGLONG m)
{
    const volatile float *f = (const volatile float *)(ULONG_PTR)m;
    int r;

    if (fabs((double)f[3]) > 1.0e-3) return 0;
    if (fabs((double)f[7]) > 1.0e-3) return 0;
    if (fabs((double)f[11]) > 1.0e-3) return 0;
    if (fabs((double)f[15] - 1.0) > 1.0e-3) return 0;
    for (r = 0; r < 3; r++) {
        double x = (double)f[r * 4], y = (double)f[r * 4 + 1];
        double z = (double)f[r * 4 + 2];
        double n = x * x + y * y + z * z;
        if (!(n > 0.0625 && n < 16.0)) return 0;   /* scale within 0.25 .. 4 */
    }
    return 1;
}

/* How far past p the committed, readable region runs. The probe walks offsets
   nobody has confirmed yet, so rather than hope the object is big enough, ask
   the OS once and refuse every read past the answer. No SEH, no guessing. */
static ULONGLONG region_end(ULONGLONG p)
{
    MEMORY_BASIC_INFORMATION mbi;
    DWORD readable = PAGE_READONLY | PAGE_READWRITE | PAGE_WRITECOPY |
                     PAGE_EXECUTE_READ | PAGE_EXECUTE_READWRITE |
                     PAGE_EXECUTE_WRITECOPY;

    memset(&mbi, 0, sizeof mbi);
    if (!VirtualQuery((LPCVOID)(ULONG_PTR)p, &mbi, sizeof mbi)) return 0;
    if (mbi.State != MEM_COMMIT) return 0;
    if (mbi.Protect & PAGE_GUARD) return 0;
    if (!(mbi.Protect & readable)) return 0;
    return (ULONGLONG)(ULONG_PTR)mbi.BaseAddress + (ULONGLONG)mbi.RegionSize;
}

/* Walk from the subjective arm body to the player work, refusing at every step
   that cannot be independently justified. Two consumers need this - the hand
   writer and the trigger - and they need to agree about who owns the arm, so
   there is one copy of the rules and each caller maps the refusal to its own
   counter. The microphone weapons are named because their native
   WeaponCamRotateShift is non-zero and would give that global two owners; for
   the trigger they are refused for the plainer reason that a microphone has no
   trigger. */
enum {
    DG_RESOLVE_OK = 0,
    DG_RESOLVE_NO_PLAYER,
    DG_RESOLVE_OWNER_MISMATCH,
    DG_RESOLVE_BAD_WEAPON,
    DG_RESOLVE_MIC
};

static int resolve_player_policy(ULONGLONG *arm_out, ULONGLONG *pwork_out,
                          LONG *weapon_out, int motion)
{
    ULONGLONG arm, work, trigger, pwork, end;
    LONG weapon;

    if (!g_b.a.gm_player_arm_body) return DG_RESOLVE_NO_PLAYER;
    arm = *(volatile ULONGLONG *)(ULONG_PTR)g_b.a.gm_player_arm_body;
    if (!plausible_ptr(arm) || arm <= 0x60) return DG_RESOLVE_NO_PLAYER;
    work = arm - 0x60;
    end = region_end(work);
    if (!end || work + 0x230 > end) return DG_RESOLVE_NO_PLAYER;
    trigger = *(volatile ULONGLONG *)(ULONG_PTR)(work + 0x228);
    if (trigger < 0x10000ULL || trigger >= 0x00007FFFFFFFFFFFULL ||
        (trigger & 3) != 0 || trigger <= 0xCF4)
        return DG_RESOLVE_NO_PLAYER;
    pwork = trigger - 0xCF4;
    end = region_end(pwork);
    if (!plausible_ptr(pwork) || !end || pwork + 0xBB8 > end)
        return DG_RESOLVE_NO_PLAYER;
    if (*(volatile ULONGLONG *)(ULONG_PTR)(pwork + 0xBA8) != arm ||
        RD32(pwork + 0xBB0) != 6)
        return DG_RESOLVE_OWNER_MISMATCH;
    weapon = RD32(pwork + 0xB90);
    if (weapon < 0 || weapon >= DG_WEAPON_COUNT) return DG_RESOLVE_BAD_WEAPON;
    if ((weapon == DG_WEAPON_MIC && !motion) || weapon == DG_WEAPON_DEMO_MIC)
        return DG_RESOLVE_MIC;

    if (arm_out) *arm_out = arm;
    if (pwork_out) *pwork_out = pwork;
    if (weapon_out) *weapon_out = weapon;
    return DG_RESOLVE_OK;
}

/* Microphone motion does not grant trigger or legacy camera-shift ownership. */
static int resolve_player(ULONGLONG *a, ULONGLONG *p, LONG *w)
{ return resolve_player_policy(a,p,w,0); }
static int resolve_motion_player(ULONGLONG *a, ULONGLONG *p, LONG *w)
{ return resolve_player_policy(a,p,w,1); }

#include "dg_blade_bridge.inl"
#include "dg_stinger_bridge.inl"
#include "dg_unarmed_prone.inl"
#include "dg_model_arm_bridge.inl"

static void interact_tick(int safe_gameplay)
{
    static unsigned diagnostic_action;
    static int diagnostic_action_safe;
    DG_INTERACT_NATIVE_INPUT in;
    DG_INTERACT_NATIVE_OUTPUT out;
    int weapon=-1;
    LONG pressure=-1;
    ULONGLONG pad=g_b.a.player_pad;
    memset(&in,0,sizeof in);
    in.tick=(uint64_t)(DWORD)g_b.c_ticks;
    if (g_controls_provider && g_controls_active) {
        in.input=g_controls_frame.interact;
        /* The blade owns the right grip: raising a held sword past the ear
         * must not open codec. Suppression retains release-to-rearm semantics. */
        if(blade_claim())in.input.suppressed|=DG_IA_CODEC;
    }
    if (in.input.sample) InterlockedIncrement(&g_interact_stats.samples);
    in.safe=safe_gameplay && g_controls_allowed &&
        GetTickCount64()>=g_controls_lease &&
        GetTickCount64()-g_controls_lease<=100u &&
        InterlockedCompareExchange(&g_b.armed,0,0) &&
        !InterlockedCompareExchange(&g_b.script_menu_only,0,0) &&
        !InterlockedCompareExchange(&g_b.s_late_unsafe,0,0) &&
        pad && RD32(pad-4)!=0;
    if (in.input.ladder) in.safe=g_controls_ladder &&
        GetTickCount64()>=g_controls_lease && GetTickCount64()-g_controls_lease<=100u &&
        !g_b.script_menu_only && pad && RD32(pad-4)!=0 && dg_bridge_controller_ladder_now();
    if (in.input.special) in.safe=g_controls_special==in.input.special &&
        GetTickCount64()>=g_controls_lease && GetTickCount64()-g_controls_lease<=100u &&
        !g_b.script_menu_only && pad && RD32(pad-4)!=0 &&
        dg_bridge_controller_special_now()==in.input.special;
    InterlockedExchange(&g_interact_stats.context,in.safe?in.input.special:-1);
    if (in.safe) {
        in.native_status=(unsigned)RD32(pad+DG_PAD_STATUS_OFFSET);
        in.native_press=(unsigned)RD32(pad+DG_PAD_PRESS_OFFSET);
        in.native_release=(unsigned)RD32(pad+DG_PAD_RELEASE_OFFSET);
        if (in.input.special==DG_CONTROLS_BEYOND || in.input.special==DG_CONTROLS_LOCKER) {
            in.native_peep_left=*(volatile unsigned char *)(ULONG_PTR)(pad+DG_PAD_PRESSURE_OFFSET+10);
            in.native_peep_right=*(volatile unsigned char *)(ULONG_PTR)(pad+DG_PAD_PRESSURE_OFFSET+11);
        }
        if (g_b.a.pad_weapon && g_b.a.pad_press_weapon) {
            in.capture_mask=(unsigned)RD32(g_b.a.pad_weapon);
            pressure=RD32(g_b.a.pad_press_weapon);
        }
        in.unarmed=interact_player_now(&in.player_identity,&weapon) &&
            weapon==0 && pressure>=0 && pressure<12 &&
            g_b.fire.state==DG_FIRE_IDLE;
        InterlockedExchange64(&g_interact_stats.actor,(LONG64)in.player_identity);
        InterlockedExchange(&g_interact_stats.weapon,(LONG)weapon);
        if (!in.player_identity) InterlockedIncrement(&g_interact_stats.actor_missing);
    }
    if (g_interact_adapter.epoch!=in.input.epoch ||
        (in.input.suppressed&DG_IA_CODEC))
        InterlockedExchange64(&g_interact_codec_pending,0);
    if (InterlockedCompareExchange(&g_action_enabled,0,0) && !in.input.special)
        in.input.suppressed|=DG_IA_ACTION;
    ia_step(&g_interact_adapter,&in,&out);
    if (g_b.log && ((in.input.levels&DG_IA_ACTION)!=diagnostic_action ||
        ((in.input.levels&DG_IA_ACTION) && in.safe!=diagnostic_action_safe)))
        g_b.log("  action input: held=%d safe=%d valid=%d suppressed=0x%X blocked=0x%X native=0x%X/0x%X output=0x%X/0x%X context=%d tick=%llu\r\n",
            (in.input.levels&DG_IA_ACTION)!=0,in.safe,in.input.valid,
            in.input.suppressed,g_interact_adapter.blocked,in.native_status,in.native_press,
            out.status,out.press,in.input.special,(unsigned long long)in.tick);
    diagnostic_action=in.input.levels&DG_IA_ACTION;
    diagnostic_action_safe=in.safe;
    if (g_interact_adapter.blocked&DG_IA_CODEC)
        InterlockedExchange64(&g_interact_codec_pending,0);
    if (!in.safe || !in.input.valid || in.input.age_ms>100)
        InterlockedExchange64(&g_interact_codec_pending,0);
    if (!in.safe) { InterlockedIncrement(&g_interact_stats.blocked); return; }
    if (out.codec_press) {
        InterlockedExchange64(&g_interact_codec_pending,(LONG64)GetTickCount64());
        InterlockedIncrement(&g_interact_stats.codec_queued);
    }
    if (out.status || out.press || out.release) InterlockedIncrement(&g_interact_stats.pad_writes);
    if (out.capture_pressure) InterlockedIncrement(&g_interact_stats.capture_writes);
    if (in.input.ladder && out.status) InterlockedIncrement(&g_interact_stats.ladder_writes);
    *(volatile LONG *)(ULONG_PTR)(pad+DG_PAD_STATUS_OFFSET)|=(LONG)out.status;
    *(volatile LONG *)(ULONG_PTR)(pad+DG_PAD_PRESS_OFFSET)|=(LONG)out.press;
    *(volatile LONG *)(ULONG_PTR)(pad+DG_PAD_RELEASE_OFFSET)|=(LONG)out.release;
    if (out.capture_pressure)
        *(volatile unsigned char *)(ULONG_PTR)(pad+DG_PAD_PRESSURE_OFFSET+pressure)=255;
    if (in.input.special==DG_CONTROLS_BEYOND || in.input.special==DG_CONTROLS_LOCKER) {
        if (out.status&1u) *(volatile unsigned char *)(ULONG_PTR)(pad+DG_PAD_PRESSURE_OFFSET+10)=255;
        if (out.status&2u) *(volatile unsigned char *)(ULONG_PTR)(pad+DG_PAD_PRESSURE_OFFSET+11)=255;
    }
}

static void hand_drive_write(void)
{
    short rot[3];
    LONG made_tick, pair;
    LONG now, age;
    volatile short *dst;
    int k;

    /* The release debt comes first and does not care whether commands are
       still requested: a stream that turned vr_arm_hand off entirely still has
       to give the wrist back. A LIVE command supersedes it without a write -
       the camera seam is publishing again, so the channel is spoken for, and
       by construction that cannot happen before the zero has had its tick:
       publishes only come from solved pairs, and solving waits out the settle
       window that starts at the same release that set this flag. Otherwise the
       zero goes through the same gates as the write below - no anchor or no
       player and the debt simply stays owed until they return. */
    if (InterlockedCompareExchange(&g_b.hand_zero_pending, 0, 0)) {
        if (InterlockedCompareExchange(&g_b.hand_command_valid, 0, 0)) {
            InterlockedExchange(&g_b.hand_zero_pending, 0);
        } else if (g_b.a.arm_cam_rotate_shift &&
                   resolve_player(NULL, NULL, NULL) == DG_RESOLVE_OK) {
            volatile short *r =
                (volatile short *)(ULONG_PTR)g_b.a.arm_cam_rotate_shift;
            r[0] = 0; r[1] = 0; r[2] = 0;
            InterlockedExchange(&g_b.hand_zero_pending, 0);
            InterlockedIncrement(&g_b.c_arm_hand_zeroed);
        }
    }
    if (!InterlockedCompareExchange(&g_b.hand_command_requested, 0, 0)) return;
    if (!hand_command_read(rot, &made_tick, &pair)) {
        InterlockedIncrement(&g_b.c_arm_hand_tick_no_command);
        return;
    }
    now = InterlockedCompareExchange(&g_b.c_ticks, 0, 0);
    age = (LONG)((DWORD)now - (DWORD)made_tick);
    if (age < 0 || age > DG_HAND_COMMAND_TICKS) {
        InterlockedIncrement(&g_b.c_arm_hand_tick_stale);
        hand_command_clear();
        return;
    }
    if (!g_b.a.arm_cam_rotate_shift) {
        InterlockedIncrement(&g_b.c_arm_hand_tick_no_player);
        return;
    }
    switch (resolve_player(NULL, NULL, NULL)) {
    case DG_RESOLVE_OK:
        break;
    case DG_RESOLVE_OWNER_MISMATCH:
        InterlockedIncrement(&g_b.c_arm_hand_tick_owner_mismatch);
        return;
    case DG_RESOLVE_BAD_WEAPON:
        InterlockedIncrement(&g_b.c_arm_hand_tick_bad_weapon);
        return;
    case DG_RESOLVE_MIC:
        InterlockedIncrement(&g_b.c_arm_hand_tick_mic);
        return;
    default:
        InterlockedIncrement(&g_b.c_arm_hand_tick_no_player);
        return;
    }

    dst = (volatile short *)(ULONG_PTR)g_b.a.arm_cam_rotate_shift;
    for (k = 0; k < 3; k++) dst[k] = rot[k];
    (void)pair; /* carried by the snapshot so a torn publication is testable */
    InterlockedExchange(&g_b.hand_drive_owned, 1);
    InterlockedIncrement(&g_b.c_arm_hand_tick_writes);
}

/* -------------------------------------------------------------- F7 fire --- */

/* The same bounded seqlock as the hand command, and for the same reason: one
   normal publisher, an invalidator that may run on another thread, and a
   reader that must never see half of one sample and half of the next. */
static int fire_lock(LONG *locked)
{
    int tries;
    for (tries = 0; tries < 8; tries++) {
        LONG s = InterlockedCompareExchange(&g_b.fire_seq, 0, 0);
        if (s & 1) continue;
        if (InterlockedCompareExchange(&g_b.fire_seq, s + 1, s) == s) {
            *locked = s + 1;
            return 1;
        }
    }
    return 0;
}

static void fire_unlock(LONG locked)
{
    MemoryBarrier();
    InterlockedExchange(&g_b.fire_seq, locked + 1);
}

static void fire_publish_legacy(const DG_BRIDGE_FIRE *cmd)
{
    LONG locked;

    if (!InterlockedCompareExchange(&g_b.armed, 0, 0)) return;
    if (!cmd) {
        InterlockedExchange(&g_b.fire_valid, 0);
        return;
    }
    if (!fire_lock(&locked)) return;
    if (cmd->valid) InterlockedExchange(&g_controls_fire_retired,0);
    InterlockedExchange(&g_b.fire_press_lo, (LONG)(DWORD)cmd->press_seq);
    InterlockedExchange(&g_b.fire_press_hi, (LONG)(DWORD)(cmd->press_seq >> 32));
    InterlockedExchange(&g_b.fire_release_lo, (LONG)(DWORD)cmd->release_seq);
    InterlockedExchange(&g_b.fire_release_hi,
                        (LONG)(DWORD)(cmd->release_seq >> 32));
    InterlockedExchange(&g_b.fire_value, (LONG)(cmd->value * 1000.0));
    InterlockedExchange(&g_b.fire_click, (LONG)(cmd->click * 1000.0));
    InterlockedExchange(&g_b.fire_stream, (LONG)cmd->stream_id);
    InterlockedExchange(&g_b.fire_tick,
                        InterlockedCompareExchange(&g_b.c_ticks, 0, 0));
    InterlockedExchange(&g_b.fire_valid, cmd->valid ? 1 : 0);
    fire_unlock(locked);
    InterlockedIncrement(&g_b.c_fire_published);
}

void dg_bridge_fire_now(const DG_BRIDGE_FIRE *cmd) {
    /* VEH publication never blocks behind a control-thread revocation. */
    if (!TryAcquireSRWLockShared(&g_controls_lock)) return;
    if (!g_controls_provider) fire_publish_legacy(cmd);
    ReleaseSRWLockShared(&g_controls_lock);
}
static int fire_read(DG_BRIDGE_FIRE *out, LONG *tick)
{
    int tries;
    if (g_controls_provider) {
        if (!g_controls_active || !g_controls_frame.fire_available) return 0;
        *out=g_controls_frame.fire;
        if (tick) *tick=g_b.c_ticks;
        return 1;
    }
    /* Retirement is absence, not a new legacy stream-zero observation.
     * Feeding an invalid old mailbox would reset active fire before its
     * existing coast/soft-abort contract sees the missing input. */
    if (InterlockedCompareExchange(&g_controls_fire_retired,0,0)) return 0;
    for (tries = 0; tries < 8; tries++) {
        LONG before = InterlockedCompareExchange(&g_b.fire_seq, 0, 0);
        LONG valid, t, plo, phi, rlo, rhi, v, c, s;
        if (before & 1) continue;
        valid = InterlockedCompareExchange(&g_b.fire_valid, 0, 0);
        plo = InterlockedCompareExchange(&g_b.fire_press_lo, 0, 0);
        phi = InterlockedCompareExchange(&g_b.fire_press_hi, 0, 0);
        rlo = InterlockedCompareExchange(&g_b.fire_release_lo, 0, 0);
        rhi = InterlockedCompareExchange(&g_b.fire_release_hi, 0, 0);
        v = InterlockedCompareExchange(&g_b.fire_value, 0, 0);
        c = InterlockedCompareExchange(&g_b.fire_click, 0, 0);
        s = InterlockedCompareExchange(&g_b.fire_stream, 0, 0);
        t = InterlockedCompareExchange(&g_b.fire_tick, 0, 0);
        MemoryBarrier();
        if (before != InterlockedCompareExchange(&g_b.fire_seq, 0, 0)) continue;
        out->press_seq = ((unsigned __int64)(DWORD)phi << 32) | (DWORD)plo;
        out->release_seq = ((unsigned __int64)(DWORD)rhi << 32) | (DWORD)rlo;
        out->value = (double)v / 1000.0;
        out->click = (double)c / 1000.0;
        out->stream_id = (unsigned long)s;
        out->valid = valid;
        if (tick) *tick = t;
        return 1;
    }
    return 0;
}

static void fire_write(const DG_FIRE_OUT *out, LONG mask, LONG pidx)
{
    volatile LONG *press = (volatile LONG *)(ULONG_PTR)
                               (g_b.a.player_pad + DG_PAD_PRESS_OFFSET);
    volatile LONG *status = (volatile LONG *)(ULONG_PTR)
                                (g_b.a.player_pad + DG_PAD_STATUS_OFFSET);
    volatile LONG *release = (volatile LONG *)(ULONG_PTR)
                                 (g_b.a.player_pad + DG_PAD_RELEASE_OFFSET);

    if (!mask) return;

    if (out->press && (*press & mask) != mask) {
        *press |= mask;
        InterlockedIncrement(&g_b.c_fire_wrote_press);
    }
    if (out->status && (*status & mask) != mask) {
        *status |= mask;
        InterlockedIncrement(&g_b.c_fire_wrote_status);
    }
    if (out->release && (*release & mask) != mask) {
        *release |= mask;
        InterlockedIncrement(&g_b.c_fire_wrote_release);
    }

    /* Pressure is a byte, not a bit, so "only add" means a maximum. A zero
       from the state machine therefore writes nothing at all: the release and
       idle ticks leave the byte exactly as the pad copy left it. That is not a
       shortcut - lowering the byte is how a cancel is spelled (SS1: press in
       [1,23] two ticks running), and we must never spell one by accident. */
    if (out->pressure > 0) {
        volatile unsigned char *slot;
        if (pidx < 0 || pidx >= DG_PAD_PRESSURE_COUNT) {
            InterlockedIncrement(&g_b.c_fire_no_index);
            return;
        }
        slot = (volatile unsigned char *)(ULONG_PTR)
                   (g_b.a.player_pad + DG_PAD_PRESSURE_OFFSET + pidx);
        if ((int)*slot < out->pressure) {
            *slot = (unsigned char)out->pressure;
            InterlockedIncrement(&g_b.c_fire_wrote_pressure);
        } else {
            InterlockedIncrement(&g_b.c_fire_pressure_kept);
        }
    }
}

/* ------------------------------------------------------------ F9 walking ---
   The left stick, from the camera seam to the pad. The channel is the fire
   channel's shape with the sequence numbers removed: a stick is a level, and
   re-writing a level is idempotent where re-writing an edge is a second
   press. */

static int move_lock(LONG *locked)
{
    int tries;
    for (tries = 0; tries < 8; tries++) {
        LONG s = InterlockedCompareExchange(&g_b.move_seq, 0, 0);
        if (s & 1) continue;
        if (InterlockedCompareExchange(&g_b.move_seq, s + 1, s) == s) {
            *locked = s + 1;
            return 1;
        }
    }
    return 0;
}

static void move_unlock(LONG locked)
{
    MemoryBarrier();
    InterlockedExchange(&g_b.move_seq, locked + 1);
}

static void move_publish_legacy(const DG_BRIDGE_MOVE *cmd)
{
    LONG locked;

    if (!InterlockedCompareExchange(&g_b.armed, 0, 0)) return;
    if (!cmd) {
        InterlockedExchange(&g_b.move_valid, 0);
        return;
    }
    if (!move_lock(&locked)) return;
    InterlockedExchange(&g_b.move_x, (LONG)(cmd->x * 1000.0));
    InterlockedExchange(&g_b.move_y, (LONG)(cmd->y * 1000.0));
    InterlockedExchange(&g_b.move_turn_x, (LONG)(cmd->turn_x * 1000.0));
    InterlockedExchange(&g_b.move_turn_valid, cmd->turn_valid ? 1 : 0);
    InterlockedExchange(&g_b.move_stream, (LONG)cmd->stream_id);
    InterlockedExchange(&g_b.move_tick_stamp,
                        InterlockedCompareExchange(&g_b.c_ticks, 0, 0));
    InterlockedExchange(&g_b.move_valid, cmd->valid ? 1 : 0);
    move_unlock(locked);
    InterlockedIncrement(&g_b.c_move_published);
}

void dg_bridge_move_now(const DG_BRIDGE_MOVE *cmd) {
    if (!TryAcquireSRWLockShared(&g_controls_lock)) return;
    if (!g_controls_provider) move_publish_legacy(cmd);
    ReleaseSRWLockShared(&g_controls_lock);
}
static int move_read(DG_BRIDGE_MOVE *out, LONG *tick)
{
    int tries;
    if (g_controls_provider) {
        if (!g_controls_active || !g_controls_frame.move_available) return 0;
        *out=g_controls_frame.move;
        if (tick) *tick=g_b.c_ticks;
        return 1;
    }
    for (tries = 0; tries < 8; tries++) {
        LONG before = InterlockedCompareExchange(&g_b.move_seq, 0, 0);
        LONG valid, x, y, s, t;
        if (before & 1) continue;
        LONG tx, tv;
        valid = InterlockedCompareExchange(&g_b.move_valid, 0, 0);
        x = InterlockedCompareExchange(&g_b.move_x, 0, 0);
        y = InterlockedCompareExchange(&g_b.move_y, 0, 0);
        tx = InterlockedCompareExchange(&g_b.move_turn_x, 0, 0);
        tv = InterlockedCompareExchange(&g_b.move_turn_valid, 0, 0);
        s = InterlockedCompareExchange(&g_b.move_stream, 0, 0);
        t = InterlockedCompareExchange(&g_b.move_tick_stamp, 0, 0);
        MemoryBarrier();
        if (before != InterlockedCompareExchange(&g_b.move_seq, 0, 0)) continue;
        /* The whole struct, every field: a channel that fills some of its
           output leaves the rest as stack noise in the consumer, which is a
           coin-flip control input. */
        memset(out, 0, sizeof *out);
        out->x = (double)x / 1000.0;
        out->y = (double)y / 1000.0;
        out->turn_x = (double)tx / 1000.0;
        out->turn_valid = (int)tv;
        out->stream_id = (unsigned long)s;
        out->valid = valid;
        if (tick) *tick = t;
        return 1;
    }
    return 0;
}

/* One tick of walking. All the decision logic lives in dg_move_step, which is
   pure and desk-tested; this function owns only the two things a desk cannot:
   the gate that says the pad may be looked at, and the store instructions
   themselves. The pad is written plainly, not interlocked - this runs on the
   game thread at the seam the game itself writes the pad from, the same
   discipline fire_write already relies on. */
/* The actuator probe: read the six deciders off the PlayerWork into the ring
   the log drains. A full ring or an unresolvable player drops the sample,
   never blocks the seam. `wrote` says whether this tick put a byte on the
   pad (the classic per-write sample) or is a dense aim-tick sample taken
   for the series itself. */
static void turn_probe_sample(unsigned char byte, int wrote)
{
    LONG w = g_b.turn_probe_w;
    DG_TURN_PROBE_SAMPLE *s;
    ULONGLONG pwork = 0, pend = 0;
    if (w - g_b.turn_probe_r >= DG_TURN_PROBE_RING) return;
    s = &g_b.turn_probe_ring[w % DG_TURN_PROBE_RING];
    memset(s, 0, sizeof *s);
    s->tick = InterlockedCompareExchange(&g_b.c_ticks, 0, 0);
    s->byte = byte;
    s->wrote = wrote ? 1 : 0;
    s->fire_state = InterlockedCompareExchange(&g_b.s_fire_state, 0, 0);
    if (resolve_player(NULL, &pwork, NULL) == DG_RESOLVE_OK)
        pend = region_end(pwork);
    if (pwork && pend && pwork + 0xD28 <= pend) {
        s->pw_ok = 1;
        s->flags = *(volatile ULONGLONG *)(ULONG_PTR)(pwork + 0xAC0);
        s->turn_vy = *(volatile short *)(ULONG_PTR)(pwork + 0x8A);
        s->rot_vy = *(volatile short *)(ULONG_PTR)(pwork + 0x82);
        s->cam_vy = *(volatile short *)(ULONG_PTR)(pwork + 0xD22);
        s->cam_pad = *(volatile short *)(ULONG_PTR)(pwork + 0xD26);
        s->action = *(volatile ULONGLONG *)(ULONG_PTR)(pwork + 0xC60);
        s->action2 = *(volatile ULONGLONG *)(ULONG_PTR)(pwork + 0xC78);
    }
    {   /* The follow's loop terms, as published (f2l floats). */
        LONG v; float f;
        v = InterlockedCompareExchange(&g_b.s_arm_aim_gap, 0, 0);
        memcpy(&f, &v, sizeof f); s->gap_deg = f;
        v = InterlockedCompareExchange(&g_b.s_arm_body_drift, 0, 0);
        memcpy(&f, &v, sizeof f); s->drift_deg = f;
        v = InterlockedCompareExchange(&g_b.s_arm_head_yaw, 0, 0);
        memcpy(&f, &v, sizeof f); s->head_deg = f;
        v = InterlockedCompareExchange(&g_b.s_arm_root_tilt, 0, 0);
        memcpy(&f, &v, sizeof f); s->root_tilt_deg = f;
        s->gap_age = s->tick -
            InterlockedCompareExchange(&g_b.arm_aim_gap_tick, 0, 0);
    }
    InterlockedIncrement(&g_b.turn_probe_w);
}

static void move_resolve_workl(const LiveImage *image)
{
    static const unsigned char pat[] = {
        0x48, 0x8B, 0x87, 0x00, 0x0D, 0x00, 0x00,   /* mov rax,[rdi+0xd00]   */
        0x0F, 0xBF, 0x48, 0x10,                     /* movsx ecx,[rax+0x10]  */
        0x48, 0x8B, 0x05, 0, 0, 0, 0,               /* mov rax,[rip+disp32]  */
        0x89, 0x88, 0x10, 0x05, 0x00, 0x00 };       /* mov [rax+0x510],ecx   */
    DWORD i, hits = 0, at = 0;
    g_b.workl_ptr = 0;
    if (!image || !image->bytes || image->size < sizeof pat) return;
    for (i = 0; i + sizeof pat <= image->size; i++) {
        if (image->bytes[i] != 0x48 || image->bytes[i + 1] != 0x8B) continue;
        if (memcmp(image->bytes + i, pat, 14) != 0) continue;
        if (memcmp(image->bytes + i + 18, pat + 18, 6) != 0) continue;
        if (!all_valid(image->valid, i, sizeof pat, image->size)) continue;
        hits++; at = i;
    }
    if (hits == 1) {
        LONG disp; DWORD rva;
        memcpy(&disp, image->bytes + at + 14, 4);
        rva = (DWORD)(at + 18 + disp);
        if (rva < image->size) g_b.workl_ptr = image->base + rva;
    }
    if (g_b.log)
        g_b.log("  move: workL anchor %s (%lu match%s)\r\n",
                g_b.workl_ptr ? "resolved" : "UNAVAILABLE - third person/prone walk refused",
                (unsigned long)hits, hits == 1 ? "" : "es");
}

void dg_bridge_pad_origin_now(LONG dir4096)
{
    InterlockedExchange(&g_b.s_cam_dir, dir4096);
}

static void move_tick(int safe_gameplay)
{
    DG_BRIDGE_MOVE cmd;
    DG_MOVE_IN in;
    DG_MOVE_OUT out;
    LONG now, tick = 0;
    int can_write = 0;
    int follow_inject = 0;

    int third_ok = 0, prone = 0;
    ULONGLONG tstat = g_b.tick_status;
    LONG cam_dir = InterlockedCompareExchange(&g_b.s_cam_dir, 0, 0);
    LONG want_third = InterlockedCompareExchange(&g_b.move_third, 0, 0);

    if (!InterlockedCompareExchange(&g_b.walk_mode, 0, 0) &&
        !InterlockedCompareExchange(&g_b.turn_mode, 0, 0)) return;

    memset(&in, 0, sizeof in);
    if (!move_read(&cmd, &tick)) {
        InterlockedIncrement(&g_b.c_move_no_command);
        return;
    }
    /* An unvouched sample and an empty channel refuse the same way: there
       is no finger this tick. Counted under no-command so every refusal in
       the heartbeat line has exactly one name. */
    if (!cmd.valid) {
        InterlockedIncrement(&g_b.c_move_no_command);
        return;
    }
    now = InterlockedCompareExchange(&g_b.c_ticks, 0, 0);
    in.have_sample = 1;
    in.sample_age = (int)(LONG)((DWORD)now - (DWORD)tick);
    in.input_ok = 1;
    in.x = cmd.x;
    in.y = cmd.y;
    in.deadzone = (double)InterlockedCompareExchange(&g_b.move_deadzone_mils,
                                                     0, 0) / 1000.0;
    /* The walk is stood down entirely when off; the turn likewise. Done by
       blanking the inputs rather than by branching around the writes below,
       so there is exactly one write path to reason about. */
    if (!InterlockedCompareExchange(&g_b.walk_mode, 0, 0)) {
        in.x = 0.0;
        in.y = 0.0;
    }
    in.turn_on = !blade_claim() && InterlockedCompareExchange(&g_b.turn_mode, 0, 0) ? 1 : 0;
    in.turn_x = cmd.turn_valid ? cmd.turn_x : 0.0;
    in.turn_gain = (double)InterlockedCompareExchange(&g_b.turn_gain_mils,
                                                      0, 0) / 1000.0;

    /* Body-follows-aim: when the player's own stick is silent, the learned
       byte sign has committed, and the hand stream published a fresh aim
       gap past the threshold, the body is steered toward the aim through
       the SAME turn path as the stick - one write mechanism, one gate
       chain, one yield rule. Sign composition: the gap is the demand's yaw
       beyond the base, so closing it needs the body drift to move TOWARD
       sign(gap) (base rotates with the body: d gap = -d drift), and dir is
       the learned [byte>128 -> drift+] sign. The injected value rides the
       normal deadzone..1 mapping so the rate ramps linearly from the
       threshold to full stick at `full`. */
    {
        LONG ft = InterlockedCompareExchange(&g_b.turn_follow_thresh_mdeg,
                                             0, 0);
        LONG ffu = InterlockedCompareExchange(&g_b.turn_follow_full_mdeg,
                                              0, 0);
        LONG acc = InterlockedCompareExchange(&g_b.turn_dir_acc, 0, 0);
        double man = in.turn_x < 0.0 ? -in.turn_x : in.turn_x;

        {
            LONG fs = InterlockedCompareExchange(&g_b.s_fire_state, 0, 0);
            int aim_on = InterlockedCompareExchange(&g_b.follow_aim, 0, 0) == 1;
            if (ft > 0 && !aim_on && fs >= DG_FIRE_DRAW && fs <= DG_FIRE_RELEASE) {
                if (InterlockedCompareExchange(&g_b.turn_probe, 0, 0)) {
                    /* The actuator probe NEEDS writes during aim - that
                       is the tick the four hypotheses disagree about.
                       Diagnostic sessions only; the marker doc says so. */
                } else {
                    InterlockedIncrement(&g_b.c_follow_aim_hold);
                    ft = 0;
                }
            }
        }
        if (in.turn_on && ft > 0 && man <= in.deadzone) {
            if (acc >= 15 || acc <= -15) {
                LONG gt = InterlockedCompareExchange(&g_b.arm_aim_gap_tick,
                                                     0, 0);
                now = InterlockedCompareExchange(&g_b.c_ticks, 0, 0);
                if (gt != 0 && (LONG)((DWORD)now - (DWORD)gt) < 30) {
                    LONG gv = InterlockedCompareExchange(&g_b.s_arm_aim_gap,
                                                         0, 0);
                    float gf;
                    double gap, th, fu, mag;
                    memcpy(&gf, &gv, sizeof gf);
                    gap = (double)gf;
                    th = (double)ft / 1000.0;
                    fu = (double)ffu / 1000.0;
                    mag = (fabs(gap) - th) / (fu > th ? fu - th : 1.0);
                    if (_finite(gap) && mag > 0.0) {
                        int dir = (acc > 0) ? 1 : -1;
                        if (mag > 1.0) mag = 1.0;
                        /* Past the deadzone by construction, and divided
                           by the gain so the marker's stick gain does not
                           double onto the follow ramp. */
                        in.turn_x = (gap > 0.0 ? 1.0 : -1.0) *
                                    (double)dir *
                                    (in.deadzone + 0.001 +
                                     mag * (0.999 - in.deadzone));
                        if (in.turn_gain > 1.0) in.turn_x /= in.turn_gain;
                        follow_inject = 1;
                    } else {
                        InterlockedIncrement(&g_b.c_follow_under);
                    }
                } else {
                    InterlockedIncrement(&g_b.c_follow_stale);
                }
            } else {
                InterlockedIncrement(&g_b.c_follow_no_sign);
            }
        }
    }

    /* The same gate as the trigger, and in the same order: state, safety,
       anchors, and the PlayerPad.enable check that says the record we would
       write is the record the player actually reads. */
    if (want_third && g_b.fps.state != DG_FPS_ACTIVE &&
        !(tstat & 0x1ULL /* PLAYER_WATCH */) &&
        g_b.a.pl_subject_move && RD32(g_b.a.pl_subject_move) == 0)
        third_ok = 1;
    prone = (tstat & 0x20ULL /* PLAYER_GROUND */) != 0;
    if ((g_b.fps.state == DG_FPS_ACTIVE || third_ok) && safe_gameplay &&
        !InterlockedCompareExchange(&g_b.s_late_unsafe, 0, 0) &&
        g_b.a.player_pad && RD32(g_b.a.player_pad - 4) != 0) {
        can_write = 1;
        in.status = (unsigned int)RD32(g_b.a.player_pad +
                                       DG_PAD_STATUS_OFFSET);
        in.analog_input = (unsigned int)(unsigned short)
            *(volatile short *)(ULONG_PTR)(g_b.a.player_pad +
                                           DG_PAD_ANALOG_OFFSET);
        in.left_dx = *(volatile unsigned char *)(ULONG_PTR)
                         (g_b.a.player_pad + DG_PAD_LEFT_DX_OFFSET);
        in.left_dy = *(volatile unsigned char *)(ULONG_PTR)
                         (g_b.a.player_pad + DG_PAD_LEFT_DY_OFFSET);
        in.right_dx = *(volatile unsigned char *)(ULONG_PTR)
                          (g_b.a.player_pad + DG_PAD_RIGHT_DX_OFFSET);
        /* The driver's bytes, kept for the heartbeat: this one line is what
           decides whether a phantom turn came from a second input path (an
           XR runtime emulating a gamepad) or from us. */
        InterlockedExchange(&g_b.s_pad_analog_raw,
            ((LONG)in.right_dx << 24) |
            ((LONG)*(volatile unsigned char *)(ULONG_PTR)
                 (g_b.a.player_pad + DG_PAD_RIGHT_DX_OFFSET + 1) << 16) |
            ((LONG)in.left_dx << 8) | (LONG)in.left_dy);
    }
    if (!can_write) {
        InterlockedIncrement(&g_b.c_move_blocked_gate);
        if (want_third && g_b.fps.state != DG_FPS_ACTIVE)
            InterlockedIncrement(&g_b.c_move_not_third);
        return;
    }

    if (!third_ok &&
        (!g_b.a.pl_subject_move || RD32(g_b.a.pl_subject_move) == 0)) {
        InterlockedIncrement(&g_b.c_move_not_subject);
        return;
    }

    if (third_ok) {
        /* Third person owns no turn: the right stick's byte would land in
           whatever the game reads it as there. Walk only. */
        in.turn_on = 0;
        in.turn_x = 0.0;
        follow_inject = 0;
    }
    if (prone && InterlockedCompareExchange(&g_b.move_prone, 0, 0))
        in.max_deflect = (int)InterlockedCompareExchange(&g_b.move_prone_max, 0, 0);
    if (InterlockedCompareExchange(&g_b.move_dir_org, 0, 0)) {
        /* Diagnostic origin: the body's own yaw (rot.vy), so a forward stick
           is always "the way the body faces" and StandStill needs no turn
           before StandRun. Reads the player work the interact layer resolves
           (works in third person, unlike the arm-body walk). */
        uint64_t id = 0; int weapon = -1;
        if (interact_player_now(&id, &weapon) && plausible_ptr(id) &&
            region_end(id) && id + 0x84 <= region_end(id))
            cam_dir = (LONG)(*(volatile short *)(ULONG_PTR)(id + 0x82) & 4095);
        else
            cam_dir = -1;
    }
    if ((third_ok || in.max_deflect) && cam_dir >= 0) {

        LONG sign = InterlockedCompareExchange(&g_b.move_dir_sign, 0, 0);
        LONG off = InterlockedCompareExchange(&g_b.move_dir_offset, 0, 0);
        LONG yaw = sign < 0 ? -cam_dir : cam_dir;
        in.dir_wanted = 1;
        in.dir_org = (int)((yaw + off + 2048) & 4095);
    }

    dg_move_step(&in, &out);
    if (out.flags & DG_MOVE_F_YIELDED)
        InterlockedIncrement(&g_b.c_move_yielded);
    if (out.flags & DG_MOVE_F_IDLE) InterlockedIncrement(&g_b.c_move_idle);
    if (out.flags & DG_MOVE_F_STALE) InterlockedIncrement(&g_b.c_move_stale);
    if (out.flags & DG_MOVE_F_TURN_YIELDED)
        InterlockedIncrement(&g_b.c_turn_yielded);
    if (out.flags & DG_MOVE_F_TURN_IDLE)
        InterlockedIncrement(&g_b.c_turn_idle);

    /* The turn first, because it is independent: a yielded or idle WALK must
       not cost the player their turn, nor the other way round. Same plain
       stores at the same seam, same wholesale per-tick replacement by the
       game's own pad copy, so stopping is the release here too. */
    if (out.turn_write) {
        *(volatile unsigned char *)(ULONG_PTR)
            (g_b.a.player_pad + DG_PAD_RIGHT_DX_OFFSET) = out.rdx;
        *(volatile short *)(ULONG_PTR)
            (g_b.a.player_pad + DG_PAD_ANALOG_OFFSET) |=
                (short)DG_MOVE_ANALOG_R_USE;
        InterlockedIncrement(&g_b.c_turn_writes);
        /* Every written byte - stick and follow alike - feeds the sign
           vote: which side of 128 spoke, and when. */
        InterlockedExchange(&g_b.turn_last_write_dir,
                            out.rdx > 128 ? 1 : -1);
        InterlockedExchange(&g_b.turn_last_write_tick,
            InterlockedCompareExchange(&g_b.c_ticks, 0, 0));
        if (follow_inject)
            InterlockedIncrement(&g_b.c_turn_follow_writes);

        if (out.rdx == 176 || out.rdx == 80)
            InterlockedIncrement(&g_b.c_turn_write_dead);
        /* The actuator probe samples the six deciders at the write itself. */
        if (InterlockedCompareExchange(&g_b.turn_probe, 0, 0))
            turn_probe_sample(out.rdx, 1);
    }

    if (!out.turn_write &&
        InterlockedCompareExchange(&g_b.turn_probe, 0, 0) >= 2) {
        LONG fs = InterlockedCompareExchange(&g_b.s_fire_state, 0, 0);

        if (InterlockedCompareExchange(&g_b.turn_probe, 0, 0) == 3 ||
            (fs >= DG_FIRE_DRAW && fs <= DG_FIRE_RELEASE))
            turn_probe_sample(128, 0);
    }
    if (!out.write) return;

    /* The write. Bytes are stores, the two flag fields are ORs - the game
       already computed press/release from status at the top of the tick, so
       an ORed direction bit cannot manufacture a press edge, which is exactly
       the property the fire work measured at this seam. The next tick's pad
       copy replaces every one of these fields wholesale, so stopping to write
       IS the release: there is nothing to restore and no way to leave a
       phantom stick behind. */
    *(volatile unsigned char *)(ULONG_PTR)
        (g_b.a.player_pad + DG_PAD_LEFT_DX_OFFSET) = out.dx;
    *(volatile unsigned char *)(ULONG_PTR)
        (g_b.a.player_pad + DG_PAD_LEFT_DY_OFFSET) = out.dy;
    *(volatile short *)(ULONG_PTR)
        (g_b.a.player_pad + DG_PAD_ANALOG_OFFSET) |= (short)out.or_analog;
    if (out.or_status)
        *(volatile LONG *)(ULONG_PTR)
            (g_b.a.player_pad + DG_PAD_STATUS_OFFSET) |= (LONG)out.or_status;
    InterlockedIncrement(&g_b.c_move_writes);
    if (out.dir_write) {
        *(volatile short *)(ULONG_PTR)
            (g_b.a.player_pad + DG_PAD_DIR_OFFSET) = (short)out.dir;
        InterlockedIncrement(&g_b.c_move_dir_writes);
        InterlockedExchange(&g_b.s_move_last_org, (LONG)in.dir_org);
        InterlockedExchange(&g_b.s_move_last_dir, (LONG)out.dir);
        /* CheckDirection already ran this tick with the driver's pad (dir
           -1, force 0): redo its two stores from OUR bytes. See
           move_resolve_workl. */
        if (InterlockedCompareExchange(&g_b.seam_is_copy, 0, 0)) {
            /* Copy seam: CheckDirection runs AFTER us this tick and derives
               PadTo, PadForce, WallTo and Liable from our stick itself. */
        } else if (g_b.workl_ptr) {
            ULONGLONG wl = *(volatile ULONGLONG *)(ULONG_PTR)g_b.workl_ptr;
            if (plausible_ptr(wl)) {
                *(volatile LONG *)(ULONG_PTR)(wl + 0x510) = (LONG)out.dir;
                *(volatile LONG *)(ULONG_PTR)(wl + 0x514) =
                    (LONG)dg_move_pad_force(out.dx, out.dy);
                InterlockedIncrement(&g_b.c_move_padto_writes);
            } else {
                InterlockedIncrement(&g_b.c_move_no_workl);
            }
        } else {
            InterlockedIncrement(&g_b.c_move_no_workl);
        }
    }
    if (third_ok) InterlockedIncrement(&g_b.c_move_third_writes);
    if (in.max_deflect) InterlockedIncrement(&g_b.c_move_prone_writes);
    if (third_ok || in.max_deflect) {
        InterlockedExchange(&g_b.mp_w_tick, now);
        InterlockedExchange(&g_b.mp_w_dir, out.dir_write ? out.dir : -2);
        InterlockedExchange(&g_b.mp_w_status,
                            (LONG)RD32(g_b.a.player_pad + DG_PAD_STATUS_OFFSET));
        InterlockedExchange(&g_b.mp_w_bytes,
                            ((LONG)out.dx << 8) | (LONG)out.dy);
    }
}

void dg_bridge_move_probe_now(ULONGLONG image_base)
{
#if DG_ENABLE_DIAGNOSTICS

    ULONGLONG pad = g_b.a.player_pad;
    LONG wt = InterlockedCompareExchange(&g_b.mp_w_tick, 0, 0);
    LONG now = InterlockedCompareExchange(&g_b.c_ticks, 0, 0);
    uint64_t id = 0; int weapon = -1;
    if (!wt || wt != now || !pad) return;
    if (InterlockedCompareExchange(&g_b.mp_c_tick, 0, 0) == now) return;
    InterlockedExchange(&g_b.mp_c_tick, now);
    InterlockedExchange(&g_b.mp_c_dir,
        (LONG)*(volatile short *)(ULONG_PTR)(pad + DG_PAD_DIR_OFFSET));
    InterlockedExchange(&g_b.mp_c_status, (LONG)RD32(pad + DG_PAD_STATUS_OFFSET));
    InterlockedExchange(&g_b.mp_c_analog,
        (LONG)(unsigned short)*(volatile short *)(ULONG_PTR)(pad + DG_PAD_ANALOG_OFFSET));
    InterlockedExchange(&g_b.mp_c_bytes,
        ((LONG)*(volatile unsigned char *)(ULONG_PTR)(pad + DG_PAD_LEFT_DX_OFFSET) << 8) |
         (LONG)*(volatile unsigned char *)(ULONG_PTR)(pad + DG_PAD_LEFT_DY_OFFSET));
    if (interact_player_now(&id, &weapon) && plausible_ptr(id) &&
        region_end(id) && id + 0xD28 <= region_end(id)) {
        InterlockedExchange64(&g_b.mp_c_act,
            (LONGLONG)*(volatile ULONGLONG *)(ULONG_PTR)(id + 0xC60));
        InterlockedExchange64(&g_b.mp_c_act2,
            (LONGLONG)*(volatile ULONGLONG *)(ULONG_PTR)(id + 0xC78));

        {
            ULONGLONG wp = *(volatile ULONGLONG *)(ULONG_PTR)(id + 0xD00);
            InterlockedExchange64(&g_b.mp_c_work_pad, (LONGLONG)wp);
            if (plausible_ptr(wp) && region_end(wp) && wp + 0x28 <= region_end(wp)) {
                InterlockedExchange(&g_b.mp_c_wp_status, (LONG)RD32(wp + DG_PAD_STATUS_OFFSET));
                InterlockedExchange(&g_b.mp_c_wp_dir,
                    (LONG)*(volatile short *)(ULONG_PTR)(wp + DG_PAD_DIR_OFFSET));
                InterlockedExchange(&g_b.mp_c_wp_bytes,
                    ((LONG)*(volatile unsigned char *)(ULONG_PTR)(wp + DG_PAD_LEFT_DX_OFFSET) << 8) |
                     (LONG)*(volatile unsigned char *)(ULONG_PTR)(wp + DG_PAD_LEFT_DY_OFFSET));
            } else {
                InterlockedExchange(&g_b.mp_c_wp_status, 0);
                InterlockedExchange(&g_b.mp_c_wp_dir, -3);
                InterlockedExchange(&g_b.mp_c_wp_bytes, 0);
            }
        }
        InterlockedExchange(&g_b.mp_c_rot,
            (LONG)*(volatile short *)(ULONG_PTR)(id + 0x82));
        InterlockedExchange(&g_b.mp_c_turn,
            (LONG)*(volatile short *)(ULONG_PTR)(id + 0x8A));
    } else {
        InterlockedExchange64(&g_b.mp_c_act, 0);
    }
    if (g_b.a.gv_pad_data)
        InterlockedExchange(&g_b.mp_c_gv_flag,
            (LONG)RD32(g_b.a.gv_pad_data + DG_GV_PAD_FLAG_OFFSET));

    if (image_base) {
        ULONGLONG wl = *(volatile ULONGLONG *)(ULONG_PTR)(image_base + 0x17DF780ULL);
        InterlockedExchange64(&g_b.mp_c_workl, (LONGLONG)wl);
        if (plausible_ptr(wl) && region_end(wl) && wl + 0x520 <= region_end(wl)) {
            InterlockedExchange(&g_b.mp_c_padto, (LONG)RD32(wl + 0x510));
            InterlockedExchange(&g_b.mp_c_wallto, (LONG)RD32(wl + 0x4F8));
            InterlockedExchange(&g_b.mp_c_liable, (LONG)RD32(wl + 0x50C));
            InterlockedExchange(&g_b.mp_c_padforce, (LONG)RD32(wl + 0x514));
        }
    }
    if (id && plausible_ptr(id) && region_end(id) && id + 0xCA8 <= region_end(id)) {
        InterlockedExchange(&g_b.mp_c_data, (LONG)RD32(id + 0xCA0));
        InterlockedExchange(&g_b.mp_c_data2, (LONG)RD32(id + 0xCA4));
    }
    InterlockedIncrement(&g_b.mp_c_seen);

#else

#endif
}

/* One tick of the trigger contract. In DRY nothing is written anywhere: the
   whole gate is evaluated, the state machine is stepped, and the answer is
   counted. That is deliberately the entire delivery of this step - a session
   can be flown with the trigger live in the log and no round spent, and the
   run that finally writes has already had its edges checked against a real
   player rather than against a test harness. */
#include "dg_coolant_trace.inl"
static void fire_tick(int safe_gameplay)
{
    DG_BRIDGE_FIRE cmd;
    DG_FIRE_IN in;
    DG_FIRE_OUT out;
    ULONGLONG pwork = 0, wp_set;
    LONG weapon = 0, now, age, tick = 0;
    LONG wmask = 0, pidx = -1;
    int m9_block=0,reload_block=0;
    static unsigned long last_stream;
    int mode = (int)InterlockedCompareExchange(&g_b.fire_mode, 0, 0);

    if (blade_claim() || mode == DG_FIRE_MODE_OFF) {
        dg_fire_reset(&g_b.fire);
        dg_recoil_reset(&g_b.recoil);
        InterlockedExchange(&g_b.s_recoil_amp, 0);
        return;
    }

    memset(&in, 0, sizeof in);
    in.click = 0.55;

    /* The age travels rather than being judged here. Whether a sample that is
       three ticks old still describes the player's finger is a question about
       the contract, so it belongs to the contract - and the answer differs
       depending on whether a weapon is already up. */
    if (!fire_read(&cmd, &tick)) {
        InterlockedIncrement(&g_b.c_fire_no_command);
    } else {
        now = InterlockedCompareExchange(&g_b.c_ticks, 0, 0);
        age = (LONG)((DWORD)now - (DWORD)tick);
        if (age < 0) age = DG_FIRE_FRESH_TICKS + 1;   /* clocks disagreeing */
        if (age > DG_FIRE_FRESH_TICKS) InterlockedIncrement(&g_b.c_fire_stale);
        /* A stream change is a new session, a new hand or a new mapping.
           Whatever pull was in flight belonged to the old one. */
        if (cmd.stream_id != last_stream) {
            last_stream = cmd.stream_id;
            dg_fire_reset(&g_b.fire);
        }
        in.have_sample = 1;
        in.sample_age = (int)age;
        in.press_seq = cmd.press_seq;
        in.release_seq = cmd.release_seq;
        in.value = cmd.value;
        in.click = cmd.click;
        in.input_ok = cmd.valid ? 1 : 0;
    }

    if(camera_fire_block(in.input_ok && in.have_sample && in.sample_age<=DG_FIRE_FRESH_TICKS,in.value)) {
        dg_fire_reset(&g_b.fire);dg_recoil_reset(&g_b.recoil);
        InterlockedExchange(&g_b.s_fire_state,DG_FIRE_IDLE);
        InterlockedExchange(&g_b.s_recoil_amp,0);
        return;
    }
    /* The game half of the gate, in the order that makes the cheapest refusal
       first. Everything here is a read. */
    if (g_b.fps.state == DG_FPS_ACTIVE && safe_gameplay &&
        !InterlockedCompareExchange(&g_b.s_late_unsafe, 0, 0) &&
        g_b.a.player_pad && g_b.a.pad_weapon &&
        resolve_player(NULL, &pwork, &weapon) == DG_RESOLVE_OK && weapon > 0) {
        /* None must not receive a weapon press: native unarmed actions can
           replace the resting finger pose. Capture has its own input owner. */
        /* PlayerPad.enable sits one int below the pad the merge point copies
           into. With it clear the player reads a different pad record and our
           contract would be written where nobody looks. */
        if (RD32(g_b.a.player_pad - 4) != 0) in.can_write = 1;
        wp_set = *(volatile ULONGLONG *)(ULONG_PTR)(pwork + 0xBA0);
        if (plausible_ptr(wp_set)) in.wtype = (unsigned int)RD32(wp_set + 0x10);
        /* Retail 2.1.0.0 ShootBullet: 0x51B9AE reads ftime2 at +C84,
           0x51BD08 returns below 8; 0x51BD32 dispatches data3 at +CA8.
           The global HOLD bit is set at 0x51B9B8. Never spend a synthetic
           trigger-release while the native draw/reload cannot consume it. */
        in.native_cancel_ready = RD32(pwork + 0xC84) >= 8;
        if (g_b.a.gm_player_status &&
            region_end(g_b.a.gm_player_status) >= g_b.a.gm_player_status + 8 &&
            (*(volatile ULONGLONG *)(ULONG_PTR)g_b.a.gm_player_status & 0x800ULL) &&
            RD32(pwork + 0xC84) >= 8 && RD32(pwork + 0xCA8) == 0)
            in.native_ready = 1;
        {
            LONG status = *(volatile LONG *)(ULONG_PTR)
                              (g_b.a.player_pad + DG_PAD_STATUS_OFFSET);
            wmask = (LONG)RD32(g_b.a.pad_weapon);
            in.physical_down = (wmask && (status & wmask)) ? 1 : 0;
        }
        /* PL_PAD_PRESS_WEAPON is an index into pressure[12], resolved at
           runtime like the mask beside it. Read here so the write path never
           touches an anchor it did not see through the gate. */
        if (g_b.a.pad_press_weapon) pidx = (LONG)RD32(g_b.a.pad_press_weapon);
        InterlockedExchange(&g_b.s_fire_index, pidx);
        /* The witness. We are at the top of the tick, so what stands in these
           two fields is what the Bluepoint layer decided AFTER our previous
           tick's write - WS_Draw where our press was taken, WS_HolsterQuick
           where it was refused. It is the only reading in this build that does
           not come from us, and the whole point of it is that it disagrees
           when we are wrong. */
        {
            LONG ws = RD32(g_b.a.player_pad + DG_PAD_WEAPON_STATE_OFFSET);
            LONG bs = RD32(g_b.a.player_pad + DG_PAD_BUTTON_STATE_OFFSET);
            InterlockedExchange(&g_b.s_weapon_state, ws);
            InterlockedExchange(&g_b.s_button_state, bs);
            if (ws >= 0 && ws < 32)
                InterlockedOr(&g_b.seen_weapon_state, (LONG)(1u << ws));
            if (bs >= 0 && bs < 32)
                InterlockedOr(&g_b.seen_button_state, (LONG)(1u << bs));
        }
    }
    if (!in.can_write) InterlockedIncrement(&g_b.c_fire_blocked_gate);

    if(mode==DG_FIRE_MODE_ON)
        m9_block=m9_fire_gate(safe_gameplay,in.can_write && wmask && pidx>=0 && pidx<12,pwork,weapon,in.physical_down,&in);
    if(mode==DG_FIRE_MODE_ON)
        reload_block = reload_tick(safe_gameplay,in.can_write && wmask && pidx>=0 && pidx<12,pwork,weapon,in.physical_down,&in);
    if(g_mod_capture) {
        if(g_mod_capture_ticks<100)g_mod_capture_ticks++;
        in.input_ok=0;
    } else g_mod_capture_ticks=0;
    mod_features_tick(safe_gameplay,(g_mod_capture_ticks>=DG_FIRE_ABORT_TICKS ||
        (g_b.fire.state==DG_FIRE_IDLE && !g_reload.driving)) &&
        g_controls_frame.m9.valid && !in.physical_down && in.value<0.1 &&
        !g_controls_frame.m9.trigger && !g_controls_frame.m9.grip);
    dg_fire_step(&g_b.fire, &in, &out);
    if(reload_block)out.flags&=~DG_FIRE_F_RELEASED;

    if (out.flags & DG_FIRE_F_DREW) InterlockedIncrement(&g_b.c_fire_drawn);
    if (out.flags & DG_FIRE_F_RELEASED)
        InterlockedIncrement(&g_b.c_fire_released);
    if (out.flags & DG_FIRE_F_ABORTING)
        InterlockedIncrement(&g_b.c_fire_aborted);
    if (out.flags & DG_FIRE_F_FORCED) InterlockedIncrement(&g_b.c_fire_forced);
    if (out.flags & DG_FIRE_F_BLOCKED_PHYS)
        InterlockedIncrement(&g_b.c_fire_blocked_phys);
    if (out.flags & DG_FIRE_F_REPEAT) InterlockedIncrement(&g_b.c_fire_repeats);
    if (out.flags & DG_FIRE_F_COASTING)
        InterlockedIncrement(&g_b.c_fire_coasting);
    if (out.flags & DG_FIRE_F_AUTO)
        InterlockedIncrement(&g_b.c_fire_auto_ticks);
    InterlockedExchange(&g_b.s_fire_state, (LONG)out.state);
    InterlockedExchange(&g_b.s_fire_pressure, (LONG)out.pressure);
    InterlockedExchange(&g_b.s_fire_wtype, (LONG)in.wtype);

    /* And only here does anything leave this file. DRY has already done every
       reading, every gate and every step above; the single difference is the
       call below. That is deliberate - the run that finally writes has had its
       edges checked against a real player, not against a harness.

       We stand down entirely while the player's own weapon button is held.
       Their input already holds the stance up, so there is nothing for us to
       add, and our release bit would end an aim we did not start. */
    if (mode == DG_FIRE_MODE_ON && in.can_write) {
        if (reload_block) reload_pad(wmask,pidx);
        else if (m9_block) m9_block_pad(wmask,pidx);
        else if (in.physical_down) InterlockedIncrement(&g_b.c_fire_yielded);
        else fire_write(&out, wmask, pidx);
    }
    coolant_trace_tick(safe_gameplay,mode,&in,&out,0); /* actual post-write pad */

    /* The kick. A shot is one impulse; the spring is stepped every tick
       whether or not one arrived, and deliberately including the ticks where
       the gate has just shut - a hand that is mid-recoil when first person
       ends still has to come back to rest, and the only way it can is by
       being stepped. This is also the only place the envelope advances: once
       per game tick, on the seam that is defined to run once per game tick. */
    if (mode == DG_FIRE_MODE_ON && (out.flags & DG_FIRE_F_RELEASED)) {
        /* ON only. In DRY no round leaves the barrel, so there is nothing to
           recoil from, and a hand that kicks without a shot would be the one
           piece of this build that lies to the player about what happened. */
        dg_recoil_fire(&g_b.recoil, 1.0);
        InterlockedIncrement(&g_b.c_recoil_kicks);
    }
    dg_recoil_step(&g_b.recoil);
    {
        LONG amp = (LONG)(dg_recoil_amplitude(&g_b.recoil) * 1000.0 + 0.5);
        InterlockedExchange(&g_b.s_recoil_amp, amp);
        if (amp > InterlockedCompareExchange(&g_b.s_recoil_worst, 0, 0))
            InterlockedExchange(&g_b.s_recoil_worst, amp);
    }
}

/* Search upward for the stride and take the FIRST hit. An exact multiple of the
   true stride validates just as well - it lands on every second joint, whose
   matrices are equally real - so "best score" would happily return double the
   answer. Lowest wins instead.
   The floor of 0xC0 is not a guess: world, screen and inv_mat are three
   FMATRIX at the head of DG_OBJ, ahead of anything a port would have touched,
   so no smaller stride is possible. */
static LONG find_stride(ULONGLONG objs, LONG n, ULONGLONG end,
                        LONG *score_out, LONG *tried_out)
{
    ULONGLONG base = objs + DG_OBJS_ARRAY;
    LONG s, tried = 0;
    LONG want = n < 6 ? n : 6;

    *score_out = 0;
    *tried_out = 0;
    /* Two joints prove nothing about a stride; the first is at offset zero and
       validates for every candidate. */
    if (want < 3) return 0;
    if (base + 0x40 > end) return 0;
    if (!looks_like_matrix(base)) return 0;

    for (s = 0xC0; s <= 0x800; s += 0x10) {
        LONG j;
        int ok = 1;
        for (j = 1; j < want; j++) {
            ULONGLONG m = base + (ULONGLONG)j * (ULONGLONG)s;
            if (m + 0x40 > end) { ok = 0; break; }
            if (!looks_like_matrix(m)) { ok = 0; break; }
        }
        if (ok) {
            *score_out = want;
            *tried_out = tried;
            return s;
        }
        tried++;
    }
    return 0;
}

/* Read-only throughout. Measures the stride and the parent table once per
   object - neither changes while a model is loaded - and refreshes the joint
   positions on every call, because those are the live pose and that is the
   point. */
static void skel_probe_now(ULONGLONG arm, int required_for_ik)
{
    ULONGLONG objs, end, base, mctrl;
    LONG n, stride, rel;
    int measured, j;

    if (!required_for_ik &&
        !InterlockedCompareExchange(&g_b.skel_probe, 0, 0)) return;

    objs = *(volatile ULONGLONG *)(ULONG_PTR)arm;
    if (!plausible_ptr(objs)) return;

    measured = (InterlockedCompareExchange(&g_b.skel_for_lo, 0, 0)
                    == (LONG)(DWORD)objs
             && InterlockedCompareExchange(&g_b.skel_for_hi, 0, 0)
                    == (LONG)(DWORD)(objs >> 32));

    if (measured) {
        rel = InterlockedCompareExchange(&g_b.skel_region_end, 0, 0);
        end = objs + (ULONGLONG)(DWORD)rel;
        stride = InterlockedCompareExchange(&g_b.skel_stride, 0, 0);
        if (!stride) return;
    } else {
        LONG score = 0, tried = 0;
        ULONGLONG span;

        end = region_end(objs);
        if (!end || end <= objs + DG_OBJS_ARRAY + 0x40) return;
        span = end - objs;
        if (span > 0x7FFFFFFFULL) span = 0x7FFFFFFFULL;
        InterlockedExchange(&g_b.skel_region_end, (LONG)(DWORD)span);
        end = objs + span;

        InterlockedExchange(&g_b.skel_objs_lo, (LONG)(DWORD)objs);
        InterlockedExchange(&g_b.skel_objs_hi, (LONG)(DWORD)(objs >> 32));

        n = (LONG)*(volatile short *)(ULONG_PTR)(objs + DG_OBJS_NMODELS);
        InterlockedExchange(&g_b.skel_n_models, n);
        if (n < 3 || n > 512) return;

        stride = find_stride(objs, n, end, &score, &tried);
        InterlockedExchange(&g_b.skel_stride, stride);
        InterlockedExchange(&g_b.skel_stride_score, score);
        InterlockedExchange(&g_b.skel_stride_tried, tried);
        if (!stride) return;

        base = objs + DG_OBJS_ARRAY;
        {
            LONG count = n < DG_SKEL_MAX ? n : DG_SKEL_MAX;
            for (j = 0; j < count; j++) {
                ULONGLONG o = base + (ULONGLONG)j * (ULONGLONG)stride;
                if (o + DG_OBJ_PARENT + 2 > end) { count = j; break; }
                InterlockedExchange(&g_b.skel_parents[j],
                    (LONG)*(volatile short *)(ULONG_PTR)(o + DG_OBJ_PARENT));
            }
            InterlockedExchange(&g_b.skel_parents_read, count);

            {
                int len = 0, cur = 6;
                while (len < DG_SKEL_CHAIN && cur >= 0 && cur < (int)count) {
                    int p;
                    InterlockedExchange(&g_b.skel_chain[len], (LONG)cur);
                    len++;
                    p = (int)InterlockedCompareExchange(
                            &g_b.skel_parents[cur], 0, 0);
                    if (p == cur) break;            /* its own parent: root */
                    cur = p;
                }
                InterlockedExchange(&g_b.skel_chain_len, (LONG)len);
            }
        }

        /* Last, so a torn read can never see this object marked measured while
           the tables above are still half written. */
        InterlockedExchange(&g_b.skel_for_lo, (LONG)(DWORD)objs);
        InterlockedExchange(&g_b.skel_for_hi, (LONG)(DWORD)(objs >> 32));
    }

    /* The live pose, every call. */
    base = objs + DG_OBJS_ARRAY;
    {
        int len = (int)InterlockedCompareExchange(&g_b.skel_chain_len, 0, 0);
        int first = (int)InterlockedCompareExchange(&g_b.skel_base, 0, 0);
        int count = (int)InterlockedCompareExchange(&g_b.skel_parents_read, 0, 0);

        /* The hand's own rotation, accumulated - the one number that turns
           "it still tumbles" into evidence. It is taken HERE, in the probe,
           because the probe runs whether or not we drive the arm: with
           vr_arm_track=off nothing else in this file reads the hierarchy at
           all, so the very run that would prove the spin is the game's own
           would otherwise record nothing. Total turned, not per-pair drift:
           the 2026-08-21 defect was smooth at under a degree per frame and
           therefore invisible to every jump-hunting instrument here, while
           amounting to fifteen revolutions in sixty-seven seconds. */
        if (len > 0) {
            int idx0 = (int)InterlockedCompareExchange(&g_b.skel_chain[0],
                                                       0, 0);
            ULONGLONG m0 = base + (ULONGLONG)idx0 * (ULONGLONG)stride;
            if (idx0 >= 0 && idx0 < count && m0 + 0x40 <= end) {
                const volatile float *f0 =
                    (const volatile float *)(ULONG_PTR)m0;
                double b[3][3], q[4];
                int r2, c2;
                for (r2 = 0; r2 < 3; r2++)
                    for (c2 = 0; c2 < 3; c2++)
                        b[r2][c2] = (double)f0[r2 * 4 + c2];
                if (dg_ik_basis_quat((const double (*)[3])b, q)) {
                    if (g_b.skel_hand_have_prev) {
                        double d = dg_ik_quat_angle(q, g_b.skel_hand_prev) *
                                   180.0 / 3.14159265358979323846;
                        if (_finite(d) && d >= 0.0 && d < 180.0) {
                            float tot;
                            LONG cur = g_b.s_skel_hand_turned;
                            memcpy(&tot, &cur, sizeof tot);
                            if (!_finite((double)tot)) tot = 0.0f;
                            InterlockedExchange(&g_b.s_skel_hand_turned,
                                                f2l(tot + (float)d));
                            InterlockedIncrement(&g_b.c_skel_hand_samples);
                        }
                    }
                    /* NET against a reference that never moves. The
                       accumulator above cannot tell winding from shaking,
                       and winding is the symptom: fifteen revolutions in
                       sixty-seven seconds arrived at well under a degree a
                       frame. Angle is folded into 0..180, so a full turn
                       reads as a sweep up and back down rather than as 360
                       - the tell is the MAX pinning near 180 and staying
                       there, against a stationary arm that never leaves
                       single digits. */
                    if (!g_b.skel_hand_have_ref) {
                        for (r2 = 0; r2 < 4; r2++) g_b.skel_hand_ref[r2] = q[r2];
                        g_b.skel_hand_have_ref = 1;
                        InterlockedExchange(&g_b.s_skel_hand_net, f2l(0.0f));
                        InterlockedExchange(&g_b.s_skel_hand_net_max, f2l(0.0f));
                    } else {
                        double n = dg_ik_quat_angle(q, g_b.skel_hand_ref) *
                                   180.0 / 3.14159265358979323846;
                        if (_finite(n) && n >= 0.0 && n <= 180.0) {
                            float mx;
                            LONG cur = g_b.s_skel_hand_net_max;
                            InterlockedExchange(&g_b.s_skel_hand_net,
                                                f2l((float)n));
                            memcpy(&mx, &cur, sizeof mx);
                            if (!_finite((double)mx) || (float)n > mx)
                                InterlockedExchange(&g_b.s_skel_hand_net_max,
                                                    f2l((float)n));
                        }
                    }
                    for (r2 = 0; r2 < 4; r2++) g_b.skel_hand_prev[r2] = q[r2];
                    g_b.skel_hand_have_prev = 1;
                }
            }
        }

        for (j = 0; j < len && j < DG_SKEL_CHAIN; j++) {
            int idx = (int)InterlockedCompareExchange(&g_b.skel_chain[j], 0, 0);
            ULONGLONG m = base + (ULONGLONG)idx * (ULONGLONG)stride;
            const volatile float *f;
            if (idx < 0 || idx >= count || m + 0x40 > end) continue;
            f = (const volatile float *)(ULONG_PTR)m;
            InterlockedExchange(&g_b.skel_chain_pos[j * 3 + 0], f2l(f[12]));
            InterlockedExchange(&g_b.skel_chain_pos[j * 3 + 1], f2l(f[13]));
            InterlockedExchange(&g_b.skel_chain_pos[j * 3 + 2], f2l(f[14]));
        }
        for (j = 0; j < DG_SKEL_WINDOW; j++) {
            int idx = first + j;
            ULONGLONG m = base + (ULONGLONG)idx * (ULONGLONG)stride;
            const volatile float *f;
            if (idx < 0 || idx >= count || m + 0x40 > end) continue;
            f = (const volatile float *)(ULONG_PTR)m;
            InterlockedExchange(&g_b.skel_window_pos[j * 3 + 0], f2l(f[12]));
            InterlockedExchange(&g_b.skel_window_pos[j * 3 + 1], f2l(f[13]));
            InterlockedExchange(&g_b.skel_window_pos[j * 3 + 2], f2l(f[14]));
        }
    }

    /* MOTION_CONTROL.trans, the per-joint translation channel. adjust is
       rotation only, so whether this one is live decides whether wrist position
       has a direct route or has to be solved for. Answering it now is cheaper
       than discovering it halfway through an IK. */
    mctrl = *(volatile ULONGLONG *)(ULONG_PTR)(arm + 0x08);
    if (plausible_ptr(mctrl)) {
        ULONGLONG t = *(volatile ULONGLONG *)(ULONG_PTR)(mctrl + 0x60);
        InterlockedExchange(&g_b.skel_mctrl_trans_lo, (LONG)(DWORD)t);
        InterlockedExchange(&g_b.skel_mctrl_trans_hi, (LONG)(DWORD)(t >> 32));
    }
}

/* ----------------------------------------- F5: tracked two-bone arm IK --- */

static int arm_ik_replay_cached(ULONGLONG arm)
{
    ULONGLONG mctrl, adjust;
    LONG joints;
    int j, k;
    if (!g_b.arm_map_cache_valid || !g_b.ik_active ||
        arm != g_b.ik_owned_arm) return 0;
    mctrl = *(volatile ULONGLONG *)(ULONG_PTR)(arm + 0x08);
    if (!plausible_ptr(mctrl) || mctrl != g_b.ik_owned_mctrl) return 0;
    joints = RD32(mctrl + 0x14);
    adjust = *(volatile ULONGLONG *)(ULONG_PTR)(mctrl + 0x48);
    if (joints <= 5 || joints > 255 || !plausible_ptr(adjust) ||
        adjust != g_b.ik_owned_adjust) return 0;
    for (j = 4; j <= 5; j++) {
        volatile float *dst;
        const float *src = &g_b.arm_map_cached_adjust[(j - 4) * 4];
        if (!((g_b.ik_owned_mask >> j) & 1)) continue;
        dst = (volatile float *)(ULONG_PTR)(adjust + j * 16);
        for (k = 0; k < 4; k++) dst[k] = src[k];
    }
    *(volatile ULONGLONG *)(ULONG_PTR)(mctrl + 0x38) |=
        g_b.ik_owned_mask & ((1ULL << 4) | (1ULL << 5));
    InterlockedIncrement(&g_b.c_arm_bent);
    InterlockedIncrement(&g_b.c_arm_tracked);
    return 1;
}

static void capture_yaw_zeros(const DG_BRIDGE_ARM_TARGET *target)
{
    g_b.head_yaw0_have = 0;
    if (target->head_yaw_valid) {
        g_b.head_yaw0_deg = target->head_yaw_rad * 180.0 / 3.14159265358979323846;
        g_b.head_yaw0_have = 1;
    }
    g_b.hand_yaw0_have = 0;
    if (target->hand_yaw_valid) {
        g_b.hand_yaw0_deg = target->hand_yaw_rad * 180.0 / 3.14159265358979323846;
        g_b.hand_yaw0_have = 1;
    }
    g_b.stick_yaw0_have = 0;
    if (target->stick_yaw_valid) {
        g_b.stick_yaw0_deg = target->stick_yaw_rad * 180.0 / 3.14159265358979323846;
        g_b.stick_yaw0_have = 1;
    }
}

static int unarmed_hand_rest(ULONGLONG arm,const double root_inv[4],
                             const DG_ADJ_FRAME *frame,double out[4]);
static int unarmed_wrist_target(ULONGLONG arm,const DG_ADJ_FRAME *frame,
    const double q4[4],const double q5[4],int cached,double out[4]);

static void arm_ik_now(ULONGLONG arm, const DG_BRIDGE_ARM_TARGET *target)
{
    ULONGLONG objs, base, end, mctrl, adjust;
    LONG stride, count, rel, joints;
    DG_ARM_MAP_IN map_in;
    DG_ARM_MAP_OUT map_out;
    double root[4][4], root_pos[3], target_world[3];
    double point[5][3], clean_point[5][3];
    double shoulder_view[3], wrist_view[3], q4[4], q5[4];
    double root_distance = 0.0, root_limit = 0.0;
    double hand_basis[3][3], root_basis[3][3];
    double root_q[4], root_q_inv[4], live_hand[4], hand[4], desired[4];
    double comp_q[4], hand_root_q[4];
    double ctrl_view_c[3], player_shoulder_c[3];
    DG_IK_HAND_STABILIZE_OUT hand_stable;
    double previous_hand_adjust[4];
    ULONGLONG mask;
    unsigned clamped = 0;
    unsigned ps_flags = 0;
    short wanted_rot[3], command_rot[3];
    int j, k, r, plausible, recover_cached;
    int have_frames = 0, have_hand = 0;
    DG_BRIDGE_ARM_TARGET effective_target;

    quat_identity(root_q);
    quat_identity(root_q_inv);
    quat_identity(live_hand);
    quat_identity(desired);
    quat_identity(previous_hand_adjust);
    memset(&hand_stable, 0, sizeof hand_stable);
    if (!target) return;
    /* None has no pistol object. Resolve the live player before choosing
       its ordinary controller arm route, and reset calibration across modes. */
    {
        ULONGLONG current = 0;
        LONG weapon = -1;
        int unarmed = resolve_player(&current, NULL, &weapon) == DG_RESOLVE_OK &&
            current == arm && weapon == 0;
        if (g_b.arm_map_unarmed != unarmed) {
            arm_map_forget();
            g_b.arm_map_unarmed = unarmed;
        }
        if (unarmed && target->absolute_aim) {
            effective_target = *target;
            effective_target.absolute_aim = 0;
            /* None lacks weapon AIM ownership, but its raw tracked grip
               still needs the live camera height through crouch/crawl. */
            effective_target.position.enabled = target->position.enabled &&
                                                 target->position.valid;
            effective_target.hand_write = target->unarmed_hand_write;
            target = &effective_target;
        }
    }
    if (target->absolute_aim) {
        DG_AIM_SELECTION selection;
        int valid = target->aim_write && target->aim_pair_id == target->pair_id &&
            target->aim_stream_id == target->stream_id && target->aim_sample_seq &&
            target->aim_sample_time > 0 && target->aim_arm == arm &&
            g_b.a.gm_player_arm_body > 0x17df698 &&
            dg_aim_capture_hand_selection(
                (uintptr_t)(g_b.a.gm_player_arm_body - 0x17df698), arm, &selection) &&
            selection.subobject == target->aim_subobject &&
            selection.subobjs == target->aim_subobjs && selection.hand == target->aim_hand &&
            selection.model == target->aim_model &&
            selection.weapon_id == target->aim_weapon_id;
        effective_target = *target;
        if (!valid) effective_target.hand_write = 0;
        if(target->position.enabled) {
            double position_frame[4];
            if(!valid || !dg_position_frame(&target->position,position_frame))
                effective_target.write=effective_target.hand_write=0;
        }
        target = &effective_target;
    }
    /* Before every duplicate/constant replay return: invalid aim may never
       leave the previous command live in the next player Action tick. */
    if(g_b.arm_map_unarmed && target->free_right.enabled && !target->free_right.valid) {
        effective_target=*target;effective_target.hand_write=0;target=&effective_target;
        memset(&g_b.free_right,0,sizeof g_b.free_right);
    }
    if (!target->hand_write) hand_command_clear();

    /* The recorder's game-side half starts truthful-empty every pass and
       fills as far as this pass actually got; the flight recorder embeds
       whatever stands here on the NEXT camera pass (same thread). This is
       the half whose absence made every tumble cost a live headset run. */
    memset(&g_b.rec_pair, 0, sizeof g_b.rec_pair);
    g_b.rec_pair.rest_drift_deg = -1.0f;    /* absence is not zero */
    /* DGREC4: the facings this pair was solved against - the view's yaw,
       the stick's turn and the aim - so the desk can put the hand beside
       what the player was looking at instead of guessing from a mirror
       video (run 14, 2026-09-03). */
    if (target->head_yaw_valid || target->stick_yaw_valid ||
        target->hand_yaw_valid) {
        g_b.rec_pair.head_yaw_deg = target->head_yaw_valid ?
            (float)(target->head_yaw_rad * 180.0 / 3.14159265358979323846)
            : 0.0f;
        g_b.rec_pair.stick_yaw_deg = target->stick_yaw_valid ?
            (float)(target->stick_yaw_rad * 180.0 / 3.14159265358979323846)
            : 0.0f;
        g_b.rec_pair.aim_yaw_deg = target->hand_yaw_valid ?
            (float)(target->hand_yaw_rad * 180.0 / 3.14159265358979323846)
            : 0.0f;
        g_b.rec_pair.flags |= DG_REC_PAIR_F_FACING;
    }
    g_b.rec_pair.wtype =
        (unsigned int)InterlockedCompareExchange(&g_b.s_fire_wtype, 0, 0);
    if (target->hand_write) g_b.rec_pair.flags |= DG_REC_PAIR_F_HAND_ON;
    InterlockedExchange(&g_b.hand_command_requested,
                        target->hand_write ? 1 : 0);
    InterlockedExchange(&g_b.s_arm_pose_flags, (LONG)target->pose_flags);
    InterlockedExchange(&g_b.s_arm_ik_weight, f2l((float)target->weight));
    for (k = 0; k < 3; k++)
        InterlockedExchange(&g_b.s_arm_ik_view[k],
                            f2l((float)target->wrist_view[k]));
    if (!target->write || target->weight <= 0.0) {
        arm_map_forget();
        return;
    }

    objs = *(volatile ULONGLONG *)(ULONG_PTR)arm;
    if (!plausible_ptr(objs)) goto refuse_unpaired;

    /* Any identity or stream change starts with a release and two clean game
       ticks. The current hierarchy may still contain our previous adjust. */
    if (!g_b.arm_map_phase || arm != g_b.arm_map_arm ||
        objs != g_b.arm_map_objs || target->stream_id != g_b.arm_map_stream) {
        InterlockedIncrement(&g_b.c_arm_pairs_seen);
        arm_map_begin(arm, objs, target);
        return;
    }

    /* Live-frame validity precedes EVERY cached write. Pointer identity in
       arm_ik_replay_cached alone does not prove that this arm still belongs
       to a resolvable player. Refuse here rather than falling through from
       constant replay into a new solve. Legacy deliberately needs no live
       heading; preserve that separate diagnostic contract.

       Check a temporary frame: replay remains byte-for-byte and must not
       replace the last solve's pair_frame with the current heading. The
       ordinary solve still acquires its own frame alongside the hierarchy. */
    if (InterlockedCompareExchange(&g_b.adjust_frame, 0, 0) == 1) {
        ULONGLONG current_arm = 0, player_work = 0, player_end = 0;
        double frame_q[4];
        int frame_ok = resolve_motion_player(&current_arm, &player_work, NULL) == DG_RESOLVE_OK &&
                       current_arm == arm;
        if (frame_ok) player_end = region_end(player_work);
        frame_ok = frame_ok && player_end && player_work + 0x84 <= player_end &&
                   arm_frame_yaw(*(volatile short *)(ULONG_PTR)(player_work + 0x82), frame_q);
        if (!frame_ok) {
            InterlockedIncrement(&g_b.c_frame_missing);
            goto refuse_unpaired;
        }
    }

    /* camera setup can hit this seam 5+ times in one tick, and the second eye
       carries the same pair id. Mapping/solving happens once; writes replay. */
    if (target->pair_id == g_b.arm_map_last_pair) {
        InterlockedIncrement(&g_b.c_arm_pair_replays);
        if (g_b.arm_map_cache_valid && !arm_ik_replay_cached(arm)) {
            InterlockedIncrement(&g_b.c_arm_ik_refused);
            arm_map_forget();
        }
        return;
    }

    if (InterlockedCompareExchange(&g_b.arm_freeze, 0, 0) == 1 &&
        g_b.arm_map_cache_valid && g_b.ik_active && arm == g_b.ik_owned_arm) {
        g_b.arm_map_last_pair = target->pair_id;
        InterlockedIncrement(&g_b.c_arm_pairs_seen);
        InterlockedExchange(&g_b.s_arm_map_pair, (LONG)target->pair_id);
        if (arm_ik_replay_cached(arm)) {
            InterlockedIncrement(&g_b.c_arm_pairs_frozen);
            return;
        }
    }

    recover_cached = g_b.arm_map_cache_valid && g_b.ik_active &&
                     arm == g_b.ik_owned_arm;
    g_b.arm_map_last_pair = target->pair_id;
    g_b.arm_map_cache_valid = 0;
    InterlockedIncrement(&g_b.c_arm_pairs_seen);
    InterlockedExchange(&g_b.s_arm_map_pair, (LONG)target->pair_id);
    InterlockedExchange(&g_b.s_arm_map_stream, (LONG)target->stream_id);

    if (g_b.c_ticks <= g_b.arm_map_settle_until) {
        InterlockedIncrement(&g_b.c_arm_pairs_calibration);
        return;
    }

    stride = InterlockedCompareExchange(&g_b.skel_stride, 0, 0);
    count = InterlockedCompareExchange(&g_b.skel_parents_read, 0, 0);
    rel = InterlockedCompareExchange(&g_b.skel_region_end, 0, 0);
    if (!stride || count <= 6 || !rel ||
        g_b.skel_parents[3] != 2 || g_b.skel_parents[4] != 3 ||
        g_b.skel_parents[5] != 4 || g_b.skel_parents[6] != 5)
        goto refuse_pair;

    base = objs + DG_OBJS_ARRAY;
    end = objs + (ULONGLONG)(DWORD)rel;
    if (base + 0x40 > end || !looks_like_matrix(base)) goto refuse_pair;
    {
        const volatile float *f = (const volatile float *)(ULONG_PTR)base;
        for (r = 0; r < 4; r++)
            for (k = 0; k < 4; k++) root[r][k] = (double)f[r * 4 + k];
        for (r = 0; r < 3; r++)
            for (k = 0; k < 3; k++) root_basis[r][k] = root[r][k];
    }
    for (j = 2; j <= 6; j++) {
        ULONGLONG m = base + (ULONGLONG)j * (ULONGLONG)stride;
        const volatile float *f;
        if (m + 0x40 > end || !looks_like_matrix(m)) goto refuse_pair;
        f = (const volatile float *)(ULONG_PTR)m;
        for (k = 0; k < 3; k++) point[j - 2][k] = (double)f[12 + k];
        /* The hand's orientation, which nothing before this build read. The
           three basis rows are the world directions of the joint's own axes;
           dg_ik_basis_quat owns the decision about transposing them. */
        if (j == 6)
            for (r = 0; r < 3; r++)
                for (k = 0; k < 3; k++)
                    hand_basis[r][k] = (double)f[r * 4 + k];
    }
    memcpy(clean_point, point, sizeof clean_point);

    /* The wrist-miss meter's reading: the previous pair predicted where the
       wrist would land under its frame; this read says where it did. The
       animation moves too between pairs, so single readings carry that
       noise; the worst and the over-20 mm count are what the legacy-versus-
       live comparison of a run reads. */
    if (g_b.have_pred_wrist) {
        double miss = vdist3(point[4], g_b.pred_wrist);
        g_b.have_pred_wrist = 0;
        /* A faded pair (weight below 1: the runtime blending the arm back
           toward its animation) is not graded - the 531-587 mm "worst" of
           the 2026-09-02 runs were all the headset being put down, and a
           meter that mostly reads pose loss says nothing about the frame. */
        if (_finite(miss) && target->weight >= 1.0) {
            float worst;
            LONG cur = g_b.s_wrist_miss_worst;
            memcpy(&worst, &cur, sizeof worst);
            if (!_finite((double)worst) || (float)miss > worst)
                InterlockedExchange(&g_b.s_wrist_miss_worst, f2l((float)miss));
            if (miss > 20.0) InterlockedIncrement(&g_b.c_wrist_miss_over);
        }
    }

    /* The adjust frame for this pair (V5.1 par. 4.3): the body yaw word read
       in the same round as the matrices above, so the strip, the solve, the
       hand paths and the roll all convert under one heading. Under legacy
       the frame is the constant and the word only feeds the recorder and
       the skew meter; under live a pair without a resolvable player has no
       frame, and every conversion below refuses at its own policy - counted
       as `missing` in the heartbeat. */
    {
        ULONGLONG pw = 0, pe = 0;
        int live = InterlockedCompareExchange(&g_b.adjust_frame, 0, 0) == 1;
        g_b.pair_rot_ok = 0;
        g_b.pair_rot_vy = 0;
        if (resolve_motion_player(NULL, &pw, NULL) == DG_RESOLVE_OK)
            pe = region_end(pw);
        if (pw && pe && pw + 0x84 <= pe) {
            g_b.pair_rot_vy = *(volatile short *)(ULONG_PTR)(pw + 0x82);
            g_b.pair_rot_ok = 1;
        }
        g_b.pair_frame.live = live;
        /* The recorder's frame word (V5.1 par. 4.5): what heading this pair
           had, and whether the live frame was in force, so a frame fault is
           reconstructible from any later replay. */
        if (g_b.pair_rot_ok) {
            g_b.rec_pair.frame_word = DG_REC_FRAME_WORD_VALID |
                                      (unsigned int)(unsigned short)g_b.pair_rot_vy;
            g_b.rec_pair.flags |= DG_REC_PAIR_F_FRAME;
        }
        if (live) g_b.rec_pair.flags |= DG_REC_PAIR_F_FRAME_LIVE;
        if (live) {
            g_b.pair_frame.valid = g_b.pair_rot_ok &&
                                   arm_frame_yaw(g_b.pair_rot_vy,
                                                 g_b.pair_frame.q);
            if (!g_b.pair_frame.valid)
                InterlockedIncrement(&g_b.c_frame_missing);
        } else {
            g_b.pair_frame.valid = 1;
            quat_identity(g_b.pair_frame.q);
        }
    }

    /* Both frames, or neither. A rig root that is not a rotation - mirrored or
       badly scaled - has no quaternion, and the hand simply stays animated
       rather than being turned by a transform nobody has verified. */
    have_frames = dg_ik_basis_quat((const double (*)[3])root_basis, root_q) &&
                  dg_ik_basis_quat((const double (*)[3])hand_basis, live_hand);
    if (have_frames) dg_ik_quat_conj(root_q, root_q_inv);
    if (have_frames) {
        for (k = 0; k < 4; k++) {
            g_b.rec_pair.root_q[k] = root_q[k];
            g_b.rec_pair.live_hand[k] = live_hand[k];
        }
        g_b.rec_pair.flags |= DG_REC_PAIR_F_FRAMES;
    }
    if (g_b.arm_root_have_q0)
        arm_reference(&g_b.right_reference,target,g_b.arm_root_q0,
                      g_b.hand_ctrl_rest,&g_b.stick_yaw0_deg);
    if (g_b.arm_map_phase == 2) g_b.rec_pair.flags |= DG_REC_PAIR_F_PHASE2;
    if (g_b.arm_root_have_q0)
        for (k = 0; k < 4; k++)
            g_b.rec_pair.root_q0[k] = g_b.arm_root_q0[k];
    if (g_b.hand_have_rest) {
        for (k = 0; k < 4; k++) {
            g_b.rec_pair.rest_view[k] = g_b.hand_rest_view[k];
            g_b.rec_pair.ctrl_rest[k] = g_b.hand_ctrl_rest[k];
        }
        g_b.rec_pair.flags |= DG_REC_PAIR_F_REST;
    }

    quat_identity(comp_q);
    for (k = 0; k < 3; k++) {
        ctrl_view_c[k] = target->wrist_view[k];
        player_shoulder_c[k] = target->player_shoulder_view[k];
    }
    for (k = 0; k < 4; k++) hand_root_q[k] = root_q[k];
    if (g_b.arm_map_phase == 2) {
        int compensated = 0;
        double drift_deg = 0.0;
        int have_drift = 0;
        if (have_frames && g_b.arm_root_have_q0) {
            double rel[4], norm;
            dg_ik_quat_mul(root_q_inv, g_b.arm_root_q0, rel);
            norm = sqrt(rel[1] * rel[1] + rel[3] * rel[3]);
            if (norm > 1.0e-6) {
                double drift;
                comp_q[0] = 0.0;
                comp_q[1] = rel[1] / norm;
                comp_q[2] = 0.0;
                comp_q[3] = rel[3] / norm;
                drift = -2.0 * atan2(comp_q[1], comp_q[3]) *
                        180.0 / 3.14159265358979323846;
                if (drift > 180.0) drift -= 360.0;
                if (drift < -180.0) drift += 360.0;

                if (InterlockedCompareExchange(&g_b.arm_comp, 0, 0) == 1) {
                    if (target->stick_yaw_valid && g_b.stick_yaw0_have) {
                        LONG hs = InterlockedCompareExchange(
                            &g_b.follow_head_sign, 0, 0);
                        double software = (hs < 0 ? -1.0 : 1.0) *
                            (target->stick_yaw_rad *
                                 180.0 / 3.14159265358979323846 -
                             g_b.stick_yaw0_deg);
                        double organic, half;
                        while (software > 180.0) software -= 360.0;
                        while (software < -180.0) software += 360.0;
                        organic = drift - software;
                        while (organic > 180.0) organic -= 360.0;
                        while (organic < -180.0) organic += 360.0;
                        half = -0.5 * organic * 3.14159265358979323846 / 180.0;
                        comp_q[1] = sin(half);
                        comp_q[3] = cos(half);
                    } else {
                        InterlockedIncrement(&g_b.c_arm_comp_no_stick);
                    }
                }
                arm_quat_rotate(comp_q, target->wrist_view, ctrl_view_c);
                arm_quat_rotate(comp_q, target->player_shoulder_view,
                                player_shoulder_c);
                /* The hand takes the same correction through its rebase
                   basis: for a pure body yaw, root * comp IS the calibration
                   root, so the orientation the player holds stays
                   world-fixed exactly. */
                dg_ik_quat_mul(root_q, comp_q, hand_root_q);
                InterlockedExchange(&g_b.s_arm_body_drift,
                                    f2l((float)drift));
                {

                    double up[3] = { 0.0, 1.0, 0.0 }, v[3], c;
                    arm_quat_rotate(root_q, up, v);
                    c = v[1] < -1.0 ? -1.0 : (v[1] > 1.0 ? 1.0 : v[1]);
                    InterlockedExchange(&g_b.s_arm_root_tilt,
                        f2l((float)(acos(c) * 180.0 /
                                    3.14159265358979323846)));
                }
                turn_dir_vote(drift);
                drift_deg = drift;
                have_drift = 1;
                compensated = 1;
            }
        }
        if (!compensated)
            InterlockedIncrement(&g_b.c_arm_body_uncompensated);
        /* THE AIM GAP is the player's FACING minus the body: the mapped
           head yaw straight off the camera path, minus the body's
           published yaw drift. Its two predecessors both lied. The
           hand-path publisher starved with vr_arm_hand=off. The
           position-path one read controller-versus-head geometry, which
           does not change when the player turns on their feet - a live
           session (2026-08-23) measured the gap pinned at 0.0 through
           eleven manual recalibrations while the follow steered on the
           CONTROLLER instead: swing the hand far enough sideways and the
           body spun after it, gap co-moving with drift at +127/+128.
           The head yaw is independent of anything the body or the hand
           does, and the drift is subtracted with exactly the sign the
           votes are trained on, so d gap = -d drift holds by
           construction and the tail-chase is structurally impossible.
           vr_follow_head_sign flips the head term if the camera's yaw
           convention proves mirrored against the root's - one live
           marker edit instead of a redeploy. */

        {
        int src = (int)InterlockedCompareExchange(&g_b.follow_src, 0, 0);
        int src_hand = src == 1;
        int facing_valid = src_hand ? target->hand_yaw_valid : target->head_yaw_valid;
        double facing_rad = src_hand ? target->hand_yaw_rad : target->head_yaw_rad;
        double facing0_deg = src_hand ? g_b.hand_yaw0_deg : g_b.head_yaw0_deg;
        int have0 = src_hand ? g_b.hand_yaw0_have : g_b.head_yaw0_have;
        if (src == 2) {
            /* The stick's own turn and nothing else: a glance or a hand
               swung sideways moves no facing here, only the right stick
               (and the calibration zero absorbs the recentre). */
            facing_valid = target->stick_yaw_valid;
            facing_rad = target->stick_yaw_rad;
            facing0_deg = g_b.stick_yaw0_deg;
            have0 = g_b.stick_yaw0_have;
        }
        if (have_drift && have0 && facing_valid) {
            LONG hs = InterlockedCompareExchange(&g_b.follow_head_sign,
                                                 0, 0);
            double head = (hs < 0 ? -1.0 : 1.0) *
                          (facing_rad * 180.0 / 3.14159265358979323846 -
                           facing0_deg);
            double gap;
            if (head > 180.0) head -= 360.0;
            if (head < -180.0) head += 360.0;
            gap = head - drift_deg;
            if (gap > 180.0) gap -= 360.0;
            if (gap < -180.0) gap += 360.0;
            InterlockedExchange(&g_b.s_arm_aim_gap, f2l((float)gap));
            InterlockedExchange(&g_b.s_arm_head_yaw, f2l((float)head));
            InterlockedExchange(&g_b.arm_aim_gap_tick,
                InterlockedCompareExchange(&g_b.c_ticks, 0, 0));
        }
        }
    }

    /* How far the hand landed from what the previous pair asked of it. This is
       measured before anything is removed or solved, because it is the one
       number that says whether the hierarchy model above is actually right. */
    if (have_frames && g_b.hand_have_desired) {
        double miss = dg_ik_quat_angle(live_hand, g_b.hand_desired) *
                      180.0 / 3.14159265358979323846;
        LONG worst = g_b.s_arm_hand_worst;
        float previous;
        memcpy(&previous, &worst, sizeof previous);
        InterlockedExchange(&g_b.s_arm_hand_residual, f2l((float)miss));
        if (target->weight >= 1.0 &&
            (!_finite((double)previous) || miss > (double)previous))
            InterlockedExchange(&g_b.s_arm_hand_worst, f2l((float)miss));
        InterlockedIncrement(&g_b.c_arm_hand_measured);
        /* The offset itself, compared to the previous pair's. Consecutive
           measured pairs only: one skipped pair and the comparison would
           span a gap the number cannot speak for. */
        {
            double inv_desired[4], off[4];
            dg_ik_quat_conj(g_b.hand_desired, inv_desired);
            dg_ik_quat_mul(live_hand, inv_desired, off);
            if (dg_ik_quat_normalize(off)) {
                if (g_b.hand_have_prev_off) {
                    double drift = dg_ik_quat_angle(off, g_b.hand_prev_off) *
                                   180.0 / 3.14159265358979323846;
                    LONG dworst = g_b.s_arm_hand_off_drift_worst;
                    float dprev;
                    memcpy(&dprev, &dworst, sizeof dprev);
                    InterlockedExchange(&g_b.s_arm_hand_off_drift,
                                        f2l((float)drift));
                    if (!_finite((double)dprev) || drift > (double)dprev)
                        InterlockedExchange(&g_b.s_arm_hand_off_drift_worst,
                                            f2l((float)drift));
                }
                for (k = 0; k < 4; k++) g_b.hand_prev_off[k] = off[k];
                g_b.hand_have_prev_off = 1;
            } else {
                g_b.hand_have_prev_off = 0;
            }
        }
    } else {
        g_b.hand_have_prev_off = 0;
    }
    g_b.hand_have_desired = 0;
    if (recover_cached) {
        mctrl = *(volatile ULONGLONG *)(ULONG_PTR)(arm + 0x08);
        if (!plausible_ptr(mctrl) || mctrl != g_b.ik_owned_mctrl)
            goto refuse_pair;
        joints = RD32(mctrl + 0x14);
        adjust = *(volatile ULONGLONG *)(ULONG_PTR)(mctrl + 0x48);
        if (joints <= 5 || joints > 255 || !plausible_ptr(adjust) ||
            adjust != g_b.ik_owned_adjust ||
            !arm_remove_cached_adjust(&g_b.pair_frame, point,
                                      g_b.arm_map_cached_adjust,
                                      clean_point)) goto refuse_pair;
        /* The removal above is only honest if the hierarchy actually CARRIES
           what it removes. If some engine path dropped our adjust_flag bits -
           a walking animation branch is the suspect the 2026-08-20 probe
           raised - the matrices are pure animation, the removal rotates the
           recovered rest by the inverse of our own adjusts, and the solve
           chases its own tail at up to 175 degrees per pair. Counted, not
           assumed. */
        {
            ULONGLONG fl = *(volatile ULONGLONG *)(ULONG_PTR)(mctrl + 0x38);
            if ((fl & ((1ULL << 4) | (1ULL << 5))) !=
                ((1ULL << 4) | (1ULL << 5)))
                InterlockedIncrement(&g_b.c_arm_adjust_bits_lost);
        }
        /* THE ECHO FOR JOINTS 4 AND 5 - the check joint 6 has had since
           2026-08-20 and these two never did. The removal above is only
           honest if the hierarchy holds what we cached: it subtracts the
           CACHED rotation from the LIVE points, so if the game changed the
           slot in between - blended it, reset it, wrote its own - we remove
           a rotation that is not there and the recovered "animation" keeps
           a residue of our own last write. The reference the roll anchor
           hangs on comes from exactly those recovered points, so a residue
           does not stay put: it is re-solved, re-written and re-recovered
           every pair. That is a compounding loop, and compounding is what a
           smooth 81 degrees a second with a calm controller looks like.
           Never measured before, so never excluded. */
        {
            const volatile float *a4s =
                (const volatile float *)(ULONG_PTR)(adjust + 4 * 16);
            const volatile float *a5s =
                (const volatile float *)(ULONG_PTR)(adjust + 5 * 16);
            double live4[4], live5[4], cache4[4], cache5[4];
            int ok = 1;
            for (k = 0; k < 4; k++) {
                live4[k] = (double)a4s[k];
                live5[k] = (double)a5s[k];
                cache4[k] = (double)g_b.arm_map_cached_adjust[k];
                cache5[k] = (double)g_b.arm_map_cached_adjust[4 + k];
                if (!_finite(live4[k]) || !_finite(live5[k])) ok = 0;
            }
            if (ok && dg_ik_quat_normalize(live4) &&
                dg_ik_quat_normalize(live5) &&
                dg_ik_quat_normalize(cache4) &&
                dg_ik_quat_normalize(cache5)) {
                double e4 = dg_ik_quat_angle(live4, cache4) *
                            180.0 / 3.14159265358979323846;
                double e5 = dg_ik_quat_angle(live5, cache5) *
                            180.0 / 3.14159265358979323846;
                double e = (e4 > e5) ? e4 : e5;
                LONG w = g_b.s_arm_adj_echo_worst;
                float prev;
                memcpy(&prev, &w, sizeof prev);
                InterlockedExchange(&g_b.s_arm_adj_echo, f2l((float)e));
                if (!_finite((double)prev) || e > (double)prev)
                    InterlockedExchange(&g_b.s_arm_adj_echo_worst,
                                        f2l((float)e));
                /* The same 5-degree bar joint 6's echo uses: one pair of
                   lag and the angle grid live below it by design. */
                if (e > 5.0) InterlockedIncrement(&g_b.c_arm_adj_echo_dirty);
            }
        }
    }
    /* clean_point, not point: on every pair after the first, point[4] is the
       wrist WE put there on the previous pair, so reading it here reports our
       own target back to us as though it were the animation's. It happened to
       be harmless - the only consumer left is the calibration pair, where the
       two are identical because nothing of ours is in the hierarchy yet - but
       the arm-basis log line claimed to show the animated wrist and was
       showing the driven one, and the offset anchor would have anchored on it
       the moment anything recalibrated mid-stream. point[2] is joint 4, which
       we rotate ABOUT rather than move, so that one never differed; it takes
       clean_point too, so a future change to what the removal touches cannot
       quietly reintroduce the same bug. */
    if (!arm_world_to_view(root, clean_point[2], shoulder_view) ||
        !arm_world_to_view(root, clean_point[4], wrist_view)) goto refuse_pair;

    for (k = 0; k < 3; k++) {
        InterlockedExchange(&g_b.s_arm_shoulder_view[k],
                            f2l((float)shoulder_view[k]));
        InterlockedExchange(&g_b.s_arm_native_wrist_view[k],
                            f2l((float)wrist_view[k]));
    }
    memset(&map_in, 0, sizeof map_in);
    map_in.anchor_shoulder =
        InterlockedCompareExchange(&g_b.arm_anchor, 0, 0) ? 1 : 0;
    map_in.player_reach = target->player_reach_view;
    for (k = 0; k < 3; k++) {
        map_in.controller_view[k] = ctrl_view_c[k];
        map_in.player_shoulder[k] = player_shoulder_c[k];
        map_in.shoulder_view[k] = shoulder_view[k];
        map_in.native_wrist_view[k] = wrist_view[k];
    }
    map_in.upper = vdist3(point[2], point[3]);
    map_in.fore = vdist3(point[3], point[4]);
    if(target->position.enabled && (target->persistent_hands || g_b.camera_position.ready)) {
        double camera_target[3];
        int ok=target->persistent_hands ? dg_hand_profile_position(&target->position,camera_target) :
            dg_position_target(&g_b.camera_position,&target->position,camera_target);
        if(!ok ||
           !arm_world_to_view(root,camera_target,map_in.desired_view)) goto refuse_pair;
        map_in.explicit_target=1;
    }
    if (!dg_arm_map_step(&g_b.arm_map, &map_in, &map_out)) {
        InterlockedExchange(&g_b.s_arm_map_flags, (LONG)map_out.flags);
        if (map_out.flags & DG_ARM_MAP_F_CALIBRATED) {
            g_b.arm_map_phase = 2;
            /* Nothing of ours is in the hierarchy on this pair - that is what
               the settle ticks are for - so the hand's rotation here is the
               animation's own, and the controller's is whatever the player
               happened to be holding. Both are stored in the root frame and
               only ever used as a pair. */
            g_b.hand_have_rest = 0;
            g_b.arm_root_have_q0 = 0;
            g_b.hand_have_last_cmd = 0;
            g_b.hand_have_prev_off = 0;
            g_b.arm_orient_have_prev = 0;
            g_b.hand_have_prev_twist = 0;
            g_b.hand_have_prev_swing = 0;
            InterlockedExchange(&g_b.hand_rest_stale, 0);
            g_b.arm_last_hand_tick = 0;
            g_b.hand_have_prev_base = 0;
            if (have_frames) {
                double rest[4], ctrl[4];
                /* And the root itself: the zero the compensation above
                   measures every later pair against. */
                for (k = 0; k < 4; k++) g_b.arm_root_q0[k] = root_q[k];
                g_b.arm_root_have_q0 = 1;
                quat_rebase(root_q_inv, live_hand, rest);
                if (unarmed_hand_rest(arm,root_q_inv,&g_b.pair_frame,rest) && g_b.log)
                    g_b.log("  hands: unarmed right wrist rest mirrored from native left\r\n");
                for (k = 0; k < 4; k++) ctrl[k] = target->hand_quat[k];
                if (dg_ik_quat_normalize(rest) &&
                    dg_ik_quat_normalize(ctrl)) {
                    for (k = 0; k < 4; k++) {
                        g_b.hand_rest_view[k] = rest[k];
                        g_b.hand_ctrl_rest[k] = ctrl[k];
                    }
                    g_b.hand_have_rest = 1;
                }
                /* The REST-freeze reference, captured here and nowhere
                   else, because this is the one pair where the hierarchy
                   provably contains nothing of ours (the settle ticks
                   exist to make that true). Stored in the ROOT frame so a
                   body turn is not mistaken for reference drift.
                   Unconditional - not gated on the freeze mode - so
                   switching vr_arm_freeze=rest on mid-run still uses a
                   clean capture instead of whatever contaminated pair it
                   happened to land on. */
                {
                    double up_w[3], fo_w[3];
                    for (k = 0; k < 3; k++) {
                        up_w[k] = point[3][k] - point[2][k];
                        fo_w[k] = point[4][k] - point[3][k];
                    }
                    arm_quat_rotate(root_q_inv, up_w, g_b.rest_ref_upper);
                    arm_quat_rotate(root_q_inv, fo_w, g_b.rest_ref_fore);
                    g_b.rest_ref_have = 1;
                }
                /* The head-yaw zero for the facing-gap publisher: the
                   calibration pair is the one moment the player is by
                   definition facing where they aim, so it defines
                   gap = 0. A head always has a heading, so unlike the
                   old controller-based zero this needs no horizontal
                   guard - it only needs the pose stream to have carried
                   a head at all. */
                capture_yaw_zeros(target);
                arm_reference(&g_b.right_reference,target,g_b.arm_root_q0,
                              g_b.hand_ctrl_rest,&g_b.stick_yaw0_deg);
            }
            InterlockedIncrement(&g_b.c_arm_pairs_calibration);
            return;
        }
        goto refuse_pair;
    }
    g_b.arm_map_phase = 2;
    InterlockedIncrement(&g_b.c_arm_pairs_eligible);
    if (map_out.flags & (DG_ARM_MAP_F_CLAMP_REACH |
                         DG_ARM_MAP_F_CLAMP_TOO_CLOSE))
        InterlockedIncrement(&g_b.c_arm_pairs_map_clamped);
    if (map_out.flags & DG_ARM_MAP_F_SOFT_REACH)
        InterlockedIncrement(&g_b.c_arm_pairs_map_soft);
    InterlockedExchange(&g_b.s_arm_map_flags, (LONG)map_out.flags);
    InterlockedExchange(&g_b.s_arm_map_scale, f2l((float)map_out.scale));
    InterlockedExchange(&g_b.s_arm_map_source_span,
                        f2l((float)map_out.source_span));
    InterlockedExchange(&g_b.s_arm_map_reach,
                        f2l((float)map_out.reach_used));
    for (k = 0; k < 3; k++) {
        InterlockedExchange(&g_b.s_arm_map_delta[k],
                            f2l((float)map_out.controller_delta[k]));
        InterlockedExchange(&g_b.s_arm_map_target[k],
                            f2l((float)map_out.target_view[k]));
    }

    for (k = 0; k < 3; k++) root_pos[k] = root[3][k];
    arm_view_to_world(root, map_out.target_view, target_world);
    if(target->position.enabled && !target->persistent_hands && !g_b.camera_position.ready &&
       !dg_position_calibrate(&g_b.camera_position,&target->position,
                              target_world,map_out.scale)) goto refuse_pair;
    /* And the other half of the kick: the hand driven back toward the
       shoulder. Always a shortening of the reach and never a lengthening -
       that is the physical direction, and it also means a kick can only ever
       make the target easier to reach. Recoil can therefore never be the thing
       that makes a pair implausible and drops a frame of hand control. */
    {
        double push = recoil_push_mm();
        if (push > 0.0) {
            if (dg_recoil_pull_back(target_world, root_pos, push))
                InterlockedIncrement(&g_b.c_recoil_push_writes);
            else
                InterlockedIncrement(&g_b.c_recoil_push_refused);
        }
    }
    plausible = arm_target_plausible(root_pos, point, target_world,
                                     &root_distance, &root_limit);
    for (k = 0; k < 3; k++) {
        InterlockedExchange(&g_b.s_arm_ik_root[k], f2l((float)root_pos[k]));
        InterlockedExchange(&g_b.s_arm_ik_target[k],
                            f2l((float)target_world[k]));
    }
    InterlockedExchange(&g_b.s_arm_ik_root_distance,
                        f2l((float)root_distance));
    InterlockedExchange(&g_b.s_arm_ik_root_limit, f2l((float)root_limit));
    /* DGREC4: the clean joints 3..6 this pass solved on and the target it
       chased, in world millimetres. The solver's own elbow and wrist and
       the published adjusts join them below. */
    for (k = 0; k < 3; k++) {
        int jj;
        for (jj = 0; jj < 4; jj++)
            g_b.rec_pair.joint_world[jj][k] = (float)clean_point[jj + 1][k];
        g_b.rec_pair.ik_target[k] = (float)target_world[k];
    }
    g_b.rec_pair.flags |= DG_REC_PAIR_F_ARM;
    if (!plausible) {
        InterlockedIncrement(&g_b.c_arm_ik_implausible);
        goto refuse_pair;
    }
    {
        /* REST freeze (vr_arm_freeze=rest). The frozen reference replaces
           the recovered one, and the angle between the two is measured
           first - that number IS the feedback loop, read directly: with
           the arm parked and the body still, a recovered reference that
           walks away from the frozen one is the strip residue
           accumulating, caught in the act. */
        double rest_ovr[2][3];
        const double (*ovr)[3] = NULL;
        DG_GRIP_ROLL_IN gr_in;
        memset(&gr_in, 0, sizeof gr_in);
        gr_in.on = InterlockedCompareExchange(&g_b.arm_uproll, 0, 0) ? 1 : 0;
        if (gr_in.on) {
            for (k = 0; k < 3; k++)
                gr_in.raw_fore[k] = point[4][k] - point[3][k];
            /* Slot read-back. Before first ownership the slots hold the
               engine's own identity, so identity IS the truth; once owned,
               only the live slots are - the strip's cache is deliberately
               not consulted here (it is documented to go dishonest when
               slots are lost or altered, and a twist about a mispredicted
               axis would move the wrist). */
            gr_in.slot_q4[3] = 1.0;
            gr_in.slot_q5[3] = 1.0;
            gr_in.have_readback = 1;
            if (InterlockedCompareExchange(&g_b.ik_active, 0, 0)) {
                gr_in.have_readback = 0;
                if (g_b.a.gm_player_arm_body) {
                    ULONGLONG uarm = *(volatile ULONGLONG *)(ULONG_PTR)
                        g_b.a.gm_player_arm_body;
                    if (plausible_ptr(uarm) && uarm == g_b.ik_owned_arm) {
                        ULONGLONG umc = *(volatile ULONGLONG *)(ULONG_PTR)
                            (uarm + 0x08);
                        if (plausible_ptr(umc) && umc == g_b.ik_owned_mctrl) {
                            ULONGLONG uadj = *(volatile ULONGLONG *)(ULONG_PTR)
                                (umc + 0x48);
                            if (plausible_ptr(uadj) &&
                                uadj == g_b.ik_owned_adjust) {
                                const volatile float *s4 =
                                    (const volatile float *)(ULONG_PTR)
                                    (uadj + 4 * 16);
                                const volatile float *s5 =
                                    (const volatile float *)(ULONG_PTR)
                                    (uadj + 5 * 16);
                                double n4 = 0.0, n5 = 0.0;
                                for (k = 0; k < 4; k++) {
                                    gr_in.slot_q4[k] = (double)s4[k];
                                    gr_in.slot_q5[k] = (double)s5[k];
                                    n4 += gr_in.slot_q4[k] * gr_in.slot_q4[k];
                                    n5 += gr_in.slot_q5[k] * gr_in.slot_q5[k];
                                }
                                if (_finite(n4) && _finite(n5) &&
                                    fabs(n4 - 1.0) < 1e-3 &&
                                    fabs(n5 - 1.0) < 1e-3)
                                    gr_in.have_readback = 1;
                            }
                        }
                    }
                }
            }
        }
        if (InterlockedCompareExchange(&g_b.arm_freeze, 0, 0) == 2 &&
            have_frames && g_b.rest_ref_have) {
            double live_up[3], live_fo[3], du, df, worst;
            arm_quat_rotate(root_q, g_b.rest_ref_upper, rest_ovr[0]);
            arm_quat_rotate(root_q, g_b.rest_ref_fore, rest_ovr[1]);
            ovr = rest_ovr;
            for (k = 0; k < 3; k++) {
                live_up[k] = clean_point[3][k] - clean_point[2][k];
                live_fo[k] = clean_point[4][k] - clean_point[3][k];
            }
            du = v_angle_deg(rest_ovr[0], live_up);
            df = v_angle_deg(rest_ovr[1], live_fo);
            worst = (du > df) ? du : df;
            if (_finite(worst) && worst >= 0.0) {
                float mx;
                g_b.rec_pair.rest_drift_deg = (float)worst;
                LONG cur = g_b.s_rest_drift_max;
                InterlockedExchange(&g_b.s_rest_drift, f2l((float)worst));
                memcpy(&mx, &cur, sizeof mx);
                if (target->weight >= 1.0 &&
                    (!_finite((double)mx) || (float)worst > mx))
                    InterlockedExchange(&g_b.s_rest_drift_max,
                                        f2l((float)worst));
            }
        }
        if (!solve_arm_adjust(&g_b.pair_frame, clean_point, target_world,
                              target->weight, ovr, &gr_in, q4, q5, &clamped))
            goto refuse_pair;
    }

    /* How much OUR OWN written rotation turns, accumulated. The 2026-08-21
       run settled that the tumble is ours (with vr_arm_track=off it stops
       dead) while the controller stayed calm and the roll anchor engaged on
       every pair - so the remaining question is whether the solution we
       write is itself winding. Totals rather than per-pair deltas for the
       reason that cost the whole day: the defect is smooth, under a degree
       a frame, and every instrument here that hunts for JUMPS called it
       calm while the arm turned fifteen times. Against this, the hand meter
       in the skeleton probe says whether the hierarchy turned more than we
       asked it to. */
    if (g_b.adj_turn_have_prev) {
        double d4 = dg_ik_quat_angle(q4, g_b.adj_turn_prev4) *
                    180.0 / 3.14159265358979323846;
        double d5 = dg_ik_quat_angle(q5, g_b.adj_turn_prev5) *
                    180.0 / 3.14159265358979323846;
        if (_finite(d4) && d4 >= 0.0 && d4 < 180.0 &&
            _finite(d5) && d5 >= 0.0 && d5 < 180.0) {
            float t4, t5;
            LONG v4 = g_b.s_adj_turned4, v5 = g_b.s_adj_turned5;
            memcpy(&t4, &v4, sizeof t4);
            memcpy(&t5, &v5, sizeof t5);
            if (!_finite((double)t4)) t4 = 0.0f;
            if (!_finite((double)t5)) t5 = 0.0f;
            InterlockedExchange(&g_b.s_adj_turned4, f2l(t4 + (float)d4));
            InterlockedExchange(&g_b.s_adj_turned5, f2l(t5 + (float)d5));
        }
    }
    for (k = 0; k < 4; k++) {
        g_b.adj_turn_prev4[k] = q4[k];
        g_b.adj_turn_prev5[k] = q5[k];
    }
    g_b.adj_turn_have_prev = 1;

    mctrl = *(volatile ULONGLONG *)(ULONG_PTR)(arm + 0x08);
    if (!plausible_ptr(mctrl)) goto refuse_pair;
    joints = RD32(mctrl + 0x14);
    adjust = *(volatile ULONGLONG *)(ULONG_PTR)(mctrl + 0x48);
    if (joints <= 5 || joints > 255 || !plausible_ptr(adjust)) goto refuse_pair;

    quat_identity(hand);
    if (target->hand_write && have_frames) {
        const volatile float *setpos =
            (const volatile float *)(ULONG_PTR)(adjust + 6 * 16);
        int all_zero = 1;
        int have_base = 0;
        double base_now[4];
        /* The gap detector. Hand pairs simply STOP for the whole of a
           cutscene or codec - whatever mixture of OFF and SUSPENDED the fps
           machine wandered through - so the stream's own silence is the one
           signal that always sees a scene change. A first pair (last tick
           0) measures nothing: there is no gap before a beginning. */
        {
            LONG now = InterlockedCompareExchange(&g_b.c_ticks, 0, 0);
            if (g_b.hand_have_rest && g_b.arm_last_hand_tick != 0 &&
                rest_stale_after_gap(g_b.arm_last_hand_tick, now))
                InterlockedExchange(&g_b.hand_rest_stale, 1);
            g_b.arm_last_hand_tick = now;
        }
        for (k = 0; k < 4; k++) {
            previous_hand_adjust[k] = (double)setpos[k];
            if (setpos[k] != 0.0f) all_zero = 0;
        }
        /* The residual measures the whole chain; this cuts it at the slot.
           What SetPos holds now is the game's own reconstruction of the
           previous pair's command, so a mismatch here is the game (or another
           writer) disagreeing with our model of it, before the hierarchy gets
           a say. One pair of lag and the 0.1-degree angle grid live in this
           number by design; the dirty threshold sits above both, so only a
           genuine disagreement can count. */
        if (!all_zero && g_b.hand_have_last_cmd) {
            double slot[4];
            for (k = 0; k < 4; k++) slot[k] = previous_hand_adjust[k];
            if (dg_ik_quat_normalize(slot)) {
                double echo = dg_ik_quat_angle(slot, g_b.hand_last_cmd) *
                              180.0 / 3.14159265358979323846;
                LONG worst = g_b.s_arm_hand_slot_echo_worst;
                float previous;
                memcpy(&previous, &worst, sizeof previous);
                InterlockedExchange(&g_b.s_arm_hand_slot_echo,
                                    f2l((float)echo));
                if (!_finite((double)previous) || echo > (double)previous)
                    InterlockedExchange(&g_b.s_arm_hand_slot_echo_worst,
                                        f2l((float)echo));
                if (echo > 5.0)
                    InterlockedIncrement(&g_b.c_arm_hand_slot_dirty);
            }
        }
        /* The animation base this pair actually stands on: live with our
           own three adjusts stripped back off, the same recovery the solve
           itself performs. Computed once here, consumed twice below. */
        {
            double aq[4], pu[4], pf[4], ph2[4], chain2[4], inv2[4];
            int okc = !all_zero && recover_cached;
            have_base = 0;
            if (okc) {
                for (k = 0; k < 4; k++)
                    aq[k] = (double)g_b.arm_map_cached_adjust[k];
                okc = okc && adjust_quat_to_world(&g_b.pair_frame, aq, pu);
                for (k = 0; k < 4; k++)
                    aq[k] = (double)g_b.arm_map_cached_adjust[4 + k];
                okc = okc && adjust_quat_to_world(&g_b.pair_frame, aq, pf);
                okc = okc && adjust_quat_to_world(&g_b.pair_frame,
                                                  previous_hand_adjust, ph2);
            }
            if (okc) {
                dg_ik_quat_mul(ph2, pf, chain2);
                dg_ik_quat_mul(chain2, pu, chain2);
                dg_ik_quat_conj(chain2, inv2);
                dg_ik_quat_mul(inv2, live_hand, base_now);
                have_base = dg_ik_quat_normalize(base_now);
            }
        }
        /* How fast that base MOVES between consecutive measured pairs. The
           post-cutscene tumble survives a fresh stream and a fresh rest
           (2026-08-20 night), so the open question is whether the base we
           read is itself coherent there: a calm base with a locked residual
           convicts the application, a wild base convicts the read. */
        if (have_base) {
            for (k = 0; k < 4; k++) g_b.rec_pair.base_now[k] = base_now[k];
            g_b.rec_pair.flags |= DG_REC_PAIR_F_BASE;
            if (g_b.hand_have_prev_base) {
                double bdrift = dg_ik_quat_angle(base_now,
                                                 g_b.hand_prev_base) *
                                180.0 / 3.14159265358979323846;
                LONG bworst = g_b.s_arm_base_drift_worst;
                float bprev;
                memcpy(&bprev, &bworst, sizeof bprev);
                InterlockedExchange(&g_b.s_arm_base_drift,
                                    f2l((float)bdrift));
                if (!_finite((double)bprev) || bdrift > (double)bprev)
                    InterlockedExchange(&g_b.s_arm_base_drift_worst,
                                        f2l((float)bdrift));
            }
            for (k = 0; k < 4; k++) g_b.hand_prev_base[k] = base_now[k];
            g_b.hand_have_prev_base = 1;
        } else {
            g_b.hand_have_prev_base = 0;
        }
        /* The automatic B-press. A long gap ended since the last pair:
           whatever the demo did to the character, the old rest pair
           describes a pose that no longer exists. Recapture it against the
           current animation base and the player's controller wherever it is
           held right now; the pair then solves against the fresh zero
           immediately. Conditions unmet leave the flag pending for the next
           pair rather than dropping it. */
        if (InterlockedCompareExchange(&g_b.hand_rest_stale, 0, 0) &&
            have_base && g_b.hand_have_rest) {
            double rest2[4], ctrl2[4];
            quat_rebase(root_q_inv, base_now, rest2);
            unarmed_hand_rest(arm,root_q_inv,&g_b.pair_frame,rest2);
            for (k = 0; k < 4; k++) ctrl2[k] = target->hand_quat[k];
            if (dg_ik_quat_normalize(rest2) &&
                dg_ik_quat_normalize(ctrl2)) {
                for (k = 0; k < 4; k++) {
                    g_b.hand_rest_view[k] = rest2[k];
                    g_b.hand_ctrl_rest[k] = ctrl2[k];
                    g_b.arm_root_q0[k] = root_q[k];
                }
                g_b.arm_root_have_q0 = 1;
                capture_yaw_zeros(target);
                arm_reference(&g_b.right_reference,target,g_b.arm_root_q0,
                              g_b.hand_ctrl_rest,&g_b.stick_yaw0_deg);
                g_b.hand_have_prev_twist = 0;
                g_b.hand_have_prev_swing = 0;
                g_b.hand_have_prev_off = 0;
                g_b.hand_have_last_cmd = 0;
                InterlockedExchange(&g_b.hand_rest_stale, 0);
                InterlockedIncrement(&g_b.c_arm_rest_recaptured);
            }
        }
        if (all_zero) {
            InterlockedIncrement(&g_b.c_arm_hand_no_setpos);
        } else {
            double unarmed_desired[4];
            const double *hand_target = target->absolute_aim ? target->aim_world : NULL;
            if (g_b.arm_map_unarmed) {
                if (!unarmed_wrist_target(arm,&g_b.pair_frame,q4,q5,
                                          recover_cached,unarmed_desired)) goto refuse_pair;
                hand_profile_stage(1,target,&target->free_right,unarmed_desired);
                if(target->free_right.enabled &&
                   !dg_free_wrist_step(&g_b.free_right,&target->free_right,
                                       unarmed_desired,unarmed_desired)) goto refuse_pair;
                hand_target=unarmed_desired;
            }
            have_hand = arm_hand_solve(hand_root_q, root_q, live_hand, q4, q5,
                                       previous_hand_adjust,
                                       target->hand_quat, recover_cached,
                                       target->absolute_aim ?
                                         fmin(target->weight, target->aim_weight) : target->weight,
                                       hand, desired,
                                       hand_target);
            if (have_hand &&
                !arm_hand_stabilize(clean_point, q4, q5, hand, desired,
                                    recoil_climb_rad(), &hand_stable))
                have_hand = 0;
        }
        /* The channel's reach (run 13, 2026-09-03). GV_NearExp4PV pulls the
           shift SVECTOR towards zero along the SHORT ARC of the 4096-unit
           turn, so a precompensated short past a half turn is folded before
           it is pulled and the engine lands three quarters of a turn from
           the ask: with the swing cap at 150 the hand asked (1820, 658,
           1303), we wrote 2426 for the roll, the engine read it as -1670,
           pulled it to -1253 and the hand sat 90 degrees from the published
           one for whole episodes (278 dirty slot echoes, residual locked at
           89.9). Nothing past 1536 units per component can be delivered in
           one pull, so the triple is renamed to the second ZYX solution
           when that fits and otherwise a nearby reachable adjust is chosen
           (radial shortening jumped at alternate-chart boundaries). The
           PROJECTED rotation is what is
           stored as the command echo and published as the desired hand, so
           the residual and the slot echo keep judging what actually left. */
        if (have_hand) {
            double fitted[4], fraction = 1.0;
            g_b.rec_pair.flags |= DG_REC_PAIR_F_CHANNEL_PROJECT;
            if (!dg_ik_ps_project(hand, dg_ik_ps_reach_units(DG_HAND_PULL_DIVISOR),
                              fitted, wanted_rot, &ps_flags, &fraction)) {
                have_hand = 0;
            } else if (ps_flags & DG_IK_PS_PROJECTED) {
                /* achieved = raw * base (the stabilize identity): keep the
                   base, replace the raw by the fitted rotation. */
                double w_raw[4], w_fit[4], inv[4], base[4], bounded[4];
                if (adjust_quat_to_world(&g_b.pair_frame, hand, w_raw) &&
                    adjust_quat_to_world(&g_b.pair_frame, fitted, w_fit)) {
                    dg_ik_quat_conj(w_raw, inv);
                    dg_ik_quat_mul(inv, desired, base);
                    dg_ik_quat_mul(w_fit, base, bounded);
                    if (dg_ik_quat_normalize(bounded)) {
                        float shortfall = (float)(1.0 - fraction), sworst;
                        LONG sv = g_b.s_arm_hand_shortfall_worst;
                        for (k = 0; k < 4; k++) {
                            hand[k] = fitted[k];
                            desired[k] = bounded[k];
                        }
                        InterlockedIncrement(&g_b.c_arm_hand_scaled);
                        g_b.rec_pair.fit_frac = (float)fraction;
                        InterlockedExchange(&g_b.s_arm_hand_fit,
                                            f2l((float)fraction));
                        memcpy(&sworst, &sv, sizeof sworst);
                        if (!_finite((double)sworst) || shortfall > sworst)
                            InterlockedExchange(&g_b.s_arm_hand_shortfall_worst,
                                                f2l(shortfall));
                    } else {
                        have_hand = 0;
                    }
                } else {
                    have_hand = 0;
                }
            } else {
                InterlockedExchange(&g_b.s_arm_hand_fit, f2l(1.0f));
                g_b.rec_pair.fit_frac = 1.0f;
            }
            if (have_hand && (ps_flags & DG_IK_PS_ALT_NAMING))
                InterlockedIncrement(&g_b.c_arm_hand_alt_named);
        }
        if (have_hand &&
            dg_ik_ps_precompensate(wanted_rot, DG_HAND_PULL_DIVISOR,
                                   command_rot) &&
            hand_command_publish(command_rot, target->pair_id)) {
            for (k = 0; k < 4; k++) g_b.hand_desired[k] = desired[k];
            g_b.hand_have_desired = 1;
            for (k = 0; k < 4; k++) {
                g_b.rec_pair.demand[k] = hand[k];
                g_b.rec_pair.desired[k] = desired[k];
            }
            g_b.rec_pair.flags |= DG_REC_PAIR_F_PUBLISHED;
            /* How far this command moved from the previous one - measured
               BEFORE the store below overwrites it. The one number that
               separates "the game's animation flaps under us" from "we are
               publishing the flapping ourselves": a calm command stream
               with a wild base convicts the game, a flapping command
               stream is ours whatever the base seems to do (the base strip
               is phase-confounded exactly when commands flap). */
            if (g_b.hand_have_last_cmd) {
                double cdrift = dg_ik_quat_angle(hand, g_b.hand_last_cmd) *
                                180.0 / 3.14159265358979323846;
                LONG cworst = g_b.s_arm_cmd_drift_worst;
                float cprev;
                memcpy(&cprev, &cworst, sizeof cprev);
                InterlockedExchange(&g_b.s_arm_cmd_drift,
                                    f2l((float)cdrift));
                if (!_finite((double)cprev) || cdrift > (double)cprev)
                    InterlockedExchange(&g_b.s_arm_cmd_drift_worst,
                                        f2l((float)cdrift));
            }
            /* The slot's expected content next pair: the adjust quaternion
               whose PS2 angles were just published. Stored after the
               stabilize pass, because those are the angles that left. */
            for (k = 0; k < 4; k++) g_b.hand_last_cmd[k] = hand[k];
            g_b.hand_have_last_cmd = 1;
            InterlockedIncrement(&g_b.c_arm_hand_written);
            if (hand_stable.flags & DG_IK_HAND_F_FORE_TWIST)
                InterlockedIncrement(&g_b.c_arm_hand_fore_twist);
            if (hand_stable.flags & (DG_IK_HAND_F_SWING_LIMITED |
                                     DG_IK_HAND_F_TWIST_LIMITED))
                InterlockedIncrement(&g_b.c_arm_hand_limited);
            g_b.rec_pair.raw_twist_deg = (float)(hand_stable.raw_twist_rad *
                                                 180.0 / 3.14159265358979323846);
            g_b.rec_pair.raw_swing_deg = (float)(hand_stable.raw_swing_rad *
                                                 180.0 / 3.14159265358979323846);
            g_b.rec_pair.flags |= DG_REC_PAIR_F_ENVELOPE;     /* DGREC4 */
            InterlockedExchange(&g_b.s_arm_hand_raw_twist,
                f2l((float)(hand_stable.raw_twist_rad *
                            180.0 / 3.14159265358979323846)));
            InterlockedExchange(&g_b.s_arm_hand_fore_twist,
                f2l((float)(hand_stable.fore_twist_rad *
                            180.0 / 3.14159265358979323846)));
            InterlockedExchange(&g_b.s_arm_hand_wrist_swing,
                f2l((float)(hand_stable.wrist_swing_rad *
                            180.0 / 3.14159265358979323846)));
            InterlockedExchange(&g_b.s_arm_hand_wrist_twist,
                f2l((float)(hand_stable.wrist_twist_rad *
                            180.0 / 3.14159265358979323846)));
        } else {
            hand_command_clear();
            InterlockedIncrement(&g_b.c_arm_hand_refused);
        }
    } else {
        hand_command_clear();
    }

    if (InterlockedCompareExchange(&g_b.bent_joint, 0, 0) >= 0)
        arm_bend_release();
    mask = (1ULL << 4) | (1ULL << 5);
    for (j = 4; j <= 5; j++) {
        const double *src = (j == 4) ? q4 : q5;
        volatile float *dst = (volatile float *)(ULONG_PTR)(adjust + j * 16);
        for (k = 0; k < 4; k++) {
            float v = (float)src[k];
            dst[k] = v;
            g_b.arm_map_cached_adjust[(j - 4) * 4 + k] = v;
        }
    }
    {   /* DGREC4: the adjusts as published, in world. */
        double w4r[4], w5r[4];
        if (adjust_quat_to_world(&g_b.pair_frame, q4, w4r) &&
            adjust_quat_to_world(&g_b.pair_frame, q5, w5r)) {
            for (k = 0; k < 4; k++) {
                g_b.rec_pair.q4_world[k] = (float)w4r[k];
                g_b.rec_pair.q5_world[k] = (float)w5r[k];
            }
        }
    }
    *(volatile ULONGLONG *)(ULONG_PTR)(mctrl + 0x38) |= mask;
    *(volatile ULONGLONG *)(ULONG_PTR)(mctrl + 0x38) &=
        ~(g_b.ik_owned_mask & ~mask);
    /* The wrist this pair's written solution predicts, under this pair's
       frame and the hierarchy model the strip rests on (the forearm
       inherits q4 then q5): joint 4 as measured, the stripped upper bone
       turned by W4, the stripped forearm by W4 then W5. The next pair's
       read grades it. Why this and not the design's axis-versus-bone
       angle: both the strip and the roll's peel use the SAME frame, so a
       wrong frame moves both together and an angle between them reads 0
       through exactly the failure it was meant to catch; the wrist's world
       position is graded by the engine itself and cannot agree with a
       wrong frame. */
    g_b.have_pred_wrist = 0;
    {
        double W4[4], W5[4], up[3], fo[3], t1[3], t2[3], t3[3];
        if (target->weight >= 1.0 &&
            adjust_quat_to_world(&g_b.pair_frame, q4, W4) &&
            adjust_quat_to_world(&g_b.pair_frame, q5, W5)) {
            for (k = 0; k < 3; k++) {
                up[k] = clean_point[3][k] - clean_point[2][k];
                fo[k] = clean_point[4][k] - clean_point[3][k];
            }
            arm_quat_rotate(W4, up, t1);
            arm_quat_rotate(W4, fo, t2);
            arm_quat_rotate(W5, t2, t3);
            for (k = 0; k < 3; k++)
                g_b.pred_wrist[k] = point[2][k] + t1[k] + t3[k];
            g_b.have_pred_wrist = 1;
        }
    }
    g_b.ik_owned_mask = mask;
    g_b.ik_owned_arm = arm;
    g_b.ik_owned_mctrl = mctrl;
    g_b.ik_owned_adjust = adjust;
    g_b.arm_map_cache_valid = 1;
    /* The skew meter (V5.1 par. 4.3/4.6): how far the heading moved between
       the frame acquisition and this commit, the bound the snapshot claim
       rests on - measured, not asserted. */
    if (g_b.pair_rot_ok) {
        ULONGLONG pw = 0, pe = 0;
        if (resolve_motion_player(NULL, &pw, NULL) == DG_RESOLVE_OK)
            pe = region_end(pw);
        if (pw && pe && pw + 0x84 <= pe) {
            short now_vy = *(volatile short *)(ULONG_PTR)(pw + 0x82);
            int d = (int)now_vy - (int)g_b.pair_rot_vy;
            float skew, worst;
            LONG cur;
            d = ((d + 2048) & 4095) - 2048;
            skew = (float)(d < 0 ? -d : d) * (360.0f / 4096.0f);
            cur = g_b.s_frame_skew_worst;
            memcpy(&worst, &cur, sizeof worst);
            if (!_finite((double)worst) || skew > worst)
                InterlockedExchange(&g_b.s_frame_skew_worst, f2l(skew));
        }
    }
    InterlockedExchange(&g_b.ik_active, 1);
    InterlockedIncrement(&g_b.c_arm_bent);
    InterlockedIncrement(&g_b.c_arm_tracked);
    InterlockedIncrement(&g_b.c_arm_pairs_accepted);
    if (clamped) InterlockedIncrement(&g_b.c_arm_ik_clamped);
    return;

refuse_pair:
    InterlockedIncrement(&g_b.c_arm_pairs_refused);
    InterlockedIncrement(&g_b.c_arm_ik_refused);
    arm_map_forget();
    return;

refuse_unpaired:
    InterlockedIncrement(&g_b.c_arm_ik_refused);
    arm_map_forget();
}

#include "dg_left_arm.inl"
#include "dg_hand_pose.inl"
#include "dg_unarmed_rest.inl"

/* ------------------------------------------- F5 step 3: the adjust space --- */

/* Alternate identity and a known rotation on one joint, and report the world
   matrices that came out. Read-then-write, in that order, because the camera
   seam runs AFTER the hierarchy pass: a quaternion written here is consumed by
   the next frame's pass, so what is readable now is the result of what was
   written last time. Pairing a matrix with this frame's input instead of last
   frame's is the one error that would produce a confident wrong answer, which
   is why the case index is carried across frames rather than recomputed. */
/* The heading words for Meetplan A' (ROLL_ONTWERP_V5.1 par. 3), stored beside
   the case they bracket. An unresolvable player leaves ok = 0 and the words
   at 0, so a run with no player never fabricates a heading. */
static void adj_probe_words(int c, int phase)
{
    ULONGLONG pwork = 0, pend = 0;
    LONG *w = (LONG *)&g_b.adj_words[(c * 2 + phase) * 5];
    LONG tick = InterlockedCompareExchange(&g_b.c_ticks, 0, 0);
    LONG rot = 0, turn = 0, cam = 0, ok = 0;
    if (c < 0 || c >= DG_ADJ_CASES) return;
    if (resolve_player(NULL, &pwork, NULL) == DG_RESOLVE_OK)
        pend = region_end(pwork);
    if (pwork && pend && pwork + 0xD28 <= pend) {
        rot = *(volatile short *)(ULONG_PTR)(pwork + 0x82);
        turn = *(volatile short *)(ULONG_PTR)(pwork + 0x8A);
        cam = *(volatile short *)(ULONG_PTR)(pwork + 0xD22);
        ok = 1;
    }
    InterlockedExchange(&w[0], tick);
    InterlockedExchange(&w[1], ok);
    InterlockedExchange(&w[2], rot);
    InterlockedExchange(&w[3], turn);
    InterlockedExchange(&w[4], cam);
}

/* Reduce one closed cycle. Conventions as derived on 2026-08-30 (recordings/
   adjust_probe_blokken_2026-08-30.log): the logged rows S are the world
   images of the local axes (column convention W = S^T), so the world-side
   left delta a case applied to the identity case is L = S_c^T . S_0, and
   its rotation axis is the antisymmetric part of L. The X case's axis is
   the world image of adjust-X; its heading is what rot.vy is fitted to. */
static void adj_cycle_note(void)
{
    double S[DG_ADJ_CASES][3][3];
    double axis[3][3], ang[3];
    DG_ADJ_CYCLE *s;
    LONG w = g_b.adj_cycle_w;
    int c, r, i, k;

    if (w - g_b.adj_cycle_r >= DG_ADJ_CYCLE_RING) return;
    for (c = 0; c < DG_ADJ_CASES; c++)
        for (r = 0; r < 3; r++)
            for (i = 0; i < 3; i++) {
                LONG v = g_b.adj_world[(c * DG_ADJ_JOINTS + 2) * 16 + r * 4 + i];
                float f;
                memcpy(&f, &v, sizeof f);
                S[c][r][i] = f;
            }
    for (c = 1; c < DG_ADJ_CASES; c++) {
        double L[3][3], tr, n;
        for (i = 0; i < 3; i++)
            for (r = 0; r < 3; r++) {
                L[i][r] = 0.0;
                for (k = 0; k < 3; k++) L[i][r] += S[c][k][i] * S[0][k][r];
            }
        tr = (L[0][0] + L[1][1] + L[2][2] - 1.0) * 0.5;
        if (tr > 1.0) tr = 1.0;
        if (tr < -1.0) tr = -1.0;
        ang[c - 1] = acos(tr) * 180.0 / 3.14159265358979323846;
        axis[c - 1][0] = L[2][1] - L[1][2];
        axis[c - 1][1] = L[0][2] - L[2][0];
        axis[c - 1][2] = L[1][0] - L[0][1];
        n = sqrt(axis[c - 1][0] * axis[c - 1][0] + axis[c - 1][1] * axis[c - 1][1] +
                 axis[c - 1][2] * axis[c - 1][2]);
        if (n > 1.0e-9)
            for (i = 0; i < 3; i++) axis[c - 1][i] /= n;
        else
            axis[c - 1][0] = axis[c - 1][1] = axis[c - 1][2] = 0.0;
    }
    s = &g_b.adj_cycle_ring[w % DG_ADJ_CYCLE_RING];
    memset(s, 0, sizeof *s);
    s->tick = InterlockedCompareExchange(&g_b.c_ticks, 0, 0);
    s->rot0 = (short)g_b.adj_words[(0 * 2 + 1) * 5 + 2];
    s->rot3 = (short)g_b.adj_words[((DG_ADJ_CASES - 1) * 2 + 1) * 5 + 2];
    s->ok = (short)(g_b.adj_words[(0 * 2 + 1) * 5 + 1] &&
                    g_b.adj_words[((DG_ADJ_CASES - 1) * 2 + 1) * 5 + 1]);
    s->yaw_deg = (float)(atan2(axis[0][2], axis[0][0]) * 180.0 /
                         3.14159265358979323846);
    for (c = 0; c < 3; c++) s->ang_deg[c] = (float)ang[c];
    s->ydot = (float)axis[1][1];
    InterlockedIncrement(&g_b.adj_cycle_w);
}

int dg_bridge_adj_cycle_take(DG_ADJ_CYCLE *out)
{
    LONG r = g_b.adj_cycle_r;
    if (r >= InterlockedCompareExchange(&g_b.adj_cycle_w, 0, 0)) return 0;
    *out = g_b.adj_cycle_ring[r % DG_ADJ_CYCLE_RING];
    InterlockedIncrement(&g_b.adj_cycle_r);
    return 1;
}

static void adj_probe_now(ULONGLONG arm)
{
    ULONGLONG objs, mctrl, adjust, base, end;
    LONG joint, stride, count, pending, deg, axis, joints, rel;
    LONG now, last;
    int idx[DG_ADJ_JOINTS];
    int i, k;
    double q[4], rad;

    if (!InterlockedCompareExchange(&g_b.adj_probe, 0, 0)) return;

    /* Leans on the skeleton probe: the parent index is what makes the delta
       solvable at all, and only that probe knows the topology. Refusing here
       rather than assuming 3/4/5 keeps the two measurements honest about
       depending on each other. */
    stride = InterlockedCompareExchange(&g_b.skel_stride, 0, 0);
    count = InterlockedCompareExchange(&g_b.skel_parents_read, 0, 0);
    rel = InterlockedCompareExchange(&g_b.skel_region_end, 0, 0);
    if (!stride || count <= 0 || !rel) return;

    joint = InterlockedCompareExchange(&g_b.adj_joint, 0, 0);
    if (joint <= 0 || joint >= count) return;

    idx[2] = joint;
    idx[1] = (int)InterlockedCompareExchange(&g_b.skel_parents[joint], 0, 0);
    idx[3] = -1;
    for (i = 0; i < count; i++) {
        if ((int)InterlockedCompareExchange(&g_b.skel_parents[i], 0, 0) == joint) {
            idx[3] = i;
            break;
        }
    }
    /* The rig root: walked, not assumed to be joint 0. The live arm happens to
       have parent[0] == -1, but a walk that ends where the parent chain ends is
       right for any rig and says so if the chain is broken. */
    idx[0] = joint;
    for (i = 0; i < count; i++) {
        int p = (int)InterlockedCompareExchange(&g_b.skel_parents[idx[0]], 0, 0);
        if (p < 0 || p >= count || p == idx[0]) break;
        idx[0] = p;
    }
    if (i >= count) return;                     /* a cycle: trust nothing */
    if (idx[1] < 0 || idx[1] >= count || idx[3] < 0) return;

    objs = *(volatile ULONGLONG *)(ULONG_PTR)arm;
    if (!plausible_ptr(objs)) return;
    end = objs + (ULONGLONG)(DWORD)rel;
    base = objs + DG_OBJS_ARRAY;

    /* One case per hierarchy pass, not one per seam call. The camera seam fires
       several times per tick in first person, so without this the same pose is
       read into both slots and the difference the whole measurement rests on is
       zero. DG_ADJ_SETTLE_TICKS carries the reasoning and the margin. */
    pending = InterlockedCompareExchange(&g_b.adj_pending_case, 0, 0);
    now = InterlockedCompareExchange(&g_b.c_ticks, 0, 0);
    last = InterlockedCompareExchange(&g_b.adj_write_tick, 0, 0);
    if (pending >= 0 &&
        (LONG)((DWORD)now - (DWORD)last) < DG_ADJ_SETTLE_TICKS) {
        InterlockedIncrement(&g_b.c_adj_held);
        return;
    }

    /* READ first, into the slot for the case written last frame. */
    if (pending >= 0 && pending < DG_ADJ_CASES) {
        for (k = 0; k < DG_ADJ_JOINTS; k++) {
            ULONGLONG m = base + (ULONGLONG)idx[k] * (ULONGLONG)stride;
            const volatile float *f;
            if (m + 0x40 > end) return;
            f = (const volatile float *)(ULONG_PTR)m;
            for (i = 0; i < 16; i++)
                InterlockedExchange(
                    &g_b.adj_world[(pending * DG_ADJ_JOINTS + k) * 16 + i],
                    f2l(f[i]));
            InterlockedExchange(&g_b.adj_indices[k], (LONG)idx[k]);
        }
        /* The heading words at the READ, beside the matrices they date. */
        adj_probe_words(pending, 1);
        InterlockedIncrement(&g_b.adj_samples[pending]);
        /* The Z read closes a cycle: reduce it while all four cases are
           the ones this cycle wrote. */
        if (pending == DG_ADJ_CASES - 1) adj_cycle_note();
    }

    /* ...then WRITE the other case, which the next frame's pass will apply. */
    mctrl = *(volatile ULONGLONG *)(ULONG_PTR)(arm + 0x08);
    if (!plausible_ptr(mctrl)) return;
    joints = RD32(mctrl + 0x14);
    if (joints < 21 || joints > 255 || joint >= joints) return;
    adjust = *(volatile ULONGLONG *)(ULONG_PTR)(mctrl + 0x48);
    if (!plausible_ptr(adjust)) return;

    /* Cycle identity, X, Y, Z. One axis measures a third of the mapping and
       leaves the rest to inference; three measure all of it, and the run is the
       costly part here, not the twelve extra matrices. */
    pending = (pending + 1) % DG_ADJ_CASES;
    if (pending < 0) pending = 0;
    q[0] = q[1] = q[2] = 0.0;
    q[3] = 1.0;
    if (pending > 0) {
        axis = pending - 1;
        deg = InterlockedCompareExchange(&g_b.adj_deg, 0, 0);
        rad = (double)deg * 3.14159265358979323846 / 180.0;
        q[axis] = sin(rad * 0.5);
        q[3] = cos(rad * 0.5);
    }
    for (i = 0; i < 4; i++)
        InterlockedExchange(&g_b.adj_quat[pending * 4 + i], f2l((float)q[i]));
    /* ...and the heading words at the WRITE, the other end of the bracket. */
    adj_probe_words(pending, 0);
    {
        volatile float *w = (volatile float *)(ULONG_PTR)(adjust + joint * 16);
        w[0] = (float)q[0];
        w[1] = (float)q[1];
        w[2] = (float)q[2];
        w[3] = (float)q[3];
    }
    /* The bit stays set for BOTH cases. Identity with the bit set is what an
       untouched joint should look like, so case 0 doubles as a control on the
       write path itself: if it differs from the unperturbed pose, the problem
       is the write, not the convention. */
    *(volatile ULONGLONG *)(ULONG_PTR)(mctrl + 0x38) |= (1ULL << joint);
    InterlockedExchange(&g_b.adj_pending_case, pending);
    InterlockedExchange(&g_b.adj_write_tick, now);
}

static void count_active_seam(void)
{
    LONG tick = InterlockedCompareExchange(&g_b.c_ticks, 0, 0);
    LONG last = InterlockedExchange(&g_b.seam_active_last_tick, tick);
    InterlockedIncrement(&g_b.c_seam_active);
    if (last != tick) InterlockedIncrement(&g_b.c_seam_active_ticks);
}

/* The theater judgment, stepped once per camera-seam status sample - i.e.
   once per frame, on the one seam that keeps running through a demo. Counters
   first (phase 0 is measurement and must not depend on the mode), then the
   two hysteresis machines, then ONE publication that every consumer reads:
   apply_transform, Present and the XR thread all see the same verdict for a
   frame instead of judging per-eye or per-thread. Runs even with the mode
   OFF, because it is passive arithmetic on words this seam already read -
   only the flank LOG is mode-gated, so a default session's log is today's. */
static void theater_seam_judge(unsigned int game, unsigned int menu,
                               ULONGLONG player)
{
    int masked = dg_theater_masked(game, menu);
    int ui_masked = dg_ui_panel_masked(menu);
    int was = g_b.thea_demo.active;
    int ui_was = g_b.thea_ui.active;
    int now, ui_now;
    LONG mode = InterlockedCompareExchange(&g_b.theater_mode, 0, 0);

    InterlockedIncrement(&g_b.c_thea_samples);
    if (masked) InterlockedIncrement(&g_b.c_thea_masked);
    if (ui_masked) InterlockedIncrement(&g_b.c_thea_ui_masked);
    if (game & 0x10000000UL) InterlockedIncrement(&g_b.c_thea_bit_demo);
    if (game & 0x08000000UL) InterlockedIncrement(&g_b.c_thea_bit_scn);
    if (game & 0x40000000UL) InterlockedIncrement(&g_b.c_thea_bit_pad);
    if (menu & 0x00000400UL) InterlockedIncrement(&g_b.c_thea_bit_radio);
    if (menu & 0x00000100UL) InterlockedIncrement(&g_b.c_thea_bit_weapon);
    if (menu & 0x00000200UL) InterlockedIncrement(&g_b.c_thea_bit_item);

    now = dg_theater_hyst_step(&g_b.thea_demo, masked,
                               DG_THEATER_ENTER_SAMPLES,
                               DG_THEATER_EXIT_SAMPLES);
    ui_now = dg_theater_hyst_step(&g_b.thea_ui, ui_masked,
                                  DG_UI_ENTER_SAMPLES, DG_UI_EXIT_SAMPLES);
    /* Held-off samples, counted across both machines: a session where the
       hysteresis never had to hold anything back and one where it was the
       only thing standing between a blinking bit and a flapping screen must
       not read the same. */
    if ((masked && !now) || (ui_masked && !ui_now))
        InterlockedIncrement(&g_b.c_thea_enter_held);
    if ((!masked && now) || (!ui_masked && ui_now))
        InterlockedIncrement(&g_b.c_thea_exit_held);

    if (now != was) {
        InterlockedIncrement(now ? &g_b.c_thea_enter : &g_b.c_thea_exit);
        if (mode != DG_THEATER_OFF)
            thea_ring_push((unsigned)(DG_THEATER_V_DEMO | (now ? 0 : 0x100)),
                           game, menu, player);
    }
    if (ui_now != ui_was) {
        InterlockedIncrement(ui_now ? &g_b.c_thea_ui_enter
                                    : &g_b.c_thea_ui_exit);
        if (mode != DG_THEATER_OFF ||
            InterlockedCompareExchange(&g_b.theater_ui_mode, 0, 0))
            thea_ring_push((unsigned)(DG_THEATER_V_UI | (ui_now ? 0 : 0x100)),
                           game, menu, player);
    }

    /* Verdict, then the stamp, in that order - the same rule as the late
       latch above: a consumer can never see a fresh stamp over a verdict
       that is still half written. */
    InterlockedExchange(&g_b.s_theater,
                        (now ? DG_THEATER_V_DEMO : 0) |
                        (ui_now ? DG_THEATER_V_UI : 0));
    InterlockedExchange(&g_b.theater_ms, (LONG)GetTickCount());
}

/* U2: one frame of synthesized front-end input, published for the pad
   detour to consume. Same shape as dg_bridge_move_now: the caller decides,
   this only carries, and a NULL withdraws. */
int dg_bridge_menu_gameover_now(void)
{
    unsigned game,menu;
    if(!g_b.armed || !g_b.a.gm_game_status || !g_b.a.gm_game_status_scn ||
       !g_b.a.gm_menu_status || !g_b.a.gm_menu_status_scn)return 0;
    game=(unsigned)RD32(g_b.a.gm_game_status)|(unsigned)RD32(g_b.a.gm_game_status_scn);
    menu=(unsigned)RD32(g_b.a.gm_menu_status)|(unsigned)RD32(g_b.a.gm_menu_status_scn);
    return (game&0x80004000u)!=0 && !(game&(DG_GAME_UNSAFE_MASK&~0x80004000u)) &&
           !(menu&DG_MENU_UNSAFE_MASK);
}
static int xr_menu_context_now(void)
{
    unsigned game;
    if(!g_b.a.gm_game_status || !g_b.a.gm_game_status_scn)return 0;
    game=(unsigned)RD32(g_b.a.gm_game_status)|(unsigned)RD32(g_b.a.gm_game_status_scn);
    if(game&0x80004000u)return dg_bridge_menu_gameover_now()?2:0;
    /* The animated title backdrop sets CUT_IN|PAUSE_DISABLE (0x240).
       Only the independently validated no-arm frontend may ignore CUT_IN;
       gameplay and demos retain their original safety masks. */
    if(dg_bridge_menu_context_ready() && !(game&(DG_GAME_UNSAFE_MASK&~0x40u)))return 1;
    return 3; /* Other existing flat menus retain their configured buttons. */
}
void dg_bridge_menu_now(const DG_BRIDGE_MENU *cmd)
{
    if (!g_b.script_menu_only && cmd && cmd->status &&
        (cmd->allow==DG_MENU_ALLOW_XR || cmd->allow==DG_MENU_ALLOW_XR_CONFIRM)) {
        LONG epoch=InterlockedCompareExchange(&g_script_menu_cancel,0,0);
        AcquireSRWLockExclusive(&g_script_menu_lock);
        g_script_menu_pending=*cmd;
        g_script_menu_pending.status &= DG_MENU_PAD_ALLOWED;
        g_script_menu_pending.clear &= DG_MENU_PAD_ALLOWED;
        g_script_menu_deadline=GetTickCount64()+100;
        g_script_menu_pending_epoch=epoch;
        g_xr_menu_context=xr_menu_context_now();
        ReleaseSRWLockExclusive(&g_script_menu_lock);
        return;
    }
    if (InterlockedCompareExchange(&g_b.script_menu_only, 0, 0)) {
        LONG epoch = InterlockedCompareExchange(&g_script_menu_cancel, 0, 0);
        if (!cmd || !cmd->allow || !cmd->status || !dg_bridge_menu_context_ready()) {
            script_menu_clear(0);
            return;
        }
        AcquireSRWLockExclusive(&g_script_menu_lock);
        g_script_menu_pending = *cmd;
        g_script_menu_pending.status &= DG_MENU_PAD_ALLOWED;
        g_script_menu_pending.clear &= DG_MENU_PAD_ALLOWED;
        g_script_menu_deadline = GetTickCount64() + 100;
        g_script_menu_pending_epoch = epoch;
        ReleaseSRWLockExclusive(&g_script_menu_lock);
        return;
    }
    if (!cmd || !cmd->allow || !cmd->status) {
        script_menu_clear(0);
        InterlockedExchange(&g_b.menu_status, 0);
        InterlockedExchange(&g_b.menu_clear, 0);
        InterlockedExchange(&g_b.menu_allow, 0);
        return;
    }
    /* Bounded by the same allowed set as the bits we add: the marker may
       decide WHICH competing button to silence, never reach outside the
       pad's own buttons to do it. */
    InterlockedExchange(&g_b.menu_clear,
                        (LONG)(cmd->clear & DG_MENU_PAD_ALLOWED));
    /* Only bits the module is allowed to name ever reach the game. The
       marker cannot widen this and neither can a bug upstream: a status
       word is a bag of bits and some of them are not menu buttons. */
    InterlockedExchange(&g_b.menu_status,
                        (LONG)((cmd->status & DG_MENU_PAD_ALLOWED) |
                        (cmd->allow == DG_CODEC_MENU_ALLOW_EXIT ? DG_CODEC_MENU_REQUEST : 0u)));
    if (cmd->allow == DG_CODEC_MENU_ALLOW_EXIT)
        InterlockedIncrement(&g_codec_exit_requested);
    InterlockedExchange(&g_b.menu_allow, 1);
    InterlockedExchange(&g_b.menu_stamp,
                        InterlockedCompareExchange(&g_b.c_ticks, 0, 0));
}

static void interact_codec_seam(void)
{
    LONG64 stamp;
    uint64_t now=GetTickCount64();
    unsigned game,menu;
    ULONGLONG player,rec;
    if (!TryAcquireSRWLockShared(&g_controls_lock)) return;
    stamp=InterlockedExchange64(&g_interact_codec_pending,0);
    if (stamp) InterlockedIncrement(&g_interact_stats.codec_dropped);
    if (!stamp || now<(uint64_t)stamp || now-(uint64_t)stamp>100u ||
        !g_controls_provider || !g_controls_allowed ||
        now<g_controls_lease || now-g_controls_lease>100u ||
        !g_b.armed || g_b.script_menu_only || g_b.s_late_unsafe ||
        !g_b.a.gm_player_status || !g_b.a.gm_game_status ||
        !g_b.a.gm_game_status_scn || !g_b.a.gm_menu_status ||
        !g_b.a.gm_menu_status_scn || !g_b.a.gv_pad_data_direct) goto done;
    player=*(volatile ULONGLONG *)(ULONG_PTR)g_b.a.gm_player_status;
    game=(unsigned)RD32(g_b.a.gm_game_status)|(unsigned)RD32(g_b.a.gm_game_status_scn);
    menu=(unsigned)RD32(g_b.a.gm_menu_status)|(unsigned)RD32(g_b.a.gm_menu_status_scn);
    if ((player&DG_PLAYER_UNSAFE_MASK) || (game&DG_GAME_UNSAFE_MASK) ||
        (menu&DG_MENU_UNSAFE_MASK)) goto done;
    rec=g_b.a.gv_pad_data_direct;
    /* Never turn native START+SELECT into a reset chord. */
    if (((unsigned)RD32(rec+DG_GV_PAD_STATUS_OFFSET) |
         (unsigned)RD32(rec+DG_GV_PAD_PRESS_OFFSET)) &
        (DG_PAD_START|DG_MENU_PAD_SEL)) goto done;
    *(volatile DWORD *)(ULONG_PTR)(rec+DG_GV_PAD_STATUS_OFFSET)|=DG_MENU_PAD_SEL;
    *(volatile DWORD *)(ULONG_PTR)(rec+DG_GV_PAD_PRESS_OFFSET)|=DG_MENU_PAD_SEL;
    InterlockedDecrement(&g_interact_stats.codec_dropped);
    InterlockedIncrement(&g_interact_stats.codec_written);
done:
    ReleaseSRWLockShared(&g_controls_lock);
}

static void pad_seam_tick(void)
{
    action_update_boundary();
    LONG bits, stamp, now;
    ULONGLONG rec;

    InterlockedIncrement(&g_b.c_pad_seam_entries);
    if (InterlockedCompareExchange(&g_b.script_menu_only, 0, 0) &&
        !dg_bridge_menu_context_ready()) {
        script_menu_clear(1);
        InterlockedIncrement(&g_b.c_pad_context_refused);
        InterlockedExchange(&g_b.start_pending, 0);
        InterlockedExchange(&g_b.menu_status, 0);
        InterlockedExchange(&g_b.menu_clear, 0);
        InterlockedExchange(&g_b.menu_allow, 0);
        return;
    }
    if (!InterlockedCompareExchange(&g_b.armed, 0, 0)) return;
    mod_menu_start_pad();

    /* The OpenXR thread queues a rising edge; this seam is immediately after
       the game's direct UpdatePad call.  Writing here gives full-screen
       readers the direct record, while arming GV_PadPress[0] for the normal
       record lets the gameplay pause path see the same one-shot on the next
       UpdatePad pass. */
    if (InterlockedExchange(&g_b.start_pending, 0)) {
        InterlockedExchange64(&g_interact_codec_pending,0);
        int menu_only = InterlockedCompareExchange(&g_b.script_menu_only, 0, 0) != 0;
        volatile DWORD *direct_status = (volatile DWORD *)(ULONG_PTR)
            (g_b.a.gv_pad_data_direct + DG_GV_PAD_STATUS_OFFSET);
        volatile DWORD *direct_press = (volatile DWORD *)(ULONG_PTR)
            (g_b.a.gv_pad_data_direct + DG_GV_PAD_PRESS_OFFSET);
        volatile DWORD *normal_flag = (volatile DWORD *)(ULONG_PTR)
            (g_b.a.gv_pad_data + DG_GV_PAD_FLAG_OFFSET);
        InterlockedIncrement(&g_b.c_start_consumed);
        if (g_b.a.gv_pad_data_direct &&
            (menu_only || (g_b.a.gv_pad_press && g_b.a.gv_pad_data))) {
            DWORD bits = menu_only ? DG_PAD_TITLE_ENTER : DG_PAD_START;
            *direct_status |= bits;
            *direct_press |= bits;
            if (!menu_only) {
                *(volatile DWORD *)(ULONG_PTR)g_b.a.gv_pad_press |= DG_PAD_START;
                *normal_flag |= DG_GV_PAD_PRESS_SCN;
            }
        }
    }

    interact_codec_seam();
    if (InterlockedCompareExchange(&g_b.menu_mode, 0, 0) < 1) return;

    if (g_b.a.gv_pad_data_direct) {
        DWORD st = *(volatile DWORD *)(ULONG_PTR)
                       (g_b.a.gv_pad_data_direct + DG_GV_PAD_STATUS_OFFSET);
        DWORD pr = *(volatile DWORD *)(ULONG_PTR)
                       (g_b.a.gv_pad_data_direct + DG_GV_PAD_PRESS_OFFSET);
        if (st) InterlockedOr(&g_b.seen_direct_status, (LONG)st);
        if (pr) InterlockedOr(&g_b.seen_direct_press, (LONG)pr);

        if (pr) {
            int pk, slot = -1;
            for (pk = 0; pk < DG_PAD_PRESS_RING; pk++) {
                if ((DWORD)g_b.press_ring_word[pk] == pr) { slot = pk; break; }
                if (!g_b.press_ring_word[pk] && slot < 0) slot = pk;
            }
            if (slot >= 0) {
                g_b.press_ring_word[slot] = (LONG)pr;
                g_b.press_ring_count[slot]++;
            }
            {   /* And in order. The same word on consecutive frames is one
                   button crossing the edge detector, not two presses, so a
                   repeat inside 120 ms is folded - otherwise one deliberate
                   press arrives as a burst and the sequence stops being
                   readable by a human, which is the only thing it is for. */
                LONG n = g_b.press_seq_n;
                LONG last = n ? g_b.press_seq_word[(n - 1) %
                                                   DG_PAD_PRESS_SEQ] : 0;
                LONG lastms = n ? g_b.press_seq_ms[(n - 1) %
                                                   DG_PAD_PRESS_SEQ] : 0;
                LONG nowms = (LONG)GetTickCount();
                if (!n || (LONG)pr != last || (nowms - lastms) > 120) {
                    g_b.press_seq_word[n % DG_PAD_PRESS_SEQ] = (LONG)pr;
                    g_b.press_seq_ms[n % DG_PAD_PRESS_SEQ] = nowms;
                    g_b.press_seq_n = n + 1;
                }
            }
        }
    }

    if (InterlockedCompareExchange(&g_b.menu_mode, 0, 0) < 2) return;
    /* XR presents can outnumber pad updates. Retain a real edge through valid
       neutral Presents, then consume once here, bounded by wall time even
       while the player tick is paused on title/game-over screens. */
    if (!g_b.script_menu_only && TryAcquireSRWLockExclusive(&g_script_menu_lock)) {
        int xr_pending=g_script_menu_pending.allow==DG_MENU_ALLOW_XR ||
                       g_script_menu_pending.allow==DG_MENU_ALLOW_XR_CONFIRM;
        if (xr_pending) {
            unsigned menu=0,game=0,send=0;int context_ok=0;
            if(g_b.a.gm_menu_status && g_b.a.gm_menu_status_scn &&
               g_b.a.gm_game_status && g_b.a.gm_game_status_scn) {
                menu=(unsigned)RD32(g_b.a.gm_menu_status)|(unsigned)RD32(g_b.a.gm_menu_status_scn);
                game=(unsigned)RD32(g_b.a.gm_game_status)|(unsigned)RD32(g_b.a.gm_game_status_scn);
                context_ok=!(menu&DG_UI_PANEL_MENU_MASK);
            }
            if(GetTickCount64()>=g_script_menu_deadline)
                InterlockedIncrement(&g_b.c_menu_stale);
            else if(!context_ok || !g_xr_menu_context || g_xr_menu_context!=xr_menu_context_now() ||
                    g_script_menu_pending_epoch!=g_script_menu_cancel)
                InterlockedIncrement(&g_b.c_menu_gate);
            else if(g_b.a.gv_pad_data_direct) {
                volatile DWORD *st=(volatile DWORD *)(ULONG_PTR)(g_b.a.gv_pad_data_direct+DG_GV_PAD_STATUS_OFFSET);
                volatile DWORD *pr=(volatile DWORD *)(ULONG_PTR)(g_b.a.gv_pad_data_direct+DG_GV_PAD_PRESS_OFFSET);
                send=dg_menu_native_confirm(g_script_menu_pending.status,
                    g_script_menu_pending.allow==DG_MENU_ALLOW_XR_CONFIRM,
                    g_xr_menu_context==1,
                    (game&0x80004000u)!=0);
                /* Never create native START+SELECT, including a physical SELECT. */
                if((send&DG_PAD_START) && ((*st|*pr)&DG_MENU_PAD_SEL))send=0;
                if(send) {
                    *st=(*st & ~g_script_menu_pending.clear)|send;
                    *pr=(*pr & ~g_script_menu_pending.clear)|send;
                    InterlockedIncrement(&g_b.c_menu_writes);
                }
            }
            memset(&g_script_menu_pending,0,sizeof g_script_menu_pending);
            g_script_menu_deadline=0;
        }
        ReleaseSRWLockExclusive(&g_script_menu_lock);
        if(xr_pending)return;
    }
    if (InterlockedCompareExchange(&g_b.script_menu_only, 0, 0)) {
        /* Never block the game's pad thread. The pending tuple is consumed
           under one lock, including the write, so a newer publication cannot
           be cleared by an older consumer. Wall time works with c_ticks=0. */
        if (!TryAcquireSRWLockExclusive(&g_script_menu_lock)) return;
        if (InterlockedCompareExchange(&g_script_menu_cancel, 0, 0) == g_script_menu_pending_epoch &&
            g_script_menu_pending.allow && g_script_menu_pending.status) {
            if (GetTickCount64() >= g_script_menu_deadline) {
                InterlockedIncrement(&g_b.c_menu_stale);
            } else if (!dg_bridge_menu_context_ready() ||
                       (g_b.a.gm_menu_status &&
                        (((unsigned int)RD32(g_b.a.gm_menu_status) |
                          (unsigned int)RD32(g_b.a.gm_menu_status_scn)) & DG_UI_PANEL_MENU_MASK))) {
                InterlockedIncrement(&g_b.c_menu_gate);
            } else if (g_b.a.gv_pad_data_direct) {
                volatile DWORD *st = (volatile DWORD *)(ULONG_PTR)
                    (g_b.a.gv_pad_data_direct + DG_GV_PAD_STATUS_OFFSET);
                volatile DWORD *pr = (volatile DWORD *)(ULONG_PTR)
                    (g_b.a.gv_pad_data_direct + DG_GV_PAD_PRESS_OFFSET);
                *st = (*st & ~g_script_menu_pending.clear) | g_script_menu_pending.status;
                *pr = (*pr & ~g_script_menu_pending.clear) | g_script_menu_pending.status;
                InterlockedIncrement(&g_b.c_menu_writes);
            }
        }
        memset(&g_script_menu_pending, 0, sizeof g_script_menu_pending);
        g_script_menu_deadline = 0;
        ReleaseSRWLockExclusive(&g_script_menu_lock);
        return;
    }
    if (!InterlockedCompareExchange(&g_b.menu_allow, 0, 0)) return;
    bits = InterlockedCompareExchange(&g_b.menu_status, 0, 0);
    if (!bits) return;

    /* A command older than a few ticks is a Present hook that stopped
       firing, not a button. Refuse it rather than repeat it - a stuck menu
       button is the worst failure this could have. */
    stamp = InterlockedCompareExchange(&g_b.menu_stamp, 0, 0);
    now = InterlockedCompareExchange(&g_b.c_ticks, 0, 0);
    if ((LONG)((DWORD)now - (DWORD)stamp) > 4) {
        InterlockedIncrement(&g_b.c_menu_stale);
        if ((unsigned int)bits & DG_CODEC_MENU_REQUEST) {
            InterlockedIncrement(&g_codec_exit_refused);
            InterlockedExchange(&g_b.menu_status, 0);
        }
        return;
    }

    /* The second half of the gate, read here on the game thread: a weapon
       or item menu must never see a byte of this. */
    if (g_b.a.gm_menu_status) {
        unsigned int menu = (unsigned int)RD32(g_b.a.gm_menu_status) |
                            (unsigned int)RD32(g_b.a.gm_menu_status_scn);
        if (menu & DG_UI_PANEL_MENU_MASK) {
            InterlockedIncrement(&g_b.c_menu_gate);
            if ((unsigned int)bits & DG_CODEC_MENU_REQUEST) {
                InterlockedIncrement(&g_codec_exit_refused);
                InterlockedExchange(&g_b.menu_status, 0);
            }
            return;
        }
    }

    rec = g_b.a.gv_pad_data_direct;
    if (!rec) return;
    if ((unsigned int)bits & DG_CODEC_MENU_REQUEST) {
        unsigned int menu = 0;
        if (g_b.a.gm_menu_status && g_b.a.gm_menu_status_scn)
            menu = (unsigned int)RD32(g_b.a.gm_menu_status) |
                   (unsigned int)RD32(g_b.a.gm_menu_status_scn);
        bits = (LONG)dg_codec_menu_route((unsigned int)bits, menu,
            (unsigned int)RD32(rec + DG_GV_PAD_STATUS_OFFSET) |
            (unsigned int)RD32(rec + DG_GV_PAD_PRESS_OFFSET));
        if (!bits) {
            InterlockedIncrement(&g_codec_exit_refused);
            InterlockedExchange(&g_b.menu_status, 0);
            return;
        }
        InterlockedIncrement(&g_codec_exit_written);
    }
    {
        /* Remove first, then add. The removal is what makes the addition
           mean anything when the runtime is feeding the same physical
           control to the game behind our back - see DG_BRIDGE_MENU.clear.
           Applied to status AND press, because the menu reads the flank
           from press and the held state from status, and a cancel left in
           either one still wins the branch. */
        DWORD drop = (DWORD)InterlockedCompareExchange(&g_b.menu_clear, 0, 0);
        volatile DWORD *st = (volatile DWORD *)(ULONG_PTR)
                                 (rec + DG_GV_PAD_STATUS_OFFSET);
        volatile DWORD *pr = (volatile DWORD *)(ULONG_PTR)
                                 (rec + DG_GV_PAD_PRESS_OFFSET);
        if (drop) {
            *st &= ~drop;
            *pr &= ~drop;
        }
        *st |= (DWORD)bits;
        *pr |= (DWORD)bits;
    }
    InterlockedIncrement(&g_b.c_menu_writes);
    /* One shot: the next frame must re-publish or nothing is pressed. */
    InterlockedExchange(&g_b.menu_status, 0);
}

void dg_bridge_screen_seam_now(void)
{
    unsigned int game, menu;
    ULONGLONG player = 0;

    if (!InterlockedCompareExchange(&g_b.armed, 0, 0)) return;
    /* The camera seam's rule, unchanged: no anchors, no sample and no
       publication - never a judgment built on address zero. */
    if (!g_b.a.gm_game_status || !g_b.a.gm_menu_status) return;

    game = (unsigned int)RD32(g_b.a.gm_game_status) |
           (unsigned int)RD32(g_b.a.gm_game_status_scn);
    menu = (unsigned int)RD32(g_b.a.gm_menu_status) |
           (unsigned int)RD32(g_b.a.gm_menu_status_scn);
    if (g_b.a.gm_player_status)
        player = *(volatile ULONGLONG *)(ULONG_PTR)g_b.a.gm_player_status;

    /* Accumulate and count BEFORE judging: a zero mask beside a zero count
       says the seam never ran, which is a different finding from a state
       that never happened - the lesson the seen_* block carries. */
    InterlockedOr(&g_b.seen_game_screen, (LONG)game);
    InterlockedOr(&g_b.seen_menu_screen, (LONG)menu);
    InterlockedIncrement(&g_b.c_screen_status);

    theater_seam_judge(game, menu, player);
}

#include "dg_hanging_visibility.inl"
void dg_bridge_arm_seam_now(const DG_BRIDGE_ARM_TARGET *target)
{
    ULONGLONG arm, objs, evm;
    LONG flag;

    /* A pair is a one-seam publication. Clear it before every gate, including
       the disarmed path, so a recorder tap can never carry an older arm or
       camera into a frame where this seam did nothing. The camera helper below
       appends only after this function returns. */
    memset(&g_b.rec_pair, 0, sizeof g_b.rec_pair);
    g_b.rec_pair.rest_drift_deg = -1.0f;
    /* Hanging is intentionally unsafe for IK, but must hide the undriven
       subjective rest-pose mesh before that unsafe gate returns. */
    hanging_visibility_now();

    if (!InterlockedCompareExchange(&g_b.armed, 0, 0)) {
        left_arm_release(); return;
    }
    InterlockedExchange(&g_b.hand_command_requested,
                        (target && target->hand_write) ? 1 : 0);

    /* Sample the status words HERE, before any state gate below can return.
       The run of 2026-08-17 left F2.5 unsettled for exactly this reason: the
       game-status word read zero across 8428 ticks including a cutscene, and
       the log could not say whether the anchor is dead or the gate was simply
       never provoked. Two things made it unanswerable, and both are here:

         - the tick seam samples at the Action() merge point, which has already
           been caught lying about the arm flag (see the late-flag comment
           below), and which may not run at all during a cutscene;
         - this function used to return above if we were not holding first
           person, so during a codec or a cutscene - precisely the states the
           gate exists for - it measured nothing.

       So this block sits ahead of every gate and needs only the anchors. If
       the game word moves here while the tick seam sees nothing, the seam was
       the problem. If it stays zero here too, through a cutscene, with the
       counter proving this ran, then the anchor is wrong and has to be
       searched for again. Either way the next run answers it instead of
       leaving two readings standing. */
    if (g_b.a.gm_game_status && g_b.a.gm_menu_status) {
        uint64_t radial_generation=radial_late_revoke();
        unsigned int game = (unsigned int)RD32(g_b.a.gm_game_status) |
                            (unsigned int)RD32(g_b.a.gm_game_status_scn);
        unsigned int menu = (unsigned int)RD32(g_b.a.gm_menu_status) |
                            (unsigned int)RD32(g_b.a.gm_menu_status_scn);
        InterlockedOr(&g_b.seen_game_late, (LONG)game);
        InterlockedOr(&g_b.seen_menu_late, (LONG)menu);
        InterlockedExchange(&g_b.s_late_game, (LONG)game);
        InterlockedExchange(&g_b.s_late_menu, (LONG)menu);
        ULONGLONG p = 0;
        if (g_b.a.gm_player_status) {
            p = *(volatile ULONGLONG *)(ULONG_PTR)g_b.a.gm_player_status;
            InterlockedOr(&g_b.seen_player_late_lo, (LONG)(DWORD)p);
            InterlockedOr(&g_b.seen_player_late_hi, (LONG)(DWORD)(p >> 32));
            InterlockedExchange(&g_b.s_late_player_lo, (LONG)(DWORD)p);
            InterlockedExchange(&g_b.s_late_player_hi, (LONG)(DWORD)(p >> 32));
        }
        /* The same verdict bridge_tick reaches, from the same three masks, but
           computed HERE - on the seam that keeps running through a cutscene
           and that the joint writers hang off. Without it those writers are
           gated on an fps.state the tick seam stopped updating when the
           cutscene started, which is the one moment the gate is named for. */
        InterlockedExchange(&g_b.s_late_unsafe,
                            ((p & DG_PLAYER_UNSAFE_MASK) != 0 ||
                             (game & DG_GAME_UNSAFE_MASK) != 0 ||
                             (menu & DG_MENU_UNSAFE_MASK) != 0) ? 1 : 0);
        /* Stamped last, so bridge_tick can never see a fresh timestamp over
           values that are still half written. */
        InterlockedExchange(&g_b.late_tick, g_b.c_ticks);
        /* Counted so that "never saw a bit" can be told apart from "never
           ran". Without this, a zero accumulator has two meanings and the
           whole measurement is worth nothing - which is the mistake this
           block exists to correct. */
        InterlockedIncrement(&g_b.c_late_status);
        radial_late_publish(p,game,menu,radial_generation);
        /* The theater judgment lives on the same sample: same words, same
           seam, one extra pure decision plus counters. See its header. */
        theater_seam_judge(game, menu, p);
    }

    if (g_b.fps.state != DG_FPS_ACTIVE ||
        InterlockedCompareExchange(&g_b.s_late_unsafe, 0, 0)) {
        /* Ownership follows the subjective view, not merely the lifetime of
           the arm object. A codec, cutscene, native leave or safety suspend
           must not carry our adjust bits into whatever draws next.

           Two verdicts, not one, and the second is the load-bearing half: the
           tick seam that maintains fps.state runs at a fifth of its normal
           rate during a codec or a cutscene while this seam runs faster than
           ever, so fps.state alone would still read ACTIVE all the way
           through. Counted separately, because "the writer stood down on the
           seam's own reading" is a different fact from "the state machine had
           already left" and the summary should not merge them. */
        if (g_b.fps.state == DG_FPS_ACTIVE)
            InterlockedIncrement(&g_b.c_seam_refused_unsafe);
        arm_map_forget();
        arm_bend_release();
        left_arm_release();
        return;
    }
    if (!g_b.a.gm_player_arm_body) { left_arm_release(); return; }
    /* Counted so a run that measured nothing can say WHY. The probes below all
       sit behind this gate, because the subjective arm's pose is only
       meaningful while it is the thing being drawn - but that means a session
       where first person was never entered produces total silence, and silence
       reads as "the probe is broken" rather than "the probe never ran". The
       run of 2026-08-17 was lost to exactly that. */
    count_active_seam();

    arm = *(volatile ULONGLONG *)(ULONG_PTR)g_b.a.gm_player_arm_body;
    if (!arm) { left_arm_release(); return; }
    skel_probe_now(arm, target != NULL);
    /* Reads only, and reads a slot nobody here writes - SetPos owns adjust[6].
       So it sits outside the three-way exclusion below rather than inside it. */
    hand_probe_read(arm);
    /* All three are exclusive by design. The measurement, IK and fixed bend
       write MOTION_CONTROL.adjust; whichever path is selected first releases
       ownership left by the previous one before it writes. */
    if (reload_native_animation()) {
        arm_map_forget();arm_bend_release();left_arm_release();return;
    }
    if (InterlockedCompareExchange(&g_b.adj_probe, 0, 0)) {
        arm_map_forget();
        arm_bend_release();
        left_arm_release();
        adj_probe_now(arm);
    } else if (target) {
        LONG accepted_before = g_b.c_arm_pairs_accepted;
        unsigned long left_before=g_left.accepted;
        memset(&g_hand_capture,0,sizeof g_hand_capture);
        arm_ik_now(arm, target);
        left_arm_now(arm, target, g_b.c_arm_pairs_accepted != accepted_before);
        hand_profile_commit(target,g_b.c_arm_pairs_accepted != accepted_before &&
            g_left.active && g_left.accepted!=left_before && g_left.wrist_world_valid);
        if(g_b.c_arm_pairs_accepted != accepted_before) hand_pose_now(arm,target);
        else if(!target->write || !g_b.ik_active || !g_left.active) hand_pose_release();
    } else {
        arm_map_forget();
        left_arm_release();
        arm_bend_now(arm, NULL);
    }
    objs = *(volatile ULONGLONG *)(ULONG_PTR)arm;
    if (!objs) return;
    flag = RD32(objs + 0x58);

    /* Sample here whether or not we are forcing, because the tick-seam sample
       lies. It reads at the Action() merge point, and the run of 2026-08-14
       had it report the arm hidden on all 2678 ticks while the player was
       plainly looking at drawn arms. This seam is after every actor, so it is
       the last word on the flag before the frame is submitted; publishing both
       is what makes the disagreement visible instead of quietly wrong. */
    InterlockedExchange(&g_b.s_arm_flag_late, flag);
    InterlockedOr(&g_b.seen_arm_flag_late, flag);
    InterlockedAnd(&g_b.held_arm_flag_late, flag);
    if (!(flag & 0x1000)) InterlockedIncrement(&g_b.c_arm_visible_late);

    /* Native CheckVWait2 hides the complete subjective arm object for None.
       A successfully driven left arm still needs that object drawn. Keep the
       existing explicit arm-show override, but do not require it for None. */
    if (!InterlockedCompareExchange(&g_b.arm_show, 0, 0) &&
        !left_arm_show_unarmed(arm, target)) return;
    WR32(objs + 0x58, (LONG)(flag & ~0x1000));
    evm = *(volatile ULONGLONG *)(ULONG_PTR)(arm + 0x30);
    if (evm) WR32(evm + 0x58, (LONG)(RD32(evm + 0x58) & ~0x100));
    InterlockedIncrement(&g_b.c_arm_forced_late);
}

int dg_bridge_theater_verdict(void)
{
    LONG bits, ms, mode;
    int door;
    if (!InterlockedCompareExchange(&g_b.armed, 0, 0)) return 0;
    door=dg_bridge_hatch_now()?4:0;
    bits = InterlockedCompareExchange(&g_b.s_theater, 0, 0);
    if (!bits) return door;
    /* Freshness before modes: a stale publication is refused whatever the
       marker says, so a camera hook that stops firing walks every consumer
       back to today's behaviour within half a second. DWORD arithmetic so
       the 49-day GetTickCount wrap compares correctly. */
    ms = InterlockedCompareExchange(&g_b.theater_ms, 0, 0);
    if ((DWORD)(GetTickCount() - (DWORD)ms) > DG_THEATER_FRESH_MS) return door;
    mode = InterlockedCompareExchange(&g_b.theater_mode, 0, 0);
    if (mode != DG_THEATER_ON) bits &= ~DG_THEATER_V_DEMO;
    if (!InterlockedCompareExchange(&g_b.theater_ui_mode, 0, 0))
        bits &= ~DG_THEATER_V_UI;
    return (int)bits|door;
}

int dg_bridge_controller_special_now(void)
{
    uint64_t player,identity;
    unsigned game,menu,gs,ms;
    int weapon;
    LONG samples,age;
    ULONGLONG late;
    if (!g_b.armed || !interact_player_now(&identity,&weapon) ||
        !interact_read(NULL,g_b.a.gm_player_status,&player,8) ||
        !interact_read(NULL,g_b.a.gm_game_status,&game,4) ||
        !interact_read(NULL,g_b.a.gm_game_status_scn,&gs,4) ||
        !interact_read(NULL,g_b.a.gm_menu_status,&menu,4) ||
        !interact_read(NULL,g_b.a.gm_menu_status_scn,&ms,4)) return 0;
    game|=gs;menu|=ms;
    samples=InterlockedCompareExchange(&g_b.c_late_status,0,0);
    age=(LONG)((DWORD)g_b.c_ticks-(DWORD)g_b.late_tick);
    late=((ULONGLONG)(DWORD)g_b.s_late_player_hi<<32)|(DWORD)g_b.s_late_player_lo;
    fold_late_status(&game,&menu,&player,(unsigned)g_b.s_late_game,
        (unsigned)g_b.s_late_menu,late,samples,age);
    if ((game&DG_GAME_UNSAFE_MASK) || (menu&DG_MENU_UNSAFE_MASK)) return 0;
    switch (player&DG_PLAYER_UNSAFE_MASK) {
    case 0x10000ULL: return (player&1ULL)?0:DG_CONTROLS_LADDER;
    case 0x1000ULL: return DG_CONTROLS_BEYOND;
    case 0x80ULL: return DG_CONTROLS_LOCKER;
    /* Down requires a fresh face press to enter Rise. Preserve every other
       unsafe bit, including DEAD/FORCE; WATCH permits either camera mode. */
    case 0x400ULL: return DG_CONTROLS_DOWNED;
    default:return 0;
    }
}
#include "dg_hanging_heading.inl"
int dg_bridge_controller_ladder_now(void)
{
    return dg_bridge_controller_special_now()==DG_CONTROLS_LADDER;
}

int dg_bridge_codec_input_now(void)
{
    unsigned menu,scenario;
    if (!g_b.armed || g_b.script_menu_only ||
        !interact_read(NULL,g_b.a.gm_menu_status,&menu,4) ||
        !interact_read(NULL,g_b.a.gm_menu_status_scn,&scenario,4)) return 0;
    menu|=scenario;
    return (menu&0x400u) && !(menu&DG_UI_PANEL_MENU_MASK);
}

int dg_bridge_controller_gameplay_now(void)
{
    ULONGLONG arm, player;
    uint64_t general_player;
    int general_weapon;
    unsigned int game, menu;
    if (!InterlockedCompareExchange(&g_b.armed,0,0) ||
        !g_b.a.gm_player_status || !g_b.a.gm_player_arm_body ||
        !g_b.a.gm_game_status || !g_b.a.gm_game_status_scn ||
        !g_b.a.gm_menu_status || !g_b.a.gm_menu_status_scn ||
        region_end(g_b.a.gm_player_status) < g_b.a.gm_player_status+8 ||
        region_end(g_b.a.gm_player_arm_body) < g_b.a.gm_player_arm_body+8 ||
        region_end(g_b.a.gm_game_status) < g_b.a.gm_game_status+4 ||
        region_end(g_b.a.gm_game_status_scn) < g_b.a.gm_game_status_scn+4 ||
        region_end(g_b.a.gm_menu_status) < g_b.a.gm_menu_status+4 ||
        region_end(g_b.a.gm_menu_status_scn) < g_b.a.gm_menu_status_scn+4)
        return 0;
    arm=*(volatile ULONGLONG *)(ULONG_PTR)g_b.a.gm_player_arm_body;
    if (!plausible_ptr(arm) && !interact_player_now(&general_player,&general_weapon))
        return 0; /* front end has neither an arm nor a current general player */
    player=*(volatile ULONGLONG *)(ULONG_PTR)g_b.a.gm_player_status;
    game=(unsigned int)RD32(g_b.a.gm_game_status) | (unsigned int)RD32(g_b.a.gm_game_status_scn);
    menu=(unsigned int)RD32(g_b.a.gm_menu_status) | (unsigned int)RD32(g_b.a.gm_menu_status_scn);
    return !(player&DG_PLAYER_UNSAFE_MASK) && !(game&DG_GAME_UNSAFE_MASK) &&
           !(menu&DG_MENU_UNSAFE_MASK);
}

int dg_bridge_camera_gate_now(DG_CAMERA_GATE *out)
{
    ULONGLONG arm, work, camera, player_status;
    unsigned int game, menu;
    int native_active, safe;
    if (!out) return 0;
    memset(out, 0, sizeof(*out));
    out->armed = InterlockedCompareExchange(&g_b.armed, 0, 0) ? 1 : 0;
    out->fps_active = (g_b.fps.state == DG_FPS_ACTIVE) ? 1 : 0;
    if (!out->armed || !out->fps_active || g_b.owner) return 0;
    if (!g_b.a.gbp_active || !g_b.a.gm_player_status ||
        !g_b.a.gm_game_status || !g_b.a.gm_game_status_scn ||
        !g_b.a.gm_menu_status || !g_b.a.gm_menu_status_scn ||
        !g_b.a.gm_player_arm_body ||
        region_end(g_b.a.gbp_active) < g_b.a.gbp_active + 4 ||
        region_end(g_b.a.gm_player_status) < g_b.a.gm_player_status + 8 ||
        region_end(g_b.a.gm_game_status) < g_b.a.gm_game_status + 4 ||
        region_end(g_b.a.gm_game_status_scn) < g_b.a.gm_game_status_scn + 4 ||
        region_end(g_b.a.gm_menu_status) < g_b.a.gm_menu_status + 4 ||
        region_end(g_b.a.gm_menu_status_scn) < g_b.a.gm_menu_status_scn + 4 ||
        region_end(g_b.a.gm_player_arm_body) < g_b.a.gm_player_arm_body + 8)
        return 0;
    native_active = RD32(g_b.a.gbp_active) ? 1 : 0;
    player_status = *(volatile ULONGLONG *)(ULONG_PTR)g_b.a.gm_player_status;
    game = (unsigned int)RD32(g_b.a.gm_game_status) |
           (unsigned int)RD32(g_b.a.gm_game_status_scn);
    menu = (unsigned int)RD32(g_b.a.gm_menu_status) |
           (unsigned int)RD32(g_b.a.gm_menu_status_scn);
    safe = (player_status & DG_PLAYER_UNSAFE_MASK) == 0 &&
           (game & DG_GAME_UNSAFE_MASK) == 0 &&
           (menu & DG_MENU_UNSAFE_MASK) == 0;
    out->native_active = native_active;
    out->safe_gameplay = safe;
    out->game_status = game;
    out->menu_status = menu;
    /* These are hard refusals independent of the player/arm chain. Keeping
       them before the arm walk is what makes an inactive native FPS or an
       unsafe scene safe even when the arm pointer is stale or unreadable. */
    if (!native_active || !safe) return 0;
    arm = *(volatile ULONGLONG *)(ULONG_PTR)g_b.a.gm_player_arm_body;
    if (!plausible_ptr(arm) || arm <= 0x60ULL ||
        region_end(arm) < arm + 8) return 0;
    work = arm - 0x60ULL;
    if (region_end(work) < work + 0x250 || RD32(work + 0x238) != 0) return 0;
    camera = *(volatile ULONGLONG *)(ULONG_PTR)(work + 0x248);
    if (!plausible_ptr(camera) || region_end(camera) < camera + 0x30) return 0;
    out->arm_camera_on = RD32(camera + 0x2C) ? 1 : 0;
    out->arm_body = arm;
    out->camera = camera;
    out->valid = native_active && safe && out->arm_camera_on;
    return out->valid;
}

long dg_bridge_fps_entry_generation(void)
{ return InterlockedCompareExchange(&g_b.c_explicit_entries,0,0); }
int dg_bridge_fps_recenter_ready(unsigned long long arm)
{ return arm && (ULONGLONG)InterlockedCompareExchange64(&g_b.calibration_ready_arm,0,0)==arm; }

int dg_bridge_view_calibration_now(long *generation, unsigned long long *identity)
{
    DG_CAMERA_GATE camera;
    uint64_t player;
    int weapon;
    *generation=0; *identity=0;
    if (!g_b.armed || g_b.owner || g_b.script_menu_only ||
        dg_bridge_theater_verdict() || !dg_bridge_controller_gameplay_now()) return 0;
    if (dg_bridge_camera_gate_now(&camera) &&
        dg_bridge_fps_recenter_ready(camera.arm_body)) {
        *generation=dg_bridge_fps_entry_generation();
        *identity=camera.arm_body;
        return 1;
    }
    /* An unsafe suspension/cutscene is not a request for top-down view.
       Wait for native OFF as well as the completed user-requested leave. */
    if (g_b.fps.state!=DG_FPS_OFF || g_b.fps.desired || !g_b.c_left ||
        !g_b.a.gbp_active || region_end(g_b.a.gbp_active)<g_b.a.gbp_active+4 ||
        RD32(g_b.a.gbp_active) || !interact_player_now(&player,&weapon)) return 0;
    *generation=InterlockedCompareExchange(&g_b.c_left,0,0);
    *identity=player;
    return 2;
}

#include "dg_camera_bob.inl"

/* The actuator probe's drain: one sample per call, oldest first. The
   producer publishes by advancing turn_probe_w after the slot is filled,
   so a sample this reads is complete. Consumer-side only ever advances
   turn_probe_r, so the pair never races. */
int dg_bridge_turn_probe_take(DG_TURN_PROBE_SAMPLE *out)
{
    LONG r = g_b.turn_probe_r;
    if (r >= InterlockedCompareExchange(&g_b.turn_probe_w, 0, 0)) return 0;
    *out = g_b.turn_probe_ring[r % DG_TURN_PROBE_RING];
    InterlockedIncrement(&g_b.turn_probe_r);
    return 1;
}

void dg_bridge_stats(DG_BRIDGE_STATS *out)
{
    coolant_trace_flush();
    memset(out, 0, sizeof(*out));
    out->ticks = g_b.c_ticks;
    out->fps_requested = g_b.c_requested;
    out->fps_entered = g_b.c_entered;
    out->fps_left = g_b.c_left;
    out->fps_transitions = g_b.c_transitions;
    out->fps_edges_injected = g_b.c_edges;
    out->fps_writes_applied = g_b.c_writes;
    out->fps_refused_mask_zero = g_b.c_mask_zero;
    out->fps_suspend_level_load = g_b.c_level_load;
    out->fps_suspend_unsafe = g_b.c_unsafe;
    out->fps_timeouts = g_b.c_timeouts;
    out->fps_events_dropped = g_b.c_dropped;
    out->fps_state = g_b.fps.state;
    out->fps_suspend_reason = g_b.fps.reason;
    out->fps_native_active = (int)g_b.s_native_active;
    out->mgshdfix_fps_owner = g_b.owner;
    out->override_value = (int)g_b.s_override;
    out->toggle_value = (int)g_b.s_toggle;
    out->move_value = (int)g_b.s_move;
    out->subject_move_value = (int)g_b.s_subject_move;
    out->subject_move_ticks = g_b.c_subject_move_ticks;
    out->subject_toggle_value = (int)g_b.s_subject_toggle;
    out->pad_subject_mask = (unsigned int)g_b.s_pad_subject;
    out->pad_stop_aim_mask = (unsigned int)g_b.s_pad_stop_aim;
    out->game_status = (unsigned int)g_b.s_game_status;
    out->menu_status = (unsigned int)g_b.s_menu_status;
    out->player_status = ((unsigned __int64)(DWORD)g_b.s_status_hi << 32) |
                         (unsigned __int64)(DWORD)g_b.s_status_lo;
    out->arm_cam_rotate_shift = g_b.a.arm_cam_rotate_shift;
    out->hand_probe_writes = g_b.c_hand_probe_writes;
    out->hand_probe_reads = g_b.c_hand_probe_reads;
    out->hand_probe_no_setpos = g_b.c_hand_probe_no_setpos;
    {
        int k;
        LONG v;
        for (k = 0; k < 3; k++) {
            out->hand_probe_wrote[k] = (short)g_b.s_hand_probe_wrote[k];
            out->hand_probe_read[k] = (short)g_b.s_hand_probe_read[k];
        }
        for (k = 0; k < 4; k++) {
            v = g_b.s_hand_probe_adjust[k];
            memcpy(&out->hand_probe_adjust6[k], &v, sizeof v);
            v = g_b.s_hand_probe_pred[k];
            memcpy(&out->hand_probe_predicted[k], &v, sizeof v);
        }
        v = g_b.s_hand_probe_worst;
        memcpy(&out->hand_probe_worst_diff, &v, sizeof v);
    }
    {
        LONG xy = g_b.s_arm_cam_rot_xy;
        out->arm_cam_rot[0] = (short)(xy & 0xFFFF);
        out->arm_cam_rot[1] = (short)((xy >> 16) & 0xFFFF);
        out->arm_cam_rot[2] = (short)g_b.s_arm_cam_rot_z;
    }
    out->arm_body = ((unsigned __int64)(DWORD)g_b.s_arm_body_hi << 32) |
                    (unsigned __int64)(DWORD)g_b.s_arm_body_lo;
    out->arm_objs = ((unsigned __int64)(DWORD)g_b.s_arm_objs_hi << 32) |
                    (unsigned __int64)(DWORD)g_b.s_arm_objs_lo;
    out->arm_flag = (unsigned int)g_b.s_arm_flag;
    out->body_cand = ((ULONGLONG)(DWORD)g_b.s_body_hi << 32)
                   | (DWORD)g_b.s_body_lo;
    out->body_objs = ((ULONGLONG)(DWORD)g_b.s_body_objs_hi << 32)
                   | (DWORD)g_b.s_body_objs_lo;
    out->body_flag = (unsigned int)g_b.s_body_flag;
    out->arm_created = g_b.c_arm_created;
    out->arm_destroyed = g_b.c_arm_destroyed;
    out->arm_forced = g_b.c_arm_forced;
    out->arm_forced_late = g_b.c_arm_forced_late;
    out->seen_player = ((unsigned __int64)(DWORD)g_b.seen_player_hi << 32) |
                       (unsigned __int64)(DWORD)g_b.seen_player_lo;
    out->seen_game = (unsigned int)g_b.seen_game;
    out->seen_menu = (unsigned int)g_b.seen_menu;
    out->seen_game_late = (unsigned int)g_b.seen_game_late;
    out->seen_menu_late = (unsigned int)g_b.seen_menu_late;
    out->seen_player_late = ((unsigned __int64)(DWORD)g_b.seen_player_late_hi << 32)
                          | (unsigned __int64)(DWORD)g_b.seen_player_late_lo;
    out->late_status_samples = g_b.c_late_status;
    out->orient_anchored = g_b.c_orient_anchored;
    out->orient_fallback = g_b.c_orient_fallback;
    out->orient_uprolled = g_b.c_orient_uprolled;
    out->uproll_skipped = g_b.c_uproll_skipped;
    out->arm_uproll_on = InterlockedCompareExchange(&g_b.arm_uproll, 0, 0)
        ? 1 : 0;
    out->arm_hand_basis_world =
        InterlockedCompareExchange(&g_b.arm_hand_basis, 0, 0) == 1 ? 1 : 0;
    {
        LONG v = g_b.s_skel_hand_turned;
        memcpy(&out->skel_hand_turned_deg, &v, sizeof v);
        v = g_b.s_adj_turned4;
        memcpy(&out->adj_turned4_deg, &v, sizeof v);
        v = g_b.s_adj_turned5;
        memcpy(&out->adj_turned5_deg, &v, sizeof v);
        out->skel_hand_samples = g_b.c_skel_hand_samples;
        v = g_b.s_skel_hand_net;
        memcpy(&out->skel_hand_net_deg, &v, sizeof v);
        v = g_b.s_skel_hand_net_max;
        memcpy(&out->skel_hand_net_max_deg, &v, sizeof v);
        v = g_b.s_rest_drift;
        memcpy(&out->rest_ref_drift_deg, &v, sizeof v);
        v = g_b.s_rest_drift_max;
        memcpy(&out->rest_ref_drift_max_deg, &v, sizeof v);
        out->rest_ref_captured = g_b.rest_ref_have ? 1 : 0;
        out->menu_anchor_ok = g_b.a.pad_press_ok ? 1 : 0;
        out->menu_detour_live =
            InterlockedCompareExchange(&g_b.pad_detour_live, 0, 0) ? 1 : 0;
        out->menu_writes = g_b.c_menu_writes;
        out->pad_seam_entries = InterlockedCompareExchange(&g_b.c_pad_seam_entries, 0, 0);
        out->start_queued = InterlockedCompareExchange(&g_b.c_start_queued, 0, 0);
        out->start_consumed = InterlockedCompareExchange(&g_b.c_start_consumed, 0, 0);
        out->pad_context_refused = InterlockedCompareExchange(&g_b.c_pad_context_refused, 0, 0);
        out->menu_seen_status = (unsigned int)g_b.seen_direct_status;
        out->menu_seen_press = (unsigned int)g_b.seen_direct_press;
        {
            int pk;
            for (pk = 0; pk < DG_PAD_PRESS_RING; pk++) {
                out->menu_press_word[pk] =
                    (unsigned int)g_b.press_ring_word[pk];
                out->menu_press_count[pk] = g_b.press_ring_count[pk];
            }
            for (pk = 0; pk < DG_PAD_PRESS_SEQ; pk++) {
                out->menu_press_seq[pk] =
                    (unsigned int)g_b.press_seq_word[pk];
                out->menu_press_seq_ms[pk] = g_b.press_seq_ms[pk];
            }
            out->menu_press_seq_n = g_b.press_seq_n;
        }
        out->menu_refused_stale = g_b.c_menu_stale;
        out->menu_refused_gate = g_b.c_menu_gate;
        v = g_b.s_arm_adj_echo;
        memcpy(&out->arm_adj_echo_deg, &v, sizeof v);
        v = g_b.s_arm_adj_echo_worst;
        memcpy(&out->arm_adj_echo_worst_deg, &v, sizeof v);
        out->arm_adj_echo_dirty = g_b.c_arm_adj_echo_dirty;
    }
    out->seen_game_screen = (unsigned int)g_b.seen_game_screen;
    out->seen_menu_screen = (unsigned int)g_b.seen_menu_screen;
    out->screen_status_samples = g_b.c_screen_status;
    out->late_status_stale = g_b.c_late_stale;
    out->seam_refused_unsafe = g_b.c_seam_refused_unsafe;
    out->seam_active_samples = g_b.c_seam_active;
    out->seam_active_ticks = g_b.c_seam_active_ticks;
    out->seen_arm_flag = (unsigned int)g_b.seen_arm_flag;
    out->seen_body_flag = (unsigned int)g_b.seen_body_flag;
    out->held_arm_flag = (unsigned int)g_b.held_arm_flag;
    out->held_body_flag = (unsigned int)g_b.held_body_flag;
    out->arm_visible_ticks = g_b.c_arm_visible;
    out->body_visible_ticks = g_b.c_body_visible;
    out->arm_work = ((ULONGLONG)(DWORD)g_b.s_work_hi << 32) | (DWORD)g_b.s_work_lo;
    out->player_work = ((ULONGLONG)(DWORD)g_b.s_pwork_hi << 32)
                     | (DWORD)g_b.s_pwork_lo;
    out->arm_camera_on = g_b.s_cam_on;
    out->arm_camera_on_ticks = g_b.c_cam_on;
    out->arm_trigger = (unsigned int)g_b.s_arm_trigger;
    out->seen_arm_trigger = (unsigned int)g_b.seen_arm_trigger;
    out->arm_flag_late = (unsigned int)g_b.s_arm_flag_late;
    out->seen_arm_flag_late = (unsigned int)g_b.seen_arm_flag_late;
    out->held_arm_flag_late = (unsigned int)g_b.held_arm_flag_late;
    out->arm_visible_late_ticks = g_b.c_arm_visible_late;
    out->arm_joints = g_b.s_arm_joints;
    out->arm_adjust = ((ULONGLONG)(DWORD)g_b.s_arm_adjust_hi << 32)
                    | (DWORD)g_b.s_arm_adjust_lo;
    out->arm_bend_writes = g_b.c_arm_bent;
    out->arm_track_writes = g_b.c_arm_tracked;
    out->arm_quat_refused = g_b.c_arm_quat_refused;
    out->arm_ik_refused = g_b.c_arm_ik_refused;
    out->arm_ik_clamped = g_b.c_arm_ik_clamped;
    out->arm_ik_implausible = g_b.c_arm_ik_implausible;
    out->arm_pairs_seen = g_b.c_arm_pairs_seen;
    out->arm_pairs_eligible = g_b.c_arm_pairs_eligible;
    out->arm_pairs_accepted = g_b.c_arm_pairs_accepted;
    out->left_pairs_accepted = g_left_accepted;
    out->left_pairs_refused = g_left_refused;
    out->left_support_pairs = g_left_support_pairs;
    out->left_status = g_left_status;
    out->left_refuse_reason = g_left_reason;
    { int li; for(li=0;li<7;li++) out->left_support_gate[li]=g_left_support_gate[li]; }
    { LONG v = g_left_blend; memcpy(&out->left_blend,&v,sizeof v); }
    { int li; for(li=0;li<3;li++) {
        LONG v=g_left_target[li]; memcpy(&out->left_target[li],&v,sizeof v);
    } }
    out->arm_pairs_refused = g_b.c_arm_pairs_refused;
    out->arm_pairs_calibration = g_b.c_arm_pairs_calibration;
    out->arm_pairs_map_clamped = g_b.c_arm_pairs_map_clamped;
    out->arm_pairs_map_soft = g_b.c_arm_pairs_map_soft;
    out->arm_pair_replays = g_b.c_arm_pair_replays;
    out->arm_pairs_frozen = g_b.c_arm_pairs_frozen;
    out->arm_release_owner_mismatch = g_b.c_arm_release_owner_mismatch;
    out->arm_hand_written = g_b.c_arm_hand_written;
    out->arm_hand_refused = g_b.c_arm_hand_refused;
    out->arm_hand_measured = g_b.c_arm_hand_measured;
    out->arm_hand_tick_writes = g_b.c_arm_hand_tick_writes;
    out->arm_hand_tick_no_command = g_b.c_arm_hand_tick_no_command;
    out->arm_hand_tick_stale = g_b.c_arm_hand_tick_stale;
    out->arm_hand_tick_no_player = g_b.c_arm_hand_tick_no_player;
    out->arm_hand_tick_owner_mismatch = g_b.c_arm_hand_tick_owner_mismatch;
    out->arm_hand_tick_bad_weapon = g_b.c_arm_hand_tick_bad_weapon;
    out->arm_hand_tick_mic = g_b.c_arm_hand_tick_mic;
    out->arm_hand_no_setpos = g_b.c_arm_hand_no_setpos;
    out->arm_hand_zeroed = g_b.c_arm_hand_zeroed;
    out->arm_body_uncompensated = g_b.c_arm_body_uncompensated;
    out->arm_comp = (int)InterlockedCompareExchange(&g_b.arm_comp, 0, 0);
    out->comp_no_stick = g_b.c_arm_comp_no_stick;
    out->move_writes = g_b.c_move_writes;
    out->move_yielded = g_b.c_move_yielded;
    out->move_idle = g_b.c_move_idle;
    out->move_stale = g_b.c_move_stale;
    out->move_no_command = g_b.c_move_no_command;
    out->move_blocked_gate = g_b.c_move_blocked_gate;
    out->move_published = g_b.c_move_published;
    out->move_mode = (int)InterlockedCompareExchange(&g_b.walk_mode, 0, 0);
    out->move_deadzone = (float)InterlockedCompareExchange(
                             &g_b.move_deadzone_mils, 0, 0) / 1000.0f;
    out->move_not_subject = g_b.c_move_not_subject;
    out->move_third = (int)InterlockedCompareExchange(&g_b.move_third, 0, 0);
    out->move_prone = (int)InterlockedCompareExchange(&g_b.move_prone, 0, 0);
    out->move_third_writes = g_b.c_move_third_writes;
    out->move_not_third = g_b.c_move_not_third;
    out->move_prone_writes = g_b.c_move_prone_writes;
    out->move_dir_writes = g_b.c_move_dir_writes;
    out->move_cam_dir = InterlockedCompareExchange(&g_b.s_cam_dir, 0, 0);
    out->move_last_org = g_b.s_move_last_org;
    out->move_last_dir = g_b.s_move_last_dir;
    out->move_padto_writes = g_b.c_move_padto_writes;
    out->move_no_workl = g_b.c_move_no_workl;
    out->tick_seam_copy = (int)InterlockedCompareExchange(&g_b.seam_is_copy, 0, 0);
    out->mp_w_tick = g_b.mp_w_tick; out->mp_w_dir = g_b.mp_w_dir;
    out->mp_w_status = (unsigned long)g_b.mp_w_status; out->mp_w_bytes = g_b.mp_w_bytes;
    out->mp_c_tick = g_b.mp_c_tick; out->mp_c_dir = g_b.mp_c_dir;
    out->mp_c_status = (unsigned long)g_b.mp_c_status; out->mp_c_bytes = g_b.mp_c_bytes;
    out->mp_c_analog = g_b.mp_c_analog; out->mp_c_rot = g_b.mp_c_rot;
    out->mp_c_turn = g_b.mp_c_turn; out->mp_c_seen = g_b.mp_c_seen;
    out->mp_c_act = (unsigned long long)InterlockedCompareExchange64(&g_b.mp_c_act, 0, 0);
    out->mp_c_act2 = (unsigned long long)InterlockedCompareExchange64(&g_b.mp_c_act2, 0, 0);
    out->move_dir_org = (int)InterlockedCompareExchange(&g_b.move_dir_org, 0, 0);
    out->mp_c_work_pad = (unsigned long long)InterlockedCompareExchange64(&g_b.mp_c_work_pad, 0, 0);
    out->mp_c_our_pad = (unsigned long long)g_b.a.player_pad;
    out->mp_c_gv_flag = (unsigned long)InterlockedCompareExchange(&g_b.mp_c_gv_flag, 0, 0);
    out->mp_c_wp_status = (unsigned long)g_b.mp_c_wp_status;
    out->mp_c_wp_dir = g_b.mp_c_wp_dir; out->mp_c_wp_bytes = g_b.mp_c_wp_bytes;
    out->mp_c_workl = (unsigned long long)InterlockedCompareExchange64(&g_b.mp_c_workl, 0, 0);
    out->mp_c_padto = g_b.mp_c_padto; out->mp_c_wallto = g_b.mp_c_wallto;
    out->mp_c_liable = g_b.mp_c_liable; out->mp_c_padforce = g_b.mp_c_padforce;
    out->mp_c_data = g_b.mp_c_data; out->mp_c_data2 = g_b.mp_c_data2;
    out->turn_writes = g_b.c_turn_writes;
    out->turn_yielded = g_b.c_turn_yielded;
    out->turn_idle = g_b.c_turn_idle;
    out->turn_mode = (int)InterlockedCompareExchange(&g_b.turn_mode, 0, 0);
    out->turn_gain = (float)InterlockedCompareExchange(
                         &g_b.turn_gain_mils, 0, 0) / 1000.0f;
    out->arm_cap_fore_deg = (float)(hand_cap_rad(&g_b.hand_fore_twist_mdeg,
                                        DG_HAND_FORE_TWIST_RAD) *
                                    180.0 / 3.14159265358979323846);
    out->arm_cap_wrist_deg = (float)(hand_cap_rad(&g_b.hand_wrist_twist_mdeg,
                                         DG_HAND_WRIST_TWIST_RAD) *
                                     180.0 / 3.14159265358979323846);
    out->arm_cap_swing_deg = (float)(hand_cap_rad(&g_b.hand_wrist_swing_mdeg,
                                         DG_HAND_WRIST_SWING_RAD) *
                                     180.0 / 3.14159265358979323846);
    out->turn_follow_writes = g_b.c_turn_follow_writes;
    out->follow_gate_no_sign = g_b.c_follow_no_sign;
    out->follow_gate_stale = g_b.c_follow_stale;
    out->follow_gate_under = g_b.c_follow_under;
    out->follow_gate_aim_hold = g_b.c_follow_aim_hold;
    out->follow_src = (int)InterlockedCompareExchange(&g_b.follow_src, 0, 0);
    out->follow_aim = (int)InterlockedCompareExchange(&g_b.follow_aim, 0, 0);
    out->adjust_frame_live = (int)InterlockedCompareExchange(&g_b.adjust_frame, 0, 0);
    out->frame_missing = g_b.c_frame_missing;
    {
        LONG v = InterlockedCompareExchange(&g_b.s_frame_skew_worst, 0, 0);
        memcpy(&out->frame_skew_worst, &v, sizeof out->frame_skew_worst);
        v = InterlockedCompareExchange(&g_b.s_wrist_miss_worst, 0, 0);
        memcpy(&out->wrist_miss_worst, &v, sizeof out->wrist_miss_worst);
    }
    out->wrist_miss_over = g_b.c_wrist_miss_over;
    out->turn_write_dead = g_b.c_turn_write_dead;
    {
        LONG gv = InterlockedCompareExchange(&g_b.s_arm_aim_gap, 0, 0);
        LONG acc = InterlockedCompareExchange(&g_b.turn_dir_acc, 0, 0);
        memcpy(&out->turn_aim_gap, &gv, sizeof out->turn_aim_gap);
        out->turn_dir_votes = acc;
        out->turn_dir = (acc >= 15) ? 1 : (acc <= -15 ? -1 : 0);
    }
    {
        LONG raw = InterlockedCompareExchange(&g_b.s_pad_analog_raw, 0, 0);
        out->pad_right_dx = (unsigned char)((raw >> 24) & 0xFF);
        out->pad_right_dy = (unsigned char)((raw >> 16) & 0xFF);
        out->pad_left_dx = (unsigned char)((raw >> 8) & 0xFF);
        out->pad_left_dy = (unsigned char)(raw & 0xFF);
    }

    out->arm_hand_fore_twist = g_b.c_arm_hand_fore_twist;
    out->arm_hand_limited = g_b.c_arm_hand_limited;
    {
        LONG v = g_b.s_arm_hand_residual;
        memcpy(&out->arm_hand_residual_deg, &v, sizeof v);
        v = g_b.s_arm_hand_worst;
        memcpy(&out->arm_hand_worst_deg, &v, sizeof v);
        v = g_b.s_arm_hand_slot_echo;
        memcpy(&out->arm_hand_slot_echo_deg, &v, sizeof v);
        v = g_b.s_arm_hand_slot_echo_worst;
        memcpy(&out->arm_hand_slot_echo_worst_deg, &v, sizeof v);
        out->arm_hand_slot_dirty = g_b.c_arm_hand_slot_dirty;
        v = g_b.s_arm_hand_off_drift;
        memcpy(&out->arm_hand_off_drift_deg, &v, sizeof v);
        v = g_b.s_arm_hand_off_drift_worst;
        memcpy(&out->arm_hand_off_drift_worst_deg, &v, sizeof v);
        out->arm_adjust_bits_lost = g_b.c_arm_adjust_bits_lost;
        out->arm_hand_reach_units = dg_ik_ps_reach_units(DG_HAND_PULL_DIVISOR);
        out->arm_hand_alt_named = g_b.c_arm_hand_alt_named;
        out->arm_hand_scaled = g_b.c_arm_hand_scaled;
        v = g_b.s_arm_hand_fit;
        memcpy(&out->arm_hand_fit_frac, &v, sizeof v);
        v = g_b.s_arm_hand_shortfall_worst;
        memcpy(&out->arm_hand_shortfall_worst, &v, sizeof v);
        out->arm_rest_recaptured = g_b.c_arm_rest_recaptured;
        v = g_b.s_arm_base_drift;
        memcpy(&out->arm_base_drift_deg, &v, sizeof v);
        v = g_b.s_arm_base_drift_worst;
        memcpy(&out->arm_base_drift_worst_deg, &v, sizeof v);
        v = g_b.s_arm_cmd_drift;
        memcpy(&out->arm_cmd_drift_deg, &v, sizeof v);
        v = g_b.s_arm_cmd_drift_worst;
        memcpy(&out->arm_cmd_drift_worst_deg, &v, sizeof v);
        v = g_b.s_arm_body_drift;
        memcpy(&out->arm_body_drift_deg, &v, sizeof v);
        v = g_b.s_arm_hand_raw_twist;
        memcpy(&out->arm_hand_raw_twist_deg, &v, sizeof v);
        v = g_b.s_arm_hand_fore_twist;
        memcpy(&out->arm_hand_fore_twist_deg, &v, sizeof v);
        v = g_b.s_arm_hand_wrist_swing;
        memcpy(&out->arm_hand_wrist_swing_deg, &v, sizeof v);
        v = g_b.s_arm_hand_wrist_twist;
        memcpy(&out->arm_hand_wrist_twist_deg, &v, sizeof v);
    }
    out->arm_map_flags = (unsigned int)g_b.s_arm_map_flags;
    out->arm_map_pair = (unsigned long)(DWORD)g_b.s_arm_map_pair;
    out->arm_map_stream = (unsigned long)(DWORD)g_b.s_arm_map_stream;
    {
        LONG v = g_b.s_arm_ik_weight;
        int i;
        memcpy(&out->arm_ik_weight, &v, sizeof v);
        for (i = 0; i < 3; i++) {
            v = g_b.s_arm_ik_view[i];
            memcpy(&out->arm_ik_view[i], &v, sizeof v);
            v = g_b.s_arm_ik_root[i];
            memcpy(&out->arm_ik_root[i], &v, sizeof v);
            v = g_b.s_arm_ik_target[i];
            memcpy(&out->arm_ik_target[i], &v, sizeof v);
        }
        v = g_b.s_arm_ik_root_distance;
        memcpy(&out->arm_ik_root_distance, &v, sizeof v);
        v = g_b.s_arm_ik_root_limit;
        memcpy(&out->arm_ik_root_limit, &v, sizeof v);
        v = g_b.s_arm_map_scale;
        memcpy(&out->arm_map_scale, &v, sizeof v);
        v = g_b.s_arm_map_source_span;
        memcpy(&out->arm_map_source_span, &v, sizeof v);
        v = g_b.s_arm_map_reach;
        memcpy(&out->arm_map_reach, &v, sizeof v);
        for (i = 0; i < 3; i++) {
            v = g_b.s_arm_map_delta[i];
            memcpy(&out->arm_map_delta[i], &v, sizeof v);
            v = g_b.s_arm_map_target[i];
            memcpy(&out->arm_map_target[i], &v, sizeof v);
            v = g_b.s_arm_shoulder_view[i];
            memcpy(&out->arm_shoulder_view[i], &v, sizeof v);
            v = g_b.s_arm_native_wrist_view[i];
            memcpy(&out->arm_native_wrist_view[i], &v, sizeof v);
        }
    }
    out->arm_anchor = (int)InterlockedCompareExchange(&g_b.arm_anchor, 0, 0);
    out->arm_pose_flags = (unsigned int)g_b.s_arm_pose_flags;
    out->arm_bend_joint = g_b.bend_joint;
    out->arm_bend_deg = g_b.bend_deg;
    out->arm_mctrl = ((ULONGLONG)(DWORD)g_b.s_arm_mctrl_hi << 32)
                   | (DWORD)g_b.s_arm_mctrl_lo;
    {
        int i;
        for (i = 0; i < 10; i++)
            out->arm_mctrl_head[i] =
                ((ULONGLONG)(DWORD)g_b.s_mctrl_dump[i * 2 + 1] << 32)
                | (DWORD)g_b.s_mctrl_dump[i * 2];
        for (i = 0; i < 8; i++)
            out->arm_obj_head[i] =
                ((ULONGLONG)(DWORD)g_b.s_obj_dump[i * 2 + 1] << 32)
                | (DWORD)g_b.s_obj_dump[i * 2];
    }
    out->skel_stride = (int)g_b.skel_stride;
    out->skel_stride_score = (int)g_b.skel_stride_score;
    out->skel_stride_tried = (int)g_b.skel_stride_tried;
    out->skel_n_models = (int)g_b.skel_n_models;
    out->skel_parents_read = (int)g_b.skel_parents_read;
    out->skel_chain_len = (int)g_b.skel_chain_len;
    out->skel_base = (int)g_b.skel_base;
    out->skel_region_end = (unsigned int)g_b.skel_region_end;
    out->skel_mctrl_trans = ((ULONGLONG)(DWORD)g_b.skel_mctrl_trans_hi << 32)
                          | (DWORD)g_b.skel_mctrl_trans_lo;
    out->skel_objs = ((ULONGLONG)(DWORD)g_b.skel_objs_hi << 32)
                   | (DWORD)g_b.skel_objs_lo;
    {
        int i;
        for (i = 0; i < DG_SKEL_MAX; i++)
            out->skel_parents[i] = (short)g_b.skel_parents[i];
        for (i = 0; i < DG_SKEL_CHAIN; i++) {
            LONG v;
            out->skel_chain[i] = (short)g_b.skel_chain[i];
            v = g_b.skel_chain_pos[i * 3 + 0];
            memcpy(&out->skel_chain_pos[i][0], &v, sizeof v);
            v = g_b.skel_chain_pos[i * 3 + 1];
            memcpy(&out->skel_chain_pos[i][1], &v, sizeof v);
            v = g_b.skel_chain_pos[i * 3 + 2];
            memcpy(&out->skel_chain_pos[i][2], &v, sizeof v);
        }
        for (i = 0; i < DG_SKEL_WINDOW; i++) {
            LONG v;
            v = g_b.skel_window_pos[i * 3 + 0];
            memcpy(&out->skel_window_pos[i][0], &v, sizeof v);
            v = g_b.skel_window_pos[i * 3 + 1];
            memcpy(&out->skel_window_pos[i][1], &v, sizeof v);
            v = g_b.skel_window_pos[i * 3 + 2];
            memcpy(&out->skel_window_pos[i][2], &v, sizeof v);
        }
    }
    out->adj_probe_joint = (int)g_b.adj_joint;
    {
        int i, k, c;
        for (i = 0; i < DG_ADJ_JOINTS; i++)
            out->adj_probe_indices[i] = (int)g_b.adj_indices[i];
        for (c = 0; c < DG_ADJ_CASES; c++)
            for (i = 0; i < 4; i++) {
                LONG v = g_b.adj_quat[c * 4 + i];
                memcpy(&out->adj_probe_quat[c][i], &v, sizeof v);
            }
        out->adj_probe_held = g_b.c_adj_held;
        for (c = 0; c < DG_ADJ_CASES; c++) {
            int ph, f;
            for (ph = 0; ph < 2; ph++)
                for (f = 0; f < 5; f++)
                    out->adj_probe_words[c][ph][f] =
                        (long)g_b.adj_words[(c * 2 + ph) * 5 + f];
        }
        for (c = 0; c < DG_ADJ_CASES; c++) {
            out->adj_probe_samples[c] = g_b.adj_samples[c];
            for (k = 0; k < DG_ADJ_JOINTS; k++)
                for (i = 0; i < 16; i++) {
                    LONG v = g_b.adj_world[(c * DG_ADJ_JOINTS + k) * 16 + i];
                    memcpy(&out->adj_probe_world[c][k][i], &v, sizeof v);
                }
        }
    }
    out->seen_pad_status = (unsigned int)g_b.seen_pad_status;
    out->pad_weapon_mask = (unsigned int)RD32(g_b.a.pad_weapon);
    out->weapon_presses = g_b.c_weapon_press;
    out->weapon_presses_in_fps = g_b.c_weapon_press_fps;
    {
        int i;
        for (i = 0; i < 4; i++) out->weapon_vk[i] = (int)g_b.s_weapon_vk[i];
    }
    out->fire_mode = (int)g_b.fire_mode;
    out->fire_published = g_b.c_fire_published;
    out->fire_no_command = g_b.c_fire_no_command;
    out->fire_stale = g_b.c_fire_stale;
    out->fire_drawn = g_b.c_fire_drawn;
    out->fire_released = g_b.c_fire_released;
    out->fire_aborted = g_b.c_fire_aborted;
    out->fire_forced = g_b.c_fire_forced;
    out->fire_blocked_phys = g_b.c_fire_blocked_phys;
    out->fire_blocked_gate = g_b.c_fire_blocked_gate;
    out->fire_repeats = g_b.c_fire_repeats;
    out->fire_coasting = g_b.c_fire_coasting;
    out->fire_auto_ticks = g_b.c_fire_auto_ticks;
    out->fire_wrote_press = g_b.c_fire_wrote_press;
    out->fire_wrote_status = g_b.c_fire_wrote_status;
    out->fire_wrote_release = g_b.c_fire_wrote_release;
    out->fire_wrote_pressure = g_b.c_fire_wrote_pressure;
    out->fire_pressure_kept = g_b.c_fire_pressure_kept;
    out->fire_no_index = g_b.c_fire_no_index;
    out->fire_yielded = g_b.c_fire_yielded;
    out->fire_press_index = (int)g_b.s_fire_index;
    out->weapon_state_seen = (unsigned int)g_b.seen_weapon_state;
    out->button_state_seen = (unsigned int)g_b.seen_button_state;
    out->weapon_state = (int)g_b.s_weapon_state;
    out->button_state = (int)g_b.s_button_state;
    out->fire_state = (int)g_b.s_fire_state;
    out->fire_pressure = (int)g_b.s_fire_pressure;
    out->fire_wtype = (unsigned int)g_b.s_fire_wtype;
    out->recoil_climb_mdeg = (int)g_b.recoil_climb_mdeg;
    out->recoil_push_um = (int)g_b.recoil_push_um;
    out->recoil_kicks = g_b.c_recoil_kicks;
    out->recoil_climb_writes = g_b.c_recoil_climb_writes;
    out->recoil_no_axis = g_b.c_recoil_no_axis;
    out->recoil_push_writes = g_b.c_recoil_push_writes;
    out->recoil_push_refused = g_b.c_recoil_push_refused;
    out->recoil_amplitude = (float)g_b.s_recoil_amp / 1000.0f;
    out->recoil_worst = (float)g_b.s_recoil_worst / 1000.0f;
    out->restored = (int)g_b.restored;
    out->thea_samples = g_b.c_thea_samples;
    out->thea_masked = g_b.c_thea_masked;
    out->thea_ui_masked = g_b.c_thea_ui_masked;
    out->thea_enter = g_b.c_thea_enter;
    out->thea_exit = g_b.c_thea_exit;
    out->thea_ui_enter = g_b.c_thea_ui_enter;
    out->thea_ui_exit = g_b.c_thea_ui_exit;
    out->thea_enter_held = g_b.c_thea_enter_held;
    out->thea_exit_held = g_b.c_thea_exit_held;
    out->thea_bit_demo = g_b.c_thea_bit_demo;
    out->thea_bit_scn = g_b.c_thea_bit_scn;
    out->thea_bit_pad = g_b.c_thea_bit_pad;
    out->thea_bit_radio = g_b.c_thea_bit_radio;
    out->thea_bit_weapon = g_b.c_thea_bit_weapon;
    out->thea_bit_item = g_b.c_thea_bit_item;
    out->thea_verdict = (int)g_b.s_theater;
    out->thea_mode = (int)g_b.theater_mode;
    out->thea_ui_on = (int)g_b.theater_ui_mode;
    out->hud_hide = (int)g_b.hud_mode;
    out->hud_writes = g_b.c_hud_writes;
    out->hud_cleared = g_b.c_hud_cleared;
}

/* ====================================================== desk tests ======= */

#ifdef DG_HOOK_TEST

static int t_camera_pair_telemetry(void)
{
    DG_REC_PAIRSTATE saved_pair = g_b.rec_pair, expect;
    MAT eye, pers;
    LONG saved_armed = InterlockedCompareExchange(&g_b.armed, 0, 0);
    int i, j, bad = 0;

    memset(&eye, 0, sizeof eye);
    eye.m[0][0] = eye.m[1][1] = eye.m[2][2] = eye.m[3][3] = 1.0f;
    memset(&pers, 0, sizeof pers);
    pers.m[0][0] = 1.7f;
    pers.m[1][1] = 2.3f;
    pers.m[2][3] = 1.0f;

    /* A valid camera is accepted only on a pair that really read its frames. */
    memset(&g_b.rec_pair, 0, sizeof g_b.rec_pair);
    g_b.rec_pair.flags = DG_REC_PAIR_F_FRAMES;
    dg_bridge_rec_camera(&eye, &pers);
    if (!(g_b.rec_pair.flags & DG_REC_PAIR_F_CAMERA)) bad++;
    for (i = 0; i < 3; i++) for (j = 0; j < 3; j++)
        if (g_b.rec_pair.camera_world[i][j] != eye.m[i][j]) bad++;
    if (g_b.rec_pair.camera_proj[0] != pers.m[0][0] ||
        g_b.rec_pair.camera_proj[1] != pers.m[1][1] ||
        g_b.rec_pair.camera_proj[2] != pers.m[2][3]) bad++;

    /* The helper must fail closed when the pair has no FRAMES publication. */
    memset(&g_b.rec_pair, 0, sizeof g_b.rec_pair);
    dg_bridge_rec_camera(&eye, &pers);
    if (g_b.rec_pair.flags & DG_REC_PAIR_F_CAMERA) bad++;
    for (i = 0; i < 3; i++) for (j = 0; j < 3; j++)
        if (g_b.rec_pair.camera_world[i][j] != 0.0f) bad++;
    for (i = 0; i < 3; i++)
        if (g_b.rec_pair.camera_proj[i] != 0.0f) bad++;

    /* The arm seam is also the lifecycle boundary: a disarmed early return
       clears every stale field, retaining only the explicit -1 absence value. */
    memset(&g_b.rec_pair, 0xA5, sizeof g_b.rec_pair);
    g_b.rec_pair.flags = DG_REC_PAIR_F_FRAMES | DG_REC_PAIR_F_CAMERA;
    g_b.rec_pair.camera_world[0][0] = 9.0f;
    InterlockedExchange(&g_b.armed, 0);
    dg_bridge_arm_seam_now(NULL);
    memset(&expect, 0, sizeof expect);
    expect.rest_drift_deg = -1.0f;
    if (memcmp(&g_b.rec_pair, &expect, sizeof expect) != 0) bad++;

    g_b.rec_pair = saved_pair;
    InterlockedExchange(&g_b.armed, saved_armed);
    printf("  %-6s camera pair: FRAMES-gated attach, absent camera fails closed, "
           "and disarmed arm seam clears stale pair\n", bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

static void t_input(DG_FPS_INPUT *in, int mode)
{
    memset(in, 0, sizeof(*in));
    in->mode = mode;
    in->native_override = 1;
    in->native_toggle = 1;
    in->pad_subject_mask = 0x0040;          /* a plausible PL_PAD_SUBJECT */
    in->pad_stop_aim_mask = 0x0008;         /* a plausible PL_PAD_STOP_AIM */
    in->safe_gameplay = 1;
}

static int t_off_never_writes(void)
{
    DG_FPS_STATE s;
    DG_FPS_INPUT in;
    DG_FPS_STEP out;
    int i;
    int writes = 0;
    dg_fps_init(&s);
    for (i = 0; i < 200; i++) {
        t_input(&in, DG_FPS_MODE_OFF);
        in.native_override = i & 1;
        in.native_toggle = (i >> 1) & 1;
        in.native_active = (i >> 2) & 1;
        in.subject_move = i % 4;
        in.pad_subject_mask = (i & 4) ? 0 : 0x40;
        in.safe_gameplay = (i & 8) == 0;
        in.toggle_request = (i & 16) != 0;
        in.level_load = (i & 32) != 0;
        dg_fps_step(&s, &in, &out);
        if (out.write != DG_FPS_WRITE_NONE) writes++;
    }
    printf("  %-6s off produces no write in 200 varied ticks (%d)\n",
           writes ? "FAIL" : "ok", writes);
    return writes ? 1 : 0;
}

/* Drives the machine the way the game would: an injected edge is confirmed by
   the native state flipping one tick later, exactly as CheckWatch would. */
static int t_toggle_ten_cycles(void)
{
    DG_FPS_STATE s;
    DG_FPS_INPUT in;
    DG_FPS_STEP out;
    int native = 0;
    int cycle;
    int edges = 0;
    int enters = 0;
    int leaves = 0;
    int bad = 0;
    int tick;

    dg_fps_init(&s);
    for (cycle = 0; cycle < 10; cycle++) {
        int phase;
        for (phase = 0; phase < 2; phase++) {
            int want = phase == 0 ? 1 : 0;
            int pending = 0;
            for (tick = 0; tick < 20; tick++) {
                t_input(&in, DG_FPS_MODE_TOGGLE);
                in.native_active = native;
                in.toggle_request = (tick == 0);
                dg_fps_step(&s, &in, &out);
                if (out.write == DG_FPS_WRITE_SUBJECT_EDGE) {
                    edges++;
                    pending = 1;
                } else if (out.write != DG_FPS_WRITE_NONE) {
                    bad++;               /* Override/Toggle already correct */
                } else if (pending) {
                    native = !native;    /* CheckWatch consumed the edge */
                    pending = 0;
                }
                if (native == want && s.state == DG_FPS_ACTIVE) enters++;
                if (native == want && want == 0 && s.state == DG_FPS_OFF)
                    leaves++;
                if (native == want && (s.state == DG_FPS_ACTIVE ||
                                       s.state == DG_FPS_OFF))
                    break;
            }
            if (native != want) bad++;
        }
    }
    if (edges != 20) bad++;
    if (s.state != DG_FPS_OFF) bad++;
    printf("  %-6s toggle: 10 enter/leave cycles, %d edges, %d enters, "
           "%d leaves, end %s\n",
           bad ? "FAIL" : "ok", edges, enters, leaves,
           dg_fps_state_name(s.state));
    return bad ? 1 : 0;
}

static int t_move_is_borrowed_only_on_request(void)
{
    DG_FPS_STATE s;
    DG_FPS_INPUT in;
    DG_FPS_STEP out;
    int tick, moves, edge_at, move_at, bad = 0;
    int native_move;

    /* NATIVE: a full enter, hold and leave, with Move never touched. */
    dg_fps_init(&s);
    moves = 0;
    native_move = 1;
    for (tick = 0; tick < 30; tick++) {
        t_input(&in, DG_FPS_MODE_TOGGLE);
        in.move_mode = DG_FPS_MOVE_NATIVE;
        in.native_move = native_move;
        in.saved_move = 1;
        in.native_active = (tick >= 3 && tick < 20);
        in.toggle_request = (tick == 0 || tick == 18);
        dg_fps_step(&s, &in, &out);
        if (out.write == DG_FPS_WRITE_MOVE) moves++;
    }
    if (moves != 0) bad++;

    /* OFF: Move must go down before the edge, and come back afterwards. */
    dg_fps_init(&s);
    edge_at = move_at = -1;
    native_move = 1;
    for (tick = 0; tick < 40; tick++) {
        t_input(&in, DG_FPS_MODE_TOGGLE);
        in.move_mode = DG_FPS_MOVE_OFF;
        in.native_move = native_move;
        in.saved_move = 1;
        in.native_active = (tick >= 8 && tick < 20);
        in.toggle_request = (tick == 0 || tick == 18);
        dg_fps_step(&s, &in, &out);
        if (out.write == DG_FPS_WRITE_MOVE) {
            native_move = out.write_value;
            if (move_at < 0 && out.write_value == 0) move_at = tick;
        }
        if (out.write == DG_FPS_WRITE_SUBJECT_EDGE && edge_at < 0)
            edge_at = tick;
    }
    if (move_at < 0) bad++;                       /* it was never lowered */
    if (edge_at < 0 || move_at >= edge_at) bad++; /* or lowered too late  */
    if (native_move != 1) bad++;                  /* or never handed back */

    dg_fps_init(&s);
    edge_at = move_at = -1;
    native_move = 0;
    for (tick = 0; tick < 40; tick++) {
        t_input(&in, DG_FPS_MODE_TOGGLE);
        in.move_mode = DG_FPS_MOVE_ON;
        in.native_move = native_move;
        in.saved_move = 0;
        in.native_active = (tick >= 8 && tick < 20);
        in.toggle_request = (tick == 0 || tick == 18);
        dg_fps_step(&s, &in, &out);
        if (out.write == DG_FPS_WRITE_MOVE) {
            native_move = out.write_value;
            if (move_at < 0 && out.write_value == 1) move_at = tick;
        }
        if (out.write == DG_FPS_WRITE_SUBJECT_EDGE && edge_at < 0)
            edge_at = tick;
    }
    if (move_at < 0) bad++;                       /* it was never raised  */
    if (edge_at < 0 || move_at >= edge_at) bad++; /* or raised too late   */
    if (native_move != 0) bad++;                  /* or never handed back */

    printf("  %-6s Move is left alone by default, driven to 0 or 1 before the "
           "edge on request and handed back after\n", bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

/* The engine drops out of first person on its own while the bridge holds a
   toggle. That happens in ordinary play - the first run with a weapon equipped
   did it six times - and the bridge used to answer every one of them with a
   fresh synthetic press of the game's own first person button. Two requests
   produced eight edges and seven entries, delivered while the player was
   aiming.
   In TOGGLE the press meant "toggle now", so the engine leaving IS the answer.
   In ALWAYS holding it through is the entire contract, so there the re-entry
   has to survive. One test, both modes, because the bug was treating them
   alike. */
static int t_engine_leaving_is_not_a_fight(void)
{
    DG_FPS_STATE s;
    DG_FPS_INPUT in;
    DG_FPS_STEP out;
    int tick, edges, bad = 0;

    /* TOGGLE: one request in, first person reached, then the engine lets go. */
    {
        DG_FPS_RIG rig={0};
        if(fps_rig_changed(&rig,100,1,1))bad++;
        if(fps_rig_changed(&rig,0,1,1))bad++; /* loading: unknown */
        if(!fps_rig_changed(&rig,200,1,1))bad++; /* new rig, Active retained */
        if(!fps_rig_changed(&rig,200,1,1))bad++; /* stays pending */
        if(fps_rig_changed(&rig,0,1,0))bad++; /* native leave acknowledged */
        if(fps_rig_changed(&rig,200,1,1))bad++;
        if(fps_rig_changed(&rig,300,0,1))bad++; /* explicit OFF */
        dg_fps_init(&s);t_input(&in,DG_FPS_MODE_TOGGLE);
        in.native_active=1;s.state=DG_FPS_ACTIVE;rig.arm=300;rig.pending=0;
        if(!fps_calibration_ready(&s,&in,&rig))bad++;
        /* Reproduce a new arm while native Active remains set. No recenter
           during the recovery delay or either injected FPS edge. */
        fps_rig_changed(&rig,400,1,1);
        if(fps_calibration_ready(&s,&in,&rig))bad++;
        rig.pending=0;
        s.state=DG_FPS_REQUEST_LEAVE;if(fps_calibration_ready(&s,&in,&rig))bad++;
        s.state=DG_FPS_OFF;if(fps_calibration_ready(&s,&in,&rig))bad++;
        s.state=DG_FPS_REQUEST_ENTER;if(fps_calibration_ready(&s,&in,&rig))bad++;
        s.state=DG_FPS_ACTIVE;if(!fps_calibration_ready(&s,&in,&rig))bad++;
        in.level_load=1;if(fps_calibration_ready(&s,&in,&rig))bad++;
        in.level_load=0;in.native_camera_missing=1;if(fps_calibration_ready(&s,&in,&rig))bad++;
        in.native_camera_missing=0;in.safe_gameplay=0;if(fps_calibration_ready(&s,&in,&rig))bad++;
        in.safe_gameplay=1;in.native_active=0;if(fps_calibration_ready(&s,&in,&rig))bad++;
        in.native_active=1;s.camera_missing_ticks=1;if(fps_calibration_ready(&s,&in,&rig))bad++;
    }
    dg_fps_init(&s);
    edges = 0;
    for (tick = 0; tick < 40; tick++) {
        t_input(&in, DG_FPS_MODE_TOGGLE);
        /* Active for the middle stretch only: the engine takes it away at 20
           without anyone asking, exactly as aiming appears to. */
        in.native_active = (tick >= 5 && tick < 20);
        in.toggle_request = (tick == 0);
        dg_fps_step(&s, &in, &out);
        if (out.write == DG_FPS_WRITE_SUBJECT_EDGE) edges++;
    }
    /* One edge, for the press the player actually made. */
    if (edges != 1) bad++;
    if (s.state != DG_FPS_SUSPENDED || !s.desired) bad++;
    /* After the native exit settles, restore once. Explicit A cancels even
       during recovery; an unsafe interval never emits a subject edge. */
    for (tick=0; tick<DG_FPS_RETRY_TICKS+1; tick++) {
        t_input(&in,DG_FPS_MODE_TOGGLE);
        dg_fps_step(&s,&in,&out);
        if(out.write==DG_FPS_WRITE_SUBJECT_EDGE) edges++;
    }
    if(edges!=2) bad++;
    in.native_active=1;dg_fps_step(&s,&in,&out);
    in.native_active=0;dg_fps_step(&s,&in,&out);
    in.toggle_request=1;dg_fps_step(&s,&in,&out);
    if(s.desired || out.write==DG_FPS_WRITE_SUBJECT_EDGE)bad++;

    /* A level retained Active but its readable native camera is OFF. Short
       gaps/unsafe intervals do not act; sustained loss uses native off/on. */
    dg_fps_init(&s); s.state=DG_FPS_ACTIVE; s.desired=1; s.owned=1;
    t_input(&in,DG_FPS_MODE_TOGGLE); in.native_active=1;
    in.native_camera_missing=1;
    for(tick=0;tick<DG_FPS_RETRY_TICKS-1;tick++) {
        dg_fps_step(&s,&in,&out);
        if(out.write==DG_FPS_WRITE_SUBJECT_EDGE)bad++;
    }
    in.safe_gameplay=0;dg_fps_step(&s,&in,&out);
    if(out.write || s.camera_missing_ticks || !s.desired)bad++;
    in.safe_gameplay=1;dg_fps_step(&s,&in,&out);
    for(tick=0;tick<DG_FPS_RETRY_TICKS;tick++) dg_fps_step(&s,&in,&out);
    if(out.write!=DG_FPS_WRITE_SUBJECT_EDGE ||
       s.state!=DG_FPS_REQUEST_LEAVE || !s.desired)bad++;
    in.native_active=0;dg_fps_step(&s,&in,&out);
    if(s.state!=DG_FPS_OFF || !s.desired)bad++;
    dg_fps_step(&s,&in,&out);
    if(out.write!=DG_FPS_WRITE_SUBJECT_EDGE || s.state!=DG_FPS_REQUEST_ENTER)bad++;
    in.native_active=1;in.native_camera_missing=0;dg_fps_step(&s,&in,&out);
    if(s.state!=DG_FPS_ACTIVE || !s.desired)bad++;
    /* A healthy camera and an explicitly OFF preference never recover. */
    for(tick=0;tick<DG_FPS_RETRY_TICKS+5;tick++) {
        dg_fps_step(&s,&in,&out);if(out.write)bad++;
    }
    s.desired=0;s.owned=0;in.native_camera_missing=1;
    for(tick=0;tick<DG_FPS_RETRY_TICKS+5;tick++) {
        dg_fps_step(&s,&in,&out);if(out.write)bad++;
    }

    /* ALWAYS: the same departure, and here the bridge must take it back. */
    dg_fps_init(&s);
    edges = 0;
    for (tick = 0; tick < 40; tick++) {
        t_input(&in, DG_FPS_MODE_ALWAYS);
        in.native_active = (tick >= 5 && tick < 20);
        dg_fps_step(&s, &in, &out);
        if (out.write == DG_FPS_WRITE_SUBJECT_EDGE) edges++;
    }
    if (edges < 2) bad++;

    printf("  %-6s VR toggle survives native exit, delayed restore and explicit cancellation\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

/* The player uses the game's own first person button while the bridge is armed
   but has never been asked for anything. This is the F2 live gate: armed, no
   toggle, play normally. The native control is a hold, so first person comes
   and goes many times, and every one of those is the player's. The bridge must
   follow without ever emitting a subject edge - the first live run bounced the
   player out of first person sixteen times because OFF treated the mismatch as
   an error and forced a leave. */
static int t_native_fps_is_left_alone(void)
{
    DG_FPS_STATE s;
    DG_FPS_INPUT in;
    DG_FPS_STEP out;
    int hold, tick;
    int edges = 0;
    int bad = 0;
    int saw_active = 0;
    int saw_off = 0;

    dg_fps_init(&s);
    for (hold = 0; hold < 16; hold++) {
        /* Held for a while, then released for a while. */
        for (tick = 0; tick < 24; tick++) {
            t_input(&in, DG_FPS_MODE_TOGGLE);
            /* Idle means the game's own values, not ours: Override and Toggle
               are back at the arm-time zeros, so the button is still a hold. */
            in.native_override = 0;
            in.native_toggle = 0;
            in.native_active = tick < 12;
            dg_fps_step(&s, &in, &out);
            if (out.write != DG_FPS_WRITE_NONE) edges++;
            if (s.held) bad++;         /* nothing raised, nothing owed back */
            if (in.native_active && s.state == DG_FPS_ACTIVE) saw_active = 1;
            if (!in.native_active && s.state == DG_FPS_OFF) saw_off = 1;
            if (s.desired) bad++;      /* nobody asked for first person */
            if (s.owned) bad++;        /* and we never entered it */
        }
    }
    if (edges) bad++;
    if (!saw_active || !saw_off) bad++;
    if (s.state != DG_FPS_OFF) bad++;
    printf("  %-6s native first person: 16 holds, %d writes, followed %d/%d, "
           "end %s\n",
           bad ? "FAIL" : "ok", edges, saw_active, saw_off,
           dg_fps_state_name(s.state));
    return bad ? 1 : 0;
}

/* Override and Toggle are borrowed, not taken: raised when a request arrives
   and handed back once the camera is out again. The order is the whole point -
   handing Toggle back while first person is still up would leave the engine
   reading a hold it never got. Drives the native side the way CheckWatch does,
   confirming an injected edge one tick later. */
static int t_claim_is_released_when_idle(void)
{
    DG_FPS_STATE s;
    DG_FPS_INPUT in;
    DG_FPS_STEP out;
    int nover = 0, ntog = 0, nactive = 0, prev_active = 0;
    int subj_toggle = 0;
    int pending = 0;
    int bad = 0;
    int entered = 0;
    int left_tick = -1;
    int released_tick = -1;
    int tick;

    dg_fps_init(&s);
    for (tick = 0; tick < 200; tick++) {

        if (nover || nactive) {
            if (nover) subj_toggle = ntog;
            if (pending) { nactive = !nactive; pending = 0; }
        }
        if (prev_active && !nactive) left_tick = tick;
        if (nactive) entered = 1;
        prev_active = nactive;

        t_input(&in, DG_FPS_MODE_TOGGLE);
        in.native_override = nover;
        in.native_toggle = ntog;
        in.native_active = nactive;
        in.toggle_request = (tick == 10 || tick == 60);   /* on, then off */
        dg_fps_step(&s, &in, &out);

        if (tick < 10 && (out.write != DG_FPS_WRITE_NONE || s.held))
            bad++;                          /* idle must cost nothing at all */

        if ((out.write == DG_FPS_WRITE_OVERRIDE ||
             out.write == DG_FPS_WRITE_TOGGLE) && out.write_value == 0) {
            if (nactive) bad++;             /* handed back mid first person */
            if (released_tick < 0) released_tick = tick;
        }

        switch (out.write) {
        case DG_FPS_WRITE_OVERRIDE: nover = out.write_value; break;
        case DG_FPS_WRITE_TOGGLE: ntog = out.write_value; break;
        case DG_FPS_WRITE_SUBJECT_EDGE: pending = 1; break;
        default: break;
        }
    }

    if (!entered) bad++;
    if (left_tick < 0 || released_tick < left_tick) bad++;
    if (nover != 0 || ntog != 0) bad++;     /* back to the arm-time values */
    /* The shadow has to come back too, and only the engine can put it back:
       release Override before Toggle and this stays 1 for the rest of the
       session, with nothing in the values we own to show for it. */
    if (subj_toggle != 0) bad++;
    if (s.held || s.owned || s.desired) bad++;
    if (s.state != DG_FPS_OFF) bad++;
    printf("  %-6s Override/Toggle raised on request, handed back at tick %d "
           "after the camera left at %d, PL_SubjectToggle back to %d\n",
           bad ? "FAIL" : "ok", released_tick, left_tick, subj_toggle);
    return bad ? 1 : 0;
}

/* The live gate's step C stopped after exactly one toggle: nine ticks after the
   camera came back out, the bridge suspended on MGSHDFIX_OWNER and stayed there,
   because handing Override back looks identical to a second owner clearing it
   unless the test knows whether we were holding it.

   Both halves matter. Idling must never latch, and a real second owner must
   still latch - a detector that has stopped detecting is the worse bug of the
   two, since it ends in two writers of one variable. */
static int t_idle_is_not_a_second_owner(void)
{
    DG_FPS_STATE s;
    DG_FPS_INPUT in;
    DG_FPS_STEP out;
    int nover = 0, ntog = 0, nactive = 0;
    int pending = 0, fights = 0;
    int bad = 0;
    int latched_tick = -1;
    int contested_tick = -1;
    int tick;

    /* Half one: the exact shape of the live run - one toggle in, one toggle
       out, then idle. Memory keeps whatever we write, as the game did. */
    dg_fps_init(&s);
    for (tick = 0; tick < 300; tick++) {
        if (pending) { nactive = !nactive; pending = 0; }
        if (latched_tick < 0 && fight_step(s.held, nover, &fights))
            latched_tick = tick;

        t_input(&in, DG_FPS_MODE_TOGGLE);
        in.native_override = nover;
        in.native_toggle = ntog;
        in.native_active = nactive;
        in.toggle_request = (tick == 10 || tick == 60);
        dg_fps_step(&s, &in, &out);

        switch (out.write) {
        case DG_FPS_WRITE_OVERRIDE: nover = out.write_value; break;
        case DG_FPS_WRITE_TOGGLE: ntog = out.write_value; break;
        case DG_FPS_WRITE_SUBJECT_EDGE: pending = 1; break;
        default: break;
        }
    }
    if (latched_tick >= 0) bad++;

    /* Half two: someone else clears Override every frame while we are trying to
       hold it. That has to latch, and quickly. */
    dg_fps_init(&s);
    nover = ntog = nactive = pending = fights = 0;
    for (tick = 0; tick < 60; tick++) {
        if (contested_tick < 0 && fight_step(s.held, nover, &fights))
            contested_tick = tick;

        t_input(&in, DG_FPS_MODE_TOGGLE);
        in.native_override = nover;
        in.native_toggle = ntog;
        in.native_active = nactive;
        in.toggle_request = (tick == 1);
        dg_fps_step(&s, &in, &out);

        if (out.write == DG_FPS_WRITE_TOGGLE) ntog = out.write_value;
        nover = 0;                      /* the other owner wins every frame */
    }
    if (contested_tick < 0 || contested_tick > 16) bad++;

    printf("  %-6s idle never reads as a second owner (latched %d), a real one "
           "still does by tick %d\n",
           bad ? "FAIL" : "ok", latched_tick, contested_tick);
    return bad ? 1 : 0;
}

static int t_player_status_mask(void)
{
    static const struct { ULONGLONG bit; const char *name; int unsafe; }
    cases[] = {
        { 0x0000000000000080ULL, "PLAYER_LOCKER",    1 },
        { 0x0000000000000400ULL, "PLAYER_DOWNED",    1 },
        { 0x0000000000000800ULL, "PLAYER_HOLD",      0 },
        { 0x0000000000008000ULL, "PLAYER_DEAD",      1 },
        { 0x0000000008000000ULL, "PLAYER_MENU_OPEN", 1 },
        { 0x0000000010000000ULL, "PLAYER_STOP",      1 },
        { 0x0000020000000000ULL, "PLAYER_PAD_OFF",   1 }
    };
    int bad = 0;
    size_t i;

    for (i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        int refused = (cases[i].bit & DG_PLAYER_UNSAFE_MASK) != 0;
        if (refused != cases[i].unsafe) {
            bad++;
            printf("  FAIL   %s: refused %d, wanted %d\n",
                   cases[i].name, refused, cases[i].unsafe);
        }
    }
    printf("  %-6s player-status mask 0x%016llX permits weapon-ready HOLD "
           "and still refuses takeover, death, menu and pad-off states\n",
           bad ? "FAIL" : "ok",
           (unsigned __int64)DG_PLAYER_UNSAFE_MASK);
    return bad ? 1 : 0;
}

/* The game-status mask is the whole of the new decision, so it is pinned bit by
   bit rather than left to whoever next edits the macro. Both directions matter:
   a bit that should suspend and does not is a camera taken over mid-cutscene,
   and a bit that suspends when it should not is first person refused during an
   alert - which is most of the game. */
static int t_game_status_mask(void)
{
    static const struct { unsigned int bit; const char *name; int unsafe; }
    cases[] = {
        { 0x00000040u, "STATE_CUT_IN",        1 },
        { 0x00004000u, "STATE_DISP_GAMEOVER", 1 },
        { 0x08000000u, "STATE_SCN_DEMO",      1 },
        { 0x10000000u, "STATE_DEMO",          1 },
        { 0x20000000u, "STATE_PRG_DEMO",      1 },
        { 0x40000000u, "STATE_PAD_DEMO",      1 },
        { 0x80000000u, "STATE_GAMEOVER",      1 },
        { 0x00000001u, "STATE_DETECT",        0 },
        { 0x00000002u, "STATE_CLEARING",      0 },
        { 0x00000010u, "STATE_CHAFF",         0 },
        { 0x00000020u, "STATE_STUN",          0 },
        { 0x00000080u, "STATE_RADAR_JAMMING", 0 },
        { 0x00000200u, "STATE_PAUSE_DISABLE", 0 },
        { 0x00400000u, "STATE_VR_ONLY",       0 },
        { 0x00800000u, "STATE_VR_ANOTHER",    0 },
        { 0x01000000u, "STATE_BOSS_SURVIVAL", 0 }
    };
    int bad = 0;
    size_t i;

    for (i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        int refused = (cases[i].bit & DG_GAME_UNSAFE_MASK) != 0;
        if (refused != cases[i].unsafe) {
            bad++;
            printf("  FAIL   %s: refused %d, wanted %d\n",
                   cases[i].name, refused, cases[i].unsafe);
        }
    }
    /* STATE_PLAY_DEMO is the composite the game itself tests, and it is the
       mask that made the anchor unique - so the four bits behind it have to be
       covered, all four, or the anchor and the policy disagree. */
    if ((0x78000000u & DG_GAME_UNSAFE_MASK) != 0x78000000u) {
        bad++;
        printf("  FAIL   STATE_PLAY_DEMO not fully covered\n");
    }
    printf("  %-6s game-status mask 0x%08X refuses demos, cut-in and gameover, "
           "and permits alert states\n",
           bad ? "FAIL" : "ok", (unsigned int)DG_GAME_UNSAFE_MASK);
    return bad ? 1 : 0;
}

/* The menu word, and the one bit in it this phase exists for. */
static int t_menu_status_mask(void)
{
    static const struct { unsigned int bit; const char *name; int unsafe; }
    cases[] = {
        { 0x00000400u, "MENU_RADIO_ON",       1 },   /* the codec */
        { 0x00000100u, "MENU_WEAPON_OPEN",    1 },
        { 0x00000200u, "MENU_ITEM_OPEN",      1 },
        { 0x00000004u, "MENU_RADAR_OFF",      0 },
        { 0x00000010u, "MENU_CAPTION_OFF",    0 },
        { 0x00000800u, "MENU_RADAR_ON",       0 },
        { 0x00001000u, "MENU_GAGE_ON",        0 },
        { 0x00008000u, "MENU_NODE_ON",        0 },
        { 0x00040000u, "MENU_RADIO_DISABLE",  0 },
        { 0x00100000u, "MENU_STREAM_CH_0",    0 },
        { 0x00200000u, "MENU_STREAM_CH_1",    0 }
    };
    int bad = 0;
    size_t i;

    for (i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        int refused = (cases[i].bit & DG_MENU_UNSAFE_MASK) != 0;
        if (refused != cases[i].unsafe) {
            bad++;
            printf("  FAIL   %s: refused %d, wanted %d\n",
                   cases[i].name, refused, cases[i].unsafe);
        }
    }
    /* MENU_RADIO_DISABLE is the trap worth naming: it means the radio cannot be
       opened, which is the opposite of a call being up. Refusing on it would
       suspend first person for exactly the stretches the game guarantees are
       uninterrupted. */
    if (DG_MENU_UNSAFE_MASK & 0x00040000u) {
        bad++;
        printf("  FAIL   MENU_RADIO_DISABLE read as if it were a call\n");
    }
    /* 0x700 is the game's own composite, not one we composed. */
    if ((unsigned int)DG_MENU_UNSAFE_MASK != 0x00000700u) {
        bad++;
        printf("  FAIL   menu mask drifted from the game's own 0x700\n");
    }
    printf("  %-6s menu-status mask 0x%08X refuses codec and open menus, and "
           "permits radar, gauge, node and stream bits\n",
           bad ? "FAIL" : "ok", (unsigned int)DG_MENU_UNSAFE_MASK);
    return bad ? 1 : 0;
}

/* Runs one input for n ticks, letting an injected edge confirm one tick later,
   and reports the state at the end. */
static void t_run(DG_FPS_STATE *s, DG_FPS_INPUT *in, int *native, int ticks)
{
    DG_FPS_STEP out;
    int pending = 0;
    int i;
    for (i = 0; i < ticks; i++) {
        in->native_active = *native;
        dg_fps_step(s, in, &out);
        if (out.write == DG_FPS_WRITE_SUBJECT_EDGE) pending = 1;
        else if (pending) { *native = !*native; pending = 0; }
    }
}

static int t_always_suspend_resume(void)
{
    static const struct { const char *name; int reason; } phases[] = {
        { "menu",     DG_FPS_REASON_UNSAFE },
        { "codec",    DG_FPS_REASON_UNSAFE },
        { "cutscene", DG_FPS_REASON_UNSAFE },
        { "load",     DG_FPS_REASON_LEVEL_LOAD },
        { "death",    DG_FPS_REASON_UNSAFE },
        { "mask0",    DG_FPS_REASON_MASK_ZERO }
    };
    DG_FPS_STATE s;
    DG_FPS_INPUT in;
    int native = 0;
    int bad = 0;
    size_t p;

    dg_fps_init(&s);
    t_input(&in, DG_FPS_MODE_ALWAYS);
    t_run(&s, &in, &native, 8);
    if (s.state != DG_FPS_ACTIVE || !native) bad++;

    for (p = 0; p < sizeof(phases) / sizeof(phases[0]); p++) {
        t_input(&in, DG_FPS_MODE_ALWAYS);
        switch (phases[p].reason) {
        case DG_FPS_REASON_LEVEL_LOAD:
            in.subject_move = 1;
            in.pad_subject_mask = 0;
            break;
        case DG_FPS_REASON_MASK_ZERO:
            in.pad_subject_mask = 0;
            break;
        default:
            in.safe_gameplay = 0;
            break;
        }
        t_run(&s, &in, &native, 5);
        if (s.state != DG_FPS_SUSPENDED || s.reason != phases[p].reason) {
            bad++;
            printf("  FAIL   always/%s -> %s(%s)\n", phases[p].name,
                   dg_fps_state_name(s.state), dg_fps_reason_name(s.reason));
        }
        /* The engine dropped first person during the interruption. */
        native = 0;
        t_input(&in, DG_FPS_MODE_ALWAYS);
        t_run(&s, &in, &native, 8);
        if (s.state != DG_FPS_ACTIVE || !native) {
            bad++;
            printf("  FAIL   always/%s did not resume (%s)\n", phases[p].name,
                   dg_fps_state_name(s.state));
        }
    }

    /* MGSHDFix owning the values is terminal, and produces no write at all. */
    {
        DG_FPS_STEP out;
        int writes = 0;
        int i;
        for (i = 0; i < 50; i++) {
            t_input(&in, DG_FPS_MODE_ALWAYS);
            in.mgshdfix_owner = 1;
            in.native_active = native;
            dg_fps_step(&s, &in, &out);
            if (out.write != DG_FPS_WRITE_NONE) writes++;
        }
        if (writes || s.state != DG_FPS_SUSPENDED ||
            s.reason != DG_FPS_REASON_MGSHDFIX_OWNER)
            bad++;
    }
    printf("  %-6s always suspends and resumes across menu/codec/cutscene/"
           "load/death/mask0 and never fights MGSHDFix\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

static int t_no_edge_without_toggle(void)
{
    DG_FPS_STATE s;
    DG_FPS_INPUT in;
    DG_FPS_STEP out;
    int bad = 0;
    int i;
    int edges = 0;
    dg_fps_init(&s);
    for (i = 0; i < 40; i++) {
        t_input(&in, DG_FPS_MODE_ALWAYS);
        in.native_override = 0;
        in.native_toggle = 0;
        dg_fps_step(&s, &in, &out);
        if (out.write == DG_FPS_WRITE_SUBJECT_EDGE) edges++;
        if (i == 0 && (out.write != DG_FPS_WRITE_OVERRIDE ||
                       out.write_value != 1))
            bad++;
    }
    if (edges) bad++;
    printf("  %-6s no subject edge before Override and Toggle are both held\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

/* The restore path is exercised against real memory: the anchors are pointed at
   locals, so apply_write and bridge_restore_once run exactly as they would in
   the game, without a game. */
static int t_restore_once(void)
{
    static const int original_override[] = { 0, 0, 0 };
    static const int original_toggle[] = { 3, 4, 5 };
    static const int original_move[] = { 1, 2, 3 };
    static const int borrow_move[] = { 1, 0, 1 };
    int override_value = original_override[0];
    int toggle_value = original_toggle[0];
    int move_value = original_move[0];
    int i;
    int bad = 0;
    DG_ANCHORS saved_anchors = g_b.a;
    LONG saved_wrote = g_b.wrote_any;
    LONG saved_wrote_move = g_b.wrote_move;
    LONG saved_restored = g_b.restored;
    LONG saved_c_writes = g_b.c_writes;
    int saved_override = g_b.saved_override;
    int saved_toggle = g_b.saved_toggle;
    int saved_move = g_b.saved_move;
    int saved_active = g_b.saved_active;
    int saved_owner = g_b.owner;

    memset(&g_b.a, 0, sizeof(g_b.a));
    g_b.a.gbp_override = (ULONGLONG)(ULONG_PTR)&override_value;
    g_b.a.gbp_toggle = (ULONGLONG)(ULONG_PTR)&toggle_value;
    g_b.a.gbp_move = (ULONGLONG)(ULONG_PTR)&move_value;
    g_b.a.gbp_active = (ULONGLONG)(ULONG_PTR)&override_value;

    for (i = 0; i < 3; i++) {
        override_value = original_override[i];
        toggle_value = original_toggle[i];
        move_value = original_move[i];
        bridge_capture_ownership_snapshot();
        if (g_b.owner || g_b.wrote_any || g_b.wrote_move || g_b.restored)
            bad++;
        bridge_restore_once();              /* no claim: no write */
        if (override_value != original_override[i] ||
            toggle_value != original_toggle[i] ||
            move_value != original_move[i] || g_b.restored)
            bad++;

        apply_write(DG_FPS_WRITE_OVERRIDE, 1);
        apply_write(DG_FPS_WRITE_TOGGLE, 1);
        if (override_value != 1 || toggle_value != 1) bad++;
        if (borrow_move[i]) {
            apply_write(DG_FPS_WRITE_MOVE, 0);
            if (move_value != 0) bad++;
        } else {
            if (move_value != original_move[i]) bad++;
            /* A value this generation did not borrow may change elsewhere;
               restoring the capture would overwrite that later owner. */
            move_value = 77;
        }

        bridge_restore_once();
        if (override_value != original_override[i] ||
            toggle_value != original_toggle[i] ||
            move_value != (borrow_move[i] ? original_move[i] : 77))
            bad++;
        override_value = 99;
        toggle_value = 98;
        move_value = 97;
        bridge_restore_once();              /* restore is once per generation */
        if (override_value != 99 || toggle_value != 98 || move_value != 97)
            bad++;
    }

    /* A non-zero Override observed at capture time is a genuine external
       owner, and remains untouched even when the old generation had debt. */
    override_value = 9;
    toggle_value = 6;
    move_value = 4;
    bridge_capture_ownership_snapshot();
    /* External changes after capture must survive: no claim was made. */
    override_value = 8;
    toggle_value = 7;
    move_value = 6;
    bridge_restore_once();
    if (!g_b.owner || override_value != 8 || toggle_value != 7 || move_value != 6)
        bad++;

    g_b.a = saved_anchors;
    InterlockedExchange(&g_b.wrote_any, saved_wrote);
    InterlockedExchange(&g_b.wrote_move, saved_wrote_move);
    InterlockedExchange(&g_b.restored, saved_restored);
    InterlockedExchange(&g_b.c_writes, saved_c_writes);
    g_b.saved_override = saved_override;
    g_b.saved_toggle = saved_toggle;
    g_b.saved_move = saved_move;
    g_b.saved_active = saved_active;
    g_b.owner = saved_owner;
    printf("  %-6s three bridge generations restore their own borrowed values once, leave unborrowed Move alone, and preserve an external owner\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

static int t_decoder(void)
{
    static const struct {
        const char *name;
        unsigned char bytes[16];
        unsigned length;
        unsigned expect;         /* 1 = decodable */
        unsigned rip;
    } cases[] = {
        { "push rbx",            { 0x53 }, 1, 1, 0 },
        { "push rdi (REX)",      { 0x41, 0x57 }, 2, 1, 0 },
        { "sub rsp,0x28",        { 0x48, 0x83, 0xEC, 0x28 }, 4, 1, 0 },
        { "sub rsp,0x188",       { 0x48, 0x81, 0xEC, 0x88, 0x01, 0x00, 0x00 },
                                 7, 1, 0 },
        { "mov [rsp+8],rbx",     { 0x48, 0x89, 0x5C, 0x24, 0x08 }, 5, 1, 0 },
        { "mov rax,rcx",         { 0x48, 0x8B, 0xC1 }, 3, 1, 0 },
        { "lea rcx,[rip+d]",     { 0x48, 0x8D, 0x0D, 0x11, 0x22, 0x33, 0x44 },
                                 7, 1, 1 },
        { "mov eax,[rip+d]",     { 0x8B, 0x05, 0x11, 0x22, 0x33, 0x44 },
                                 6, 1, 1 },
        { "mov [rip+d],imm32",   { 0xC7, 0x05, 1, 2, 3, 4, 5, 6, 7, 8 },
                                 10, 1, 1 },
        { "movaps [rsp+20],xmm0",{ 0x0F, 0x29, 0x44, 0x24, 0x20 }, 5, 1, 0 },
        { "movss xmm0,[rip+d]",  { 0xF3, 0x0F, 0x10, 0x05, 1, 2, 3, 4 },
                                 8, 1, 1 },
        { "movzx eax,byte [rcx]",{ 0x0F, 0xB6, 0x01 }, 3, 1, 0 },
        { "test eax,eax",        { 0x85, 0xC0 }, 2, 1, 0 },
        { "cmp dword [rbx+8],0", { 0x83, 0x7B, 0x08, 0x00 }, 4, 1, 0 },
        { "mov rax,imm64",       { 0x48, 0xB8, 1, 2, 3, 4, 5, 6, 7, 8 },
                                 10, 1, 0 },
        { "mov eax,imm32",       { 0xB8, 1, 2, 3, 4 }, 5, 1, 0 },
        { "call rel32",          { 0xE8, 1, 2, 3, 4 }, 5, 1, 0 },
        { "jmp rel8",            { 0xEB, 0x10 }, 2, 1, 0 },
        { "jne rel32",           { 0x0F, 0x85, 1, 2, 3, 4 }, 6, 1, 0 },
        { "ret",                 { 0xC3 }, 1, 1, 0 },
        { "imul eax,[rcx],7",    { 0x6B, 0x01, 0x07 }, 3, 1, 0 },
        { "test dword [rcx],1",  { 0xF7, 0x01, 1, 0, 0, 0 }, 6, 1, 0 },
        { "not dword [rcx]",     { 0xF7, 0x11 }, 2, 1, 0 },
        { "sib+disp32",          { 0x48, 0x8B, 0x84, 0x8B, 1, 2, 3, 4 },
                                 8, 1, 0 },
        { "addr32 prefix",       { 0x67, 0x8B, 0x01 }, 0, 0, 0 },
        { "three-byte 0F38",     { 0x66, 0x0F, 0x38, 0x00, 0xC1 }, 0, 0, 0 },
        { "unknown D6",          { 0xD6 }, 0, 0, 0 }
    };
    size_t i;
    int bad = 0;
    for (i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        DG_INSN insn;
        int got = dg_x64_decode(cases[i].bytes, 16, &insn);
        int ok = (got != 0) == (cases[i].expect != 0) &&
                 (!got || (insn.length == cases[i].length &&
                           insn.rip_relative == cases[i].rip));
        if (!ok) {
            bad++;
            printf("  FAIL   decode %-22s got %d len %u rip %u\n",
                   cases[i].name, got, got ? insn.length : 0,
                   got ? insn.rip_relative : 0);
        }
    }
    printf("  %-6s x64 length decoder, %d encodings, unknown forms refused\n",
           bad ? "FAIL" : "ok", (int)(sizeof(cases) / sizeof(cases[0])));
    return bad ? 1 : 0;
}

/* A relocated RIP-relative instruction must still address the same byte. */
static int t_relocation(void)
{
    unsigned char src[16];
    unsigned char dst[16];
    DG_INSN insn;
    LONG disp = 0x1000;
    LONG moved;
    LONGLONG new_disp;
    const unsigned char *src_end;
    const unsigned char *dst_end;
    int bad = 0;

    memset(src, 0x90, sizeof(src));
    memset(dst, 0x90, sizeof(dst));
    src[0] = 0x48; src[1] = 0x8D; src[2] = 0x0D;            /* lea rcx,[rip+d] */
    memcpy(src + 3, &disp, sizeof(disp));
    if (!dg_x64_decode(src, sizeof(src), &insn) || !insn.rip_relative ||
        insn.length != 7 || insn.disp_offset != 3)
        bad++;
    memcpy(dst, src, insn.length);
    src_end = src + insn.length;
    dst_end = dst + insn.length;
    new_disp = (LONGLONG)(src_end + disp) - (LONGLONG)dst_end;
    if (new_disp > 0x7FFFFFFFLL || new_disp < -0x80000000LL) bad++;
    moved = (LONG)new_disp;
    memcpy(dst + insn.disp_offset, &moved, sizeof(moved));
    if ((dst + insn.length + moved) != (src + insn.length + disp)) bad++;
    printf("  %-6s trampoline relocation keeps the RIP-relative target fixed\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

static volatile LONG g_detour_hits;
static void detour_probe(void) { InterlockedIncrement(&g_detour_hits); }

/* End to end, on a function this test assembles itself. Never on game code:
   the point is to prove the stub, the trampoline and the E9 all agree, and a
   hand-written subject is the only one whose bytes are known exactly. */
static int t_detour_end_to_end(void)
{
    static const unsigned char body[] = {
        0x48, 0x89, 0x5C, 0x24, 0x08,   /* mov [rsp+8], rbx  <- patch site */
        0x8B, 0xC1,                     /* mov eax, ecx                   */
        0x83, 0xC0, 0x07,               /* add eax, 7                     */
        0x48, 0x8B, 0x5C, 0x24, 0x08,   /* mov rbx, [rsp+8]               */
        0xC3                            /* ret                            */
    };
    unsigned char *code;
    int (*fn)(int);
    DG_DETOUR d;
    const char *why = "";
    int bad = 0;
    int before;
    int after;

    code = (unsigned char *)VirtualAlloc(NULL, 0x1000,
                                         MEM_RESERVE | MEM_COMMIT,
                                         PAGE_EXECUTE_READWRITE);
    if (!code) {
        printf("  FAIL   detour end-to-end: no code page\n");
        return 1;
    }
    memset(code, 0xCC, 0x1000);
    memcpy(code, body, sizeof(body));
    FlushInstructionCache(GetCurrentProcess(), code, 0x1000);
    fn = (int (*)(int))code;

    before = fn(35);
    InterlockedExchange(&g_detour_hits, 0);
    if (!dg_detour_install(&d, code, (void *)detour_probe,
                           code, code + sizeof(body), &why)) {
        printf("  FAIL   detour end-to-end: install refused (%s)\n", why);
        VirtualFree(code, 0, MEM_RELEASE);
        return 1;
    }
    after = fn(35);
    dg_detour_remove(&d);

    if (before != 42 || after != 42) bad++;
    if (InterlockedCompareExchange(&g_detour_hits, 0, 0) != 1) bad++;
    if (d.stolen != 5) bad++;
    if (fn(35) != 42) bad++;
    if (memcmp(code, body, sizeof(body)) != 0) bad++;
    VirtualFree(code, 0, MEM_RELEASE);

    printf("  %-6s detour: 5 bytes stolen, callback ran once, result and "
           "bytes restored\n", bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

static int t_detour_refusals(void)
{
    unsigned char buf[64];
    DG_DETOUR d;
    const char *why = "";
    int bad = 0;

    /* A relative call inside the stolen range. */
    memset(buf, 0x90, sizeof(buf));
    buf[0] = 0xE8; buf[1] = 0; buf[2] = 0; buf[3] = 0; buf[4] = 0;
    if (dg_detour_install(&d, buf, (void *)detour_probe, buf, buf + 16, &why)) {
        bad++;
        dg_detour_remove(&d);
    }

    /* An undecodable byte inside the stolen range. */
    memset(buf, 0x90, sizeof(buf));
    buf[2] = 0xD6;
    if (dg_detour_install(&d, buf, (void *)detour_probe, buf, buf + 16, &why)) {
        bad++;
        dg_detour_remove(&d);
    }

    /* A branch elsewhere in the function that lands inside the patch site. */
    memset(buf, 0x90, sizeof(buf));
    buf[16] = 0xEB; buf[17] = (unsigned char)(0xFF - 16);  /* jmp back into it */
    if (dg_detour_install(&d, buf, (void *)detour_probe, buf, buf + 32, &why)) {
        bad++;
        dg_detour_remove(&d);
    }

    printf("  %-6s detour refuses relative branches, unknown bytes and "
           "branch targets inside the patch site\n", bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

/* The shared resolver must report nothing rather than something partial. */
static int t_anchor_gate(void)
{
    unsigned char bytes[0x2000];
    unsigned char valid[0x2000];
    IMAGE_NT_HEADERS64 nt;
    IMAGE_SECTION_HEADER sections[3];
    RUNTIME_FUNCTION runtime[2];
    LiveImage image;
    DG_ANCHORS anchors;
    int bad = 0;

    memset(runtime, 0, sizeof(runtime));
    init_test_image(&image, bytes, valid, sizeof(bytes), &nt, sections,
                    runtime, 2);
    runtime[0].BeginAddress = 0x100;
    runtime[0].EndAddress = 0x200;
    sync_test_runtime(&image, runtime, 2);

    /* One of the thirteen patterns present is still not thirteen. */
    emit_rip_mov(&image, 0x110, 0x8B, image.base + 0x1000);
    if (dg_anchors_resolve(&image, &anchors)) bad++;
    if (anchors.ok) bad++;
    if (anchors.merge_seam || anchors.gbp_override) bad++;

    printf("  %-6s shared anchor resolve is all-or-nothing\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

/* The relationship layer, exercised directly.
 *
 * Building a synthetic image that satisfies all thirteen byte patterns would test
 * the scanner, not the reasoning on top of it. What is worth pinning here is
 * the reasoning: a set of targets that is individually plausible but jointly
 * wrong has to be refused. Each case below is a single field changed from an
 * otherwise passing set, so a failure names exactly one broken premise. */
static void fill_plausible_results(PatternResult r[PATTERN_COUNT])
{
    size_t i;
    ULONGLONG base = 0x140000000ULL;
    for (i = 0; i < PATTERN_COUNT; i++) {
        memset(&r[i], 0, sizeof(r[i]));
        r[i].ok = 1;
        r[i].count = 1;
    }
    r[0].targets[0] = base + 0x10;   /* gBP_Toggle          */
    r[0].targets[1] = base + 0x40;   /* PL_SubjectToggle    */
    r[1].targets[0] = base + 0x20;   /* gBP_Override        */
    r[1].targets[1] = base + 0x30;   /* gBP_Active          */
    r[2].targets[0] = base + 0x30;   /* Active, second site */
    r[3].targets[0] = base + 0x18;   /* gBP_Move            */
    r[3].targets[1] = base + 0x48;   /* PL_SubjectMove      */
    r[7].targets[0] = base + 0x80;   /* GM_PlayerStatus     */
    r[8].targets[1] = base + 0x100;  /* GM_GameStatus       */
    r[8].targets[0] = base + 0x104;  /* GM_GameStatusScn    */
    r[9].targets[1] = base + 0x108;  /* GM_MenuStatus       */
    r[9].targets[0] = base + 0x10C;  /* GM_MenuStatusScn    */
    r[10].targets[0] = base + 0x200; /* GM_PlayerArmBody, read side  */
    r[11].targets[0] = base + 0x200; /* GM_PlayerArmBody, write side */
    r[12].targets[1] = base + 0x300; /* ArmCamRotateShift            */
    r[12].targets[0] = base + 0x302; /* ...and .vy, two bytes into it */
}

static int t_arm_body_needs_two_agreeing_anchors(void)
{
    PatternResult r[PATTERN_COUNT];
    int bad = 0;

    fill_plausible_results(r);
    if (!check_relationships(r, 0)) bad++;          /* the baseline must pass */

    /* The whole reason there are two anchors: a read that drifted onto the
       neighbouring pointer is individually well-formed and jointly wrong. */
    fill_plausible_results(r);
    r[11].targets[0] += 8;
    if (check_relationships(r, 0)) bad++;

    /* Agreeing on the wrong thing is not agreement either - landing on a word
       we already hold means the displacement resolved into a neighbour. */
    fill_plausible_results(r);
    r[10].targets[0] = r[11].targets[0] = r[9].targets[1];
    if (check_relationships(r, 0)) bad++;

    fill_plausible_results(r);
    r[10].targets[0] = r[11].targets[0] = r[7].targets[0];
    if (check_relationships(r, 0)) bad++;

    /* And a missing half is not a pass with a default. */
    fill_plausible_results(r);
    r[11].ok = 0;
    if (check_relationships(r, 0)) bad++;

    /* ArmCamRotateShift is one SVECTOR, not two globals that happen to sit
       near each other. Anything but exactly two bytes apart means one of the
       displacements resolved somewhere this anchor cannot vouch for. */
    fill_plausible_results(r);
    r[12].targets[0] = r[12].targets[1] + 4;
    if (check_relationships(r, 0)) bad++;

    fill_plausible_results(r);
    r[12].targets[0] = r[12].targets[1];
    if (check_relationships(r, 0)) bad++;

    /* Reading .vy BELOW the address the lea takes is the same error mirrored,
       and it has to be refused in that direction too. */
    fill_plausible_results(r);
    r[12].targets[0] = r[12].targets[1] - 2;
    if (check_relationships(r, 0)) bad++;

    /* And it must be a struct of its own rather than a word already held. */
    fill_plausible_results(r);
    r[12].targets[1] = r[10].targets[0];
    r[12].targets[0] = r[12].targets[1] + 2;
    if (check_relationships(r, 0)) bad++;

    fill_plausible_results(r);
    r[12].ok = 0;
    if (check_relationships(r, 0)) bad++;

    printf("  %-6s arm body is refused unless a read in routine.c and the "
           "write in pl_arm.c name the same new global, and ArmCamRotateShift "
           "unless its address and its .vy are one SVECTOR nothing else "
           "owns\n", bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

/* F5's first write into a live model, exercised on a model made of locals.
   The addresses it uses are derived - Work+0x60 for the body, +0x08 for m_ctrl,
   +0x48 for the adjust array - and a derivation that is one field off writes
   into whatever is next door. So the gate matters as much as the write: the
   n_joints check must actually refuse, joint 6 must actually be refused, and
   release must leave the joint exactly as an untouched joint looks. */
/* The stride is searched for rather than computed, so the search is the thing
   that has to be tested: it must find a planted stride, refuse memory that is
   not a skeleton, and return the LOWEST stride rather than a multiple of it -
   a multiple validates perfectly well, it just lands on every second joint and
   would hand the IK a topology that is silently half a rig. */

static int t_cutscene_reaches_the_safety_gate(void)
{
    /* Measured 2026-08-17, one session, same anchors. */
    const unsigned int TICK_GAME = 0x00000000u, TICK_MENU = 0x00005800u;
    const unsigned int CAM_GAME  = 0x18000240u, CAM_MENU  = 0x00345C0Fu;
    unsigned int game, menu;
    ULONGLONG player;
    int bad = 0;

    /* The tick seam alone calls that session safe. That is the bug. */
    if ((TICK_GAME & DG_GAME_UNSAFE_MASK) != 0) bad++;
    if ((TICK_MENU & DG_MENU_UNSAFE_MASK) != 0) bad++;
    /* The camera seam alone would not have. */
    if ((CAM_GAME & DG_GAME_UNSAFE_MASK) == 0) bad++;
    if ((CAM_MENU & DG_MENU_UNSAFE_MASK) == 0) bad++;

    /* Folded, with a fresh latch: the gate now trips on both words. */
    game = TICK_GAME; menu = TICK_MENU; player = 0;
    if (!fold_late_status(&game, &menu, &player, CAM_GAME, CAM_MENU, 0, 3960, 1))
        bad++;
    if ((game & DG_GAME_UNSAFE_MASK) == 0) bad++;
    if ((menu & DG_MENU_UNSAFE_MASK) == 0) bad++;

    /* Folding is an OR, so it can only ever make the bridge stand down - it
       must never clear a bit the tick seam already set. */
    game = 0xFFFFFFFFu; menu = 0xFFFFFFFFu; player = ~0ULL;
    fold_late_status(&game, &menu, &player, 0, 0, 0, 1, 0);
    if (game != 0xFFFFFFFFu || menu != 0xFFFFFFFFu || player != ~0ULL) bad++;

    /* The player word folds too - Snake showed up only at the camera seam. */
    game = 0; menu = 0; player = 0;
    fold_late_status(&game, &menu, &player, 0, 0, 0x0310192000002010ULL, 10, 0);
    if (player != 0x0310192000002010ULL) bad++;

    /* A latch nobody has written is refused, or an unarmed session would
       inherit whatever zero happens to mean. */
    game = TICK_GAME; menu = TICK_MENU; player = 0;
    if (fold_late_status(&game, &menu, &player, CAM_GAME, CAM_MENU, 0, 0, 0))
        bad++;
    if (game != TICK_GAME) bad++;

    /* And a stale one is refused, so a camera hook that stops firing cannot
       pin the bridge into suspend for the rest of the session. */
    game = TICK_GAME; menu = TICK_MENU; player = 0;
    if (fold_late_status(&game, &menu, &player, CAM_GAME, CAM_MENU, 0,
                         3960, DG_LATE_STATUS_TICKS + 1)) bad++;
    if (game != TICK_GAME) bad++;
    /* The boundary itself is inside. */
    game = TICK_GAME; menu = TICK_MENU; player = 0;
    if (!fold_late_status(&game, &menu, &player, CAM_GAME, CAM_MENU, 0,
                          3960, DG_LATE_STATUS_TICKS)) bad++;
    /* A negative age means the counters disagree, which cannot happen and is
       therefore precisely what not to trust. */
    game = TICK_GAME; menu = TICK_MENU; player = 0;
    if (fold_late_status(&game, &menu, &player, CAM_GAME, CAM_MENU, 0, 10, -1))
        bad++;

    printf("  %-6s cutscene reaches the gate: the measured tick-seam words read"
           " safe and the camera-seam words do not, folding trips it, and a"
           " missing or stale latch is refused\n", bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

static int t_skeleton_is_measured_not_assumed(void)
{

    enum { STRIDE = 0x1A0, JOINTS = 12 };
    static unsigned char blob[DG_OBJS_ARRAY + JOINTS * STRIDE];
    static unsigned char obj[0x40], mc[0x70];
    ULONGLONG arm = (ULONGLONG)(ULONG_PTR)obj;
    /* A chain that is deliberately NOT the HUMAN21 reading: 6's parent is 5,
       then 4, then 3, then straight to the root at 0. If the walk ever stops
       following parent indices and starts assuming 5/4/3, this still passes -
       so the last hop skips 1 and 2 to make an assumption visible. */
    static const short parent[JOINTS] = { 0, 0, 1, 0, 3, 4, 5, 6, 7, 0, 9, 10 };
    int bad = 0, j;

    memset(blob, 0, sizeof(blob));
    memset(obj, 0, sizeof(obj));
    memset(mc, 0, sizeof(mc));
    *(ULONGLONG *)(obj + 0x00) = (ULONGLONG)(ULONG_PTR)blob;
    *(ULONGLONG *)(obj + 0x08) = (ULONGLONG)(ULONG_PTR)mc;
    *(ULONGLONG *)(mc + 0x60) = 0x1234ULL;      /* MOTION_CONTROL.trans */
    *(short *)(blob + DG_OBJS_NMODELS) = (short)JOINTS;

    InterlockedExchange(&g_b.skel_probe, 1);
    InterlockedExchange(&g_b.skel_base, 3);

    /* Zeroed memory is not a skeleton: m[3][3] is 0, not 1. Nothing found. */
    InterlockedExchange(&g_b.skel_for_lo, 0);
    InterlockedExchange(&g_b.skel_for_hi, 0);
    InterlockedExchange(&g_b.skel_stride, 0);
    skel_probe_now(arm, 0);
    if (g_b.skel_stride != 0) bad++;
    if (g_b.skel_n_models != JOINTS) bad++;     /* but the count was read */

    /* Now plant real joints at STRIDE. */
    for (j = 0; j < JOINTS; j++) {
        unsigned char *o = blob + DG_OBJS_ARRAY + j * STRIDE;
        float *m = (float *)o;
        m[0] = m[5] = m[10] = m[15] = 1.0f;     /* identity basis */
        m[12] = (float)(j * 10);                /* translation */
        m[13] = (float)(j * 20);
        m[14] = (float)(j * 30);
        *(short *)(o + DG_OBJ_PARENT) = parent[j];
    }
    InterlockedExchange(&g_b.skel_for_lo, 0);
    InterlockedExchange(&g_b.skel_for_hi, 0);
    InterlockedExchange(&g_b.skel_stride, 0);
    skel_probe_now(arm, 0);

    if (g_b.skel_stride != STRIDE) bad++;       /* lowest, not a multiple */
    if (g_b.skel_stride_score != 6) bad++;
    if (g_b.skel_parents_read != JOINTS) bad++;
    if (g_b.skel_mctrl_trans_lo != 0x1234) bad++;

    /* The topology, read off the model rather than off HUMAN21. */
    if (g_b.skel_chain_len != 5) bad++;
    else {
        static const LONG want[5] = { 6, 5, 4, 3, 0 };
        for (j = 0; j < 5; j++)
            if (g_b.skel_chain[j] != want[j]) bad++;
    }

    /* Positions follow the chain, so bone lengths are differences of these. */
    {
        float x;
        memcpy(&x, (const void *)&g_b.skel_chain_pos[0], sizeof x);
        if (x != 60.0f) bad++;                  /* joint 6 */
        memcpy(&x, (const void *)&g_b.skel_chain_pos[1 * 3 + 1], sizeof x);
        if (x != 100.0f) bad++;                 /* joint 5, y */
        memcpy(&x, (const void *)&g_b.skel_window_pos[0 * 3 + 2], sizeof x);
        if (x != 90.0f) bad++;                  /* window base 3, z */
    }

    /* A second call must not re-measure - it is cached per object - but the
       live pose must still refresh, because that is what the probe is for. */
    {
        float *m = (float *)(blob + DG_OBJS_ARRAY + 6 * STRIDE);
        float x;
        m[12] = 777.0f;
        InterlockedExchange(&g_b.skel_stride_tried, -1);
        skel_probe_now(arm, 0);
        if (g_b.skel_stride_tried != -1) bad++;         /* no re-scan */
        memcpy(&x, (const void *)&g_b.skel_chain_pos[0], sizeof x);
        if (x != 777.0f) bad++;                         /* but pose moved */
    }

    /* Off means off. */
    InterlockedExchange(&g_b.skel_probe, 0);
    InterlockedExchange(&g_b.skel_stride, 0);
    InterlockedExchange(&g_b.skel_for_lo, 0);
    InterlockedExchange(&g_b.skel_for_hi, 0);
    skel_probe_now(arm, 0);
    if (g_b.skel_stride != 0) bad++;

    /* Tracking needs the same bounded skeleton measurement even when the
       optional logging probe is off. `required_for_ik` bypasses only the
       switch, never the region, matrix, count or topology checks. */
    skel_probe_now(arm, 1);
    if (g_b.skel_stride != STRIDE) bad++;

    InterlockedExchange(&g_b.skel_for_lo, 0);
    InterlockedExchange(&g_b.skel_for_hi, 0);
    InterlockedExchange(&g_b.skel_stride, 0);
    InterlockedExchange(&g_b.skel_base, 0);

    printf("  %-6s skeleton: stride is found not assumed (lowest, not a "
           "multiple), zeroed memory yields none, the chain is walked from "
           "joint 6, pose refreshes without re-scanning, and IK can require "
           "the same bounded read while the logging probe is off\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

/* The camera seam may run several times in one tick. The diagnostic rate must
   therefore divide by ticks that actually reached the active seam, not by all
   session ticks (including third person), or it understates the cadence that
   determines the adjust hold interval. */
static int t_active_seam_rate_uses_active_ticks(void)
{
    enum { TICKS = 12, SEAMS_PER_TICK = 5 };
    int t, s;
    int bad = 0;

    InterlockedExchange(&g_b.c_seam_active, 0);
    InterlockedExchange(&g_b.c_seam_active_ticks, 0);
    InterlockedExchange(&g_b.seam_active_last_tick, -1);
    for (t = 0; t < TICKS; t++) {
        InterlockedExchange(&g_b.c_ticks, 100 + t);
        for (s = 0; s < SEAMS_PER_TICK; s++) count_active_seam();
    }
    if (g_b.c_seam_active != TICKS * SEAMS_PER_TICK) bad++;
    if (g_b.c_seam_active_ticks != TICKS) bad++;
    printf("  %-6s active seam rate: %ld seams / %ld active ticks = %.2f\n",
           bad ? "FAIL" : "ok", g_b.c_seam_active,
           g_b.c_seam_active_ticks,
           g_b.c_seam_active_ticks
               ? (double)g_b.c_seam_active / (double)g_b.c_seam_active_ticks
               : 0.0);
    return bad ? 1 : 0;
}

/* The run of 2026-08-18 came back with all adjust cases holding byte-identical
   matrices. The cause was cadence, not conventions: the camera seam fires 5.32
   times per tick in first person while the hierarchy pass runs once, so a probe
   that turned its case over per seam call read a pose its own write had never
   reached. This drives the probe through exactly that cadence against a fake
   pass that consumes adjust[] once per tick, and requires the two cases to come
   back DIFFERENT - which is precisely what the live run could not produce. */
static int t_adjust_probe_holds_a_case_for_a_whole_pass(void)
{
    enum { STRIDE = 0x1A0, JOINTS = 12, SEAMS_PER_TICK = 5, TICKS = 80 };
    static unsigned char blob[DG_OBJS_ARRAY + JOINTS * STRIDE];
    static unsigned char obj[0x40], mc[0x70];
    static float adjust[24 * 4];
    static const short parent[JOINTS] = { 0, 0, 1, 0, 3, 4, 5, 6, 7, 0, 9, 10 };
    ULONGLONG arm = (ULONGLONG)(ULONG_PTR)obj;
    int bad = 0, j, t, s;
    float c0j, c1j, c0c, c1c;

    memset(blob, 0, sizeof blob);
    memset(obj, 0, sizeof obj);
    memset(mc, 0, sizeof mc);
    memset(adjust, 0, sizeof adjust);
    *(ULONGLONG *)(obj + 0x00) = (ULONGLONG)(ULONG_PTR)blob;
    *(ULONGLONG *)(obj + 0x08) = (ULONGLONG)(ULONG_PTR)mc;
    *(short *)(blob + DG_OBJS_NMODELS) = (short)JOINTS;
    *(LONG *)(mc + 0x14) = 24;                          /* n_joints */
    *(ULONGLONG *)(mc + 0x48) = (ULONGLONG)(ULONG_PTR)adjust;
    for (j = 0; j < JOINTS; j++) {
        unsigned char *o = blob + DG_OBJS_ARRAY + j * STRIDE;
        float *m = (float *)o;
        m[0] = m[5] = m[10] = m[15] = 1.0f;
        m[12] = (float)(j * 10);
        m[13] = (float)(j * 20);
        m[14] = (float)(j * 30);
        *(short *)(o + DG_OBJ_PARENT) = parent[j];
    }

    /* The probe refuses to guess topology, so the skeleton has to be measured
       first - the same dependency it has live. */
    InterlockedExchange(&g_b.skel_probe, 1);
    InterlockedExchange(&g_b.skel_base, 3);
    InterlockedExchange(&g_b.skel_for_lo, 0);
    InterlockedExchange(&g_b.skel_for_hi, 0);
    InterlockedExchange(&g_b.skel_stride, 0);
    skel_probe_now(arm, 0);
    if (g_b.skel_stride != STRIDE) {
        printf("  FAIL   adjust probe: the skeleton did not measure, so "
               "nothing below could mean anything\n");
        return 1;
    }

    InterlockedExchange(&g_b.adj_probe, 1);
    InterlockedExchange(&g_b.adj_joint, 5);
    InterlockedExchange(&g_b.adj_deg, 30);
    InterlockedExchange(&g_b.adj_axis, 0);
    InterlockedExchange(&g_b.adj_pending_case, -1);
    InterlockedExchange(&g_b.adj_write_tick, 0);
    InterlockedExchange(&g_b.c_adj_held, 0);
    InterlockedExchange(&g_b.adj_samples[0], 0);
    InterlockedExchange(&g_b.adj_samples[1], 0);
    InterlockedExchange(&g_b.c_ticks, 0);

    for (t = 0; t < TICKS; t++) {
        /* The fake hierarchy pass: once per tick, BEFORE the seams, which is
           where the real one sits relative to the camera hook. It moves the
           joint and its child by whatever adjust[] holds, so a read can only
           ever see a write that a pass actually consumed. */
        /* Each axis gets its own decade, so a case that reads back under the
           wrong index is a visibly different number rather than a near miss. */
        float *mj = (float *)(blob + DG_OBJS_ARRAY + 5 * STRIDE);
        float *mk = (float *)(blob + DG_OBJS_ARRAY + 6 * STRIDE);
        float d = adjust[5 * 4 + 0] * 100.0f + adjust[5 * 4 + 1] * 10000.0f +
                  adjust[5 * 4 + 2] * 1000000.0f;
        mj[12] = 1000.0f + d;
        mk[12] = 2000.0f + d;
        /* The rotation half of the fake pass, for the per-cycle reduction:
           the engine model measured on 2026-08-30 maps adjust X/Y/Z onto
           world -Z/+Y/+X (a -90 degree yaw frame). Write the joint's 3x3 as
           the logged rows would read it - world images of the local axes,
           i.e. the transpose of the world rotation - so the reduction's
           oracle is an independent -90.00 and three 30.00 degree angles. */
        {
            static const double AX[3][3] = { { 0.0, 0.0, -1.0 },
                                             { 0.0, 1.0,  0.0 },
                                             { 1.0, 0.0,  0.0 } };
            int which = -1, a, b;
            double R[3][3], ang;
            if (adjust[5 * 4 + 0] != 0.0f) which = 0;
            else if (adjust[5 * 4 + 1] != 0.0f) which = 1;
            else if (adjust[5 * 4 + 2] != 0.0f) which = 2;
            ang = (which >= 0) ? 30.0 * 3.14159265358979323846 / 180.0 : 0.0;
            for (a = 0; a < 3; a++)
                for (b = 0; b < 3; b++) {
                    double u = (which >= 0) ? AX[which][a] : 0.0;
                    double v = (which >= 0) ? AX[which][b] : 0.0;
                    double e = 0.0;
                    double kk[3][3] = { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 },
                                        { 0.0, 0.0, 0.0 } };
                    if (which >= 0) {
                        kk[0][1] = -AX[which][2]; kk[0][2] =  AX[which][1];
                        kk[1][0] =  AX[which][2]; kk[1][2] = -AX[which][0];
                        kk[2][0] = -AX[which][1]; kk[2][1] =  AX[which][0];
                        e = kk[a][b];
                    }
                    R[a][b] = (a == b ? 1.0 : 0.0) * cos(ang) + sin(ang) * e +
                              (1.0 - cos(ang)) * u * v;
                }
            for (a = 0; a < 3; a++)
                for (b = 0; b < 3; b++) mj[a * 4 + b] = (float)R[b][a];
        }
        InterlockedExchange(&g_b.c_ticks, t);
        for (s = 0; s < SEAMS_PER_TICK; s++)
            adj_probe_now(arm);
    }

    {
        long total = 0;
        for (j = 0; j < DG_ADJ_CASES; j++) {
            if (g_b.adj_samples[j] <= 0) bad++;
            total += g_b.adj_samples[j];
        }
        /* One turn-over per settle window at most, whatever the seam rate. */
        if (total > TICKS / DG_ADJ_SETTLE_TICKS + 1) bad++;
    }
    if (g_b.c_adj_held <= 0) bad++;             /* the holds have to be real */
    /* Meetplan A' bracket: every sampled case carries a write-side and a
       read-side word set, the read at least the settle window after the
       write, and with no player on this desk both sides say ok = 0 rather
       than inventing a heading. */
    {
        int bracketed = 0;
        for (j = 0; j < DG_ADJ_CASES; j++) {
            const volatile LONG *w0 = &g_b.adj_words[(j * 2 + 0) * 5];
            const volatile LONG *w1 = &g_b.adj_words[(j * 2 + 1) * 5];
            if (g_b.adj_samples[j] <= 0) continue;
            /* A case written after its last read (the cycle's tail) carries
               a write newer than its read; every other case reads no sooner
               than the settle window after its write - and there have to BE
               such cases, or the read side was never written at all. */
            if (w1[0] >= w0[0]) {
                if (w1[0] - w0[0] < DG_ADJ_SETTLE_TICKS) bad++;
                else bracketed++;
            }
            if (!w0[1] && (w0[2] || w0[3] || w0[4])) bad++;
            if (!w1[1] && (w1[2] || w1[3] || w1[4])) bad++;
        }
        if (bracketed < DG_ADJ_CASES - 1) bad++;
    }
    /* The per-cycle reduction against the fake pass's independent model:
       adjust-X rotated about world -Z, so its heading is -90.00; every case
       recovers its 30.00 degrees; the Y case's axis is world up; no player
       on the desk, so the words are 0 and the cycle says so. */
    {
        DG_ADJ_CYCLE cy;
        int got = 0;
        while (dg_bridge_adj_cycle_take(&cy)) {
            got++;
            if (fabs((double)cy.yaw_deg + 90.0) > 0.05) bad++;
            for (j = 0; j < 3; j++)
                if (fabs((double)cy.ang_deg[j] - 30.0) > 0.05) bad++;
            if ((double)cy.ydot < 0.9999) bad++;
            if (cy.ok != 0 || cy.rot0 != 0 || cy.rot3 != 0) bad++;
        }
        if (got < 1) bad++;
    }
    /* The root is walked to the end of the parent chain, not assumed. */
    if (g_b.adj_indices[0] != 0 || g_b.adj_indices[1] != 4 ||
        g_b.adj_indices[2] != 5 || g_b.adj_indices[3] != 6) bad++;

    /* 30 degrees about one axis writes sin(15) = 0.258819 into that component,
       and the fake pass gives each axis its own decade: identity lands the
       joint at 1000, X at 1025.88, Y at 3588.19, Z at 259819. Reading a case
       back under the wrong index is therefore off by orders of magnitude, not
       by a rounding error - which is the point of the spread. */
    {
        static const float want[DG_ADJ_CASES] =
            { 0.0f, 25.8819f, 2588.19f, 258819.0f };
        for (j = 0; j < DG_ADJ_CASES; j++) {
            float gj, gc;
            memcpy(&gj, (const void *)
                   &g_b.adj_world[(j * DG_ADJ_JOINTS + 2) * 16 + 12], sizeof gj);
            memcpy(&gc, (const void *)
                   &g_b.adj_world[(j * DG_ADJ_JOINTS + 3) * 16 + 12], sizeof gc);
            if (!(gj > 1000.0f + want[j] * 0.999f - 0.1f &&
                  gj < 1000.0f + want[j] * 1.001f + 0.1f)) bad++;
            if (!(gc > 2000.0f + want[j] * 0.999f - 0.1f &&
                  gc < 2000.0f + want[j] * 1.001f + 0.1f)) bad++;
        }
        /* The headline, stated on its own: the cases have to DIFFER. Identical
           cases were the live failure, and a spread of ranges alone would let a
           regression read as four near misses instead of as that. */
        memcpy(&c0j, (const void *)
               &g_b.adj_world[(0 * DG_ADJ_JOINTS + 2) * 16 + 12], sizeof c0j);
        memcpy(&c1j, (const void *)
               &g_b.adj_world[(1 * DG_ADJ_JOINTS + 2) * 16 + 12], sizeof c1j);
        memcpy(&c0c, (const void *)
               &g_b.adj_world[(2 * DG_ADJ_JOINTS + 2) * 16 + 12], sizeof c0c);
        memcpy(&c1c, (const void *)
               &g_b.adj_world[(3 * DG_ADJ_JOINTS + 2) * 16 + 12], sizeof c1c);
        if (c0j == c1j || c1j == c0c || c0c == c1c) bad++;
    }

    InterlockedExchange(&g_b.adj_probe, 0);
    InterlockedExchange(&g_b.adj_pending_case, -1);
    InterlockedExchange(&g_b.skel_probe, 0);
    InterlockedExchange(&g_b.skel_stride, 0);
    InterlockedExchange(&g_b.skel_for_lo, 0);
    InterlockedExchange(&g_b.skel_for_hi, 0);
    InterlockedExchange(&g_b.skel_base, 0);
    InterlockedExchange(&g_b.c_ticks, 0);

    printf("  %-6s adjust probe: one case per hierarchy pass, so five seams a "
           "tick cannot read a pose the write never reached; identity and all "
           "three axes come back distinct, correctly paired, and the root is "
           "walked to the end of the parent chain\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

static int t_bend_writes_one_joint_behind_its_gate(void)
{
    /* An OBJECT (objs, m_ctrl, ...) and a MOTION_CONTROL, laid out by offset
       rather than by struct, because the offsets are the thing under test. */
    static unsigned char obj[0x40], mc[0x60];
    static float adj[21 * 4];
    ULONGLONG arm = (ULONGLONG)(ULONG_PTR)obj;
    int bad = 0, i;
    double want_s = sin(45.0 * 3.14159265358979323846 / 180.0 * 0.5);
    double want_c = cos(45.0 * 3.14159265358979323846 / 180.0 * 0.5);

    memset(obj, 0, sizeof(obj));
    memset(mc, 0, sizeof(mc));
    for (i = 0; i < 21 * 4; i++) adj[i] = -1.0f;
    *(ULONGLONG *)(obj + 0x08) = (ULONGLONG)(ULONG_PTR)mc;
    *(ULONGLONG *)(mc + 0x48) = (ULONGLONG)(ULONG_PTR)adj;

    g_b.a.gm_player_arm_body = 0;                 /* release must not need it */
    InterlockedExchange(&g_b.bend_deg, 45);
    InterlockedExchange(&g_b.bend_joint, 5);
    InterlockedExchange(&g_b.bent_joint, -1);

    /* Too few joints to be a skeleton we recognise: nothing may be written. */
    *(int *)(mc + 0x14) = 20;
    arm_bend_now(arm, NULL);
    if (adj[5 * 4] != -1.0f) bad++;
    if (*(ULONGLONG *)(mc + 0x38) != 0) bad++;

    /* Nor a count that is obvious nonsense. */
    *(int *)(mc + 0x14) = 100000;
    arm_bend_now(arm, NULL);
    if (adj[5 * 4] != -1.0f) bad++;

    /* 55 is what the subjective arm actually reports, and joint 5 is inside
       it, so the write is allowed. The old gate demanded exactly 21 and
       refused this - which is the case that cost a live run. */
    *(int *)(mc + 0x14) = 55;
    arm_bend_now(arm, NULL);
    if (fabs(adj[5 * 4 + 0] - want_s) > 1e-6) bad++;
    if (adj[5 * 4 + 1] != 0.0f || adj[5 * 4 + 2] != 0.0f) bad++;
    if (fabs(adj[5 * 4 + 3] - want_c) > 1e-6) bad++;
    if (*(ULONGLONG *)(mc + 0x38) != (1ULL << 5)) bad++;
    if (adj[4 * 4] != -1.0f || adj[6 * 4] != -1.0f) bad++;  /* neighbours */

    /* A joint outside the skeleton is refused rather than written past the
       end of the array, which is the failure this gate exists to prevent. */
    {
        float guard = adj[20 * 4];
        InterlockedExchange(&g_b.bend_joint, 19);
        *(int *)(mc + 0x14) = 19;
        arm_bend_now(arm, NULL);
        if (adj[19 * 4] != -1.0f) bad++;
        if (adj[20 * 4] != guard) bad++;
        *(int *)(mc + 0x14) = 55;
        InterlockedExchange(&g_b.bend_joint, 5);
    }

    /* Release leaves an identity quaternion and a clear bit, which is exactly
       what a joint nobody has touched looks like. */
    g_b.a.gm_player_arm_body = (ULONGLONG)(ULONG_PTR)&arm;
    arm_bend_release();
    if (adj[5 * 4 + 0] != 0.0f || adj[5 * 4 + 3] != 1.0f) bad++;
    if (*(ULONGLONG *)(mc + 0x38) != 0) bad++;
    /* ...and only once: a second release must not write a joint back. */
    adj[5 * 4 + 3] = 7.0f;
    arm_bend_release();
    if (adj[5 * 4 + 3] != 7.0f) bad++;

    {
        double q[4] = { 0.0, 0.5, 0.0, 0.8660254037844387 };
        *(int *)(mc + 0x14) = 55;
        arm_bend_now(arm, q);
        if (adj[5 * 4 + 1] != 0.5f) bad++;
        if (*(ULONGLONG *)(mc + 0x38) != (1ULL << 5)) bad++;
    }

    /* A quaternion that is not unit length is refused outright. Tracking hands
       over garbage on the frame it drops a controller, and a zero here does
       not tilt the model, it collapses it. */
    {
        double bad_q[4] = { 0.0, 0.0, 0.0, 0.0 };
        adj[5 * 4 + 1] = -1.0f;
        arm_bend_now(arm, bad_q);
        if (adj[5 * 4 + 1] != -1.0f) bad++;
    }

    /* Neither pose nor angle: release, never hold the last one. */
    InterlockedExchange(&g_b.bend_deg, 0);
    InterlockedExchange(&g_b.bent_joint, 5);
    g_b.a.gm_player_arm_body = (ULONGLONG)(ULONG_PTR)&arm;
    arm_bend_now(arm, NULL);
    if (adj[5 * 4 + 3] != 1.0f) bad++;
    if (*(ULONGLONG *)(mc + 0x38) != 0) bad++;
    g_b.a.gm_player_arm_body = 0;
    InterlockedExchange(&g_b.bend_deg, 45);

    /* Joint 6 belongs to SetPos, which writes it every frame. */
    for (i = 0; i < 21 * 4; i++) adj[i] = -1.0f;
    *(ULONGLONG *)(mc + 0x38) = 0;
    InterlockedExchange(&g_b.bend_joint, 6);
    arm_bend_now(arm, NULL);
    if (adj[6 * 4] != -1.0f) bad++;
    if (*(ULONGLONG *)(mc + 0x38) != 0) bad++;

    g_b.a.gm_player_arm_body = 0;
    InterlockedExchange(&g_b.bend_deg, 0);
    InterlockedExchange(&g_b.bend_joint, 5);
    InterlockedExchange(&g_b.bent_joint, -1);

    printf("  %-6s joint rotation: skeleton-gated, tracked pose beats the "
           "fixed angle, non-unit refused, no pose releases, joint 6 refused\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

static double test_quat_angle(const double q[4])
{
    double w = fabs(q[3]);
    if (w > 1.0) w = 1.0;
    return 2.0 * acos(w);
}

static int t_ik_glue_uses_measured_adjust_space(void)
{
    static const double joints[5][3] = {
        {   0.0,   0.0,   0.0 },       /* joint 2 */
        {   0.0,   0.0, 100.0 },       /* joint 3: outward pole basis */
        {   0.0,   0.0,   0.0 },       /* joint 4: shoulder */
        { 200.0,   0.0,   0.0 },       /* joint 5: elbow */
        { 400.0,   0.0,   0.0 }        /* joint 6: wrist */
    };
    static const double target[3] = { 250.0, -150.0, 100.0 };
    static const double far_target[3] = { 1400.0, 0.0, 0.0 };
    static const double root_pos[3] = { 0.0, 0.0, 0.0 };
    static const double view[3] = { -100.0, -200.0, 300.0 };
    static const double root[4][4] = {
        { 0.0, 1.0, 0.0, 0.0 },
        { 0.0, 0.0, 1.0, 0.0 },
        { 1.0, 0.0, 0.0, 0.0 },
        { 10.0, 20.0, 30.0, 1.0 }
    };
    double q4[4], q5[4], h4[4], h5[4];
    double world_x[4] = { 0.5, 0.0, 0.0, 0.8660254037844387 };
    double adjust[4], recovered[4], bad_joints[5][3], mapped[3], unmapped[3];
    double distance, limit;
    static unsigned char obj[0x40], mc[0x60];
    static float joint_adjust[21 * 4];
    ULONGLONG arm = (ULONGLONG)(ULONG_PTR)obj;
    unsigned clamped = 0;
    double a, h;
    int i, bad = 0;

    /* The live four-axis fit says a world +X rotation is adjust +Z, within the
       measured sub-degree residual. Preserve both its axis and its angle. */
    world_quat_to_adjust(&ADJ_FRAME_LEGACY, world_x, adjust);
    if (!(adjust[2] > 0.499 && fabs(adjust[0]) < 0.003 &&
          fabs(adjust[1]) < 0.003 && fabs(adjust[3] - world_x[3]) < 1e-6))
        bad++;
    if (!adjust_quat_to_world(&ADJ_FRAME_LEGACY, adjust, recovered)) bad++;
    if (fabs(recovered[0] - world_x[0]) > 1e-8 ||
        fabs(recovered[1] - world_x[1]) > 1e-8 ||
        fabs(recovered[2] - world_x[2]) > 1e-8 ||
        fabs(recovered[3] - world_x[3]) > 1e-8) bad++;

    /* The bridge, not the hook, applies the arm root. This cyclic basis and
       translation are the independent other half of the hook's view test. */
    arm_view_to_world(root, view, mapped);
    if (fabs(mapped[0] - 310.0) > 1e-9 ||
        fabs(mapped[1] + 80.0) > 1e-9 ||
        fabs(mapped[2] + 170.0) > 1e-9) bad++;
    if (!arm_world_to_view(root, mapped, unmapped)) bad++;
    for (i = 0; i < 3; i++)
        if (fabs(unmapped[i] - view[i]) > 1e-9) bad++;
    if (!arm_target_plausible(root_pos, joints, target,
                              &distance, &limit)) bad++;
    if (fabs(distance - sqrt(95000.0)) > 1e-9 ||
        fabs(limit - 500.0) > 1e-9) bad++;
    if (arm_target_plausible(root_pos, joints, far_target,
                             &distance, &limit)) bad++;

    if (!solve_arm_adjust(&ADJ_FRAME_LEGACY, joints, target, 1.0, NULL, NULL, q4, q5, &clamped)) bad++;
    if (clamped != 0) bad++;
    if (!solve_arm_adjust(&ADJ_FRAME_LEGACY, joints, target, 0.5, NULL, NULL, h4, h5, NULL)) bad++;
    for (i = 0; i < 2; i++) {
        const double *full = i ? q5 : q4;
        const double *half = i ? h5 : h4;
        double nf = sqrt(full[0]*full[0] + full[1]*full[1] +
                         full[2]*full[2] + full[3]*full[3]);
        double nh = sqrt(half[0]*half[0] + half[1]*half[1] +
                         half[2]*half[2] + half[3]*half[3]);
        if (fabs(nf - 1.0) > 1e-9 || fabs(nh - 1.0) > 1e-9) bad++;
        a = test_quat_angle(full);
        h = test_quat_angle(half);
        if (!(a > 1e-4) || fabs(h - 0.5 * a) > 1e-8) bad++;
    }

    memcpy(bad_joints, joints, sizeof bad_joints);
    memcpy(bad_joints[3], bad_joints[2], sizeof bad_joints[3]);
    if (solve_arm_adjust(&ADJ_FRAME_LEGACY, bad_joints, target, 1.0, NULL, NULL, q4, q5, NULL)) bad++;
    if (solve_arm_adjust(&ADJ_FRAME_LEGACY, joints, target, 0.0, NULL, NULL, q4, q5, NULL)) bad++;

    /* Leaving the subjective view is an ownership transition of its own. It
       must release both joints even though the IK writer is not entered. */
    memset(obj, 0, sizeof obj);
    memset(mc, 0, sizeof mc);
    memset(joint_adjust, 0, sizeof joint_adjust);
    *(ULONGLONG *)(obj + 0x08) = (ULONGLONG)(ULONG_PTR)mc;
    *(LONG *)(mc + 0x14) = 55;
    *(ULONGLONG *)(mc + 0x48) = (ULONGLONG)(ULONG_PTR)joint_adjust;
    joint_adjust[4 * 4] = 0.5f; joint_adjust[4 * 4 + 3] = 0.5f;
    joint_adjust[5 * 4] = 0.5f; joint_adjust[5 * 4 + 3] = 0.5f;
    /* Joint 6 belongs to the animation on this run. It is loaded with a
       recognisable rotation and a flag bit we never claimed, and must come
       back untouched: releasing more than was taken is as wrong as releasing
       less. */
    joint_adjust[6 * 4] = 0.6f; joint_adjust[6 * 4 + 3] = 0.8f;
    *(ULONGLONG *)(mc + 0x38) = (1ULL << 4) | (1ULL << 5) | (1ULL << 6);
    g_b.a.gm_player_arm_body = (ULONGLONG)(ULONG_PTR)&arm;
    g_b.ik_owned_arm = arm;
    g_b.ik_owned_mctrl = (ULONGLONG)(ULONG_PTR)mc;
    g_b.ik_owned_adjust = (ULONGLONG)(ULONG_PTR)joint_adjust;
    g_b.ik_owned_mask = (1ULL << 4) | (1ULL << 5);
    InterlockedExchange(&g_b.ik_active, 1);
    InterlockedExchange(&g_b.armed, 1);
    g_b.fps.state = DG_FPS_OFF;
    dg_bridge_arm_seam_now(NULL);
    if (joint_adjust[4 * 4] != 0.0f || joint_adjust[4 * 4 + 3] != 1.0f ||
        joint_adjust[5 * 4] != 0.0f || joint_adjust[5 * 4 + 3] != 1.0f ||
        joint_adjust[6 * 4] != 0.6f || joint_adjust[6 * 4 + 3] != 0.8f ||
        *(ULONGLONG *)(mc + 0x38) != (1ULL << 6) || g_b.ik_active != 0) bad++;
    if (g_b.ik_owned_mask != 0) bad++;

    /* Even a stale/legacy ownership bit for joint 6 is not authority to touch
       SetPos's slot or flag. Only joints 4 and 5 come back. */
    joint_adjust[4 * 4] = 0.5f; joint_adjust[4 * 4 + 3] = 0.5f;
    joint_adjust[5 * 4] = 0.5f; joint_adjust[5 * 4 + 3] = 0.5f;
    joint_adjust[6 * 4] = 0.6f; joint_adjust[6 * 4 + 3] = 0.8f;
    *(ULONGLONG *)(mc + 0x38) = (1ULL << 4) | (1ULL << 5) | (1ULL << 6);
    g_b.ik_owned_arm = arm;
    g_b.ik_owned_mctrl = (ULONGLONG)(ULONG_PTR)mc;
    g_b.ik_owned_adjust = (ULONGLONG)(ULONG_PTR)joint_adjust;
    g_b.ik_owned_mask = (1ULL << 4) | (1ULL << 5) | (1ULL << 6);
    InterlockedExchange(&g_b.ik_active, 1);
    arm_ik_release();
    if (joint_adjust[4 * 4] != 0.0f || joint_adjust[4 * 4 + 3] != 1.0f ||
        joint_adjust[5 * 4] != 0.0f || joint_adjust[5 * 4 + 3] != 1.0f ||
        joint_adjust[6 * 4] != 0.6f || joint_adjust[6 * 4 + 3] != 0.8f ||
        *(ULONGLONG *)(mc + 0x38) != (1ULL << 6) ||
        g_b.ik_owned_mask != 0) bad++;

    /* A replacement model must never receive a release meant for the old
       model. Identity mismatch clears local ownership and writes no byte. */
    joint_adjust[4 * 4] = 0.25f;
    joint_adjust[5 * 4] = 0.75f;
    *(ULONGLONG *)(mc + 0x38) = (1ULL << 4) | (1ULL << 5);
    g_b.ik_owned_arm = arm + 8;
    g_b.ik_owned_mctrl = (ULONGLONG)(ULONG_PTR)mc;
    g_b.ik_owned_adjust = (ULONGLONG)(ULONG_PTR)joint_adjust;
    g_b.ik_owned_mask = (1ULL << 4) | (1ULL << 5);
    InterlockedExchange(&g_b.ik_active, 1);
    {
        LONG mismatches = g_b.c_arm_release_owner_mismatch;
        arm_ik_release();
        if (joint_adjust[4 * 4] != 0.25f ||
            joint_adjust[5 * 4] != 0.75f ||
            *(ULONGLONG *)(mc + 0x38) != ((1ULL << 4) | (1ULL << 5)) ||
            g_b.c_arm_release_owner_mismatch != mismatches + 1) bad++;
    }
    InterlockedExchange(&g_b.armed, 0);
    g_b.a.gm_player_arm_body = 0;

    printf("  %-6s arm IK glue: live-root view transform, dynamic reach "
           "envelope, dynamic lengths, measured world-to-adjust conjugation, "
           "identity blend, degenerate-input refusal, and inactive-view "
           "release\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

/* Axis/angle to {x,y,z,w}, written out here rather than borrowed so the test
   states its own inputs. */
static void th_axis(double ax, double ay, double az, double deg, double q[4])
{
    double n = sqrt(ax * ax + ay * ay + az * az);
    double h = deg * 3.14159265358979323846 / 360.0;
    double s = sin(h);
    q[0] = ax / n * s; q[1] = ay / n * s; q[2] = az / n * s; q[3] = cos(h);
}

/* The signed twist of a (near-)pure-twist world quaternion about a unit
   axis, degrees. Written out here so the tests measure with their own
   ruler instead of the helper's. */
static double tg_twist_deg(const double q[4], const double axis[3])
{
    double d = q[0] * axis[0] + q[1] * axis[1] + q[2] * axis[2];
    return 2.0 * atan2(d, q[3]) * 180.0 / 3.14159265358979323846;
}

/* The carried-up residual the grip roll exists to close, replicated from
   the DESIGN (carry perp-of-up across the chain, compare against perp on
   the displayed bone) rather than read out of the helper. */
static double tg_residual_deg(const double q4a[4], const double q5a[4],
                              const double anim_fore[3], const double up[3])
{
    double w4[4], w5[4], chain[4], fn[3], an[3], axisv[3];
    double perp[3], carried[3], wanted[3], cx[3];
    double n, d;
    int k;
    if (!adjust_quat_to_world(&ADJ_FRAME_LEGACY, q4a, w4) ||
        !adjust_quat_to_world(&ADJ_FRAME_LEGACY, q5a, w5)) return 1e9;
    dg_ik_quat_mul(w5, w4, chain);
    n = sqrt(anim_fore[0] * anim_fore[0] + anim_fore[1] * anim_fore[1] +
             anim_fore[2] * anim_fore[2]);
    for (k = 0; k < 3; k++) fn[k] = anim_fore[k] / n;
    arm_quat_rotate(chain, fn, axisv);
    n = sqrt(axisv[0] * axisv[0] + axisv[1] * axisv[1] + axisv[2] * axisv[2]);
    for (k = 0; k < 3; k++) an[k] = axisv[k] / n;
    d = up[0] * fn[0] + up[1] * fn[1] + up[2] * fn[2];
    for (k = 0; k < 3; k++) perp[k] = up[k] - d * fn[k];
    arm_quat_rotate(chain, perp, carried);
    d = up[0] * an[0] + up[1] * an[1] + up[2] * an[2];
    for (k = 0; k < 3; k++) wanted[k] = up[k] - d * an[k];
    n = sqrt(carried[0] * carried[0] + carried[1] * carried[1] +
             carried[2] * carried[2]);
    if (!(n > 1e-9)) return 1e9;
    for (k = 0; k < 3; k++) carried[k] /= n;
    n = sqrt(wanted[0] * wanted[0] + wanted[1] * wanted[1] +
             wanted[2] * wanted[2]);
    if (!(n > 1e-9)) return 1e9;
    for (k = 0; k < 3; k++) wanted[k] /= n;
    cx[0] = carried[1] * wanted[2] - carried[2] * wanted[1];
    cx[1] = carried[2] * wanted[0] - carried[0] * wanted[2];
    cx[2] = carried[0] * wanted[1] - carried[1] * wanted[0];
    return atan2(cx[0] * an[0] + cx[1] * an[1] + cx[2] * an[2],
                 carried[0] * wanted[0] + carried[1] * wanted[1] +
                 carried[2] * wanted[2]) * 180.0 / 3.14159265358979323846;
}

/* t_grip_roll - the vr_arm_uproll owner (ROLL_ONTWERP_V3 par. 5). Every leg
   asserts its own preconditions in the fixture, because a leg whose
   precondition silently fails is a leg that passes while testing nothing. */
static int t_grip_roll(void)
{
    static const double joints[5][3] = {
        {   0.0,   0.0,   0.0 },
        {   0.0,   0.0, 100.0 },
        {   0.0,   0.0,   0.0 },
        { 200.0,   0.0,   0.0 },
        { 400.0,   0.0,   0.0 }
    };
    static const double target[3] = { 250.0, -150.0, 100.0 };
    static const double t2[3] = { 150.0, -250.0, 50.0 };
    DG_GRIP_ROLL_IN gr;
    double p4[4], p5[4], c4[4], c5[4], r4[4], r5[4];
    double fore[3], up[3];
    double res_plain, res_corr;
    unsigned clamped;
    LONG up0, sk0, fb0;
    int k, bad = 0, pb = 0;
#define TG_LEG(name) do { if (bad != pb) { \
        printf("    [grip-roll leg %s] +%d\n", name, bad - pb); pb = bad; } \
    } while (0)

    memset(&gr, 0, sizeof gr);
    gr.on = 1;
    gr.have_readback = 1;
    gr.slot_q4[3] = 1.0;
    gr.slot_q5[3] = 1.0;
    for (k = 0; k < 3; k++) gr.raw_fore[k] = joints[4][k] - joints[3][k];
    fore[0] = 200.0; fore[1] = 0.0; fore[2] = 0.0;
    up[0] = 0.0; up[1] = 1.0; up[2] = 0.0;
    g_b.arm_orient_have_prev = 0;

    /* Off is bit-for-bit off: NULL and on=0 produce the same quaternions
       and move no counter. */
    up0 = g_b.c_orient_uprolled; sk0 = g_b.c_uproll_skipped;
    if (!solve_arm_adjust(&ADJ_FRAME_LEGACY, joints, target, 1.0, NULL, NULL, p4, p5,
                          &clamped)) bad++;
    {
        DG_GRIP_ROLL_IN off = gr;
        off.on = 0;
        g_b.arm_orient_have_prev = 0;
        if (!solve_arm_adjust(&ADJ_FRAME_LEGACY, joints, target, 1.0, NULL, &off, c4, c5,
                              &clamped)) bad++;
        if (memcmp(p4, c4, sizeof p4) || memcmp(p5, c5, sizeof p5)) bad++;
    }
    if (g_b.c_orient_uprolled != up0 || g_b.c_uproll_skipped != sk0) bad++;
    TG_LEG("off");

    /* Alignment: the plain rig must carry a real residual (the
       precondition), the corrected rig must close it, the counter must
       move by exactly one, and q4 - the elbow's joint - must not move. */
    res_plain = tg_residual_deg(p4, p5, fore, up);
    if (!(fabs(res_plain) > 5.0 && fabs(res_plain) < 179.0)) bad++;
    up0 = g_b.c_orient_uprolled;
    g_b.arm_orient_have_prev = 0;
    if (!solve_arm_adjust(&ADJ_FRAME_LEGACY, joints, target, 1.0, NULL, &gr, c4, c5,
                          &clamped)) bad++;
    res_corr = tg_residual_deg(c4, c5, fore, up);
    if (!(fabs(res_corr) < 0.01)) bad++;
    if (memcmp(p4, c4, sizeof p4)) bad++;
    if (g_b.c_orient_uprolled != up0 + 1) bad++;
    TG_LEG("align");

    /* Fail-closed: an untrusted read-back leaves the solve EXACTLY the
       plain one and counts the skip instead of guessing. */
    {
        DG_GRIP_ROLL_IN nr = gr;
        nr.have_readback = 0;
        sk0 = g_b.c_uproll_skipped; up0 = g_b.c_orient_uprolled;
        g_b.arm_orient_have_prev = 0;
        if (!solve_arm_adjust(&ADJ_FRAME_LEGACY, joints, target, 1.0, NULL, &nr, r4, r5,
                              &clamped)) bad++;
        if (memcmp(r5, p5, sizeof r5)) bad++;
        if (g_b.c_uproll_skipped != sk0 + 1 ||
            g_b.c_orient_uprolled != up0) bad++;
    }
    TG_LEG("fail-closed");

    /* Divergence: the slots say the animation is NOT what the solver was
       fed - the exact case the cache-trusting version got wrong. The
       displayed forearm direction (chain applied to the slot-derived
       animation bone) must be identical with and without the correction,
       and the correction itself must be a real rotation. */
    {
        DG_GRIP_ROLL_IN dv = gr;
        double s4w[4], s5w[4], t1[3], anim[3];
        double w4p[4], w5p[4], w4c[4], w5c[4], chp[4], chc[4];
        double dp[3], dc[3], n1, n2, dot;
        th_axis(0.0, 0.0, 1.0, 20.0, s4w);
        th_axis(0.0, 1.0, 0.0, 30.0, s5w);
        world_quat_to_adjust(&ADJ_FRAME_LEGACY, s4w, dv.slot_q4);
        world_quat_to_adjust(&ADJ_FRAME_LEGACY, s5w, dv.slot_q5);
        anim[0] = 200.0; anim[1] = 0.0; anim[2] = 0.0;
        arm_quat_rotate(s4w, anim, t1);
        arm_quat_rotate(s5w, t1, dv.raw_fore);
        g_b.arm_orient_have_prev = 0;
        if (!solve_arm_adjust(&ADJ_FRAME_LEGACY, joints, target, 1.0, NULL, &dv, c4, c5,
                              &clamped)) bad++;
        if (!(dg_ik_quat_angle(c5, p5) * 180.0 /
              3.14159265358979323846 > 0.1)) bad++;
        if (!adjust_quat_to_world(&ADJ_FRAME_LEGACY, p4, w4p) || !adjust_quat_to_world(&ADJ_FRAME_LEGACY, p5, w5p) ||
            !adjust_quat_to_world(&ADJ_FRAME_LEGACY, c4, w4c) || !adjust_quat_to_world(&ADJ_FRAME_LEGACY, c5, w5c))
            bad++;
        dg_ik_quat_mul(w5p, w4p, chp);
        dg_ik_quat_mul(w5c, w4c, chc);
        arm_quat_rotate(chp, anim, dp);
        arm_quat_rotate(chc, anim, dc);
        n1 = sqrt(dp[0] * dp[0] + dp[1] * dp[1] + dp[2] * dp[2]);
        n2 = sqrt(dc[0] * dc[0] + dc[1] * dc[1] + dc[2] * dc[2]);
        dot = (dp[0] * dc[0] + dp[1] * dc[1] + dp[2] * dc[2]) / (n1 * n2);
        if (dot > 1.0) dot = 1.0;
        if (!(acos(dot) * 180.0 / 3.14159265358979323846 < 1e-3)) bad++;
    }
    TG_LEG("divergence");

    /* Purity is an ANCHORED claim (ROLL_ONTWERP_V3 par. 2.4): the straight
       rest of the glue rig has no arm plane and falls back to continuity,
       which is history-bearing by design. This bent-rest rig anchors - the
       counter is the witness - and there the same pair twice is bitwise
       the same answer, and a three-solve loop returning to its first
       target returns to its first answer. */
    {
        static const double joints_bent[5][3] = {
            {   0.0,   0.0,   0.0 },
            {   0.0,   0.0, 100.0 },
            {   0.0,   0.0,   0.0 },
            { 180.0,   0.0,  80.0 },
            { 380.0,   0.0,  80.0 }
        };
        LONG an0 = g_b.c_orient_anchored;
        DG_GRIP_ROLL_IN gb = gr;
        for (k = 0; k < 3; k++)
            gb.raw_fore[k] = joints_bent[4][k] - joints_bent[3][k];
        g_b.arm_orient_have_prev = 0;
        if (!solve_arm_adjust(&ADJ_FRAME_LEGACY, joints_bent, target, 1.0, NULL, &gb, r4, r5,
                              &clamped)) bad++;
        if (g_b.c_orient_anchored != an0 + 1) bad++;
        g_b.arm_orient_have_prev = 0;
        if (!solve_arm_adjust(&ADJ_FRAME_LEGACY, joints_bent, target, 1.0, NULL, &gb, c4, c5,
                              &clamped)) bad++;
        if (memcmp(r4, c4, sizeof r4) || memcmp(r5, c5, sizeof r5)) bad++;
        if (!solve_arm_adjust(&ADJ_FRAME_LEGACY, joints_bent, t2, 1.0, NULL, &gb, c4, c5,
                              &clamped)) bad++;
        if (!solve_arm_adjust(&ADJ_FRAME_LEGACY, joints_bent, target, 1.0, NULL, &gb, c4, c5,
                              &clamped)) bad++;
        for (k = 0; k < 4; k++)
            if (fabs(c4[k] - r4[k]) > 1e-12 ||
                fabs(c5[k] - r5[k]) > 1e-12) bad++;
    }
    TG_LEG("purity");

    /* The fade oracle, hand-computed in the design (ROLL_ONTWERP_V3 par.
       2.3) and asserted against the helper directly: exact through 150
       degrees, the three in-band fractions, zero at the branch, and a
       continuous single-signed sweep. */
    {
        static const struct { double th, alpha; } tab[] = {
            { 120.0, 120.0 }, { 150.0, 150.0 },
            { 157.5, 132.890625 }, { 165.0, 82.5 }, { 172.5, 26.953125 }
        };
        double q4a[4], q5a[4], wq[4], back[4], axx[3];
        double alpha, prev_alpha, th;
        int i;
        axx[0] = 1.0; axx[1] = 0.0; axx[2] = 0.0;
        for (i = 0; i < 5; i++) {
            DG_GRIP_ROLL_IN fg = gr;
            fg.raw_fore[0] = 1.0; fg.raw_fore[1] = 0.0; fg.raw_fore[2] = 0.0;
            memset(q4a, 0, sizeof q4a); q4a[3] = 1.0;
            th_axis(1.0, 0.0, 0.0, -tab[i].th, wq);
            world_quat_to_adjust(&ADJ_FRAME_LEGACY, wq, q5a);
            arm_grip_roll(&ADJ_FRAME_LEGACY, &fg, q4a, q5a);
            if (!adjust_quat_to_world(&ADJ_FRAME_LEGACY, q5a, back)) bad++;
            alpha = tg_twist_deg(back, axx) + tab[i].th;
            if (fabs(alpha - tab[i].alpha) >
                ((tab[i].th <= 150.0) ? 1e-6 : 1e-5)) {
                printf("    [fade dbg] th=%.3f expect=%.9f got=%.9f "
                       "raw_twist=%.9f\n", tab[i].th, tab[i].alpha, alpha,
                       tg_twist_deg(back, axx));
                bad++;
            }
        }
        {
            DG_GRIP_ROLL_IN fg = gr;
            fg.raw_fore[0] = 1.0; fg.raw_fore[1] = 0.0; fg.raw_fore[2] = 0.0;
            memset(q4a, 0, sizeof q4a); q4a[3] = 1.0;
            th_axis(1.0, 0.0, 0.0, -(180.0 - 1e-6), wq);
            world_quat_to_adjust(&ADJ_FRAME_LEGACY, wq, q5a);
            arm_grip_roll(&ADJ_FRAME_LEGACY, &fg, q4a, q5a);
            if (!adjust_quat_to_world(&ADJ_FRAME_LEGACY, q5a, back)) bad++;
            alpha = tg_twist_deg(back, axx) + (180.0 - 1e-6);
            if (!(fabs(alpha) < 1e-3)) bad++;
        }
        prev_alpha = 0.0;
        for (th = 1.0; th < 179.5; th += 0.5) {
            DG_GRIP_ROLL_IN fg = gr;
            fg.raw_fore[0] = 1.0; fg.raw_fore[1] = 0.0; fg.raw_fore[2] = 0.0;
            memset(q4a, 0, sizeof q4a); q4a[3] = 1.0;
            th_axis(1.0, 0.0, 0.0, -th, wq);
            world_quat_to_adjust(&ADJ_FRAME_LEGACY, wq, q5a);
            arm_grip_roll(&ADJ_FRAME_LEGACY, &fg, q4a, q5a);
            if (!adjust_quat_to_world(&ADJ_FRAME_LEGACY, q5a, back)) bad++;
            alpha = tg_twist_deg(back, axx) + th;
            if (!_finite(alpha) || alpha < -1e-6) bad++;
            /* Continuity, not monotonicity: the fade's steepest slope is
               |d(alpha)/d(th)| ~ 7.8 deg/deg near 165 (s' peaks at 1.5),
               so a 0.5-degree step moves alpha at most ~3.9 degrees. */
            if (th > 1.0 && fabs(alpha - prev_alpha) > 4.5) bad++;
            prev_alpha = alpha;
        }
    }
    TG_LEG("fade-oracle");

    /* Partiality inside the vertical band: a forearm 5 degrees from
       vertical must be corrected PARTIALLY - strictly between 20 and 80
       percent of the demanded 5-degree roll. A mutation that keeps the
       gate but drops the weight corrects fully here and dies. */
    {
        DG_GRIP_ROLL_IN fg = gr;
        double fdir[3], q4a[4], q5a[4], wq[4], back[4], alpha;
        double s5 = sin(5.0 * 3.14159265358979323846 / 180.0);
        double c5v = cos(5.0 * 3.14159265358979323846 / 180.0);
        fdir[0] = s5; fdir[1] = c5v; fdir[2] = 0.0;
        fg.raw_fore[0] = fdir[0]; fg.raw_fore[1] = fdir[1];
        fg.raw_fore[2] = fdir[2];
        memset(q4a, 0, sizeof q4a); q4a[3] = 1.0;
        th_axis(fdir[0], fdir[1], fdir[2], -5.0, wq);
        world_quat_to_adjust(&ADJ_FRAME_LEGACY, wq, q5a);
        arm_grip_roll(&ADJ_FRAME_LEGACY, &fg, q4a, q5a);
        if (!adjust_quat_to_world(&ADJ_FRAME_LEGACY, q5a, back)) bad++;
        alpha = tg_twist_deg(back, fdir) + 5.0;
        if (!(alpha > 1.0 && alpha < 4.0)) bad++;
    }
    TG_LEG("partiality");

    /* The up axis: with the bone along +X and world up the helper's own
       +Y, a -30-degree demand closes exactly. The rewired-axis mutation
       (up_w sideways, parallel to this bone) fades the correction to
       nothing here and the residual stands - this leg dies. */
    {
        DG_GRIP_ROLL_IN fg = gr;
        double q4a[4], q5a[4], wq[4], back[4], axx[3], resid;
        axx[0] = 1.0; axx[1] = 0.0; axx[2] = 0.0;
        fg.raw_fore[0] = 1.0; fg.raw_fore[1] = 0.0; fg.raw_fore[2] = 0.0;
        memset(q4a, 0, sizeof q4a); q4a[3] = 1.0;
        th_axis(1.0, 0.0, 0.0, -30.0, wq);
        world_quat_to_adjust(&ADJ_FRAME_LEGACY, wq, q5a);
        arm_grip_roll(&ADJ_FRAME_LEGACY, &fg, q4a, q5a);
        if (!adjust_quat_to_world(&ADJ_FRAME_LEGACY, q5a, back)) bad++;
        resid = tg_twist_deg(back, axx);
        if (!(fabs(resid) < 1e-6)) bad++;
    }
    TG_LEG("up-axis");

    /* Blend purity: at weight 0.5 the corrected forearm quaternion is the
       half-blended one composed with a PURE twist about the displayed
       bone - and the fixture first proves its base and twist do not
       commute, so the leg cannot pass by commutativity. */
    {
        double h4[4], h5[4], hc4[4], hc5[4];
        double w4[4], w5[4], wc5[4], ch[4], delta[4], conj[4];
        double axv[3], n, para, perp2;
        double ab[4], ba[4];
        g_b.arm_orient_have_prev = 0;
        if (!solve_arm_adjust(&ADJ_FRAME_LEGACY, joints, target, 0.5, NULL, NULL, h4, h5,
                              &clamped)) bad++;
        g_b.arm_orient_have_prev = 0;
        if (!solve_arm_adjust(&ADJ_FRAME_LEGACY, joints, target, 0.5, NULL, &gr, hc4, hc5,
                              &clamped)) bad++;
        if (!adjust_quat_to_world(&ADJ_FRAME_LEGACY, h4, w4) || !adjust_quat_to_world(&ADJ_FRAME_LEGACY, h5, w5) ||
            !adjust_quat_to_world(&ADJ_FRAME_LEGACY, hc5, wc5)) bad++;
        dg_ik_quat_mul(w5, w4, ab);
        dg_ik_quat_mul(w4, w5, ba);
        if (!(dg_ik_quat_angle(ab, ba) * 180.0 /
              3.14159265358979323846 > 1.0)) bad++;
        dg_ik_quat_mul(w5, w4, ch);
        arm_quat_rotate(ch, fore, axv);
        n = sqrt(axv[0] * axv[0] + axv[1] * axv[1] + axv[2] * axv[2]);
        for (k = 0; k < 3; k++) axv[k] /= n;
        dg_ik_quat_conj(w5, conj);
        dg_ik_quat_mul(wc5, conj, delta);
        para = fabs(delta[0] * axv[0] + delta[1] * axv[1] +
                    delta[2] * axv[2]);
        perp2 = sqrt(delta[0] * delta[0] + delta[1] * delta[1] +
                     delta[2] * delta[2] - para * para);
        if (!(para > 1e-4)) bad++;
        if (!(perp2 < 1e-6)) bad++;
        if (memcmp(h4, hc4, sizeof h4)) bad++;
    }
    TG_LEG("blend");

    /* CONTINUED scope: this straight-rest rig takes the fallback path (the
       counter is the witness), two different seeded histories give two
       different bases there, and the correction stays a pure twist about
       the displayed bone on both. */
    {
        double a5[4], b5[4], x4[4];
        fb0 = g_b.c_orient_fallback;
        g_b.arm_orient_have_prev = 0;
        if (!solve_arm_adjust(&ADJ_FRAME_LEGACY, joints, target, 1.0, NULL, &gr, x4, a5,
                              &clamped)) bad++;
        if (!solve_arm_adjust(&ADJ_FRAME_LEGACY, joints, t2, 1.0, NULL, &gr, x4, b5,
                              &clamped)) bad++;
        if (g_b.c_orient_fallback < fb0 + 1) bad++;
        if (!(dg_ik_quat_angle(a5, b5) * 180.0 /
              3.14159265358979323846 > 0.01)) bad++;
    }

    TG_LEG("fallback-history");
#undef TG_LEG
    g_b.arm_orient_have_prev = 0;
    printf("  %-6s grip roll: off is off, alignment closed with the elbow "
           "untouched, fail-closed read-back, wrist fixed under slot "
           "divergence, bitwise purity, hand-computed fade oracle, in-band "
           "partiality, up-axis oracle, blend twist-purity, fallback "
           "history\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

/* Lay a world rotation into a joint matrix the way the hierarchy pass does:
   row r is the world direction of local axis r. */
static void th_write_basis(float *m, const double q[4])
{
    double axis[3], world[3];
    int r, k;
    for (r = 0; r < 3; r++) {
        axis[0] = axis[1] = axis[2] = 0.0;
        axis[r] = 1.0;
        arm_quat_rotate(q, axis, world);
        for (k = 0; k < 3; k++) m[r * 4 + k] = (float)world[k];
        m[r * 4 + 3] = 0.0f;
    }
    m[15] = 1.0f;
}

static double th_angle_between(const double a[4], const double b[4])
{
    return dg_ik_quat_angle(a, b) * 180.0 / 3.14159265358979323846;
}

/* The engine's pull on the shift SVECTOR, with the part run 13 (2026-09-03)
   proved: GV_NearExp4PV measures the way back to zero along the SHORT ARC of
   the 4096-unit turn, so a short written past a half turn is folded before
   it is pulled. A write of 2426 - the 4/3 precompensation of 1820 - lands
   at -1253, three quarters of a turn from the ask, which the hand shows as a
   90-degree flip. Inside the half turn this is the plain x - x/4 the older
   legs modelled. */
static int th_engine_pull(int x)
{
    int w = x % 4096;
    if (w < 0) w += 4096;
    if (w >= 2048) w -= 4096;
    return w - w / 4;
}

/* Compose our cached shoulder/forearm adjustments plus SetPos's independently
   supplied hand adjustment exactly as the engine does. */
static int th_next_hand(const double anim[4], const double hand_adjust[4],
                        double out[4])
{
    double a[3][4], w[3][4], chain[4];
    int j, k;
    for (j = 0; j < 2; j++) {
        for (k = 0; k < 4; k++)
            a[j][k] = (double)g_b.arm_map_cached_adjust[j * 4 + k];
        if (!adjust_quat_to_world(&ADJ_FRAME_LEGACY, a[j], w[j])) return 0;
    }
    for (k = 0; k < 4; k++) a[2][k] = hand_adjust[k];
    if (!adjust_quat_to_world(&ADJ_FRAME_LEGACY, a[2], w[2])) return 0;
    dg_ik_quat_mul(w[1], w[0], chain);          /* fore then upper */
    dg_ik_quat_mul(w[2], chain, chain);         /* hand outermost */
    dg_ik_quat_mul(chain, anim, out);
    return 1;
}

/* Rotate an entire fake rig rigidly about its root's position: every joint
   basis turned by `turn` in world, every joint position swung around the
   root. This is what a body turn looks like to the seam - the hierarchy
   moves as one piece and the player's input does not. */
static void th_turn_rig(unsigned char *joints, int stride, int count,
                        const double turn[4])
{
    double origin[3], rows[3][3], q[4], nq[4], p[3], r[3];
    const float *root = (const float *)joints;
    int j, k, rr;
    for (k = 0; k < 3; k++) origin[k] = (double)root[12 + k];
    for (j = 0; j < count; j++) {
        float *m = (float *)(joints + j * stride);
        for (rr = 0; rr < 3; rr++)
            for (k = 0; k < 3; k++) rows[rr][k] = (double)m[rr * 4 + k];
        if (!dg_ik_basis_quat((const double (*)[3])rows, q)) continue;
        dg_ik_quat_mul(turn, q, nq);
        for (k = 0; k < 3; k++) p[k] = (double)m[12 + k] - origin[k];
        th_write_basis(m, nq);
        arm_quat_rotate(turn, p, r);
        for (k = 0; k < 3; k++) m[12 + k] = (float)(origin[k] + r[k]);
    }
}

/* The shoulder-relative world offset a pair published: where the driven
   hand target sits relative to the character's shoulder, in world
   coordinates. The shoulder itself rides the body; this offset belongs to
   the player, and a body yaw must leave it alone. */
static void th_pair_offset(const double live_root_q[4],
                           const double root_pos[3], double offset[3])
{
    double shoulder_view[3], shoulder_world[3];
    float f;
    LONG v;
    int k;
    for (k = 0; k < 3; k++) {
        v = InterlockedCompareExchange(&g_b.s_arm_shoulder_view[k], 0, 0);
        memcpy(&f, &v, sizeof f);
        shoulder_view[k] = (double)f;
    }
    arm_quat_rotate(live_root_q, shoulder_view, shoulder_world);
    for (k = 0; k < 3; k++) {
        v = InterlockedCompareExchange(&g_b.s_arm_ik_target[k], 0, 0);
        memcpy(&f, &v, sizeof f);
        offset[k] = (double)f - (root_pos[k] + shoulder_world[k]);
    }
}

/* The probe is only worth running if our copy of the game's conversion is
   faithful, so this pins it against facts that do not come from the copy:
   single-axis rotations, whose quaternions are known without any Euler
   convention at all, and a composed case built by multiplying three axis
   quaternions with the IK module's own product. */
/* The hole this closes, exercised through the real seam entry point.
 *
 * The tick seam is the only writer of fps.state, and it nearly stops during a
 * codec or a cutscene while the camera seam speeds up - 465 ticks against 2474
 * camera samples in the window of dg_hook.f5_adj_nosignal_2026-08-18 that
 * contained both. So fps.state is not merely late during those states, it is
 * frozen at whatever it read before they began, which is ACTIVE.
 *
 * The test therefore never calls the tick seam at all. It plants ACTIVE, moves
 * the status words underneath, and requires the writer to stand down anyway -
 * which is exactly the situation the log measured and which no test covered
 * before this one. */
static int t_writer_stands_down_on_the_seams_own_reading(void)
{
    enum { STRIDE = 0x180, JOINTS = 7 };
    static unsigned char blob[DG_OBJS_ARRAY + JOINTS * STRIDE];
    static unsigned char obj[0x40], mc[0x70];
    static float adjust[55 * 4];
    static LONG game, game_scn, menu, menu_scn;
    static ULONGLONG player;
    ULONGLONG arm = (ULONGLONG)(ULONG_PTR)obj;
    DG_ANCHORS saved_anchors = g_b.a;
    DG_BRIDGE_ARM_TARGET target;
    LONG refused;
    int j, bad = 0;

    memset(blob, 0, sizeof blob);
    memset(obj, 0, sizeof obj);
    memset(mc, 0, sizeof mc);
    memset(adjust, 0, sizeof adjust);
    *(ULONGLONG *)(obj + 0x00) = (ULONGLONG)(ULONG_PTR)blob;
    *(ULONGLONG *)(obj + 0x08) = (ULONGLONG)(ULONG_PTR)mc;
    *(LONG *)(mc + 0x14) = 55;
    *(ULONGLONG *)(mc + 0x48) = (ULONGLONG)(ULONG_PTR)adjust;
    for (j = 0; j < JOINTS; j++) {
        float *m = (float *)(blob + DG_OBJS_ARRAY + j * STRIDE);
        m[0] = m[5] = m[10] = m[15] = 1.0f;
    }
    ((float *)(blob + DG_OBJS_ARRAY + 3 * STRIDE))[14] = 100.0f;
    ((float *)(blob + DG_OBJS_ARRAY + 5 * STRIDE))[12] = 200.0f;
    ((float *)(blob + DG_OBJS_ARRAY + 6 * STRIDE))[12] = 300.0f;
    ((float *)(blob + DG_OBJS_ARRAY + 6 * STRIDE))[13] = 173.20508f;

    InterlockedExchange(&g_b.skel_stride, STRIDE);
    InterlockedExchange(&g_b.skel_parents_read, JOINTS);
    InterlockedExchange(&g_b.skel_region_end, (LONG)sizeof blob);
    for (j = 0; j < JOINTS; j++) InterlockedExchange(&g_b.skel_parents[j], 0);
    InterlockedExchange(&g_b.skel_parents[3], 2);
    InterlockedExchange(&g_b.skel_parents[4], 3);
    InterlockedExchange(&g_b.skel_parents[5], 4);
    InterlockedExchange(&g_b.skel_parents[6], 5);

    game = game_scn = menu = menu_scn = 0;
    player = 0;
    g_b.a.gm_game_status = (ULONGLONG)(ULONG_PTR)&game;
    g_b.a.gm_game_status_scn = (ULONGLONG)(ULONG_PTR)&game_scn;
    g_b.a.gm_menu_status = (ULONGLONG)(ULONG_PTR)&menu;
    g_b.a.gm_menu_status_scn = (ULONGLONG)(ULONG_PTR)&menu_scn;
    g_b.a.gm_player_status = (ULONGLONG)(ULONG_PTR)&player;
    g_b.a.gm_player_arm_body = (ULONGLONG)(ULONG_PTR)&arm;
    g_b.a.arm_cam_rotate_shift = 0;
    InterlockedExchange(&g_b.armed, 1);
    InterlockedExchange(&g_b.ik_active, 0);
    InterlockedExchange(&g_b.c_seam_refused_unsafe, 0);
    InterlockedExchange(&g_b.c_ticks, DG_ADJ_SETTLE_TICKS + 1);
    /* ACTIVE, and never revisited - the tick seam is not called even once. */
    g_b.fps.state = DG_FPS_ACTIVE;
    arm_map_forget();

    memset(&target, 0, sizeof target);
    target.write = 1;
    target.weight = 1.0;
    target.stream_id = 11;
    target.pair_id = 900;
    target.wrist_view[0] = 770.0;
    target.hand_quat[3] = 1.0;

    /* Safe: acquisition, settle and calibration proceed as usual, so the test
       is not passing merely because nothing works. */
    for (j = 0; j < 6; j++) {
        target.pair_id++;
        InterlockedIncrement(&g_b.c_ticks);
        dg_bridge_arm_seam_now(&target);
    }
    if (*(ULONGLONG *)(mc + 0x38) == 0) bad++;      /* it did write */
    if (g_b.c_seam_refused_unsafe != 0) bad++;

    /* A story cutscene starts. STATE_SCN_DEMO, in the scenario half of the
       word - the half a reader that took only GM_GameStatus would miss. */
    refused = g_b.c_seam_refused_unsafe;
    game_scn = 0x08000000;
    target.pair_id++;
    InterlockedIncrement(&g_b.c_ticks);
    dg_bridge_arm_seam_now(&target);
    if (g_b.c_seam_refused_unsafe != refused + 1) bad++;
    if (*(ULONGLONG *)(mc + 0x38) != 0) bad++;      /* and it let go */
    if (g_b.fps.state != DG_FPS_ACTIVE) bad++;      /* while still "ACTIVE" */

    /* It stays stood down for as long as the scene runs. */
    for (j = 0; j < 20; j++) {
        target.pair_id++;
        InterlockedIncrement(&g_b.c_ticks);
        dg_bridge_arm_seam_now(&target);
    }
    if (*(ULONGLONG *)(mc + 0x38) != 0) bad++;
    if (g_b.c_seam_refused_unsafe != refused + 21) bad++;

    /* The scene ends, a codec call starts. MENU_RADIO_ON, the only bit that
       can see one, and again in the scenario half. */
    game_scn = 0;
    menu_scn = 0x00000400;
    target.pair_id++;
    InterlockedIncrement(&g_b.c_ticks);
    dg_bridge_arm_seam_now(&target);
    if (*(ULONGLONG *)(mc + 0x38) != 0) bad++;
    if (g_b.c_seam_refused_unsafe != refused + 22) bad++;

    /* An alarm is not a cutscene. STATE_DETECT must not stand the writer down,
       or a chase becomes the one time the arm stops working. */
    menu_scn = 0;
    game = 0x00000004;
    for (j = 0; j < 6; j++) {
        target.pair_id++;
        InterlockedIncrement(&g_b.c_ticks);
        dg_bridge_arm_seam_now(&target);
    }
    if (*(ULONGLONG *)(mc + 0x38) == 0) bad++;
    if (g_b.c_seam_refused_unsafe != refused + 22) bad++;

    InterlockedExchange(&g_b.armed, 0);
    arm_map_forget();
    g_b.a = saved_anchors;
    InterlockedExchange(&g_b.skel_stride, 0);
    InterlockedExchange(&g_b.skel_parents_read, 0);
    InterlockedExchange(&g_b.skel_region_end, 0);
    InterlockedExchange(&g_b.c_ticks, 0);
    InterlockedExchange(&g_b.c_seam_refused_unsafe, 0);
    g_b.fps.state = DG_FPS_OFF;

    printf("  %-6s writer safety: with fps.state frozen at ACTIVE - which is "
           "what a codec or cutscene does to the tick seam - the joint writer "
           "still stands down on the camera seam's own reading of a scenario "
           "demo and of the codec, and an alarm still counts as gameplay\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

static int t_set_pos_quat_matches_the_game(void)
{
    double rot[3], q[4], want[4], qx[4], qy[4], qz[4], chain[4];
    int bad = 0, k;

    /* Units first: 4096 PS2 units is a full turn, so 1024 is a right angle and
       half of it lands on cos/sin of 45 degrees. Everything below depends on
       that scale being right, and nothing below could tell a wrong scale from
       a wrong axis if this were not checked on its own. */
    rot[0] = 0.0; rot[1] = 1024.0; rot[2] = 0.0;
    set_pos_quat(rot, q);
    if (fabs(q[0]) > 1e-9 || fabs(q[1] - 0.70710678) > 1e-6 ||
        fabs(q[2]) > 1e-9 || fabs(q[3] - 0.70710678) > 1e-6) bad++;

    /* A pure X turn, which both branches happen to agree on - so this pins the
       axis without depending on which branch ran. */
    rot[0] = 1024.0; rot[1] = 0.0; rot[2] = 0.0;
    set_pos_quat(rot, q);
    if (fabs(q[0] - 0.70710678) > 1e-6 || fabs(q[1]) > 1e-9 ||
        fabs(q[2]) > 1e-9 || fabs(q[3] - 0.70710678) > 1e-6) bad++;

    /* And the branch that makes vy != 0 non-negotiable. With vy zero SetPos
       takes GM_RotToQuatXAfterY, which forces q.vy = 0 and never looks at vz
       at all: a quarter turn about Z asked for this way is silently discarded.
       That is the whole reason the solver must keep vy off zero, and it is
       worth a test rather than a comment. */
    rot[0] = 0.0; rot[1] = 0.0; rot[2] = 1024.0;
    set_pos_quat(rot, q);
    if (fabs(q[3] - 1.0) > 1e-9) bad++;          /* identity: vz thrown away */

    /* One unit of vy - 0.088 degrees, invisible - switches to the full XYZ
       path and the same vz now arrives. */
    rot[0] = 0.0; rot[1] = 1.0; rot[2] = 1024.0;
    set_pos_quat(rot, q);
    if (fabs(q[2] - 0.70710678) > 1e-4 || fabs(q[3] - 0.70710678) > 1e-4) bad++;
    if (fabs(q[1]) > 1e-2) bad++;                /* and vy really is tiny */

    /* All zero is identity, which is what the 2026-08-18 run observed the game
       writing into adjust[6] on all 115 arm-camera ticks. */
    rot[0] = rot[1] = rot[2] = 0.0;
    set_pos_quat(rot, q);
    if (q[0] != 0.0 || q[1] != 0.0 || q[2] != 0.0 || fabs(q[3] - 1.0) > 1e-12)
        bad++;

    /* The composition order, built independently: three axis quaternions
       multiplied with dg_ik_quat_mul, which the IK suite proves separately.
       MT_EulerToQuatXYZ is q = Rz(vz) * Ry(vy) * Rx(vx) - X applied first. A
       different order gives a visibly different rotation for these angles. */
    rot[0] = 300.0; rot[1] = -700.0; rot[2] = 450.0;
    set_pos_quat(rot, q);
    {
        double ax = rot[0] * 2.0 * 3.14159265358979323846 / 4096.0;
        double ay = rot[1] * 2.0 * 3.14159265358979323846 / 4096.0;
        double az = rot[2] * 2.0 * 3.14159265358979323846 / 4096.0;
        qx[0] = sin(ax * 0.5); qx[1] = 0.0; qx[2] = 0.0; qx[3] = cos(ax * 0.5);
        qy[0] = 0.0; qy[1] = sin(ay * 0.5); qy[2] = 0.0; qy[3] = cos(ay * 0.5);
        qz[0] = 0.0; qz[1] = 0.0; qz[2] = sin(az * 0.5); qz[3] = cos(az * 0.5);
        dg_ik_quat_mul(qz, qy, chain);
        dg_ik_quat_mul(chain, qx, want);
    }
    for (k = 0; k < 4; k++)
        if (fabs(q[k] - want[k]) > 1e-9) bad++;

    /* The reverse order must NOT match, or the check above would pass on any
       implementation at all. */
    dg_ik_quat_mul(qx, qy, chain);
    dg_ik_quat_mul(chain, qz, want);
    {
        double worst = 0.0;
        for (k = 0; k < 4; k++)
            if (fabs(q[k] - want[k]) > worst) worst = fabs(q[k] - want[k]);
        if (worst < 0.01) bad++;
    }

    /* Every result is a rotation. */
    rot[0] = -1900.0; rot[1] = 2048.0; rot[2] = 1700.0;
    set_pos_quat(rot, q);
    {
        double n = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        if (fabs(n - 1.0) > 1e-9) bad++;
    }

    printf("  %-6s SetPos conversion: 4096 units to a turn, XYZ Euler applied "
           "X first, and the vy=0 branch really does discard vz - which is why "
           "the hand solver may never write vy zero\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

/* The probe's own hands: what it writes, what it clamps, and that it gives the
   SVECTOR back. It writes into a global the game owns, so "stops writing" is
   not good enough - it has to put zero there, which is the value every
   weapon's table entry holds. */

static int t_the_two_halves_of_the_conversion_agree(void)
{
    static const double AXES[8][3] = {
        {  1.0,  0.0,  0.0 }, {  0.0,  1.0,  0.0 }, {  0.0,  0.0,  1.0 },
        {  0.6, -0.8,  0.0 }, {  0.3,  0.2, -0.9 }, { -0.5,  0.5,  0.7 },
        {  0.9,  0.1,  0.4 }, { -0.2, -0.7,  0.6 }
    };
    double worst = 0.0, worst_clamped = 0.0;
    int samples = 0, clamped = 0, singular = 0, bad = 0;
    int a, d;

    for (a = 0; a < 8; a++) {
        double n = sqrt(AXES[a][0] * AXES[a][0] + AXES[a][1] * AXES[a][1] +
                        AXES[a][2] * AXES[a][2]);
        for (d = -175; d <= 175; d += 5) {
            double half = (double)d * 3.14159265358979323846 / 360.0;
            double s = sin(half) / n;
            double want[4], got[4], rot[3];
            short ps[3];
            unsigned flags = 0;
            double miss;
            int k;

            want[0] = AXES[a][0] * s;
            want[1] = AXES[a][1] * s;
            want[2] = AXES[a][2] * s;
            want[3] = cos(half);

            if (!dg_ik_quat_to_ps_angles(want, ps, &flags)) { bad++; continue; }
            /* Never zero, or SetPos would take the branch that throws vz away
               and this whole round trip would be measuring the wrong function. */
            if (ps[1] == 0) bad++;
            if (flags & DG_IK_PS_VY_CLAMPED) clamped++;
            if (flags & DG_IK_PS_SINGULAR) singular++;

            for (k = 0; k < 3; k++) rot[k] = (double)ps[k];
            set_pos_quat(rot, got);
            miss = dg_ik_quat_angle(want, got) * 180.0 /
                   3.14159265358979323846;
            samples++;
            if (flags & DG_IK_PS_VY_CLAMPED) {
                if (miss > worst_clamped) worst_clamped = miss;
                if (miss > 0.176) bad++;
            } else {
                if (miss > worst) worst = miss;
                if (miss > 0.132) bad++;
            }
        }
    }
    if (samples < 500) bad++;
    /* Both interesting cases have to actually occur, or the bounds above were
       never exercised and the numbers printed below mean nothing. */
    if (!clamped) bad++;

    /* And the poles, which is where an Euler decomposition is worth least and
       a silent answer would be worth nothing at all. */
    {
        double pole[4], got[4], rot[3];
        short ps[3];
        unsigned flags = 0;
        double half = 3.14159265358979323846 / 4.0;   /* 90 deg about Y */
        int k;
        pole[0] = 0.0; pole[1] = sin(half); pole[2] = 0.0; pole[3] = cos(half);
        if (!dg_ik_quat_to_ps_angles(pole, ps, &flags)) bad++;
        if (!(flags & DG_IK_PS_SINGULAR)) bad++;      /* reported, not hidden */
        if (ps[1] != 1024 && ps[1] != -1024) bad++;
        for (k = 0; k < 3; k++) rot[k] = (double)ps[k];
        set_pos_quat(rot, got);
        if (dg_ik_quat_angle(pole, got) * 180.0 / 3.14159265358979323846 > 0.2)
            bad++;
    }

    printf("  %-6s conversion halves agree: %d rotations survive the inverse "
           "in dg_ik.c and the forward transcription here, worst %.4f deg "
           "(%.4f on the %d vy-clamped), %d at a gimbal pole and every one of "
           "them reported\n",
           bad ? "FAIL" : "ok", samples, worst, worst_clamped, clamped,
           singular);
    return bad ? 1 : 0;
}

static int t_hand_probe_owns_and_returns_the_svector(void)
{
    short live[3] = { 111, 222, 333 };
    DG_BRIDGE_CONFIG cfg;
    ULONGLONG saved = g_b.a.arm_cam_rotate_shift;
    LONG writes;
    int bad = 0;

    g_b.a.arm_cam_rotate_shift = (ULONGLONG)(ULONG_PTR)live;
    InterlockedExchange(&g_b.hand_probe_owned, 0);
    InterlockedExchange(&g_b.c_hand_probe_writes, 0);

    /* Off means untouched, not zeroed: a probe nobody asked for must leave the
       game exactly as it found it. */
    InterlockedExchange(&g_b.hand_probe, 0);
    hand_probe_write();
    if (live[0] != 111 || live[1] != 222 || live[2] != 333) bad++;
    if (g_b.c_hand_probe_writes != 0) bad++;

    memset(&cfg, 0, sizeof cfg);
    cfg.fps_mode = DG_FPS_MODE_TOGGLE;
    cfg.hand_probe = 1;
    cfg.hand_probe_rot[0] = 40;
    cfg.hand_probe_rot[1] = 256;
    cfg.hand_probe_rot[2] = -90;
    dg_bridge_configure(&cfg);
    writes = g_b.c_hand_probe_writes;
    hand_probe_write();
    if (live[0] != 40 || live[1] != 256 || live[2] != -90) bad++;
    if (g_b.c_hand_probe_writes != writes + 1) bad++;
    if (!g_b.hand_probe_owned) bad++;

    /* A short read as an angle wraps, so a value past half a turn is not a
       bigger rotation - it is a different one. Clamped, both ways. */
    cfg.hand_probe_rot[0] = 40000;
    cfg.hand_probe_rot[1] = -40000;
    cfg.hand_probe_rot[2] = 0;
    dg_bridge_configure(&cfg);
    hand_probe_write();
    if (live[0] != 2048 || live[1] != -2048 || live[2] != 0) bad++;

    /* Turning it off hands the SVECTOR back rather than leaving our angle in
       a global the game will pick up again next time the arm camera comes on. */
    cfg.hand_probe = 0;
    dg_bridge_configure(&cfg);
    if (live[0] != 0 || live[1] != 0 || live[2] != 0) bad++;
    if (g_b.hand_probe_owned) bad++;

    /* And a second release is not a second write into someone else's memory. */
    live[0] = 7;
    hand_probe_release();
    if (live[0] != 7) bad++;

    g_b.a.arm_cam_rotate_shift = saved;
    InterlockedExchange(&g_b.hand_probe, 0);
    InterlockedExchange(&g_b.hand_probe_owned, 0);
    InterlockedExchange(&g_b.c_hand_probe_writes, 0);

    printf("  %-6s hand probe: off writes nothing, on writes the configured "
           "angles clamped to half a turn, and release puts back the 0,0,0 "
           "every weapon's own table holds\n", bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

static int t_hand_tick_writer_is_fail_closed(void)
{
    union { ULONGLONG align; unsigned char b[0x300]; } actor;
    union { ULONGLONG align; unsigned char b[0xD20]; } player;
    ULONGLONG arm, arm_slot;
    ULONGLONG saved_arm_anchor = g_b.a.gm_player_arm_body;
    ULONGLONG saved_shift = g_b.a.arm_cam_rotate_shift;
    short shift[3] = { 7, 8, 9 };
    short command[3] = { 100, -200, 300 };
    LONG n;
    int bad = 0;

    memset(&actor, 0, sizeof actor);
    memset(&player, 0, sizeof player);
    arm = (ULONGLONG)(ULONG_PTR)(actor.b + 0x60);
    arm_slot = arm;
    *(ULONGLONG *)(actor.b + 0x228) =
        (ULONGLONG)(ULONG_PTR)(player.b + 0xCF4);
    *(ULONGLONG *)(player.b + 0xBA8) = arm;
    *(LONG *)(player.b + 0xBB0) = 6;
    *(LONG *)(player.b + 0xB90) = 1;
    g_b.a.gm_player_arm_body = (ULONGLONG)(ULONG_PTR)&arm_slot;
    g_b.a.arm_cam_rotate_shift = (ULONGLONG)(ULONG_PTR)shift;
    InterlockedExchange(&g_b.hand_command_requested, 1);
    InterlockedExchange(&g_b.c_ticks, 100);
    hand_command_clear();

    n = g_b.c_arm_hand_tick_writes;
    if (!hand_command_publish(command, 41)) bad++;
    hand_drive_write();
    if (shift[0] != 100 || shift[1] != -200 || shift[2] != 300) bad++;
    if (g_b.c_arm_hand_tick_writes != n + 1) bad++;

    shift[0] = 11;
    *(LONG *)(player.b + 0xB90) = DG_WEAPON_MIC;
    if (resolve_motion_player(NULL,NULL,NULL) != DG_RESOLVE_OK ||
        resolve_player(NULL,NULL,NULL) != DG_RESOLVE_MIC) bad++;
    n = g_b.c_arm_hand_tick_mic;
    hand_drive_write();
    if (shift[0] != 11 || g_b.c_arm_hand_tick_mic != n + 1) bad++;

    *(LONG *)(player.b + 0xB90) = 99;
    n = g_b.c_arm_hand_tick_bad_weapon;
    hand_drive_write();
    if (shift[0] != 11 || g_b.c_arm_hand_tick_bad_weapon != n + 1) bad++;

    *(LONG *)(player.b + 0xB90) = 1;
    *(ULONGLONG *)(player.b + 0xBA8) = arm + 8;
    n = g_b.c_arm_hand_tick_owner_mismatch;
    hand_drive_write();
    if (shift[0] != 11 || g_b.c_arm_hand_tick_owner_mismatch != n + 1) bad++;
    *(ULONGLONG *)(player.b + 0xBA8) = arm;

    arm_slot = 0;
    n = g_b.c_arm_hand_tick_no_player;
    hand_drive_write();
    if (shift[0] != 11 || g_b.c_arm_hand_tick_no_player != n + 1) bad++;
    arm_slot = arm;

    InterlockedExchange(&g_b.c_ticks, 103);
    n = g_b.c_arm_hand_tick_stale;
    hand_drive_write();
    if (shift[0] != 11 || g_b.c_arm_hand_tick_stale != n + 1) bad++;

    n = g_b.c_arm_hand_tick_no_command;
    hand_drive_write();
    if (shift[0] != 11 || g_b.c_arm_hand_tick_no_command != n + 1) bad++;

    hand_command_clear();
    InterlockedExchange(&g_b.hand_command_requested, 0);
    InterlockedExchange(&g_b.c_ticks, 0);
    g_b.a.gm_player_arm_body = saved_arm_anchor;
    g_b.a.arm_cam_rotate_shift = saved_shift;

    printf("  %-6s hand tick gate: coherent command writes once; stale, null "
           "player, owner mismatch, invalid weapon and microphone each refuse "
           "without touching ArmCamRotateShift\n", bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

/* The wrist channel's release. Our command sits in ArmCamRotateShift when a
   stream ends, the game only decays it 25% per frame, and the next stream's
   rest capture reads the hierarchy two ticks later - so unless the release
   WRITES the zero, every recalibration inherits half the previous stream's
   wrist error. A session of 2026-08-19 rode that ratchet through nine
   B-press recalibrations without ever unpinning the envelope. */
static int t_release_gives_the_wrist_back(void)
{
    union { ULONGLONG align; unsigned char b[0x300]; } actor;
    union { ULONGLONG align; unsigned char b[0xD20]; } player;
    ULONGLONG arm, arm_slot;
    ULONGLONG saved_arm_anchor = g_b.a.gm_player_arm_body;
    ULONGLONG saved_shift = g_b.a.arm_cam_rotate_shift;
    short shift[3] = { 7, 8, 9 };
    short command[3] = { 100, -200, 300 };
    LONG n;
    int bad = 0;

    memset(&actor, 0, sizeof actor);
    memset(&player, 0, sizeof player);
    arm = (ULONGLONG)(ULONG_PTR)(actor.b + 0x60);
    arm_slot = arm;
    *(ULONGLONG *)(actor.b + 0x228) =
        (ULONGLONG)(ULONG_PTR)(player.b + 0xCF4);
    *(ULONGLONG *)(player.b + 0xBA8) = arm;
    *(LONG *)(player.b + 0xBB0) = 6;
    *(LONG *)(player.b + 0xB90) = 1;
    g_b.a.gm_player_arm_body = (ULONGLONG)(ULONG_PTR)&arm_slot;
    g_b.a.arm_cam_rotate_shift = (ULONGLONG)(ULONG_PTR)shift;
    InterlockedExchange(&g_b.hand_command_requested, 1);
    InterlockedExchange(&g_b.c_ticks, 100);
    InterlockedExchange(&g_b.hand_drive_owned, 0);
    InterlockedExchange(&g_b.hand_zero_pending, 0);
    hand_command_clear();

    /* A release before we ever wrote owes nothing: the channel is the game's
       and a zero would stomp whatever it was doing with it. */
    InterlockedExchange(&g_b.hand_zero_pending, 0);
    if (InterlockedCompareExchange(&g_b.hand_drive_owned, 0, 0)) bad++;
    arm_map_forget();
    if (InterlockedCompareExchange(&g_b.hand_zero_pending, 0, 0)) bad++;
    hand_drive_write();
    if (shift[0] != 7 || shift[1] != 8 || shift[2] != 9) bad++;

    /* Write once: now we own the channel. */
    if (!hand_command_publish(command, 41)) bad++;
    hand_drive_write();
    if (shift[0] != 100 || shift[1] != -200 || shift[2] != 300) bad++;
    if (!InterlockedCompareExchange(&g_b.hand_drive_owned, 0, 0)) bad++;

    /* A stream restart owes a zero, and the next tick pays it - exactly
       once. */
    n = g_b.c_arm_hand_zeroed;
    arm_map_forget();
    if (!InterlockedCompareExchange(&g_b.hand_zero_pending, 0, 0)) bad++;
    hand_drive_write();
    if (shift[0] != 0 || shift[1] != 0 || shift[2] != 0) bad++;
    if (g_b.c_arm_hand_zeroed != n + 1) bad++;
    shift[0] = 5;
    hand_drive_write();
    if (shift[0] != 5) bad++;
    if (g_b.c_arm_hand_zeroed != n + 1) bad++;

    /* The debt goes through the same gates as every other wrist write. A
       refused player keeps it OWED rather than dropping it. */
    if (!hand_command_publish(command, 41)) bad++;
    hand_drive_write();                       /* own the channel again */
    if (shift[0] != 100) bad++;
    arm_map_forget();
    *(LONG *)(player.b + 0xB90) = DG_WEAPON_MIC;
    hand_drive_write();
    if (shift[0] != 100) bad++;               /* untouched through the gate */
    if (!InterlockedCompareExchange(&g_b.hand_zero_pending, 0, 0)) bad++;
    *(LONG *)(player.b + 0xB90) = 1;
    hand_drive_write();
    if (shift[0] != 0) bad++;                 /* paid once the gate opens */

    /* The OTHER release moment: a stream BEGIN owes the same zero. Forget and
       begin are separate functions and only one of them being wired is
       exactly the mutation that survived the first sweep of this test. */
    if (!hand_command_publish(command, 41)) bad++;
    hand_drive_write();
    if (shift[0] != 100) bad++;
    {
        DG_BRIDGE_ARM_TARGET t2;
        memset(&t2, 0, sizeof t2);
        t2.pair_id = 1; t2.stream_id = 1; t2.weight = 1.0;
        n = g_b.c_arm_hand_zeroed;
        arm_map_begin(0, 0, &t2);
        if (!InterlockedCompareExchange(&g_b.hand_zero_pending, 0, 0)) bad++;
        hand_drive_write();
        if (shift[0] != 0 || g_b.c_arm_hand_zeroed != n + 1) bad++;
    }

    /* A live command supersedes an owed zero: the channel is spoken for
       again, and a zero landing after it would stomp a real wrist. */
    InterlockedExchange(&g_b.hand_zero_pending, 1);
    if (!hand_command_publish(command, 41)) bad++;
    n = g_b.c_arm_hand_zeroed;
    hand_drive_write();
    if (shift[0] != 100 || shift[1] != -200 || shift[2] != 300) bad++;
    if (InterlockedCompareExchange(&g_b.hand_zero_pending, 0, 0)) bad++;
    /* Withdraw the command the ordinary way - a clear alone is a hiccup, not
       a release, so it owes nothing. */
    hand_command_clear();
    if (InterlockedCompareExchange(&g_b.hand_zero_pending, 0, 0)) bad++;
    shift[1] = 44;
    hand_drive_write();                       /* no command, no debt */
    if (shift[1] != 44) bad++;
    if (g_b.c_arm_hand_zeroed != n) bad++;

    hand_command_clear();
    InterlockedExchange(&g_b.hand_zero_pending, 0);
    InterlockedExchange(&g_b.hand_drive_owned, 0);
    InterlockedExchange(&g_b.hand_command_requested, 0);
    InterlockedExchange(&g_b.c_ticks, 0);
    g_b.a.gm_player_arm_body = saved_arm_anchor;
    g_b.a.arm_cam_rotate_shift = saved_shift;

    printf("  %-6s wrist release: a stream restart writes the zero the decay "
           "only drifts toward - once, through the gates, only if we ever "
           "wrote, and never over a live command\n", bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

/* F9: the walking write, end to end across the seams. dg_move_test.c already
   proves the decision table; what only exists here is the crossing - a camera
   seam publishing a level while the game tick consumes it - and the stores
   themselves: which bytes, which flags, and the guarantee that every refusal
   leaves the pad byte-identical. The pad is compared wholesale after every
   refused tick for the same reason the fire test does it: "it did not write"
   is worth nothing unless it is measured. */
/* Third person and prone (2026-09-11). Same fixture as the walk test: a
   fake pad and a fake PL_SubjectMove word, move_tick driven directly. */
static int t_walking_in_third_person_and_prone(void)
{
    union { ULONGLONG align; unsigned char b[0x40]; } padmem, padwas;
    ULONGLONG pad;
    ULONGLONG saved_pad = g_b.a.player_pad;
    ULONGLONG saved_subj = g_b.a.pl_subject_move;
    ULONGLONG saved_status = g_b.tick_status;
    LONG saved_state = g_b.fps.state;
    LONG saved_mode = g_b.walk_mode, saved_turn = g_b.turn_mode;
    LONG saved_third = g_b.move_third, saved_prone = g_b.move_prone;
    LONG saved_cam = g_b.s_cam_dir, saved_sign = g_b.move_dir_sign;
    LONG saved_off = g_b.move_dir_offset, saved_max = g_b.move_prone_max;
    ULONGLONG saved_workl = g_b.workl_ptr;
    LONG subject_word = 0;
    DG_BRIDGE_MOVE cmd;
    LONG n;
    int bad = 0;
    short dir;

    union { ULONGLONG align; unsigned char b[0x520]; } worklmem;
    ULONGLONG workl_var = (ULONGLONG)(ULONG_PTR)worklmem.b;
    memset(&worklmem, 0, sizeof worklmem);
    *(LONG *)(worklmem.b + 0x510) = -1;
    g_b.workl_ptr = (ULONGLONG)(ULONG_PTR)&workl_var;
#define TP_RESET() do { memset(&padmem, 0, sizeof padmem); \
        *(LONG *)(padmem.b + 0) = 1; \
        padmem.b[4 + DG_PAD_LEFT_DX_OFFSET] = 128; \
        padmem.b[4 + DG_PAD_LEFT_DY_OFFSET] = 128; \
        padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] = 128; \
        padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET + 1] = 128; \
        *(short *)(padmem.b + 4 + DG_PAD_DIR_OFFSET) = (short)0x7777; \
        padwas = padmem; } while (0)

    pad = (ULONGLONG)(ULONG_PTR)(padmem.b + 4);
    g_b.a.player_pad = pad;
    g_b.a.pl_subject_move = (ULONGLONG)(ULONG_PTR)&subject_word;
    g_b.fps.state = DG_FPS_OFF;
    InterlockedExchange(&g_b.s_late_unsafe, 0);
    InterlockedExchange(&g_b.armed, 1);
    InterlockedExchange(&g_b.walk_mode, 1);
    InterlockedExchange(&g_b.turn_mode, 1);
    InterlockedExchange(&g_b.move_deadzone_mils, 150);
    InterlockedExchange(&g_b.turn_gain_mils, 1000);
    InterlockedExchange(&g_b.move_valid, 0);
    InterlockedExchange(&g_b.c_ticks, 500);
    InterlockedExchange(&g_b.move_third, 0);
    InterlockedExchange(&g_b.move_prone, 0);
    InterlockedExchange(&g_b.move_prone_max, 120);
    InterlockedExchange(&g_b.move_dir_offset, 0);
    InterlockedExchange(&g_b.move_dir_sign, 1);
    InterlockedExchange(&g_b.s_cam_dir, 1024);
    g_b.tick_status = 0;
    memset(&cmd, 0, sizeof cmd);
    cmd.valid = 1; cmd.y = 1.0; cmd.turn_x = 1.0; cmd.turn_valid = 1;

    /* 1. Feature off: third person refuses at the gate, byte-identical. */
    TP_RESET();
    dg_bridge_move_now(&cmd);
    n = g_b.c_move_blocked_gate;
    move_tick(1);
    if (g_b.c_move_blocked_gate != n + 1) bad++;
    if (memcmp(&padmem, &padwas, sizeof padmem)) bad++;

    /* 2. On: a forward stick writes bytes, the UDLR bit, L_USE and
       pad->dir = camera yaw (stick up: VecDir2 2048, + yaw + 2048). The
       right stick stays silent in third person. */
    InterlockedExchange(&g_b.move_third, 1);
    TP_RESET();
    dg_bridge_move_now(&cmd);
    n = g_b.c_move_writes;
    move_tick(1);
    if (g_b.c_move_writes != n + 1) bad++;
    if (padmem.b[4 + DG_PAD_LEFT_DY_OFFSET] != 1) bad++;
    if (!(*(LONG *)(padmem.b + 4 + DG_PAD_STATUS_OFFSET) & DG_MOVE_PAD_U)) bad++;
    if (!(*(short *)(padmem.b + 4 + DG_PAD_ANALOG_OFFSET) & DG_MOVE_ANALOG_L_USE)) bad++;
    if (*(short *)(padmem.b + 4 + DG_PAD_ANALOG_OFFSET) & DG_MOVE_ANALOG_R_USE) bad++;
    if (padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] != 128) bad++;
    dir = *(short *)(padmem.b + 4 + DG_PAD_DIR_OFFSET);
    if (dir != 1024) bad++;
    if (g_b.c_move_third_writes < 1) bad++;
    /* And CheckDirection's two words, redone from our bytes: PadTo = dir,
       PadForce = full stick = (127-48)*256/80 = 252. */
    if (*(LONG *)(worklmem.b + 0x510) != 1024) bad++;
    if (*(LONG *)(worklmem.b + 0x514) != 252) bad++;

    /* 3. The game's own first-person look (PLAYER_WATCH) refuses. */
    g_b.tick_status = 0x1ULL;
    TP_RESET();
    dg_bridge_move_now(&cmd);
    n = g_b.c_move_blocked_gate;
    move_tick(1);
    if (g_b.c_move_blocked_gate != n + 1) bad++;
    if (memcmp(&padmem, &padwas, sizeof padmem)) bad++;
    g_b.tick_status = 0;

    /* 4. PL_SubjectMove nonzero while we do not hold first person refuses:
       the bytes would mean something else there. */
    subject_word = 1;
    TP_RESET();
    dg_bridge_move_now(&cmd);
    n = g_b.c_move_blocked_gate;
    move_tick(1);
    if (g_b.c_move_blocked_gate != n + 1) bad++;
    if (memcmp(&padmem, &padwas, sizeof padmem)) bad++;
    subject_word = 0;

    /* 5. No camera yaw published: bytes still go, dir is left alone. */
    InterlockedExchange(&g_b.s_cam_dir, -1);
    TP_RESET();
    dg_bridge_move_now(&cmd);
    n = g_b.c_move_writes;
    move_tick(1);
    if (g_b.c_move_writes != n + 1) bad++;
    if (*(short *)(padmem.b + 4 + DG_PAD_DIR_OFFSET) != (short)0x7777) bad++;
    InterlockedExchange(&g_b.s_cam_dir, 1024);

    /* 6. Mirror knob: sign -1 flips the yaw, offset adds. */
    InterlockedExchange(&g_b.move_dir_sign, -1);
    InterlockedExchange(&g_b.move_dir_offset, 100);
    TP_RESET();
    dg_bridge_move_now(&cmd);
    move_tick(1);
    dir = *(short *)(padmem.b + 4 + DG_PAD_DIR_OFFSET);
    if (dir != ((-1024 + 100 + 2048 + 2048) & 4095)) bad++;
    InterlockedExchange(&g_b.move_dir_sign, 1);
    InterlockedExchange(&g_b.move_dir_offset, 0);

    /* 7. Prone in first person. Off: the old full byte, dir untouched.
       On: deflection capped at 120 (byte 8), dir written, own counter. */
    InterlockedExchange(&g_b.move_third, 0);
    g_b.fps.state = DG_FPS_ACTIVE;
    subject_word = 1;
    g_b.tick_status = 0x20ULL;
    TP_RESET();
    dg_bridge_move_now(&cmd);
    n = g_b.c_move_writes;
    move_tick(1);
    if (g_b.c_move_writes != n + 1) bad++;
    if (padmem.b[4 + DG_PAD_LEFT_DY_OFFSET] != 1) bad++;
    if (*(short *)(padmem.b + 4 + DG_PAD_DIR_OFFSET) != (short)0x7777) bad++;
    InterlockedExchange(&g_b.move_prone, 1);
    TP_RESET();
    dg_bridge_move_now(&cmd);
    n = g_b.c_move_prone_writes;
    move_tick(1);
    if (g_b.c_move_prone_writes != n + 1) bad++;
    if (padmem.b[4 + DG_PAD_LEFT_DY_OFFSET] != 8) bad++;
    if (!(*(LONG *)(padmem.b + 4 + DG_PAD_STATUS_OFFSET) & DG_MOVE_PAD_U)) bad++;
    dir = *(short *)(padmem.b + 4 + DG_PAD_DIR_OFFSET);
    if (dir != 1024) bad++;
    /* Prone force: deflection 120 -> (120-48)*256/80 = 230, under the run
       threshold 252 and above the walk one 150: the ordinary crawl. */
    if (*(LONG *)(worklmem.b + 0x510) != 1024) bad++;
    if (*(LONG *)(worklmem.b + 0x514) != 230) bad++;
    /* Copy seam live: no redo at all (CheckDirection does it), no refusal. */
    InterlockedExchange(&g_b.seam_is_copy, 1);
    *(LONG *)(worklmem.b + 0x510) = -1; *(LONG *)(worklmem.b + 0x514) = 0;
    TP_RESET();
    dg_bridge_move_now(&cmd);
    n = g_b.c_move_no_workl;
    move_tick(1);
    if (g_b.c_move_no_workl != n) bad++;
    if (*(LONG *)(worklmem.b + 0x510) != -1) bad++;
    if (padmem.b[4 + DG_PAD_LEFT_DY_OFFSET] != 8) bad++;
    InterlockedExchange(&g_b.seam_is_copy, 0);
    /* No workL anchor: bytes still go, the refusal is counted. */
    g_b.workl_ptr = 0;
    TP_RESET();
    dg_bridge_move_now(&cmd);
    n = g_b.c_move_no_workl;
    move_tick(1);
    if (g_b.c_move_no_workl != n + 1) bad++;
    if (padmem.b[4 + DG_PAD_LEFT_DY_OFFSET] != 8) bad++;
    g_b.workl_ptr = (ULONGLONG)(ULONG_PTR)&workl_var;
    /* Standing in first person (no GROUND bit): the cap and dir stay away. */
    g_b.tick_status = 0;
    TP_RESET();
    dg_bridge_move_now(&cmd);
    move_tick(1);
    if (padmem.b[4 + DG_PAD_LEFT_DY_OFFSET] != 1) bad++;
    if (*(short *)(padmem.b + 4 + DG_PAD_DIR_OFFSET) != (short)0x7777) bad++;
#undef TP_RESET

    g_b.a.player_pad = saved_pad;
    g_b.a.pl_subject_move = saved_subj;
    g_b.tick_status = saved_status;
    g_b.fps.state = saved_state;
    InterlockedExchange(&g_b.walk_mode, saved_mode);
    InterlockedExchange(&g_b.turn_mode, saved_turn);
    InterlockedExchange(&g_b.move_third, saved_third);
    InterlockedExchange(&g_b.move_prone, saved_prone);
    InterlockedExchange(&g_b.s_cam_dir, saved_cam);
    InterlockedExchange(&g_b.move_dir_sign, saved_sign);
    InterlockedExchange(&g_b.move_dir_offset, saved_off);
    InterlockedExchange(&g_b.move_prone_max, saved_max);
    InterlockedExchange(&g_b.move_valid, 0);
    g_b.workl_ptr = saved_workl;
    printf("  %-6s third person walks on its own gate (off, WATCH, subject word) "
           "with pad->dir from the camera yaw and no turn; prone caps the "
           "deflection and writes dir; standing first person is untouched\n",
           bad ? "FAIL" : "PASS");
    return bad;
}

static int t_walking_speaks_only_over_silence(void)
{
    union { ULONGLONG align; unsigned char b[0x40]; } padmem, padwas;
    ULONGLONG pad;
    ULONGLONG saved_pad = g_b.a.player_pad;
    ULONGLONG saved_subj = g_b.a.pl_subject_move;
    LONG saved_state = g_b.fps.state;
    LONG saved_mode = g_b.walk_mode;
    LONG saved_turn = g_b.turn_mode;
    LONG subject_word = 1;
    DG_BRIDGE_MOVE cmd;
    LONG n;
    int bad = 0, i;

    memset(&padmem, 0, sizeof padmem);
    pad = (ULONGLONG)(ULONG_PTR)(padmem.b + 4);
    *(LONG *)(padmem.b + 0) = 1;                    /* PlayerPad.enable */
    /* A neutral pad the way the driver leaves one: centred stick bytes. */
    padmem.b[4 + DG_PAD_LEFT_DX_OFFSET] = 128;
    padmem.b[4 + DG_PAD_LEFT_DY_OFFSET] = 128;
    padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] = 128;
    padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET + 1] = 128;

    g_b.a.player_pad = pad;
    g_b.a.pl_subject_move = (ULONGLONG)(ULONG_PTR)&subject_word;
    g_b.fps.state = DG_FPS_ACTIVE;
    InterlockedExchange(&g_b.s_late_unsafe, 0);
    InterlockedExchange(&g_b.armed, 1);
    InterlockedExchange(&g_b.walk_mode, 1);
    InterlockedExchange(&g_b.turn_mode, 0);
    InterlockedExchange(&g_b.move_deadzone_mils, 150);
    InterlockedExchange(&g_b.turn_gain_mils, 1000);
    InterlockedExchange(&g_b.move_valid, 0);
    InterlockedExchange(&g_b.c_ticks, 500);
    padwas = padmem;

    memset(&cmd, 0, sizeof cmd);
    cmd.valid = 1;
    cmd.y = 1.0;

    /* Mode off is inert even with a full stick published. */
    InterlockedExchange(&g_b.walk_mode, 0);
    dg_bridge_move_now(&cmd);
    n = g_b.c_move_writes;
    move_tick(1);
    if (g_b.c_move_writes != n || memcmp(&padmem, &padwas, sizeof padmem)) bad++;
    InterlockedExchange(&g_b.walk_mode, 1);

    /* Full forward, published five times per tick the way the camera seam
       does, consumed once: forward is a LOW dy byte, the analog-in-use flag,
       and PAD_U - the three fields the game needs to walk, together. */
    for (i = 0; i < 5; i++) dg_bridge_move_now(&cmd);
    n = g_b.c_move_writes;
    move_tick(1);
    if (g_b.c_move_writes != n + 1) bad++;
    if (padmem.b[4 + DG_PAD_LEFT_DY_OFFSET] != 1 ||
        padmem.b[4 + DG_PAD_LEFT_DX_OFFSET] != 128) bad++;
    if (!(*(short *)(padmem.b + 4 + DG_PAD_ANALOG_OFFSET) &
          (short)DG_MOVE_ANALOG_L_USE)) bad++;
    if ((*(LONG *)(padmem.b + 4 + DG_PAD_STATUS_OFFSET) &
         (LONG)DG_MOVE_PAD_UDLR) != (LONG)DG_MOVE_PAD_U) bad++;

    /* The player's own dpad silences a full synthetic stick, byte for byte. */
    memset(&padmem, 0, sizeof padmem);
    *(LONG *)(padmem.b + 0) = 1;
    padmem.b[4 + DG_PAD_LEFT_DX_OFFSET] = 128;
    padmem.b[4 + DG_PAD_LEFT_DY_OFFSET] = 128;
    *(LONG *)(padmem.b + 4 + DG_PAD_STATUS_OFFSET) = (LONG)DG_MOVE_PAD_L;
    padwas = padmem;
    dg_bridge_move_now(&cmd);
    n = g_b.c_move_yielded;
    move_tick(1);
    if (g_b.c_move_yielded != n + 1) bad++;
    if (memcmp(&padmem, &padwas, sizeof padmem)) bad++;

    /* A deflected physical stick does the same. */
    *(LONG *)(padmem.b + 4 + DG_PAD_STATUS_OFFSET) = 0;
    padmem.b[4 + DG_PAD_LEFT_DX_OFFSET] = 200;
    padwas = padmem;
    dg_bridge_move_now(&cmd);
    n = g_b.c_move_yielded;
    move_tick(1);
    if (g_b.c_move_yielded != n + 1) bad++;
    if (memcmp(&padmem, &padwas, sizeof padmem)) bad++;

    /* Inside the deadzone: named idle, pad untouched - a resting stick leaves
       no fingerprint at all. */
    padmem.b[4 + DG_PAD_LEFT_DX_OFFSET] = 128;
    padwas = padmem;
    cmd.y = 0.05;
    dg_bridge_move_now(&cmd);
    n = g_b.c_move_idle;
    move_tick(1);
    if (g_b.c_move_idle != n + 1) bad++;
    if (memcmp(&padmem, &padwas, sizeof padmem)) bad++;

    /* A publisher that died mid-walk stops the character: stale, untouched.
       This is the axis version of "no coasting into a wall". */
    cmd.y = 1.0;
    dg_bridge_move_now(&cmd);
    InterlockedExchange(&g_b.c_ticks, 500 + DG_MOVE_FRESH_TICKS + 1);
    n = g_b.c_move_stale;
    move_tick(1);
    if (g_b.c_move_stale != n + 1) bad++;
    if (memcmp(&padmem, &padwas, sizeof padmem)) bad++;
    InterlockedExchange(&g_b.c_ticks, 500);

    /* The gate: first person not held means not one byte, and the refusal has
       its own name. Same for an unvouched controller. */
    g_b.fps.state = DG_FPS_OFF;
    dg_bridge_move_now(&cmd);
    n = g_b.c_move_blocked_gate;
    move_tick(1);
    if (g_b.c_move_blocked_gate != n + 1) bad++;
    if (memcmp(&padmem, &padwas, sizeof padmem)) bad++;
    g_b.fps.state = DG_FPS_ACTIVE;

    cmd.valid = 0;
    dg_bridge_move_now(&cmd);
    n = g_b.c_move_no_command;
    move_tick(1);
    if (g_b.c_move_no_command != n + 1) bad++;
    if (memcmp(&padmem, &padwas, sizeof padmem)) bad++;

    /* PlayerPad.enable clear means the player reads a different record:
       writing ours would land where nobody looks, so it is the gate again. */
    cmd.valid = 1;
    *(LONG *)(padmem.b + 0) = 0;
    padwas = padmem;
    dg_bridge_move_now(&cmd);
    n = g_b.c_move_blocked_gate;
    move_tick(1);
    if (g_b.c_move_blocked_gate != n + 1) bad++;
    if (memcmp(&padmem, &padwas, sizeof padmem)) bad++;

    *(LONG *)(padmem.b + 0) = 1;
    padwas = padmem;
    cmd.valid = 1;
    cmd.y = 1.0;
    subject_word = 0;
    dg_bridge_move_now(&cmd);
    n = g_b.c_move_not_subject;
    move_tick(1);
    if (g_b.c_move_not_subject != n + 1) bad++;
    if (memcmp(&padmem, &padwas, sizeof padmem)) bad++;
    /* And an absent anchor refuses identically - fail closed, never a read
       through zero. */
    g_b.a.pl_subject_move = 0;
    dg_bridge_move_now(&cmd);
    n = g_b.c_move_not_subject;
    move_tick(1);
    if (g_b.c_move_not_subject != n + 1) bad++;
    if (memcmp(&padmem, &padwas, sizeof padmem)) bad++;
    g_b.a.pl_subject_move = (ULONGLONG)(ULONG_PTR)&subject_word;
    subject_word = 1;

    /* The turn half. Full right on the right stick becomes a right_dx byte
       past the game's margin plus the R_USE flag - and the walk bytes stay
       exactly where the walk left them: the two halves are independent. The
       pad is re-neutralised in full first: the dpad leg above memsets it and
       leaves right_dx at 0, which the yield correctly reads as a physical
       full-left stick - a fine test of the yield, but not the one this leg
       is making. */
    memset(&padmem, 0, sizeof padmem);
    *(LONG *)(padmem.b + 0) = 1;
    padmem.b[4 + DG_PAD_LEFT_DX_OFFSET] = 128;
    padmem.b[4 + DG_PAD_LEFT_DY_OFFSET] = 128;
    padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] = 128;
    padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET + 1] = 128;
    InterlockedExchange(&g_b.turn_mode, 1);
    memset(&cmd, 0, sizeof cmd);
    cmd.valid = 1;
    cmd.turn_valid = 1;
    cmd.turn_x = 1.0;
    dg_bridge_move_now(&cmd);
    n = g_b.c_turn_writes;
    move_tick(1);
    if (g_b.c_turn_writes != n + 1) bad++;
    if (padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] <= 128 + DG_MOVE_GAME_MARGIN)
        bad++;
    if (!(*(short *)(padmem.b + 4 + DG_PAD_ANALOG_OFFSET) &
          (short)DG_MOVE_ANALOG_R_USE)) bad++;
    if (padmem.b[4 + DG_PAD_LEFT_DX_OFFSET] != 128 ||
        padmem.b[4 + DG_PAD_LEFT_DY_OFFSET] != 128) bad++;

    /* Left deflection turns left: a LOW byte. */
    padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] = 128;
    *(short *)(padmem.b + 4 + DG_PAD_ANALOG_OFFSET) = 0;
    cmd.turn_x = -1.0;
    dg_bridge_move_now(&cmd);
    move_tick(1);
    if (padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] >= 128 - DG_MOVE_GAME_MARGIN)
        bad++;

    padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] = 128;
    *(short *)(padmem.b + 4 + DG_PAD_ANALOG_OFFSET) = 0;
    cmd.turn_x = 0.16;
    dg_bridge_move_now(&cmd);
    n = g_b.c_turn_writes;
    move_tick(1);
    if (g_b.c_turn_writes != n + 1) bad++;
    if (padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] < 128 + DG_MOVE_SUBJECT_MARGIN)
        bad++;

    /* The player's own right stick silences the turn - and ONLY the turn:
       the walk must still be free to speak on the same tick. */
    padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] = 220;   /* physical right stick */
    *(short *)(padmem.b + 4 + DG_PAD_ANALOG_OFFSET) = 0;
    padwas = padmem;
    cmd.y = 1.0;
    cmd.turn_x = -1.0;
    dg_bridge_move_now(&cmd);
    n = g_b.c_turn_yielded;
    move_tick(1);
    if (g_b.c_turn_yielded != n + 1) bad++;
    if (padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] != 220) bad++;      /* theirs */
    if (padmem.b[4 + DG_PAD_LEFT_DY_OFFSET] == 128) bad++;       /* ours   */
    padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] = 128;

    /* And the mirror: a physical LEFT stick yields the walk while the turn
       still speaks. */
    padmem.b[4 + DG_PAD_LEFT_DX_OFFSET] = 20;
    padmem.b[4 + DG_PAD_LEFT_DY_OFFSET] = 128;
    *(short *)(padmem.b + 4 + DG_PAD_ANALOG_OFFSET) = 0;
    dg_bridge_move_now(&cmd);
    n = g_b.c_move_yielded;
    move_tick(1);
    if (g_b.c_move_yielded != n + 1) bad++;
    if (padmem.b[4 + DG_PAD_LEFT_DX_OFFSET] != 20) bad++;        /* theirs */
    if (padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] >= 128 - DG_MOVE_GAME_MARGIN)
        bad++;                                                   /* ours   */

    /* Both sticks at once: two writes, one tick, neither suppressing the
       other - walking while turning is the whole point of putting the turn on
       the stick at all. */
    padmem.b[4 + DG_PAD_LEFT_DX_OFFSET] = 128;
    padmem.b[4 + DG_PAD_LEFT_DY_OFFSET] = 128;
    padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] = 128;
    *(short *)(padmem.b + 4 + DG_PAD_ANALOG_OFFSET) = 0;
    *(LONG *)(padmem.b + 4 + DG_PAD_STATUS_OFFSET) = 0;
    cmd.y = 1.0;
    cmd.turn_x = 1.0;
    cmd.turn_valid = 1;
    dg_bridge_move_now(&cmd);
    n = g_b.c_turn_writes;
    i = (int)g_b.c_move_writes;
    move_tick(1);
    if (g_b.c_turn_writes != n + 1 || (int)g_b.c_move_writes != i + 1) bad++;
    if (padmem.b[4 + DG_PAD_LEFT_DY_OFFSET] == 128) bad++;
    if (padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] <= 128 + DG_MOVE_GAME_MARGIN)
        bad++;

    /* Walk OFF with turn ON: the left stick must not reach the pad - the
       off-switch is a blanking of the input, and this leg is what notices if
       that blanking ever falls out. */
    InterlockedExchange(&g_b.walk_mode, 0);
    padmem.b[4 + DG_PAD_LEFT_DX_OFFSET] = 128;
    padmem.b[4 + DG_PAD_LEFT_DY_OFFSET] = 128;
    padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] = 128;
    *(short *)(padmem.b + 4 + DG_PAD_ANALOG_OFFSET) = 0;
    /* Status too: the leg above left its own PAD_U in it, and a stray dpad
       bit turns this leg into a yield test - which would pass with the very
       blanking it exists to check ripped out. */
    *(LONG *)(padmem.b + 4 + DG_PAD_STATUS_OFFSET) = 0;
    dg_bridge_move_now(&cmd);
    move_tick(1);
    if (padmem.b[4 + DG_PAD_LEFT_DX_OFFSET] != 128 ||
        padmem.b[4 + DG_PAD_LEFT_DY_OFFSET] != 128) bad++;
    if (padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] <= 128 + DG_MOVE_GAME_MARGIN)
        bad++;
    InterlockedExchange(&g_b.walk_mode, 1);

    /* An unvouched right controller is a centred turn, not a frozen one. */
    padmem.b[4 + DG_PAD_LEFT_DX_OFFSET] = 128;
    padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] = 128;
    *(short *)(padmem.b + 4 + DG_PAD_ANALOG_OFFSET) = 0;
    padwas = padmem;
    cmd.y = 0.0;
    cmd.turn_valid = 0;
    cmd.turn_x = 1.0;
    dg_bridge_move_now(&cmd);
    n = g_b.c_turn_writes;
    move_tick(1);
    if (g_b.c_turn_writes != n) bad++;
    if (padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] != 128) bad++;

    InterlockedExchange(&g_b.move_valid, 0);
    InterlockedExchange(&g_b.walk_mode, saved_mode);
    InterlockedExchange(&g_b.turn_mode, saved_turn);
    InterlockedExchange(&g_b.armed, 0);
    InterlockedExchange(&g_b.c_ticks, 0);
    g_b.a.player_pad = saved_pad;
    g_b.a.pl_subject_move = saved_subj;
    g_b.fps.state = saved_state;

    printf("  %-6s walking+turning: coherent pad writes on their own sticks, "
           "independent yields, and every refusal - yield, idle, stale, gate, "
           "unvouched, wrong record, not-subject - leaves the pad "
           "byte-identical\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

/* U2: the pad seam, the one place this project writes into the game's own
   input record. Everything here is a refusal except the last leg, because a
   stuck or misdirected menu button is the worst failure this feature has:
   the front-end's confirm sits on "overwrite this save". */
/* The ordered press log is the instrument that will name this build's
   confirm button, and the last reading of this same data was wrong: a
   histogram was read, a guess was made about which of two words was
   confirm, and it was the other one. So the order gets a test - including
   the fold, because a fold that swallowed repeats would turn five
   deliberate presses into one entry and mislead exactly as badly. */
static int t_press_order_keeps_presses_in_order(void)
{
    union { ULONGLONG align; unsigned char b[0x40]; } rec;
    static LONG menu_word, menu_scn;
    DG_ANCHORS saved = g_b.a;
    LONG saved_mode = g_b.menu_mode;
    DWORD *press = (DWORD *)(rec.b + DG_GV_PAD_PRESS_OFFSET);
    int j, bad = 0;

    memset(&rec, 0, sizeof rec);
    menu_word = 0;
    menu_scn = 0;
    memset(&g_b.a, 0, sizeof g_b.a);
    g_b.a.gv_pad_data_direct = (ULONGLONG)(ULONG_PTR)rec.b;
    g_b.a.gm_menu_status = (ULONGLONG)(ULONG_PTR)&menu_word;
    g_b.a.gm_menu_status_scn = (ULONGLONG)(ULONG_PTR)&menu_scn;
    InterlockedExchange(&g_b.armed, 1);
    InterlockedExchange(&g_b.c_ticks, 100);
    /* MEASURE, deliberately: the whole point of this record is that it is
       filled while the mod writes nothing, so the game's own buttons can be
       read without ours mixed in. */
    InterlockedExchange(&g_b.menu_mode, 1);
    g_b.press_seq_n = 0;
    for (j = 0; j < DG_PAD_PRESS_SEQ; j++) {
        g_b.press_seq_word[j] = 0;
        g_b.press_seq_ms[j] = 0;
    }

    *press = 0x00000020;   pad_seam_tick();     /* circle */
    if (g_b.press_seq_n != 1) bad++;
    if (g_b.press_seq_word[0] != 0x00000020) bad++;

    /* The same word again on the very next frame is one button crossing the
       edge detector twice, not a second press. */
    pad_seam_tick();
    if (g_b.press_seq_n != 1) bad++;

    *press = 0x00000040;   pad_seam_tick();     /* cross */
    *press = 0x00000020;   pad_seam_tick();     /* circle again */
    if (g_b.press_seq_n != 3) bad++;
    if (g_b.press_seq_word[1] != 0x00000040) bad++;
    if (g_b.press_seq_word[2] != 0x00000020) bad++;

    /* A frame with no press is not an event. */
    *press = 0;            pad_seam_tick();
    if (g_b.press_seq_n != 3) bad++;

    /* And a genuine repeat, far enough apart to be a human pressing the
       same key twice, must NOT be folded - five confirms collapsing into
       one entry would misname the button as surely as reading the wrong
       half of a pair did. */
    Sleep(150);
    *press = 0x00000020;   pad_seam_tick();
    if (g_b.press_seq_n != 4) bad++;
    if (g_b.press_seq_word[3] != 0x00000020) bad++;

    /* Past the kept window the OLDEST go, and the running total keeps
       counting - a log that silently dropped the newest would show the
       navigation and hide the confirm. */
    for (j = 0; j < DG_PAD_PRESS_SEQ; j++) {
        *press = (DWORD)(0x00010000u + (unsigned)j);
        pad_seam_tick();
    }
    if (g_b.press_seq_n != 4 + DG_PAD_PRESS_SEQ) bad++;
    {
        LONG total = g_b.press_seq_n;
        LONG newest = g_b.press_seq_word[(total - 1) % DG_PAD_PRESS_SEQ];
        LONG oldest_kept =
            g_b.press_seq_word[(total - DG_PAD_PRESS_SEQ) % DG_PAD_PRESS_SEQ];
        if (newest != (LONG)(0x00010000u + DG_PAD_PRESS_SEQ - 1)) bad++;
        if (oldest_kept != 0x00010000) bad++;
    }

    g_b.a = saved;
    InterlockedExchange(&g_b.menu_mode, saved_mode);
    InterlockedExchange(&g_b.armed, 0);
    g_b.press_seq_n = 0;

    printf("  %-6s press order: distinct presses are kept in sequence, one "
           "button across two frames is one entry, a real repeat 150 ms "
           "later is two, an empty frame is none, and past %d the oldest "
           "are dropped while the newest and the total survive\n",
           bad ? "FAIL" : "ok", DG_PAD_PRESS_SEQ);
    return bad ? 1 : 0;
}

static int t_menu_fresh_start_config(void)
{
    static const int values[] = { 0, 1, 2, -1, 3 };
    void *saved;
    DG_BRIDGE_CONFIG cfg;
    DG_BRIDGE_MENU saved_cmd = g_script_menu_pending;
    ULONGLONG saved_deadline = g_script_menu_deadline;
    LONG saved_epoch = g_script_menu_pending_epoch, saved_cancel = g_script_menu_cancel;
    char path[MAX_PATH];
    const char *leaf;
    int i, bad = 0;
    /* The real startup can only reach its fingerprint refusal in this test
       executable. Refuse to run this test under the game's filename. */
    if (!GetModuleFileNameA(NULL, path, sizeof path)) return 1;
    leaf = strrchr(path, '\\'); leaf = leaf ? leaf + 1 : path;
    if (!_stricmp(leaf, GAME_EXE)) return 1;
    saved = malloc(sizeof g_b);
    if (!saved) return 1;
    memcpy(saved, &g_b, sizeof g_b);
    memset(&cfg, 0, sizeof cfg);
    if (sane_menu_mode(NULL) != 0) bad++;
    for (i = 0; i < (int)(sizeof values / sizeof values[0]); i++) {
        int expected = i < 3 ? values[i] : 0;
        memset(&g_b, 0, sizeof g_b);
        g_b.menu_mode = 99;
        cfg.menu_mode = values[i];
        if (dg_bridge_start(NULL, &cfg) != 0 || g_b.started || g_b.armed ||
            g_b.pad_detour_live || g_b.menu_mode != expected) bad++;
    }
    memcpy(&g_b, saved, sizeof g_b);
    free(saved);
    g_script_menu_pending = saved_cmd; g_script_menu_deadline = saved_deadline;
    g_script_menu_pending_epoch = saved_epoch; g_script_menu_cancel = saved_cancel;
    return bad;
}

static int t_script_menu_mailbox(void)
{
    void *saved = malloc(sizeof g_b);
    DG_BRIDGE_MENU saved_cmd = g_script_menu_pending;
    ULONGLONG saved_deadline = g_script_menu_deadline, arm = 0;
    LONG saved_epoch = g_script_menu_pending_epoch, saved_cancel = g_script_menu_cancel;
    DWORD direct[10] = {0};
    DG_BRIDGE_MENU cmd;
    int bad = 0;
    if (!saved) return 1;
    memcpy(saved, &g_b, sizeof g_b);
    memset(&g_b, 0, sizeof g_b);
    g_b.armed = 1; g_b.script_menu_only = 1; g_b.menu_mode = 2;
    g_b.a.gm_player_arm_body = (ULONGLONG)(ULONG_PTR)&arm;
    g_b.a.gv_pad_data_direct = (ULONGLONG)(ULONG_PTR)direct;
    cmd.status = 0x20; cmd.clear = 0x40; cmd.allow = 1;
    dg_bridge_menu_now(&cmd);
    /* Split coverage: dg_hook's test_script_menu_session tests the real
       preserve-neutral predicate. Here no publication models that decision;
       this does not simulate an integrated Present/pad scheduling run. */
    pad_seam_tick();
    if (direct[1] != 0x20 || direct[2] != 0x20 || g_b.c_menu_writes != 1) bad++;
    pad_seam_tick();
    if (g_b.c_menu_writes != 1) bad++;
    dg_bridge_menu_now(&cmd);
    g_script_menu_deadline = GetTickCount64(); /* deterministic expiry; no sleep */
    pad_seam_tick();
    if (g_b.c_menu_writes != 1 || g_b.c_menu_stale != 1 || g_b.c_ticks) bad++;
    dg_bridge_menu_now(&cmd);
    dg_bridge_menu_now(NULL);
    pad_seam_tick();
    if (g_b.c_menu_writes != 1) bad++;
    dg_bridge_menu_now(&cmd);
    cmd.status = 0x40; cmd.clear = 0x20;
    dg_bridge_menu_now(&cmd);
    pad_seam_tick();
    if (direct[1] != 0x40 || direct[2] != 0x40 || g_b.c_menu_writes != 2) bad++;
    dg_bridge_menu_now(&cmd);
    arm = 1;
    pad_seam_tick();
    arm = 0;
    pad_seam_tick();
    if (g_b.c_menu_writes != 2) bad++;
    dg_bridge_menu_now(&cmd);
    cmd.status = 0;
    dg_bridge_menu_now(&cmd);
    cmd.status = 0x40;
    pad_seam_tick();
    if (g_b.c_menu_writes != 2) bad++;
    dg_bridge_menu_now(&cmd);
    dg_bridge_stop(); /* started==0 must still withdraw the pending mailbox */
    g_b.script_menu_only = 1;
    pad_seam_tick();
    if (g_b.c_menu_writes != 2) bad++;
    dg_bridge_menu_now(&cmd);
    AcquireSRWLockExclusive(&g_script_menu_lock);
    script_menu_clear(1); /* failed try-lock must still invalidate publication */
    ReleaseSRWLockExclusive(&g_script_menu_lock);
    pad_seam_tick();
    if (g_b.c_menu_writes != 2) bad++;
    g_b.script_menu_only = 0;
    dg_bridge_menu_now(&cmd);
    cmd.status = 0;
    dg_bridge_menu_now(&cmd); /* ordinary route retains neutral withdrawal */
    pad_seam_tick();
    if (g_b.c_menu_writes != 2) bad++;
    memcpy(&g_b, saved, sizeof g_b);
    free(saved);
    g_script_menu_pending = saved_cmd; g_script_menu_deadline = saved_deadline;
    g_script_menu_pending_epoch = saved_epoch; g_script_menu_cancel = saved_cancel;
    return bad;
}

static int t_script_menu_context_guard(void)
{
    static ULONGLONG arm;
    DG_ANCHORS saved_a = g_b.a;
    LONG saved_armed = g_b.armed, saved_only = g_b.script_menu_only;
    LONG saved_pending = g_b.start_pending, saved_status = g_b.menu_status;
    LONG saved_allow = g_b.menu_allow, saved_clear = g_b.menu_clear;
    LONG saved_pad_live = g_b.pad_detour_live;
    LONG saved_menu_mode = g_b.menu_mode;
    LONG saved_entries = g_b.c_pad_seam_entries, saved_queued = g_b.c_start_queued;
    LONG saved_consumed = g_b.c_start_consumed, saved_refused = g_b.c_pad_context_refused;
    int bad = 0;
    memset(&g_b.a, 0, sizeof g_b.a);
    g_b.armed = 1;
    g_b.script_menu_only = 1;
    g_b.start_pending = 0;
    g_b.pad_detour_live = 1;
    g_b.a.pad_press_ok = 1;
    if (dg_bridge_menu_context_ready()) bad++; /* missing anchor */
    g_b.a.gm_player_arm_body = (ULONGLONG)(ULONG_PTR)&arm;
    arm = 0;
    if (!dg_bridge_menu_context_ready()) bad++;
    dg_bridge_start_now();
    if (g_b.start_pending != 1) bad++;
    if (g_b.c_start_queued != saved_queued + 1) bad++;
    /* A queued START and menu publication become invalid before the pad
       seam. Invalid nonzero is rejected without dereferencing the object. */
    arm = 1;
    if (dg_bridge_menu_context_ready()) bad++;
    g_b.menu_status = 0x8000;
    g_b.menu_clear = 0x100;
    g_b.menu_allow = 1;
    pad_seam_tick(); /* no pad addresses: a write would fault */
    if (g_b.start_pending || g_b.menu_status || g_b.menu_allow || g_b.menu_clear) bad++;
    if (g_b.c_pad_seam_entries != saved_entries + 1 ||
        g_b.c_pad_context_refused != saved_refused + 1 ||
        g_b.c_start_consumed != saved_consumed) bad++;
    dg_bridge_start_now();
    if (g_b.start_pending) bad++;
    if (g_b.c_start_queued != saved_queued + 1) bad++;
    arm = (ULONGLONG)(ULONG_PTR)&arm; /* valid nonzero object also refuses */
    if (dg_bridge_menu_context_ready()) bad++;
    arm = 0;
    g_b.menu_mode = 0;
    dg_bridge_start_now();
    pad_seam_tick(); /* consumed counts dequeue, independently of pad address availability */
    if (g_b.c_start_queued != saved_queued + 2 ||
        g_b.c_start_consumed != saved_consumed + 1 ||
        g_b.c_pad_seam_entries != saved_entries + 2 ||
        g_b.c_pad_context_refused != saved_refused + 1) bad++;
    {
        DWORD direct[10] = {0}, normal[10] = {0}, normal_before[10];
        DWORD scenario = 0x10;
        direct[1] = 0x20; direct[2] = 0x40;
        normal[9] = 0x400;
        memcpy(normal_before, normal, sizeof normal);
        g_b.a.gv_pad_data_direct = (ULONGLONG)(ULONG_PTR)direct;
        g_b.a.gv_pad_data = (ULONGLONG)(ULONG_PTR)normal;
        g_b.a.gv_pad_press = (ULONGLONG)(ULONG_PTR)&scenario;
        dg_bridge_start_now();
        pad_seam_tick();
        if (direct[1] != 0x00100020 || direct[2] != 0x00100040 ||
            scenario != 0x10 || memcmp(normal, normal_before, sizeof normal)) bad++;
        /* The frontend-only route needs no normal/scenario addresses. */
        direct[1] = 0x20; direct[2] = 0x40;
        g_b.a.gv_pad_data = g_b.a.gv_pad_press = 0;
        dg_bridge_start_now();
        pad_seam_tick();
        if (direct[1] != 0x00100020 || direct[2] != 0x00100040) bad++;
        /* A context change after queueing still prevents every write. */
        direct[1] = 0x20; direct[2] = 0x40;
        dg_bridge_start_now();
        arm = 1;
        pad_seam_tick();
        if (direct[1] != 0x20 || direct[2] != 0x40 || g_b.start_pending) bad++;
        arm = 0;
        /* Ordinary START retains the original three-record behavior. */
        g_b.script_menu_only = 0;
        g_b.a.gv_pad_data = (ULONGLONG)(ULONG_PTR)normal;
        g_b.a.gv_pad_press = (ULONGLONG)(ULONG_PTR)&scenario;
        dg_bridge_start_now();
        pad_seam_tick();
        if (direct[1] != 0x820 || direct[2] != 0x840 ||
            scenario != 0x810 || normal[9] != 0x420) bad++;
    }
    g_b.armed = 0;
    if (dg_bridge_menu_context_ready()) bad++;
    g_b.a = saved_a;
    g_b.armed = saved_armed; g_b.script_menu_only = saved_only;
    g_b.start_pending = saved_pending; g_b.menu_status = saved_status;
    g_b.menu_allow = saved_allow; g_b.menu_clear = saved_clear;
    g_b.pad_detour_live = saved_pad_live;
    g_b.menu_mode = saved_menu_mode;
    InterlockedExchange(&g_b.c_pad_seam_entries, saved_entries);
    InterlockedExchange(&g_b.c_start_queued, saved_queued);
    InterlockedExchange(&g_b.c_start_consumed, saved_consumed);
    InterlockedExchange(&g_b.c_pad_context_refused, saved_refused);
    bad += t_script_menu_mailbox();
    bad += t_menu_fresh_start_config();
    printf("  %-6s script menu context: input guard, title bit, mailbox expiry/consume/replace/withdraw and normal isolation\n", bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

static int t_menu_seam_writes_only_where_it_may(void)
{
    union { ULONGLONG align; unsigned char b[0x40]; } rec;
    static LONG menu_word, menu_scn;
    DG_ANCHORS saved = g_b.a;
    LONG saved_mode = g_b.menu_mode;
    DG_BRIDGE_MENU cmd;
    int bad = 0;
    DWORD *status = (DWORD *)(rec.b + DG_GV_PAD_STATUS_OFFSET);
    DWORD *press  = (DWORD *)(rec.b + DG_GV_PAD_PRESS_OFFSET);

    memset(&rec, 0, sizeof rec);
    menu_word = 0;
    menu_scn = 0;
    memset(&g_b.a, 0, sizeof g_b.a);
    g_b.a.gv_pad_data_direct = (ULONGLONG)(ULONG_PTR)rec.b;
    g_b.a.gm_menu_status = (ULONGLONG)(ULONG_PTR)&menu_word;
    g_b.a.gm_menu_status_scn = (ULONGLONG)(ULONG_PTR)&menu_scn;
    InterlockedExchange(&g_b.armed, 1);
    InterlockedExchange(&g_b.c_ticks, 100);
    InterlockedExchange(&g_b.menu_mode, 2);
    InterlockedExchange(&g_b.c_menu_writes, 0);
    InterlockedExchange(&g_b.c_menu_stale, 0);
    InterlockedExchange(&g_b.c_menu_gate, 0);

    cmd.status = DG_MENU_PAD_D;
    cmd.allow = 1;

    /* Mode below write: decided, published, and still not written. */
    InterlockedExchange(&g_b.menu_mode, 1);
    dg_bridge_menu_now(&cmd);
    pad_seam_tick();
    if (*status || *press || g_b.c_menu_writes) bad++;
    InterlockedExchange(&g_b.menu_mode, 2);

    /* A withdrawn command writes nothing - stopping IS the release. */
    dg_bridge_menu_now(NULL);
    pad_seam_tick();
    if (*status || *press || g_b.c_menu_writes) bad++;

    /* The ordinary case: status AND press, because the menus read the
       flank and UpdatePad has already derived it by the time we run. */
    dg_bridge_menu_now(&cmd);
    pad_seam_tick();
    if (*status != DG_MENU_PAD_D || *press != DG_MENU_PAD_D) bad++;
    if (g_b.c_menu_writes != 1) bad++;

    /* One shot. A second pass without a new command must add nothing: a
       repeat here would be a held button the player never held. */
    *status = 0;
    *press = 0;
    pad_seam_tick();
    if (*status || *press || g_b.c_menu_writes != 1) bad++;

    /* Bits outside the allowed set never reach the record, however they
       got into the command. */
    cmd.status = DG_MENU_PAD_D | 0x00000004u;   /* 0x4 is not a menu button */
    dg_bridge_menu_now(&cmd);
    pad_seam_tick();
    if (*status != DG_MENU_PAD_D || *press != DG_MENU_PAD_D) bad++;
    cmd.status = DG_MENU_PAD_D;

    /* A weapon or item menu is out of scope by the user's decision, and the
       flat-screen condition upstream is TRUE while one is open - so the
       refusal has to live here, on the game thread, reading the live word. */
    *status = 0;
    *press = 0;
    menu_word = 0x00000100;                     /* MENU_WEAPON_OPEN */
    dg_bridge_menu_now(&cmd);
    pad_seam_tick();
    if (*status || *press) bad++;
    if (g_b.c_menu_gate != 1) bad++;
    menu_word = 0;
    menu_scn = 0x00000200;                      /* MENU_ITEM_OPEN, scn half */
    dg_bridge_menu_now(&cmd);
    pad_seam_tick();
    if (*status || *press) bad++;
    if (g_b.c_menu_gate != 2) bad++;
    menu_scn = 0;

    /* A command from a Present hook that stopped firing is refused rather
       than repeated. */
    dg_bridge_menu_now(&cmd);
    InterlockedExchange(&g_b.c_ticks, 200);
    pad_seam_tick();
    if (*status || *press) bad++;
    if (g_b.c_menu_stale != 1) bad++;
    InterlockedExchange(&g_b.c_ticks, 100);

    /* An unarmed bridge writes nothing, whatever was published. */
    dg_bridge_menu_now(&cmd);
    InterlockedExchange(&g_b.armed, 0);
    pad_seam_tick();
    if (*status || *press) bad++;
    InterlockedExchange(&g_b.armed, 1);

    /* No record address: refuse rather than write to zero. */
    g_b.a.gv_pad_data_direct = 0;
    dg_bridge_menu_now(&cmd);
    pad_seam_tick();
    if (*status || *press) bad++;
    g_b.a.gv_pad_data_direct = (ULONGLONG)(ULONG_PTR)rec.b;

    *status = 0x00040040u;                      /* as if the runtime set it */
    *press = 0x00040040u;
    cmd.status = DG_MENU_PAD_A;
    cmd.clear = 0x00040040u;
    dg_bridge_menu_now(&cmd);
    pad_seam_tick();
    if (*status != DG_MENU_PAD_A || *press != DG_MENU_PAD_A) bad++;

    /* A clear reaching outside the allowed set is refused like any other
       bit: the marker may pick which BUTTON to silence, never more. */
    *status = 0x01000000u;
    *press = 0x01000000u;
    cmd.status = DG_MENU_PAD_A;
    cmd.clear = 0x01000000u;
    dg_bridge_menu_now(&cmd);
    pad_seam_tick();
    if (*status != (0x01000000u | DG_MENU_PAD_A)) bad++;
    cmd.clear = 0u;

    /* And with nothing to say, nothing is cleared either - a clear is not
       a licence to hold a button down for the player. */
    *status = 0x00040040u;
    *press = 0x00040040u;
    cmd.status = 0u;
    cmd.clear = 0x00040040u;
    dg_bridge_menu_now(&cmd);
    pad_seam_tick();
    if (*status != 0x00040040u || *press != 0x00040040u) bad++;
    cmd.status = DG_MENU_PAD_D;
    cmd.clear = 0u;

    /* Codec B consumes on the actual post-UpdatePad seam. The native cancel
       word differs from the configured front-end word; SELECT also reaches
       frequency selection independently of the runtime region assignment. */
    *status = *press = 0;
    menu_scn = 0x400u;
    cmd.status = 0x00040020u;
    cmd.allow = DG_CODEC_MENU_ALLOW_EXIT;
    dg_bridge_menu_now(&cmd);
    pad_seam_tick();
    if (*status != 0x00040140u || *press != 0x00040140u) bad++;
    *status = *press = 0;
    pad_seam_tick();
    if (*status || *press) bad++;
    dg_bridge_menu_now(&cmd);
    menu_scn = 0; /* codec ended between publication and consumption */
    pad_seam_tick();
    if (*status || *press || g_b.menu_status) bad++;
    menu_scn = 0x400u;
    *status = DG_MENU_PAD_STA;
    dg_bridge_menu_now(&cmd);
    pad_seam_tick();
    if (*status != DG_MENU_PAD_STA || *press || g_b.menu_status) bad++;
    *status = 0;
    *press = DG_MENU_PAD_STA;
    dg_bridge_menu_now(&cmd);
    pad_seam_tick();
    if (*status || *press != DG_MENU_PAD_STA || g_b.menu_status) bad++;
    *status = *press = 0;
    menu_scn = 0;
    cmd.allow = 1;
    cmd.status = DG_MENU_PAD_D;

    /* And the record is otherwise untouched: only two dwords ever move. */
    {
        int k;
        memset(&rec, 0, sizeof rec);
        *status = 0;
        *press = 0;
        dg_bridge_menu_now(&cmd);
        pad_seam_tick();
        for (k = 0; k < 0x40; k++) {
            if (k >= DG_GV_PAD_STATUS_OFFSET &&
                k < DG_GV_PAD_STATUS_OFFSET + 4) continue;
            if (k >= DG_GV_PAD_PRESS_OFFSET &&
                k < DG_GV_PAD_PRESS_OFFSET + 4) continue;
            if (rec.b[k] != 0) bad++;
        }
    }

    InterlockedExchange(&g_b.armed, 0);
    InterlockedExchange(&g_b.menu_mode, saved_mode);
    InterlockedExchange(&g_b.menu_status, 0);
    InterlockedExchange(&g_b.menu_allow, 0);
    InterlockedExchange(&g_b.c_menu_writes, 0);
    InterlockedExchange(&g_b.c_menu_stale, 0);
    InterlockedExchange(&g_b.c_menu_gate, 0);
    InterlockedExchange(&g_b.c_ticks, 0);
    g_b.a = saved;

    printf("  %-6s menu seam: the front-end record takes status and press "
           "together, exactly once per command, only the allowed bits, and "
           "never at all with the mode down, the command withdrawn or stale, "
           "the bridge unarmed, no record, or a weapon/item menu open\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

static int t_body_follow_speaks_for_the_aim(void)
{
    union { ULONGLONG align; unsigned char b[0x40]; } padmem;
    ULONGLONG saved_pad = g_b.a.player_pad;
    ULONGLONG saved_subj = g_b.a.pl_subject_move;
    LONG saved_state = g_b.fps.state;
    LONG saved_mode = g_b.walk_mode;
    LONG saved_turn = g_b.turn_mode;
    LONG subject_word = 1;
    DG_BRIDGE_MOVE cmd;
    LONG n, fw, acc;
    int bad = 0, i, k;

    memset(&padmem, 0, sizeof padmem);
    g_b.a.player_pad = (ULONGLONG)(ULONG_PTR)(padmem.b + 4);
    *(LONG *)(padmem.b + 0) = 1;
    padmem.b[4 + DG_PAD_LEFT_DX_OFFSET] = 128;
    padmem.b[4 + DG_PAD_LEFT_DY_OFFSET] = 128;
    padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] = 128;
    padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET + 1] = 128;
    g_b.a.pl_subject_move = (ULONGLONG)(ULONG_PTR)&subject_word;
    g_b.fps.state = DG_FPS_ACTIVE;
    InterlockedExchange(&g_b.s_late_unsafe, 0);
    InterlockedExchange(&g_b.armed, 1);
    InterlockedExchange(&g_b.walk_mode, 1);
    InterlockedExchange(&g_b.turn_mode, 1);
    InterlockedExchange(&g_b.move_deadzone_mils, 150);
    InterlockedExchange(&g_b.turn_gain_mils, 1000);
    InterlockedExchange(&g_b.turn_follow_thresh_mdeg, 40000);
    InterlockedExchange(&g_b.turn_follow_full_mdeg, 90000);
    InterlockedExchange(&g_b.turn_dir_acc, 0);
    InterlockedExchange(&g_b.move_valid, 0);
    /* A previous test may have left the fire machine mid-hold, and a held
       trigger tightens the dead band below - every threshold assertion
       here assumes the walking band. */
    InterlockedExchange(&g_b.s_fire_state, DG_FIRE_IDLE);
    InterlockedExchange(&g_b.c_ticks, 500);

    /* The sign learns from drift responses to written bytes: twenty pairs
       of +1 degree under a fresh positive-side write commit the
       accumulator; a recalibration snap and sub-noise wiggle vote
       nothing. */
    g_b.turn_vote_have_prev = 0;
    InterlockedExchange(&g_b.turn_last_write_dir, 0);
    turn_dir_vote(10.0);
    InterlockedExchange(&g_b.turn_last_write_dir, 1);
    InterlockedExchange(&g_b.turn_last_write_tick, 499);
    for (i = 1; i <= 20; i++) turn_dir_vote(10.0 + (double)i);
    acc = InterlockedCompareExchange(&g_b.turn_dir_acc, 0, 0);
    if (acc < 15) bad++;
    turn_dir_vote(120.0);               /* +90 in one pair: a recal snap */
    if (InterlockedCompareExchange(&g_b.turn_dir_acc, 0, 0) != acc) bad++;
    turn_dir_vote(120.05);              /* under the noise floor */
    if (InterlockedCompareExchange(&g_b.turn_dir_acc, 0, 0) != acc) bad++;

    /* The gap arrives as a published fact; who publishes it is the
       position path's business and is tested there
       (t_the_gap_reads_the_facing_minus_the_body). This test owns the
       CONSUMER: what move_tick does with a fresh +60. */
    InterlockedExchange(&g_b.s_arm_aim_gap, f2l(60.0f));
    InterlockedExchange(&g_b.arm_aim_gap_tick, 500);
    (void)k;

    /* A fresh +60 gap past the 40 threshold with a committed positive
       sign: the follow turns, on the positive byte side, counted as its
       own. */
    memset(&cmd, 0, sizeof cmd);
    cmd.valid = 1;
    cmd.turn_valid = 1;
    dg_bridge_move_now(&cmd);
    n = g_b.c_turn_writes;
    fw = g_b.c_turn_follow_writes;
    move_tick(1);
    if (g_b.c_turn_writes != n + 1) bad++;
    if (g_b.c_turn_follow_writes != fw + 1) bad++;
    if (padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] < 128 + DG_MOVE_SUBJECT_MARGIN)
        bad++;

    /* A committed NEGATIVE sign flips the byte side for the same gap. */
    padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] = 128;
    *(short *)(padmem.b + 4 + DG_PAD_ANALOG_OFFSET) = 0;
    InterlockedExchange(&g_b.turn_dir_acc, -30);
    dg_bridge_move_now(&cmd);
    move_tick(1);
    if (padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] > 128 - DG_MOVE_SUBJECT_MARGIN)
        bad++;
    InterlockedExchange(&g_b.turn_dir_acc, 30);

    /* And the mirror gap: aiming the other way round flips the byte too -
       the follow turns TOWARD the aim, not one habitual way. */
    padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] = 128;
    *(short *)(padmem.b + 4 + DG_PAD_ANALOG_OFFSET) = 0;
    InterlockedExchange(&g_b.s_arm_aim_gap, f2l(-60.0f));
    dg_bridge_move_now(&cmd);
    move_tick(1);
    if (padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] > 128 - DG_MOVE_SUBJECT_MARGIN)
        bad++;
    InterlockedExchange(&g_b.s_arm_aim_gap, f2l(60.0f));

    /* Under the threshold: silence. */
    padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] = 128;
    *(short *)(padmem.b + 4 + DG_PAD_ANALOG_OFFSET) = 0;
    InterlockedExchange(&g_b.s_arm_aim_gap, f2l(20.0f));
    dg_bridge_move_now(&cmd);
    n = g_b.c_turn_writes;
    fw = g_b.c_follow_under;
    move_tick(1);
    if (g_b.c_turn_writes != n) bad++;
    if (g_b.c_follow_under != fw + 1) bad++;
    if (padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] != 128) bad++;

    {
        DG_MOVE_IN fi;
        DG_MOVE_OUT fo;
        memset(&fi, 0, sizeof fi);
        fi.have_sample = 1;
        fi.input_ok = 1;
        fi.deadzone = 0.15;
        fi.turn_on = 1;
        fi.turn_gain = 1.0;
        fi.right_dx = 128;      /* a centred physical stick: no yield */
        fi.turn_x = 0.1501;
        dg_move_step(&fi, &fo);
        if (!fo.turn_write || fo.rdx <= 176) bad++;
        fi.turn_x = -0.1501;
        dg_move_step(&fi, &fo);
        if (!fo.turn_write || fo.rdx >= 80) bad++;
    }

    /* The trigger holds the weapon up: the follow stands down, however
       loud the gap - vanilla MGS2 roots the body in first-person aim,
       so a write there cannot turn it and only drags the game's aim off
       the player's turn (737 writes, 0.3 degrees of drift, measured
       2026-08-23). A LOUD gap stays silent, the stand-down is counted,
       and the byte never leaves centre. */
    InterlockedExchange(&g_b.s_arm_aim_gap, f2l(60.0f));
    InterlockedExchange(&g_b.s_fire_state, DG_FIRE_HOLD);
    dg_bridge_move_now(&cmd);
    n = g_b.c_turn_writes;
    fw = g_b.c_follow_aim_hold;
    move_tick(1);
    if (g_b.c_turn_writes != n) bad++;
    if (g_b.c_follow_aim_hold != fw + 1) bad++;
    if (padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] != 128) bad++;
    /* The actuator probe bypasses the stand-down on purpose: same loud
       gap, same held trigger, but with vr_turn_probe on the follow DOES
       write - the diagnostic run needs writes during aim - and the
       sample ring holds the write it just measured. */
    InterlockedExchange(&g_b.turn_probe, 1);
    dg_bridge_move_now(&cmd);
    n = g_b.c_turn_writes;
    fw = g_b.turn_probe_w;
    move_tick(1);
    if (g_b.c_turn_writes != n + 1) bad++;
    if (g_b.turn_probe_w != fw + 1) bad++;
    {
        DG_TURN_PROBE_SAMPLE ps;
        int got = 0;
        while (dg_bridge_turn_probe_take(&ps)) got = 1;
        if (!got || ps.fire_state != DG_FIRE_HOLD || ps.wrote != 1) bad++;
        /* The sample carries the loop term the follow steered on: the
           60-degree gap set above, bit-exact through the f2l store. */
        if (!got || ps.gap_deg != 60.0f) bad++;
    }
    /* DENSE: with the weapon up, a tick that writes nothing still leaves a
       sample (wrote 0, byte 128) - the per-tick series the 2026-09-01
       par. 9 measurement needs - and a tick with the weapon down leaves
       none, so a dense session logs only aim time. */
    InterlockedExchange(&g_b.turn_probe, 2);
    InterlockedExchange(&g_b.s_arm_aim_gap, f2l(0.0f));
    padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] = 128;
    *(short *)(padmem.b + 4 + DG_PAD_ANALOG_OFFSET) = 0;
    dg_bridge_move_now(&cmd);
    n = g_b.c_turn_writes;
    fw = g_b.turn_probe_w;
    move_tick(1);
    if (g_b.c_turn_writes != n) bad++;
    if (g_b.turn_probe_w != fw + 1) bad++;
    {
        DG_TURN_PROBE_SAMPLE ps;
        int got = 0;
        while (dg_bridge_turn_probe_take(&ps)) got = 1;
        if (!got || ps.wrote != 0 || ps.byte != 128 ||
            ps.fire_state != DG_FIRE_HOLD) bad++;
    }
    InterlockedExchange(&g_b.s_fire_state, DG_FIRE_IDLE);
    dg_bridge_move_now(&cmd);
    fw = g_b.turn_probe_w;
    move_tick(1);
    if (g_b.turn_probe_w != fw) bad++;
    /* `all` speaks with the weapon down too. */
    InterlockedExchange(&g_b.turn_probe, 3);
    dg_bridge_move_now(&cmd);
    fw = g_b.turn_probe_w;
    move_tick(1);
    if (g_b.turn_probe_w != fw + 1) bad++;
    {
        DG_TURN_PROBE_SAMPLE ps;
        while (dg_bridge_turn_probe_take(&ps)) {}
    }
    InterlockedExchange(&g_b.turn_probe, 2);
    InterlockedExchange(&g_b.s_fire_state, DG_FIRE_HOLD);
    InterlockedExchange(&g_b.s_arm_aim_gap, f2l(60.0f));
    InterlockedExchange(&g_b.turn_probe, 0);

    InterlockedExchange(&g_b.follow_aim, 1);
    InterlockedExchange(&g_b.follow_src, 1);
    padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] = 128;
    *(short *)(padmem.b + 4 + DG_PAD_ANALOG_OFFSET) = 0;
    dg_bridge_move_now(&cmd);
    n = g_b.c_turn_writes;
    fw = g_b.c_turn_follow_writes;
    {
        LONG ah = g_b.c_follow_aim_hold;
        move_tick(1);
        if (g_b.c_turn_writes != n + 1) bad++;
        if (g_b.c_turn_follow_writes != fw + 1) bad++;
        if (g_b.c_follow_aim_hold != ah) bad++;
    }
    InterlockedExchange(&g_b.follow_aim, 0);
    InterlockedExchange(&g_b.follow_src, 2);
    padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] = 128;
    *(short *)(padmem.b + 4 + DG_PAD_ANALOG_OFFSET) = 0;
    dg_bridge_move_now(&cmd);
    n = g_b.c_turn_writes;
    fw = g_b.c_follow_aim_hold;
    move_tick(1);
    if (g_b.c_turn_writes != n) bad++;
    if (g_b.c_follow_aim_hold != fw + 1) bad++;
    InterlockedExchange(&g_b.follow_src, 0);
    padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] = 128;
    *(short *)(padmem.b + 4 + DG_PAD_ANALOG_OFFSET) = 0;
    /* And the moment the weapon drops, the same gap speaks again. */
    InterlockedExchange(&g_b.s_fire_state, DG_FIRE_IDLE);
    dg_bridge_move_now(&cmd);
    n = g_b.c_turn_writes;
    fw = g_b.c_turn_follow_writes;
    move_tick(1);
    if (g_b.c_turn_writes != n + 1) bad++;
    if (g_b.c_turn_follow_writes != fw + 1) bad++;
    InterlockedExchange(&g_b.s_arm_aim_gap, f2l(20.0f));
    padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] = 128;
    *(short *)(padmem.b + 4 + DG_PAD_ANALOG_OFFSET) = 0;

    /* A stale gap: the hand stream died, the body stands still. */
    InterlockedExchange(&g_b.s_arm_aim_gap, f2l(60.0f));
    InterlockedExchange(&g_b.arm_aim_gap_tick, 400);
    dg_bridge_move_now(&cmd);
    n = g_b.c_turn_writes;
    fw = g_b.c_follow_stale;
    move_tick(1);
    if (g_b.c_turn_writes != n) bad++;
    if (g_b.c_follow_stale != fw + 1) bad++;

    /* An uncommitted sign: silence, however loud the gap. */
    InterlockedExchange(&g_b.arm_aim_gap_tick, 500);
    InterlockedExchange(&g_b.turn_dir_acc, 5);
    dg_bridge_move_now(&cmd);
    n = g_b.c_turn_writes;
    fw = g_b.c_follow_no_sign;
    move_tick(1);
    if (g_b.c_turn_writes != n) bad++;
    if (g_b.c_follow_no_sign != fw + 1) bad++;
    InterlockedExchange(&g_b.turn_dir_acc, 30);

    /* A marker-declared sign arms the follow without a single lesson: the
       2026-08-23 session sat mute on a -31 degree gap because the votes
       were still 0. Seeded -1, the same fresh +60 gap turns immediately,
       on the negative byte side; a non-declaration (0) leaves the
       accumulator exactly as learning left it. */
    InterlockedExchange(&g_b.turn_dir_acc, 0);
    turn_dir_seed(0);
    if (InterlockedCompareExchange(&g_b.turn_dir_acc, 0, 0) != 0) bad++;
    turn_dir_seed(-1);
    if (InterlockedCompareExchange(&g_b.turn_dir_acc, 0, 0) != -30) bad++;
    padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] = 128;
    *(short *)(padmem.b + 4 + DG_PAD_ANALOG_OFFSET) = 0;
    dg_bridge_move_now(&cmd);
    fw = g_b.c_turn_follow_writes;
    move_tick(1);
    if (g_b.c_turn_follow_writes != fw + 1) bad++;
    if (padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] > 128 - DG_MOVE_SUBJECT_MARGIN)
        bad++;
    InterlockedExchange(&g_b.turn_dir_acc, 30);

    /* The player's stick always wins, and the write is theirs, not the
       follow's. */
    padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] = 128;
    *(short *)(padmem.b + 4 + DG_PAD_ANALOG_OFFSET) = 0;
    cmd.turn_x = -1.0;
    dg_bridge_move_now(&cmd);
    n = g_b.c_turn_writes;
    fw = g_b.c_turn_follow_writes;
    move_tick(1);
    if (g_b.c_turn_writes != n + 1) bad++;
    if (g_b.c_turn_follow_writes != fw) bad++;
    if (padmem.b[4 + DG_PAD_RIGHT_DX_OFFSET] > 128 - DG_MOVE_SUBJECT_MARGIN)
        bad++;

    InterlockedExchange(&g_b.turn_follow_thresh_mdeg, 0);
    InterlockedExchange(&g_b.turn_follow_full_mdeg, 0);
    InterlockedExchange(&g_b.turn_dir_acc, 0);
    InterlockedExchange(&g_b.turn_last_write_dir, 0);
    InterlockedExchange(&g_b.turn_last_write_tick, 0);
    InterlockedExchange(&g_b.s_arm_aim_gap, 0);
    InterlockedExchange(&g_b.arm_aim_gap_tick, 0);
    g_b.turn_vote_have_prev = 0;
    InterlockedExchange(&g_b.move_valid, 0);
    InterlockedExchange(&g_b.walk_mode, saved_mode);
    InterlockedExchange(&g_b.turn_mode, saved_turn);
    InterlockedExchange(&g_b.armed, 0);
    InterlockedExchange(&g_b.c_ticks, 0);
    g_b.a.player_pad = saved_pad;
    g_b.a.pl_subject_move = saved_subj;
    g_b.fps.state = saved_state;

    printf("  %-6s body follow: the sign commits from drift responses "
           "(snaps and noise voteless), the consumer steers on the aim "
           "gap it steers on, a committed sign turns the right byte side, "
           "threshold, staleness, an uncommitted sign and the player's "
           "own stick each silence it AND say so on their own counter, "
           "and a marker-declared sign arms the follow with zero "
           "lessons\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

/* The trigger's crossing, end to end, on the seams that actually carry it.
 *
 * dg_fire_test.c already proves the contract itself. What is left is the part
 * that only exists here: a camera seam publishing while a game tick consumes,
 * a staleness bound, the game-side gate, and - the claim this whole step is
 * built around - that DRY writes absolutely nothing. The pad buffer below is
 * checked byte for byte after every single tick, because "it only evaluates"
 * is exactly the kind of promise that is worth nothing unless it is measured.
 */
static int t_the_trigger_crosses_the_seams(void)
{
    union { ULONGLONG align; unsigned char b[0x300]; } actor;
    union { ULONGLONG align; unsigned char b[0xD20]; } player;
    union { ULONGLONG align; unsigned char b[0x40]; } wp_set;
    /* enable, then the 40 bytes the merge point copies, with room to spare */
    union { ULONGLONG align; unsigned char b[0x40]; } padmem, padwas;
    ULONGLONG arm, arm_slot, pad;
    ULONGLONG saved_arm = g_b.a.gm_player_arm_body;
    ULONGLONG saved_pad = g_b.a.player_pad;
    ULONGLONG saved_mask = g_b.a.pad_weapon;
    ULONGLONG saved_player_status = g_b.a.gm_player_status;
    ULONGLONG native_status = 0x800ULL;
    LONG saved_state = g_b.fps.state;
    LONG mask = 0x0008;                 /* a plausible PL_PAD_WEAPON (PAD_R1) */
    DG_BRIDGE_FIRE cmd;
    LONG n;
    int bad = 0, i;

    memset(&actor, 0, sizeof actor);
    memset(&player, 0, sizeof player);
    memset(&wp_set, 0, sizeof wp_set);
    memset(&padmem, 0, sizeof padmem);

    arm = (ULONGLONG)(ULONG_PTR)(actor.b + 0x60);
    arm_slot = arm;
    *(ULONGLONG *)(actor.b + 0x228) = (ULONGLONG)(ULONG_PTR)(player.b + 0xCF4);
    *(ULONGLONG *)(player.b + 0xBA8) = arm;
    *(LONG *)(player.b + 0xBB0) = 6;
    *(LONG *)(player.b + 0xB90) = 1;                    /* a real weapon */
    *(ULONGLONG *)(player.b + 0xBA0) = (ULONGLONG)(ULONG_PTR)wp_set.b;
    *(LONG *)(wp_set.b + 0x10) = (LONG)DG_FIRE_WP_PRESSURE;

    pad = (ULONGLONG)(ULONG_PTR)(padmem.b + 4);         /* enable sits below */
    *(LONG *)(padmem.b + 0) = 1;                        /* PlayerPad.enable */

    g_b.a.gm_player_arm_body = (ULONGLONG)(ULONG_PTR)&arm_slot;
    g_b.a.gm_player_status = (ULONGLONG)(ULONG_PTR)&native_status;
    *(LONG *)(player.b + 0xC84) = 8;
    g_b.a.player_pad = pad;
    g_b.a.pad_weapon = (ULONGLONG)(ULONG_PTR)&mask;
    g_b.fps.state = DG_FPS_ACTIVE;
    InterlockedExchange(&g_b.s_late_unsafe, 0);
    InterlockedExchange(&g_b.armed, 1);
    InterlockedExchange(&g_b.fire_mode, DG_FIRE_MODE_DRY);
    InterlockedExchange(&g_b.c_ticks, 500);
    dg_fire_reset(&g_b.fire);
    InterlockedExchange(&g_b.fire_valid, 0);
    padwas = padmem;

    memset(&cmd, 0, sizeof cmd);
    cmd.click = 0.55;
    cmd.value = 0.60;
    cmd.valid = 1;
    cmd.stream_id = 3;

    /* A pull, offered by five camera seams inside one tick the way the real
       seam does, then consumed once. */
    cmd.press_seq = 1;
    n = g_b.c_fire_drawn;
    for (i = 0; i < 5; i++) dg_bridge_fire_now(&cmd);
    fire_tick(1);
    if (g_b.c_fire_drawn != n + 1) bad++;
    if (memcmp(&padmem, &padwas, sizeof padmem) != 0) bad++;

    /* held: no more draws, no shot */
    {
        LONG drew = g_b.c_fire_drawn;
        n = g_b.c_fire_released;
        for (i = 0; i < 10; i++) {
            dg_bridge_fire_now(&cmd);
            fire_tick(1);
            if (memcmp(&padmem, &padwas, sizeof padmem) != 0) bad++;
        }
        if (g_b.c_fire_drawn != drew) bad++;
        if (g_b.c_fire_released != n) bad++;
        if (g_b.s_fire_pressure < DG_FIRE_TH) bad++;
    }

    /* Full pull: synthetic native release, physical trigger stays held. */
    cmd.value = 0.9;
    dg_bridge_fire_now(&cmd);
    /* Native draw, reload/wall and absent HOLD each defer the firing edge. */
    *(LONG *)(player.b + 0xC84) = 7;
    fire_tick(1);
    if (g_b.c_fire_released != n) bad++;
    *(LONG *)(player.b + 0xC84) = 8;
    *(LONG *)(player.b + 0xCA8) = 3;
    fire_tick(1);
    if (g_b.c_fire_released != n) bad++;
    *(LONG *)(player.b + 0xCA8) = 0;
    native_status = 0;
    fire_tick(1);
    if (g_b.c_fire_released != n) bad++;
    native_status = 0x800ULL;
    fire_tick(1);
    if (g_b.c_fire_released != n + 1) bad++;
    if (memcmp(&padmem, &padwas, sizeof padmem) != 0) bad++;

    /* An aim that outlives the publications. The camera seam and the game tick
       are not locked to each other - the dry run published 0.8 samples per tick
       - so an aim has to survive a few quiet ticks without the weapon dropping,
       and the age has to reach the contract instead of being judged here. */
    {
        LONG n_coast = g_b.c_fire_coasting;
        cmd.value = 0.60;
        cmd.press_seq = 2;
        cmd.release_seq = 1;
        dg_bridge_fire_now(&cmd);
        fire_tick(1);                           /* draws */
        if (g_b.fire.state != DG_FIRE_HOLD) bad++;
        for (i = 1; i <= 6; i++) {              /* nobody publishes for six */
            InterlockedExchange(&g_b.c_ticks, 500 + i);
            fire_tick(1);
        }
        if (g_b.c_fire_coasting <= n_coast) bad++;
        if (g_b.fire.state != DG_FIRE_HOLD) bad++;
        if (memcmp(&padmem, &padwas, sizeof padmem) != 0) bad++;
        /* and the pull still ends in exactly one release when it comes back */
        n = g_b.c_fire_released;
        cmd.value = 0.9;
        InterlockedExchange(&g_b.c_ticks, 507);
        dg_bridge_fire_now(&cmd);
        fire_tick(1);
        if (g_b.c_fire_released != n + 1) bad++;
    }
    InterlockedExchange(&g_b.c_ticks, 500);

    /* a command nobody refreshed goes stale rather than repeating forever */
    cmd.press_seq = 3;
    dg_bridge_fire_now(&cmd);
    InterlockedExchange(&g_b.c_ticks, 510);
    n = g_b.c_fire_stale;
    fire_tick(1);
    if (g_b.c_fire_stale != n + 1) bad++;
    InterlockedExchange(&g_b.c_ticks, 510);

    /* the camera seam standing the trigger down entirely */
    n = g_b.c_fire_drawn;
    dg_bridge_fire_now(NULL);
    cmd.press_seq = 4;
    fire_tick(1);
    if (g_b.c_fire_drawn != n) bad++;

    /* the player's own weapon button, seen through the pad the game filled */
    dg_fire_reset(&g_b.fire);
    *(LONG *)(padmem.b + 4 + DG_PAD_STATUS_OFFSET) = mask;
    padwas = padmem;
    cmd.press_seq = 5;
    dg_bridge_fire_now(&cmd);
    n = g_b.c_fire_blocked_phys;
    {
        LONG drew = g_b.c_fire_drawn;
        fire_tick(1);
        if (g_b.c_fire_blocked_phys != n + 1) bad++;
        if (g_b.c_fire_drawn != drew) bad++;
    }
    *(LONG *)(padmem.b + 4 + DG_PAD_STATUS_OFFSET) = 0;
    padwas = padmem;

    /* the Bluepoint pad layer switched off: the player reads a different pad
       record, so our contract would be written where nobody looks */
    dg_fire_reset(&g_b.fire);
    *(LONG *)(padmem.b + 0) = 0;
    padwas = padmem;
    cmd.press_seq = 6;
    dg_bridge_fire_now(&cmd);
    n = g_b.c_fire_blocked_gate;
    {
        LONG drew = g_b.c_fire_drawn;
        fire_tick(1);
        if (g_b.c_fire_blocked_gate != n + 1) bad++;
        if (g_b.c_fire_drawn != drew) bad++;
    }
    *(LONG *)(padmem.b + 0) = 1;
    padwas = padmem;

    /* out of first person, and the camera seam's own unsafe reading */
    dg_fire_reset(&g_b.fire);
    g_b.fps.state = DG_FPS_OFF;
    cmd.press_seq = 7;
    dg_bridge_fire_now(&cmd);
    n = g_b.c_fire_blocked_gate;
    fire_tick(1);
    if (g_b.c_fire_blocked_gate != n + 1) bad++;
    g_b.fps.state = DG_FPS_ACTIVE;
    InterlockedExchange(&g_b.s_late_unsafe, 1);
    dg_bridge_fire_now(&cmd);
    n = g_b.c_fire_blocked_gate;
    fire_tick(1);
    if (g_b.c_fire_blocked_gate != n + 1) bad++;
    InterlockedExchange(&g_b.s_late_unsafe, 0);

    /* the weapon type reaches the contract from the live wp_set */
    dg_fire_reset(&g_b.fire);
    *(LONG *)(wp_set.b + 0x10) = (LONG)DG_FIRE_WP_CONSECUTIVE;
    cmd.press_seq = 8;
    cmd.release_seq = 7;
    dg_bridge_fire_now(&cmd);
    fire_tick(1);
    if ((unsigned int)g_b.s_fire_wtype != DG_FIRE_WP_CONSECUTIVE) bad++;

    /* a stream change forgets a pull that belonged to the old session */
    cmd.stream_id = 4;
    cmd.press_seq = 8;              /* same number, different stream */
    dg_bridge_fire_now(&cmd);
    n = g_b.c_fire_drawn;
    fire_tick(1);
    if (g_b.c_fire_drawn != n + 1) bad++;

    /* off means off: not one counter moves and the machine forgets */
    InterlockedExchange(&g_b.fire_mode, DG_FIRE_MODE_OFF);
    cmd.stream_id = 5;
    cmd.press_seq = 9;
    cmd.release_seq = 8;
    dg_bridge_fire_now(&cmd);
    n = g_b.c_fire_drawn;
    for (i = 0; i < 5; i++) fire_tick(1);
    if (g_b.c_fire_drawn != n) bad++;
    if (g_b.fire.state != DG_FIRE_IDLE) bad++;

    if (memcmp(&padmem, &padwas, sizeof padmem) != 0) bad++;

    g_b.a.gm_player_arm_body = saved_arm;
    g_b.a.gm_player_status = saved_player_status;
    g_b.a.player_pad = saved_pad;
    g_b.a.pad_weapon = saved_mask;
    g_b.fps.state = saved_state;
    InterlockedExchange(&g_b.armed, 0);
    InterlockedExchange(&g_b.fire_mode, DG_FIRE_MODE_OFF);
    InterlockedExchange(&g_b.fire_valid, 0);
    InterlockedExchange(&g_b.c_ticks, 0);
    dg_fire_reset(&g_b.fire);

    printf("  %-6s trigger crosses the seams: five camera-seam offers are one "
           "draw, six ticks with nobody publishing coast the aim instead of "
           "dropping it, the release is one release, and a stale command, a "
           "stood-down seam, the player's own button, PlayerPad disabled, "
           "leaving first person and an unsafe camera reading each refuse - "
           "with the pad buffer byte-identical after every tick\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

/* Action() refills the pad from GV_PadData before our seam runs, so a tick
   always starts from the player's own input and never from what we wrote last
   tick. Modelled here, because without it the checks below would be reading
   our own writes back and calling them the player's. */
static void pad_refill(unsigned char *base, LONG own_status, LONG own_press,
                       LONG own_release, int slot, int own_pressure)
{
    *(LONG *)(base + DG_PAD_STATUS_OFFSET) = own_status;
    *(LONG *)(base + DG_PAD_PRESS_OFFSET) = own_press;
    *(LONG *)(base + DG_PAD_RELEASE_OFFSET) = own_release;
    memset(base + DG_PAD_PRESSURE_OFFSET, 0, DG_PAD_PRESSURE_COUNT);
    if (slot >= 0 && slot < DG_PAD_PRESSURE_COUNT)
        base[DG_PAD_PRESSURE_OFFSET + slot] = (unsigned char)own_pressure;
}

/* The invariant, checked over the whole buffer rather than field by field:
   outside the four contract fields nothing changed at all, and inside them
   nothing was taken away - no bit cleared, no pressure byte lowered. A write
   that got an offset wrong fails this even if every field check passes. */
static int pad_only_added(const unsigned char *now, const unsigned char *was,
                          size_t len)
{
    size_t i;
    for (i = 0; i < len; i++) {
        /* the pad sits at base+4: status/press/release span [8,20), the
           twelve pressure bytes [28,40) */
        if (i >= 8 && i < 20) {
            if ((now[i] & was[i]) != was[i]) return 0;
        } else if (i >= 28 && i < 40) {
            if (now[i] < was[i]) return 0;
        } else if (now[i] != was[i]) {
            return 0;
        }
    }
    return 1;
}

/* ON, and the one rule it lives by: we only ever ADD to the pad. Every check
   here is a subtraction the write must not make. */
static int t_the_trigger_only_ever_adds(void)
{
    union { ULONGLONG align; unsigned char b[0x300]; } actor;
    union { ULONGLONG align; unsigned char b[0xD20]; } player;
    union { ULONGLONG align; unsigned char b[0x40]; } wp_set;
    union { ULONGLONG align; unsigned char b[0x40]; } padmem, padwas;
    ULONGLONG arm, arm_slot, pad;
    ULONGLONG saved_arm = g_b.a.gm_player_arm_body;
    ULONGLONG saved_pad = g_b.a.player_pad;
    ULONGLONG saved_mask = g_b.a.pad_weapon;
    ULONGLONG saved_idx = g_b.a.pad_press_weapon;
    ULONGLONG saved_player_status = g_b.a.gm_player_status;
    ULONGLONG native_status = 0x800ULL;
    LONG saved_state = g_b.fps.state;
    LONG mask = 0x0008;                 /* a plausible PL_PAD_WEAPON (PAD_R1) */
    LONG pidx = 4;                      /* a plausible PL_PAD_PRESS_WEAPON */
    LONG mine = 0x1234;                 /* bits that are the player's, not ours */
    DG_BRIDGE_FIRE cmd;
    unsigned char *base;
    int deep = DG_FIRE_TH;
    LONG n, n2;
    int bad = 0;

    memset(&actor, 0, sizeof actor);
    memset(&player, 0, sizeof player);
    memset(&wp_set, 0, sizeof wp_set);
    memset(&padmem, 0, sizeof padmem);

    arm = (ULONGLONG)(ULONG_PTR)(actor.b + 0x60);
    arm_slot = arm;
    *(ULONGLONG *)(actor.b + 0x228) = (ULONGLONG)(ULONG_PTR)(player.b + 0xCF4);
    *(ULONGLONG *)(player.b + 0xBA8) = arm;
    *(LONG *)(player.b + 0xBB0) = 6;
    *(LONG *)(player.b + 0xB90) = 1;
    *(ULONGLONG *)(player.b + 0xBA0) = (ULONGLONG)(ULONG_PTR)wp_set.b;
    *(LONG *)(wp_set.b + 0x10) = (LONG)DG_FIRE_WP_PRESSURE;

    base = padmem.b + 4;
    pad = (ULONGLONG)(ULONG_PTR)base;
    *(LONG *)(padmem.b + 0) = 1;                        /* PlayerPad.enable */

    g_b.a.gm_player_arm_body = (ULONGLONG)(ULONG_PTR)&arm_slot;
    g_b.a.gm_player_status = (ULONGLONG)(ULONG_PTR)&native_status;
    *(LONG *)(player.b + 0xC84) = 8;
    g_b.a.player_pad = pad;
    g_b.a.pad_weapon = (ULONGLONG)(ULONG_PTR)&mask;
    g_b.a.pad_press_weapon = (ULONGLONG)(ULONG_PTR)&pidx;
    g_b.fps.state = DG_FPS_ACTIVE;
    InterlockedExchange(&g_b.s_late_unsafe, 0);
    InterlockedExchange(&g_b.armed, 1);
    InterlockedExchange(&g_b.fire_mode, DG_FIRE_MODE_ON);
    InterlockedExchange(&g_b.c_ticks, 500);
    dg_fire_reset(&g_b.fire);
    InterlockedExchange(&g_b.fire_valid, 0);

    memset(&cmd, 0, sizeof cmd);
    cmd.click = 0.55;
    cmd.value = 0.60;
    cmd.valid = 1;
    cmd.stream_id = 11;

    /* 0. The mode survives the journey from the marker to the writer. This
          is not plumbing pedantry: a build once parsed `on`, said so in its
          session banner, and then had a second gate downgrade it to OFF - so
          the run wrote nothing and the log did not say why. The banner
          reports intent; only this reports what the writer will honour. */
    {
        DG_BRIDGE_CONFIG cfg;
        LONG before = g_b.fire_mode;
        memset(&cfg, 0, sizeof cfg);
        cfg.fire_mode = DG_FIRE_MODE_ON;
        dg_bridge_configure(&cfg);
        if (g_b.fire_mode != DG_FIRE_MODE_ON) bad++;
        cfg.fire_mode = DG_FIRE_MODE_DRY;
        dg_bridge_configure(&cfg);
        if (g_b.fire_mode != DG_FIRE_MODE_DRY) bad++;
        cfg.fire_mode = 99;                     /* and a typo is never a write */
        dg_bridge_configure(&cfg);
        if (g_b.fire_mode != DG_FIRE_MODE_OFF) bad++;
        InterlockedExchange(&g_b.fire_mode, before);
    }
    g_b.fps.state = DG_FPS_ACTIVE;
    InterlockedExchange(&g_b.armed, 1);
    InterlockedExchange(&g_b.fire_mode, DG_FIRE_MODE_ON);

    /* None: partial pull, full detent and release must leave the entire pad
       untouched even in ON mode, including an unequip during an active aim. */
    {
        int j;
        const double values[] = { 0.60, 0.95, 0.0 };
        *(LONG *)(player.b + 0xB90) = 0;
        for (j = 0; j < 3; ++j) {
            pad_refill(base, 0, 0, 0, pidx, 0);
            padwas = padmem;
            cmd.value = values[j];
            cmd.press_seq = 1;
            cmd.release_seq = j == 2 ? 1 : 0;
            dg_bridge_fire_now(&cmd);
            fire_tick(1);
            if (memcmp(&padmem, &padwas, sizeof padmem) != 0) bad++;
            if (g_b.fire.state != DG_FIRE_IDLE) bad++;
        }
        g_b.fire.state = DG_FIRE_HOLD;
        cmd.value = 0.95;
        dg_bridge_fire_now(&cmd);
        fire_tick(1);
        if (memcmp(&padmem, &padwas, sizeof padmem) != 0) bad++;
        if (g_b.fire.state != DG_FIRE_IDLE) bad++;
        *(LONG *)(player.b + 0xB90) = 1;
        cmd.value = 0.60;
        cmd.release_seq = 0;
        dg_fire_reset(&g_b.fire);
    }

    /* 1. The draw. The weapon bit into press and status, the analogue travel
          into pressure[idx], and nothing else in the buffer moved. */
    pad_refill(base, 0, 0, 0, pidx, 0);
    padwas = padmem;
    cmd.press_seq = 1;
    dg_bridge_fire_now(&cmd);
    fire_tick(1);
    if ((*(LONG *)(base + DG_PAD_PRESS_OFFSET) & mask) != mask) bad++;
    if ((*(LONG *)(base + DG_PAD_STATUS_OFFSET) & mask) != mask) bad++;
    if (*(LONG *)(base + DG_PAD_RELEASE_OFFSET) != 0) bad++;
    if (base[DG_PAD_PRESSURE_OFFSET + pidx] != (unsigned char)deep) bad++;
    /* and never into the cancel window, whatever the trigger travel was */
    if (base[DG_PAD_PRESSURE_OFFSET + pidx] < DG_FIRE_TH) bad++;
    if (!pad_only_added(padmem.b, padwas.b, sizeof padmem)) bad++;

    /* 2. Holding, with the player pressing other buttons at the same time.
          Not one of their bits may go missing. */
    pad_refill(base, mine, mine, mine, pidx, 0);
    padwas = padmem;
    dg_bridge_fire_now(&cmd);
    fire_tick(1);
    if ((*(LONG *)(base + DG_PAD_STATUS_OFFSET) & mine) != mine) bad++;
    if ((*(LONG *)(base + DG_PAD_PRESS_OFFSET) & mine) != mine) bad++;
    if ((*(LONG *)(base + DG_PAD_RELEASE_OFFSET) & mine) != mine) bad++;
    if ((*(LONG *)(base + DG_PAD_STATUS_OFFSET) & mask) != mask) bad++;
    if (!pad_only_added(padmem.b, padwas.b, sizeof padmem)) bad++;

    /* 3. Pressure only ever goes up. The game's own byte is already deeper
          than ours, so ours is dropped and said so in the log. */
    pad_refill(base, 0, 0, 0, pidx, 250);
    padwas = padmem;
    n = g_b.c_fire_pressure_kept;
    n2 = g_b.c_fire_wrote_pressure;
    dg_bridge_fire_now(&cmd);
    fire_tick(1);
    if (base[DG_PAD_PRESSURE_OFFSET + pidx] != 250) bad++;
    if (g_b.c_fire_pressure_kept != n + 1) bad++;
    if (g_b.c_fire_wrote_pressure != n2) bad++;
    if (!pad_only_added(padmem.b, padwas.b, sizeof padmem)) bad++;

    /* 4. Crossing the firing detent. Status is withheld, not cleared - it is
          the shot. The release bit is what tells the Bluepoint layer. */
    pad_refill(base, 0, 0, 0, pidx, 0);
    padwas = padmem;
    cmd.value = 0.9;
    n = g_b.c_fire_released;
    dg_bridge_fire_now(&cmd);
    fire_tick(1);
    if (g_b.c_fire_released != n + 1) bad++;
    if (*(LONG *)(base + DG_PAD_STATUS_OFFSET) != 0) bad++;
    if ((*(LONG *)(base + DG_PAD_RELEASE_OFFSET) & mask) != mask) bad++;
    if (base[DG_PAD_PRESSURE_OFFSET + pidx] != 0) bad++;
    if (!pad_only_added(padmem.b, padwas.b, sizeof padmem)) bad++;

    /* 5. The player's own weapon button. Not one byte, not even the fields we
          would have OR-ed a bit we can see is already there: their input owns
          the stance and our release would end an aim we did not start. */
    dg_fire_reset(&g_b.fire);
    pad_refill(base, mask, 0, 0, pidx, 0);
    padwas = padmem;
    cmd.press_seq = 2;
    cmd.release_seq = 1;
    n = g_b.c_fire_yielded;
    dg_bridge_fire_now(&cmd);
    fire_tick(1);
    if (g_b.c_fire_yielded != n + 1) bad++;
    if (memcmp(&padmem, &padwas, sizeof padmem) != 0) bad++;

    /* 6. An index that is not an index into pressure[12]. The bits still go
          in - they are what fires a pistol - and the array is not touched. */
    dg_fire_reset(&g_b.fire);
    pidx = 99;
    pad_refill(base, 0, 0, 0, -1, 0);
    padwas = padmem;
    cmd.press_seq = 3;
    n = g_b.c_fire_no_index;
    dg_bridge_fire_now(&cmd);
    fire_tick(1);
    if (g_b.c_fire_no_index != n + 1) bad++;
    if ((*(LONG *)(base + DG_PAD_STATUS_OFFSET) & mask) != mask) bad++;
    if (memcmp(base + DG_PAD_PRESSURE_OFFSET, padwas.b + 4 +
               DG_PAD_PRESSURE_OFFSET, DG_PAD_PRESSURE_COUNT) != 0) bad++;
    if (!pad_only_added(padmem.b, padwas.b, sizeof padmem)) bad++;
    pidx = 4;

    /* 7. And the same tick in DRY, which is the whole claim about DRY: ON and
          DRY differ by the write and by nothing else. */
    dg_fire_reset(&g_b.fire);
    InterlockedExchange(&g_b.fire_mode, DG_FIRE_MODE_DRY);
    pad_refill(base, 0, 0, 0, pidx, 0);
    padwas = padmem;
    cmd.press_seq = 4;
    n = g_b.c_fire_drawn;
    dg_bridge_fire_now(&cmd);
    fire_tick(1);
    if (g_b.c_fire_drawn != n + 1) bad++;       /* it still decided to draw */
    if (memcmp(&padmem, &padwas, sizeof padmem) != 0) bad++;   /* and wrote nothing */

    g_b.a.gm_player_arm_body = saved_arm;
    g_b.a.gm_player_status = saved_player_status;
    g_b.a.player_pad = saved_pad;
    g_b.a.pad_weapon = saved_mask;
    g_b.a.pad_press_weapon = saved_idx;
    g_b.fps.state = saved_state;
    InterlockedExchange(&g_b.armed, 0);
    InterlockedExchange(&g_b.fire_mode, DG_FIRE_MODE_OFF);
    InterlockedExchange(&g_b.fire_valid, 0);
    InterlockedExchange(&g_b.c_ticks, 0);
    dg_fire_reset(&g_b.fire);

    printf("  %-6s trigger only ever adds: a draw sets the weapon bit in press "
           "and status and the travel in pressure[idx], a hold leaves every "
           "other button the player is pressing intact, a deeper byte of "
           "theirs survives ours, letting go withholds status rather than "
           "clearing it, their own weapon button stands us down to not one "
           "byte, a bad pressure index still fires a pistol, the same tick "
           "in DRY writes nothing, and the mode the marker asked for is the "
           "mode the writer honours\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

/* The kick, where it meets the bridge. dg_recoil already answers for the shape
   of the envelope; what can only be checked here is that a shot reaches it,
   that it advances exactly once per game tick, and that it comes home even
   when everything else has been taken away. */
static int t_the_kick_is_ours_and_comes_home(void)
{
    union { ULONGLONG align; unsigned char b[0x300]; } actor;
    union { ULONGLONG align; unsigned char b[0xD20]; } player;
    union { ULONGLONG align; unsigned char b[0x40]; } wp_set;
    union { ULONGLONG align; unsigned char b[0x40]; } padmem;
    ULONGLONG arm, arm_slot, pad;
    ULONGLONG saved_arm = g_b.a.gm_player_arm_body;
    ULONGLONG saved_pad = g_b.a.player_pad;
    ULONGLONG saved_mask = g_b.a.pad_weapon;
    ULONGLONG saved_idx = g_b.a.pad_press_weapon;
    ULONGLONG saved_player_status = g_b.a.gm_player_status;
    ULONGLONG native_status = 0x800ULL;
    LONG saved_state = g_b.fps.state;
    LONG mask = 0x0008, pidx = 4;
    DG_BRIDGE_FIRE cmd;
    unsigned char *base;
    DG_RECOIL mirror;
    LONG n, released;
    int bad = 0, i;

    memset(&actor, 0, sizeof actor);
    memset(&player, 0, sizeof player);
    memset(&wp_set, 0, sizeof wp_set);
    memset(&padmem, 0, sizeof padmem);

    arm = (ULONGLONG)(ULONG_PTR)(actor.b + 0x60);
    arm_slot = arm;
    *(ULONGLONG *)(actor.b + 0x228) = (ULONGLONG)(ULONG_PTR)(player.b + 0xCF4);
    *(ULONGLONG *)(player.b + 0xBA8) = arm;
    *(LONG *)(player.b + 0xBB0) = 6;
    *(LONG *)(player.b + 0xB90) = 1;
    *(ULONGLONG *)(player.b + 0xBA0) = (ULONGLONG)(ULONG_PTR)wp_set.b;
    *(LONG *)(wp_set.b + 0x10) = (LONG)DG_FIRE_WP_PRESSURE;

    base = padmem.b + 4;
    pad = (ULONGLONG)(ULONG_PTR)base;
    *(LONG *)(padmem.b + 0) = 1;

    g_b.a.gm_player_arm_body = (ULONGLONG)(ULONG_PTR)&arm_slot;
    g_b.a.gm_player_status = (ULONGLONG)(ULONG_PTR)&native_status;
    *(LONG *)(player.b + 0xC84) = 8;
    g_b.a.player_pad = pad;
    g_b.a.pad_weapon = (ULONGLONG)(ULONG_PTR)&mask;
    g_b.a.pad_press_weapon = (ULONGLONG)(ULONG_PTR)&pidx;
    g_b.fps.state = DG_FPS_ACTIVE;
    InterlockedExchange(&g_b.s_late_unsafe, 0);
    InterlockedExchange(&g_b.armed, 1);
    InterlockedExchange(&g_b.fire_mode, DG_FIRE_MODE_ON);
    InterlockedExchange(&g_b.recoil_climb_mdeg, 6000);
    InterlockedExchange(&g_b.recoil_push_um, 18000);
    InterlockedExchange(&g_b.c_ticks, 500);
    dg_fire_reset(&g_b.fire);
    dg_recoil_reset(&g_b.recoil);
    InterlockedExchange(&g_b.s_recoil_amp, 0);
    InterlockedExchange(&g_b.fire_valid, 0);

    memset(&cmd, 0, sizeof cmd);
    cmd.click = 0.55;
    cmd.value = 0.60;
    cmd.valid = 1;
    cmd.stream_id = 21;

    /* 1. A shot reaches the spring, and exactly one impulse comes out of it.
          Five camera-seam offers of the same pull are still one round. The
          counters are session-wide and an earlier category has already fired
          in ON mode, so every check below is a delta. */
    pad_refill(base, 0, 0, 0, pidx, 0);
    cmd.press_seq = 1;
    n = g_b.c_recoil_kicks;
    for (i = 0; i < 5; i++) dg_bridge_fire_now(&cmd);
    fire_tick(1);                                   /* draws */
    if (g_b.c_recoil_kicks != n) bad++;             /* aiming is not firing */
    if (InterlockedCompareExchange(&g_b.s_recoil_amp, 0, 0) != 0) bad++;

    pad_refill(base, 0, 0, 0, pidx, 0);
    cmd.value = 0.9;
    n = g_b.c_recoil_kicks;
    for (i = 0; i < 5; i++) dg_bridge_fire_now(&cmd);
    fire_tick(1);                                   /* the shot */
    if (g_b.c_recoil_kicks != n + 1) bad++;

    /* 2. And it advances exactly once per game tick. This is the whole reason
          the spring lives on the tick seam: the camera seam runs about five
          times a tick, and an envelope stepped there would run five times too
          fast. Mirrored against dg_recoil itself, tick for tick. */
    dg_recoil_reset(&mirror);
    dg_recoil_fire(&mirror, 1.0);
    dg_recoil_step(&mirror);
    for (i = 0; i < 25; i++) {
        LONG want = (LONG)(dg_recoil_amplitude(&mirror) * 1000.0 + 0.5);
        if (InterlockedCompareExchange(&g_b.s_recoil_amp, 0, 0) != want) {
            bad++;
            break;
        }
        /* several camera-seam offers inside the one tick, exactly as the real
           seam makes them - none of which may move the envelope */
        dg_bridge_fire_now(&cmd);
        dg_bridge_fire_now(&cmd);
        dg_bridge_fire_now(&cmd);
        pad_refill(base, 0, 0, 0, pidx, 0);
        fire_tick(1);
        dg_recoil_step(&mirror);
    }

    /* 3. It comes home even when everything else has been taken away. A hand
          that is mid-recoil when first person ends still has to settle. */
    dg_fire_reset(&g_b.fire);
    dg_recoil_reset(&g_b.recoil);
    dg_recoil_fire(&g_b.recoil, 1.0);
    for (i = 0; i < 3; i++) dg_recoil_step(&g_b.recoil);
    /* ...and the spring has to be genuinely moving before the gate shuts, or
       "it came home" would be true of a spring that never left. */
    if (!(dg_recoil_amplitude(&g_b.recoil) > 0.5)) bad++;
    g_b.fps.state = DG_FPS_OFF;
    for (i = 0; i < DG_RECOIL_SETTLE_TICKS; i++) fire_tick(0);
    if (InterlockedCompareExchange(&g_b.s_recoil_amp, 0, 0) != 0) bad++;
    if (dg_recoil_amplitude(&g_b.recoil) != 0.0) bad++;
    g_b.fps.state = DG_FPS_ACTIVE;

    /* 4. DRY does not kick. No round left the barrel, so a hand that jumped
          would be this build lying about what happened. */
    dg_fire_reset(&g_b.fire);
    dg_recoil_reset(&g_b.recoil);
    InterlockedExchange(&g_b.fire_mode, DG_FIRE_MODE_DRY);
    n = g_b.c_recoil_kicks;
    released = g_b.c_fire_released;
    pad_refill(base, 0, 0, 0, pidx, 0);
    cmd.press_seq = 2;
    cmd.release_seq = 1;
    dg_bridge_fire_now(&cmd);
    fire_tick(1);
    pad_refill(base, 0, 0, 0, pidx, 0);
    cmd.value = 0.9;
    dg_bridge_fire_now(&cmd);
    fire_tick(1);
    if (g_b.c_fire_released != released + 1) bad++;  /* it did release */
    if (g_b.c_recoil_kicks != n) bad++;              /* and still did not kick */
    if (InterlockedCompareExchange(&g_b.s_recoil_amp, 0, 0) != 0) bad++;

    /* 5. OFF forgets a kick in flight rather than leaving it parked. */
    dg_recoil_fire(&g_b.recoil, 1.0);
    dg_recoil_step(&g_b.recoil);
    InterlockedExchange(&g_b.fire_mode, DG_FIRE_MODE_OFF);
    fire_tick(1);
    if (dg_recoil_amplitude(&g_b.recoil) != 0.0) bad++;
    if (InterlockedCompareExchange(&g_b.s_recoil_amp, 0, 0) != 0) bad++;

    /* 6. And the marker cannot ask for a kick that points the wrong way or
          folds the wrist. */
    {
        DG_BRIDGE_CONFIG cfg;
        memset(&cfg, 0, sizeof cfg);
        cfg.recoil_climb_mdeg = -5000;
        cfg.recoil_push_um = -5000;
        dg_bridge_configure(&cfg);
        if (g_b.recoil_climb_mdeg != 0 || g_b.recoil_push_um != 0) bad++;
        cfg.recoil_climb_mdeg = 999999;
        cfg.recoil_push_um = 999999;
        dg_bridge_configure(&cfg);
        if (g_b.recoil_climb_mdeg > 30000) bad++;
        if (g_b.recoil_push_um > 120000) bad++;
        /* and turning it off drops a spring that is still moving */
        dg_recoil_fire(&g_b.recoil, 1.0);
        dg_recoil_step(&g_b.recoil);
        cfg.recoil_climb_mdeg = 0;
        cfg.recoil_push_um = 0;
        dg_bridge_configure(&cfg);
        if (dg_recoil_amplitude(&g_b.recoil) != 0.0) bad++;
    }

    g_b.a.gm_player_arm_body = saved_arm;
    g_b.a.gm_player_status = saved_player_status;
    g_b.a.player_pad = saved_pad;
    g_b.a.pad_weapon = saved_mask;
    g_b.a.pad_press_weapon = saved_idx;
    g_b.fps.state = saved_state;
    InterlockedExchange(&g_b.armed, 0);
    InterlockedExchange(&g_b.fire_mode, DG_FIRE_MODE_OFF);
    InterlockedExchange(&g_b.recoil_climb_mdeg, 0);
    InterlockedExchange(&g_b.recoil_push_um, 0);
    InterlockedExchange(&g_b.fire_valid, 0);
    InterlockedExchange(&g_b.c_ticks, 0);
    dg_fire_reset(&g_b.fire);
    dg_recoil_reset(&g_b.recoil);
    InterlockedExchange(&g_b.s_recoil_amp, 0);

    printf("  %-6s the kick is ours and comes home: a shot is one impulse and "
           "aiming is none, the envelope advances once per game tick however "
           "many camera seams offer the same pull, it settles to exactly zero "
           "after first person ends, DRY never kicks because nothing was "
           "fired, and the marker cannot ask for a kick that points downward "
           "or folds the wrist\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

static int t_hand_follows_the_controller(void)
{
    enum { STRIDE = 0x180, JOINTS = 7 };
    static unsigned char blob[DG_OBJS_ARRAY + JOINTS * STRIDE];
    static unsigned char obj[0x40], mc[0x70];
    static float adjust[55 * 4];
    ULONGLONG arm = (ULONGLONG)(ULONG_PTR)obj;
    DG_BRIDGE_ARM_TARGET target;
    float *hand_m;
    double root_q[4], root_rows[3][3], anim[4], ctrl0[4], turn[4], ctrl1[4];
    double delta_view[4], delta_world[4], expect[4], achieved[4];
    double setpos_hand[4], pulled[3];
    short command[3];
    LONG command_tick, command_pair;
    LONG written;
    int j, k, r, bad = 0;

    memset(blob, 0, sizeof blob);
    memset(obj, 0, sizeof obj);
    memset(mc, 0, sizeof mc);
    memset(adjust, 0, sizeof adjust);
    adjust[6 * 4 + 3] = 1.0f; /* SetPos ran and supplied identity. */
    *(ULONGLONG *)(obj + 0x00) = (ULONGLONG)(ULONG_PTR)blob;
    *(ULONGLONG *)(obj + 0x08) = (ULONGLONG)(ULONG_PTR)mc;
    *(LONG *)(mc + 0x14) = 55;
    *(ULONGLONG *)(mc + 0x48) = (ULONGLONG)(ULONG_PTR)adjust;
    for (j = 0; j < JOINTS; j++) {
        float *m = (float *)(blob + DG_OBJS_ARRAY + j * STRIDE);
        m[0] = m[5] = m[10] = m[15] = 1.0f;
    }

    /* A rig root that is NOT the identity, because the whole question this
       test exists for is whether the controller's rotation is carried into
       the arm's frame rather than assumed to already be in it. */
    th_axis(0.0, 1.0, 0.0, 37.0, root_q);
    th_write_basis((float *)(blob + DG_OBJS_ARRAY), root_q);
    ((float *)(blob + DG_OBJS_ARRAY))[12] = 40.0f;
    ((float *)(blob + DG_OBJS_ARRAY))[13] = -10.0f;
    ((float *)(blob + DG_OBJS_ARRAY))[14] = 25.0f;
    for (r = 0; r < 3; r++)
        for (k = 0; k < 3; k++)
            root_rows[r][k] =
                (double)((float *)(blob + DG_OBJS_ARRAY))[r * 4 + k];

    /* The same 200/200 arm the lifecycle test uses, so the mapper and solver
       have something legal to chew on; this test is only about joint 6. */
    ((float *)(blob + DG_OBJS_ARRAY + 3 * STRIDE))[14] = 100.0f;
    ((float *)(blob + DG_OBJS_ARRAY + 5 * STRIDE))[12] = 200.0f;
    ((float *)(blob + DG_OBJS_ARRAY + 6 * STRIDE))[12] = 300.0f;
    ((float *)(blob + DG_OBJS_ARRAY + 6 * STRIDE))[13] = 173.20508f;
    hand_m = (float *)(blob + DG_OBJS_ARRAY + 6 * STRIDE);
    th_axis(0.3, 0.2, -0.9, 64.0, anim);
    th_write_basis(hand_m, anim);
    hand_m[12] = 300.0f;
    hand_m[13] = 173.20508f;
    hand_m[14] = 0.0f;

    InterlockedExchange(&g_b.skel_stride, STRIDE);
    InterlockedExchange(&g_b.skel_parents_read, JOINTS);
    InterlockedExchange(&g_b.skel_region_end, (LONG)sizeof blob);
    for (j = 0; j < JOINTS; j++) InterlockedExchange(&g_b.skel_parents[j], 0);
    InterlockedExchange(&g_b.skel_parents[3], 2);
    InterlockedExchange(&g_b.skel_parents[4], 3);
    InterlockedExchange(&g_b.skel_parents[5], 4);
    InterlockedExchange(&g_b.skel_parents[6], 5);
    g_b.a.gm_player_arm_body = (ULONGLONG)(ULONG_PTR)&arm;
    InterlockedExchange(&g_b.ik_active, 0);
    InterlockedExchange(&g_b.c_ticks, 0);
    InterlockedExchange(&g_b.c_arm_hand_written, 0);
    InterlockedExchange(&g_b.c_arm_hand_refused, 0);
    InterlockedExchange(&g_b.c_arm_hand_measured, 0);
    InterlockedExchange(&g_b.s_arm_hand_worst, f2l(0.0f));
    arm_map_forget();

    /* The basis read has to survive the round trip through a float matrix,
       or every angle below is measuring the wrong thing. */
    {
        double read_back[4];
        if (!dg_ik_basis_quat((const double (*)[3])root_rows, read_back)) bad++;
        if (th_angle_between(read_back, root_q) > 1e-3) bad++;
    }

    th_axis(0.5, -0.4, 0.76, 23.0, ctrl0);
    memset(&target, 0, sizeof target);
    target.write = 1;
    target.hand_write = 1;
    target.weight = 1.0;
    target.stream_id = 9;
    target.pair_id = 300;
    target.wrist_view[0] = 770.0;
    /* A view facing, so the DGREC4 facing record has something to hold. */
    target.head_yaw_valid = 1;
    target.head_yaw_rad = 0.25;
    for (k = 0; k < 4; k++) target.hand_quat[k] = ctrl0[k];

    /* Acquisition, two settle ticks, then the calibration pair. */
    arm_ik_now(arm, &target);
    InterlockedExchange(&g_b.c_ticks, DG_ADJ_SETTLE_TICKS + 1);
    target.pair_id++;
    arm_ik_now(arm, &target);
    if (!g_b.hand_have_rest) bad++;
    if (*(ULONGLONG *)(mc + 0x38) != 0) bad++;

    /* The rest pose is stored in the ROOT frame, so it must not be the world
       rotation it was read as - otherwise the rebase is a no-op and this test
       would pass on a build that never converted anything. */
    if (th_angle_between(g_b.hand_rest_view, anim) < 1.0) bad++;

    /* Now turn the controller by a known amount and let one pair write. */
    th_axis(0.0, 0.0, 1.0, 50.0, turn);
    dg_ik_quat_mul(turn, ctrl0, ctrl1);
    for (k = 0; k < 4; k++) target.hand_quat[k] = ctrl1[k];
    written = g_b.c_arm_hand_written;
    InterlockedIncrement(&g_b.c_ticks);
    target.pair_id++;
    arm_ik_now(arm, &target);
    if (g_b.c_arm_hand_written != written + 1) bad++;
    if (g_b.c_arm_hand_refused != 0) bad++;
    /* The recorder's arm half (DGREC4, 2026-09-03): the pass that published
       this pair left the joints it solved on, the target it chased, the
       adjusts it wrote and the envelope's axis in the pair record - the
       fields whose absence kept the run-14 wrist question at the headset.
       The clean forearm keeps the rig's 200 mm whatever the adjusts did,
       the target is the one the meter published, the axis and the adjusts
       are unit, and a 50 degree ask fits the channel whole. */
    {
        const DG_REC_PAIRSTATE *pp = &g_b.rec_pair;
        double fl, n;
        float tf;
        LONG tv;
        if (!(pp->flags & DG_REC_PAIR_F_ARM)) bad++;
        if (!(pp->flags & DG_REC_PAIR_F_ENVELOPE)) bad++;
        if (!(pp->flags & DG_REC_PAIR_F_FACING)) bad++;
        fl = sqrt(((double)pp->joint_world[3][0] - pp->joint_world[2][0]) *
                  ((double)pp->joint_world[3][0] - pp->joint_world[2][0]) +
                  ((double)pp->joint_world[3][1] - pp->joint_world[2][1]) *
                  ((double)pp->joint_world[3][1] - pp->joint_world[2][1]) +
                  ((double)pp->joint_world[3][2] - pp->joint_world[2][2]) *
                  ((double)pp->joint_world[3][2] - pp->joint_world[2][2]));
        if (fabs(fl - 200.0) > 0.05) bad++;
        for (k = 0; k < 3; k++) {
            tv = g_b.s_arm_ik_target[k];
            memcpy(&tf, &tv, sizeof tf);
            if (tf != pp->ik_target[k]) bad++;
        }
        n = sqrt((double)pp->fore_axis[0] * pp->fore_axis[0] +
                 (double)pp->fore_axis[1] * pp->fore_axis[1] +
                 (double)pp->fore_axis[2] * pp->fore_axis[2]);
        if (fabs(n - 1.0) > 1e-4) bad++;
        n = sqrt((double)pp->q4_world[0] * pp->q4_world[0] +
                 (double)pp->q4_world[1] * pp->q4_world[1] +
                 (double)pp->q4_world[2] * pp->q4_world[2] +
                 (double)pp->q4_world[3] * pp->q4_world[3]);
        if (fabs(n - 1.0) > 1e-4) bad++;
        if (pp->fit_frac != 1.0f) bad++;
        if (fabs((double)pp->head_yaw_deg - 0.25 * 180.0 /
                 3.14159265358979323846) > 1e-3) bad++;
        /* The solver's wrist is where the IK put it: within the rig's reach
           of the shoulder (joint 4) and no further than the target. */
        n = sqrt(((double)pp->ik_wrist[0] - pp->joint_world[1][0]) *
                 ((double)pp->ik_wrist[0] - pp->joint_world[1][0]) +
                 ((double)pp->ik_wrist[1] - pp->joint_world[1][1]) *
                 ((double)pp->ik_wrist[1] - pp->joint_world[1][1]) +
                 ((double)pp->ik_wrist[2] - pp->joint_world[1][2]) *
                 ((double)pp->ik_wrist[2] - pp->joint_world[1][2]));
        if (!(n > 0.0 && n <= 400.0 * 0.99 + 1e-6)) bad++;
    }
    if (*(ULONGLONG *)(mc + 0x38) != ((1ULL << 4) | (1ULL << 5))) bad++;
    if (g_b.ik_owned_mask != ((1ULL << 4) | (1ULL << 5))) bad++;
    /* Joint 6 remains byte-for-byte SetPos's; the result crossed seams as an
       SVECTOR command instead. */
    if (adjust[6 * 4 + 0] != 0.0f || adjust[6 * 4 + 1] != 0.0f ||
        adjust[6 * 4 + 2] != 0.0f || adjust[6 * 4 + 3] != 1.0f) bad++;
    if (!hand_command_read(command, &command_tick, &command_pair)) bad++;
    if (command_pair != (LONG)target.pair_id) bad++;
    for (k = 0; k < 3; k++) pulled[k] = (double)th_engine_pull((int)command[k]);
    set_pos_quat(pulled, setpos_hand);
    if (fabs(setpos_hand[0]) + fabs(setpos_hand[1]) + fabs(setpos_hand[2]) <
        1.0e-4) bad++;

    /* The previous hand adjustment comes from SetPos's live slot, not our
       q4/q5 cache. If this input is ignored, the two solves below collapse to
       the same answer and the test catches the old feedback bug. */
    {
        double aq4[4], aq5[4], id[4] = { 0.0, 0.0, 0.0, 1.0 };
        double world_fb[4], adjust_fb[4], h0[4], h1[4], d0[4], d1[4];
        for (k = 0; k < 4; k++) {
            aq4[k] = (double)g_b.arm_map_cached_adjust[k];
            aq5[k] = (double)g_b.arm_map_cached_adjust[4 + k];
        }
        th_axis(1.0, 0.0, 0.0, 20.0, world_fb);
        world_quat_to_adjust(&ADJ_FRAME_LEGACY, world_fb, adjust_fb);
        if (!arm_hand_solve(root_q, root_q, anim, aq4, aq5, id, ctrl1, 1, 1.0,
                            h0, d0, NULL) ||
            !arm_hand_solve(root_q, root_q, anim, aq4, aq5, adjust_fb, ctrl1, 1, 1.0,
                            h1, d1, NULL) || th_angle_between(h0, h1) < 10.0) bad++;
        {
            double saved_ctrl[4], saved_rest[4], absolute[4];
            memcpy(saved_ctrl,g_b.hand_ctrl_rest,sizeof saved_ctrl);
            memcpy(saved_rest,g_b.hand_rest_view,sizeof saved_rest);
            th_axis(0.0,1.0,0.0,35.0,absolute);
            if (!arm_hand_solve(root_q,root_q,anim,aq4,aq5,id,ctrl1,1,1.0,h0,d0,absolute)) bad++;
            th_axis(1.0,0.0,0.0,75.0,g_b.hand_ctrl_rest);
            th_axis(0.0,0.0,1.0,-60.0,g_b.hand_rest_view);
            if (!arm_hand_solve(root_q,root_q,anim,aq4,aq5,id,ctrl1,1,1.0,h1,d1,absolute) ||
                th_angle_between(h0,h1)>1e-4 || th_angle_between(d0,absolute)>1e-4 ||
                th_angle_between(d1,absolute)>1e-4) bad++;
            memcpy(g_b.hand_ctrl_rest,saved_ctrl,sizeof saved_ctrl);
            memcpy(g_b.hand_rest_view,saved_rest,sizeof saved_rest);
        }
    }

    /* The independent expectation: the controller's turn since calibration,
       carried into the arm's frame, applied on top of the pose the hand was
       animated into at calibration. Nothing here goes through the solver. */
    dg_ik_quat_conj(ctrl0, delta_view);
    dg_ik_quat_mul(ctrl1, delta_view, delta_view);
    quat_rebase(root_q, delta_view, delta_world);
    if (!dg_ik_quat_normalize(delta_world)) bad++;
    dg_ik_quat_mul(delta_world, anim, expect);
    if (!th_next_hand(anim, setpos_hand, achieved)) bad++;
    /* The command crosses the independently tested PS2-angle quantiser and
       is now split over two float joints. Its measured worst round-trip floor
       is 0.0962 degrees, so 0.10 is the smallest honest integration bound. */
    if (th_angle_between(achieved, expect) > 0.10) bad++;

    /* A 50 degree turn of the controller has to be a 50 degree turn of the
       hand: a rebase that silently dropped the rotation would still satisfy
       the line above if `expect` were computed the same wrong way. */
    if (fabs(th_angle_between(achieved, anim) - 50.0) > 0.10) bad++;

    /* Play that pass into the matrices and let the next pair measure itself.
       The residual is the number the live run will be judged on. */
    th_write_basis(hand_m, achieved);
    hand_m[12] = 300.0f;
    hand_m[13] = 173.20508f;
    hand_m[14] = 0.0f;
    for (k = 0; k < 4; k++) adjust[6 * 4 + k] = (float)setpos_hand[k];
    InterlockedIncrement(&g_b.c_ticks);
    target.pair_id++;
    arm_ik_now(arm, &target);
    if (g_b.c_arm_hand_measured < 1) bad++;
    {
        LONG v = g_b.s_arm_hand_residual;
        float miss;
        memcpy(&miss, &v, sizeof miss);
        if (!(miss >= 0.0f) || miss > 0.10f) bad++;
    }

    /* And a residual that cannot tell a hit from a miss is not a measurement.
       Put the hand somewhere it was never asked to be and require the next
       pair to say so. */
    {
        double wrong[4], off[4];
        th_axis(0.0, 1.0, 0.0, 20.0, off);
        dg_ik_quat_mul(off, achieved, wrong);
        for (k = 0; k < 4; k++) g_b.hand_desired[k] = achieved[k];
        g_b.hand_have_desired = 1;
        th_write_basis(hand_m, wrong);
        hand_m[12] = 300.0f;
        hand_m[13] = 173.20508f;
        hand_m[14] = 0.0f;
        InterlockedIncrement(&g_b.c_ticks);
        target.pair_id++;
        arm_ik_now(arm, &target);
        {
            LONG v = g_b.s_arm_hand_residual;
            float miss;
            memcpy(&miss, &v, sizeof miss);
            if (fabs((double)miss - 20.0) > 0.05) bad++;
        }

        /* The offset-drift discriminator on that same wrong pose: the same
           wrong offset on two consecutive measured pairs is a CONSTANT frame
           error and must read near-zero drift; a changed offset must read as
           exactly the angle between the two offsets. This is the number that
           picks between a per-weapon hand frame and a composition error. */
        {
            double wrong2[4], off2[4], offq1[4], offq2[4], inv[4];
            double expected;
            LONG v2;
            float d;
            for (k = 0; k < 4; k++) g_b.hand_desired[k] = achieved[k];
            g_b.hand_have_desired = 1;
            InterlockedIncrement(&g_b.c_ticks);
            target.pair_id++;
            arm_ik_now(arm, &target);
            v2 = g_b.s_arm_hand_off_drift;
            memcpy(&d, &v2, sizeof d);
            if (!(d >= 0.0f) || d > 0.05f) bad++;

            th_axis(1.0, 0.0, 0.0, 20.0, off2);
            dg_ik_quat_mul(off2, achieved, wrong2);
            th_write_basis(hand_m, wrong2);
            hand_m[12] = 300.0f;
            hand_m[13] = 173.20508f;
            hand_m[14] = 0.0f;
            for (k = 0; k < 4; k++) g_b.hand_desired[k] = achieved[k];
            g_b.hand_have_desired = 1;
            InterlockedIncrement(&g_b.c_ticks);
            target.pair_id++;
            arm_ik_now(arm, &target);
            dg_ik_quat_conj(achieved, inv);
            dg_ik_quat_mul(wrong, inv, offq1);
            dg_ik_quat_mul(wrong2, inv, offq2);
            expected = th_angle_between(offq1, offq2);
            if (expected < 10.0) bad++;
            v2 = g_b.s_arm_hand_off_drift;
            memcpy(&d, &v2, sizeof d);
            if (fabs((double)d - expected) > 0.05) bad++;
            v2 = g_b.s_arm_hand_off_drift_worst;
            memcpy(&d, &v2, sizeof d);
            if ((double)d < expected - 0.05) bad++;
        }
        th_write_basis(hand_m, achieved);
        hand_m[12] = 300.0f;
        hand_m[13] = 173.20508f;
        hand_m[14] = 0.0f;
    }

    /* Four zero floats are the instrument's "SetPos did not run" sentinel,
       not a quaternion and not a 0.98 conversion miss. It must suppress the
       publication while joints 4/5 remain legal. */
    {
        LONG no_setpos = g_b.c_arm_hand_no_setpos;
        for (k = 0; k < 4; k++) adjust[6 * 4 + k] = 0.0f;
        InterlockedIncrement(&g_b.c_ticks);
        target.pair_id++;
        arm_ik_now(arm, &target);
        if (g_b.c_arm_hand_no_setpos != no_setpos + 1) bad++;
        if (hand_command_read(command, NULL, NULL)) bad++;
        if (*(ULONGLONG *)(mc + 0x38) != ((1ULL << 4) | (1ULL << 5))) bad++;
        for (k = 0; k < 4; k++) adjust[6 * 4 + k] = (float)setpos_hand[k];
    }

    /* The removal witness: a pair that arrives with any of our adjust_flag
       bits gone means the hierarchy is pure animation and the removal would
       un-rotate a rotation that was never applied. It must be counted - one
       missing bit is enough - and the pair's own write puts the bits back,
       so the very next pair counts nothing. */
    {
        LONG lost = g_b.c_arm_adjust_bits_lost;
        *(ULONGLONG *)(mc + 0x38) = (1ULL << 4);
        InterlockedIncrement(&g_b.c_ticks);
        target.pair_id++;
        arm_ik_now(arm, &target);
        if (g_b.c_arm_adjust_bits_lost != lost + 1) bad++;
        if (*(ULONGLONG *)(mc + 0x38) != ((1ULL << 4) | (1ULL << 5))) bad++;
        InterlockedIncrement(&g_b.c_ticks);
        target.pair_id++;
        arm_ik_now(arm, &target);
        if (g_b.c_arm_adjust_bits_lost != lost + 1) bad++;
    }

    /* The slot echo: the residual's measurement cut at SetPos's slot. This
       simplified rig does not carry the full joint-4/5 inheritance the live
       hierarchy has, so consecutive commands here legitimately differ by a
       few degrees and no absolute smallness is asserted after choreography.
       What IS the instrument's contract: a slot that echoes the previous
       command exactly must read clean, and a slot holding something the
       previous pair never published must be measured at its true angle and
       counted - or the live discriminator cannot tell a conversion the game
       refused from a hierarchy that misapplied it. */
    {
        LONG dirty0;
        LONG v;
        float e;
        double off[4], poison[4];

        for (k = 0; k < 4; k++)
            adjust[6 * 4 + k] = (float)g_b.hand_last_cmd[k];
        InterlockedIncrement(&g_b.c_ticks);
        target.pair_id++;
        arm_ik_now(arm, &target);
        dirty0 = g_b.c_arm_hand_slot_dirty;
        v = g_b.s_arm_hand_slot_echo;
        memcpy(&e, &v, sizeof e);
        if (!(e >= 0.0f) || e > 0.01f) bad++;

        /* And through the game's own reconstruction: read the command just
           published, put it through the modeled pull and conversion, and the
           echo must be the angle grid and nothing more. This is the case
           that pins WHAT was stored - only the adjust quaternion whose
           angles actually left can echo clean through the real path. */
        {
            short cx[3];
            double pulled2[3], slot2[4];
            if (!hand_command_read(cx, NULL, NULL)) bad++;
            for (k = 0; k < 3; k++)
                pulled2[k] = (double)th_engine_pull((int)cx[k]);
            set_pos_quat(pulled2, slot2);
            for (k = 0; k < 4; k++) adjust[6 * 4 + k] = (float)slot2[k];
            InterlockedIncrement(&g_b.c_ticks);
            target.pair_id++;
            arm_ik_now(arm, &target);
            v = g_b.s_arm_hand_slot_echo;
            memcpy(&e, &v, sizeof e);
            /* 0.0962 is the measured quantiser round-trip floor; a command
               whose vy rounds to zero is pushed one unit off it (0.088 deg,
               the branch-pinning clamp) on top. 0.20 covers both and is
               still 25 times under the dirty threshold. */
            if (!(e >= 0.0f) || e > 0.20f) bad++;
            if (g_b.c_arm_hand_slot_dirty != dirty0) bad++;
        }

        th_axis(0.0, 0.0, 1.0, 90.0, off);
        dg_ik_quat_mul(off, g_b.hand_last_cmd, poison);
        for (k = 0; k < 4; k++) adjust[6 * 4 + k] = (float)poison[k];
        InterlockedIncrement(&g_b.c_ticks);
        target.pair_id++;
        arm_ik_now(arm, &target);
        v = g_b.s_arm_hand_slot_echo;
        memcpy(&e, &v, sizeof e);
        if (fabs((double)e - 90.0) > 0.1) bad++;
        if (g_b.c_arm_hand_slot_dirty != dirty0 + 1) bad++;
        v = g_b.s_arm_hand_slot_echo_worst;
        memcpy(&e, &v, sizeof e);
        if (e < 89.9f) bad++;
        for (k = 0; k < 4; k++) adjust[6 * 4 + k] = (float)setpos_hand[k];
    }

    /* The automatic B-press. The threshold first, as arithmetic: a boundary
       gap is still a blip, one tick past it is a scene change. */
    if (rest_stale_after_gap(0, DG_HAND_REST_STALE_TICKS)) bad++;
    if (!rest_stale_after_gap(0, DG_HAND_REST_STALE_TICKS + 1)) bad++;

    /* Then the machinery end to end: a pair-stream gap exactly AT the
       threshold re-anchors nothing; one past it raises the flag, which
       PENDS across a pair that cannot recapture (SetPos absent) and fires
       on the next one that can, replacing BOTH halves of the rest pair -
       the controller side must become what the player holds NOW, not the
       calibration-day pose - and re-anchoring the body-yaw zero. */
    {
        LONG recaptured = g_b.c_arm_rest_recaptured;
        double old_rest[4];
        float cache_at_capture[8];
        for (k = 0; k < 4; k++) old_rest[k] = g_b.hand_ctrl_rest[k];
        if (th_angle_between(old_rest, ctrl1) < 1.0) bad++;
        /* Boundary gap: the +1 below makes now - last == the threshold. */
        InterlockedExchangeAdd(&g_b.c_ticks, DG_HAND_REST_STALE_TICKS - 1);
        InterlockedIncrement(&g_b.c_ticks);
        target.pair_id++;
        arm_ik_now(arm, &target);
        if (g_b.c_arm_rest_recaptured != recaptured) bad++;
        if (InterlockedCompareExchange(&g_b.hand_rest_stale, 0, 0)) bad++;
        /* One past the threshold, against an absent SetPos: detected AND
           pended, in one pair. */
        for (k = 0; k < 4; k++) adjust[6 * 4 + k] = 0.0f;
        InterlockedExchangeAdd(&g_b.c_ticks, DG_HAND_REST_STALE_TICKS);
        InterlockedIncrement(&g_b.c_ticks);
        target.pair_id++;
        arm_ik_now(arm, &target);
        if (g_b.c_arm_rest_recaptured != recaptured) bad++;
        if (!InterlockedCompareExchange(&g_b.hand_rest_stale, 0, 0)) bad++;
        for (k = 0; k < 4; k++) adjust[6 * 4 + k] = (float)setpos_hand[k];
        /* Dropped on purpose so the assert below can only be satisfied by
           the recapture actually re-anchoring the body-yaw zero. */
        g_b.arm_root_have_q0 = 0;
        /* The capture strips with the cache AS THE PAIR FINDS IT - the
           previous pair's, the one the hierarchy carries - so the
           replication below has to read it BEFORE the pair overwrites it
           with its own solve. */
        for (k = 0; k < 8; k++)
            cache_at_capture[k] = g_b.arm_map_cached_adjust[k];
        InterlockedIncrement(&g_b.c_ticks);
        target.pair_id++;
        arm_ik_now(arm, &target);
        if (g_b.c_arm_rest_recaptured != recaptured + 1) bad++;
        if (InterlockedCompareExchange(&g_b.hand_rest_stale, 0, 0)) bad++;
        if (th_angle_between(g_b.hand_ctrl_rest, ctrl1) > 1e-6) bad++;
        if (!g_b.arm_root_have_q0) bad++;
        if (!g_b.hand_have_desired) bad++;
        /* The recaptured rest must BE the current animation base: live with
           the three cached adjusts stripped back off, carried into the root
           frame - replicated here from the same inputs the pair had. (This
           rig never bakes the adjusts into its matrices, so that base is
           NOT `anim` here - the real hierarchy's is, as adjust-bits-lost
           witnesses live - and asserting the strip rather than anim tests
           the code's contract instead of the rig's shortcut.) */
        {
            double aq2[4], pu2[4], pf2[4], ph3[4], ch2[4], iv2[4], an2[4];
            double rinv[4], expect_rest[4];
            for (k = 0; k < 4; k++)
                aq2[k] = (double)cache_at_capture[k];
            if (!adjust_quat_to_world(&ADJ_FRAME_LEGACY, aq2, pu2)) bad++;
            for (k = 0; k < 4; k++)
                aq2[k] = (double)cache_at_capture[4 + k];
            if (!adjust_quat_to_world(&ADJ_FRAME_LEGACY, aq2, pf2)) bad++;
            if (!adjust_quat_to_world(&ADJ_FRAME_LEGACY, setpos_hand, ph3)) bad++;
            dg_ik_quat_mul(ph3, pf2, ch2);
            dg_ik_quat_mul(ch2, pu2, ch2);
            dg_ik_quat_conj(ch2, iv2);
            dg_ik_quat_mul(iv2, achieved, an2);
            dg_ik_quat_conj(root_q, rinv);
            quat_rebase(rinv, an2, expect_rest);
            if (!dg_ik_quat_normalize(expect_rest)) bad++;
            /* 1e-4 degrees: the cached adjusts round-trip through floats
               between the capture and this replication. */
            if (th_angle_between(g_b.hand_rest_view, expect_rest) > 1e-4)
                bad++;
        }
    }

    /* The recoil, seen from the seam that consumes it rather than the one
       that steps it. Two things can only be checked here.

       That the climb reaches the hand at all: with a kick standing in the
       spring the published SVECTOR has to differ from the one the same pose
       produced without it, or the whole feature is a counter that moves and
       nothing else.

       And - the one that matters more - that the camera seam only READS the
       envelope. This seam runs about five times per game tick. If anything on
       this path stepped the spring, recoil would run five times too fast and
       decay five times too soon, and the only symptom would be that it felt
       slightly wrong. So: the amplitude before, several full seam passes, the
       amplitude after, and they must be the same number. */
    {
        short plain[3], kicked[3];
        double spring_before, spring_after;
        LONG amp_before, amp_after;
        int moved = 0;

        target.hand_write = 1;
        InterlockedExchange(&g_b.recoil_climb_mdeg, 0);
        InterlockedExchange(&g_b.recoil_push_um, 0);
        InterlockedExchange(&g_b.s_recoil_amp, 0);
        InterlockedIncrement(&g_b.c_ticks);
        target.pair_id++;
        arm_ik_now(arm, &target);
        if (!hand_command_read(plain, NULL, NULL)) bad++;

        /* A kick standing at full amplitude, as the tick seam would leave it. */
        InterlockedExchange(&g_b.recoil_climb_mdeg, 6000);
        InterlockedExchange(&g_b.recoil_push_um, 18000);
        InterlockedExchange(&g_b.s_recoil_amp, 1000);
        dg_recoil_reset(&g_b.recoil);
        dg_recoil_fire(&g_b.recoil, 1.0);
        dg_recoil_step(&g_b.recoil);
        amp_before = InterlockedCompareExchange(&g_b.s_recoil_amp, 0, 0);
        /* The spring itself, not just the LONG the tick seam publishes from
           it: only the tick seam ever writes that LONG, so watching it could
           not see a camera seam quietly stepping the envelope underneath. */
        spring_before = dg_recoil_amplitude(&g_b.recoil);
        if (!(spring_before > 0.0)) bad++;

        for (r = 0; r < 5; r++) {          /* one tick's worth of camera seams */
            target.pair_id++;
            arm_ik_now(arm, &target);
        }
        if (hand_command_read(kicked, NULL, NULL)) {
            for (k = 0; k < 3; k++) if (kicked[k] != plain[k]) moved = 1;
        } else {
            bad++;
        }
        if (!moved) bad++;                 /* the climb never reached the hand */

        amp_after = InterlockedCompareExchange(&g_b.s_recoil_amp, 0, 0);
        spring_after = dg_recoil_amplitude(&g_b.recoil);
        if (amp_after != amp_before) bad++;
        if (spring_after != spring_before) bad++;  /* a seam stepped it */
        if (g_b.c_recoil_climb_writes <= 0) bad++;

        InterlockedExchange(&g_b.recoil_climb_mdeg, 0);
        InterlockedExchange(&g_b.recoil_push_um, 0);
        InterlockedExchange(&g_b.s_recoil_amp, 0);
        dg_recoil_reset(&g_b.recoil);
    }

    /* The channel's reach (run 13, 2026-09-03). A hand asked 175 degrees
       from its calibration - the swing cap opened to its 180 limit the way
       the marker can - wants a joint-6 adjust the SVECTOR cannot carry: past
       1536 units in a component the engine folds the precompensated short
       and lands three quarters of a turn away. The bridge must shorten the
       adjust along its own axis, store the SHORTENED rotation as the command
       echo and publish it as the desired hand, and the engine's own
       reconstruction - fold, pull, convert, compose onto the rig - must then
       land on it: a clean echo and a clean residual where the naive write
       measured 90 degrees on every pair of the episode. */
    {
        double turn_big[4], ctrl_big[4], slot_big[4], next_hand[4];
        double pulled_big[3], base_big[4];
        short cmd_big[3];
        LONG scaled0 = g_b.c_arm_hand_scaled;
        LONG dirty_big, v_big;
        float e_big, fr_big;
        InterlockedExchange(&g_b.hand_wrist_swing_mdeg, 180000);
        th_axis(0.0, 0.0, 1.0, 175.0, turn_big);
        dg_ik_quat_mul(turn_big, ctrl0, ctrl_big);
        for (k = 0; k < 4; k++) target.hand_quat[k] = ctrl_big[k];
        InterlockedIncrement(&g_b.c_ticks);
        target.pair_id++;
        arm_ik_now(arm, &target);
        /* The ask really was past the reach and was shortened - or this leg
           tests nothing. A fraction inside (0.5, 1): the run-13 data needed
           0.81 at worst. */
        if (g_b.c_arm_hand_scaled != scaled0 + 1) bad++;
        v_big = g_b.s_arm_hand_fit;
        memcpy(&fr_big, &v_big, sizeof fr_big);
        if (!(fr_big > 0.5f && fr_big < 1.0f)) bad++;
        v_big = g_b.s_arm_hand_shortfall_worst;
        memcpy(&e_big, &v_big, sizeof e_big);
        if (fabs((double)e_big - (1.0 - (double)fr_big)) > 1e-6) bad++;
        if (!hand_command_read(cmd_big, NULL, NULL)) bad++;
        for (k = 0; k < 3; k++)
            pulled_big[k] = (double)th_engine_pull((int)cmd_big[k]);
        set_pos_quat(pulled_big, slot_big);
        /* What the channel delivers is what was stored as the command. */
        if (th_angle_between(slot_big, g_b.hand_last_cmd) > 0.20) bad++;
        /* The game runs it: the slot holds the reconstruction, the hierarchy
           composes it onto the animation, the next pair measures both. */
        for (k = 0; k < 4; k++) adjust[6 * 4 + k] = (float)slot_big[k];
        /* Composed onto the animation base the bridge recovered for this
           pair (live with our own adjusts stripped), which is the pose the
           earlier legs left in the rig - not the calibration animation. */
        if (!g_b.hand_have_prev_base) bad++;
        for (k = 0; k < 4; k++) base_big[k] = g_b.hand_prev_base[k];
        if (!th_next_hand(base_big, slot_big, next_hand)) bad++;
        th_write_basis(hand_m, next_hand);
        hand_m[12] = 300.0f;
        hand_m[13] = 173.20508f;
        hand_m[14] = 0.0f;
        dirty_big = g_b.c_arm_hand_slot_dirty;
        InterlockedIncrement(&g_b.c_ticks);
        target.pair_id++;
        arm_ik_now(arm, &target);
        v_big = g_b.s_arm_hand_residual;
        memcpy(&e_big, &v_big, sizeof e_big);
        if (!(e_big >= 0.0f) || e_big > 0.20f) bad++;
        v_big = g_b.s_arm_hand_slot_echo;
        memcpy(&e_big, &v_big, sizeof e_big);
        if (!(e_big >= 0.0f) || e_big > 0.20f) bad++;
        if (g_b.c_arm_hand_slot_dirty != dirty_big) bad++;
        /* Back to the rig the closing checks expect. */
        InterlockedExchange(&g_b.hand_wrist_swing_mdeg, 0);
        for (k = 0; k < 4; k++) target.hand_quat[k] = ctrl1[k];
        for (k = 0; k < 4; k++) adjust[6 * 4 + k] = (float)setpos_hand[k];
    }

    /* Absolute-aim refusal must invalidate an already published command even
       when arm processing takes a duplicate or constant-cache early return.
       Keep caller hand_write=1: only the absolute validity guard may refuse
       these commands. Invalid metadata short-circuits before retail reads. */
    {
        DG_BRIDGE_ARM_TARGET rejected = target;
        short old_command[3] = {72, 8, -32}, read_command[3];
        LONG replay0 = g_b.c_arm_pair_replays;
        LONG frozen0 = g_b.c_arm_pairs_frozen;
        LONG written0 = g_b.c_arm_hand_written;
        LONG saved_freeze = InterlockedCompareExchange(&g_b.arm_freeze, 0, 0);
        int guard_bad = 0;
        if (!g_b.arm_map_cache_valid || !g_b.ik_active ||
            target.pair_id != g_b.arm_map_last_pair) guard_bad++;
        rejected.absolute_aim = 1;
        rejected.hand_write = 1;
        rejected.aim_write = 0;
        InterlockedExchange(&g_b.hand_command_requested, 1);
        if (!hand_command_publish(old_command, rejected.pair_id) ||
            !hand_command_read(read_command, NULL, NULL)) guard_bad++;
        arm_ik_now(arm, &rejected);
        if (g_b.c_arm_pair_replays != replay0 + 1 ||
            !g_b.arm_map_cache_valid || g_b.c_arm_hand_written != written0 ||
            InterlockedCompareExchange(&g_b.hand_command_valid, 0, 0) ||
            InterlockedCompareExchange(&g_b.hand_command_requested, 0, 0) ||
            hand_command_read(read_command, NULL, NULL)) guard_bad++;

        /* Fresh pair, valid-looking payload, deliberately wrong aim pair ID:
           this must still refuse before the constant replay branch. */
        rejected.pair_id++;
        rejected.aim_write = 1;
        rejected.aim_pair_id = rejected.pair_id - 1;
        rejected.aim_stream_id = rejected.stream_id;
        rejected.aim_sample_seq = 1;
        rejected.aim_sample_time = 1;
        InterlockedExchange(&g_b.arm_freeze, 1);
        InterlockedExchange(&g_b.hand_command_requested, 1);
        if (!hand_command_publish(old_command, rejected.pair_id) ||
            !hand_command_read(read_command, NULL, NULL)) guard_bad++;
        arm_ik_now(arm, &rejected);
        if (g_b.c_arm_pairs_frozen != frozen0 + 1 ||
            !g_b.arm_map_cache_valid || g_b.c_arm_hand_written != written0 ||
            InterlockedCompareExchange(&g_b.hand_command_valid, 0, 0) ||
            InterlockedCompareExchange(&g_b.hand_command_requested, 0, 0) ||
            hand_command_read(read_command, NULL, NULL)) guard_bad++;
        InterlockedExchange(&g_b.arm_freeze, saved_freeze);
        target.pair_id = rejected.pair_id;
        if (guard_bad) printf("    FAIL absolute aim invalidation before duplicate/constant replay (%d)\n", guard_bad);
        bad += guard_bad;
    }

    /* Turning the hand off leaves SetPos's joint 6 intact and the arm working. */
    target.hand_write = 0;
    InterlockedIncrement(&g_b.c_ticks);
    target.pair_id++;
    arm_ik_now(arm, &target);
    if (*(ULONGLONG *)(mc + 0x38) != ((1ULL << 4) | (1ULL << 5))) bad++;
    if (g_b.ik_owned_mask != ((1ULL << 4) | (1ULL << 5))) bad++;
    for (k = 0; k < 4; k++)
        if (adjust[6 * 4 + k] != (float)setpos_hand[k]) bad++;
    if (g_b.c_arm_pairs_accepted <= 0) bad++;

    arm_map_forget();
    if (*(ULONGLONG *)(mc + 0x38) != 0) bad++;
    g_b.a.gm_player_arm_body = 0;
    InterlockedExchange(&g_b.skel_stride, 0);
    InterlockedExchange(&g_b.skel_parents_read, 0);
    InterlockedExchange(&g_b.skel_region_end, 0);
    InterlockedExchange(&g_b.c_ticks, 0);

    printf("  %-6s hand drive: a non-identity rig root, a 50 degree controller "
           "turn published across seams as a precompensated SVECTOR, SetPos's "
           "joint 6 left intact, achieved rotation measured as residual, and a "
           "standing recoil kick that changes the published hand while five "
           "camera seams in one tick leave the envelope exactly where it "
           "was, a 150 degree ask past the channel's reach shortened "
           "along its axis and landing clean through the folded pull, and "
           "the pair record carrying the solved joints, target, adjusts and "
           "envelope axis (DGREC4)\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

/* The arm must not orbit a turning body. The published camera never follows
   body yaw, so the player's input arrives in a world-fixed frame while the
   arm root turns with the character - the tolling-arm videos, one revolution
   of accumulated body yaw at a time. Three legs. A body YAW about its own up
   axis leaves the shoulder-relative world offset and the desired hand
   orientation exactly where the player holds them, and reads out in the
   drift telemetry. A body PITCH is carried: the view input is left alone and
   arm and hand pitch with the torso - a compensation built from the full
   relative rotation instead of its yaw twist fails this leg. And a pair with
   no captured calibration root runs uncompensated and is counted, never
   guessed. The calibration root deliberately contains pitch as well as yaw:
   pure-yaw roots commute with body yaws, and under commuting rotations a
   relative rotation composed in the wrong order would be invisible. */
static int t_the_arm_ignores_the_bodys_yaw(void)
{
    enum { STRIDE = 0x180, JOINTS = 7 };
    static unsigned char blob[DG_OBJS_ARRAY + JOINTS * STRIDE];
    static unsigned char rig0[DG_OBJS_ARRAY + JOINTS * STRIDE];
    static unsigned char obj[0x40], mc[0x70];
    static float adjust[55 * 4];
    ULONGLONG arm = (ULONGLONG)(ULONG_PTR)obj;
    DG_BRIDGE_ARM_TARGET target;
    float *hand_m;
    double y37[4], x20[4], root_q[4], anim[4], ctrl0[4];
    double spin[4], turn[4], live_root[4], expect_desired[4];
    double ytw[4], ytw_inv[4], swing[4], yw[4], yw_ctrl[4];
    double root_pos[3] = { 40.0, -10.0, 25.0 };
    LONG saved_basis = InterlockedCompareExchange(&g_b.arm_hand_basis, 0, 0);
    double sv[3], wv[3], tmp[3];
    double base_offset[3], offset_now[3], expect_offset[3];
    double base_desired[4], base_view[3], view_now[3];
    double scale;
    LONG uncomp, saved_anchor, v;
    float f;
    int j, k, bad = 0;

    memset(blob, 0, sizeof blob);
    memset(obj, 0, sizeof obj);
    memset(mc, 0, sizeof mc);
    memset(adjust, 0, sizeof adjust);
    adjust[6 * 4 + 3] = 1.0f; /* SetPos ran and supplied identity. */
    *(ULONGLONG *)(obj + 0x00) = (ULONGLONG)(ULONG_PTR)blob;
    *(ULONGLONG *)(obj + 0x08) = (ULONGLONG)(ULONG_PTR)mc;
    *(LONG *)(mc + 0x14) = 55;
    *(ULONGLONG *)(mc + 0x48) = (ULONGLONG)(ULONG_PTR)adjust;
    for (j = 0; j < JOINTS; j++) {
        float *m = (float *)(blob + DG_OBJS_ARRAY + j * STRIDE);
        m[0] = m[5] = m[10] = m[15] = 1.0f;
    }
    th_axis(0.0, 1.0, 0.0, 37.0, y37);
    th_axis(1.0, 0.0, 0.0, 8.0, x20);
    dg_ik_quat_mul(y37, x20, root_q);
    th_write_basis((float *)(blob + DG_OBJS_ARRAY), root_q);
    for (k = 0; k < 3; k++)
        ((float *)(blob + DG_OBJS_ARRAY))[12 + k] = (float)root_pos[k];
    ((float *)(blob + DG_OBJS_ARRAY + 3 * STRIDE))[14] = 100.0f;
    ((float *)(blob + DG_OBJS_ARRAY + 5 * STRIDE))[12] = 200.0f;
    hand_m = (float *)(blob + DG_OBJS_ARRAY + 6 * STRIDE);
    th_axis(0.3, 0.2, -0.9, 64.0, anim);
    th_write_basis(hand_m, anim);
    hand_m[12] = 300.0f;
    hand_m[13] = 173.20508f;
    hand_m[14] = 0.0f;
    memcpy(rig0, blob, sizeof blob);

    InterlockedExchange(&g_b.skel_stride, STRIDE);
    InterlockedExchange(&g_b.skel_parents_read, JOINTS);
    InterlockedExchange(&g_b.skel_region_end, (LONG)sizeof blob);
    for (j = 0; j < JOINTS; j++) InterlockedExchange(&g_b.skel_parents[j], 0);
    InterlockedExchange(&g_b.skel_parents[3], 2);
    InterlockedExchange(&g_b.skel_parents[4], 3);
    InterlockedExchange(&g_b.skel_parents[5], 4);
    InterlockedExchange(&g_b.skel_parents[6], 5);
    g_b.a.gm_player_arm_body = (ULONGLONG)(ULONG_PTR)&arm;
    saved_anchor = InterlockedCompareExchange(&g_b.arm_anchor, 0, 0);
    InterlockedExchange(&g_b.arm_anchor, 1);
    InterlockedExchange(&g_b.ik_active, 0);
    InterlockedExchange(&g_b.c_ticks, 0);
    InterlockedExchange(&g_b.arm_hand_basis, 1);
    arm_map_forget();

    /* The player: shoulder anchored, controller held along the direction of
       the character's own animated wrist, so the mapped target stays close
       to the animation and every leg below is read far from any clamp. */
    for (k = 0; k < 3; k++) tmp[k] = 0.0 - root_pos[k];
    arm_quat_unrotate(root_q, tmp, sv);
    tmp[0] = 300.0 - root_pos[0];
    tmp[1] = 173.20508 - root_pos[1];
    tmp[2] = 0.0 - root_pos[2];
    arm_quat_unrotate(root_q, tmp, wv);

    th_axis(0.5, -0.4, 0.76, 23.0, ctrl0);
    memset(&target, 0, sizeof target);
    target.write = 1;
    target.hand_write = 1;
    target.weight = 1.0;
    target.stream_id = 11;
    target.pair_id = 500;
    target.player_reach_view = 600.0;
    target.player_shoulder_view[0] = 60.0;
    target.player_shoulder_view[1] = -80.0;
    target.player_shoulder_view[2] = 30.0;
    for (k = 0; k < 3; k++)
        target.wrist_view[k] = target.player_shoulder_view[k] +
                               (wv[k] - sv[k]);
    for (k = 0; k < 4; k++) target.hand_quat[k] = ctrl0[k];
    /* The stick term rides along from calibration on, so the organic leg
       below has a zero to measure against. */
    target.stick_yaw_valid = 1;
    target.stick_yaw_rad = 0.0;
    InterlockedExchange(&g_b.arm_comp, 0);

    /* Acquisition, settle, calibration - which must capture the root. */
    arm_ik_now(arm, &target);
    InterlockedExchange(&g_b.c_ticks, DG_ADJ_SETTLE_TICKS + 1);
    target.pair_id++;
    arm_ik_now(arm, &target);
    if (!g_b.arm_root_have_q0) bad++;
    if (!g_b.hand_have_rest) bad++;
    uncomp = g_b.c_arm_body_uncompensated;

    /* One probe pair to read the mapper's scale, so the real baseline can
       put the target at 80 percent of the way to the animated wrist without
       hard-coding the scale formula here. */
    InterlockedIncrement(&g_b.c_ticks);
    target.pair_id++;
    arm_ik_now(arm, &target);
    v = InterlockedCompareExchange(&g_b.s_arm_map_scale, 0, 0);
    memcpy(&f, &v, sizeof f);
    scale = (double)f;
    if (!(scale > 0.2) || !(scale < 1.5)) { bad++; scale = 0.5716; }
    for (k = 0; k < 3; k++)
        target.wrist_view[k] = target.player_shoulder_view[k] +
                               (wv[k] - sv[k]) * (0.95 / scale);

    /* The baseline pair: an unturned body. The controller has not turned
       since calibration, so the desired hand pose must be the animated one -
       the fact the yaw leg stands on. */
    InterlockedIncrement(&g_b.c_ticks);
    target.pair_id++;
    arm_ik_now(arm, &target);
    if (!g_b.hand_have_desired) bad++;
    th_pair_offset(root_q, root_pos, base_offset);
    for (k = 0; k < 4; k++) base_desired[k] = g_b.hand_desired[k];
    for (k = 0; k < 3; k++) {
        v = InterlockedCompareExchange(&g_b.s_arm_map_target[k], 0, 0);
        memcpy(&f, &v, sizeof f);
        base_view[k] = (double)f;
    }
    if (th_angle_between(base_desired, anim) > 0.01) bad++;
    if (sqrt(base_offset[0] * base_offset[0] +
             base_offset[1] * base_offset[1] +
             base_offset[2] * base_offset[2]) < 100.0) bad++;
    v = InterlockedCompareExchange(&g_b.s_arm_body_drift, 0, 0);
    memcpy(&f, &v, sizeof f);
    if (fabs((double)f) > 0.01) bad++;
    if (g_b.c_arm_body_uncompensated != uncomp) bad++;

    /* Leg 1: the body yaws 25 degrees about its own up axis - and the
       stick term says the player turned 25 too. Under vr_arm_comp=full
       (default) that makes no difference: every degree of body yaw is
       compensated and the hand stays world-fixed. */
    th_axis(0.0, 1.0, 0.0, 25.0, spin);
    quat_rebase(root_q, spin, turn);
    memcpy(blob, rig0, sizeof blob);
    th_turn_rig(blob + DG_OBJS_ARRAY, STRIDE, JOINTS, turn);
    dg_ik_quat_mul(turn, root_q, live_root);
    target.stick_yaw_rad = 25.0 * 3.14159265358979323846 / 180.0;
    InterlockedIncrement(&g_b.c_ticks);
    target.pair_id++;
    arm_ik_now(arm, &target);
    if (!g_b.hand_have_desired) bad++;
    th_pair_offset(live_root, root_pos, offset_now);
    for (k = 0; k < 3; k++)
        if (fabs(offset_now[k] - base_offset[k]) > 0.5) bad++;
    /* The hand: world-fixed against the yaw. This rig's body yaws about
       its OWN (8-degree-canted) up axis, so the turn has a 3.5-degree
       swing beside its world yaw, and the swing is carried, as a crouch
       is: expected = swing(turn) * anim. The live body yaws about world
       up (run 9: swing 0.65 deg mean), where this is anim itself. */
    quat_ytwist(turn, ytw);
    dg_ik_quat_conj(ytw, ytw_inv);
    dg_ik_quat_mul(turn, ytw_inv, swing);
    dg_ik_quat_mul(swing, anim, expect_desired);
    if (th_angle_between(g_b.hand_desired, expect_desired) > 0.1) bad++;
    if (!(th_angle_between(g_b.hand_desired, base_desired) < 5.0)) bad++;
    v = InterlockedCompareExchange(&g_b.s_arm_body_drift, 0, 0);
    memcpy(&f, &v, sizeof f);
    if (fabs((double)f - 25.0) > 0.05) bad++;
    if (g_b.c_arm_body_uncompensated != uncomp) bad++;

    /* Leg 1b: the same body yaw and the same stick term under
       vr_arm_comp=organic: the body merely followed the stick, so nothing
       is compensated - the offset and the hand turn WITH the body, the
       drift telemetry still reads the full 25, and the no-stick counter
       stays put. Then, with the stick zero gone, organic falls back to the
       full compensation and says so. */
    {
        LONG ns0 = g_b.c_arm_comp_no_stick;
        InterlockedExchange(&g_b.arm_comp, 1);
        /* The ROOM frame advances the controller by the stick turn, so
           the controller quat carries the 25 too. */
        th_axis(0.0, 1.0, 0.0, 25.0, yw);
        dg_ik_quat_mul(yw, ctrl0, yw_ctrl);
        for (k = 0; k < 4; k++) target.hand_quat[k] = yw_ctrl[k];
        InterlockedIncrement(&g_b.c_ticks);
        target.pair_id++;
        arm_ik_now(arm, &target);
        th_pair_offset(live_root, root_pos, offset_now);
        arm_quat_rotate(turn, base_offset, expect_offset);
        for (k = 0; k < 3; k++)
            if (fabs(offset_now[k] - expect_offset[k]) > 0.5) bad++;
        /* The hand turned WITH the controller: yaw(25) * anim from the
           left, plus the rig's own-axis swing. The conjugation
           turn * anim * turn^-1 that stood here until 2026-09-02 was the
           defect itself: a cant about the hand's own axis. */
        dg_ik_quat_mul(yw, anim, expect_desired);
        dg_ik_quat_mul(swing, expect_desired, expect_desired);
        if (th_angle_between(g_b.hand_desired, expect_desired) > 0.1) bad++;
        v = InterlockedCompareExchange(&g_b.s_arm_body_drift, 0, 0);
        memcpy(&f, &v, sizeof f);
        if (fabs((double)f - 25.0) > 0.05) bad++;
        if (g_b.c_arm_comp_no_stick != ns0) bad++;
        g_b.stick_yaw0_have = 0;
        InterlockedIncrement(&g_b.c_ticks);
        target.pair_id++;
        arm_ik_now(arm, &target);
        if (g_b.c_arm_comp_no_stick != ns0 + 1) bad++;
        th_pair_offset(live_root, root_pos, offset_now);
        for (k = 0; k < 3; k++)
            if (fabs(offset_now[k] - base_offset[k]) > 0.5) bad++;
        g_b.stick_yaw0_have = 1;
        for (k = 0; k < 4; k++) target.hand_quat[k] = ctrl0[k];
        InterlockedExchange(&g_b.arm_comp, 0);
        target.stick_yaw_rad = 0.0;
    }

    /* Leg 1c: the live case, hand-computed. The body yaws 25 about WORLD
       up (run 9: the root's rotation since calibration is a world-up yaw
       to 0.65 deg), the stick says 25, organic comp, the controller
       carries the 25: the hand is exactly yaw(25) * anim =
       [0.053615, 0.290274, -0.515743, 0.804286] (Hamilton product of
       axis(0,1,0) 25 deg and axis(0.3,0.2,-0.9) 64 deg, computed by hand
       outside this code). The old basis missed it by 23.6 degrees. Then
       the same at 180/180, where the old basis missed by 113. */
    {
        static const double want25[4] =
            { 0.053615, 0.290274, -0.515743, 0.804286 };
        static const double deg[2] = { 25.0, 180.0 };
        int i;
        InterlockedExchange(&g_b.arm_comp, 1);
        for (i = 0; i < 2; i++) {
            th_axis(0.0, 1.0, 0.0, deg[i], yw);
            memcpy(blob, rig0, sizeof blob);
            th_turn_rig(blob + DG_OBJS_ARRAY, STRIDE, JOINTS, yw);
            dg_ik_quat_mul(yw, ctrl0, yw_ctrl);
            for (k = 0; k < 4; k++) target.hand_quat[k] = yw_ctrl[k];
            target.stick_yaw_rad = deg[i] * 3.14159265358979323846 / 180.0;
            InterlockedIncrement(&g_b.c_ticks);
            target.pair_id++;
            arm_ik_now(arm, &target);
            if (!g_b.hand_have_desired) bad++;
            dg_ik_quat_mul(yw, anim, expect_desired);
            if (th_angle_between(g_b.hand_desired, expect_desired) > 0.1) bad++;
            if (i == 0 && th_angle_between(g_b.hand_desired, want25) > 0.05)
                bad++;
        }
        for (k = 0; k < 4; k++) target.hand_quat[k] = ctrl0[k];
        target.stick_yaw_rad = 0.0;
        InterlockedExchange(&g_b.arm_comp, 0);
    }

    /* Leg 2: the body pitches 15 degrees about its own side axis - a
       crouch. Pitch is CARRIED, not compensated: the view input stays
       untouched, the offset and the hand pitch with the torso, the drift
       telemetry stays zero. */
    th_axis(1.0, 0.0, 0.0, 15.0, spin);
    quat_rebase(root_q, spin, turn);
    memcpy(blob, rig0, sizeof blob);
    th_turn_rig(blob + DG_OBJS_ARRAY, STRIDE, JOINTS, turn);
    dg_ik_quat_mul(turn, root_q, live_root);
    InterlockedIncrement(&g_b.c_ticks);
    target.pair_id++;
    arm_ik_now(arm, &target);
    for (k = 0; k < 3; k++) {
        v = InterlockedCompareExchange(&g_b.s_arm_map_target[k], 0, 0);
        memcpy(&f, &v, sizeof f);
        view_now[k] = (double)f;
    }
    for (k = 0; k < 3; k++)
        if (fabs(view_now[k] - base_view[k]) > 0.05) bad++;
    th_pair_offset(live_root, root_pos, offset_now);
    arm_quat_rotate(turn, base_offset, expect_offset);
    for (k = 0; k < 3; k++)
        if (fabs(offset_now[k] - expect_offset[k]) > 0.5) bad++;
    /* The hand pitches with the torso: turn * anim, where the rig's own
       hand went (th_turn_rig turns every bone from the left). */
    dg_ik_quat_mul(turn, anim, expect_desired);
    if (th_angle_between(g_b.hand_desired, expect_desired) > 0.1) bad++;
    v = InterlockedCompareExchange(&g_b.s_arm_body_drift, 0, 0);
    memcpy(&f, &v, sizeof f);
    if (fabs((double)f) > 0.05) bad++;
    if (g_b.c_arm_body_uncompensated != uncomp) bad++;

    /* Leg 3: no captured root. Fail open, loudly: the pair still runs - the
       arm was usable for twenty hours before any of this existed - but the
       input goes through unrotated and the counter says so. */
    th_axis(0.0, 1.0, 0.0, 25.0, spin);
    quat_rebase(root_q, spin, turn);
    memcpy(blob, rig0, sizeof blob);
    th_turn_rig(blob + DG_OBJS_ARRAY, STRIDE, JOINTS, turn);
    dg_ik_quat_mul(turn, root_q, live_root);
    g_b.arm_root_have_q0 = 0;
    InterlockedIncrement(&g_b.c_ticks);
    target.pair_id++;
    arm_ik_now(arm, &target);
    if (g_b.c_arm_body_uncompensated != uncomp + 1) bad++;
    th_pair_offset(live_root, root_pos, offset_now);
    tmp[0] = 0.0;
    for (k = 0; k < 3; k++)
        tmp[0] += (offset_now[k] - base_offset[k]) *
                  (offset_now[k] - base_offset[k]);
    if (!(sqrt(tmp[0]) > 50.0)) bad++;
    g_b.arm_root_have_q0 = 1;

    /* Leg 4: a rest recapture after a gap (the body turned 25, the stick
       says 25, organic). The root zero moves to the turned body, and the
       stick/head zeros must move with it: the pair after the recapture
       then compensates nothing and leaves the offset where the turned body
       put it, with the drift telemetry reading 0. A recapture that
       re-zeroed the root alone would read drift 0 against software 25 and
       turn the input the wrong way by 25. */
    {
        LONG rc0 = g_b.c_arm_rest_recaptured, ns0 = g_b.c_arm_comp_no_stick;
        InterlockedExchange(&g_b.arm_comp, 1);
        target.stick_yaw_rad = 25.0 * 3.14159265358979323846 / 180.0;
        target.head_yaw_valid = 1;
        target.head_yaw_rad = 25.0 * 3.14159265358979323846 / 180.0;
        InterlockedExchange(&g_b.hand_rest_stale, 1);
        InterlockedIncrement(&g_b.c_ticks);
        target.pair_id++;
        arm_ik_now(arm, &target);
        if (g_b.c_arm_rest_recaptured != rc0 + 1) bad++;
        if (!g_b.stick_yaw0_have || fabs(g_b.stick_yaw0_deg - 25.0) > 0.01)
            bad++;
        if (!g_b.head_yaw0_have || fabs(g_b.head_yaw0_deg - 25.0) > 0.01)
            bad++;
        InterlockedIncrement(&g_b.c_ticks);
        target.pair_id++;
        arm_ik_now(arm, &target);
        v = InterlockedCompareExchange(&g_b.s_arm_body_drift, 0, 0);
        memcpy(&f, &v, sizeof f);
        if (fabs((double)f) > 0.05) bad++;
        if (g_b.c_arm_comp_no_stick != ns0) bad++;
        th_pair_offset(live_root, root_pos, offset_now);
        arm_quat_rotate(turn, base_offset, expect_offset);
        for (k = 0; k < 3; k++)
            if (fabs(offset_now[k] - expect_offset[k]) > 0.5) bad++;
        InterlockedExchange(&g_b.arm_comp, 0);
        target.stick_yaw_rad = 0.0;
        target.head_yaw_valid = 0;
    }

    arm_map_forget();
    InterlockedExchange(&g_b.arm_hand_basis, saved_basis);
    if (*(ULONGLONG *)(mc + 0x38) != 0) bad++;
    g_b.a.gm_player_arm_body = 0;
    InterlockedExchange(&g_b.arm_anchor, saved_anchor);
    InterlockedExchange(&g_b.skel_stride, 0);
    InterlockedExchange(&g_b.skel_parents_read, 0);
    InterlockedExchange(&g_b.skel_region_end, 0);
    InterlockedExchange(&g_b.c_ticks, 0);
    InterlockedExchange(&g_b.s_arm_body_drift, f2l(0.0f));

    printf("  %-6s body-yaw compensation: a 25 degree body yaw leaves the "
           "player's shoulder offset and hand orientation world-fixed and "
           "reads 25.0 in the drift telemetry, under organic the hand turns "
           "WITH the stick from the left (hand-computed yaw(25)*anim, and "
           "180/180), a 15 degree body pitch carries arm and hand and "
           "leaves the input untouched, a pair with no calibration root "
           "runs uncompensated and is counted, and a rest recapture moves "
           "the stick/head zeros with the root zero\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

/* The walking turn, reproduced at the desk. The three quasi-static legs
   above pass, yet the 2026-08-20 headset runs lock the residual near 180
   with a wandering offset exactly while the body turns during vr_move
   locomotion. So: the same rig, the same frozen player, but the body now
   turns a few degrees EVERY pair while the harness game faithfully applies
   the previous pair's published command (pull, conversion, composition onto
   the turned animation) - the loop the static legs never close. Diagnostic
   first: it prints what it measures, and the assertion is only that the
   instruments produced numbers at all. */
static int t_probe_the_walking_turn(void)
{
    enum { STRIDE = 0x180, JOINTS = 7, TURN_PAIRS = 60 };
    static unsigned char blob[DG_OBJS_ARRAY + JOINTS * STRIDE];
    static unsigned char rig0[DG_OBJS_ARRAY + JOINTS * STRIDE];
    static unsigned char obj[0x40], mc[0x70];
    static float adjust[55 * 4];
    ULONGLONG arm = (ULONGLONG)(ULONG_PTR)obj;
    DG_BRIDGE_ARM_TARGET target;
    float *hand_m;
    double y37[4], x20[4], root_q[4], anim[4], ctrl0[4];
    double spin[4], turn[4], anim_now[4], achieved[4];
    double setpos_prev[4], pulled[3];
    double root_pos[3] = { 40.0, -10.0, 25.0 };
    double sv[3], wv[3], tmp[3];
    double scale;
    double res_max = 0.0, drift_max = 0.0, res_late = 0.0, drift_late = 0.0;
    double probe_q4[4], probe_q5[4], dq_max = 0.0, base_max = 0.0;
    double cmd_max = 0.0;
    int late_n = 0, measured0, published0, lim_frozen = -1;
    short cmd[3];
    LONG saved_anchor, v;
    float f;
    int j, k, p, bad = 0;

    memset(blob, 0, sizeof blob);
    memset(obj, 0, sizeof obj);
    memset(mc, 0, sizeof mc);
    memset(adjust, 0, sizeof adjust);
    adjust[6 * 4 + 3] = 1.0f;
    *(ULONGLONG *)(obj + 0x00) = (ULONGLONG)(ULONG_PTR)blob;
    *(ULONGLONG *)(obj + 0x08) = (ULONGLONG)(ULONG_PTR)mc;
    *(LONG *)(mc + 0x14) = 55;
    *(ULONGLONG *)(mc + 0x48) = (ULONGLONG)(ULONG_PTR)adjust;
    for (j = 0; j < JOINTS; j++) {
        float *m = (float *)(blob + DG_OBJS_ARRAY + j * STRIDE);
        m[0] = m[5] = m[10] = m[15] = 1.0f;
    }
    th_axis(0.0, 1.0, 0.0, 37.0, y37);
    th_axis(1.0, 0.0, 0.0, 8.0, x20);
    dg_ik_quat_mul(y37, x20, root_q);
    th_write_basis((float *)(blob + DG_OBJS_ARRAY), root_q);
    for (k = 0; k < 3; k++)
        ((float *)(blob + DG_OBJS_ARRAY))[12 + k] = (float)root_pos[k];
    ((float *)(blob + DG_OBJS_ARRAY + 3 * STRIDE))[14] = 100.0f;
    ((float *)(blob + DG_OBJS_ARRAY + 5 * STRIDE))[12] = 200.0f;
    hand_m = (float *)(blob + DG_OBJS_ARRAY + 6 * STRIDE);
    th_axis(0.3, 0.2, -0.9, 64.0, anim);
    th_write_basis(hand_m, anim);
    hand_m[12] = 300.0f;
    hand_m[13] = 173.20508f;
    hand_m[14] = 0.0f;
    memcpy(rig0, blob, sizeof blob);

    InterlockedExchange(&g_b.skel_stride, STRIDE);
    InterlockedExchange(&g_b.skel_parents_read, JOINTS);
    InterlockedExchange(&g_b.skel_region_end, (LONG)sizeof blob);
    for (j = 0; j < JOINTS; j++) InterlockedExchange(&g_b.skel_parents[j], 0);
    InterlockedExchange(&g_b.skel_parents[3], 2);
    InterlockedExchange(&g_b.skel_parents[4], 3);
    InterlockedExchange(&g_b.skel_parents[5], 4);
    InterlockedExchange(&g_b.skel_parents[6], 5);
    g_b.a.gm_player_arm_body = (ULONGLONG)(ULONG_PTR)&arm;
    saved_anchor = InterlockedCompareExchange(&g_b.arm_anchor, 0, 0);
    InterlockedExchange(&g_b.arm_anchor, 1);
    InterlockedExchange(&g_b.ik_active, 0);
    InterlockedExchange(&g_b.c_ticks, 0);
    arm_map_forget();

    for (k = 0; k < 3; k++) tmp[k] = 0.0 - root_pos[k];
    arm_quat_unrotate(root_q, tmp, sv);
    tmp[0] = 300.0 - root_pos[0];
    tmp[1] = 173.20508 - root_pos[1];
    tmp[2] = 0.0 - root_pos[2];
    arm_quat_unrotate(root_q, tmp, wv);

    th_axis(0.5, -0.4, 0.76, 23.0, ctrl0);
    memset(&target, 0, sizeof target);
    target.write = 1;
    target.hand_write = 1;
    target.weight = 1.0;
    target.stream_id = 12;
    target.pair_id = 900;
    target.player_reach_view = 600.0;
    target.player_shoulder_view[0] = 60.0;
    target.player_shoulder_view[1] = -80.0;
    target.player_shoulder_view[2] = 30.0;
    for (k = 0; k < 3; k++)
        target.wrist_view[k] = target.player_shoulder_view[k] +
                               (wv[k] - sv[k]);
    for (k = 0; k < 4; k++) target.hand_quat[k] = ctrl0[k];

    arm_ik_now(arm, &target);
    InterlockedExchange(&g_b.c_ticks, DG_ADJ_SETTLE_TICKS + 1);
    target.pair_id++;
    arm_ik_now(arm, &target);
    if (!g_b.arm_root_have_q0 || !g_b.hand_have_rest) bad++;

    InterlockedIncrement(&g_b.c_ticks);
    target.pair_id++;
    arm_ik_now(arm, &target);
    v = InterlockedCompareExchange(&g_b.s_arm_map_scale, 0, 0);
    memcpy(&f, &v, sizeof f);
    scale = (double)f;
    if (!(scale > 0.2) || !(scale < 1.5)) { bad++; scale = 0.5716; }
    for (k = 0; k < 3; k++)
        target.wrist_view[k] = target.player_shoulder_view[k] +
                               (wv[k] - sv[k]) * (0.95 / scale);

    /* Baseline pair, unturned, and its command becomes the first thing the
       game's SetPos holds during the turn. */
    InterlockedIncrement(&g_b.c_ticks);
    target.pair_id++;
    arm_ik_now(arm, &target);
    if (!hand_command_read(cmd, NULL, NULL)) bad++;
    for (k = 0; k < 3; k++) pulled[k] = (double)th_engine_pull((int)cmd[k]);
    set_pos_quat(pulled, setpos_prev);
    measured0 = (int)g_b.c_arm_hand_measured;
    published0 = (int)g_b.c_arm_hand_written;
    /* Earlier categories leave their own stats behind; the base-drift
       assertions below need a clean slate to be deterministic. */
    InterlockedExchange(&g_b.s_arm_base_drift, f2l(0.0f));
    InterlockedExchange(&g_b.s_arm_base_drift_worst, f2l(0.0f));
    InterlockedExchange(&g_b.s_arm_cmd_drift, f2l(0.0f));
    InterlockedExchange(&g_b.s_arm_cmd_drift_worst, f2l(0.0f));

    /* The turn: 3 degrees of body yaw per pair, the player frozen in world,
       the game applying last pair's command onto the freshly turned
       animation exactly as SetPos would. The one extra pair at the end
       holds the turn still and rotates the WRITTEN hand basis alone by 20
       degrees - the base-drift instrument must read back exactly that. */
    for (p = 1; p <= TURN_PAIRS + 1; p++) {
        double yawdeg = 3.0 * (double)((p <= TURN_PAIRS) ? p : TURN_PAIRS);
        th_axis(0.0, 1.0, 0.0, yawdeg, spin);
        quat_rebase(root_q, spin, turn);
        memcpy(blob, rig0, sizeof blob);
        th_turn_rig(blob + DG_OBJS_ARRAY, STRIDE, JOINTS, turn);
        dg_ik_quat_mul(turn, anim, anim_now);
        /* The game's hierarchy carries last pair's q4/q5 in its MATRICES,
           and the pair path removes them again to recover the animation.
           A rig without them makes that removal rotate the recovered rest
           by the inverse of our own adjusts - a feedback that is pure
           harness artifact. So apply them, positions and hand basis both,
           exactly as the engine would. */
        {
            double aq[4], a4w[4], a5w[4], rel3[3], rot3[3];
            float *m4 = (float *)(blob + DG_OBJS_ARRAY + 4 * STRIDE);
            float *m5 = (float *)(blob + DG_OBJS_ARRAY + 5 * STRIDE);
            double p4[3], p5[3], p6[3];
            for (k = 0; k < 4; k++)
                aq[k] = (double)g_b.arm_map_cached_adjust[k];
            if (!adjust_quat_to_world(&ADJ_FRAME_LEGACY, aq, a4w)) { bad++; break; }
            for (k = 0; k < 4; k++)
                aq[k] = (double)g_b.arm_map_cached_adjust[4 + k];
            if (!adjust_quat_to_world(&ADJ_FRAME_LEGACY, aq, a5w)) { bad++; break; }
            for (k = 0; k < 3; k++) {
                p4[k] = (double)m4[12 + k];
                p5[k] = (double)m5[12 + k];
                p6[k] = (double)hand_m[12 + k];
            }
            for (k = 0; k < 3; k++) rel3[k] = p5[k] - p4[k];
            arm_quat_rotate(a4w, rel3, rot3);
            for (k = 0; k < 3; k++) p5[k] = p4[k] + rot3[k];
            for (k = 0; k < 3; k++) rel3[k] = p6[k] - p4[k];
            arm_quat_rotate(a4w, rel3, rot3);
            for (k = 0; k < 3; k++) p6[k] = p4[k] + rot3[k];
            for (k = 0; k < 3; k++) rel3[k] = p6[k] - p5[k];
            arm_quat_rotate(a5w, rel3, rot3);
            for (k = 0; k < 3; k++) p6[k] = p5[k] + rot3[k];
            for (k = 0; k < 3; k++) {
                m5[12 + k] = (float)p5[k];
                hand_m[12 + k] = (float)p6[k];
            }
        }
        if (!th_next_hand(anim_now, setpos_prev, achieved)) { bad++; break; }
        if (p == TURN_PAIRS + 1) {
            double extra[4];
            th_axis(0.0, 1.0, 0.0, 20.0, extra);
            dg_ik_quat_mul(extra, achieved, achieved);
        }
        th_write_basis(hand_m, achieved);
        for (k = 0; k < 4; k++) adjust[6 * 4 + k] = (float)setpos_prev[k];
        InterlockedIncrement(&g_b.c_ticks);
        target.pair_id++;
        arm_ik_now(arm, &target);
        if (hand_command_read(cmd, NULL, NULL)) {
            for (k = 0; k < 3; k++)
                pulled[k] = (double)th_engine_pull((int)cmd[k]);
            set_pos_quat(pulled, setpos_prev);
        }
        /* The onset pair still measures against a pre-turn prediction, so
           the discontinuity of STARTING the turn is excluded; from pair 3 on
           everything must move at turn rate. Joint deltas are tracked in
           adjust space - the thing the hierarchy actually inherits. */
        {
            double q4n[4], q5n[4], d;
            for (k = 0; k < 4; k++) {
                q4n[k] = (double)g_b.arm_map_cached_adjust[k];
                q5n[k] = (double)g_b.arm_map_cached_adjust[4 + k];
            }
            if (p >= 3 && p <= TURN_PAIRS) {
                d = th_angle_between(q4n, probe_q4);
                if (d > dq_max) dq_max = d;
                d = th_angle_between(q5n, probe_q5);
                if (d > dq_max) dq_max = d;
                v = g_b.s_arm_hand_residual;
                memcpy(&f, &v, sizeof f);
                if ((double)f > res_max) res_max = (double)f;
                v = g_b.s_arm_hand_off_drift;
                memcpy(&f, &v, sizeof f);
                if ((double)f > drift_max) drift_max = (double)f;
                v = g_b.s_arm_base_drift;
                memcpy(&f, &v, sizeof f);
                if ((double)f > base_max) base_max = (double)f;
                v = g_b.s_arm_cmd_drift;
                memcpy(&f, &v, sizeof f);
                if ((double)f > cmd_max) cmd_max = (double)f;
            }
            if (p == 2) lim_frozen = (int)g_b.c_arm_hand_limited;
            for (k = 0; k < 4; k++) {
                probe_q4[k] = q4n[k];
                probe_q5[k] = q5n[k];
            }
        }
        if (p > TURN_PAIRS - 10 && p <= TURN_PAIRS) {
            LONG rv = g_b.s_arm_hand_residual;
            float rf;
            memcpy(&rf, &rv, sizeof rf);
            res_late += (double)rf;
            rv = g_b.s_arm_hand_off_drift;
            memcpy(&rf, &rv, sizeof rf);
            drift_late += (double)rf;
            late_n++;
        }
    }
    if (late_n > 0) { res_late /= late_n; drift_late /= late_n; }
    if ((int)g_b.c_arm_hand_measured - measured0 < TURN_PAIRS - 2) bad++;
    if ((int)g_b.c_arm_hand_written - published0 < TURN_PAIRS - 2) bad++;
    /* The teeth. Residual = the one-pair lag, nothing more; the offset walks
       at far below the turn rate; no joint ever jumps faster than a few
       times the turn per pair - the pre-fix probe measured 175 here - and
       the envelope never saturates once the loop is closed faithfully. */
    if (!(res_max < 6.0)) bad++;
    if (!(drift_max < 2.0)) bad++;
    if (!(dq_max < 20.0)) bad++;
    if (!(base_max < 8.0)) bad++;
    if (lim_frozen < 0 || (int)g_b.c_arm_hand_limited != lim_frozen) bad++;
    if (!g_b.arm_orient_have_prev) bad++;
    if (!g_b.hand_have_prev_twist) bad++;
    /* The swing branch memory must be standing after a driven run, and its
       reference must BE the applied (cap-bounded) angle - an unbounded
       reference is the runaway class all over again. */
    if (!g_b.hand_have_prev_swing) bad++;
    if (fabs(g_b.hand_prev_swing_rad) > DG_HAND_WRIST_SWING_RAD + 1e-9)
        bad++;
    /* The controlled jump: the last pair turned the written basis alone by
       20 degrees, and both the current reading and the run's worst must say
       so - the loop itself never exceeded the turn-rate scale. The command
       stream compensates that same jump, so its drift instrument must see
       it too, and must have been calm through the plain turn. */
    {
        float bd;
        v = g_b.s_arm_base_drift;
        memcpy(&bd, &v, sizeof bd);
        if (fabs((double)bd - 20.0) > 1.5) bad++;
        v = g_b.s_arm_base_drift_worst;
        memcpy(&bd, &v, sizeof bd);
        if (bd < 19.0f) bad++;
        if (!(cmd_max < 15.0)) bad++;
        v = g_b.s_arm_cmd_drift;
        memcpy(&bd, &v, sizeof bd);
        /* The command answers a 20-degree basis jump partly through the
           wrist split, so ~10 is what actually leaves; the window pins
           that it reacts at all and not wildly. */
        if ((double)bd < 5.0 || (double)bd > 30.0) bad++;
        v = g_b.s_arm_cmd_drift_worst;
        memcpy(&bd, &v, sizeof bd);
        if ((double)bd < 5.0) bad++;
    }

    /* The remembered twist must BE the last APPLIED one - fore plus wrist,
       bounded by the caps - or the unwrap is anchored to garbage: a raw
       reference winds up whole turns and pins the clamps forever. */
    {
        float ft, wt;
        v = g_b.s_arm_hand_fore_twist;
        memcpy(&ft, &v, sizeof ft);
        v = g_b.s_arm_hand_wrist_twist;
        memcpy(&wt, &v, sizeof wt);
        if (fabs(g_b.hand_prev_twist_rad * 180.0 / 3.14159265358979323846 -
                 ((double)ft + (double)wt)) > 0.01) bad++;
    }

    /* One deliberately CLAMPED stabilize, straight in: a 190-degree twist
       demand applies as the capped 145, and the branch memory must hold
       the APPLIED sum - a raw reference here reads 170 and is the runaway
       that latched the clamps on 2026-08-20. Runs AFTER the stats
       consistency check above: this direct call bypasses arm_ik_now, so
       the fore/wrist stat slots deliberately stay at the loop's values. */
    {
        double cl[5][3];
        double a4i[4] = { 0.0, 0.0, 0.0, 1.0 };
        double a5i[4] = { 0.0, 0.0, 0.0, 1.0 };
        double twq[4], swq[4], wq[4], handl[4], achl[4], stored_deg;
        DG_IK_HAND_STABILIZE_OUT rep;
        memset(cl, 0, sizeof cl);
        cl[3][0] = 200.0;
        cl[4][0] = 400.0;                    /* forearm along +X */
        spin[0] = sin(0.5 * 190.0 * 3.14159265358979323846 / 180.0);
        spin[1] = 0.0;
        spin[2] = 0.0;
        spin[3] = cos(0.5 * 190.0 * 3.14159265358979323846 / 180.0);
        for (k = 0; k < 4; k++) twq[k] = spin[k];
        th_axis(0.0, 1.0, 0.0, 10.0, swq);
        dg_ik_quat_mul(twq, swq, wq);
        world_quat_to_adjust(&ADJ_FRAME_LEGACY, wq, handl);
        for (k = 0; k < 4; k++) achl[k] = wq[k];
        g_b.hand_have_prev_twist = 0;
        g_b.hand_have_prev_swing = 0;
        if (!arm_hand_stabilize(cl, a4i, a5i, handl, achl, 0.0, &rep))
            bad++;
        stored_deg = g_b.hand_prev_twist_rad * 180.0 /
                     3.14159265358979323846;
        if (fabs(fabs(stored_deg) - 145.0) > 0.01) bad++;
    }

    /* And the swing seam, straight in like the twist above: a pure swing
       walked 170 -> 190 about the same axis must keep the clamped wrist on
       that axis (memoryless, the short arc flips to -Y at 180 and the
       applied 70 teleports 140 degrees - the 2026-08-21 body-turn flap),
       and a demand home at -30 must fold straight back through the bounded
       applied reference instead of latching at the cap. */
    {
        double cl[5][3];
        double a4s[4], a5s[4], swq[4], handl[4], achl[4];
        DG_IK_HAND_STABILIZE_OUT rep;
        int leg;
        /* After the fold-back at -30 the walk climbs again, past the cap
           and across the seam: 200 is a name 40 longer than its short arc
           -160 and outside the 70 cap, so it is renamed (2026-09-03) and
           the applied crosses to -70; 260 is then -100 on that branch and
           the applied stays -70 - the raw reference is wired through and
           walks the renamed branch. */
        double legs_deg[7] = { 170.0, 190.0, -30.0, 40.0, 120.0, 200.0,
                               260.0 };
        memset(cl, 0, sizeof cl);
        cl[3][0] = 200.0;
        cl[4][0] = 400.0;                    /* forearm along +X */
        g_b.hand_have_prev_twist = 0;
        g_b.hand_have_prev_swing = 0;
        for (leg = 0; leg < 7; leg++) {
            a4s[0] = 0.0; a4s[1] = 0.0; a4s[2] = 0.0; a4s[3] = 1.0;
            a5s[0] = 0.0; a5s[1] = 0.0; a5s[2] = 0.0; a5s[3] = 1.0;
            th_axis(0.0, 1.0, 0.0, legs_deg[leg], swq);
            world_quat_to_adjust(&ADJ_FRAME_LEGACY, swq, handl);
            for (k = 0; k < 4; k++) achl[k] = swq[k];
            if (!arm_hand_stabilize(cl, a4s, a5s, handl, achl, 0.0, &rep))
                bad++;
            if (leg == 0 &&
                fabs(rep.wrist_swing_rad * 180.0 /
                     3.14159265358979323846 - 70.0) > 0.01) bad++;
            if (leg == 1) {
                if (fabs(rep.raw_swing_rad * 180.0 /
                         3.14159265358979323846 - 190.0) > 0.01) bad++;
                if (rep.wrist_swing_axis[1] < 0.999) bad++;
            }
            if (leg == 2 &&
                fabs(rep.wrist_swing_rad * 180.0 /
                     3.14159265358979323846 + 30.0) > 0.01) bad++;
            if (leg == 5) {
                if (fabs(rep.raw_swing_rad * 180.0 /
                         3.14159265358979323846 + 160.0) > 0.01) bad++;
                if (fabs(rep.wrist_swing_rad * 180.0 /
                         3.14159265358979323846 + 70.0) > 0.01) bad++;
            }
            if (leg == 6) {
                if (fabs(rep.raw_swing_rad * 180.0 /
                         3.14159265358979323846 + 100.0) > 0.01) bad++;
                if (fabs(rep.wrist_swing_rad * 180.0 /
                         3.14159265358979323846 + 70.0) > 0.01) bad++;
                if (fabs(g_b.hand_prev_swing_raw_rad * 180.0 /
                         3.14159265358979323846 + 100.0) > 0.01) bad++;
            }
        }
    }

    /* The envelope is a KNOB, and the whole point of the knob is that a
       raised cap actually reaches the solver: the same 190-degree demand
       that applies as 145 by default must apply as 190 when the marker
       says 135,55. The arm's entire roll authority runs through here, so
       a knob that silently kept the default would be the defect it exists
       to fix. Restored to the defaults before the legs below. */
    {
        double cl[5][3];
        double a4s[4], a5s[4], twq[4], swq[4], wq[4], handl[4], achl[4];
        DG_IK_HAND_STABILIZE_OUT rep;
        double half = 0.5 * 190.0 * 3.14159265358979323846 / 180.0;
        memset(cl, 0, sizeof cl);
        cl[3][0] = 200.0;
        cl[4][0] = 400.0;
        a4s[0] = 0.0; a4s[1] = 0.0; a4s[2] = 0.0; a4s[3] = 1.0;
        a5s[0] = 0.0; a5s[1] = 0.0; a5s[2] = 0.0; a5s[3] = 1.0;
        twq[0] = sin(half); twq[1] = 0.0; twq[2] = 0.0; twq[3] = cos(half);
        th_axis(0.0, 1.0, 0.0, 10.0, swq);
        dg_ik_quat_mul(twq, swq, wq);
        world_quat_to_adjust(&ADJ_FRAME_LEGACY, wq, handl);
        for (k = 0; k < 4; k++) achl[k] = wq[k];
        InterlockedExchange(&g_b.hand_fore_twist_mdeg, 135000);
        InterlockedExchange(&g_b.hand_wrist_twist_mdeg, 55000);
        g_b.hand_have_prev_twist = 0;
        g_b.hand_have_prev_swing = 0;
        if (!arm_hand_stabilize(cl, a4s, a5s, handl, achl, 0.0, &rep))
            bad++;
        /* Memoryless, so the short arc names this -170: the raised cap
           takes 135 of it and the wrist the remaining 35. */
        if (fabs(rep.fore_twist_rad * 180.0 /
                 3.14159265358979323846 + 135.0) > 0.01) bad++;
        if (fabs(rep.wrist_twist_rad * 180.0 /
                 3.14159265358979323846 + 35.0) > 0.01) bad++;
        /* And a zero slot is the built-in default, never a zero envelope:
           a configure that never ran must not pin every hand at nothing.
           The inputs are rebuilt because arm_hand_stabilize writes its
           arguments back - feeding the first call's outputs in again would
           ask a different question and quietly pass. */
        a4s[0] = 0.0; a4s[1] = 0.0; a4s[2] = 0.0; a4s[3] = 1.0;
        a5s[0] = 0.0; a5s[1] = 0.0; a5s[2] = 0.0; a5s[3] = 1.0;
        world_quat_to_adjust(&ADJ_FRAME_LEGACY, wq, handl);
        for (k = 0; k < 4; k++) achl[k] = wq[k];
        InterlockedExchange(&g_b.hand_fore_twist_mdeg, 0);
        InterlockedExchange(&g_b.hand_wrist_twist_mdeg, 0);
        g_b.hand_have_prev_twist = 0;
        g_b.hand_have_prev_swing = 0;
        if (!arm_hand_stabilize(cl, a4s, a5s, handl, achl, 0.0, &rep))
            bad++;
        if (fabs(rep.fore_twist_rad * 180.0 /
                 3.14159265358979323846 + 90.0) > 0.01) bad++;
        if (fabs(rep.wrist_twist_rad * 180.0 /
                 3.14159265358979323846 + 55.0) > 0.01) bad++;
    }

    /* And the twist rename through the same wrapper: a demand walking
       -100, -150 keeps its names (the second is outside the 145 cap but
       IS the short arc); -200 is outside the cap and 40 longer than its
       short arc 160, so it is renamed and the applied crosses to +145;
       from there -250, -300, -326 are 110, 60, 34 and apply exactly - the
       branch this leaves in the raw reference is 34, never the wound -326
       that pinned the run-11 hand (2026-09-03). A demand of -60 then
       applies exactly. */
    {
        double cl[5][3];
        double a4s[4], a5s[4], twq[4], swq[4], wq[4], handl[4], achl[4];
        DG_IK_HAND_STABILIZE_OUT rep;
        int leg;
        double legs_deg[7] = { -100.0, -150.0, -200.0, -250.0, -300.0,
                               -326.0, -60.0 };
        memset(cl, 0, sizeof cl);
        cl[3][0] = 200.0;
        cl[4][0] = 400.0;                    /* forearm along +X */
        g_b.hand_have_prev_twist = 0;
        g_b.hand_have_prev_swing = 0;
        for (leg = 0; leg < 7; leg++) {
            double half = 0.5 * legs_deg[leg] *
                          3.14159265358979323846 / 180.0;
            a4s[0] = 0.0; a4s[1] = 0.0; a4s[2] = 0.0; a4s[3] = 1.0;
            a5s[0] = 0.0; a5s[1] = 0.0; a5s[2] = 0.0; a5s[3] = 1.0;
            twq[0] = sin(half); twq[1] = 0.0; twq[2] = 0.0;
            twq[3] = cos(half);
            th_axis(0.0, 1.0, 0.0, 10.0, swq);
            dg_ik_quat_mul(twq, swq, wq);
            world_quat_to_adjust(&ADJ_FRAME_LEGACY, wq, handl);
            for (k = 0; k < 4; k++) achl[k] = wq[k];
            if (!arm_hand_stabilize(cl, a4s, a5s, handl, achl, 0.0, &rep))
                bad++;
            if (leg == 1 &&
                fabs(rep.raw_twist_rad * 180.0 /
                     3.14159265358979323846 + 150.0) > 0.01) bad++;
            if (leg == 2) {
                if (fabs(rep.raw_twist_rad * 180.0 /
                         3.14159265358979323846 - 160.0) > 0.01) bad++;
                if (fabs(g_b.hand_prev_twist_rad * 180.0 /
                         3.14159265358979323846 - 145.0) > 0.01) bad++;
            }
            if (leg == 5) {
                if (fabs(rep.raw_twist_rad * 180.0 /
                         3.14159265358979323846 - 34.0) > 0.01) bad++;
                if (fabs(g_b.hand_prev_twist_rad * 180.0 /
                         3.14159265358979323846 - 34.0) > 0.01) bad++;
                if (fabs(g_b.hand_prev_twist_raw_rad * 180.0 /
                         3.14159265358979323846 - 34.0) > 0.01) bad++;
            }
            if (leg == 6 &&
                fabs(g_b.hand_prev_twist_rad * 180.0 /
                     3.14159265358979323846 + 60.0) > 0.01) bad++;
        }
    }

    arm_map_forget();
    g_b.a.gm_player_arm_body = 0;
    InterlockedExchange(&g_b.arm_anchor, saved_anchor);
    InterlockedExchange(&g_b.skel_stride, 0);
    InterlockedExchange(&g_b.skel_parents_read, 0);
    InterlockedExchange(&g_b.skel_region_end, 0);
    InterlockedExchange(&g_b.c_ticks, 0);
    InterlockedExchange(&g_b.s_arm_body_drift, f2l(0.0f));

    printf("  %-6s walking turn: 3 deg of body yaw per pair for %d pairs "
           "with the game loop closed faithfully - residual stays the "
           "one-pair lag (max %.2f), the offset walks at %.2f deg max, no "
           "joint jumps past %.2f deg per pair (the memoryless solve "
           "measured 175 here), and the wrist envelope never saturates\n",
           bad ? "FAIL" : "ok", TURN_PAIRS, res_max, drift_max, dq_max);
    return bad ? 1 : 0;
}

static int t_arm_mapping_is_once_per_pair(void)
{
    enum { STRIDE = 0x180, JOINTS = 7, FEEDBACK_PAIRS = 256 };
    static unsigned char blob[DG_OBJS_ARRAY + JOINTS * STRIDE];
    static unsigned char obj[0x40], mc[0x70];
    static float adjust[55 * 4];
    ULONGLONG arm = (ULONGLONG)(ULONG_PTR)obj;
    DG_BRIDGE_ARM_TARGET target;
    float feedback_ref[8];
    const double clean_upper[3] = { 200.0, 0.0, 0.0 };
    const double clean_fore[3] = { 100.0, 173.20508, 0.0 };
    double a4[4], a5[4], wq4[4], wq5[4];
    double live_upper[3], inherited_fore[3], live_fore[3];
    int j, k, bad = 0;

    memset(blob, 0, sizeof blob);
    memset(obj, 0, sizeof obj);
    memset(mc, 0, sizeof mc);
    memset(adjust, 0, sizeof adjust);
    *(ULONGLONG *)(obj + 0x00) = (ULONGLONG)(ULONG_PTR)blob;
    *(ULONGLONG *)(obj + 0x08) = (ULONGLONG)(ULONG_PTR)mc;
    *(LONG *)(mc + 0x14) = 55;
    *(ULONGLONG *)(mc + 0x48) = (ULONGLONG)(ULONG_PTR)adjust;
    for (j = 0; j < JOINTS; j++) {
        float *m = (float *)(blob + DG_OBJS_ARRAY + j * STRIDE);
        m[0] = m[5] = m[10] = m[15] = 1.0f;
    }
    /* 2->3 is the outward pole basis; 4->5->6 is a 200/200 arm whose
       animated wrist is bent inside the mapper's deliberate 99% ceiling. */
    ((float *)(blob + DG_OBJS_ARRAY + 3 * STRIDE))[14] = 100.0f;
    ((float *)(blob + DG_OBJS_ARRAY + 5 * STRIDE))[12] = 200.0f;
    ((float *)(blob + DG_OBJS_ARRAY + 6 * STRIDE))[12] = 300.0f;
    ((float *)(blob + DG_OBJS_ARRAY + 6 * STRIDE))[13] = 173.20508f;

    InterlockedExchange(&g_b.skel_stride, STRIDE);
    InterlockedExchange(&g_b.skel_parents_read, JOINTS);
    InterlockedExchange(&g_b.skel_region_end, (LONG)sizeof blob);
    for (j = 0; j < JOINTS; j++) InterlockedExchange(&g_b.skel_parents[j], 0);
    InterlockedExchange(&g_b.skel_parents[3], 2);
    InterlockedExchange(&g_b.skel_parents[4], 3);
    InterlockedExchange(&g_b.skel_parents[5], 4);
    InterlockedExchange(&g_b.skel_parents[6], 5);
    g_b.a.gm_player_arm_body = (ULONGLONG)(ULONG_PTR)&arm;
    InterlockedExchange(&g_b.ik_active, 0);
    InterlockedExchange(&g_b.c_ticks, 0);
    InterlockedExchange(&g_b.c_arm_pairs_seen, 0);
    InterlockedExchange(&g_b.c_arm_pairs_eligible, 0);
    InterlockedExchange(&g_b.c_arm_pairs_accepted, 0);
    InterlockedExchange(&g_b.c_arm_pairs_refused, 0);
    InterlockedExchange(&g_b.c_arm_pairs_calibration, 0);
    InterlockedExchange(&g_b.c_arm_pair_replays, 0);
    arm_map_forget();

    memset(&target, 0, sizeof target);
    target.write = 1;
    target.weight = 1.0;
    target.stream_id = 7;
    target.pair_id = 100;
    target.wrist_view[0] = 770.0;
    target.hand_quat[3] = 1.0;

    /* Identity acquisition and all extra seams of that first pair are blank. */
    arm_ik_now(arm, &target);
    arm_ik_now(arm, &target);
    if (*(ULONGLONG *)(mc + 0x38) != 0) bad++;

    InterlockedExchange(&g_b.c_ticks, 1);
    target.pair_id++;
    arm_ik_now(arm, &target);
    if (*(ULONGLONG *)(mc + 0x38) != 0) bad++;

    /* After two clean ticks, one whole pair captures calibration and remains
       blank. Only the following pair may write. */
    InterlockedExchange(&g_b.c_ticks, DG_ADJ_SETTLE_TICKS + 1);
    target.pair_id++;
    arm_ik_now(arm, &target);
    arm_ik_now(arm, &target);
    if (*(ULONGLONG *)(mc + 0x38) != 0) bad++;
    target.pair_id++;
    arm_ik_now(arm, &target);
    if (*(ULONGLONG *)(mc + 0x38) != ((1ULL << 4) | (1ULL << 5))) bad++;
    if (g_b.c_arm_pairs_seen != 4 || g_b.c_arm_pairs_eligible != 1 ||
        g_b.c_arm_pairs_accepted != 1 || g_b.c_arm_pairs_refused != 0 ||
        g_b.c_arm_pairs_calibration != 3) bad++;

    /* Model the next hierarchy pass exactly as the runtime seam sees it: the
       fixed animation directions inherit q4, and the child forearm then q5.
       Across 256 fresh pair ids a fixed controller must keep producing the
       same float quaternions. Reading those adjusted matrices as a new rest
       pose makes the old implementation roll a little farther every pair. */
    for (j = 0; j < FEEDBACK_PAIRS; j++) {
        float *elbow = (float *)(blob + DG_OBJS_ARRAY + 5 * STRIDE);
        float *wrist = (float *)(blob + DG_OBJS_ARRAY + 6 * STRIDE);
        for (k = 0; k < 4; k++) {
            a4[k] = (double)g_b.arm_map_cached_adjust[k];
            a5[k] = (double)g_b.arm_map_cached_adjust[4 + k];
        }
        if (!adjust_quat_to_world(&ADJ_FRAME_LEGACY, a4, wq4) ||
            !adjust_quat_to_world(&ADJ_FRAME_LEGACY, a5, wq5)) {
            bad++;
            break;
        }
        arm_quat_rotate(wq4, clean_upper, live_upper);
        arm_quat_rotate(wq4, clean_fore, inherited_fore);
        arm_quat_rotate(wq5, inherited_fore, live_fore);
        for (k = 0; k < 3; k++) {
            elbow[12 + k] = (float)live_upper[k];
            wrist[12 + k] = (float)(live_upper[k] + live_fore[k]);
        }
        InterlockedIncrement(&g_b.c_ticks);
        target.pair_id++;
        arm_ik_now(arm, &target);
        if (!g_b.arm_map_cache_valid) {
            bad++;
            break;
        }
        if (j == 0) {
            memcpy(feedback_ref, g_b.arm_map_cached_adjust,
                   sizeof feedback_ref);
        } else {
            for (k = 0; k < 8; k++)
                if (fabs((double)g_b.arm_map_cached_adjust[k] -
                         (double)feedback_ref[k]) > 1e-5) bad++;
        }
    }

    /* Destroy the output and move the live wrist. A repeat of the same pair
       must replay the cached quaternions byte-for-byte, not solve again from
       the now different hierarchy. */
    memset(adjust + 4 * 4, 0, 8 * sizeof(float));
    ((float *)(blob + DG_OBJS_ARRAY + 6 * STRIDE))[12] = -900.0f;
    arm_ik_now(arm, &target);
    for (j = 0; j < 2; j++)
        for (k = 0; k < 4; k++)
            if (adjust[(j + 4) * 4 + k] !=
                g_b.arm_map_cached_adjust[j * 4 + k]) bad++;
    if (g_b.c_arm_pairs_seen != 4 + FEEDBACK_PAIRS ||
        g_b.c_arm_pairs_accepted != 1 + FEEDBACK_PAIRS ||
        g_b.c_arm_pair_replays != 3) bad++;

    /* Restore the owner identity before release; the moved joint is never read
       on this path, and release clears only the exact object we wrote. */
    arm_map_forget();
    if (*(ULONGLONG *)(mc + 0x38) != 0) bad++;
    g_b.a.gm_player_arm_body = 0;
    InterlockedExchange(&g_b.skel_stride, 0);
    InterlockedExchange(&g_b.skel_parents_read, 0);
    InterlockedExchange(&g_b.skel_region_end, 0);
    InterlockedExchange(&g_b.c_ticks, 0);

    printf("  %-6s arm map lifecycle: two-tick settle and calibration suppress "
           "whole pairs, one solve is replayed byte-identically across three "
           "extra seams, and 256 adjusted hierarchy passes do not roll\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

static int t_arm_freeze_replays_one_pair_and_never_solves(void)
{
    enum { STRIDE = 0x180, JOINTS = 7, FREEZE_PAIRS = 256 };
    static unsigned char blob[DG_OBJS_ARRAY + JOINTS * STRIDE];
    static unsigned char obj[0x40], mc[0x70];
    static float adjust[55 * 4];
    ULONGLONG arm = (ULONGLONG)(ULONG_PTR)obj;
    DG_BRIDGE_ARM_TARGET target;
    float frozen[8];
    LONG accepted_at_freeze;
    int j, k, bad = 0;

    /* The same synthetic hierarchy the mapping test uses: a 200/200 arm on
       a 7-joint skeleton with an outward pole. */
    memset(blob, 0, sizeof blob);
    memset(obj, 0, sizeof obj);
    memset(mc, 0, sizeof mc);
    memset(adjust, 0, sizeof adjust);
    *(ULONGLONG *)(obj + 0x00) = (ULONGLONG)(ULONG_PTR)blob;
    *(ULONGLONG *)(obj + 0x08) = (ULONGLONG)(ULONG_PTR)mc;
    *(LONG *)(mc + 0x14) = 55;
    *(ULONGLONG *)(mc + 0x48) = (ULONGLONG)(ULONG_PTR)adjust;
    for (j = 0; j < JOINTS; j++) {
        float *m = (float *)(blob + DG_OBJS_ARRAY + j * STRIDE);
        m[0] = m[5] = m[10] = m[15] = 1.0f;
    }
    ((float *)(blob + DG_OBJS_ARRAY + 3 * STRIDE))[14] = 100.0f;
    ((float *)(blob + DG_OBJS_ARRAY + 5 * STRIDE))[12] = 200.0f;
    ((float *)(blob + DG_OBJS_ARRAY + 6 * STRIDE))[12] = 300.0f;
    ((float *)(blob + DG_OBJS_ARRAY + 6 * STRIDE))[13] = 173.20508f;

    InterlockedExchange(&g_b.skel_stride, STRIDE);
    InterlockedExchange(&g_b.skel_parents_read, JOINTS);
    InterlockedExchange(&g_b.skel_region_end, (LONG)sizeof blob);
    for (j = 0; j < JOINTS; j++) InterlockedExchange(&g_b.skel_parents[j], 0);
    InterlockedExchange(&g_b.skel_parents[3], 2);
    InterlockedExchange(&g_b.skel_parents[4], 3);
    InterlockedExchange(&g_b.skel_parents[5], 4);
    InterlockedExchange(&g_b.skel_parents[6], 5);
    g_b.a.gm_player_arm_body = (ULONGLONG)(ULONG_PTR)&arm;
    InterlockedExchange(&g_b.ik_active, 0);
    InterlockedExchange(&g_b.c_ticks, 0);
    InterlockedExchange(&g_b.c_arm_pairs_accepted, 0);
    InterlockedExchange(&g_b.c_arm_pairs_frozen, 0);
    InterlockedExchange(&g_b.arm_freeze, 0);
    arm_map_forget();

    memset(&target, 0, sizeof target);
    target.write = 1;
    target.weight = 1.0;
    target.stream_id = 7;
    target.pair_id = 100;
    target.wrist_view[0] = 770.0;
    target.hand_quat[3] = 1.0;

    /* The marker is on from the very first pair, deliberately: a freeze
       that swallowed acquisition would leave the arm dead and look like a
       success. Settle and calibration still have to run to completion, so
       drive pairs until one is accepted rather than hard-coding how many
       that takes - that count belongs to the mapping test, not this one. */
    InterlockedExchange(&g_b.arm_freeze, 1);
    for (j = 0; j < 16 && !g_b.c_arm_pairs_accepted; j++) {
        InterlockedExchange(&g_b.c_ticks,
                            (LONG)(DG_ADJ_SETTLE_TICKS + 1 + j));
        arm_ik_now(arm, &target);
        target.pair_id++;
    }
    if (*(ULONGLONG *)(mc + 0x38) != ((1ULL << 4) | (1ULL << 5))) bad++;
    if (g_b.c_arm_pairs_accepted != 1) bad++;
    if (g_b.c_arm_pairs_frozen != 0) bad++;   /* nothing to replay yet */
    memcpy(frozen, g_b.arm_map_cached_adjust, sizeof frozen);
    accepted_at_freeze = g_b.c_arm_pairs_accepted;

    /* Now the experiment. The controller MOVES - a new wrist target every
       pair, sweeping the whole reachable arc - and the hierarchy is
       rewritten under us the way the runtime rewrites it. A frozen arm
       must ignore all of it: identical bytes, no further solve. If this
       test can be made to pass by a freeze that quietly re-solves, the
       live experiment proves nothing, so the accepted counter is checked
       as strictly as the bytes. */
    for (j = 0; j < FREEZE_PAIRS; j++) {
        double a = (double)j * 0.05;
        float *wrist = (float *)(blob + DG_OBJS_ARRAY + 6 * STRIDE);
        target.wrist_view[0] = 300.0 + 400.0 * cos(a);
        target.wrist_view[1] = 400.0 * sin(a);
        target.wrist_view[2] = 100.0 * sin(a * 0.5);
        wrist[12] = (float)(300.0 + 50.0 * cos(a));
        wrist[13] = (float)(173.0 + 50.0 * sin(a));
        InterlockedIncrement(&g_b.c_ticks);
        target.pair_id++;
        arm_ik_now(arm, &target);
        for (k = 0; k < 8; k++)
            if (g_b.arm_map_cached_adjust[k] != frozen[k]) { bad++; break; }
        for (k = 0; k < 4; k++) {
            if (adjust[4 * 4 + k] != frozen[k]) { bad++; break; }
            if (adjust[5 * 4 + k] != frozen[4 + k]) { bad++; break; }
        }
    }
    if (g_b.c_arm_pairs_accepted != accepted_at_freeze) bad++;
    if (g_b.c_arm_pairs_frozen != FREEZE_PAIRS) bad++;
    if (*(ULONGLONG *)(mc + 0x38) != ((1ULL << 4) | (1ULL << 5))) bad++;

    /* And it must not be a trapdoor: switching the marker back off returns
       the next pair to the solver, or an evening ends with a frozen arm
       and no idea why. */
    ((float *)(blob + DG_OBJS_ARRAY + 5 * STRIDE))[12] = 200.0f;
    ((float *)(blob + DG_OBJS_ARRAY + 6 * STRIDE))[12] = 300.0f;
    ((float *)(blob + DG_OBJS_ARRAY + 6 * STRIDE))[13] = 173.20508f;
    InterlockedExchange(&g_b.arm_freeze, 0);
    target.wrist_view[0] = 770.0;
    target.wrist_view[1] = 0.0;
    target.wrist_view[2] = 0.0;
    for (j = 0; j < 8 &&
                g_b.c_arm_pairs_accepted == accepted_at_freeze; j++) {
        InterlockedIncrement(&g_b.c_ticks);
        target.pair_id++;
        arm_ik_now(arm, &target);
    }
    if (g_b.c_arm_pairs_accepted != accepted_at_freeze + 1) bad++;

    arm_map_forget();
    g_b.a.gm_player_arm_body = 0;
    InterlockedExchange(&g_b.skel_stride, 0);
    InterlockedExchange(&g_b.skel_parents_read, 0);
    InterlockedExchange(&g_b.skel_region_end, 0);
    InterlockedExchange(&g_b.c_ticks, 0);
    InterlockedExchange(&g_b.c_arm_pairs_frozen, 0);

    printf("  %-6s arm freeze (constant-replay): one accepted pair is written "
           "byte-identically across %d pairs while the controller sweeps and "
           "the hierarchy moves, no further solve runs, and clearing the "
           "marker hands the very next pair back to the solver\n",
           bad ? "FAIL" : "ok", FREEZE_PAIRS);
    return bad ? 1 : 0;
}

/* The instrument the freeze experiment will be read through, so it gets a
   test of its own. The accumulator beside it sums absolute per-frame steps
   and therefore cannot tell a hand that WINDS from a hand that merely
   shakes - and winding is the whole symptom: fifteen revolutions in
   sixty-seven seconds arrived at well under a degree per frame, invisible
   to every jump-hunting number in this file. A meter that cannot separate
   those two would send the next evening the same way as the last five. */
static int t_hand_net_meter_separates_winding_from_shaking(void)
{
    enum { STRIDE = 0x180, JOINTS = 7, TURN_STEPS = 36, SHAKE_STEPS = 72 };
    static unsigned char blob[DG_OBJS_ARRAY + JOINTS * STRIDE];
    static unsigned char obj[0x40], mc[0x70];
    ULONGLONG arm = (ULONGLONG)(ULONG_PTR)obj;
    ULONGLONG objs = (ULONGLONG)(ULONG_PTR)blob;
    float *hand = (float *)(blob + DG_OBJS_ARRAY + 6 * STRIDE);
    double wind_total, wind_max, wind_end, shake_total, shake_max;
    int j, bad = 0;

    memset(blob, 0, sizeof blob);
    memset(obj, 0, sizeof obj);
    memset(mc, 0, sizeof mc);
    *(ULONGLONG *)(obj + 0x00) = objs;
    *(ULONGLONG *)(obj + 0x08) = (ULONGLONG)(ULONG_PTR)mc;
    for (j = 0; j < JOINTS; j++) {
        float *m = (float *)(blob + DG_OBJS_ARRAY + j * STRIDE);
        m[0] = m[5] = m[10] = m[15] = 1.0f;
    }

    /* Discovery is not what is on trial: hand the probe a measured object
       so it goes straight to the live-pose read, with joint 6 - the hand -
       at the head of the chain, exactly where the runtime walk leaves it. */
    InterlockedExchange(&g_b.skel_for_lo, (LONG)(DWORD)objs);
    InterlockedExchange(&g_b.skel_for_hi, (LONG)(DWORD)(objs >> 32));
    InterlockedExchange(&g_b.skel_stride, STRIDE);
    InterlockedExchange(&g_b.skel_region_end, (LONG)sizeof blob);
    InterlockedExchange(&g_b.skel_parents_read, JOINTS);
    InterlockedExchange(&g_b.skel_chain_len, 1);
    InterlockedExchange(&g_b.skel_chain[0], 6);
    InterlockedExchange(&g_b.s_skel_hand_turned, f2l(0.0f));
    InterlockedExchange(&g_b.c_skel_hand_samples, 0);
    g_b.skel_hand_have_prev = 0;
    g_b.skel_hand_have_ref = 0;

    /* One full turn, ten degrees at a time. */
    for (j = 0; j <= TURN_STEPS; j++) {
        double a = (double)j * 10.0 * 3.14159265358979323846 / 180.0;
        hand[0] = (float)cos(a);  hand[1] = (float)-sin(a);
        hand[4] = (float)sin(a);  hand[5] = (float)cos(a);
        hand[10] = 1.0f; hand[15] = 1.0f;
        skel_probe_now(arm, 1);
    }
    {
        LONG v = g_b.s_skel_hand_turned;   float f;
        memcpy(&f, &v, sizeof f); wind_total = (double)f;
        v = g_b.s_skel_hand_net_max; memcpy(&f, &v, sizeof f);
        wind_max = (double)f;
        v = g_b.s_skel_hand_net; memcpy(&f, &v, sizeof f);
        wind_end = (double)f;
    }
    /* A full revolution: 360 degrees of travel, the net angle sweeps out to
       a half turn and comes home. Folded into 0..180, so the tell is the
       MAX, not the endpoint - which is back at the reference and would read
       as perfect stillness on its own. */
    if (fabs(wind_total - 360.0) > 2.0) bad++;
    if (wind_max < 178.0) bad++;
    if (wind_end > 2.0) bad++;

    /* The same amount of travel, none of it net: five degrees each way,
       twice as many steps. The accumulator cannot tell this apart from the
       revolution above; the net meter must. */
    InterlockedExchange(&g_b.s_skel_hand_turned, f2l(0.0f));
    g_b.skel_hand_have_prev = 0;
    g_b.skel_hand_have_ref = 0;
    for (j = 0; j <= SHAKE_STEPS; j++) {
        double a = ((j & 1) ? 5.0 : 0.0) * 3.14159265358979323846 / 180.0;
        hand[0] = (float)cos(a);  hand[1] = (float)-sin(a);
        hand[4] = (float)sin(a);  hand[5] = (float)cos(a);
        hand[10] = 1.0f; hand[15] = 1.0f;
        skel_probe_now(arm, 1);
    }
    {
        LONG v = g_b.s_skel_hand_turned;   float f;
        memcpy(&f, &v, sizeof f); shake_total = (double)f;
        v = g_b.s_skel_hand_net_max; memcpy(&f, &v, sizeof f);
        shake_max = (double)f;
    }
    if (fabs(shake_total - 360.0) > 5.0) bad++;   /* same total variation */
    if (shake_max > 6.0) bad++;                   /* and no net rotation */

    InterlockedExchange(&g_b.skel_for_lo, 0);
    InterlockedExchange(&g_b.skel_for_hi, 0);
    InterlockedExchange(&g_b.skel_stride, 0);
    InterlockedExchange(&g_b.skel_region_end, 0);
    InterlockedExchange(&g_b.skel_parents_read, 0);
    InterlockedExchange(&g_b.skel_chain_len, 0);
    InterlockedExchange(&g_b.s_skel_hand_turned, f2l(0.0f));
    InterlockedExchange(&g_b.c_skel_hand_samples, 0);
    g_b.skel_hand_have_prev = 0;
    g_b.skel_hand_have_ref = 0;

    printf("  %-6s hand net meter: one revolution and %d shakes travel the "
           "same %.0f/%.0f degrees, and only the revolution shows it - net "
           "max %.0f against %.0f, with the wound hand back at %.0f from its "
           "reference so the endpoint alone would have called it still\n",
           bad ? "FAIL" : "ok", SHAKE_STEPS / 2, wind_total, shake_total,
           wind_max, shake_max, wind_end);
    return bad ? 1 : 0;
}

/* The live findings, in order: solving every pair = fifteen revolutions a
   minute; replaying one constant pair = dead still (831 pairs, 2026-08-22).
   So the winding is made by the solve loop, and the named suspect was its
   REST REFERENCE, recovered each pair from the live matrices by stripping
   our own previous write back out of them. This test builds that loop with
   a deliberately imperfect strip - a constant 2-degree roll residue about
   the upper arm every hierarchy pass, in exactly the degree of freedom
   joints 4/5 cannot see - and it DISPROVED the simple version of the
   theory at the desk: the anchored solver absorbs a constant residue once
   and sits still, a fixed point, not a spiral (walk over 90 passes: ~0).
   That is locked in below as a regression bar, because an anchor that
   starts winding under this rig has lost the property that makes it an
   anchor. What the rest freeze still settles LIVE: the live loop has
   moving animation, a moving controller and body compensation that this
   rig does not - if freezing the reference stops the live tumble, the
   recovered-reference channel is implicated under those live conditions;
   if not, that channel is acquitted entirely. Either way the freeze must
   provably (1) keep aiming alive - it is not the constant-replay of step
   1 - (2) actually substitute the reference, visible as the solutions
   parting from the recovered-reference run by about the residue, and
   (3) read the residue on its drift meter. */
static int t_rest_freeze_breaks_the_strip_feedback_loop(void)
{
    enum { STRIDE = 0x180, JOINTS = 7, PASSES = 90 };
    static unsigned char blob[DG_OBJS_ARRAY + JOINTS * STRIDE];
    static unsigned char obj[0x40], mc[0x70];
    static float adjust[55 * 4];
    ULONGLONG arm = (ULONGLONG)(ULONG_PTR)obj;
    DG_BRIDGE_ARM_TARGET target;
    const double clean_upper[3] = { 200.0, 0.0, 0.0 };
    const double clean_fore[3] = { 100.0, 173.20508, 0.0 };
    double first45[8], steady45[2][8], walk_deg[2], aim_deg[2], split_deg;
    float drift_max;
    int mode, j, k, bad = 0;

    memset(obj, 0, sizeof obj);
    memset(mc, 0, sizeof mc);
    *(ULONGLONG *)(obj + 0x00) = (ULONGLONG)(ULONG_PTR)blob;
    *(ULONGLONG *)(obj + 0x08) = (ULONGLONG)(ULONG_PTR)mc;
    *(LONG *)(mc + 0x14) = 55;
    *(ULONGLONG *)(mc + 0x48) = (ULONGLONG)(ULONG_PTR)adjust;

    g_b.a.gm_player_arm_body = (ULONGLONG)(ULONG_PTR)&arm;
    InterlockedExchange(&g_b.skel_stride, STRIDE);
    InterlockedExchange(&g_b.skel_parents_read, JOINTS);
    InterlockedExchange(&g_b.skel_region_end, (LONG)sizeof blob);
    for (j = 0; j < JOINTS; j++) InterlockedExchange(&g_b.skel_parents[j], 0);
    InterlockedExchange(&g_b.skel_parents[3], 2);
    InterlockedExchange(&g_b.skel_parents[4], 3);
    InterlockedExchange(&g_b.skel_parents[5], 4);
    InterlockedExchange(&g_b.skel_parents[6], 5);

    /* Once with the recovered reference, once with the frozen one; the same
       residue both times. */
    for (mode = 0; mode < 2; mode++) {
        int have_first = 0;
        memset(blob, 0, sizeof blob);
        memset(adjust, 0, sizeof adjust);
        for (j = 0; j < JOINTS; j++) {
            float *m = (float *)(blob + DG_OBJS_ARRAY + j * STRIDE);
            m[0] = m[5] = m[10] = m[15] = 1.0f;
        }
        ((float *)(blob + DG_OBJS_ARRAY + 3 * STRIDE))[14] = 100.0f;
        ((float *)(blob + DG_OBJS_ARRAY + 5 * STRIDE))[12] = 200.0f;
        ((float *)(blob + DG_OBJS_ARRAY + 6 * STRIDE))[12] = 300.0f;
        ((float *)(blob + DG_OBJS_ARRAY + 6 * STRIDE))[13] = 173.20508f;

        InterlockedExchange(&g_b.ik_active, 0);
        InterlockedExchange(&g_b.arm_freeze, mode ? 2 : 0);
        InterlockedExchange(&g_b.s_rest_drift, f2l(0.0f));
        InterlockedExchange(&g_b.s_rest_drift_max, f2l(0.0f));
        InterlockedExchange(&g_b.c_ticks, 0);
        arm_map_forget();
        g_b.arm_map_phase = 0;
        g_b.rest_ref_have = 0;

        memset(&target, 0, sizeof target);
        target.write = 1;
        target.weight = 1.0;
        target.stream_id = 7;
        target.pair_id = 100 + mode * 1000;
        target.wrist_view[0] = 770.0;
        target.hand_quat[3] = 1.0;

        for (j = 0; j < 16 && !g_b.arm_map_cache_valid; j++) {
            InterlockedExchange(&g_b.c_ticks,
                                (LONG)(DG_ADJ_SETTLE_TICKS + 1 + j));
            arm_ik_now(arm, &target);
            target.pair_id++;
        }
        if (!g_b.arm_map_cache_valid) { bad++; break; }
        if (mode == 1 && !g_b.rest_ref_have) bad++;

        /* The hierarchy pass, as the runtime performs it, PLUS the
           residue: after inheriting our adjusts the wrist is rolled two
           degrees about the upper arm. Perfect strips were proven
           byte-stable for 256 passes elsewhere; this is the imperfect
           one. */
        for (j = 0; j < PASSES; j++) {
            float *elbow = (float *)(blob + DG_OBJS_ARRAY + 5 * STRIDE);
            float *wrist = (float *)(blob + DG_OBJS_ARRAY + 6 * STRIDE);
            double a4[4], a5[4], wq4[4], wq5[4];
            double live_upper[3], inherited[3], live_fore[3], axis[3];
            double eps[4], rolled[3], half, n;
            for (k = 0; k < 4; k++) {
                a4[k] = (double)g_b.arm_map_cached_adjust[k];
                a5[k] = (double)g_b.arm_map_cached_adjust[4 + k];
            }
            if (!adjust_quat_to_world(&ADJ_FRAME_LEGACY, a4, wq4) ||
                !adjust_quat_to_world(&ADJ_FRAME_LEGACY, a5, wq5)) { bad++; break; }
            arm_quat_rotate(wq4, clean_upper, live_upper);
            arm_quat_rotate(wq4, clean_fore, inherited);
            arm_quat_rotate(wq5, inherited, live_fore);
            n = sqrt(live_upper[0] * live_upper[0] +
                     live_upper[1] * live_upper[1] +
                     live_upper[2] * live_upper[2]);
            if (!(n > 1e-9)) { bad++; break; }
            for (k = 0; k < 3; k++) axis[k] = live_upper[k] / n;
            half = 0.5 * 2.0 * 3.14159265358979323846 / 180.0;
            eps[0] = axis[0] * sin(half);
            eps[1] = axis[1] * sin(half);
            eps[2] = axis[2] * sin(half);
            eps[3] = cos(half);
            arm_quat_rotate(eps, live_fore, rolled);
            for (k = 0; k < 3; k++) {
                elbow[12 + k] = (float)live_upper[k];
                wrist[12 + k] = (float)(live_upper[k] + rolled[k]);
            }
            InterlockedIncrement(&g_b.c_ticks);
            target.pair_id++;
            arm_ik_now(arm, &target);
            if (!g_b.arm_map_cache_valid) { bad++; break; }
            if (!have_first) {
                for (k = 0; k < 8; k++)
                    first45[k] = (double)g_b.arm_map_cached_adjust[k];
                have_first = 1;
            }
        }
        for (k = 0; k < 8; k++)
            steady45[mode][k] = (double)g_b.arm_map_cached_adjust[k];
        if (have_first) {
            double w4 = dg_ik_quat_angle(first45, steady45[mode]) *
                        180.0 / 3.14159265358979323846;
            double w5 = dg_ik_quat_angle(first45 + 4, steady45[mode] + 4) *
                        180.0 / 3.14159265358979323846;
            walk_deg[mode] = (w4 > w5) ? w4 : w5;
        } else {
            walk_deg[mode] = -1.0;
        }

        /* Aiming must stay alive in BOTH runs - the rest freeze is not the
           constant-replay of step 1, and this is also the assert that
           catches a freeze-mode gate regressed back to "any non-zero
           freezes everything". */
        {
            double before[4], after[4];
            for (k = 0; k < 4; k++)
                before[k] = (double)g_b.arm_map_cached_adjust[4 + k];
            target.wrist_view[1] = 300.0;
            InterlockedIncrement(&g_b.c_ticks);
            target.pair_id++;
            arm_ik_now(arm, &target);
            for (k = 0; k < 4; k++)
                after[k] = (double)g_b.arm_map_cached_adjust[4 + k];
            aim_deg[mode] = g_b.arm_map_cache_valid
                ? dg_ik_quat_angle(before, after) *
                  180.0 / 3.14159265358979323846
                : -1.0;
        }
    }
    /* A faded pair with the animation thrown 90 degrees off the frozen
       reference: the maximum must not take it. The runtime's own blends
       (weapon down, 108-111 degrees in the 2026-09-02 runs) otherwise hide
       the strip number this meter exists for. */
    {
        float *elbow = (float *)(blob + DG_OBJS_ARRAY + 5 * STRIDE);
        float *wrist = (float *)(blob + DG_OBJS_ARRAY + 6 * STRIDE);
        elbow[12] = 0.0f; elbow[13] = 200.0f; elbow[14] = 0.0f;
        wrist[12] = 0.0f; wrist[13] = 200.0f; wrist[14] = 200.0f;
        target.weight = 0.5;
        InterlockedIncrement(&g_b.c_ticks);
        target.pair_id++;
        arm_ik_now(arm, &target);
        target.weight = 1.0;
    }
    {
        LONG v = g_b.s_rest_drift_max;
        memcpy(&drift_max, &v, sizeof drift_max);
    }
    {
        double s4 = dg_ik_quat_angle(steady45[0], steady45[1]) *
                    180.0 / 3.14159265358979323846;
        double s5 = dg_ik_quat_angle(steady45[0] + 4, steady45[1] + 4) *
                    180.0 / 3.14159265358979323846;
        split_deg = (s4 > s5) ? s4 : s5;
    }

    /* The regression bar for the desk finding: the anchored solver holds a
       constant residue at a fixed point. If either run walks, the anchor
       has stopped anchoring. */
    if (!(walk_deg[0] >= 0.0 && walk_deg[0] < 5.0)) bad++;
    if (!(walk_deg[1] >= 0.0 && walk_deg[1] < 5.0)) bad++;
    /* The substitution is real: the two steady states differ by about the
       residue - the recovered run absorbed it, the frozen run refused it.
       A rest freeze that quietly kept using the recovered reference makes
       this zero. */
    if (!(split_deg > 0.5 && split_deg < 6.0)) bad++;
    /* Aiming lives in both modes. */
    if (!(aim_deg[0] > 5.0 && aim_deg[1] > 5.0)) bad++;
    /* And the drift meter read the residue it refused to follow. */
    if (!((double)drift_max > 1.0 && (double)drift_max < 6.0)) bad++;

    arm_map_forget();
    InterlockedExchange(&g_b.arm_freeze, 0);
    g_b.rest_ref_have = 0;
    g_b.a.gm_player_arm_body = 0;
    InterlockedExchange(&g_b.skel_stride, 0);
    InterlockedExchange(&g_b.skel_parents_read, 0);
    InterlockedExchange(&g_b.skel_region_end, 0);
    InterlockedExchange(&g_b.c_ticks, 0);
    InterlockedExchange(&g_b.s_rest_drift, f2l(0.0f));
    InterlockedExchange(&g_b.s_rest_drift_max, f2l(0.0f));

    printf("  %-6s rest freeze vs strip residue: a constant 2-degree strip "
           "error does NOT wind the anchored solver (walks %.2f/%.2f deg in "
           "%d passes - the live tumble needs more than this), the frozen "
           "reference visibly parts from the recovered one by %.1f deg, "
           "aiming stays alive in both (%.0f/%.0f deg on a target step), "
           "and the drift meter read the %.1f-degree residue\n",
           bad ? "FAIL" : "ok", walk_deg[0], walk_deg[1], PASSES, split_deg,
           aim_deg[0], aim_deg[1], (double)drift_max);
    return bad ? 1 : 0;
}

/* The aim gap is what turns Snake's body toward where the player FACES.
   Two publishers died before this one. The hand-path publisher starved
   with vr_arm_hand=off. The position-path publisher read the controller-
   versus-head geometry, which does not change when the player turns on
   their feet: a live session (2026-08-23) measured the gap pinned at 0.0
   through eleven manual recalibrations, while a sideways hand swing spun
   the body after the CONTROLLER, gap co-moving with drift at +127/+128.
   This publisher reads the mapped head yaw minus the published body
   drift, so this test pins the four properties the consumer steers by:
   the gap is zero at calibration, it follows the FACING one-for-one, it
   ignores the controller entirely, and a body turn moves it by exactly
   MINUS the published drift ("d gap = -d drift", tail-chase impossible).
   Plus the escape hatch: vr_follow_head_sign flips the head term only. */
static int t_the_gap_reads_the_facing_minus_the_body(void)
{
    enum { STRIDE = 0x180, JOINTS = 7 };
    static unsigned char blob[DG_OBJS_ARRAY + JOINTS * STRIDE];
    static unsigned char obj[0x40], mc[0x70];
    static float adjust[55 * 4];
    ULONGLONG arm = (ULONGLONG)(ULONG_PTR)obj;
    DG_BRIDGE_ARM_TARGET target;
    float gap0, gap_head, gap_hand, gap_body, drift_body, gap_flip;
    float gap_src_hand, gap_src_glance, gap_src_stick, gap_src_stick2;
    LONG gv;
    int j, k, bad = 0;

    memset(blob, 0, sizeof blob);
    memset(obj, 0, sizeof obj);
    memset(mc, 0, sizeof mc);
    memset(adjust, 0, sizeof adjust);
    *(ULONGLONG *)(obj + 0x00) = (ULONGLONG)(ULONG_PTR)blob;
    *(ULONGLONG *)(obj + 0x08) = (ULONGLONG)(ULONG_PTR)mc;
    *(LONG *)(mc + 0x14) = 55;
    *(ULONGLONG *)(mc + 0x48) = (ULONGLONG)(ULONG_PTR)adjust;
    for (j = 0; j < JOINTS; j++) {
        float *m = (float *)(blob + DG_OBJS_ARRAY + j * STRIDE);
        m[0] = m[5] = m[10] = m[15] = 1.0f;
    }
    ((float *)(blob + DG_OBJS_ARRAY + 3 * STRIDE))[14] = 100.0f;
    ((float *)(blob + DG_OBJS_ARRAY + 5 * STRIDE))[12] = 200.0f;
    ((float *)(blob + DG_OBJS_ARRAY + 6 * STRIDE))[12] = 300.0f;
    ((float *)(blob + DG_OBJS_ARRAY + 6 * STRIDE))[13] = 173.20508f;

    g_b.a.gm_player_arm_body = (ULONGLONG)(ULONG_PTR)&arm;
    InterlockedExchange(&g_b.skel_stride, STRIDE);
    InterlockedExchange(&g_b.skel_parents_read, JOINTS);
    InterlockedExchange(&g_b.skel_region_end, (LONG)sizeof blob);
    for (j = 0; j < JOINTS; j++) InterlockedExchange(&g_b.skel_parents[j], 0);
    InterlockedExchange(&g_b.skel_parents[3], 2);
    InterlockedExchange(&g_b.skel_parents[4], 3);
    InterlockedExchange(&g_b.skel_parents[5], 4);
    InterlockedExchange(&g_b.skel_parents[6], 5);
    InterlockedExchange(&g_b.ik_active, 0);
    InterlockedExchange(&g_b.arm_freeze, 0);
    InterlockedExchange(&g_b.c_ticks, 0);
    InterlockedExchange(&g_b.s_arm_aim_gap, f2l(0.0f));
    InterlockedExchange(&g_b.arm_aim_gap_tick, 0);
    arm_map_forget();
    g_b.arm_map_phase = 0;

    memset(&target, 0, sizeof target);
    target.write = 1;
    target.weight = 1.0;
    target.stream_id = 7;
    target.pair_id = 100;
    target.wrist_view[0] = 770.0;
    target.hand_quat[3] = 1.0;
    target.head_yaw_valid = 1;
    target.head_yaw_rad = 0.0;
    InterlockedExchange(&g_b.follow_head_sign, 1);
    InterlockedExchange(&g_b.follow_src, 0);
    g_b.hand_yaw0_have = 0;
    g_b.stick_yaw0_have = 0;

    for (j = 0; j < 16 && !g_b.arm_map_cache_valid; j++) {
        InterlockedExchange(&g_b.c_ticks,
                            (LONG)(DG_ADJ_SETTLE_TICKS + 1 + j));
        arm_ik_now(arm, &target);
        target.pair_id++;
    }
    if (!g_b.arm_map_cache_valid) bad++;
    if (!g_b.head_yaw0_have) bad++;
    gv = InterlockedCompareExchange(&g_b.s_arm_aim_gap, 0, 0);
    memcpy(&gap0, &gv, sizeof gap0);
    if (fabs((double)gap0) > 0.5) bad++;
    if (InterlockedCompareExchange(&g_b.arm_aim_gap_tick, 0, 0) !=
        InterlockedCompareExchange(&g_b.c_ticks, 0, 0)) bad++;

    /* The player turns their FACING 30 degrees - by stick or on their
       feet, upstream makes those the same motion. The controller and the
       body stay. */
    target.head_yaw_rad = 30.0 * 3.14159265358979323846 / 180.0;
    InterlockedIncrement(&g_b.c_ticks);
    target.pair_id++;
    arm_ik_now(arm, &target);
    gv = InterlockedCompareExchange(&g_b.s_arm_aim_gap, 0, 0);
    memcpy(&gap_head, &gv, sizeof gap_head);
    if (fabs((double)gap_head - 30.0) > 0.5) bad++;

    /* The controller swings 30 degrees; the facing holds. The gap must
       not move at all - the body steering after the HAND is the measured
       live defect this publisher replaced. */
    target.head_yaw_rad = 0.0;
    InterlockedIncrement(&g_b.c_ticks);
    target.pair_id++;
    arm_ik_now(arm, &target);
    {
        double a = (90.0 + 30.0) * 3.14159265358979323846 / 180.0;
        target.wrist_view[0] = 770.0 * sin(a);
        target.wrist_view[2] = 770.0 * cos(a);
    }
    InterlockedIncrement(&g_b.c_ticks);
    target.pair_id++;
    arm_ik_now(arm, &target);
    gv = InterlockedCompareExchange(&g_b.s_arm_aim_gap, 0, 0);
    memcpy(&gap_hand, &gv, sizeof gap_hand);
    if (fabs((double)gap_hand) > 0.5) bad++;

    /* The body turns; facing and controller hold. The published drift
       names the body's turn in whatever sign the basis convention
       produces - the contract is not the sign itself but the SUM: with
       the facing at zero, gap + drift = 0, which is exactly the
       "d gap = -d drift" the follow's stability rests on. */
    target.wrist_view[0] = 770.0;
    target.wrist_view[2] = 0.0;
    {
        float *root = (float *)(blob + DG_OBJS_ARRAY);
        double a = 30.0 * 3.14159265358979323846 / 180.0;
        root[0] = (float)cos(a);  root[2] = (float)sin(a);
        root[8] = (float)-sin(a); root[10] = (float)cos(a);
    }
    InterlockedIncrement(&g_b.c_ticks);
    target.pair_id++;
    arm_ik_now(arm, &target);
    gv = InterlockedCompareExchange(&g_b.s_arm_aim_gap, 0, 0);
    memcpy(&gap_body, &gv, sizeof gap_body);
    gv = InterlockedCompareExchange(&g_b.s_arm_body_drift, 0, 0);
    memcpy(&drift_body, &gv, sizeof drift_body);
    if (!(fabs((double)drift_body) > 25.0)) bad++;
    if (!(fabs((double)gap_body + (double)drift_body) < 1.0)) bad++;

    /* The marker's escape hatch flips the head term and nothing else:
       facing +30 under sign -1 reads as -30, the drift term unmoved. */
    InterlockedExchange(&g_b.follow_head_sign, -1);
    target.head_yaw_rad = 30.0 * 3.14159265358979323846 / 180.0;
    InterlockedIncrement(&g_b.c_ticks);
    target.pair_id++;
    arm_ik_now(arm, &target);
    gv = InterlockedCompareExchange(&g_b.s_arm_aim_gap, 0, 0);
    memcpy(&gap_flip, &gv, sizeof gap_flip);
    if (fabs((double)gap_flip - (-30.0 - (double)drift_body)) > 1.0) bad++;
    InterlockedExchange(&g_b.follow_head_sign, 1);

    InterlockedExchange(&g_b.follow_src, 1);
    target.head_yaw_rad = 0.0;
    target.hand_yaw_valid = 0;
    {
        LONG t0 = InterlockedCompareExchange(&g_b.arm_aim_gap_tick, 0, 0);
        InterlockedIncrement(&g_b.c_ticks);
        target.pair_id++;
        arm_ik_now(arm, &target);
        if (InterlockedCompareExchange(&g_b.arm_aim_gap_tick, 0, 0) != t0)
            bad++;
    }

    arm_map_forget();
    g_b.arm_map_phase = 0;
    target.hand_yaw_valid = 1;
    target.hand_yaw_rad = 10.0 * 3.14159265358979323846 / 180.0;
    for (j = 0; j < 16 && !g_b.arm_map_cache_valid; j++) {
        InterlockedIncrement(&g_b.c_ticks);
        arm_ik_now(arm, &target);
        target.pair_id++;
    }
    if (!g_b.hand_yaw0_have) bad++;
    gv = InterlockedCompareExchange(&g_b.s_arm_aim_gap, 0, 0);
    memcpy(&gap0, &gv, sizeof gap0);
    if (fabs((double)gap0) > 0.5) bad++;
    /* The gun swings 30 degrees past its epoch: the gap follows the gun. */
    target.hand_yaw_rad = 40.0 * 3.14159265358979323846 / 180.0;
    InterlockedIncrement(&g_b.c_ticks);
    target.pair_id++;
    arm_ik_now(arm, &target);
    gv = InterlockedCompareExchange(&g_b.s_arm_aim_gap, 0, 0);
    memcpy(&gap_src_hand, &gv, sizeof gap_src_hand);
    if (fabs((double)gap_src_hand - 30.0) > 0.5) bad++;

    target.head_yaw_rad = 30.0 * 3.14159265358979323846 / 180.0;
    InterlockedIncrement(&g_b.c_ticks);
    target.pair_id++;
    arm_ik_now(arm, &target);
    gv = InterlockedCompareExchange(&g_b.s_arm_aim_gap, 0, 0);
    memcpy(&gap_src_glance, &gv, sizeof gap_src_glance);
    if (fabs((double)gap_src_glance - 30.0) > 0.5) bad++;

    target.hand_yaw_valid = 0;
    {
        LONG t0 = InterlockedCompareExchange(&g_b.arm_aim_gap_tick, 0, 0);
        InterlockedIncrement(&g_b.c_ticks);
        target.pair_id++;
        arm_ik_now(arm, &target);
        if (InterlockedCompareExchange(&g_b.arm_aim_gap_tick, 0, 0) != t0)
            bad++;
    }
    InterlockedExchange(&g_b.follow_src, 0);
    target.head_yaw_rad = 0.0;

    /* vr_turn_follow_src=stick: only the software turn is a facing. Its
       zero is captured at calibration like the others; a stick turn of 30
       degrees publishes 30, and neither a 30-degree glance nor a 30-degree
       hand swing moves it. */
    InterlockedExchange(&g_b.follow_src, 2);
    arm_map_forget();
    g_b.arm_map_phase = 0;
    target.hand_yaw_valid = 1;
    target.hand_yaw_rad = 0.0;
    target.stick_yaw_valid = 1;
    target.stick_yaw_rad = -20.0 * 3.14159265358979323846 / 180.0;
    for (j = 0; j < 16 && !g_b.arm_map_cache_valid; j++) {
        InterlockedIncrement(&g_b.c_ticks);
        arm_ik_now(arm, &target);
        target.pair_id++;
    }
    if (!g_b.stick_yaw0_have) bad++;
    gv = InterlockedCompareExchange(&g_b.s_arm_aim_gap, 0, 0);
    memcpy(&gap0, &gv, sizeof gap0);
    if (fabs((double)gap0) > 0.5) bad++;
    target.stick_yaw_rad = 10.0 * 3.14159265358979323846 / 180.0;
    InterlockedIncrement(&g_b.c_ticks);
    target.pair_id++;
    arm_ik_now(arm, &target);
    gv = InterlockedCompareExchange(&g_b.s_arm_aim_gap, 0, 0);
    memcpy(&gap_src_stick, &gv, sizeof gap_src_stick);
    if (fabs((double)gap_src_stick - 30.0) > 0.5) bad++;
    target.head_yaw_rad = 30.0 * 3.14159265358979323846 / 180.0;
    target.hand_yaw_rad = 30.0 * 3.14159265358979323846 / 180.0;
    InterlockedIncrement(&g_b.c_ticks);
    target.pair_id++;
    arm_ik_now(arm, &target);
    gv = InterlockedCompareExchange(&g_b.s_arm_aim_gap, 0, 0);
    memcpy(&gap_src_stick2, &gv, sizeof gap_src_stick2);
    if (fabs((double)gap_src_stick2 - 30.0) > 0.5) bad++;
    InterlockedExchange(&g_b.follow_src, 0);
    target.head_yaw_rad = 0.0;
    target.hand_yaw_rad = 0.0;
    target.stick_yaw_valid = 0;

    arm_map_forget();
    g_b.a.gm_player_arm_body = 0;
    InterlockedExchange(&g_b.skel_stride, 0);
    InterlockedExchange(&g_b.skel_parents_read, 0);
    InterlockedExchange(&g_b.skel_region_end, 0);
    InterlockedExchange(&g_b.c_ticks, 0);
    InterlockedExchange(&g_b.s_arm_aim_gap, f2l(0.0f));
    InterlockedExchange(&g_b.arm_aim_gap_tick, 0);
    g_b.head_yaw0_have = 0;

    printf("  %-6s the gap reads the facing minus the body: zero at "
           "calibration, a 30-degree facing turn publishes %+.1f, a "
           "30-degree CONTROLLER swing publishes %+.1f (the body no "
           "longer chases the hand), a body turn publishes drift %+.1f "
           "and moves the gap to %+.1f (summing to zero - the tail-chase "
           "is structurally dead), the head-sign hatch flips the "
           "facing term to %+.1f, and the hand source is mute without a "
           "tracked epoch, follows the gun to %+.1f, ignores a glance "
           "(%+.1f) and goes mute again when tracking drops; the stick "
           "source follows a 30-degree software turn to %+.1f and holds "
           "it (%+.1f) through a glance and a hand swing\n",
           bad ? "FAIL" : "ok", (double)gap_head, (double)gap_hand,
           (double)drift_body, (double)gap_body, (double)gap_flip,
           (double)gap_src_hand, (double)gap_src_glance,
           (double)gap_src_stick, (double)gap_src_stick2);
    return bad ? 1 : 0;
}

static int t_theater_mask_is_narrower_than_the_gate(void)
{
    int bad = 0;

    /* Each mask bit alone must trip the judgment... */
    if (!dg_theater_masked(0x10000000u, 0)) bad++;      /* STATE_DEMO */
    if (!dg_theater_masked(0x08000000u, 0)) bad++;      /* STATE_SCN_DEMO */
    if (!dg_theater_masked(0x40000000u, 0)) bad++;      /* STATE_PAD_DEMO */
    if (!dg_theater_masked(0, 0x00000400u)) bad++;      /* MENU_RADIO_ON */

    if (dg_theater_masked(0x20000000u, 0)) bad++;       /* STATE_PRG_DEMO */
    if (dg_theater_masked(0x00000040u, 0)) bad++;       /* STATE_CUT_IN */
    if (!dg_theater_masked(0x80000000u, 0)) bad++;      /* STATE_GAMEOVER */
    if (!dg_theater_masked(0x00004000u, 0)) bad++;      /* DISP_GAMEOVER */
    if (dg_theater_masked(0x00000001u, 0)) bad++;       /* STATE_DETECT */

    /* The theater mask is a strict SUBSET of the safety gate: everything the
       theater switches on, the writers were already standing down for. If
       someone ever narrows the gate to match the theater, this fails. */
    if ((DG_THEATER_GAME_MASK & ~DG_GAME_UNSAFE_MASK) != 0) bad++;
    if ((DG_THEATER_MENU_MASK & ~DG_MENU_UNSAFE_MASK) != 0) bad++;
    /* ...and the gate keeps the two bits the theater refuses. */
    if (!(DG_GAME_UNSAFE_MASK & 0x20000000u)) bad++;
    if (!(DG_GAME_UNSAFE_MASK & 0x00000040u)) bad++;

    /* The measured session words of 2026-08-17: the camera seam saw a real
       cutscene (0x18000240 also carries CUT_IN and PAUSE_DISABLE - the mask
       must trip on the demo bits, not on those) and a codec-bearing menu
       word; the tick seam's words must trip nothing. */
    if (!dg_theater_masked(0x18000240u, 0x00145803u)) bad++;
    if (!dg_theater_masked(0, 0x00345C0Fu)) bad++;
    if (dg_theater_masked(0x00000000u, 0x00005800u)) bad++;

    /* The panel judgment: full menus only, and the codec is not its bit. */
    if (!dg_ui_panel_masked(0x00000100u)) bad++;        /* WEAPON_OPEN */
    if (!dg_ui_panel_masked(0x00000200u)) bad++;        /* ITEM_OPEN */
    if (dg_ui_panel_masked(0x00000400u)) bad++;         /* RADIO: theater's */
    if (dg_ui_panel_masked(0x00001800u)) bad++;         /* RADAR/GAGE_ON */

    /* The game's own hide-command bits must trip neither judgment - they are
       what vr_hud writes, and a HUD hide that opened the theater would be a
       feedback loop. And the hide set is exactly the four visibility
       commands, captions excluded. */
    if (dg_theater_masked(0, DG_HUD_HIDE_BITS)) bad++;
    if (dg_ui_panel_masked(DG_HUD_HIDE_BITS)) bad++;
    if (DG_HUD_HIDE_BITS != 0x0000000Fu) bad++;
    if (DG_HUD_HIDE_BITS & 0x00000010u) bad++;          /* MENU_CAPTION_OFF */

    printf("  %-6s theater mask: the three demo bits and the codec bit trip "
           "it, gameover also enters; boss immortality (PRG_DEMO) and cut-ins "
           "do not, it stays a subset of the safety gate, and the panel "
           "judgment is the two full menus and nothing else\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

static int t_theater_hysteresis_holds_both_flanks(void)
{
    DG_THEATER_HYST h;
    int i, bad = 0;

    /* Enter needs N CONSECUTIVE masked samples. N-1 then one clean sample
       starts the count over - a blinking bit can never accumulate in. */
    memset(&h, 0, sizeof h);
    for (i = 0; i < DG_THEATER_ENTER_SAMPLES - 1; i++)
        if (dg_theater_hyst_step(&h, 1, DG_THEATER_ENTER_SAMPLES,
                                 DG_THEATER_EXIT_SAMPLES)) bad++;
    if (dg_theater_hyst_step(&h, 0, DG_THEATER_ENTER_SAMPLES,
                             DG_THEATER_EXIT_SAMPLES)) bad++;
    for (i = 0; i < DG_THEATER_ENTER_SAMPLES - 1; i++)
        if (dg_theater_hyst_step(&h, 1, DG_THEATER_ENTER_SAMPLES,
                                 DG_THEATER_EXIT_SAMPLES)) bad++;
    if (!dg_theater_hyst_step(&h, 1, DG_THEATER_ENTER_SAMPLES,
                              DG_THEATER_EXIT_SAMPLES)) bad++;

    /* Exit needs M consecutive clean samples, and one masked sample mid-count
       resets it - the scripted-transition blink (pad release -> cancel ->
       next demo) must ride through as one theater, not two. */
    for (i = 0; i < DG_THEATER_EXIT_SAMPLES - 1; i++)
        if (!dg_theater_hyst_step(&h, 0, DG_THEATER_ENTER_SAMPLES,
                                  DG_THEATER_EXIT_SAMPLES)) bad++;
    if (!dg_theater_hyst_step(&h, 1, DG_THEATER_ENTER_SAMPLES,
                              DG_THEATER_EXIT_SAMPLES)) bad++;
    for (i = 0; i < DG_THEATER_EXIT_SAMPLES - 1; i++)
        if (!dg_theater_hyst_step(&h, 0, DG_THEATER_ENTER_SAMPLES,
                                  DG_THEATER_EXIT_SAMPLES)) bad++;
    if (dg_theater_hyst_step(&h, 0, DG_THEATER_ENTER_SAMPLES,
                             DG_THEATER_EXIT_SAMPLES)) bad++;

    /* Steady states hold without drift in either direction. */
    for (i = 0; i < 500; i++)
        if (dg_theater_hyst_step(&h, 0, DG_THEATER_ENTER_SAMPLES,
                                 DG_THEATER_EXIT_SAMPLES)) bad++;
    for (i = 0; i < DG_THEATER_ENTER_SAMPLES; i++)
        dg_theater_hyst_step(&h, 1, DG_THEATER_ENTER_SAMPLES,
                             DG_THEATER_EXIT_SAMPLES);
    for (i = 0; i < 500; i++)
        if (!dg_theater_hyst_step(&h, 1, DG_THEATER_ENTER_SAMPLES,
                                  DG_THEATER_EXIT_SAMPLES)) bad++;

    /* The thresholds themselves are part of the contract: enter fast, exit
       slow, and the asymmetry is the design (section 2.5). A mutant that
       removes either collapses one of them to 1. */
    if (DG_THEATER_ENTER_SAMPLES < 2 || DG_THEATER_EXIT_SAMPLES < 2) bad++;
    if (DG_THEATER_EXIT_SAMPLES <= DG_THEATER_ENTER_SAMPLES) bad++;
    if (DG_UI_ENTER_SAMPLES < 2 || DG_UI_EXIT_SAMPLES < 2) bad++;

    printf("  %-6s theater hysteresis: enter only after %d consecutive masked "
           "samples, exit only after %d clean ones, any contradiction resets "
           "the count, and both steady states hold\n",
           bad ? "FAIL" : "ok",
           DG_THEATER_ENTER_SAMPLES, DG_THEATER_EXIT_SAMPLES);
    return bad ? 1 : 0;
}

static int t_theater_verdict_is_one_publication_fail_closed(void)
{
    static LONG game, game_scn, menu, menu_scn;
    static ULONGLONG player;
    DG_ANCHORS saved_anchors = g_b.a;
    LONG ring0;
    int i, bad = 0;

    game = game_scn = menu = menu_scn = 0;
    player = 0;
    memset(&g_b.a, 0, sizeof g_b.a);
    g_b.a.gm_game_status = (ULONGLONG)(ULONG_PTR)&game;
    g_b.a.gm_game_status_scn = (ULONGLONG)(ULONG_PTR)&game_scn;
    g_b.a.gm_menu_status = (ULONGLONG)(ULONG_PTR)&menu;
    g_b.a.gm_menu_status_scn = (ULONGLONG)(ULONG_PTR)&menu_scn;
    g_b.a.gm_player_status = (ULONGLONG)(ULONG_PTR)&player;
    memset(&g_b.thea_demo, 0, sizeof g_b.thea_demo);
    memset(&g_b.thea_ui, 0, sizeof g_b.thea_ui);
    InterlockedExchange(&g_b.s_theater, 0);
    InterlockedExchange(&g_b.theater_mode, DG_THEATER_MEASURE);
    InterlockedExchange(&g_b.theater_ui_mode, 0);
    InterlockedExchange(&g_b.c_thea_enter, 0);
    InterlockedExchange(&g_b.c_thea_exit, 0);
    InterlockedExchange(&g_b.c_thea_ui_enter, 0);
    InterlockedExchange(&g_b.c_thea_ui_exit, 0);
    InterlockedExchange(&g_b.armed, 1);
    g_b.fps.state = DG_FPS_OFF;          /* the seam's later gates return */
    ring0 = InterlockedCompareExchange(&g_thea_ring_head, 0, 0);

    /* A cutscene in the SCENARIO half of the word, the half a single-anchor
       reader would miss. The judgment must publish exactly at the enter
       threshold - and the seam is the ONE writer: everything downstream reads
       this publication rather than judging per-eye or per-thread. */
    game_scn = 0x08000000;
    for (i = 0; i < DG_THEATER_ENTER_SAMPLES - 1; i++) {
        dg_bridge_arm_seam_now(NULL);
        if (InterlockedCompareExchange(&g_b.s_theater, 0, 0)) bad++;
    }
    dg_bridge_arm_seam_now(NULL);
    if (InterlockedCompareExchange(&g_b.s_theater, 0, 0) != DG_THEATER_V_DEMO)
        bad++;
    if (g_b.c_thea_enter != 1) bad++;
    /* MEASURE logged the flank... */
    if (InterlockedCompareExchange(&g_thea_ring_head, 0, 0) != ring0 + 1)
        bad++;
    /* ...but MEASURE never sets a consumer bit - that is the whole mode. */
    if (dg_bridge_theater_verdict() != 0) bad++;

    /* ON reports it, freshly published as it is. */
    InterlockedExchange(&g_b.theater_mode, DG_THEATER_ON);
    if (dg_bridge_theater_verdict() != DG_THEATER_V_DEMO) bad++;

    /* A stale publication is refused whatever the marker says: a camera hook
       that stops firing must never pin the theater on. */
    InterlockedExchange(&g_b.theater_ms,
                        (LONG)(GetTickCount() - DG_THEATER_FRESH_MS - 100));
    if (dg_bridge_theater_verdict() != 0) bad++;
    dg_bridge_arm_seam_now(NULL);        /* one live seam re-freshens it */
    if (dg_bridge_theater_verdict() != DG_THEATER_V_DEMO) bad++;

    /* An unarmed bridge reports nothing, whatever was last published. */
    InterlockedExchange(&g_b.armed, 0);
    if (dg_bridge_theater_verdict() != 0) bad++;
    InterlockedExchange(&g_b.armed, 1);

    /* The scene ends; the exit hysteresis holds the verdict up for the whole
       exit window and not one sample longer. */
    game_scn = 0;
    for (i = 0; i < DG_THEATER_EXIT_SAMPLES - 1; i++) {
        dg_bridge_arm_seam_now(NULL);
        if (!InterlockedCompareExchange(&g_b.s_theater, 0, 0)) bad++;
    }
    dg_bridge_arm_seam_now(NULL);
    if (InterlockedCompareExchange(&g_b.s_theater, 0, 0)) bad++;
    if (g_b.c_thea_exit != 1) bad++;
    if (dg_bridge_theater_verdict() != 0) bad++;

    /* The panel judgment rides the same publication under its own bit and its
       own marker gate: a weapon menu with vr_ui off reaches no consumer. */
    menu = 0x00000100;
    for (i = 0; i < DG_UI_ENTER_SAMPLES; i++) dg_bridge_arm_seam_now(NULL);
    if (InterlockedCompareExchange(&g_b.s_theater, 0, 0) != DG_THEATER_V_UI)
        bad++;
    if (dg_bridge_theater_verdict() != 0) bad++;
    InterlockedExchange(&g_b.theater_ui_mode, 1);
    if (dg_bridge_theater_verdict() != DG_THEATER_V_UI) bad++;
    /* And vr_ui gates only its own bit, not the theater's. */
    InterlockedExchange(&g_b.theater_mode, DG_THEATER_OFF);
    if (dg_bridge_theater_verdict() != DG_THEATER_V_UI) bad++;

    /* An anchor that never resolved measures nothing and switches nothing:
       the whole status block is behind the anchor test, so the publication
       simply never happens. Fail-open here would be a theater driven by
       whatever address zero holds. */
    g_b.a.gm_game_status = 0;
    menu = 0;
    for (i = 0; i < DG_THEATER_EXIT_SAMPLES + DG_UI_EXIT_SAMPLES; i++)
        dg_bridge_arm_seam_now(NULL);
    if (InterlockedCompareExchange(&g_b.s_theater, 0, 0) != DG_THEATER_V_UI)
        bad++;                           /* frozen, not re-judged... */
    Sleep(DG_THEATER_FRESH_MS + 50);     /* ...and staleness retires it */
    if (dg_bridge_theater_verdict() != 0) bad++;

    /* UI-U1: the PRESENT seam reaches the same judgment through the same
       one publication - and it is the only seam that can, because a menu
       pauses the actor the tick seam lives in and the camera block the
       camera seam hangs off. Driven here rather than in a test of its own
       so it is measured against the identical rig and the identical
       thresholds the camera seam is held to. */
    g_b.a.gm_game_status = (ULONGLONG)(ULONG_PTR)&game;
    memset(&g_b.thea_demo, 0, sizeof g_b.thea_demo);
    memset(&g_b.thea_ui, 0, sizeof g_b.thea_ui);
    InterlockedExchange(&g_b.s_theater, 0);
    InterlockedExchange(&g_b.seen_menu_screen, 0);
    InterlockedExchange(&g_b.seen_game_screen, 0);
    InterlockedExchange(&g_b.c_screen_status, 0);
    InterlockedExchange(&g_b.theater_mode, DG_THEATER_ON);
    InterlockedExchange(&g_b.theater_ui_mode, 1);
    menu = 0x00000100;                       /* MENU_WEAPON_OPEN */
    {
        LONG samples0 = g_b.c_screen_status;
        for (i = 0; i < DG_UI_ENTER_SAMPLES; i++) dg_bridge_screen_seam_now();
        if (g_b.c_screen_status != samples0 + DG_UI_ENTER_SAMPLES) bad++;
        if (dg_bridge_theater_verdict() != DG_THEATER_V_UI) bad++;
        /* The accumulator is what makes the live log readable: the bit the
           other two seams can never hold has to show up in THIS mask. */
        if (!((unsigned int)g_b.seen_menu_screen & 0x100u)) bad++;
        /* Reads only - the seam must never write the game's own words. */
        if (menu != 0x00000100 || game != 0 || player != 0) bad++;

        /* Unarmed: no sample, no count, no publication. */
        InterlockedExchange(&g_b.armed, 0);
        samples0 = g_b.c_screen_status;
        dg_bridge_screen_seam_now();
        if (g_b.c_screen_status != samples0) bad++;
        InterlockedExchange(&g_b.armed, 1);

        /* An unresolved anchor likewise: never a judgment built on address
           zero, and the count stays put so the log can still tell "never
           looked" from "never happened". */
        g_b.a.gm_game_status = 0;
        samples0 = g_b.c_screen_status;
        dg_bridge_screen_seam_now();
        if (g_b.c_screen_status != samples0) bad++;
        g_b.a.gm_game_status = (ULONGLONG)(ULONG_PTR)&game;

        /* And it retires a scene through the same exit hysteresis, so the
           present seam cannot pin the quad on after the menu closes. */
        menu = 0;
        for (i = 0; i < DG_UI_EXIT_SAMPLES; i++) dg_bridge_screen_seam_now();
        if (dg_bridge_theater_verdict() != 0) bad++;
    }

    /* Game Over must use that same Present publication even when the game
       camera continues handing off frames. Continue remains independently
       admitted, then gameplay resumes after the normal exit hysteresis. */
    game = (LONG)0x80000000u;
    for (i=0;i<DG_THEATER_ENTER_SAMPLES;i++) dg_bridge_screen_seam_now();
    if(dg_bridge_theater_verdict()!=DG_THEATER_V_DEMO)bad++;
    if(!dg_bridge_menu_gameover_now())bad++;
    game=0;game_scn=0x00004000;
    dg_bridge_screen_seam_now();
    if(dg_bridge_theater_verdict()!=DG_THEATER_V_DEMO)bad++;
    if(!dg_bridge_menu_gameover_now())bad++;
    game_scn=0;
    for(i=0;i<DG_THEATER_EXIT_SAMPLES;i++)dg_bridge_screen_seam_now();
    if(dg_bridge_theater_verdict()!=0 || dg_bridge_menu_gameover_now())bad++;

    InterlockedExchange(&g_b.armed, 0);
    g_b.a = saved_anchors;
    memset(&g_b.thea_demo, 0, sizeof g_b.thea_demo);
    memset(&g_b.thea_ui, 0, sizeof g_b.thea_ui);
    InterlockedExchange(&g_b.s_theater, 0);
    InterlockedExchange(&g_b.seen_menu_screen, 0);
    InterlockedExchange(&g_b.seen_game_screen, 0);
    InterlockedExchange(&g_b.c_screen_status, 0);
    InterlockedExchange(&g_b.theater_mode, 0);
    InterlockedExchange(&g_b.theater_ui_mode, 0);
    InterlockedExchange(&g_b.c_thea_enter, 0);
    InterlockedExchange(&g_b.c_thea_exit, 0);
    InterlockedExchange(&g_b.c_thea_ui_enter, 0);
    InterlockedExchange(&g_b.c_thea_ui_exit, 0);
    InterlockedExchange(&g_thea_ring_tail,
                        InterlockedCompareExchange(&g_thea_ring_head, 0, 0));
    g_b.fps.state = DG_FPS_OFF;

    printf("  %-6s theater verdict: the camera seam is the one publisher, "
           "measure counts but never switches, on reports only a FRESH "
           "verdict, unarmed or unanchored reports none, and each marker key "
           "gates only its own judgment\n", bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

static int t_hud_hide_borrows_the_games_own_bits(void)
{
    static LONG menu;
    DG_ANCHORS saved_anchors = g_b.a;
    int bad = 0;

    memset(&g_b.a, 0, sizeof g_b.a);
    g_b.a.gm_menu_status = (ULONGLONG)(ULONG_PTR)&menu;
    InterlockedExchange(&g_b.hud_mode, 0);
    InterlockedExchange(&g_b.hud_held, 0);
    InterlockedExchange(&g_b.c_hud_writes, 0);
    InterlockedExchange(&g_b.c_hud_cleared, 0);

    /* Off writes nothing, held or not - and holds no debt. */
    menu = 0x00001800;                   /* RADAR_ON | GAGE_ON */
    hud_tick();
    if (menu != 0x00001800 || g_b.c_hud_writes || g_b.hud_held) bad++;

    /* On asserts exactly the four hide commands, once. */
    InterlockedExchange(&g_b.hud_mode, 1);
    hud_tick();
    if (menu != 0x0000180F) bad++;
    if (g_b.c_hud_writes != 1 || !g_b.hud_held) bad++;
    /* Already-set bits are not re-written - the counter stays honest about
       how often the game actually fought the hide. */
    hud_tick();
    if (g_b.c_hud_writes != 1) bad++;
    /* The game clears them (scene change); the next tick re-asserts. */
    menu = 0x00001800;
    hud_tick();
    if (menu != 0x0000180F || g_b.c_hud_writes != 2) bad++;

    /* The game's own caption hide is preserved through both directions:
       0x10 is not ours and must survive the release untouched. */
    menu |= 0x00000010;
    InterlockedExchange(&g_b.hud_mode, 0);
    hud_tick();
    if (menu != 0x00001810) bad++;       /* our four gone, caption kept */
    if (g_b.c_hud_cleared != 1 || g_b.hud_held) bad++;
    /* The release fires once, not per tick. */
    menu = 0x0000000F;                   /* someone else's hide bits */
    hud_tick();
    if (menu != 0x0000000F || g_b.c_hud_cleared != 1) bad++;

    /* No anchor, no write, no crash - fail closed like everything else. */
    g_b.a.gm_menu_status = 0;
    InterlockedExchange(&g_b.hud_mode, 1);
    hud_tick();
    if (g_b.c_hud_writes != 2 || g_b.hud_held) bad++;

    g_b.a = saved_anchors;
    InterlockedExchange(&g_b.hud_mode, 0);
    InterlockedExchange(&g_b.hud_held, 0);
    InterlockedExchange(&g_b.c_hud_writes, 0);
    InterlockedExchange(&g_b.c_hud_cleared, 0);

    printf("  %-6s hud hide: vr_hud=off asserts the game's own four hide "
           "commands and re-asserts them when the game clears them, release "
           "clears exactly those four once, captions and foreign bits "
           "survive, and a missing anchor writes nothing\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

/* ROLL_ONTWERP_V5.1 par. 6.1: the adjust frame's helpers against ABSOLUTE
   hand-computed oracles (17 digits, recordings/adjust_probe_blokken_2026-08-
   30.log) - never a round trip alone, which a consistently wrong pair of
   helpers passes. The decisive tie: the live frame at the 2026-08-18
   session's heading (rot.vy 1024) must reproduce the legacy constant, so the
   two modes agree where the constant was measured and differ everywhere
   else by exactly the body's turn. */
static int t_adjust_frame(void)
{
    static const double f_yaw73[4] =
        { 0.0, 0.59482278675134126, 0.0, 0.80385686061721728 };
    static const double s15 = 0.25881904510252076, c15 = 0.96592582628906829;
    static const double wX[4] =
        { 0.075671365431334875, 0.0, -0.24750988376535193, 0.9659258262890682 };
    static const double wZ[4] =
        { 0.24750988376535193, 0.0, 0.075671365431334875, 0.9659258262890682 };
    static const double wY[4] =
        { 0.0, 0.25881904510252079, 0.0, 0.96592582628906831 };
    static const double wTilt[4] =
        { 0.2444626263770959, -0.040488218862836309, 0.074739725353443393,
          0.9659258262890682 };
    static const double sentinel[4] = { 12345.0, -1.0, 7.0, 99.0 };
    DG_ADJ_FRAME live, bad_frame;
    double aX[4] = { s15, 0.0, 0.0, c15 };
    double aY[4] = { 0.0, s15, 0.0, c15 };
    double aZ[4] = { 0.0, 0.0, s15, c15 };
    double out[4], back[4], wrong[4], t[4], fc[4], f_full[4], tilt9[4];
    double f1024[4], f0[4], fm2048[4], xhat[3] = { 1.0, 0.0, 0.0 }, img[3];
    double leg[4], liv[4];
    int bad = 0, k;

    memset(&live, 0, sizeof live);
    live.live = 1; live.valid = 1;
    for (k = 0; k < 4; k++) live.q[k] = f_yaw73[k];

    /* absolute-conjugation leg: X and Z, and the commuting Y as a documented
       negative control (identical for every yaw - a Y-only oracle is blind). */
    if (!adjust_quat_to_world(&live, aX, out)) bad++;
    for (k = 0; k < 4; k++) if (fabs(out[k] - wX[k]) > 1e-9) bad++;
    if (!adjust_quat_to_world(&live, aZ, out)) bad++;
    for (k = 0; k < 4; k++) if (fabs(out[k] - wZ[k]) > 1e-9) bad++;
    if (!adjust_quat_to_world(&live, aY, out)) bad++;
    for (k = 0; k < 4; k++) if (fabs(out[k] - wY[k]) > 1e-9) bad++;
    /* The wrong side, f* (x) a (x) f, lands far away on the X case: the
       axis flips sign on z (146 degrees between the two AXES, 57.3 degrees
       between the two quaternions). Stated so the mutation has a number. */
    dg_ik_quat_conj(f_yaw73, fc);
    dg_ik_quat_mul(fc, aX, t);
    dg_ik_quat_mul(t, f_yaw73, wrong);
    if (dg_ik_quat_angle(wrong, wX) * 180.0 / 3.14159265358979323846 < 40.0)
        bad++;

    /* tilt leg: a full root (yaw73 (x) tilt9 about X) with A = 30 about Z -
       not X, which commutes with the tilt - differs from the yaw-only
       answer by 9.000 degrees of axis; the helper must give the yaw-only. */
    tilt9[0] = sin(4.5 * 3.14159265358979323846 / 180.0); tilt9[1] = 0.0;
    tilt9[2] = 0.0; tilt9[3] = cos(4.5 * 3.14159265358979323846 / 180.0);
    dg_ik_quat_mul(f_yaw73, tilt9, f_full);
    dg_ik_quat_mul(f_full, aZ, t);
    dg_ik_quat_conj(f_full, fc);
    dg_ik_quat_mul(t, fc, out);
    for (k = 0; k < 4; k++) if (fabs(out[k] - wTilt[k]) > 1e-9) bad++;
    if (!adjust_quat_to_world(&live, aZ, out)) bad++;
    if (dg_ik_quat_angle(out, wTilt) * 180.0 / 3.14159265358979323846 < 2.0)
        bad++;

    /* round trip, tier 1 only: consistency, not correctness. */
    if (!adjust_quat_to_world(&live, aX, out) ||
        !world_quat_to_adjust(&live, out, back)) bad++;
    for (k = 0; k < 4; k++) if (fabs(back[k] - aX[k]) > 1e-9) bad++;

    /* fail-closed leg: NULL, NaN and a non-unit frame all return 0 and leave
       the output exactly as they found it. */
    for (k = 0; k < 4; k++) out[k] = sentinel[k];
    if (adjust_quat_to_world(NULL, aX, out)) bad++;
    if (memcmp(out, sentinel, sizeof sentinel) != 0) bad++;
    bad_frame = live; bad_frame.q[1] = sqrt(-1.0);
    if (adjust_quat_to_world(&bad_frame, aX, out)) bad++;
    if (world_quat_to_adjust(&bad_frame, aX, out)) bad++;
    if (memcmp(out, sentinel, sizeof sentinel) != 0) bad++;
    bad_frame = live; for (k = 0; k < 4; k++) bad_frame.q[k] *= 1.01;
    if (adjust_quat_to_world(&bad_frame, aX, out)) bad++;
    if (memcmp(out, sentinel, sizeof sentinel) != 0) bad++;
    bad_frame = live; bad_frame.valid = 0;
    if (adjust_quat_to_world(&bad_frame, aX, out)) bad++;
    if (memcmp(out, sentinel, sizeof sentinel) != 0) bad++;

    if (!arm_frame_yaw(0, f0)) bad++;
    if (fabs(f0[0]) > 1e-12 || fabs(f0[1]) > 1e-12 || fabs(f0[2]) > 1e-12 ||
        fabs(f0[3] - 1.0) > 1e-12) bad++;
    if (!arm_frame_yaw(1024, f1024)) bad++;
    if (fabs(f1024[1] - 0.70710678118654752) > 1e-12 ||
        fabs(f1024[3] - 0.70710678118654752) > 1e-12 ||
        fabs(f1024[0]) > 1e-12 || fabs(f1024[2]) > 1e-12) bad++;
    arm_quat_rotate(f1024, xhat, img);
    if (fabs(img[0]) > 1e-9 || fabs(img[1]) > 1e-9 || fabs(img[2] + 1.0) > 1e-9)
        bad++;
    if (!arm_frame_yaw(-2048, fm2048)) bad++;
    if (fabs(fm2048[1] + 1.0) > 1e-12 || fabs(fm2048[3]) > 1e-12) bad++;
    /* The tie between the two modes: at the constant's own heading the live
       conversion reproduces the legacy one to within the constant's own fit
       residual (0.188 degrees) plus its 0.1-degree heading error. */
    live.live = 1; live.valid = 1;
    for (k = 0; k < 4; k++) live.q[k] = f1024[k];
    if (!adjust_quat_to_world(&live, aX, liv) ||
        !adjust_quat_to_world(&ADJ_FRAME_LEGACY, aX, leg)) bad++;
    if (dg_ik_quat_angle(liv, leg) * 180.0 / 3.14159265358979323846 > 0.6) bad++;
    if (!adjust_quat_to_world(&live, aZ, liv) ||
        !adjust_quat_to_world(&ADJ_FRAME_LEGACY, aZ, leg)) bad++;
    if (dg_ik_quat_angle(liv, leg) * 180.0 / 3.14159265358979323846 > 0.6) bad++;

    printf("  %-6s adjust frame: yaw73 conjugation lands on the hand-computed "
           "X/Z/Y oracles to 1e-9 (the wrong side sits 146 degrees off), a "
           "tilted root would move the Z case 9 degrees and the helper stays "
           "yaw-only, NULL/NaN/non-unit/invalid frames refuse and leave the "
           "output untouched, rot.vy 0/1024/-2048 give identity/[0 sin45 0 "
           "cos45]/[0 -1 0 0] with 1024 sending x to world -z, and at that "
           "heading live reproduces the 2026-08-18 constant within 0.6 "
           "degrees\n", bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

/* ---- the seam fixture (ROLL_ONTWERP_V5.1 par. 6.2) ------------------------
   A test-engine that composes the fake hierarchy from the SLOTS under the
   measured frame model, with the test's own quaternion arithmetic on the
   dg_ik primitives (none of the helpers under test): W_j = (f A5 f*)(f A4 f*)
   W_anim for the forearm chain, f the yaw of the actor's heading word. The
   legs then run the REAL seam functions - arm_ik_now, arm_remove_cached_
   adjust, arm_ik_replay_cached - against it, with the live slot, the cache
   and the frame deliberately three different things where a leg needs
   them to be. */
typedef struct {
    double p4[3];              /* joint 4 (elbow root) in world */
    double upper[3];           /* the ANIMATION's upper bone, 4 -> 5 */
    double fore[3];            /* the ANIMATION's forearm, 5 -> 6 */
} TQ_RIG;

static void tq_frame(short rot, double f[4])
{
    /* The engine's own statement of the measurement: heading -rot*360/4096,
       quaternion angle minus the heading (dg_ik's yaw sends x to heading
       -phi). Written here again on purpose, not via arm_frame_yaw. */
    double heading = -(double)rot * (360.0 / 4096.0);
    double half = -0.5 * heading * 3.14159265358979323846 / 180.0;
    f[0] = 0.0; f[1] = sin(half); f[2] = 0.0; f[3] = cos(half);
}

static void tq_rot(const double q[4], const double v[3], double o[3])
{
    double ux = q[0], uy = q[1], uz = q[2], w = q[3];
    double cx = uy * v[2] - uz * v[1], cy = uz * v[0] - ux * v[2],
           cz = ux * v[1] - uy * v[0];
    o[0] = v[0] + 2.0 * (w * cx + (uy * cz - uz * cy));
    o[1] = v[1] + 2.0 * (w * cy + (uz * cx - ux * cz));
    o[2] = v[2] + 2.0 * (w * cz + (ux * cy - uy * cx));
}

static void tq_world_of(const double f[4], const float a[4], double w[4])
{
    double A[4], t[4], fc[4];
    int k;
    for (k = 0; k < 4; k++) A[k] = (double)a[k];
    dg_ik_quat_mul(f, A, t);
    dg_ik_quat_conj(f, fc);
    dg_ik_quat_mul(t, fc, w);
    dg_ik_quat_normalize(w);
}

/* One hierarchy pass: joints 5 and 6 placed from the slots under `rot`. */
static void tq_engine(unsigned char *blob, int stride, const float *adjust,
                      short rot, const TQ_RIG *rig, double p6_out[3])
{
    double f[4], w4[4], w5[4], w45[4], up[3], t[3], fo[3], p5[3], p6[3];
    float *m4 = (float *)(blob + DG_OBJS_ARRAY + 4 * stride);
    float *m5 = (float *)(blob + DG_OBJS_ARRAY + 5 * stride);
    float *m6 = (float *)(blob + DG_OBJS_ARRAY + 6 * stride);
    int k, r;
    tq_frame(rot, f);
    tq_world_of(f, adjust + 4 * 4, w4);
    tq_world_of(f, adjust + 5 * 4, w5);
    /* The forearm inherits q4 first, then q5: W5 . W4 . fore, the strip's
       peel order in reverse and the roll's chain_w = b5w (x) b4w. */
    tq_rot(w4, rig->upper, up);
    tq_rot(w4, rig->fore, t);
    tq_rot(w5, t, fo);
    for (k = 0; k < 3; k++) {
        p5[k] = rig->p4[k] + up[k];
        p6[k] = p5[k] + fo[k];
        m4[12 + k] = (float)rig->p4[k];
        m5[12 + k] = (float)p5[k];
        m6[12 + k] = (float)p6[k];
        if (p6_out) p6_out[k] = p6[k];
    }
    /* Joint 6's basis rows: world images of its local axes = W45 applied
       to the unit axes (the animation's hand basis is the identity). */
    dg_ik_quat_mul(w5, w4, w45);
    for (r = 0; r < 3; r++) {
        double e[3] = { 0.0, 0.0, 0.0 }, img[3];
        e[r] = 1.0;
        tq_rot(w45, e, img);
        for (k = 0; k < 3; k++) m6[r * 4 + k] = (float)img[k];
    }
}

static double tq_vangle(const double a[3], const double b[3])
{
    return v_angle_deg(a, b);
}

static int t_adjust_frame_seam(void)
{
    enum { STRIDE = 0x180, JOINTS = 7 };
    static unsigned char blob[DG_OBJS_ARRAY + JOINTS * STRIDE];
    static union { ULONGLONG align; unsigned char b[0x240]; } actor;
    static union { ULONGLONG align; unsigned char b[0xD40]; } player;
    static unsigned char mc[0x70];
    static float adjust[55 * 4];
    ULONGLONG arm;
    ULONGLONG saved_anchor = g_b.a.gm_player_arm_body;
    LONG saved_frame = InterlockedCompareExchange(&g_b.adjust_frame, 0, 0);
    LONG saved_uproll = InterlockedCompareExchange(&g_b.arm_uproll, 0, 0);
    DG_BRIDGE_ARM_TARGET target;
    TQ_RIG rig;
    double q4_1[4], q5_1[4], W4[4], W5[4], f1[4], f2[4], fc[4], t[4];
    double exp4[4], exp5[4], got4[4], got5[4];
    double fore_a[3], fore_b[3], p6a[3], p6b[3];
    float miss;
    LONG lv, n0;
    int j, k, bad = 0, pb = 0;
#define TS_LEG(name) do { if (bad != pb) { \
        printf("    [seam leg %s] +%d\n", name, bad - pb); pb = bad; } \
    } while (0)

    memset(blob, 0, sizeof blob);
    memset(&actor, 0, sizeof actor);
    memset(&player, 0, sizeof player);
    memset(mc, 0, sizeof mc);
    memset(adjust, 0, sizeof adjust);
    arm = (ULONGLONG)(ULONG_PTR)(actor.b + 0x60);
    *(ULONGLONG *)(actor.b + 0x60) = (ULONGLONG)(ULONG_PTR)blob;
    *(ULONGLONG *)(actor.b + 0x68) = (ULONGLONG)(ULONG_PTR)mc;
    *(ULONGLONG *)(actor.b + 0x228) = (ULONGLONG)(ULONG_PTR)(player.b + 0xCF4);
    *(ULONGLONG *)(player.b + 0xBA8) = arm;
    *(LONG *)(player.b + 0xBB0) = 6;
    *(LONG *)(player.b + 0xB90) = 1;
    *(LONG *)(mc + 0x14) = 55;
    *(ULONGLONG *)(mc + 0x48) = (ULONGLONG)(ULONG_PTR)adjust;
    for (j = 0; j < JOINTS; j++) {
        float *m = (float *)(blob + DG_OBJS_ARRAY + j * STRIDE);
        m[0] = m[5] = m[10] = m[15] = 1.0f;
    }
    ((float *)(blob + DG_OBJS_ARRAY + 3 * STRIDE))[14] = 100.0f;
    for (j = 0; j < 55; j++) adjust[j * 4 + 3] = 1.0f;
    rig.p4[0] = rig.p4[1] = rig.p4[2] = 0.0;
    rig.upper[0] = 200.0; rig.upper[1] = 0.0; rig.upper[2] = 0.0;
    rig.fore[0] = 100.0; rig.fore[1] = 173.20508; rig.fore[2] = 0.0;
    tq_engine(blob, STRIDE, adjust, 0, &rig, NULL);

    g_b.a.gm_player_arm_body = (ULONGLONG)(ULONG_PTR)&arm;
    InterlockedExchange(&g_b.skel_stride, STRIDE);
    InterlockedExchange(&g_b.skel_parents_read, JOINTS);
    InterlockedExchange(&g_b.skel_region_end, (LONG)sizeof blob);
    for (j = 0; j < JOINTS; j++) InterlockedExchange(&g_b.skel_parents[j], 0);
    InterlockedExchange(&g_b.skel_parents[3], 2);
    InterlockedExchange(&g_b.skel_parents[4], 3);
    InterlockedExchange(&g_b.skel_parents[5], 4);
    InterlockedExchange(&g_b.skel_parents[6], 5);
    InterlockedExchange(&g_b.ik_active, 0);
    InterlockedExchange(&g_b.arm_freeze, 0);
    InterlockedExchange(&g_b.arm_uproll, 0);
    InterlockedExchange(&g_b.adjust_frame, 1);
    InterlockedExchange(&g_b.c_ticks, 0);
    InterlockedExchange(&g_b.s_wrist_miss_worst, f2l(0.0f));
    InterlockedExchange(&g_b.c_wrist_miss_over, 0);
    g_b.have_pred_wrist = 0;
    arm_map_forget();
    g_b.arm_map_phase = 0;
    g_b.arm_orient_have_prev = 0;

    memset(&target, 0, sizeof target);
    target.write = 1;
    target.weight = 1.0;
    target.stream_id = 9;
    target.pair_id = 300;
    target.wrist_view[0] = 250.0;
    target.wrist_view[1] = -150.0;
    target.wrist_view[2] = 100.0;
    target.hand_quat[3] = 1.0;
    target.head_yaw_valid = 1;

    /* Calibrate under heading 0 (rot.vy 0): the live frame is the identity
       there, and every pair must have acquired it - `missing` stays put. */
    *(short *)(player.b + 0x82) = 0;
    n0 = g_b.c_frame_missing;
    for (j = 0; j < 24 && !g_b.arm_map_cache_valid; j++) {
        InterlockedExchange(&g_b.c_ticks, (LONG)(DG_ADJ_SETTLE_TICKS + 1 + j));
        arm_ik_now(arm, &target);
        tq_engine(blob, STRIDE, adjust, 0, &rig, NULL);
        target.pair_id++;
    }
    if (!g_b.arm_map_cache_valid) bad++;
    if (!g_b.pair_frame.live || !g_b.pair_frame.valid) bad++;
    if (g_b.c_frame_missing != n0) bad++;
    TS_LEG("calibrate-live");

    /* Settle: a few more pairs at the same target so the solution stands. */
    for (j = 0; j < 4; j++) {
        InterlockedIncrement(&g_b.c_ticks);
        arm_ik_now(arm, &target);
        tq_engine(blob, STRIDE, adjust, 0, &rig, NULL);
        target.pair_id++;
    }
    for (k = 0; k < 4; k++) {
        q4_1[k] = (double)g_b.arm_map_cached_adjust[k];
        q5_1[k] = (double)g_b.arm_map_cached_adjust[4 + k];
    }
    /* The wrist-miss meter under the right frame: the engine puts the
       wrist where the solution predicted, to float precision. */
    InterlockedExchange(&g_b.s_wrist_miss_worst, f2l(0.0f));
    InterlockedIncrement(&g_b.c_ticks);
    arm_ik_now(arm, &target);
    tq_engine(blob, STRIDE, adjust, 0, &rig, NULL);
    target.pair_id++;
    InterlockedIncrement(&g_b.c_ticks);
    arm_ik_now(arm, &target);
    tq_engine(blob, STRIDE, adjust, 0, &rig, NULL);
    target.pair_id++;
    lv = g_b.s_wrist_miss_worst; memcpy(&miss, &lv, sizeof miss);
    if (!(miss < 0.5f)) bad++;
    TS_LEG("wrist-miss-live");

    /* strip leg (par. 6.2): known q4 = 20 about adjust-Z, q5 = 30 about
       adjust-Y in the slots AND the cache, the engine posing them under
       heading 73 (rot.vy -831 -> 73.04 degrees); the strip with that frame
       recovers the animation bones within 0.10 degrees, and the same call
       under the calibration heading (identity - yesterday's) misses by the
       hand-computed 14.30 (upper) / 22.79 (fore) degrees for THIS rig
       (scratchpad strip_check.py, 2026-09-01; asserted at >= 12 / >= 20). */
    {
        static const float kq4[4] = { 0.0f, 0.0f, 0.17364817766693033f, 0.98480775301220806f };
        static const float kq5[4] = { 0.0f, 0.25881904510252076f, 0.0f, 0.96592582628906829f };
        float saved_slots[8], saved_cache[8];
        double live[5][3], clean[5][3], up[3], fo[3];
        DG_ADJ_FRAME fr_ok, fr_stale;
        memcpy(saved_slots, adjust + 16, sizeof saved_slots);
        memcpy(saved_cache, g_b.arm_map_cached_adjust, sizeof saved_cache);
        memcpy(adjust + 16, kq4, sizeof kq4);
        memcpy(adjust + 20, kq5, sizeof kq5);
        memcpy(g_b.arm_map_cached_adjust, kq4, sizeof kq4);
        memcpy(g_b.arm_map_cached_adjust + 4, kq5, sizeof kq5);
        tq_engine(blob, STRIDE, adjust, -831, &rig, NULL);
        for (j = 2; j <= 6; j++) {
            const float *m = (const float *)(blob + DG_OBJS_ARRAY + j * STRIDE);
            for (k = 0; k < 3; k++) live[j - 2][k] = (double)m[12 + k];
        }
        memset(&fr_ok, 0, sizeof fr_ok);
        fr_ok.live = 1; fr_ok.valid = 1;
        arm_frame_yaw(-831, fr_ok.q);
        if (!arm_remove_cached_adjust(&fr_ok, live, g_b.arm_map_cached_adjust,
                                      clean)) bad++;
        for (k = 0; k < 3; k++) {
            up[k] = clean[3][k] - clean[2][k];
            fo[k] = clean[4][k] - clean[3][k];
        }
        if (tq_vangle(up, rig.upper) > 0.10) bad++;
        if (tq_vangle(fo, rig.fore) > 0.10) bad++;
        memset(&fr_stale, 0, sizeof fr_stale);
        fr_stale.live = 1; fr_stale.valid = 1;
        quat_identity(fr_stale.q);
        if (!arm_remove_cached_adjust(&fr_stale, live, g_b.arm_map_cached_adjust,
                                      clean)) bad++;
        for (k = 0; k < 3; k++) {
            up[k] = clean[3][k] - clean[2][k];
            fo[k] = clean[4][k] - clean[3][k];
        }
        if (!(tq_vangle(up, rig.upper) >= 12.0)) bad++;
        if (!(tq_vangle(fo, rig.fore) >= 20.0)) bad++;
        memcpy(adjust + 16, saved_slots, sizeof saved_slots);
        memcpy(g_b.arm_map_cached_adjust, saved_cache, sizeof saved_cache);
        tq_engine(blob, STRIDE, adjust, 0, &rig, NULL);
    }
    TS_LEG("strip");

    /* seam leg: the body turns 90 degrees (rot.vy 1024) with the slots
       untouched - the engine re-composes them under the new heading. The
       next pair strips with the NEW frame, recovers the same animation,
       solves the same world answer, and commits it re-expressed in the new
       frame: q_2 = f2* W f2. A strip under yesterday's heading would
       recover bent bones and commit something else. */
    tq_frame(0, f1);
    tq_frame(1024, f2);
    dg_ik_quat_mul(f1, q4_1, t); dg_ik_quat_conj(f1, fc); dg_ik_quat_mul(t, fc, W4);
    dg_ik_quat_mul(f1, q5_1, t); dg_ik_quat_mul(t, fc, W5);
    dg_ik_quat_conj(f2, fc);
    dg_ik_quat_mul(fc, W4, t); dg_ik_quat_mul(t, f2, exp4);
    dg_ik_quat_mul(fc, W5, t); dg_ik_quat_mul(t, f2, exp5);
    *(short *)(player.b + 0x82) = 1024;
    tq_engine(blob, STRIDE, adjust, 1024, &rig, NULL);
    InterlockedIncrement(&g_b.c_ticks);
    arm_ik_now(arm, &target);
    target.pair_id++;
    for (k = 0; k < 4; k++) {
        got4[k] = (double)g_b.arm_map_cached_adjust[k];
        got5[k] = (double)g_b.arm_map_cached_adjust[4 + k];
    }
    if (dg_ik_quat_angle(got4, exp4) * 180.0 / 3.14159265358979323846 > 0.5) bad++;
    if (dg_ik_quat_angle(got5, exp5) * 180.0 / 3.14159265358979323846 > 0.5) bad++;
    tq_engine(blob, STRIDE, adjust, 1024, &rig, NULL);
    TS_LEG("seam-turn");

    /* The 2026-08-30 failure on the desk: under legacy the constant is the
       frame of heading -90 (rot.vy 1024); at heading -180 (2048) it is 90
       degrees wrong. A CONVERGED wrong-frame loop hides that from every
       self-consistent meter (strip error and re-application cancel while
       q stands still), so the target MOVES each pair here: with the right
       frame the prediction stays exact whatever q does; with the wrong one
       the wrist lands a forearm's worth away. Live at the same heading, same
       moving target, stays exact. */
    {
        static const double tA[3] = { 250.0, -150.0, 100.0 };
        static const double tB[3] = { 150.0, -250.0,  50.0 };
        int i;
        *(short *)(player.b + 0x82) = 2048;
        tq_engine(blob, STRIDE, adjust, 2048, &rig, NULL);
        for (j = 0; j < 3; j++) {
            InterlockedIncrement(&g_b.c_ticks);
            arm_ik_now(arm, &target);
            tq_engine(blob, STRIDE, adjust, 2048, &rig, NULL);
            target.pair_id++;
        }
        InterlockedExchange(&g_b.s_wrist_miss_worst, f2l(0.0f));
        for (i = 0; i < 6; i++) {
            for (k = 0; k < 3; k++) target.wrist_view[k] = (i & 1) ? tB[k] : tA[k];
            InterlockedIncrement(&g_b.c_ticks);
            arm_ik_now(arm, &target);
            tq_engine(blob, STRIDE, adjust, 2048, &rig, NULL);
            target.pair_id++;
        }
        lv = g_b.s_wrist_miss_worst; memcpy(&miss, &lv, sizeof miss);
        if (!(miss < 0.5f)) bad++;
        TS_LEG("live-at-180");
        InterlockedExchange(&g_b.adjust_frame, 0);
        for (j = 0; j < 3; j++) {
            InterlockedIncrement(&g_b.c_ticks);
            arm_ik_now(arm, &target);
            tq_engine(blob, STRIDE, adjust, 2048, &rig, NULL);
            target.pair_id++;
        }
        InterlockedExchange(&g_b.s_wrist_miss_worst, f2l(0.0f));
        for (i = 0; i < 6; i++) {
            for (k = 0; k < 3; k++) target.wrist_view[k] = (i & 1) ? tB[k] : tA[k];
            InterlockedIncrement(&g_b.c_ticks);
            arm_ik_now(arm, &target);
            tq_engine(blob, STRIDE, adjust, 2048, &rig, NULL);
            target.pair_id++;
        }
        lv = g_b.s_wrist_miss_worst; memcpy(&miss, &lv, sizeof miss);
        if (!(miss > 50.0f)) bad++;
        TS_LEG("legacy-at-180");
        InterlockedExchange(&g_b.adjust_frame, 1);
        for (k = 0; k < 3; k++) target.wrist_view[k] = tA[k];
        for (j = 0; j < 4; j++) {
            InterlockedIncrement(&g_b.c_ticks);
            arm_ik_now(arm, &target);
            tq_engine(blob, STRIDE, adjust, 2048, &rig, NULL);
            target.pair_id++;
        }
    }

    /* replay/cache leg: constant replay writes the CACHE, bit for bit, and
       converts nothing; the engine under a heading 90 degrees on turns the
       world forearm by exactly that. Slot, cache and frame are three
       different things here: the slots are scribbled first, the cache is
       the truth replay must restore, and the frame moves underneath. */
    if (!g_b.arm_map_cache_valid || !g_b.ik_active) bad++;
    tq_engine(blob, STRIDE, adjust, 2048, &rig, p6a);
    for (k = 0; k < 8; k++) adjust[16 + k] = 0.123f * (float)(k + 1);
    if (!arm_ik_replay_cached(arm)) bad++;
    if (memcmp(adjust + 16, g_b.arm_map_cached_adjust, 8 * sizeof(float)) != 0)
        bad++;
    tq_engine(blob, STRIDE, adjust, 2048, &rig, p6a);
    {
        const float *m5 = (const float *)(blob + DG_OBJS_ARRAY + 5 * STRIDE);
        for (k = 0; k < 3; k++) fore_a[k] = p6a[k] - (double)m5[12 + k];
    }
    /* The body turns 90 degrees: in the engine the animation rig turns
       WITH the actor (it hangs off him) and so does the frame - the slots
       stay. Then the world forearm's heading turns by exactly the frame
       delta's yaw and its height does not change: the bone-vector oracle,
       not a quaternion delta. */
    {
        TQ_RIG turned = rig;
        double f3[4], f2c[4], delta[4], ha, hb, d;
        tq_frame(3072, f3);
        tq_frame(2048, f2c);
        dg_ik_quat_conj(f2c, fc);
        dg_ik_quat_mul(f3, fc, delta);
        tq_rot(delta, rig.upper, turned.upper);
        tq_rot(delta, rig.fore, turned.fore);
        tq_engine(blob, STRIDE, adjust, 3072, &turned, p6b);
        {
            const float *m5 = (const float *)(blob + DG_OBJS_ARRAY + 5 * STRIDE);
            for (k = 0; k < 3; k++) fore_b[k] = p6b[k] - (double)m5[12 + k];
        }
        ha = atan2(fore_a[2], fore_a[0]) * 180.0 / 3.14159265358979323846;
        hb = atan2(fore_b[2], fore_b[0]) * 180.0 / 3.14159265358979323846;
        d = hb - ha;
        while (d > 180.0) d -= 360.0;
        while (d < -180.0) d += 360.0;
        if (fabs(d + 90.0) > 0.01) bad++;
        if (fabs(fore_b[1] - fore_a[1]) > 0.01) bad++;
    }
    tq_engine(blob, STRIDE, adjust, 2048, &rig, NULL);
    TS_LEG("replay-cache");

    /* axis/cache leg: the roll's axis comes from the SLOTS, never the cache.
       The cache is scaled off-unit (same direction: the strip normalises it
       and is unharmed); the slots hold the truth. A read-back that
       trusted the cache would fail its own unit check and count a skip. */
    {
        LONG sk0;
        float saved_cache[8];
        memcpy(saved_cache, g_b.arm_map_cached_adjust, sizeof saved_cache);
        for (k = 0; k < 8; k++) g_b.arm_map_cached_adjust[k] *= 1.5f;
        InterlockedExchange(&g_b.arm_uproll, 1);
        sk0 = g_b.c_uproll_skipped;
        InterlockedIncrement(&g_b.c_ticks);
        arm_ik_now(arm, &target);
        tq_engine(blob, STRIDE, adjust, 2048, &rig, NULL);
        target.pair_id++;
        if (g_b.c_uproll_skipped != sk0) bad++;
        InterlockedExchange(&g_b.arm_uproll, 0);
        memcpy(g_b.arm_map_cached_adjust, saved_cache, sizeof saved_cache);
    }
    TS_LEG("axis-from-slots");

    /* Cached replay validity, independent of the solve's geometry oracle.
       Real solves warm the cache; then poison the actual slots/mask, change
       only current player ownership, and observe writes/release directly. */
    {
        static union { ULONGLONG align; unsigned char b[0x240]; } other_actor;
        static union { ULONGLONG align; unsigned char b[0xD40]; } other_player;
        const ULONGLONG bits = (1ULL << 4) | (1ULL << 5);
        const ULONGLONG foreign_bit = 1ULL << 9;
        ULONGLONG other_arm = (ULONGLONG)(ULONG_PTR)(other_actor.b + 0x60);
        ULONGLONG anchor = g_b.a.gm_player_arm_body;
        LONG freeze_saved = g_b.arm_freeze;
        int mode, loss;
        memset(&other_actor, 0, sizeof other_actor);
        memset(&other_player, 0, sizeof other_player);
        *(ULONGLONG *)(other_actor.b + 0x228) =
            (ULONGLONG)(ULONG_PTR)(other_player.b + 0xCF4);
        *(ULONGLONG *)(other_player.b + 0xBA8) = other_arm;
        *(LONG *)(other_player.b + 0xBB0) = 6;
        *(LONG *)(other_player.b + 0xB90) = 1;
        for (mode = 0; mode < 2; mode++) for (loss = 0; loss < 3; loss++) {
            DG_BRIDGE_ARM_TARGET replay_target;
            float cached[8], sentinels[8];
            DG_ADJ_FRAME saved_pair_frame;
            LONG writes_before, refused_before, missing_before;
            int start_bad = bad;
            g_b.a.gm_player_arm_body = anchor;
            *(LONG *)(player.b + 0xBB0) = 6;
            *(short *)(player.b + 0x82) = 2048;
            InterlockedExchange(&g_b.arm_freeze, 0);
            InterlockedExchange(&g_b.adjust_frame, 1);
            arm_map_forget();
            for (k = 0; k < 8; k++) adjust[16+k] = (k == 3 || k == 7) ? 1.0f : 0.0f;
            for (j = 0; j < 24 && !g_b.arm_map_cache_valid; j++) {
                tq_engine(blob, STRIDE, adjust, 2048, &rig, NULL);
                InterlockedIncrement(&g_b.c_ticks);
                target.pair_id++;
                arm_ik_now(arm, &target);
            }
            if (!g_b.arm_map_cache_valid || !g_b.ik_active) bad++;
            memcpy(cached, g_b.arm_map_cached_adjust, sizeof cached);
            replay_target = target;
            replay_target.pair_id = g_b.arm_map_last_pair + (mode ? 1 : 0);
            /* A pending hand command must be cleared by refusal/forget,
               not accidentally by the separate !hand_write guard. */
            replay_target.hand_write = 1;
            InterlockedExchange(&g_b.arm_freeze, mode);
            saved_pair_frame = g_b.pair_frame;
            *(short *)(player.b + 0x82) = 3072;
            for (k = 0; k < 8; k++) adjust[16+k] = .125f*(k+1);
            *(ULONGLONG *)(mc + 0x38) = foreign_bit;
            writes_before = g_b.c_arm_tracked;
            arm_ik_now(arm, &replay_target);
            if (memcmp(adjust + 16, cached, sizeof cached) ||
                *(ULONGLONG *)(mc + 0x38) != (foreign_bit | bits) ||
                g_b.c_arm_tracked != writes_before + 1 ||
                memcmp(&g_b.pair_frame, &saved_pair_frame, sizeof saved_pair_frame)) bad++;

            for (k = 0; k < 8; k++) sentinels[k] = adjust[16+k] = .25f*(k+1);
            *(ULONGLONG *)(mc + 0x38) = foreign_bit;
            InterlockedExchange(&g_b.hand_command_valid, 1);
            if (mode) replay_target.pair_id++;
            if (loss == 0) *(LONG *)(player.b + 0xBB0) = 5;
            else if (loss == 1) g_b.a.gm_player_arm_body = 0;
            else g_b.a.gm_player_arm_body = (ULONGLONG)(ULONG_PTR)&other_arm;
            writes_before = g_b.c_arm_tracked;
            refused_before = g_b.c_arm_ik_refused;
            missing_before = g_b.c_frame_missing;
            arm_ik_now(arm, &replay_target);
            if (g_b.c_arm_tracked != writes_before ||
                g_b.c_arm_ik_refused != refused_before + 1 ||
                g_b.c_frame_missing != missing_before + 1 ||
                g_b.arm_map_cache_valid || g_b.ik_active ||
                g_b.hand_command_valid || g_b.arm_map_phase) bad++;
            if (*(ULONGLONG *)(mc + 0x38) != foreign_bit) bad++;
            if (loss == 0) {
                /* Same adjust owner: release to identity is permitted. */
                static const float identity_pair[8] = {0,0,0,1,0,0,0,1};
                if (memcmp(adjust + 16, identity_pair, sizeof identity_pair)) bad++;
            } else if (memcmp(adjust + 16, sentinels, sizeof sentinels)) {
                /* Missing/replaced anchor: never touch the old slots. */
                bad++;
            }
            g_b.a.gm_player_arm_body = anchor;
            *(LONG *)(player.b + 0xBB0) = 6;
            *(short *)(player.b + 0x82) = 2048;
            InterlockedExchange(&g_b.arm_freeze, 0);
            /* The same pair cannot resurrect the cache on return. */
            replay_target.hand_write = 0;
            arm_ik_now(arm, &replay_target);
            if (g_b.arm_map_cache_valid || g_b.ik_active || g_b.arm_map_phase != 1) bad++;
            /* No ownership bits means a real engine ignores old slot bytes.
               This small test-engine has no mask input, so give it the
               equivalent identity slots before the resumed hierarchy pass. */
            for (k = 0; k < 8; k++) adjust[16+k] = (k == 3 || k == 7) ? 1.0f : 0.0f;
            for (j = 0; j < 24 && !g_b.arm_map_cache_valid; j++) {
                tq_engine(blob, STRIDE, adjust, 2048, &rig, NULL);
                InterlockedIncrement(&g_b.c_ticks);
                replay_target.pair_id++;
                arm_ik_now(arm, &replay_target);
            }
            if (!g_b.arm_map_cache_valid || !g_b.ik_active) bad++;
            if (bad != start_bad)
                printf("    replay validity %s loss=%d: %d failures\n",
                       mode ? "freeze" : "duplicate", loss, bad-start_bad);
            target.pair_id = replay_target.pair_id + 1;
        }
        /* Compatibility control: legacy deliberately has no live heading
           dependency. Its valid cached replay may work without a resolver. */
        arm_map_forget();
        *(short *)(player.b + 0x82) = 2048;
        for (j = 0; j < 24 && !g_b.arm_map_cache_valid; j++) {
            tq_engine(blob, STRIDE, adjust, 2048, &rig, NULL);
            InterlockedIncrement(&g_b.c_ticks);
            target.pair_id++;
            arm_ik_now(arm, &target);
        }
        if (!g_b.arm_map_cache_valid) bad++;
        {
            DG_BRIDGE_ARM_TARGET legacy_target = target;
            LONG tracked = g_b.c_arm_tracked;
            legacy_target.pair_id = g_b.arm_map_last_pair;
            InterlockedExchange(&g_b.adjust_frame, 0);
            g_b.a.gm_player_arm_body = 0;
            arm_ik_now(arm, &legacy_target);
            if (g_b.c_arm_tracked != tracked + 1 || !g_b.arm_map_cache_valid) bad++;
        }
        g_b.a.gm_player_arm_body = anchor;
        InterlockedExchange(&g_b.adjust_frame, 1);
        InterlockedExchange(&g_b.arm_freeze, freeze_saved);
        target.pair_id++;
    }
    TS_LEG("live-replay-validity-and-recovery");

    /* A pair with no resolvable player under live has no frame: it refuses
       (missing counts), and nothing is written. */
    {
        LONG acc0 = g_b.c_arm_pairs_accepted, ref0 = g_b.c_arm_ik_refused;
        n0 = g_b.c_frame_missing;
        *(LONG *)(player.b + 0xBB0) = 5;             /* owner check fails */
        InterlockedIncrement(&g_b.c_ticks);
        arm_ik_now(arm, &target);
        target.pair_id++;
        if (g_b.c_frame_missing != n0 + 1) bad++;
        if (g_b.c_arm_pairs_accepted != acc0) bad++;
        if (g_b.c_arm_ik_refused == ref0) bad++;
        *(LONG *)(player.b + 0xBB0) = 6;
    }
    TS_LEG("no-heading-refuses");

    /* Fades and pose loss must not reach the meter (runs 9/10, 2026-09-02:
       worst 531-587 mm, every one of them the headset being put down). The
       engine here is moved by hand after each pose - the stand-in for the
       runtime blending the arm elsewhere while the pair is faded. */
    {
        float *wrist = (float *)(blob + DG_OBJS_ARRAY + 6 * STRIDE);
        InterlockedExchange(&g_b.adjust_frame, 1);
        *(short *)(player.b + 0x82) = 2048;
        /* The headingless refusal above forgot the map: calibrate again and
           settle, so a prediction actually stands when the fade arrives. */
        for (j = 0; j < 24 && !g_b.arm_map_cache_valid; j++) {
            InterlockedIncrement(&g_b.c_ticks);
            arm_ik_now(arm, &target);
            tq_engine(blob, STRIDE, adjust, 2048, &rig, NULL);
            target.pair_id++;
        }
        if (!g_b.arm_map_cache_valid) bad++;
        for (j = 0; j < 4; j++) {
            InterlockedIncrement(&g_b.c_ticks);
            arm_ik_now(arm, &target);
            tq_engine(blob, STRIDE, adjust, 2048, &rig, NULL);
            target.pair_id++;
        }
        InterlockedExchange(&g_b.s_wrist_miss_worst, f2l(0.0f));
        InterlockedExchange(&g_b.c_wrist_miss_over, 0);
        /* A full pair predicts; a plausible faded pair leaves NO prediction
           behind (a 300 mm wrist would be refused as implausible before it
           could write, so this pair reads the engine's own wrist). */
        InterlockedIncrement(&g_b.c_ticks);
        arm_ik_now(arm, &target);
        tq_engine(blob, STRIDE, adjust, 2048, &rig, NULL);
        target.pair_id++;
        if (!g_b.have_pred_wrist) bad++;
        target.weight = 0.5;
        InterlockedIncrement(&g_b.c_ticks);
        arm_ik_now(arm, &target);
        tq_engine(blob, STRIDE, adjust, 2048, &rig, NULL);
        target.pair_id++;
        if (g_b.have_pred_wrist) bad++;
        /* Then a full pair predicts, the engine lands the wrist 300 mm
           away, and the faded pair that reads it grades nothing; the full
           pair after it has nothing to grade either. */
        target.weight = 1.0;
        InterlockedIncrement(&g_b.c_ticks);
        arm_ik_now(arm, &target);
        tq_engine(blob, STRIDE, adjust, 2048, &rig, NULL);
        target.pair_id++;
        if (!g_b.have_pred_wrist) bad++;
        wrist[12] += 300.0f;
        target.weight = 0.5;
        InterlockedIncrement(&g_b.c_ticks);
        arm_ik_now(arm, &target);
        tq_engine(blob, STRIDE, adjust, 2048, &rig, NULL);
        target.pair_id++;
        wrist[12] += 300.0f;
        target.weight = 1.0;
        InterlockedIncrement(&g_b.c_ticks);
        arm_ik_now(arm, &target);
        tq_engine(blob, STRIDE, adjust, 2048, &rig, NULL);
        target.pair_id++;
        lv = g_b.s_wrist_miss_worst; memcpy(&miss, &lv, sizeof miss);
        if (!(miss < 0.5f)) bad++;
        if (g_b.c_wrist_miss_over != 0) bad++;
        TS_LEG("fade-not-graded");

        /* Pose loss: a full pair predicts, the pose goes (weight 0 forgets
           the map), the wrist is elsewhere when tracking returns, and the
           recalibration's first read must not grade the dead prediction.
           The 600 mm the previous leg moved the wrist made its last pair
           implausible (the map forgot itself), so calibrate again first. */
        for (j = 0; j < 24 && !g_b.arm_map_cache_valid; j++) {
            InterlockedIncrement(&g_b.c_ticks);
            arm_ik_now(arm, &target);
            tq_engine(blob, STRIDE, adjust, 2048, &rig, NULL);
            target.pair_id++;
        }
        for (j = 0; j < 4; j++) {
            InterlockedIncrement(&g_b.c_ticks);
            arm_ik_now(arm, &target);
            tq_engine(blob, STRIDE, adjust, 2048, &rig, NULL);
            target.pair_id++;
        }
        InterlockedExchange(&g_b.s_wrist_miss_worst, f2l(0.0f));
        InterlockedExchange(&g_b.c_wrist_miss_over, 0);
        InterlockedIncrement(&g_b.c_ticks);
        arm_ik_now(arm, &target);
        tq_engine(blob, STRIDE, adjust, 2048, &rig, NULL);
        target.pair_id++;
        if (!g_b.have_pred_wrist) {
            printf("    pose-loss: the full pair before the loss predicted "
                   "nothing (cache %ld)\n", (long)g_b.arm_map_cache_valid);
            bad++;
        }
        target.weight = 0.0;
        InterlockedIncrement(&g_b.c_ticks);
        arm_ik_now(arm, &target);
        target.pair_id++;
        if (g_b.have_pred_wrist) {
            printf("    pose-loss: the prediction survived the loss\n");
            bad++;
        }
        target.weight = 1.0;
        wrist[12] += 300.0f;
        /* The first read after tracking returns is the one that would grade
           the dead prediction against a wrist that moved 300 mm meanwhile. */
        InterlockedIncrement(&g_b.c_ticks);
        arm_ik_now(arm, &target);
        tq_engine(blob, STRIDE, adjust, 2048, &rig, NULL);
        target.pair_id++;
        lv = g_b.s_wrist_miss_worst; memcpy(&miss, &lv, sizeof miss);
        if (!(miss < 0.5f)) {
            printf("    pose-loss: the dead prediction was graded, %.1f mm\n",
                   (double)miss);
            bad++;
        }
        for (j = 0; j < 24 && !g_b.arm_map_cache_valid; j++) {
            InterlockedIncrement(&g_b.c_ticks);
            arm_ik_now(arm, &target);
            tq_engine(blob, STRIDE, adjust, 2048, &rig, NULL);
            target.pair_id++;
        }
        if (!g_b.arm_map_cache_valid) {
            printf("    pose-loss: no recalibration within 24 pairs\n");
            bad++;
        }
        /* Calibration-time reads are not this meter's claim. */
        InterlockedExchange(&g_b.s_wrist_miss_worst, f2l(0.0f));
        InterlockedExchange(&g_b.c_wrist_miss_over, 0);
        TS_LEG("pose-loss-drops-prediction");

        /* And the meter is still a meter: a full pair predicts, the engine
           misplaces the wrist, the next full pair grades it. */
        for (j = 0; j < 24 && !g_b.arm_map_cache_valid; j++) {
            InterlockedIncrement(&g_b.c_ticks);
            arm_ik_now(arm, &target);
            tq_engine(blob, STRIDE, adjust, 2048, &rig, NULL);
            target.pair_id++;
        }
        for (j = 0; j < 4; j++) {
            InterlockedIncrement(&g_b.c_ticks);
            arm_ik_now(arm, &target);
            tq_engine(blob, STRIDE, adjust, 2048, &rig, NULL);
            target.pair_id++;
        }
        InterlockedExchange(&g_b.s_wrist_miss_worst, f2l(0.0f));
        InterlockedExchange(&g_b.c_wrist_miss_over, 0);
        InterlockedIncrement(&g_b.c_ticks);
        arm_ik_now(arm, &target);
        tq_engine(blob, STRIDE, adjust, 2048, &rig, NULL);
        target.pair_id++;
        if (!g_b.have_pred_wrist) bad++;
        wrist[12] += 300.0f;
        InterlockedIncrement(&g_b.c_ticks);
        arm_ik_now(arm, &target);
        tq_engine(blob, STRIDE, adjust, 2048, &rig, NULL);
        target.pair_id++;
        lv = g_b.s_wrist_miss_worst; memcpy(&miss, &lv, sizeof miss);
        if (!(miss > 100.0f)) bad++;
        if (g_b.c_wrist_miss_over != 1) bad++;
        TS_LEG("meter-still-grades");
    }

    arm_map_forget();
    InterlockedExchange(&g_b.ik_active, 0);
    InterlockedExchange(&g_b.adjust_frame, saved_frame);
    InterlockedExchange(&g_b.arm_uproll, saved_uproll);
    g_b.a.gm_player_arm_body = saved_anchor;
    InterlockedExchange(&g_b.skel_stride, 0);
    InterlockedExchange(&g_b.skel_parents_read, 0);
    InterlockedExchange(&g_b.skel_region_end, 0);
    InterlockedExchange(&g_b.c_ticks, 0);
    g_b.have_pred_wrist = 0;
    InterlockedExchange(&g_b.s_wrist_miss_worst, f2l(0.0f));
    InterlockedExchange(&g_b.c_wrist_miss_over, 0);
    g_b.pair_frame = ADJ_FRAME_LEGACY;
#undef TS_LEG

    printf("  %-6s adjust frame seam: through the real arm_ik_now against a "
           "test-engine that composes the slots under the measured frame - "
           "calibrates live with no missing heading, the wrist lands where the "
           "solution predicted (<0.5 mm), the strip recovers the animation "
           "under its own heading (0.10 deg) and misses by 14/23 deg under "
           "yesterday's, a 90-degree body turn re-expresses the same world "
           "answer in the new frame (0.5 deg), legacy at heading -180 with a "
           "moving target misses the wrist by >50 mm where live stays under "
           "0.5 mm, constant replay writes "
           "the cache bit for bit and the engine turns the forearm exactly 90 "
           "under the next heading, the roll reads the slots not the cache, "
           "a headingless pair refuses, and neither a faded pair nor a pose "
           "loss reaches the wrist meter while a real miss still does\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

#include "dg_camera_bob_test.inl"
#include "dg_view_calibration_test.inl"

static int t_camera_gate_now(void)
{
    DG_ANCHORS saved_anchors = g_b.a;
    DG_FPS_STATE saved_fps = g_b.fps;
    LONG saved_armed = InterlockedCompareExchange(&g_b.armed, 0, 0);
    int saved_owner = g_b.owner;
    volatile LONG native_active = 1;
    volatile ULONGLONG player_status = 0;
    volatile LONG game_status = 0, game_status_scn = 0;
    volatile LONG menu_status = 0, menu_status_scn = 0;
    volatile ULONGLONG arm_value;
    unsigned char arm_mem[0x400], camera_mem[0x40];
    DG_CAMERA_GATE out;
    SYSTEM_INFO si;
    void *guard = NULL;
    DWORD old_protect;
    int bad = 0;

    memset(arm_mem, 0, sizeof arm_mem);
    memset(camera_mem, 0, sizeof camera_mem);
    arm_value = (ULONGLONG)(ULONG_PTR)(arm_mem + 0x100);
    *(volatile ULONGLONG *)(ULONG_PTR)(arm_value - 0x60 + 0x248) =
        (ULONGLONG)(ULONG_PTR)camera_mem;
    *(volatile LONG *)(ULONG_PTR)((ULONGLONG)(ULONG_PTR)camera_mem + 0x2C) = 1;

    memset(&g_b.a, 0, sizeof g_b.a);
    g_b.a.gbp_active = (ULONGLONG)(ULONG_PTR)&native_active;
    g_b.a.gm_player_status = (ULONGLONG)(ULONG_PTR)&player_status;
    g_b.a.gm_game_status = (ULONGLONG)(ULONG_PTR)&game_status;
    g_b.a.gm_game_status_scn = (ULONGLONG)(ULONG_PTR)&game_status_scn;
    g_b.a.gm_menu_status = (ULONGLONG)(ULONG_PTR)&menu_status;
    g_b.a.gm_menu_status_scn = (ULONGLONG)(ULONG_PTR)&menu_status_scn;
    g_b.a.gm_player_arm_body = (ULONGLONG)(ULONG_PTR)&arm_value;
    InterlockedExchange(&g_b.armed, 1);
    g_b.fps.state = DG_FPS_ACTIVE;
    g_b.owner = 0;

    /* Button context is raw and mode-independent. A must also enter FPS;
       theater rendering settings must not grant permission in a menu. */
    {
        LONG saved_request=g_b.toggle_request, saved_request_ms=g_b.toggle_request_ms;
        if (!dg_bridge_controller_gameplay_now()) bad++;
        g_b.fps.state=DG_FPS_OFF;
        native_active=0;
        if (!dg_bridge_controller_gameplay_now()) bad++;
        g_b.fps.state=DG_FPS_ACTIVE; native_active=1;
        menu_status=DG_MENU_UNSAFE_MASK;
        if (dg_bridge_controller_gameplay_now()) bad++;
        menu_status=0; menu_status_scn=DG_MENU_UNSAFE_MASK;
        if (dg_bridge_controller_gameplay_now()) bad++;
        menu_status_scn=0; game_status=DG_GAME_UNSAFE_MASK;
        if (dg_bridge_controller_gameplay_now()) bad++;
        game_status=0; game_status_scn=DG_GAME_UNSAFE_MASK;
        if (dg_bridge_controller_gameplay_now()) bad++;
        game_status_scn=0; player_status=DG_PLAYER_UNSAFE_MASK;
        if (dg_bridge_controller_gameplay_now()) bad++;
        player_status=0;
        { ULONGLONG saved_arm=arm_value;
          arm_value=0;
          if (dg_bridge_controller_gameplay_now()) bad++;
          arm_value=saved_arm;
        }
        dg_bridge_request_toggle();
        if (!controller_toggle_pending(1,(DWORD)g_b.toggle_request_ms)) bad++;
        if (controller_toggle_pending(0,(DWORD)g_b.toggle_request_ms) || g_b.toggle_request) bad++;
        if (controller_toggle_pending(1,(DWORD)g_b.toggle_request_ms)) bad++;
        dg_bridge_request_toggle();
        if (controller_toggle_pending(1,(DWORD)g_b.toggle_request_ms+101) || g_b.toggle_request) bad++;
        dg_bridge_request_toggle(); dg_bridge_cancel_toggle();
        if (g_b.toggle_request) bad++;
        g_b.toggle_request=saved_request; g_b.toggle_request_ms=saved_request_ms;
    }

    if (!dg_bridge_camera_gate_now(&out) || !out.valid || !out.native_active ||
        !out.safe_gameplay || !out.arm_camera_on || out.arm_body != arm_value ||
        out.camera != (ULONGLONG)(ULONG_PTR)camera_mem)
        bad++;

    /* The resolved work record must advertise channel zero and a live camera
       object. Each refusal below exercises the corresponding bounded walk. */
    *(volatile LONG *)(ULONG_PTR)(arm_value - 0x60 + 0x238) = 1;
    if (dg_bridge_camera_gate_now(&out)) bad++;
    *(volatile LONG *)(ULONG_PTR)(arm_value - 0x60 + 0x238) = 0;
    *(volatile ULONGLONG *)(ULONG_PTR)(arm_value - 0x60 + 0x248) = 0;
    if (dg_bridge_camera_gate_now(&out)) bad++;
    *(volatile ULONGLONG *)(ULONG_PTR)(arm_value - 0x60 + 0x248) =
        (ULONGLONG)(ULONG_PTR)camera_mem;

    /* Native inactivity and every unsafe status word must stop before the
       arm walk, while still publishing the words that explain the refusal. */
    native_active = 0;
    if (dg_bridge_camera_gate_now(&out) || out.native_active ||
        !out.safe_gameplay || out.arm_body || out.camera)
        bad++;
    native_active = 1;

    player_status = DG_PLAYER_UNSAFE_MASK;
    if (dg_bridge_camera_gate_now(&out) || out.safe_gameplay)
        bad++;
    player_status = 0;
    game_status = DG_GAME_UNSAFE_MASK;
    if (dg_bridge_camera_gate_now(&out) || out.safe_gameplay ||
        out.game_status != (unsigned int)game_status)
        bad++;
    game_status = 0;
    game_status_scn = DG_GAME_UNSAFE_MASK;
    if (dg_bridge_camera_gate_now(&out) || out.safe_gameplay ||
        out.game_status != (unsigned int)game_status_scn)
        bad++;
    game_status_scn = 0;
    menu_status = DG_MENU_UNSAFE_MASK;
    if (dg_bridge_camera_gate_now(&out) || out.safe_gameplay ||
        out.menu_status != (unsigned int)menu_status)
        bad++;
    menu_status = 0;
    menu_status_scn = DG_MENU_UNSAFE_MASK;
    if (dg_bridge_camera_gate_now(&out) || out.safe_gameplay ||
        out.menu_status != (unsigned int)menu_status_scn)
        bad++;
    menu_status_scn = 0;

    *(volatile LONG *)(ULONG_PTR)((ULONGLONG)(ULONG_PTR)camera_mem + 0x2C) = 0;
    if (dg_bridge_camera_gate_now(&out) || !out.safe_gameplay || out.arm_camera_on)
        bad++;
    *(volatile LONG *)(ULONG_PTR)((ULONGLONG)(ULONG_PTR)camera_mem + 0x2C) = 1;

    /* Session and ownership gates retain their diagnostics and do not touch
       the anchors at all. */
    g_b.owner = 1;
    if (dg_bridge_camera_gate_now(&out) || !out.armed || !out.fps_active)
        bad++;
    g_b.owner = 0;
    InterlockedExchange(&g_b.armed, 0);
    if (dg_bridge_camera_gate_now(&out) || out.armed || !out.fps_active)
        bad++;
    InterlockedExchange(&g_b.armed, 1);
    g_b.fps.state = DG_FPS_OFF;
    if (dg_bridge_camera_gate_now(&out) || !out.armed || out.fps_active)
        bad++;
    g_b.fps.state = DG_FPS_ACTIVE;

    /* Null, no-access, and a committed span too short for the requested read
       all fail closed without relying on an exception handler. */
    g_b.a.gm_game_status = 0;
    if (dg_bridge_camera_gate_now(&out)) bad++;
    g_b.a.gm_game_status = (ULONGLONG)(ULONG_PTR)&game_status;
    guard = VirtualAlloc(NULL, 0x1000, MEM_RESERVE | MEM_COMMIT,
                         PAGE_NOACCESS);
    if (!guard) {
        bad++;
    } else {
        g_b.a.gm_game_status = (ULONGLONG)(ULONG_PTR)guard;
        if (dg_bridge_camera_gate_now(&out)) bad++;
        g_b.a.gm_game_status = (ULONGLONG)(ULONG_PTR)&game_status;
    }
    if (guard) VirtualFree(guard, 0, MEM_RELEASE);

    GetSystemInfo(&si);
    guard = VirtualAlloc(NULL, si.dwPageSize * 2, MEM_RESERVE | MEM_COMMIT,
                         PAGE_READWRITE);
    if (!guard || !VirtualProtect((unsigned char *)guard + si.dwPageSize,
                                  si.dwPageSize, PAGE_NOACCESS, &old_protect)) {
        if (guard) VirtualFree(guard, 0, MEM_RELEASE);
        bad++;
    } else {
        g_b.a.gm_game_status = (ULONGLONG)(ULONG_PTR)
            ((unsigned char *)guard + si.dwPageSize - 2);
        if (dg_bridge_camera_gate_now(&out)) bad++;
        VirtualProtect((unsigned char *)guard + si.dwPageSize, si.dwPageSize,
                       old_protect, &old_protect);
        VirtualFree(guard, 0, MEM_RELEASE);
        g_b.a.gm_game_status = (ULONGLONG)(ULONG_PTR)&game_status;
    }

    guard = VirtualAlloc(NULL, si.dwPageSize, MEM_RESERVE | MEM_COMMIT,
                         PAGE_READWRITE);
    if (!guard) {
        bad++;
    } else {
        arm_value = (ULONGLONG)(ULONG_PTR)
            ((unsigned char *)guard + si.dwPageSize - 8);
        if (dg_bridge_camera_gate_now(&out)) bad++;
        VirtualFree(guard, 0, MEM_RELEASE);
        arm_value = (ULONGLONG)(ULONG_PTR)(arm_mem + 0x100);
    }
    g_b.a.gm_player_arm_body = 0;
    if (dg_bridge_camera_gate_now(&out)) bad++;
    g_b.a.gm_player_arm_body = (ULONGLONG)(ULONG_PTR)&arm_value;

    g_b.a = saved_anchors;
    g_b.fps = saved_fps;
    g_b.owner = saved_owner;
    InterlockedExchange(&g_b.armed, saved_armed);
    printf("  %-6s camera gate: real ACTIVE/owned/safe channel-0 arm-camera "
           "passes; native-inactive, owner, lifecycle, player/game/menu "
           "including SCN, camera-off, null, no-access and short-span cells "
           "refuse with bounded reads\n", bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

#include "dg_position_bridge_test.h"
#include "dg_left_arm_test.inl"
#include "dg_left_model_test.inl"
#include "dg_unarmed_right_test.inl"
#include "dg_hand_profile_test.inl"
#include "dg_unarmed_prone_test.inl"
#include "dg_hand_pose_test.inl"
#include "dg_interact_bridge_test.inl"
#include "dg_hanging_visibility_test.inl"

#include "dg_native_hud_fixture.h"
static int t_native_hud_relocation(void) {
    const unsigned char *data[]={hud_fixture_life,hud_fixture_frame,hud_fixture_coolant};
    size_t lengths[]={sizeof hud_fixture_life,sizeof hud_fixture_frame,sizeof hud_fixture_coolant};
    size_t offsets[]={0x5769da-0x576730,0x11e083-0x11df40,0x51f9b1-0x51f880};
    unsigned char *code;DG_DETOUR detour;const char *why;int i,bad=0;
    for(i=0;i<3;i++) {
        code=VirtualAlloc(NULL,4096,MEM_RESERVE|MEM_COMMIT,PAGE_EXECUTE_READWRITE);
        if(!code){bad++;continue;}
        memcpy(code,data[i],lengths[i]);
        if(!dg_detour_install_ex(&detour,code+offsets[i],native_hud_life,code,code+lengths[i],&why,1)) {
            printf("  FAIL native hook %d relocation: %s\n",i,why);bad++;
        } else {
            dg_detour_remove(&detour);
            if(memcmp(code,data[i],lengths[i]))bad++;
        }
        VirtualFree(code,0,MEM_RELEASE);
    }
    printf("  %s native HUD/coolant: all three retail seams install and restore\n",bad?"FAIL":"ok");
    return bad;
}
static int t_coolant_pad_alignment(void) {
    __declspec(align(8)) unsigned words[4]={0,0,0x80,0};
    uint64_t p=(uint64_t)(ULONG_PTR)&words[1];int bad=0;
    if((p&7)!=4 || coolant_pad_status(p)!=0x80)bad++;
    if(coolant_pad_status(0)!=0xffffffffu || coolant_pad_status(p+1)!=0xffffffffu)bad++;
    printf("  %s coolant pad: real four-byte alignment readable; null/misaligned rejected\n",bad?"FAIL":"ok");
    return bad;
}
static int t_native_hud_selective(void) {
    unsigned char gauge[0x60]={0},work[0x790]={0},sprites[5][0x40]={0};
    uint64_t regs[16]={0};unsigned prim=0x22;int i,bad=0;
    LONG armed=g_b.armed,late=g_b.s_late_unsafe,menu=g_b.script_menu_only,requested=g_native_hud_requested;
    int fps=g_b.fps.state,live=g_native_hud_live;
    g_b.armed=1;g_b.s_late_unsafe=0;g_b.script_menu_only=0;g_b.fps.state=DG_FPS_ACTIVE;
    g_native_hud_requested=g_native_hud_live=1;
    *(uint64_t *)(gauge+0x38)=(uint64_t)(ULONG_PTR)&prim;regs[11]=(uint64_t)(ULONG_PTR)gauge;
    native_hud_life(regs+16);if(prim!=0x122)bad++;
    prim=0x22;*(short *)(gauge+0x24)=1;native_hud_life(regs+16);if(prim!=0x22)bad++;
    *(short *)(gauge+0x24)=0;
    for(i=0;i<5;i++){*(unsigned *)(sprites[i]+0x30)=0x42;*(uint64_t *)(work+0x768+i*8)=(uint64_t)(ULONG_PTR)sprites[i];}
    regs[11]=(uint64_t)(ULONG_PTR)work;native_hud_frame(regs+16);
    for(i=0;i<5;i++)if(*(unsigned *)(sprites[i]+0x30)!=(i?0x8042:0x42))bad++;
    for(i=0;i<5;i++) {
        prim=0x22;regs[11]=(uint64_t)(ULONG_PTR)gauge;
        g_native_hud_requested=i!=0;g_b.armed=i!=1;g_b.s_late_unsafe=i==2;
        g_b.script_menu_only=i==3;g_b.fps.state=i==4?0:DG_FPS_ACTIVE;
        native_hud_life(regs+16);if(prim!=0x22)bad++;
        *(unsigned *)(sprites[1]+0x30)=0x42;regs[11]=(uint64_t)(ULONG_PTR)work;
        native_hud_frame(regs+16);if(*(unsigned *)(sprites[1]+0x30)!=0x42)bad++;
    }
    g_b.armed=armed;g_b.s_late_unsafe=late;g_b.script_menu_only=menu;g_b.fps.state=fps;
    g_native_hud_requested=requested;g_native_hud_live=live;
    printf("  %s native HUD: LIFE only, exactly four frame sprites, inactive gates\n",bad?"FAIL":"ok");
    return bad;
}

#include "dg_menu_recovery_test.inl"
int dg_bridge_self_test(void)
{
    int bad = 0;
    bad += t_native_hud_relocation();
    bad += t_native_hud_selective();
    bad += t_coolant_pad_alignment();
    bad += t_twohand_latch();
    bad += t_interact_native_writer();
    bad += t_hanging_visibility();
    bad += t_interact_codec_direct();
    bad += t_left_seam();
    bad += t_left_model_transport();
    bad += t_unarmed_right();
    bad += t_persistent_hand_profile();
    bad += t_unarmed_prone_tables();
    bad += t_hand_pose_mirror();
    bad += t_hand_pose_mirror_heading();
    bad += t_camera_pair_telemetry();
    bad += t_cutscene_reaches_the_safety_gate();
    bad += t_adjust_frame();
    bad += t_adjust_frame_seam();
    bad += t_skeleton_is_measured_not_assumed();
    bad += t_active_seam_rate_uses_active_ticks();
    bad += t_adjust_probe_holds_a_case_for_a_whole_pass();
    bad += t_bend_writes_one_joint_behind_its_gate();
    bad += t_ik_glue_uses_measured_adjust_space();
    bad += t_grip_roll();
    bad += t_arm_mapping_is_once_per_pair();
    bad += t_arm_freeze_replays_one_pair_and_never_solves();
    bad += t_hand_net_meter_separates_winding_from_shaking();
    bad += t_rest_freeze_breaks_the_strip_feedback_loop();
    bad += t_the_gap_reads_the_facing_minus_the_body();
    bad += t_writer_stands_down_on_the_seams_own_reading();
    bad += t_set_pos_quat_matches_the_game();
    bad += t_hand_probe_owns_and_returns_the_svector();
    bad += t_hand_tick_writer_is_fail_closed();
    bad += t_release_gives_the_wrist_back();
    bad += t_the_two_halves_of_the_conversion_agree();
    bad += t_the_trigger_crosses_the_seams();
    bad += t_walking_speaks_only_over_silence();
    bad += t_walking_in_third_person_and_prone();
    bad += t_body_follow_speaks_for_the_aim();
    bad += t_press_order_keeps_presses_in_order();
    bad += t_menu_seam_writes_only_where_it_may();
    bad += t_xr_menu_recovery();
    bad += t_script_menu_context_guard();
    bad += t_the_trigger_only_ever_adds();
    bad += t_the_kick_is_ours_and_comes_home();
    bad += t_hand_follows_the_controller();
    bad += t_the_arm_ignores_the_bodys_yaw();
    bad += t_probe_the_walking_turn();
    bad += t_off_never_writes();
    bad += t_toggle_ten_cycles();
    bad += t_native_fps_is_left_alone();
    bad += t_engine_leaving_is_not_a_fight();
    bad += t_move_is_borrowed_only_on_request();
    bad += t_claim_is_released_when_idle();
    bad += t_idle_is_not_a_second_owner();
    bad += t_player_status_mask();
    bad += t_game_status_mask();
    bad += t_menu_status_mask();
    bad += t_theater_mask_is_narrower_than_the_gate();
    bad += t_theater_hysteresis_holds_both_flanks();
    bad += t_theater_verdict_is_one_publication_fail_closed();
    bad += t_hud_hide_borrows_the_games_own_bits();
    bad += t_always_suspend_resume();
    bad += t_no_edge_without_toggle();
    bad += t_restore_once();
    bad += t_decoder();
    bad += t_relocation();
    bad += t_detour_end_to_end();
    bad += t_detour_refusals();
    bad += t_anchor_gate();
    bad += t_arm_body_needs_two_agreeing_anchors();
    bad += t_camera_gate_now();
    bad += t_camera_standing_height();
    bad += t_view_calibration_gate();
    return bad;
}

#endif
