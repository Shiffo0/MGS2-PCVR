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
static unsigned g_controls_epoch;
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
static void mobile_resolve(const LiveImage *im);
static void mobile_install(void);
static void mobile_stop(void);
static void coolant_resolve(const LiveImage *im);
static void coolant_install(void);
static void coolant_stop(void);
#include "dg_shared_motion.inl"
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
    g_controls_epoch++;
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
    mobile_resolve(&image);
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
    mobile_install();
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








}

void dg_bridge_rec_camera(const MAT *eye, const MAT *pers)
{







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
    mobile_stop();
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






































































}

/* One tick of the trigger contract. In DRY nothing is written anywhere: the
   whole gate is evaluated, the state machine is stepped, and the answer is
   counted. That is deliberately the entire delivery of this step - a session
   can be flown with the trigger live in the log and no round spent, and the
   run that finally writes has already had its edges checked against a real
   player rather than against a test harness. */
#include "dg_coolant_trace.inl"
#include "dg_mobile_tools.inl"
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

    coolant_lease.ready=0;
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
    if(weapon==14 && mode==DG_FIRE_MODE_ON && in.can_write && in.input_ok &&
       !in.physical_down && !reload_block && !m9_block && !g_mod_capture &&
       out.status && wmask && pidx>=0 && pidx<DG_PAD_PRESSURE_COUNT) {
        coolant_lease.player=pwork;coolant_lease.pad=g_b.a.player_pad;
        coolant_lease.tick=(unsigned)g_b.c_ticks;coolant_lease.stream=cmd.stream_id;
        coolant_lease.press=cmd.press_seq;coolant_lease.mask=(unsigned)wmask;
        coolant_lease.pressure=out.pressure;coolant_lease.ready=1;
        coolant_command=cmd;coolant_command_tick=(unsigned)g_b.c_ticks;
        coolant_epoch=g_controls_epoch;
    }

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
    /* The displayed failure menu is authoritative even when the preceding
       scripted sequence leaves demo/cut-in flags set. GAMEOVER alone also
       covers the death sequence, before there is a menu to control. */
    return (game&0x00004000u)!=0 && !(menu&DG_MENU_UNSAFE_MASK);
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
    if(g_b.log && g_mobile_tools.live)g_b.log("  mobile tools: turn_calls=%ld step_calls=%ld pose_changes=%ld anchor_changes=%ld (consumer counts, not movement acceptance)\r\n",g_mobile_tools.turns,g_mobile_tools.steps,g_mobile_tools.poses,g_mobile_tools.anchors);
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






















































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































