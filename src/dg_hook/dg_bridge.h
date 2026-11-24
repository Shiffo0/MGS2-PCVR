#ifndef DG_BRIDGE_H
#define DG_BRIDGE_H
/* Home menu: bit 1 reload, bit 2 M9; changes apply on the game tick. */
void dg_bridge_mod_menu_capture(int capture);
void dg_bridge_mod_menu_request(unsigned values);
void dg_bridge_mod_menu_status(unsigned *actual,unsigned *available);
#include <stdint.h>
#include "dg_magwell.h"
int dg_bridge_magwell_snapshot(DG_MAGWELL *out);
int dg_bridge_magwell_allowed(void);
#include "dg_position_target.h"
#include "dg_m9_runtime.h"
#include "dg_free_wrist.h"
#include "dg_hand_profile.h"
void dg_bridge_hand_profile_get(DG_HAND_PROFILE *out);
/* Worker only: load once, then atomically save accepted manual updates.
   Returns 1 saved, 2 loaded, 3 defaults, -1 I/O/validation failure, 0 no change. */
int dg_bridge_hand_profile_worker(void);
#include "dg_interact_adapter.h"

/* MAT is the camera matrix shared with the hook.  Keeping it here makes the
   recorder camera tap part of the bridge ABI without duplicating the type. */
#include "dg_xr.h"

/* Distinct pad press words kept for the front-end button hunt. Eight is
   room for a keystroke, a stick, and the noise around them. */
#define DG_PAD_PRESS_RING 8

#define DG_PAD_PRESS_SEQ 32

/* ------------------------------------------------------------- config --- */

enum {
    DG_FPS_MODE_OFF = 0,        /* the bridge writes nothing, ever */
    DG_FPS_MODE_TOGGLE = 1,     /* a request toggles native first person */
    DG_FPS_MODE_ALWAYS = 2      /* held during safe gameplay only */
};

enum {
    DG_FPS_MOVE_NATIVE = 0,
    DG_FPS_MOVE_OFF = 1,        /* hold it at 0 while we hold first person */
    DG_FPS_MOVE_ON = 2          /* hold it at 1, which is what places the arm */
};

enum {
    DG_SKEL_MAX = 64,           /* joints whose parent index is reported */
    DG_SKEL_CHAIN = 8,          /* depth walked up from the hand */
    DG_SKEL_WINDOW = 8
};

enum {

    DG_ADJ_JOINTS = 4,

    DG_ADJ_CASES = 4,

    DG_ADJ_SETTLE_TICKS = 2
};

typedef struct {
    int fps_mode;               /* DG_FPS_MODE_*; default OFF, fail-closed */
    int unarmed_prone_enabled;  /* internal: XR controller-arm session only */
    /* vr_camera_yaw: opt-in camera base-heading anchor. 0 is legacy/off. */
    int camera_yaw_anchor;

    int fps_move;               /* DG_FPS_MOVE_*; default NATIVE */

    int arm_show;               /* 0 = observe only, 1 = force visible */

    int work_probe;             /* 0 = don't touch it, 1 = read and report */

    int turn_probe;             /* 0 = off, 1 = measure (and write in aim),
                                   2 = dense: also one sample per aim tick */
    /* Grip roll (vr_arm_uproll). The chain's twist about the forearm axis is
       an emergent leftover nobody owns - the pose anchor serves the elbow and
       the fore residual is deliberately swing-only - and it reads as pistol
       cant. With this on, the bridge pins that twist to world up, composed
       AFTER the weight blend as a pure twist about the forearm direction the
       screen actually shows. Off is bit-for-bit the old solver. */
    int arm_uproll;             /* 0 = off, 1 = own the grip roll */

    int arm_hand_basis;         /* 0 = root (old), 1 = world */

    int arm_bend_deg;           /* degrees about the joint's X axis, 0 = off */
    int arm_bend_joint;         /* HUMAN21_* index, default 5 = right forearm */

    int skel_probe;             /* 0 = off, 1 = measure and report */
    int skel_base;              /* first joint of the reported window */

    int adjust_probe;           /* 0 = off, 1 = alternate and report */
    int adjust_probe_joint;     /* which joint to perturb, default 5 */
    int adjust_probe_deg;       /* rotation magnitude, default 30 */
    int adjust_probe_axis;      /* 0 = X, 1 = Y, 2 = Z */

    int hand_probe;             /* 0 = off, 1 = write hand_probe_rot */
    int hand_probe_rot[3];      /* the constant, in PS2 units, 4096 per turn */
    /* The trigger. DRY evaluates the whole contract every tick and writes
       nothing, which is what makes it possible to see the edges of a real
       session - repeats, refusals, lost tracking mid-aim - without spending a
       magazine to find out. */
    int fire_mode;
    int pistol_reload; /* startup-only USP/SOCOM gesture reload */
    int m9_slide; /* startup-only opt-in; native adapter must validate */
    int hf_blade; /* startup-only gesture adapter, default off */
    int blade_live;

    int blade_haptic_off;           /* vr_blade_haptic=off */
    int fp_rehit_off;               /* vr_fp_rehit=off */
    int blade_hit_speed_set;        /* vr_blade_hit_speed=<m/s> given */
    double blade_hit_speed;

    int dev_god, dev_unseen, dev_reload, dev_jump, dev_boot;
    int dev_nodeath, dev_notimer, dev_keys;
    int dev_actors, dev_freeze;
    int dev_place, dev_spawn;
    int dev_pin, dev_kdprobe;                 /* devstage5; dev_nodeath 2 = rise */
    int dev_pool, dev_spawn_n, dev_spawn_kind; /* testlevel: pool 1 on / 2 auto, 1-4, 0 any / 1 tengu / 2 soldier */
    int dev_rehit_test, dev_rehit_test2;
    int dev_punchprobe;                       /* vr_dev_punchprobe: 0 off, 1 on (extra 150), 2 far (950) */
    int dev_equip, dev_place_mm;              /* devstage8: vr_dev_equip on/off, vr_dev_place_mm 500..5000 (0 = old distances) */
    char dev_script_stage[9];                 /* script swap: vr_dev_script=<stage> ("" = off), DG_DEV_STAGE_MAX */
    char dev_script_file[260];                /* script swap: vr_dev_script_file=<path to a BGCL .gcx> (to end of line) */
    int dev_grenadeprobe;                     /* vr_dev_grenadeprobe: 0 off, 1 on (read-only throw probe, dg_grenade_probe.inl) */
    /* The kick, which is ours to invent - the game animates recoil onto the
       arm joints and we overwrite those joints every camera frame, so a shot
       has no physical consequence unless we give it one. Both are peaks for a
       single round; zero for either turns that half off, and zero for both
       turns the whole thing off. Kept as thousandths and micrometres so the
       marker can be read and written without a float anywhere near the
       parser. */
    /* F9: the left stick walks the character while we hold first person.
       Off by default and fail-closed like everything else; the deadzone is
       carried in thousandths so the parser never touches a float. The write
       contract is dg_move.h's: we only ever speak when the player's own pad
       is silent, so no tick can diminish a physical input. */
    int move_mode;              /* 0 = off, 1 = write the pad */
    int move_deadzone_mils;     /* stick magnitude * 1000 below which: silence */
    /* Right-stick turning through the game's own subjective turn. 0 = off,
       1 = smooth (the only mode there is an honest mechanism for - see
       dg_move.h on why snap is deliberately absent). gain scales the
       deflection before the byte mapping, thousandths. */
    int turn_mode;
    int turn_gain_mils;

    int move_third;             /* 0 = off, 1 = on */
    int move_prone;             /* 0 = off, 1 = on */
    int move_prone_max;         /* byte deflection cap, 34..127; default 120 */
    int move_dir_offset;        /* added to the camera yaw, 0..4095 */
    int move_dir_sign;          /* +1, or -1 to mirror the camera yaw */
    int move_dir_org;
    /* Body-follows-aim. When the yaw between the published hand demand and
       the character's animated base exceeds thresh, the body is turned
       toward the aim through the same subjective-turn bytes as the stick,
       ramping linearly to full stick rate at `full`. This is the cure for
       the wound-arm tumbles: a body left facing away from the aim keeps
       the wrist envelope pinned and the demand parked on its seams; the
       game's own third-person body turn is the mechanism that unwinds it.
       thresh 0 = off. Milli-degrees, so the parser stays integer. */
    /* The hand's roll/swing envelope, milli-degrees, 0 = built-in default.
       This is the arm's WHOLE roll authority (joints 4/5 are solved
       swing-only by construction), so when it saturates the hand stops
       following the controller - see the defines in dg_bridge.c. */
    /* vr_menu: 0 off, 1 measure (decide and count, write nothing), 2 write
       through the pad detour. Anything the marker cannot name stays 0. */
    int menu_mode;
    int script_menu_only;       /* opt-in: input only while player arm is absent */

    int tick_seam;

    int arm_freeze;
    int hand_fore_twist_mdeg;
    int hand_wrist_twist_mdeg;
    int hand_wrist_swing_mdeg;
    int turn_follow_thresh_mdeg;
    int turn_follow_full_mdeg;

    int turn_dir_override;
    /* +1/-1 flips the facing gap's head-yaw term if the camera yaw
       convention proves mirrored against the character root's; 0 (an
       unset marker) means +1. */
    int follow_head_sign;

    int follow_src;

    int follow_aim;

    int arm_comp;

    int adjust_frame;
    int recoil_climb_mdeg;      /* muzzle climb at the wrist, milli-degrees */
    int recoil_push_um;         /* hand driven back toward the shoulder, um */
    /* Where the tracked hand is anchored on the character. 0 keeps the
       original offset anchor - the arm starts at the animation's wrist and
       moves from there - and 1 is the absolute shoulder anchor, which puts the
       character's hand where the player's hand is relative to the player's own
       shoulder. See the header of dg_arm_map.h; the difference is the whole
       reason the arm hung at the character's hip. */
    int arm_anchor;

    int theater_mode;           /* DG_THEATER_*; default OFF */

    int theater_ui;             /* 0 = off, 1 = menu panel judgment feeds it */

    int hud_mode;               /* 0 = leave the HUD alone, 1 = hide it */

    int grip_mode, grip_grab_ms, grip_release_ms;

    int twohand_rifles, twohand_steer;
    /* vr_pistol_reload_v2 (startup-only, needs vr_pistol_reload=on, default off):
       eject with the right grip + direct fill. Appended last: no offset moves. */
    int pistol_reload_v2;

    int psg_rifle;
    int optics;

    int psg_gesture_set, psg_open_cm, psg_close_cm, psg_open_deg, psg_close_deg;

    int blade_armplace;

    int twohand_reach, twohand_anchor, twohand_latch_enter_mm, twohand_latch_exit_mm;

    int blade_follow;

    int dev_psgprobe;

    int psg_move;

    int left_reach_centre;
} DG_BRIDGE_CONFIG;

typedef struct {
    int valid;
    int armed;
    int fps_active;
    int native_active;
    int safe_gameplay;
    int arm_camera_on;
    unsigned int game_status;
    unsigned int menu_status;
    unsigned __int64 arm_body;
    unsigned __int64 camera;
} DG_CAMERA_GATE;

enum {
    DG_THEATER_OFF = 0,
    DG_THEATER_MEASURE = 1,
    DG_THEATER_ON = 2
};

/* Verdict bits reported by dg_bridge_theater_verdict(). */
enum {
    DG_THEATER_V_DEMO = 1,      /* demo/codec theater judgment is standing */
    DG_THEATER_V_UI = 2,        /* full-menu panel judgment is standing */
    DG_THEATER_V_FAKE = 8       /* Fission Mailed: flat screen + native camera, controls stay on */
};

enum {
    DG_FIRE_MODE_OFF = 0,
    DG_FIRE_MODE_DRY,       /* evaluate and count; touch nothing */
    DG_FIRE_MODE_ON         /* also write the pad: rounds go downrange */
};

typedef struct {
    int      write;

    unsigned long pair_id;

    unsigned long stream_id;
    unsigned long left_stream_id; /* physical calibration and camera/arm identity */
    int persistent_hands;
    unsigned long hand_calibration_request;
    double   wrist_view[3];

    double   player_shoulder_view[3];
    /* The player's own arm length in the same mapped units, so the scale is a
       ratio of two arm lengths rather than of one arm to one arbitrary
       instant of elbow bend. Only the shoulder anchor reads it. */
    double   player_reach_view;
    /* The mapped head yaw (radians, game view convention), for the
       facing-gap publisher. It moves with the player's physical turn and
       with the stick's software turn, and with NOTHING the body or the
       controller does - the property both previous gap publishers lacked
       (one starved, one was blind to the player turning on their feet). */
    double   head_yaw_rad;
    int      head_yaw_valid;

    double   hand_yaw_rad;
    int      hand_yaw_valid;
    /* The stick's software turn alone, in the same mapped frame: the mapped
       head yaw minus the head's LOCAL (untouched) yaw through the same
       mapping. A physical head turn moves both terms together and cancels
       exactly; what remains is the turned reference - the stick's offset
       plus the recentre's constant, which the calibration zero absorbs. */
    double   stick_yaw_rad;
    int      stick_yaw_valid;
    double   hand_quat[4];      /* latched with the wrist, in the same frame */

    int      hand_write;
    double   weight;            /* identity -> solved adjust, 0..1 */
    unsigned pose_flags;        /* DG_POSE_F_* copied for diagnostics only */
    int absolute_aim;           /* opt-in right-hand pistol root basis */
    DG_POSITION_INPUT position;
    DG_POSITION_INPUT left_position; /* Independent camera/raw left-grip sample. */
    DG_M9_SAMPLE m9_pose; /* Raw pair used by this exact arm solve. */
    struct {
        double eyes[2][3], head_q[4], grip_m[3], grip_q[4];
        int valid, left, right;
    } optics;
    DG_FREE_WRIST_INPUT free_right,free_left;
    int aim_write;
    double aim_world[4];
    double aim_weight;
    unsigned long aim_pair_id;
    unsigned long aim_stream_id;
    unsigned long long aim_sample_seq;
    long long aim_sample_time;
    unsigned long long aim_arm, aim_subobject, aim_subobjs, aim_hand, aim_model;
    unsigned long long aim_weapon_id;
    /* Independent left grip, sampled from the same XR frame as the right. */
    int left_enabled, left_valid, twohand_enabled, hands_coherent;
    int unarmed_hand_write; /* raw tracking preference before pistol gating */
    float observed_right_trigger; /* same XR frame as hand_quat, diagnostics only */
    float observed_right_stick_x;
    unsigned long calibration_id; /* physical reference, independent of rig */
    double calibration_camera[4]; /* native base-camera orientation */
    double left_wrist_view[3], left_shoulder_view[3], left_weight;
    double hands_distance_m;
    unsigned long long left_sample_seq;
    long long left_sample_time;
    /* Blade block: how far the visible left hand has moved onto the blade
       (0..1), from the hand's own grip blend in the producer. The guard
       itself is decided elsewhere on the raw sample and never reads this. */
    double brace_weight;
} DG_BRIDGE_ARM_TARGET;

/* -------------------------------------------------------- state machine --- */

enum {
    DG_FPS_OFF = 0,
    DG_FPS_REQUEST_ENTER,
    DG_FPS_ACTIVE,
    DG_FPS_SUSPENDED,
    DG_FPS_REQUEST_LEAVE
};

enum {
    DG_FPS_REASON_NONE = 0,
    DG_FPS_REASON_MODE_OFF,
    DG_FPS_REASON_MASK_ZERO,
    DG_FPS_REASON_LEVEL_LOAD,       /* subject-move pad pattern after a load */
    DG_FPS_REASON_MGSHDFIX_OWNER,   /* someone else already holds Override */
    DG_FPS_REASON_UNSAFE,           /* not controllable gameplay */
    DG_FPS_REASON_TIMEOUT,          /* the native state never confirmed */
    DG_FPS_REASON_NATIVE_EXIT       /* preserve explicit VR preference */
};

enum {
    DG_FPS_WRITE_NONE = 0,
    DG_FPS_WRITE_OVERRIDE,
    DG_FPS_WRITE_TOGGLE,
    DG_FPS_WRITE_SUBJECT_EDGE,
    DG_FPS_WRITE_MOVE
};

typedef struct {
    int mode;
    int native_override;
    int native_toggle;
    int native_active;
    int native_camera_missing;      /* camera off or replaced rig needs native entry */
    int native_move;
    int subject_move;
    unsigned int pad_subject_mask;
    unsigned int pad_stop_aim_mask;
    int safe_gameplay;

    unsigned int game_status;
    unsigned int menu_status;
    int mgshdfix_owner;
    int toggle_request;             /* one user request, DG_FPS_MODE_TOGGLE */
    int level_load;                 /* a load happened with subject move on */
    int move_mode;                  /* DG_FPS_MOVE_* */
    int saved_override;             /* the values found before we ever wrote, */
    int saved_toggle;               /* so idle can hand them straight back */
    int saved_move;

    unsigned long long player_status;
    int rehit;
} DG_FPS_INPUT;

typedef struct {
    int state;
    int reason;
    int desired;                    /* desired first person, kept across suspends */
    int owned;                      /* this first person session is ours to undo */
    int held;                       /* we raised Override/Toggle and owe them back */
    unsigned int ticks;             /* ticks in the current state */
    unsigned int edges;             /* subject edges emitted, bounded counter */
    unsigned int camera_missing_ticks;

    unsigned int clock;             /* steps since init, never reset */
    unsigned int off_clock;         /* clock when a toggle last turned desired off; 0 none */
    int rehit_armed;                /* suspended by a knockdown with desired kept */
    unsigned int grace_until;       /* toggles ignored while clock < this */
    unsigned int rehit_restored, rehit_ignored, rehit_returns;

    int damage_off;
    unsigned int rehit_restore_age;
    int rehit_restore_episode;

    unsigned int damage_ticks, damage_expired;
} DG_FPS_STATE;

typedef struct {
    int write;                      /* DG_FPS_WRITE_* */
    int write_value;
    int transitioned;               /* state or reason changed this tick */
    int from_state;
    int from_reason;
    /* Set only when a toggle_request actually flipped `desired`. The caller
       clears its pending flag on this and nothing else: every suspend gate
       returns before the request is read, and a request swallowed during a
       menu is a button press the player never gets back. */
    int request_consumed;
} DG_FPS_STEP;

/* Pure. No globals, no game access, no clock. */
void dg_fps_init(DG_FPS_STATE *state);
void dg_fps_step(DG_FPS_STATE *state, const DG_FPS_INPUT *in, DG_FPS_STEP *out);

const char *dg_fps_state_name(int state);
const char *dg_fps_reason_name(int reason);

/* ----------------------------------------------------------- telemetry --- */

typedef struct {
    long fps_requested;
    long fps_entered;
    long fps_left;
    long fps_transitions;
    long fps_edges_injected;
    long fps_writes_applied;
    long fps_refused_mask_zero;
    long fps_suspend_level_load;
    long fps_suspend_unsafe;
    long fps_timeouts;
    long fps_events_dropped;
    long ticks;
    int  fps_state;
    int  fps_suspend_reason;
    int  fps_native_active;
    int  mgshdfix_fps_owner;
    int  override_value;
    int  toggle_value;
    int  move_value;
    int  subject_move_value;
    long subject_move_ticks;        /* ...and ticks it was non-zero, all session */
    int  subject_toggle_value;
    unsigned int pad_subject_mask;
    unsigned int pad_stop_aim_mask;
    unsigned __int64 player_status;
    unsigned int game_status;
    unsigned int menu_status;

    unsigned __int64 arm_body;
    unsigned __int64 arm_objs;
    unsigned int arm_flag;

    unsigned __int64 arm_cam_rotate_shift;
    short arm_cam_rot[3];
    /* The calibration probe. wrote/read are the same tick: written at the tick
       seam ahead of the arm actor, read back at the camera seam after SetPos
       has consumed it, so their ratio IS the smoothing. adjust6 is what SetPos
       left in the hand's slot and predicted is what we computed the read-back
       should produce; worst_diff between them is the whole conversion claim in
       one number. Both counters are here so "measured nothing" can be told
       apart from "measured zero". */
    long hand_probe_writes;
    long hand_probe_reads;
    long hand_probe_no_setpos;
    short hand_probe_wrote[3];
    short hand_probe_read[3];
    float hand_probe_adjust6[4];
    float hand_probe_predicted[4];
    float hand_probe_worst_diff;

    unsigned __int64 body_cand;
    unsigned __int64 body_objs;
    unsigned int body_flag;
    long arm_created;
    long arm_destroyed;
    long arm_forced;                /* ticks the visibility probe wrote on */
    long arm_forced_late;           /* ...and from the camera hook, later */

    unsigned __int64 seen_player;
    unsigned int seen_game;
    unsigned int seen_menu;

    unsigned int seen_game_late;
    unsigned int seen_menu_late;
    unsigned __int64 seen_player_late;
    long late_status_samples;

    long late_status_stale;

    long orient_anchored;
    long orient_fallback;
    /* The grip-roll owner (vr_arm_uproll): pairs whose forearm quaternion
       was actually up-rolled (|applied twist| > 1e-6 rad), and pairs the
       owner skipped FAIL-CLOSED because the adjust slots could not be read
       back trustworthily. A skipped count that climbs during play means the
       read-back path is unhealthy and the correction is silently absent -
       the counter is what keeps that from looking like "the fix works". */
    long orient_uprolled;
    long uproll_skipped;
    int  arm_uproll_on;
    int  arm_hand_basis_world;  /* vr_arm_hand_basis, for the heartbeat */
    /* Accumulated rotation, degrees: what the hierarchy's hand actually
       turned (measured passively, so it keeps working with the arm drive
       off) against what our own two written joints turned. The tumble is
       smooth, so only totals reveal it - per-pair deltas read as calm. */
    /* Joints 4 and 5's own slot echo: how far the hierarchy's adjust has
       moved from what we cached there. The strip that recovers the
       "animation" skeleton - and with it the roll anchor's reference -
       assumes this is zero, and until now nothing checked. */

    int  menu_anchor_ok;
    int  menu_detour_live;
    long pad_seam_entries;
    long start_queued;
    long start_consumed;
    long pad_context_refused;
    long menu_writes;

    unsigned int menu_seen_status;
    unsigned int menu_seen_press;

    unsigned int menu_press_word[DG_PAD_PRESS_RING];
    long menu_press_count[DG_PAD_PRESS_RING];
    unsigned int menu_press_seq[DG_PAD_PRESS_SEQ];
    long menu_press_seq_ms[DG_PAD_PRESS_SEQ];
    long menu_press_seq_n;      /* total presses, may exceed the kept window */
    long menu_refused_stale;
    long menu_refused_gate;
    float arm_adj_echo_deg;
    float arm_adj_echo_worst_deg;
    long  arm_adj_echo_dirty;
    float skel_hand_turned_deg;
    float adj_turned4_deg;
    float adj_turned5_deg;
    long  skel_hand_samples;
    /* NET rotation of the hand against a reference captured once, beside
       the accumulator above. The accumulator sums absolute per-frame steps,
       so it measures TOTAL VARIATION: an arm that shakes and an arm that
       winds report the same thing, and the acceptance bar for the tumble
       has to be bounded NET rotation over time. This is that number. World
       frame, so a player who turns their body is counted too - stand still
       while reading it. */
    float skel_hand_net_deg;
    float skel_hand_net_max_deg;
    /* REST freeze instrumentation: the angle between the frozen rest
       reference and the one the strip would have used this pair. This is
       the feedback measured DIRECTLY - if the recovered rest walks away
       from the frozen one while the arm stays still, the strip residue is
       caught in the act, with a number. */
    float rest_ref_drift_deg;
    float rest_ref_drift_max_deg;
    long  rest_ref_captured;    /* 0 until a calibration pair supplied it */
    unsigned int seen_game_screen;
    unsigned int seen_menu_screen;
    long screen_status_samples;
    /* Camera seams where the state machine still read ACTIVE but the seam's
       own reading of the three status words said stand down. Non-zero here
       with unsafe suspends at zero is the codec/cutscene case the tick seam
       is too slow to catch. */
    long seam_refused_unsafe;
    /* ...and how many of those got past the first-person gate to reach the
       skeleton and adjust probes. Zero here with a configured probe means the
       session never entered first person, which is a different finding from a
       probe that ran and found nothing - and indistinguishable without it. */
    long seam_active_samples;
    long seam_active_ticks;        /* distinct ticks represented by them */
    unsigned int seen_arm_flag;
    unsigned int seen_body_flag;

    unsigned int held_arm_flag;
    unsigned int held_body_flag;
    long arm_visible_ticks;
    long body_visible_ticks;

    unsigned __int64 arm_work;
    unsigned __int64 player_work;
    int  arm_camera_on;
    long arm_camera_on_ticks;
    unsigned int arm_trigger;
    unsigned int seen_arm_trigger;

    unsigned int arm_flag_late;
    unsigned int seen_arm_flag_late;
    unsigned int held_arm_flag_late;
    long arm_visible_late_ticks;
    /* F5 step 1. arm_joints must read 21 for HUMAN21, and is the cheapest
       possible check that the whole struct reading is right - it is sampled
       even when the bend is off, and no write happens unless it holds. */
    int  arm_joints;
    unsigned __int64 arm_adjust;
    long arm_bend_writes;
    long arm_track_writes;      /* of those, driven by a controller */
    long arm_quat_refused;      /* non-unit quaternions rejected */
    long arm_ik_refused;        /* target/skeleton/solve failed closed */
    long arm_ik_clamped;        /* solves that reported any reach/limit clamp */
    long arm_ik_implausible;    /* target was outside the arm-root envelope */
    long arm_pairs_seen;        /* unique perceived stereo/mono pose pairs */
    long arm_pairs_eligible;    /* after settle/calibration, before solve */
    long arm_pairs_accepted;    /* unique pairs that produced a cached solve */
    long left_pairs_accepted, left_pairs_refused, left_support_pairs;
    long left_swing_frames, left_swing_refused, left_swing_applied;   /* long-weapon steering */
    long left_swing_lever_mm, left_swing_deci_deg;                     /* last swing: lever, angle x10 */
    int left_status, left_refuse_reason;
    long left_support_gate[7];
    long left_support_overreach, left_anchor_kept;
    long left_centre_used, left_centre_resets, left_centre_clamped;   /* option A: filtered reach centre */
    float left_blend, left_target[3];
    long arm_pairs_refused;
    long arm_pairs_calibration; /* settle or capture pairs intentionally blank */
    long arm_pairs_map_clamped;
    long arm_pairs_map_soft;    /* pairs in the stateless outer soft zone */
    long arm_pair_replays;      /* extra seams/second eye using pair cache */
    long arm_pairs_frozen;      /* pairs served from the frozen pair, unsolved */
    long arm_hand_written;      /* pairs that published a hand command */
    long arm_hand_refused;      /* hand solve failed; joints 4/5 still written */
    long arm_hand_measured;     /* residual samples behind the two figures below */
    long arm_hand_tick_writes;
    long arm_hand_tick_no_command;
    long arm_hand_tick_stale;
    long arm_hand_tick_no_player;
    long arm_hand_tick_owner_mismatch;
    long arm_hand_tick_bad_weapon;
    long arm_hand_tick_mic;
    long arm_hand_no_setpos;

    long arm_hand_zeroed;
    /* F9 walking. writes counts ticks the pad was actually written; yielded
       counts ticks the player's own movement input silenced us, which is the
       whole write contract in one number. */
    long move_writes;
    long move_yielded;
    long move_idle;
    long move_stale;
    long move_no_command;
    long move_blocked_gate;
    long move_published;
    int  move_mode;
    float move_deadzone;
    long move_not_subject;

    int  move_third, move_prone;
    long move_third_writes;     /* walk bytes written while not in first person */
    long move_not_third;        /* wanted third person, refused (WATCH/subject/pad) */
    long move_prone_writes;     /* walk bytes written capped, prone */
    long move_dir_writes;       /* pad->dir stores */
    long move_padto_writes;     /* workL->padTo/padForce redone from our bytes */
    int  tick_seam_copy;
    long move_no_workl;         /* dir written but workL unavailable */
    long move_cam_dir;          /* camera yaw the seam last published, -1 = none */
    long move_last_org, move_last_dir;
    /* Move probe (diagnostic): the last third-person/prone write as made at
       the tick seam, and the pad + player words as the camera seam saw them
       AFTER the player routine ran in that same tick. */
    long mp_w_tick, mp_w_dir, mp_w_bytes;
    unsigned long mp_w_status;
    long mp_c_tick, mp_c_dir, mp_c_bytes, mp_c_analog, mp_c_rot, mp_c_turn;
    unsigned long mp_c_status;
    unsigned long long mp_c_act, mp_c_act2;
    long mp_c_seen;

    unsigned long long mp_c_work_pad, mp_c_our_pad;
    unsigned long mp_c_gv_flag;
    unsigned long mp_c_wp_status;       /* status/dir/bytes read THROUGH work->pad */
    long mp_c_wp_dir, mp_c_wp_bytes;

    unsigned long long mp_c_workl;
    long mp_c_padto, mp_c_wallto, mp_c_liable, mp_c_padforce, mp_c_data, mp_c_data2;
    int  move_dir_org;
    long turn_writes;
    long turn_yielded;
    long turn_idle;
    int  turn_mode;
    float turn_gain;

    /* The envelope in force, degrees - "limited" above is only readable
       against these. */
    float arm_cap_fore_deg, arm_cap_wrist_deg, arm_cap_swing_deg;
    long turn_follow_writes;
    float turn_aim_gap;
    int  turn_dir;
    long turn_dir_votes;
    int  follow_src;            /* 0 = head, 1 = hand, 2 = stick (vr_turn_follow_src) */
    int  follow_aim;            /* 0 = hold under fire, 1 = on (vr_turn_follow_aim) */
    int  arm_comp;              /* 0 = full, 1 = organic (vr_arm_comp) */
    long comp_no_stick;         /* organic asked, no stick zero: full used */
    int  adjust_frame_live;
    long frame_missing;
    float frame_skew_worst;     /* degrees the heading moved inside one pair */
    float wrist_miss_worst;     /* mm: predicted wrist vs the next read */
    long  wrist_miss_over;      /* pairs with a wrist miss over 20 mm */

    long follow_gate_no_sign, follow_gate_stale, follow_gate_under;
    /* Move ticks on which the follow stood down because the trigger held
       the weapon up - the pad actuator cannot turn a body vanilla MGS2
       roots during first-person aim, it only disturbs the aim. */
    long follow_gate_aim_hold;

    long turn_write_dead;

    unsigned char pad_right_dx, pad_right_dy, pad_left_dx, pad_left_dy;

    long arm_hand_fore_twist;   /* pairs that transferred twist to joint 5 */
    long arm_hand_limited;      /* pairs whose wrist safety envelope saturated */

    float arm_hand_residual_deg;
    float arm_hand_worst_deg;

    float arm_hand_slot_echo_deg;
    float arm_hand_slot_echo_worst_deg;
    long arm_hand_slot_dirty;   /* pairs whose slot echo exceeded 5 deg */
    /* How much the residual's offset quaternion moved since the previous
       measured pair. Large residual + near-zero drift = a constant frame
       error (a per-weapon hand frame); drift comparable to the residual =
       a composition/order error. Only consecutive measured pairs count. */
    float arm_hand_off_drift_deg;
    float arm_hand_off_drift_worst_deg;

    long arm_adjust_bits_lost;

    long arm_hand_reach_units;
    long arm_hand_alt_named;
    long arm_hand_scaled;
    float arm_hand_fit_frac;
    float arm_hand_shortfall_worst;
    /* Automatic B-presses: hand rest pairs recaptured because a suspension
       lasted long enough (a cutscene, a codec) to hide a scene change from
       the stream. One per such resume is healthy; a stream of them without
       demos would mean the safety gate is flapping. */
    long arm_rest_recaptured;
    /* Pair-to-pair motion of the stripped animation base itself. Ordinary
       animation moves this a few degrees per pair; a base that leaps tens
       of degrees while the model stands still means the READ is broken -
       the post-cutscene suspect - whereas a calm base under a locked
       residual convicts the application instead. */
    float arm_base_drift_deg;
    float arm_base_drift_worst_deg;
    /* Pair-to-pair motion of our own published command. A calm command
       stream under a wild base convicts the game's animation; a flapping
       command stream is ours - and it poisons the base reading through the
       one-pair strip phase, so judge cmd first, base second. */
    float arm_cmd_drift_deg;
    float arm_cmd_drift_worst_deg;
    float arm_hand_raw_twist_deg;
    float arm_hand_fore_twist_deg;
    float arm_hand_wrist_swing_deg;
    float arm_hand_wrist_twist_deg;
    long arm_release_owner_mismatch; /* no write: model identity had changed */
    unsigned int arm_map_flags;
    unsigned long arm_map_pair;
    unsigned long arm_map_stream;
    float arm_map_scale;
    float arm_map_source_span;
    float arm_map_reach;
    float arm_map_delta[3];
    float arm_map_target[3];
    /* The character's own shoulder and animated wrist in the arm-root basis.
       Published for one reason: this basis is not the camera's view basis and
       nothing else in the log says which way its axes point. These two vectors
       do - a right arm hanging at rest puts its wrist below and outboard of
       its shoulder - so which sign the arm map needs becomes a reading rather
       than an argument. */
    float arm_shoulder_view[3];
    float arm_native_wrist_view[3];
    int arm_anchor;             /* 0 = offset anchor, 1 = shoulder anchor */

    float arm_body_drift_deg;
    long arm_body_uncompensated;
    float arm_ik_weight;
    float arm_ik_view[3];
    float arm_ik_root[3];
    float arm_ik_target[3];
    float arm_ik_root_distance;
    float arm_ik_root_limit;
    unsigned int arm_pose_flags;
    int  arm_bend_joint;
    int  arm_bend_deg;

    unsigned __int64 arm_mctrl;
    unsigned __int64 arm_mctrl_head[10];
    /* ...and the OBJECT it came from, dumped before any gate, so a refusal
       one level up can still say why. +0x00 is objs and +0x30 is evmobj,
       both known, which makes the dump check itself. */
    unsigned __int64 arm_obj_head[8];

    int  skel_stride;
    int  skel_stride_score;
    int  skel_stride_tried;         /* candidates rejected before the hit */
    int  skel_n_models;
    int  skel_parents_read;         /* how many of skel_parents are valid */
    short skel_parents[DG_SKEL_MAX];
    /* Walked up from joint 6, the one proven index: hand, then its parent, and
       so on to the root. This is the answer to "which indices are the arm". */
    short skel_chain[DG_SKEL_CHAIN];
    int  skel_chain_len;
    /* World translation of the chain joints, in game units (1 unit = 1 mm, as
       F3 established from the 61.6-unit eye separation). Bone lengths are the
       distances between consecutive entries. */
    float skel_chain_pos[DG_SKEL_CHAIN][3];
    /* ...and of a configurable window, for reading the rest of the rig. */
    int  skel_base;
    float skel_window_pos[DG_SKEL_WINDOW][3];

    unsigned __int64 skel_mctrl_trans;
    unsigned __int64 skel_objs;
    unsigned int skel_region_end;   /* bytes of the committed region past objs */

    int   adj_probe_joint;                  /* the joint that was perturbed */
    int   adj_probe_indices[DG_ADJ_JOINTS]; /* root, parent, joint, child */
    float adj_probe_quat[DG_ADJ_CASES][4];  /* what each case wrote, {x,y,z,w} */
    float adj_probe_world[DG_ADJ_CASES][DG_ADJ_JOINTS][16];
    long  adj_probe_samples[DG_ADJ_CASES];
    long  adj_probe_held;           /* seams that waited for a fresh tick */

    long  adj_probe_words[DG_ADJ_CASES][2][5];
    unsigned int seen_pad_status;   /* every pad bit seen down this session */
    unsigned int pad_weapon_mask;
    long weapon_presses;            /* rising edges of it */
    long weapon_presses_in_fps;     /* ...of which, while first person was up */
    int  weapon_vk[4];              /* keys down at the last weapon edge */

    int  fire_mode;                 /* DG_FIRE_MODE_* as configured */
    long fire_published;            /* trigger samples the camera seam offered */
    long fire_no_command;           /* ticks with no fresh sample at all */
    long fire_stale;                /* ...with one too old to act on */
    long fire_drawn;                /* pulls the contract accepted */
    long fire_released;             /* releases: where a pistol's shot lands */
    long fire_aborted;              /* cancel windows entered */
    long fire_forced;               /* aims dropped because the gates went */
    long fire_blocked_phys;         /* pulls refused: the player's own button */
    long fire_blocked_gate;         /* ticks the game side refused */
    long fire_repeats;              /* the same pull offered again */
    long fire_coasting;             /* aim ticks held across a sample gap */
    long fire_auto_ticks;           /* ticks over the full-auto pressure line */
    long fire_wrote_press;          /* pad writes, per field, ON mode only */
    long fire_wrote_status;
    long fire_wrote_release;
    long fire_wrote_pressure;
    long fire_pressure_kept;        /* the game's own byte was already higher */
    long fire_no_index;
    long fire_yielded;              /* ticks stood down under the player's own
                                       weapon button */
    int  fire_press_index;

    unsigned int weapon_state_seen;
    unsigned int button_state_seen;
    int  weapon_state;              /* the last of each, for the heartbeat */
    int  button_state;
    int  fire_state;                /* DG_FIRE_* right now */
    int  fire_pressure;             /* what the last tick would have written */
    unsigned int fire_wtype;
    int  recoil_climb_mdeg;         /* as configured */
    int  recoil_push_um;
    long recoil_kicks;              /* shots that reached the spring */
    long recoil_climb_writes;       /* camera frames that carried a climb */
    long recoil_no_axis;            /* ...and those a vertical forearm refused */
    long recoil_push_writes;        /* camera frames that shortened the reach */
    long recoil_push_refused;       /* ...and those too close to the shoulder */
    float recoil_amplitude;         /* right now, in single-shot peaks */
    float recoil_worst;             /* the highest this session */
    int  restored;
    /* Theater phase 0 telemetry, all counted at the camera seam. The samples
       count is the denominator that makes every zero meaningful: a mask bit
       never seen with samples at zero says the seam never ran, not that the
       state never happened - the same lesson as late_status_samples. */
    long thea_samples;              /* judgments computed (one per seam status
                                       sample, i.e. one per frame) */
    long thea_masked;               /* ...where the theater mask had a bit */
    long thea_ui_masked;            /* ...where the menu-panel mask had one */
    long thea_enter, thea_exit;     /* theater judgment flanks */
    long thea_ui_enter, thea_ui_exit;
    long thea_enter_held;           /* masked samples the enter hysteresis
                                       refused to act on yet */
    long thea_exit_held;            /* clean samples the exit hysteresis sat out */

    long thea_bit_demo;
    long thea_bit_scn;
    long thea_bit_pad;
    long thea_bit_radio;
    long thea_bit_weapon;
    long thea_bit_item;
    int  thea_verdict;              /* raw published bits, unmasked by mode */
    int  thea_mode;                 /* DG_THEATER_* as configured */
    int  thea_ui_on;
    int  hud_hide;                  /* as configured */
    long hud_writes;                /* ticks the hide bits were (re)asserted */
    long hud_cleared;               /* release writes (at most 1 per toggle) */
} DG_BRIDGE_STATS;

/* ----------------------------------------------------------- lifecycle --- */

/* Resolves the anchors in this process and installs the drive point. Returns 0
   and stays completely inert on any miss; the reason is logged once. Never
   called with mode off - an off bridge is simply not started. */
int  dg_bridge_start(void (*log)(const char *fmt, ...),
                     const DG_BRIDGE_CONFIG *cfg);

void dg_bridge_configure(const DG_BRIDGE_CONFIG *cfg);

/* Ask for one toggle in DG_FPS_MODE_TOGGLE. Safe from any thread. */
void dg_bridge_request_toggle(void);
/* HF-blade haptic event bits (dg_blade_haptic.h), taken and cleared. */
unsigned dg_bridge_blade_events(void);
void dg_bridge_cancel_toggle(void);
/* Game-thread-only raw context, independent of theater display settings and
   native FPS state (A must also be able to enter FPS). */
int dg_bridge_controller_gameplay_now(void);
int dg_bridge_controller_radial_now(void);

/* Queue one native START press.  The pad seam consumes it exactly once and
   publishes it to both the direct and normal pad records, so it reaches the
   pause path as well as front-end screens. */
void dg_bridge_start_now(void);
int dg_bridge_menu_context_ready(void); /* read-only anchor value, no object dereference */
int dg_bridge_menu_gameover_now(void);

typedef struct DG_BRIDGE_MENU {
    unsigned int status;

    unsigned int clear;
    int allow; /* 0 withdraw, 1 front-end, 2 codec back (revalidated at consume) */
} DG_BRIDGE_MENU;

/* Publish one frame's menu input. Called from the Present seam; consumed by
   the pad detour on the next pad update. NULL withdraws, like move/fire. */
void dg_bridge_menu_now(const DG_BRIDGE_MENU *cmd);

struct DG_REC_PAIRSTATE;
void dg_bridge_rec_pair(struct DG_REC_PAIRSTATE *out);
void dg_bridge_rec_camera(const MAT *eye, const MAT *pers);
/* The game's OWN camera yaw (before the head pose), 0..4095 in the
   engine's direction units, or -1 when the seam had no rigid camera. Feeds
   pad->dir for the third-person and prone walks. */
void dg_bridge_pad_origin_now(LONG dir4096);
/* Final VR camera heading for blade locomotion and native guard geometry. */
void dg_bridge_blade_camera(const float world[4][4]);
/* Camera seam: sample the pad record and the player's act/yaw words after
   the player routine, for the move probe (diagnostic only). */
void dg_bridge_move_probe_now(ULONGLONG image_base);

void dg_bridge_arm_seam_now(const DG_BRIDGE_ARM_TARGET *target);

typedef struct {
    unsigned __int64 press_seq;
    unsigned __int64 release_seq;
    double value;                   /* raw trigger travel, 0..1 */
    double click;                   /* the configured press threshold */
    int    valid;                   /* tracked, active, fresh enough to act on */
    unsigned long stream_id;        /* a change forgets any pull in progress */
} DG_BRIDGE_FIRE;

void dg_bridge_fire_now(const DG_BRIDGE_FIRE *cmd);

/* One left thumbstick, published from the same camera seam and consumed on
   the game tick. A level is exactly what a stick is, so unlike the trigger no
   sequence numbers are needed - writing the same level five times per tick is
   idempotent. A NULL command stands walking down. */
typedef struct {
    double x, y;                    /* -1..1, +x right, +y up (OpenXR) */
    int    valid;                   /* the XR side vouches for the controller */
    /* The RIGHT stick's X, carried on the same level crossing because the two
       sticks are one gesture system and one staleness. Vouched separately:
       the right controller can sleep while the left walks. */
    double turn_x;
    int    turn_valid;
    unsigned long stream_id;        /* diagnostics only */
} DG_BRIDGE_MOVE;

void dg_bridge_move_now(const DG_BRIDGE_MOVE *cmd);

#include "dg_m9_runtime.h"
#include "dg_blade.h"
#include "dg_stinger_mode.h"
#include "dg_nikita_mode.h"
int dg_bridge_blade_enabled(void);
int dg_bridge_stinger_enabled(void);
typedef struct {
    DG_M9_SAMPLE m9;
    DG_BLADE_SAMPLE blade;
    DG_STINGER_INPUT stinger;
    DG_NIKITA_INPUT nikita;
    DG_NIKITA_STEER_INPUT nikita_steer;
    DG_BRIDGE_FIRE fire;
    DG_BRIDGE_MOVE move;
    DG_INTERACT_SAMPLE interact;
    int fire_available, move_available;

    struct { int valid, ear; double squeeze; } reload_eject;
    /* psg_v8: the right A (first-person toggle) down in the same XR snapshot as `fire`.
     * Read only by the vr_psg_move hip-shot note in fire_tick. Appended last. */
    int fire_fp_button;
    /* m9_grab_probe (diag only): the left squeeze LEVEL and the pose ages of the XR frame the
     * m9 sample came from. The sample itself carries only squeeze_click. Appended last. */
    struct { int valid; double left_squeeze; unsigned left_age_ms, right_age_ms, aim_age_ms; } m9_diag;
} DG_BRIDGE_CONTROLS_FRAME;
/* Separate native gameplay, tracking-only, ladder and radial admissions. */
enum { DG_CONTROLS_NONE=0, DG_CONTROLS_GAMEPLAY=1, DG_CONTROLS_TURN_ONLY=2,
       DG_CONTROLS_LADDER=DG_IA_LADDER_CONTEXT, DG_CONTROLS_RADIAL_ONLY=4,
       DG_CONTROLS_BEYOND=DG_IA_BEYOND_CONTEXT, DG_CONTROLS_LOCKER=DG_IA_LOCKER_CONTEXT,
       DG_CONTROLS_DOWNED=DG_IA_DOWNED_CONTEXT };
#include "dg_action_owner.h"
int dg_bridge_zoom_now(uint64_t *identity,float *angle);
/* Controls provider only: caller holds the controls snapshot lock. */
int dg_bridge_psg_scope_active(void);
int dg_bridge_psg_render_zoom(uint64_t *identity,float *angle);
int dg_bridge_psg_zoom_now(uint64_t *identity,float *angle);
int dg_bridge_camera_stick_owned(void);
int dg_bridge_optics_grip_owned(int item);
int dg_bridge_optics_zoom_now(uint64_t *identity,float *angle);
typedef void (*DG_ACTION_PROVIDER)(DG_ACTION_SAMPLE *sample);
void dg_bridge_action_register(DG_ACTION_PROVIDER provider);
void dg_bridge_action_context(int allowed);
int dg_bridge_hatch_now(void);
int dg_bridge_action_ladder_now(void); /* 0 none, 2 turning, 3 opening */
typedef void (*DG_BRIDGE_CONTROLS_PROVIDER)(void *user, int allowed,
    int fire_idle, uint64_t now_ms, uint64_t tick, DG_BRIDGE_CONTROLS_FRAME *out);
typedef void (*DG_BRIDGE_CONTROLS_STOP)(void *user);
void dg_bridge_controls_register(DG_BRIDGE_CONTROLS_PROVIDER provider,
    DG_BRIDGE_CONTROLS_STOP stop, void *user);
void dg_bridge_controls_context(int allowed);
void dg_bridge_controls_context_modes(int allowed,int ladder);
void dg_bridge_controls_context_special(int allowed,int special);
int dg_bridge_controller_ladder_now(void);
void dg_bridge_controls_context_ex(int gameplay, int radial);
void dg_bridge_controls_context_all(int gameplay, int special, int radial);
int dg_bridge_controller_special_now(void);
int dg_bridge_hanging_heading_now(double *heading);
int dg_bridge_codec_input_now(void);

/* Drains the bounded transition ring into the log. Logger side only: the game
   thread never formats a string. */
void dg_bridge_drain_log(void);

/* Restores the preserved native values exactly once, removes the detour and
   goes inert. Idempotent. */
void dg_bridge_stop(void);

void dg_bridge_stats(DG_BRIDGE_STATS *out);

#define DG_TURN_PROBE_RING 256
typedef struct {
    long tick;                 /* bridge tick of the sample */
    unsigned char byte;        /* the right_dx that was written (128 = none) */
    unsigned char wrote;       /* 1 = this tick wrote the byte, 0 = dense idle */
    long fire_state;           /* DG_FIRE_* at that tick */
    int pw_ok;
    unsigned long long flags;
    short turn_vy;             /* +0x8A - the stick's own consumer */
    short rot_vy;              /* +0x82 - the body yaw it eases into */
    short cam_vy;              /* +0xD22 - windowed-aim yaw */
    short cam_pad;             /* +0xD26 - window centre */
    unsigned long long action; /* +0xC60 - base act */
    unsigned long long action2;

    float gap_deg;             /* published aim gap = head facing - drift */
    float drift_deg;           /* body yaw drift (root yaw twist vs epoch) */
    float head_deg;            /* head facing term of the gap */
    float root_tilt_deg;       /* angle of the root's up from world up */
    long gap_age;              /* ticks since the gap was published */
} DG_TURN_PROBE_SAMPLE;
int dg_bridge_turn_probe_take(DG_TURN_PROBE_SAMPLE *out); /* 1 = got one */

typedef struct {
    int    live;               /* 0 = the constant, 1 = conjugate with q */
    int    valid;              /* 0 = no heading acquired; conversions refuse */
    double q[4];               /* yaw-only, {x, y, z, w} */
} DG_ADJ_FRAME;

#define DG_ADJ_CYCLE_RING 128
typedef struct {
    long  tick;               /* bridge tick of the Z read that closed it */
    short rot0, rot3;         /* rot.vy at the identity read / the Z read */
    short ok;                 /* both word reads resolved a player */
    float yaw_deg;            /* atan2(axisX.z, axisX.x) of the X case */
    float ang_deg[3];         /* rotation angle recovered per case X,Y,Z */
    float ydot;               /* axisY . world up (1 = pure yaw frame) */
} DG_ADJ_CYCLE;
int dg_bridge_adj_cycle_take(DG_ADJ_CYCLE *out);        /* 1 = got one */

/* The theater judgment, for the three consumers (camera hand-back in
   apply_transform, mono capture in Present, the quad on the XR side). Returns
   DG_THEATER_V_* bits, already gated: 0 unless the bridge is armed, the
   camera-seam publication is FRESH (a seam that stops firing must never pin
   the theater on), and the marker enabled that judgment's consumer half
   (vr_theater=on for DEMO, vr_ui=on for UI). MEASURE never sets a bit -
   that is the entire difference between measuring and switching. Safe from
   any thread; interlocked reads only. */
int dg_bridge_theater_verdict(void);
int dg_bridge_fake_gameover_now(void);
int dg_bridge_blade_braced(void);
/* vr_grip_ms as last configured, packed (dg_grip_pack); any thread. */
long dg_bridge_grip_packed(void);
/* vr_twohand_rifles / vr_twohand_steer as last configured: bit 0 rifles on,
   bit 1 steering on; any thread. */
long dg_bridge_twohand_flags(void);

#define DG_TWOHAND_F_REACH_RELEASE 4
#define DG_TWOHAND_F_ANCHOR_LIVE   8
/* vr_twohand_latch as last configured, packed enter_mm | exit_mm << 16; any thread. */
long dg_bridge_twohand_latch(void);
/* vr_psg_rifle as last configured (1 on). */
int dg_bridge_psg_rifle_enabled(void);
#include "dg_psg_gesture.h"
/* vr_psg_gesture thresholds as last configured (defaults when absent). */
void dg_bridge_psg_gesture_cfg(DG_PSG_GESTURE_CFG *out);
/* 1 while the PSG rifle lease is held AND the gesture says hip (scope
   closed): the camera must not apply the PSG zoom, the grips must not zoom
   and the shot leaves the muzzle. Game thread and camera thread. */
int dg_bridge_psg_hip_now(void);
long dg_bridge_fake_gameover_frames(void);
/* Read current status and ownership words at the camera seam. No cached
   heartbeat/fps verdict is sufficient for this gate. */
int dg_bridge_camera_gate_now(DG_CAMERA_GATE *out);
/* Central head camera, never a per-eye or controller matrix. NULL revokes. */
void dg_bridge_psg_camera(const float world[4][4]);
long dg_bridge_fps_entry_generation(void);
/* Confirmed safe view, independent of weapon/arm pose eligibility.
   1 = first person, 2 = an explicit return to third person, 0 = not ready. */
int dg_bridge_view_calibration_now(long *generation, unsigned long long *identity);
int dg_bridge_fps_recenter_ready(unsigned long long arm);
/* Read-only, safe owned FPS + grounded standing locomotion only. */
int dg_bridge_camera_standing_height_now(float *height);

void dg_bridge_screen_seam_now(void);

/* Hide original LIFE/radar frame only while the VR FPS bridge is active. */
void dg_bridge_native_hud_configure(int enabled);

#endif
