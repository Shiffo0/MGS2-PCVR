

#ifndef DG_ARM_MAP_H
#define DG_ARM_MAP_H

enum {
    /* This call created the calibration anchor. */
    DG_ARM_MAP_F_CALIBRATED     = 1u << 0,
    /* There is deliberately no writable target on this call. */
    DG_ARM_MAP_F_NOT_READY      = 1u << 1,
    /* Target was outside upper+fore and was pulled back on the same ray. */
    DG_ARM_MAP_F_CLAMP_REACH    = 1u << 2,
    /* Target was inside |upper-fore| and was pushed out on the same ray. */
    DG_ARM_MAP_F_CLAMP_TOO_CLOSE = 1u << 3,
    /* NULL, non-finite, zero-span or inconsistent character input. */
    DG_ARM_MAP_F_BAD_INPUT      = 1u << 4,
    /* Target entered the continuous outer reach-compression zone. */
    DG_ARM_MAP_F_SOFT_REACH     = 1u << 5
};

typedef struct {
    double controller_view[3];
    double shoulder_view[3];
    double native_wrist_view[3];
    /* The PLAYER's own shoulder, in the same frame and units as
       controller_view - which is head-relative, so this is an anthropometric
       offset from the headset and not anything the game knows. Read only when
       anchor_shoulder is set, and required to be finite when it is. */
    double player_shoulder[3];
    /* The PLAYER's shoulder-to-wrist length at full extension, same units.
       Anthropometry again; required to be finite and positive when
       anchor_shoulder is set, and unread otherwise. */
    double player_reach;
    double upper;               /* live shoulder-to-elbow length, mm */
    double fore;                /* live elbow-to-wrist length, mm */
    int anchor_shoulder;        /* 0 = offset anchor, 1 = shoulder anchor */
    int explicit_target;        /* camera-derived root-local target; same reach policy */
    double desired_view[3];
} DG_ARM_MAP_IN;

typedef struct {
    double target_view[3];      /* usable only when dg_arm_map_step returns 1 */
    double controller_delta[3]; /* since calibration, before scaling */
    double scale;
    double source_span;         /* calibration controller-to-shoulder span */
    double reach_used;          /* |target_view - live shoulder_view| */
    unsigned flags;             /* DG_ARM_MAP_F_* */
} DG_ARM_MAP_OUT;

/* Caller-owned state.  Do not edit members directly; reset is explicit so a
   tracking/FPS/character transition cannot accidentally reuse an old anchor. */
typedef struct {
    double controller0_view[3];
    double wrist0_view[3];
    double shoulder0_view[3];   /* stable root-local anatomical reference */
    double source_span;
    double upper;
    double fore;
    int calibrated;
    /* Which anchor the span above was measured for. Changing anchor without a
       reset would keep one anchor's scale and apply it to the other's
       geometry, so the mismatch is BAD_INPUT rather than a silent blend. */
    int anchor_shoulder;
} DG_ARM_MAP_STATE;

/* Clears the calibration.  The next valid step will be CALIBRATED|NOT_READY. */
void dg_arm_map_reset(DG_ARM_MAP_STATE *state);

/* Returns 1 only when target_view may be handed to the IK solver.  Returns 0
   for first-sample NOT_READY and for BAD_INPUT.  BAD_INPUT never changes a
   previously valid calibration and *out is zero apart from its flags. */
int dg_arm_map_step(DG_ARM_MAP_STATE *state, const DG_ARM_MAP_IN *in,
                    DG_ARM_MAP_OUT *out);

#endif
