

#ifndef DG_MOVE_H
#define DG_MOVE_H

#define DG_MOVE_PAD_U           0x00001000u
#define DG_MOVE_PAD_R           0x00002000u
#define DG_MOVE_PAD_D           0x00004000u
#define DG_MOVE_PAD_L           0x00008000u
#define DG_MOVE_PAD_UDLR        0x0000F000u
#define DG_MOVE_ANALOG_L_USE    0x0004u
#define DG_MOVE_ANALOG_R_USE    0x0002u
#define DG_MOVE_GAME_MARGIN     32             /* get_analog_margin, BP build */

#define DG_MOVE_SUBJECT_MARGIN  49

#define DG_MOVE_PRONE_DEFLECT   120

/* Ticks a published sample stays actable. Same bar as the trigger: the stick
   is sampled at the camera seam (~5x/tick), so anything older than a few
   ticks means the publisher died and the character must stop, not coast. */
#define DG_MOVE_FRESH_TICKS     4

enum {
    DG_MOVE_F_ACTIVE  = 1u << 0,   /* out fields are meant to be written */
    DG_MOVE_F_YIELDED = 1u << 1,   /* physical movement input present */
    DG_MOVE_F_IDLE    = 1u << 2,   /* our stick is inside the deadzone */
    DG_MOVE_F_STALE   = 1u << 3,   /* sample too old to speak for a finger */
    DG_MOVE_F_CLAMPED = 1u << 4,   /* |stick| > 1 was pulled back to 1 */
    /* The turn half's own verdicts, independent of the walk's: a player
       holding the physical right stick must not lose walking, and vice
       versa. */
    DG_MOVE_F_TURN_ACTIVE  = 1u << 5,
    DG_MOVE_F_TURN_YIELDED = 1u << 6,
    DG_MOVE_F_TURN_IDLE    = 1u << 7,
};

typedef struct {
    /* The thumbstick as OpenXR reports it: +x right, +y up, both -1..1. */
    double x, y;
    int    have_sample;     /* a command was read from the channel at all */
    int    sample_age;      /* ticks since it was published */
    int    input_ok;        /* the XR side vouches for the controller */
    double deadzone;        /* 0..0.9; below this magnitude we stay silent */
    /* The turn half. turn_on gates it entirely; turn_x is the right
       stick's X as OpenXR reports it (+x right, -1..1); turn_gain scales the
       deflection before the byte mapping (the game's own hstep curve does
       the rest). */
    int    turn_on;
    double turn_x;
    double turn_gain;           /* 0.1..3.0; outside that, clamped */
    /* The pad exactly as the game's own driver left it this tick. */
    unsigned int  status;
    unsigned int  analog_input;
    unsigned char left_dx, left_dy;
    unsigned char right_dx;     /* the physical right stick, for the yield */
    /* Byte-deflection ceiling for the walk (0 = none). Values at or below
       the game margin are ignored, so a cap can never produce a stick the
       game discards. Used prone: see DG_MOVE_PRONE_DEFLECT. */
    int    max_deflect;

    int    dir_wanted;
    int    dir_org;             /* 0..4095 */
} DG_MOVE_IN;

typedef struct {
    int write;                     /* 1 = the caller should write the pad */
    unsigned char dx, dy;          /* the bytes to store */
    int turn_write;                /* 1 = the caller should write rdx */
    unsigned char rdx;             /* right_dx byte for the game's own turn */
    unsigned int  or_status;       /* UDLR bits to OR into status */
    unsigned int  or_analog;       /* flag bits to OR into analog_input */
    unsigned flags;                /* DG_MOVE_F_* */
    int dir_write;                 /* 1 = also store dir (below) at +0x10 */
    int dir;                       /* 0..4095, or -1 = inside the margin */
} DG_MOVE_OUT;

int dg_move_pad_dir(unsigned char dx, unsigned char dy, int org);

#define DG_MOVE_FORCE_MARGIN    48
int dg_move_pad_force(unsigned char dx, unsigned char dy);

/* Stateless: one sample in, one decision out. Never writes anything itself -
   the caller owns game memory - and write==0 always comes with every out
   field zeroed, so a caller that forgets to check cannot smear stale bytes. */
void dg_move_step(const DG_MOVE_IN *in, DG_MOVE_OUT *out);

#endif
