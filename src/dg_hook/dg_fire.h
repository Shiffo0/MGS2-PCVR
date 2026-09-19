

#ifndef DG_FIRE_H
#define DG_FIRE_H

#define DG_FIRE_TH              24
#define DG_FIRE_TH2             60
#define DG_FIRE_PRESSURE_MAX    255

/* Inside the cancel window on purpose, and never reachable by moving the
   trigger: the ordinary map floors at DG_FIRE_TH. So a cancel is always a
   decision by this state machine and never an accident of how far the player
   happened to pull. */
#define DG_FIRE_ABORT_PRESSURE  12
/* The game needs two consecutive ready-stance ticks in the window (it compares
   the current pressure against work->sv2.vx, which it stores each tick). Three
   costs nothing and survives a tick where the stance did not run. */
#define DG_FIRE_ABORT_TICKS     3

/* Separate aim travel (XR click) from the deliberate firing detent. */
#define DG_FIRE_TRIGGER_SHOOT   0.85
#define DG_FIRE_TRIGGER_REARM   0.70

/* WeaponSet.type bits, read live from work->wp_set->type rather than kept in a
   table here - the game already carries that table and ours could drift. */
/* Coolant uses held status in JetSpray, not the pistol release contract. */
#define DG_FIRE_WP_COOLANT      0x00008062u
#define DG_FIRE_WP_PRESSURE     0x0080u
#define DG_FIRE_WP_CONSECUTIVE  0x0010u

/* How old a controller sample may be, in game ticks, and still describe now.
 *
 * This is measured, not chosen: the 2026-08-19 dry run published 6142 trigger
 * samples across 7677 ticks - 0.8 per tick - because the camera seam and the
 * game tick are not locked to each other. Borrowing the hand command's bound of
 * 2 ticks called a quarter of all ticks stale.
 */
#define DG_FIRE_FRESH_TICKS     4

/* And how long an aim already in progress rides out a gap before the
 * controller counts as gone. A missing sample is not a missing hand: the first
 * costs nothing to wait out, the second has to lower the weapon without firing
 * it. At 60 ticks a second this is a third of a second, which is far longer
 * than any gap the dry run produced and far shorter than a player would take
 * to notice their aim had frozen.
 */
#define DG_FIRE_GRACE_TICKS     20

enum {
    DG_FIRE_IDLE = 0,
    DG_FIRE_DRAW,       /* one tick: press + status, raises the weapon */
    DG_FIRE_HOLD,       /* status held, pressure tracks the trigger */
    DG_FIRE_RELEASE,    /* one tick: status withheld -> the shot */
    DG_FIRE_ABORT,      /* soft pressure, lowers the weapon with no shot */
    DG_FIRE_STANDDOWN,  /* one tick after an abort, to clear the button */
    DG_FIRE_WAIT_REARM  /* pistol fired: wait for the finger below the detent */
};

enum {
    DG_FIRE_F_DREW          = 1u << 0,  /* a pull was accepted this tick */
    DG_FIRE_F_RELEASED      = 1u << 1,  /* status withheld: a pistol fires here */
    DG_FIRE_F_ABORTING      = 1u << 2,  /* running the cancel window */
    DG_FIRE_F_FORCED        = 1u << 3,  /* lost the write right while aiming */
    DG_FIRE_F_BLOCKED_PHYS  = 1u << 4,  /* the player's own button was down */
    DG_FIRE_F_BLOCKED_INPUT = 1u << 5,  /* no usable controller sample */
    DG_FIRE_F_REPEAT        = 1u << 6,  /* the same pull offered again */
    DG_FIRE_F_AUTO          = 1u << 7,  /* pressure is over the full-auto line */
    DG_FIRE_F_COASTING      = 1u << 8   /* holding an aim across a sample gap */
};

typedef struct {
    /* Monotonic per-hand counters from the XR layer's hysteresis. Levels lie
       when a frame is repeated or arrives twice; these do not. */
    unsigned long long press_seq;
    unsigned long long release_seq;
    double value;           /* raw trigger, 0..1 */
    double click;           /* the configured press threshold, 0..1 */
    /* Whether a sample could be read at all this tick, and how many game ticks
       ago the camera seam published it. Separate from input_ok on purpose: one
       says the crossing was quiet, the other says the hand is gone, and they
       call for opposite behaviour while a weapon is up. */
    int have_sample;
    int sample_age;
    int input_ok;           /* tracked, active, this hand is the driven one */
    int can_write;          /* every bridge gate: FPS, safety, player, weapon */
    int physical_down;      /* the game's own PL_PAD_WEAPON status bit */
    unsigned int wtype;     /* live WeaponSet.type */
    int native_ready;      /* ShootBullet: PLAYER_HOLD, ftime2 >= 8, data3 == 0 */
    int native_cancel_ready; /* ftime2 >= 8: pressure is consumed also at walls */
} DG_FIRE_IN;

typedef struct {
    int press;              /* OR PL_PAD_WEAPON into pad.press */
    int status;             /* OR PL_PAD_WEAPON into pad.status */
    int release;            /* OR PL_PAD_WEAPON into pad.release */
    int pressure;           /* store into pad.pressure[idx]; -1 = do not touch */
    int state;
    unsigned int flags;
} DG_FIRE_OUT;

typedef struct {
    int state;
    int ticks;                          /* ticks spent in the current state */
    int gap;                            /* consecutive ticks without a sample */
    int last_pressure;                  /* what to hold while coasting */
    unsigned long long handled_press;   /* the pull this machine already took */
    int started;
    int full_pull;                     /* automatic firing detent latch */
    unsigned int weapon_type;          /* abort if the live weapon changes type */
} DG_FIRE;

void dg_fire_reset(DG_FIRE *f);

/* The trigger's travel above the click threshold, mapped onto the game's
   pressure byte. Floors at DG_FIRE_TH so an aiming hold can never fall into
   the cancel window, and reaches DG_FIRE_PRESSURE_MAX at a full pull. Where
   DG_FIRE_TH2 falls inside that range is what decides how far an automatic
   weapon has to be pulled before it goes to work. */
int dg_fire_pressure(double value, double click);

/* One game tick. `out` is written on every call, including the refusals. */
void dg_fire_step(DG_FIRE *f, const DG_FIRE_IN *in, DG_FIRE_OUT *out);

#endif /* DG_FIRE_H */
