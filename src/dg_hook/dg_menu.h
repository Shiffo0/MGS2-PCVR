

#ifndef DG_MENU_H
#define DG_MENU_H

#define DG_MENU_PAD_U       0x00001000u
#define DG_MENU_PAD_D       0x00004000u
#define DG_MENU_PAD_L       0x00008000u
#define DG_MENU_PAD_R       0x00002000u
#define DG_MENU_PAD_UDLR    0x0000F000u

#define DG_MENU_PAD_X       0x00000010u  /* PS2 triangle */
#define DG_MENU_PAD_A       0x00000020u  /* PS2 circle   */
#define DG_MENU_PAD_B       0x00000040u  /* PS2 cross    */
#define DG_MENU_PAD_Y       0x00000080u  /* PS2 square   */
#define DG_MENU_PAD_SEL     0x00000100u
#define DG_MENU_PAD_STA     0x00000800u
#define DG_MENU_PAD_TITLE_ENTER 0x00100000u /* retail title direct.press bit 20 */
/* Bounded XR mailbox; confirmation intent is kept separate from mapped bits. */
#define DG_MENU_ALLOW_XR 3
#define DG_MENU_ALLOW_XR_CONFIRM 4

/* Everything this module is ever allowed to name. A configured button outside
   this set is refused rather than written: the point of the marker is to pick
   among the game's buttons, not to reach arbitrary bits of the status word. */

#define DG_MENU_PAD_EXTENDED 0x00FF0000u

#define DG_MENU_PAD_ALLOWED \
    (DG_MENU_PAD_UDLR | DG_MENU_PAD_X | DG_MENU_PAD_A | DG_MENU_PAD_B | \
     DG_MENU_PAD_Y | DG_MENU_PAD_SEL | DG_MENU_PAD_STA | \
     DG_MENU_PAD_EXTENDED)

#define DG_MENU_DEFAULT_CONFIRM DG_MENU_PAD_B
#define DG_MENU_DEFAULT_CANCEL  DG_MENU_PAD_A

/* Repeat shape. 350 ms before the first repeat and 120 ms between them is the
   ordinary desktop cadence; both are marker-settable and both are clamped
   here so no marker can produce a divide-by-nothing or a pulse train. */
#define DG_MENU_DEFAULT_INITIAL_MS 350
#define DG_MENU_DEFAULT_REPEAT_MS  120
#define DG_MENU_MIN_INITIAL_MS      50
#define DG_MENU_MAX_INITIAL_MS    2000
#define DG_MENU_MIN_REPEAT_MS       40
#define DG_MENU_MAX_REPEAT_MS     1000

/* A menu step wants a decisive push, not a lean: half deflection. Well above
   the walking deadzone on purpose - this stick is not steering anything. */
#define DG_MENU_DEFAULT_DEADZONE 0.5

/* ------------------------------------------------------------- verdicts --- */

#define DG_MENU_F_CLOSED     0x0001u /* no front-end screen is up */
#define DG_MENU_F_NO_INPUT   0x0002u /* no frame, or the controller is not vouched for */
#define DG_MENU_F_BAD_INPUT  0x0004u /* a stick axis was not a finite number */
#define DG_MENU_F_BAD_CONFIG 0x0008u /* the configured buttons are unusable */
#define DG_MENU_F_YIELDED    0x0010u /* the player's own pad is being used */
#define DG_MENU_F_IDLE       0x0020u /* inside the deadzone, no button edge */
#define DG_MENU_F_STEP       0x0040u /* a direction pulse this frame */
#define DG_MENU_F_REPEAT     0x0080u /* ...and it was an auto-repeat, not the first */
#define DG_MENU_F_CONFIRM    0x0100u
#define DG_MENU_F_CANCEL     0x0200u
#define DG_MENU_F_CLAMPED    0x0400u /* the runtime reported |stick| > 1 */

typedef struct {
    /* Where we are. front_end is the caller's judgment that a full-screen 2D
       menu is being shown; the module never guesses at it. */
    int front_end;
    int have_sample;                /* an XR frame was actually read */
    int input_ok;                   /* that frame vouches for this controller */

    double x, y;                    /* thumbstick, OpenXR: +x right, +y up */
    double deadzone;

    double trigger;                 /* raw travel, 0..1 */
    double trigger_click;           /* the configured press threshold */
    int    button_a;                /* controller A / X - confirm, beside the trigger */
    int    button_b;                /* controller B / Y - cancel */

    unsigned long now_ms;           /* monotonic; wrap is handled */
    int initial_ms;
    int repeat_ms;

    unsigned int confirm_bit;       /* game status bits, from the marker */
    unsigned int cancel_bit;

    /* The player's own pad, when the caller can read it. Without this there is
       no yield - which is why a caller that cannot read it must also not be
       writing anything. */
    unsigned int phys_status;
    int phys_known;
} DG_MENU_IN;

typedef struct {
    unsigned int status;            /* the bits to arm this frame; 0 = none */
    int write;                      /* status != 0 and it should be armed */
    unsigned int flags;
} DG_MENU_OUT;

/* Carried between calls by the caller, so the module itself stays a pure
   function of (state, input). Zero it once; dg_menu_step owns it after that. */
typedef struct {
    unsigned int dir;               /* the direction bit currently held, 0 = none */
    unsigned long next_fire_ms;
    int have_timer;
    int confirm_was;                /* button levels, for edge detection */
    int cancel_was;
    int window_open;                /* front_end held on the previous call */
} DG_MENU_STATE;

void dg_menu_step(DG_MENU_STATE *state, const DG_MENU_IN *in, DG_MENU_OUT *out);

static unsigned int dg_menu_native_confirm(unsigned int status,int confirm,int pregame,int gameover)
{
    status &= DG_MENU_PAD_ALLOWED;
    if (!confirm) return status;
    if (gameover) return (status & ~DG_MENU_PAD_SEL) | DG_MENU_PAD_STA;
    if (pregame) return status | DG_MENU_PAD_TITLE_ENTER;
    return status;
}

#endif
