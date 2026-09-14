/* dg_menu.c - pure decision logic for driving a full-screen 2D menu from a
   VR controller. See dg_menu.h for the contract and for why every emission is
   a ONE-FRAME pulse. */

#include <math.h>
#include <string.h>
#include "dg_menu.h"

static int finite_ok(double v)
{
    /* MSVC C: no isfinite in old modes; the project's pure modules use this
       identity form throughout. */
    return v == v && v <= 1.0e12 && v >= -1.0e12;
}

static int clamp_int(int v, int lo, int hi, int fallback)
{
    if (v < lo || v > hi) return fallback;
    return v;
}

/* Wrap-safe "has this deadline passed?". now_ms is a millisecond tick that
   wraps every 49 days; comparing the two as unsigned magnitudes would, on the
   wrap, either fire every frame for a month or never fire again. The signed
   difference is correct across the wrap and needs no special case. */
static int due(unsigned long now, unsigned long deadline)
{
    return (long)(now - deadline) >= 0;
}

/* Is the configured pair usable at all?
 *
 * Three ways it is not, and each of them would do damage rather than nothing:
 * a zero bit writes no button while every counter says it did; equal bits mean
 * confirm and cancel are the same press; and a bit inside PAD_UDLR would make
 * "confirm" also move the cursor one step before committing - a menu that
 * picks the wrong entry every time. Outside the allowed set is refused too:
 * the marker exists to choose among the game's buttons, not to reach into the
 * rest of the status word. */
static int config_ok(const DG_MENU_IN *in)
{
    unsigned int both = in->confirm_bit | in->cancel_bit;
    if (!in->confirm_bit || !in->cancel_bit) return 0;
    if (in->confirm_bit & in->cancel_bit) return 0;
    if (both & DG_MENU_PAD_UDLR) return 0;
    if (both & ~DG_MENU_PAD_ALLOWED) return 0;
    return 1;
}

/* Is the confirm control down THIS frame - not whether it just went down.
 *
 * The threshold is guarded here rather than trusted from the caller, and the
 * reason is a specific accident: the test is `>=`, so a click of 0.0 makes an
 * untouched trigger read as held, and the very first front-end frame then
 * confirms whatever the menu happens to be offering. A marker cannot reach
 * that today - the XR config validates trigger_fire into (deadzone, 1.0] - but
 * "cannot reach it today" is not the same as safe, and this module is supposed
 * to be usable from a caller that has not read that validation.
 *
 * The face button and the trigger share one level, and therefore one edge:
 * they are the same intent, and pulling the trigger and then also pressing A
 * must not confirm twice. */
static int confirm_level(const DG_MENU_IN *in)
{
    double click = in->trigger_click;
    if (in->button_a) return 1;
    if (!finite_ok(click) || click <= 0.0 || click > 1.0) click = 0.55;
    return in->trigger >= click ? 1 : 0;
}

static unsigned int direction_bit(double x, double y, double deadzone,
                                  unsigned int *flags)
{
    double magnitude = sqrt(x * x + y * y);
    double ax, ay;

    if (magnitude > 1.0) {
        /* A runtime may report the corners of a square stick as > 1. */
        x /= magnitude;
        y /= magnitude;
        magnitude = 1.0;
        *flags |= DG_MENU_F_CLAMPED;
    }
    if (magnitude <= deadzone) return 0;

    ax = x < 0.0 ? -x : x;
    ay = y < 0.0 ? -y : y;
    if (ay >= ax) return y > 0.0 ? DG_MENU_PAD_U : DG_MENU_PAD_D;
    return x > 0.0 ? DG_MENU_PAD_R : DG_MENU_PAD_L;
}

/* The window is closed, or we cannot act in it. Everything that could turn
   into a phantom press when it reopens is dropped here - but the BUTTON LEVELS
   are kept, and that is the whole point of the function.
   A trigger held from gameplay into a load screen must not confirm the first
   thing the menu offers. Remembering that it was already down means the next
   call sees no rising edge, which is exactly right: the player has to let go
   and press again. */
static void close_window(DG_MENU_STATE *state, const DG_MENU_IN *in,
                         int track_buttons)
{
    state->dir = 0;
    state->have_timer = 0;
    state->next_fire_ms = 0;
    state->window_open = 0;
    if (track_buttons) {
        state->confirm_was = confirm_level(in);
        state->cancel_was = in->button_b ? 1 : 0;
    }
}

void dg_menu_step(DG_MENU_STATE *state, const DG_MENU_IN *in,
                  DG_MENU_OUT *out)
{
    unsigned int want;
    unsigned int physical_guard;
    int initial_ms;
    int repeat_ms;
    double deadzone;
    int confirm_down;
    int cancel_down;

    if (!out) return;
    memset(out, 0, sizeof *out);
    if (!state || !in) return;

    if (!in->front_end) {
        /* Levels still tracked: see close_window. The stick is not, because a
           stick held across the boundary is harmless - it produces a fresh
           direction change on the first open frame, which is a step the player
           is asking for. */
        close_window(state, in, 1);
        out->flags = DG_MENU_F_CLOSED;
        return;
    }

    /* An unusable button pair is refused whole. Not "navigate but do not
       confirm": half a control scheme in a menu the player cannot see their
       way out of is worse than none. */
    if (!config_ok(in)) {
        close_window(state, in, 0);
        out->flags = DG_MENU_F_BAD_CONFIG;
        return;
    }

    if (!in->have_sample || !in->input_ok) {
        /* Levels deliberately NOT updated: we do not know them. A controller
           that drops out mid-press and returns still pressed then produces no
           edge, and a genuine release-then-press still does. */
        state->dir = 0;
        state->have_timer = 0;
        out->flags = DG_MENU_F_NO_INPUT;
        return;
    }
    if (!finite_ok(in->x) || !finite_ok(in->y) || !finite_ok(in->trigger) ||
        !finite_ok(in->deadzone)) {
        state->dir = 0;
        state->have_timer = 0;
        out->flags = DG_MENU_F_BAD_INPUT;
        return;
    }

    state->window_open = 1;

    physical_guard = DG_MENU_PAD_UDLR | in->confirm_bit | in->cancel_bit;
    if (in->phys_known && (in->phys_status & physical_guard)) {
        state->dir = 0;
        state->have_timer = 0;
        state->confirm_was = confirm_level(in);
        state->cancel_was = in->button_b ? 1 : 0;
        out->flags = DG_MENU_F_YIELDED;
        return;
    }

    deadzone = in->deadzone;
    if (deadzone < 0.05) deadzone = 0.05;
    if (deadzone > 0.9) deadzone = 0.9;
    initial_ms = clamp_int(in->initial_ms, DG_MENU_MIN_INITIAL_MS,
                           DG_MENU_MAX_INITIAL_MS,
                           DG_MENU_DEFAULT_INITIAL_MS);
    repeat_ms = clamp_int(in->repeat_ms, DG_MENU_MIN_REPEAT_MS,
                          DG_MENU_MAX_REPEAT_MS, DG_MENU_DEFAULT_REPEAT_MS);

    want = direction_bit(in->x, in->y, deadzone, &out->flags);

    if (want != state->dir) {
        /* A direction CHANGE always steps immediately, including the change
           from nothing to something. Waiting would make the first press of
           every menu feel broken, and rolling the stick from up to left would
           silently skip the left step. */
        state->dir = want;
        if (want) {
            out->status |= want;
            out->flags |= DG_MENU_F_STEP;
            state->next_fire_ms = in->now_ms + (unsigned long)initial_ms;
            state->have_timer = 1;
        } else {
            state->have_timer = 0;
        }
    } else if (want && state->have_timer && due(in->now_ms,
                                                state->next_fire_ms)) {
        out->status |= want;
        out->flags |= DG_MENU_F_STEP | DG_MENU_F_REPEAT;
        /* From NOW, not from the missed deadline. A frame hitch must not be
           repaid as a burst of steps the player never asked for. */
        state->next_fire_ms = in->now_ms + (unsigned long)repeat_ms;
    }

    confirm_down = confirm_level(in);
    cancel_down = in->button_b ? 1 : 0;

    if (confirm_down && !state->confirm_was) {
        out->status |= in->confirm_bit;
        out->flags |= DG_MENU_F_CONFIRM;
    }
    if (cancel_down && !state->cancel_was) {
        out->status |= in->cancel_bit;
        out->flags |= DG_MENU_F_CANCEL;
    }
    state->confirm_was = confirm_down;
    state->cancel_was = cancel_down;

    if (out->status) {
        out->write = 1;
    } else {
        out->flags |= DG_MENU_F_IDLE;
    }
}
