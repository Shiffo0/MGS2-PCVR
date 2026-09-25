/* dg_move.c - pure decision logic for left-stick subjective walking. */

#include <math.h>
#include <string.h>
#include "dg_move.h"

static int finite_ok(double v)
{
    /* MSVC C: no isfinite in old modes; the project's pure modules use this
       identity form throughout. */
    return v == v && v <= 1.0e12 && v >= -1.0e12;
}





static int physically_moving(const DG_MOVE_IN *in)
{
    int dx = (int)in->left_dx - 128;
    int dy = (int)in->left_dy - 128;
    if (dx < 0) dx = -dx;
    if (dy < 0) dy = -dy;
    if (in->status & DG_MOVE_PAD_UDLR) return 1;
    if (in->analog_input & DG_MOVE_ANALOG_L_USE) return 1;
    return dx > DG_MOVE_GAME_MARGIN || dy > DG_MOVE_GAME_MARGIN;
}

/* And the same question for the right stick, which is the turn's to yield
   to. Left and right are judged separately on purpose: a player steering
   with a physical right stick has not surrendered walking, nor the other way
   around. */
static int physically_turning(const DG_MOVE_IN *in)
{
    int dx = (int)in->right_dx - 128;
    if (dx < 0) dx = -dx;
    if (in->analog_input & DG_MOVE_ANALOG_R_USE) return 1;
    return dx > DG_MOVE_GAME_MARGIN;
}

static int margin_scaled_byte(double mag, double dz)
{
    double scaled = (mag - dz) / (1.0 - dz);
    if (scaled > 1.0) scaled = 1.0;
    return (int)floor((double)DG_MOVE_SUBJECT_MARGIN +
                      scaled * (double)(127 - DG_MOVE_SUBJECT_MARGIN) +
                      0.5);
}

/* The turn half, evaluated after the walk half so its flags simply join the
   same word. Kept out of dg_move_step's early returns: a stale or yielded
   WALK says nothing about the finger on the OTHER stick. */
int dg_move_pad_dir(unsigned char dx, unsigned char dy, int org)
{

    float vx = (float)((int)dx - 128), vz = (float)((int)dy - 128);
    float m = (float)DG_MOVE_GAME_MARGIN;
    float d;
    short res;
    if (vx > -m && vx < m && vz > -m && vz < m) return -1;
    d = atan2f(vx, vz);
    res = (short)(4095 & (short)((d * 2048.0f / 3.14159265358979f) + 0.5f));
    return ((int)res + org) & 4095;
}

int dg_move_pad_force(unsigned char dx, unsigned char dy)
{
    int vx = (int)dx - 128, vy = (int)dy - 128;
    int mag = (int)(sqrtf((float)(vx * vx + vy * vy)) + 0.5f);
    float f;
    if (mag > 128) mag = 128;
    if (mag < DG_MOVE_FORCE_MARGIN) return 0;
    f = (float)(mag - DG_MOVE_FORCE_MARGIN) * 256.0f / (float)(128 - DG_MOVE_FORCE_MARGIN);
    return (int)f;
}

static void turn_step(const DG_MOVE_IN *in, DG_MOVE_OUT *out)
{
    double x, mag, dz, gain;
    int b;

    if (!in->turn_on) return;
    if (!finite_ok(in->turn_x)) return;

    if (physically_turning(in)) {
        out->flags |= DG_MOVE_F_TURN_YIELDED;
        return;
    }

    gain = in->turn_gain;
    if (!finite_ok(gain) || gain < 0.1) gain = 0.1;
    if (gain > 3.0) gain = 3.0;
    x = in->turn_x * gain;
    mag = x < 0.0 ? -x : x;
    if (mag > 1.0) mag = 1.0;

    dz = in->deadzone;
    if (dz < 0.0) dz = 0.0;
    if (dz > 0.9) dz = 0.9;
    if (mag <= dz) {
        out->flags |= DG_MOVE_F_TURN_IDLE;
        return;
    }

    b = margin_scaled_byte(mag, dz);

    out->rdx = (unsigned char)(x >= 0.0 ? 128 + b : 128 - b);
    out->or_analog |= DG_MOVE_ANALOG_R_USE;
    out->turn_write = 1;
    out->flags |= DG_MOVE_F_TURN_ACTIVE;
}

void dg_move_step(const DG_MOVE_IN *in, DG_MOVE_OUT *out)
{
    double x, y, mag, dz, scaled, k;
    int bx, by;

    memset(out, 0, sizeof *out);
    if (!in || !in->have_sample || !in->input_ok) return;
    if (!finite_ok(in->x) || !finite_ok(in->y) || !finite_ok(in->deadzone))
        return;

    if (in->sample_age < 0 || in->sample_age > DG_MOVE_FRESH_TICKS) {
        out->flags = DG_MOVE_F_STALE;
        return;
    }

    /* The turn rides the same freshness but neither of the walk's other
       refusals: each stick answers for its own finger. */
    turn_step(in, out);

    /* The player first, always. Note the order: a stale check cannot yield
       and a yield cannot look idle - each refusal names its real reason. */
    if (physically_moving(in)) {
        out->flags |= DG_MOVE_F_YIELDED;
        return;
    }

    x = in->x;
    y = in->y;
    mag = sqrt(x * x + y * y);
    if (mag > 1.0) {
        /* A runtime may report corners of a square stick as > 1. */
        x /= mag;
        y /= mag;
        mag = 1.0;
        out->flags |= DG_MOVE_F_CLAMPED;
    }

    dz = in->deadzone;
    if (dz < 0.0) dz = 0.0;
    if (dz > 0.9) dz = 0.9;
    if (mag <= dz) {
        out->flags |= DG_MOVE_F_IDLE;
        return;
    }

    /* Rescale the live range so our first speakable deflection already clears
       the game's own margin. deadzone..1 maps onto (MARGIN+1)..127; without
       this a stick at 20% passes our gate, lands inside the game's margin,
       and is discarded there - a walk that "works" in every counter and moves
       nobody. */
    scaled = (mag - dz) / (1.0 - dz);
    scaled = (double)(DG_MOVE_GAME_MARGIN + 1) +
             scaled * (double)(127 - (DG_MOVE_GAME_MARGIN + 1));
    /* The prone ceiling: magnitude only, direction untouched. */
    if (in->max_deflect > DG_MOVE_GAME_MARGIN + 1 &&
        scaled > (double)(in->max_deflect > 127 ? 127 : in->max_deflect))
        scaled = (double)(in->max_deflect > 127 ? 127 : in->max_deflect);
    k = scaled / mag;

    bx = 128 + (int)floor(x * k + 0.5);
    by = 128 - (int)floor(y * k + 0.5);
    if (bx < 1) bx = 1;
    if (bx > 255) bx = 255;
    if (by < 1) by = 1;
    if (by > 255) by = 255;

    out->dx = (unsigned char)bx;
    out->dy = (unsigned char)by;
    out->or_analog = DG_MOVE_ANALOG_L_USE;

    if (bx < 128 - DG_MOVE_GAME_MARGIN) out->or_status |= DG_MOVE_PAD_L;
    else if (bx > 128 + DG_MOVE_GAME_MARGIN) out->or_status |= DG_MOVE_PAD_R;
    if (by < 128 - DG_MOVE_GAME_MARGIN) out->or_status |= DG_MOVE_PAD_U;
    else if (by > 128 + DG_MOVE_GAME_MARGIN) out->or_status |= DG_MOVE_PAD_D;

    out->write = 1;
    out->flags |= DG_MOVE_F_ACTIVE;
    if (in->dir_wanted) {
        out->dir = dg_move_pad_dir(out->dx, out->dy, in->dir_org & 4095);
        out->dir_write = 1;
    }
}
