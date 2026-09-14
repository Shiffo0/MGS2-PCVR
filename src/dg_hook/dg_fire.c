#include "dg_fire.h"

void dg_fire_reset(DG_FIRE *f)
{
    if (!f) return;
    f->state = DG_FIRE_IDLE;
    f->ticks = 0;
    f->gap = 0;
    f->last_pressure = DG_FIRE_TH;
    f->handled_press = 0;
    f->started = 0;
    f->full_pull = 0;
    f->weapon_type = 0;
}

int dg_fire_pressure(double value, double click)
{
    double span, t;
    int p;

    if (!(click > 0.0) || !(click < 1.0)) click = 0.55;
    span = 1.0 - click;
    t = (value - click) / span;
    if (!(t > 0.0)) t = 0.0;          /* also catches NaN */
    if (t > 1.0) t = 1.0;

    p = DG_FIRE_TH + (int)(t * (double)(DG_FIRE_PRESSURE_MAX - DG_FIRE_TH) + 0.5);
    if (p < DG_FIRE_TH) p = DG_FIRE_TH;
    if (p > DG_FIRE_PRESSURE_MAX) p = DG_FIRE_PRESSURE_MAX;
    return p;
}

static void clear(DG_FIRE_OUT *out)
{
    out->press = 0;
    out->status = 0;
    out->release = 0;
    out->pressure = -1;
    out->flags = 0;
}

/* Held is derived from the two monotonic counters rather than from the level,
   so a repeated or duplicated XR frame cannot invent an edge. */
static int held(const DG_FIRE_IN *in)
{
    return in->press_seq > in->release_seq;
}

static void enter(DG_FIRE *f, int state)
{
    f->state = state;
    f->ticks = 0;
}

/* A sample that still describes now: one that was readable, that the XR side
   vouched for, and that is not older than the crossing's own cadence. */
static int fresh(const DG_FIRE_IN *in)
{
    return in->have_sample && in->input_ok &&
           in->value >= 0.0 && in->value <= 1.0 &&
           in->sample_age >= 0 && in->sample_age <= DG_FIRE_FRESH_TICKS;
}

void dg_fire_step(DG_FIRE *f, const DG_FIRE_IN *in, DG_FIRE_OUT *out)
{
    int aiming, current, threshold_weapon;

    if (!f || !in || !out) {
        if (out) { clear(out); out->state = DG_FIRE_IDLE; }
        return;
    }
    clear(out);

    /* Losing the right to write is not the same as losing the controller. The
       gates that can take it away - leaving first person, a codec, a cutscene,
       the player work going away - also take away the ready stance the shot
       would have come from, so there is nothing left to fire and nothing we
       could write anyway. Recorded, because a build that silently drops aim
       commands should be visible in the log. */
    if (!in->can_write) {
        if (f->state != DG_FIRE_IDLE) out->flags |= DG_FIRE_F_FORCED;
        enter(f, DG_FIRE_IDLE);
        out->state = f->state;
        return;
    }

    current = fresh(in);
    threshold_weapon = ((in->wtype | (f->state != DG_FIRE_IDLE ? f->weapon_type : 0)) &
                        (DG_FIRE_WP_PRESSURE | DG_FIRE_WP_CONSECUTIVE)) != 0;
    if (current) f->gap = 0;
    else if (f->gap < DG_FIRE_GRACE_TICKS + 1) f->gap++;

    aiming = (f->state == DG_FIRE_DRAW || f->state == DG_FIRE_HOLD);

    if (aiming && !current) {
        if (f->gap <= DG_FIRE_GRACE_TICKS) {
            /* Coast. The crossing went quiet for a tick or two, which the
               measured cadence says is ordinary; the hand has not gone
               anywhere. Hold exactly what was last asked for - dropping the
               status bit here is the shot, and re-deriving pressure from a
               sample we do not have would be inventing input. */
            out->status = 1;
            /* A gap must never sustain an automatic burst. Keep the stance
               up at aim pressure, but require a fresh deep pull to fire. */
            out->pressure = threshold_weapon ? DG_FIRE_TH : f->last_pressure;
            out->flags |= DG_FIRE_F_COASTING;
            if (out->pressure >= DG_FIRE_TH2) out->flags |= DG_FIRE_F_AUTO;
            out->state = f->state;
            return;
        }
        /* Long enough to mean the controller itself is gone. This is the case
           the cancel window exists for: the stance is real, the player is
           still standing there, and simply going quiet would fire the gun. */
        out->flags |= DG_FIRE_F_BLOCKED_INPUT;
        enter(f, DG_FIRE_ABORT);
    }

    switch (f->state) {

    case DG_FIRE_IDLE:
        if (!current) {
            out->flags |= DG_FIRE_F_BLOCKED_INPUT;
            break;
        }
        if (!held(in)) break;
        /* One pull, one draw. A pull is identified by its sequence number, so
           replayed samples, both eyes of a stereo pair and several camera
           seams inside one tick all describe the same pull rather than
           several. */
        if (f->started && in->press_seq <= f->handled_press) {
            out->flags |= DG_FIRE_F_REPEAT;
            break;
        }
        /* Never take a button the player is already holding. Their input owns
           the stance; ours would only be able to end it. */
        if (in->physical_down) {
            /* Consumed, not merely skipped: a pull that began underneath the
               player's own button must not quietly take over the stance the
               moment they let go. They pull again, or nothing happens. */
            f->handled_press = in->press_seq;
            f->started = 1;
            out->flags |= DG_FIRE_F_BLOCKED_PHYS;
            break;
        }
        f->handled_press = in->press_seq;
        f->started = 1;
        f->weapon_type = in->wtype;
        f->full_pull = 0;
        enter(f, DG_FIRE_DRAW);
        /* fall through: the draw is this tick, not the next one */

    case DG_FIRE_DRAW:
        f->full_pull = threshold_weapon && in->value >= DG_FIRE_TRIGGER_SHOOT;
        out->press = 1;
        out->status = 1;
        out->pressure = threshold_weapon ? DG_FIRE_TH : dg_fire_pressure(in->value, in->click);
        f->last_pressure = out->pressure;
        out->flags |= DG_FIRE_F_DREW;
        if (out->pressure >= DG_FIRE_TH2) out->flags |= DG_FIRE_F_AUTO;
        enter(f, DG_FIRE_HOLD);
        break;

    case DG_FIRE_HOLD:
        if (threshold_weapon && (!held(in) || in->wtype != f->weapon_type)) {
            enter(f, DG_FIRE_ABORT);
            goto abort_tick;
        }
        if (threshold_weapon) {
            if (in->value >= DG_FIRE_TRIGGER_SHOOT) f->full_pull = 1;
            else if (in->value <= DG_FIRE_TRIGGER_REARM) f->full_pull = 0;
            if ((in->wtype & DG_FIRE_WP_PRESSURE) && f->full_pull &&
                in->native_ready) {
                /* The native pistol release happens at the firing detent,
                   only once the real ready stance can consume it. */
                out->release = 1;
                out->pressure = 0;
                out->flags |= DG_FIRE_F_RELEASED;
                enter(f, DG_FIRE_WAIT_REARM);
                break;
            }
            out->status = 1;
            out->pressure = ((in->wtype & DG_FIRE_WP_CONSECUTIVE) &&
                             f->full_pull) ?
                            DG_FIRE_PRESSURE_MAX : DG_FIRE_TH;
            f->last_pressure = out->pressure;
            if (out->pressure >= DG_FIRE_TH2) out->flags |= DG_FIRE_F_AUTO;
            break;
        }
        if (!held(in)) {
            enter(f, DG_FIRE_RELEASE);
            /* fall through: the shot is this tick */
        } else {
            out->status = 1;
            out->pressure = dg_fire_pressure(in->value, in->click);
            f->last_pressure = out->pressure;
            if (out->pressure >= DG_FIRE_TH2) out->flags |= DG_FIRE_F_AUTO;
            f->ticks++;
            break;
        }
        /* fall through */

    case DG_FIRE_RELEASE:
        /* Status is withheld rather than cleared - we never clear a bit, and
           for a pistol this withholding is the shot. The release bit is set
           because that is what the pad would carry, and it is what tells the
           Bluepoint layer to holster. */
        out->release = 1;
        out->pressure = 0;
        out->flags |= DG_FIRE_F_RELEASED;
        enter(f, DG_FIRE_IDLE);
        break;

    case DG_FIRE_ABORT:
    abort_tick:
        out->status = 1;
        out->pressure = DG_FIRE_ABORT_PRESSURE;
        out->flags |= DG_FIRE_F_ABORTING;
        /* A quick release while drawing must wait for the native handler;
           it ignores pressure before ftime2==8. Once cancellation starts,
           data3 may leave zero, so finish the remaining soft ticks. */
        if (!threshold_weapon || in->native_cancel_ready || f->ticks > 0) {
            if (++f->ticks >= DG_FIRE_ABORT_TICKS) enter(f, DG_FIRE_STANDDOWN);
        }
        break;

    case DG_FIRE_WAIT_REARM:
        if (!current) break;
        if (!held(in)) {
            enter(f, DG_FIRE_IDLE);
        } else if (in->value <= DG_FIRE_TRIGGER_REARM &&
                   in->wtype == f->weapon_type) {
            /* A deliberate shallow pull re-arms; a held deep sample cannot
               produce another draw or another shot. */
            f->full_pull = 0;
            enter(f, DG_FIRE_DRAW);
            out->press = 1;
            out->status = 1;
            out->pressure = DG_FIRE_TH;
            f->last_pressure = DG_FIRE_TH;
            out->flags |= DG_FIRE_F_DREW;
            enter(f, DG_FIRE_HOLD);
        }
        break;

    case DG_FIRE_STANDDOWN:
        out->release = 1;
        out->pressure = 0;
        enter(f, DG_FIRE_IDLE);
        break;

    default:
        enter(f, DG_FIRE_IDLE);
        break;
    }

    out->state = f->state;
}
