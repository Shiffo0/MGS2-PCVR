#ifndef DG_BLADE_H
#define DG_BLADE_H
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include "dg_grip_blend.h"

typedef struct {
    uint64_t source, sequence, context, now_ms;
    int64_t time_ns;
    int valid, allowed, grip;
    double hand[3], head[3], forward[3], head_forward[3];
    /* The other (left) hand: block brace. Not part of stroke recognition. */
    int left_valid, left_grip;
    double left[3];
} DG_BLADE_SAMPLE;

#define DG_BLADE_START_SPEED 0.8   /* m/s, head-relative: a stroke begins */
#define DG_BLADE_END_SPEED   0.5   /* m/s floor; a stroke ends below max(this, 35% of its peak) */
#define DG_BLADE_MIN_PEAK    1.6   /* m/s: slower strokes are repositioning */
#define DG_BLADE_MIN_HAND    1.0   /* m/s of the hand itself: leaning the head is no swing */
#define DG_BLADE_MIN_SLASH   0.22  /* m across/up/down */
#define DG_BLADE_MIN_STAB    0.20  /* m forward */
#define DG_BLADE_MIN_MS      40
#define DG_BLADE_MAX_MS      700
#define DG_BLADE_COOLDOWN_MS 200
#define DG_BLADE_PENDING_MS  150  /* an attack waits this long for an opposing follow-up */
#define DG_BLADE_LATE_MS     500  /* a held attack still fires this late after a failed follow-up */
#define DG_BLADE_MAX_SPEED   25.0  /* m/s: faster is a tracking jump, not a swing */
/* A slash may reach forward: lateral >= this share of |forward| is still a
   slash (the stab rule is judged first and keeps priority). */
#define DG_BLADE_SLASH_REACH 0.6

#define DG_BLADE_LIVE_LEN        0.90
#define DG_BLADE_LIVE_TIP_START  1.5   /* m/s */
#define DG_BLADE_LIVE_TIP_HOLD   1.0   /* m/s */

enum { DG_BLADE_NONE=0, DG_BLADE_SLASH=1, DG_BLADE_STAB=2 };
/* Pair rule: of two opposing strokes inside PENDING_MS the one with the higher
   peak fires, the other is the wind-up; a tie goes to the later one. */
enum { DG_BLADE_PAIR_NONE=0, DG_BLADE_PAIR_EARLIER_FASTER, DG_BLADE_PAIR_LATER_FASTER };
enum { DG_BLADE_END_STOP=1, DG_BLADE_END_REVERSE=2, DG_BLADE_END_TIMEOUT=3 };
/* Why an evaluated stroke did (not) attack. */
enum { DG_BLADE_R_FIRED=0, DG_BLADE_R_SLOW, DG_BLADE_R_SHORT, DG_BLADE_R_BRIEF,
       DG_BLADE_R_LONG, DG_BLADE_R_BACK, DG_BLADE_R_POINT, DG_BLADE_R_AMBIGUOUS,
       DG_BLADE_R_COOLDOWN, DG_BLADE_R_WINDUP, DG_BLADE_R_COUNT };

typedef struct {
    double disp[3];            /* head-relative travel, start -> end */
    double x, y, z;            /* the same in the stroke frame: right, up, forward */
    double peak;               /* m/s */
    double hand_peak;          /* m/s, hand alone (not head-relative) */
    double pointing;           /* blade forward . stroke forward at the peak */
    uint64_t start_ms, end_ms;
    int end;                   /* DG_BLADE_END_* */
    int attack, reason;
    unsigned char ax, ay;
} DG_BLADE_STROKE;

typedef struct {
    DG_BLADE_SAMPLE last;
    double vel[3], right[3], forward[3], start[3];
    double peak, hand_peak, pointing;
    uint64_t start_ms, fired_ms;
    int have, framed, active;
    DG_BLADE_STROKE pending;
    int have_pending, contested;
    double tip_speed;          /* m/s of the blade tip this step; 0 when unknown */
    double hand_step;          /* m/s of the raw hand step this sample (tracking space); 0 when unknown */
} DG_BLADE_STATE;

typedef struct {
    int attack; unsigned char x,y;
    /* Diagnostics: the stroke judged in this step (report 0 when none), and
       a replaced wind-up, if the judged stroke displaced one. */
    int report, dropped;
    DG_BLADE_STROKE stroke, windup;
    /* When a pair of opposing strokes was decided this step: which rule fired
       which stroke (DG_BLADE_PAIR_*), 0 otherwise. */
    int pair;
    /* Diagnostics: why this step threw the recogniser state away
       (DG_BLADE_RESET_*, 0 = it did not), and whether it was a duplicate. */
    int reset, duplicate;
} DG_BLADE_OUTPUT;
/* Why dg_blade_step reset the recogniser. SELECTION is the bridge's own
   reset (player or hand selection changed); the step never reports it. */
enum { DG_BLADE_RESET_NONE=0, DG_BLADE_RESET_NOT_ALLOWED, DG_BLADE_RESET_INVALID,
       DG_BLADE_RESET_CONTEXT, DG_BLADE_RESET_BACKWARDS, DG_BLADE_RESET_GAP,
       DG_BLADE_RESET_DT, DG_BLADE_RESET_SPEED, DG_BLADE_RESET_HEAD,
       DG_BLADE_RESET_SELECTION, DG_BLADE_RESET_COUNT };
static const char *dg_blade_reset_name(int r) {
    static const char *n[DG_BLADE_RESET_COUNT]={"none","not-allowed","invalid","source/context",
        "backwards","gap>100ms","dt","speed>25","head-step>8cm","selection"};
    return r>=0 && r<DG_BLADE_RESET_COUNT ? n[r] : "?";
}

static double dg_blade_dot(const double *a,const double *b) {
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
}
static void dg_blade_reset(DG_BLADE_STATE *s) { memset(s,0,sizeof *s); }
static int dg_blade_sample_valid(const DG_BLADE_SAMPLE *in) {
    int k; double a,b;
    if(!in->valid || !in->allowed || !in->source || !in->sequence || in->time_ns<=0)return 0;
    for(k=0;k<3;k++) if(!_finite(in->hand[k]) || !_finite(in->head[k]) ||
        !_finite(in->forward[k]) || !_finite(in->head_forward[k]) ||
        fabs(in->hand[k]-in->head[k])>2.5)return 0;
    a=dg_blade_dot(in->forward,in->forward); b=dg_blade_dot(in->head_forward,in->head_forward);
    return fabs(a-1)<.01 && fabs(b-1)<.01;
}
/* Stroke frame from the head's yaw. Looking straight up/down keeps the
   previous frame; without any frame yet the stroke cannot be classified. */
static void dg_blade_frame(DG_BLADE_STATE *s,const DG_BLADE_SAMPLE *in) {
    double n=sqrt(in->head_forward[0]*in->head_forward[0]+in->head_forward[2]*in->head_forward[2]);
    if(n>.2) {
        s->forward[0]=in->head_forward[0]/n;s->forward[1]=0;s->forward[2]=in->head_forward[2]/n;
        s->right[0]=-s->forward[2];s->right[1]=0;s->right[2]=s->forward[0];
        s->framed=1;
    }
}
/* Classify a finished stroke. Sets attack/reason/axes; never fires itself. */
static void dg_blade_judge(DG_BLADE_STROKE *t) {
    double lateral=sqrt(t->x*t->x+t->y*t->y),m;
    t->attack=DG_BLADE_NONE;t->ax=t->ay=128;
    if(t->peak<DG_BLADE_MIN_PEAK || t->hand_peak<DG_BLADE_MIN_HAND){t->reason=DG_BLADE_R_SLOW;return;}
    if(t->end_ms-t->start_ms<DG_BLADE_MIN_MS){t->reason=DG_BLADE_R_BRIEF;return;}
    if(t->end==DG_BLADE_END_TIMEOUT){t->reason=DG_BLADE_R_LONG;return;}
    if(t->z>=DG_BLADE_MIN_STAB && t->z>1.5*lateral) {
        if(t->pointing<=.7){t->reason=DG_BLADE_R_POINT;return;}
        t->attack=DG_BLADE_STAB;t->reason=DG_BLADE_R_FIRED;return;
    }
    if(-t->z>=DG_BLADE_MIN_STAB && -t->z>1.5*lateral){t->reason=DG_BLADE_R_BACK;return;}
    if(lateral>=DG_BLADE_MIN_SLASH && lateral>=DG_BLADE_SLASH_REACH*fabs(t->z)) {
        t->attack=DG_BLADE_SLASH;t->reason=DG_BLADE_R_FIRED;
        /* Saturate the dominant axis past native RStickDir's 96-byte margin.
         * Preserving the ratio retains the native diagonal sectors. */
        m=fmax(fabs(t->x),fabs(t->y));
        t->ax=(unsigned char)(128.5+127*t->x/m);
        t->ay=(unsigned char)(128.5-127*t->y/m);
        return;
    }
    t->reason=(sqrt(lateral*lateral+t->z*t->z)<DG_BLADE_MIN_SLASH)?DG_BLADE_R_SHORT:DG_BLADE_R_AMBIGUOUS;
}
static void dg_blade_fire(DG_BLADE_STATE *s,DG_BLADE_OUTPUT *out,const DG_BLADE_STROKE *t,uint64_t now) {
    out->attack=t->attack;out->x=t->ax;out->y=t->ay;s->fired_ms=now;
}
/* End the running stroke at `rel` and judge it. */
static DG_BLADE_STROKE dg_blade_close(DG_BLADE_STATE *s,const double rel[3],uint64_t now,int end) {
    DG_BLADE_STROKE t;int k;
    memset(&t,0,sizeof t);
    for(k=0;k<3;k++)t.disp[k]=rel[k]-s->start[k];
    t.x=dg_blade_dot(t.disp,s->right);t.y=t.disp[1];t.z=dg_blade_dot(t.disp,s->forward);
    t.peak=s->peak;t.hand_peak=s->hand_peak;t.pointing=s->pointing;t.start_ms=s->start_ms;t.end_ms=now;t.end=end;
    s->active=0;
    dg_blade_judge(&t);
    return t;
}
static void dg_blade_begin(DG_BLADE_STATE *s,const DG_BLADE_SAMPLE *in,const double from[3],double speed,double hand_speed) {
    int k;
    dg_blade_frame(s,in);
    for(k=0;k<3;k++)s->start[k]=from[k];
    s->start_ms=s->last.now_ms;s->peak=speed;s->hand_peak=hand_speed;s->active=s->framed;
    s->pointing=dg_blade_dot(in->forward,s->forward);
}
/* Fire (or refuse inside the cooldown) and report. */
static void dg_blade_release(DG_BLADE_STATE *s,DG_BLADE_OUTPUT *out,DG_BLADE_STROKE t,uint64_t now) {
    if(t.attack && s->fired_ms && now-s->fired_ms<DG_BLADE_COOLDOWN_MS) {
        t.attack=DG_BLADE_NONE;t.reason=DG_BLADE_R_COOLDOWN;t.ax=t.ay=128;
    }
    if(t.attack)dg_blade_fire(s,out,&t,now);
    out->stroke=t;out->report=1;
}
/* Wind-up pair: peaks within DG_BLADE_PAIR_TIE (m/s) are a tie, which the later
   stroke wins; outside it the higher peak fires. 0.1 m/s is ~2 mm per 16 ms
   tick, below what a 60 Hz tick can tell apart for the same arm. */
#define DG_BLADE_PAIR_TIE 0.1
/* A stroke ended. An attack waits DG_BLADE_PENDING_MS: a stroke starting the
   other way inside that time makes it a wind-up and is judged instead. */
static void dg_blade_settle(DG_BLADE_STATE *s,DG_BLADE_OUTPUT *out,DG_BLADE_STROKE t,uint64_t now) {
    if(s->have_pending && s->contested) {
        DG_BLADE_STROKE p=s->pending;
        s->have_pending=s->contested=0;
        if(t.attack && p.peak>t.peak+DG_BLADE_PAIR_TIE) {

            t.attack=DG_BLADE_NONE;t.reason=DG_BLADE_R_WINDUP;t.ax=t.ay=128;
            dg_blade_release(s,out,p,now);out->windup=t;out->dropped=1;out->pair=DG_BLADE_PAIR_EARLIER_FASTER;
        } else if(t.attack) {
            p.attack=DG_BLADE_NONE;p.reason=DG_BLADE_R_WINDUP;p.ax=p.ay=128;
            dg_blade_release(s,out,t,now);out->windup=p;out->dropped=1;out->pair=DG_BLADE_PAIR_LATER_FASTER;
        } else if(now-p.end_ms<=DG_BLADE_LATE_MS) {
            dg_blade_release(s,out,p,now);out->windup=t;out->dropped=1; /* the failed follow-up */
        } else {
            p.attack=DG_BLADE_NONE;p.reason=DG_BLADE_R_WINDUP;p.ax=p.ay=128;
            out->stroke=t;out->report=1;out->windup=p;out->dropped=1;
        }
        return;
    }
    if(s->have_pending) { /* not expected: a starting stroke releases or contests it */
        DG_BLADE_STROKE p=s->pending;s->have_pending=0;dg_blade_release(s,out,p,now);
        if(!t.attack){out->windup=t;out->dropped=1;return;}
    }
    if(t.attack){s->pending=t;s->have_pending=1;s->contested=0;return;}
    out->stroke=t;out->report=1;
}
/* A stroke just began while an attack waits: the other way contests it,
   the same way releases it now. */
static void dg_blade_contest(DG_BLADE_STATE *s,DG_BLADE_OUTPUT *out,uint64_t now) {
    DG_BLADE_STROKE p;
    if(!s->have_pending || s->contested || out->report)return;
    if(s->active && dg_blade_dot(s->vel,s->pending.disp)<0){s->contested=1;return;}
    p=s->pending;s->have_pending=0;dg_blade_release(s,out,p,now);
}
static DG_BLADE_OUTPUT dg_blade_step(DG_BLADE_STATE *s,const DG_BLADE_SAMPLE *in) {
    DG_BLADE_OUTPUT out;
    double rel[3],last_rel[3],v[3],head_delta[3],hand_delta[3],dt,speed,raw,hand_speed,d,n;int k;
    memset(&out,0,sizeof out);out.x=out.y=128;
    if(!dg_blade_sample_valid(in)) {
        DG_BLADE_SAMPLE t=*in;t.allowed=1;
        out.reset=dg_blade_sample_valid(&t)?DG_BLADE_RESET_NOT_ALLOWED:DG_BLADE_RESET_INVALID;
        dg_blade_reset(s);return out;
    }
    if(s->have) {
        if(in->source!=s->last.source || in->context!=s->last.context)out.reset=DG_BLADE_RESET_CONTEXT;
        else if(in->sequence<s->last.sequence || in->now_ms<s->last.now_ms ||
                in->time_ns<s->last.time_ns)out.reset=DG_BLADE_RESET_BACKWARDS;
        else if(in->now_ms-s->last.now_ms>100)out.reset=DG_BLADE_RESET_GAP;
        if(out.reset)dg_blade_reset(s);
    }
    s->tip_speed=0;s->hand_step=0;
    if(!s->have) {s->last=*in;s->have=1;dg_blade_frame(s,in);return out;}
    /* A duplicated publication cannot gain time, speed, or another attack. */
    if(in->sequence==s->last.sequence){out.duplicate=1;return out;}
    dt=(double)(in->time_ns-s->last.time_ns)*1e-9;
    if(dt<.001 || dt>.1) {out.reset=DG_BLADE_RESET_DT;dg_blade_reset(s);return out;}
    {   /* Blade tip speed, tracking space: holding the blade still is zero
           however the head moves; a wrist flick counts. */
        double tip[3];
        for(k=0;k<3;k++)tip[k]=(in->hand[k]+DG_BLADE_LIVE_LEN*in->forward[k])-
                               (s->last.hand[k]+DG_BLADE_LIVE_LEN*s->last.forward[k]);
        s->tip_speed=sqrt(dg_blade_dot(tip,tip))/dt;
        for(k=0;k<3;k++)tip[k]=in->hand[k]-s->last.hand[k];
        s->hand_step=sqrt(dg_blade_dot(tip,tip))/dt;
    }
    for(k=0;k<3;k++) {
        head_delta[k]=in->head[k]-s->last.head[k];
        hand_delta[k]=in->hand[k]-s->last.hand[k];
        rel[k]=in->hand[k]-in->head[k];
        last_rel[k]=s->last.hand[k]-s->last.head[k];
        v[k]=(rel[k]-last_rel[k])/dt;
    }
    raw=sqrt(dg_blade_dot(v,v));
    hand_speed=sqrt(dg_blade_dot(hand_delta,hand_delta))/dt;
    if(raw>DG_BLADE_MAX_SPEED || dg_blade_dot(head_delta,head_delta)>.0064) {
        out.reset=raw>DG_BLADE_MAX_SPEED?DG_BLADE_RESET_SPEED:DG_BLADE_RESET_HEAD;
        dg_blade_reset(s);return out;
    }
    for(k=0;k<3;k++)s->vel[k]=.5*v[k]+.5*s->vel[k];
    speed=sqrt(dg_blade_dot(s->vel,s->vel));
    s->last=*in;
    if(s->active) {
        double disp[3];
        for(k=0;k<3;k++)disp[k]=rel[k]-s->start[k];
        d=sqrt(dg_blade_dot(disp,disp));
        if(speed>s->peak){s->peak=speed;s->pointing=dg_blade_dot(in->forward,s->forward);}
        if(hand_speed>s->hand_peak)s->hand_peak=hand_speed;
        n=d*speed;
        if(d>.05 && speed>=DG_BLADE_START_SPEED && n>0 && dg_blade_dot(s->vel,disp)/n<-.2) {
            /* Reversal: this sample already belongs to the next stroke. */
            DG_BLADE_STROKE t=dg_blade_close(s,last_rel,in->now_ms,DG_BLADE_END_REVERSE);
            dg_blade_settle(s,&out,t,in->now_ms);
            dg_blade_begin(s,in,last_rel,speed,hand_speed);
            dg_blade_contest(s,&out,in->now_ms);
        } else if(speed<fmax(DG_BLADE_END_SPEED,.35*s->peak)) {
            /* A sample that already went back belongs after the turning point. */
            dg_blade_settle(s,&out,dg_blade_close(s,dg_blade_dot(v,disp)<0?last_rel:rel,
                in->now_ms,DG_BLADE_END_STOP),in->now_ms);
        } else if(in->now_ms-s->start_ms>DG_BLADE_MAX_MS) {
            dg_blade_settle(s,&out,dg_blade_close(s,rel,in->now_ms,DG_BLADE_END_TIMEOUT),in->now_ms);
        }
    } else if(speed>=DG_BLADE_START_SPEED) {
        dg_blade_begin(s,in,last_rel,speed,hand_speed);
        dg_blade_contest(s,&out,in->now_ms);
    }
    /* An uncontested attack fires when its wait ends; a contest whose
       follow-up never became a stroke releases it too. */
    if(s->have_pending && !out.report &&
       ((!s->contested && in->now_ms-s->pending.end_ms>=DG_BLADE_PENDING_MS) ||
        (s->contested && !s->active))) {
        DG_BLADE_STROKE p=s->pending;s->have_pending=s->contested=0;
        dg_blade_release(s,&out,p,in->now_ms);
    }
    return out;
}

enum { DG_BLADE_LIVE_NO=0, DG_BLADE_LIVE_MODE, DG_BLADE_LIVE_ADMIT, DG_BLADE_LIVE_GUARD,
       DG_BLADE_LIVE_POSTURE, DG_BLADE_LIVE_STROKE, DG_BLADE_LIVE_SLOW, DG_BLADE_LIVE_JUMP,
       DG_BLADE_LIVE_CONFIRM, DG_BLADE_LIVE_THRESHOLD, DG_BLADE_LIVE_REHIT, DG_BLADE_LIVE_ON };

#define DG_BLADE_LIVE_THR_HOLD 1.5
#define DG_BLADE_REHIT_MS      500
typedef struct { int on, why; uint64_t on_ms; int pend; uint64_t pend_seq;
                 double hit_speed, stroke_peak; uint64_t stroke_id, contact_ms, contact_stroke; } DG_BLADE_LIVE;

static void dg_blade_live_contact(DG_BLADE_LIVE *g,uint64_t now) {
    if(!g)return;
    g->contact_ms=now;g->contact_stroke=g->stroke_id;
}
static int dg_blade_live_step(DG_BLADE_LIVE *g,int mode,const DG_BLADE_STATE *s,
    int admitted,int guard,int posture_bad,uint64_t now) {
    int was=g->on,thr=g->hit_speed>0;
    double start=thr?DG_BLADE_LIVE_THR_HOLD:DG_BLADE_LIVE_TIP_START,hold=thr?DG_BLADE_LIVE_THR_HOLD:DG_BLADE_LIVE_TIP_HOLD;
    g->on=0;
    if(s && s->active) {
        if(s->start_ms!=g->stroke_id){g->stroke_id=s->start_ms;g->stroke_peak=0;}
        if(_finite(s->tip_speed) && s->tip_speed>g->stroke_peak &&
           _finite(s->hand_step) && s->hand_step<=DG_BLADE_MAX_SPEED)g->stroke_peak=s->tip_speed;
    } else {g->stroke_id=0;g->stroke_peak=0;}
    if(mode!=1 && mode!=2)g->why=DG_BLADE_LIVE_MODE;
    else if(!admitted)g->why=DG_BLADE_LIVE_ADMIT;
    else if(guard)g->why=DG_BLADE_LIVE_GUARD;
    else if(posture_bad)g->why=DG_BLADE_LIVE_POSTURE;
    else if(!s || !s->active)g->why=DG_BLADE_LIVE_STROKE;
    else if(!_finite(s->tip_speed) ||
            s->tip_speed<(was?hold:start))g->why=DG_BLADE_LIVE_SLOW;
    else if(!_finite(s->hand_step) || s->hand_step>DG_BLADE_MAX_SPEED)g->why=DG_BLADE_LIVE_JUMP;
    else if(thr && !was && g->stroke_peak<g->hit_speed)g->why=DG_BLADE_LIVE_THRESHOLD;
    else if(thr && !was && g->contact_ms && now>=g->contact_ms && now-g->contact_ms<DG_BLADE_REHIT_MS &&
            g->contact_stroke==g->stroke_id)g->why=DG_BLADE_LIVE_REHIT;
    else if(!was && !(g->pend && s->last.sequence!=g->pend_seq)) {
        g->pend=1;g->pend_seq=s->last.sequence;g->why=DG_BLADE_LIVE_CONFIRM;
        return 0;
    }
    else {g->on=1;g->why=DG_BLADE_LIVE_ON;if(!was)g->on_ms=now;}
    g->pend=0;
    return g->on;
}

#define DG_BLADE_LIVE_STAB_MIN 0.05
static int dg_blade_live_kind(const DG_BLADE_STATE *s) {
    double d[3],x,y,z;int k;
    if(!s || !s->active || !s->framed)return DG_BLADE_SLASH;
    for(k=0;k<3;k++)d[k]=(s->last.hand[k]-s->last.head[k])-s->start[k];
    x=dg_blade_dot(d,s->right);y=d[1];z=dg_blade_dot(d,s->forward);
    if(z>=DG_BLADE_LIVE_STAB_MIN && z>1.5*sqrt(x*x+y*y) && s->pointing>.7)return DG_BLADE_STAB;
    return DG_BLADE_SLASH;
}static const char *dg_blade_live_name(int w) {
    static const char *names[]={"none","mode","not-admitted","guard","posture","no-stroke","slow","jump","confirming","below-threshold","rehit-block","on"};
    return w>=0 && w<=DG_BLADE_LIVE_ON ? names[w] : "?";
}

#define DG_BLADE_BRACE_LEN     0.90
#define DG_BLADE_BRACE_NEAR    0.20
#define DG_BLADE_BRACE_FAR     0.35
#define DG_BLADE_BRACE_HOLD_MS 150
typedef struct { int active, grip_held, pressed; uint64_t valid_ms; double dist, along; } DG_BLADE_BRACE;
static double dg_blade_brace_distance(const DG_BLADE_SAMPLE *in,double *along) {
    double v[3],c[3],t;int k;
    for(k=0;k<3;k++)v[k]=in->left[k]-in->hand[k];
    *along=t=dg_blade_dot(v,in->forward);
    if(t<0)t=0; if(t>DG_BLADE_BRACE_LEN)t=DG_BLADE_BRACE_LEN;
    for(k=0;k<3;k++)c[k]=v[k]-t*in->forward[k];
    return sqrt(dg_blade_dot(c,c));
}
/* The left hand's resting point while braced: the closest point of the blade
 * line, kept between DG_BLADE_SNAP_MIN and DG_BLADE_SNAP_MAX along it. */
#define DG_BLADE_SNAP_MIN 0.30
#define DG_BLADE_SNAP_MAX 0.60
static int dg_blade_brace_snap(const double grip[3],const double forward[3],double left[3]) {
    double v[3],t,n=sqrt(dg_blade_dot(forward,forward));int k;
    if(!(n>.5) || !_finite(n))return 0;
    for(k=0;k<3;k++){if(!_finite(grip[k]) || !_finite(left[k]))return 0;v[k]=left[k]-grip[k];}
    t=dg_blade_dot(v,forward)/n;
    if(t<DG_BLADE_SNAP_MIN)t=DG_BLADE_SNAP_MIN; if(t>DG_BLADE_SNAP_MAX)t=DG_BLADE_SNAP_MAX;
    for(k=0;k<3;k++)left[k]=grip[k]+t*forward[k]/n;
    return 1;
}

typedef struct { double point[3]; int have; } DG_BLADE_SNAP_MEMO;
/* The visible left hand while blocking (raw tracking metres, in/out). `braced`
 * is the guard's state, decided by dg_blade_brace_step on the raw sample; this
 * only moves the drawn hand towards its resting point on the blade through
 * the hand's grip blend and never feeds back into the guard. Returns the
 * weight applied (0 = the controller position, 1 = the snap point). Without a
 * blade line (or a failed snap) the memo's last point is used while the
 * weight is above 0; a non-finite controller position is never moved. */
static double dg_blade_brace_visible(DG_GRIP_BLEND *b,DG_BLADE_SNAP_MEMO *memo,int braced,long long now_ns,
    const DG_GRIP_TIMES *t,int have_blade,const double grip[3],const double forward[3],double left[3]) {
    double snap[3],w=dg_grip_blend_step(b,braced,now_ns,t);int k;
    if(!(w>0)){if(memo)memo->have=0;return 0;}
    for(k=0;k<3;k++)if(!_finite(left[k]))return 0;
    memcpy(snap,left,sizeof snap);
    if(have_blade && dg_blade_brace_snap(grip,forward,snap)) {
        if(memo){memcpy(memo->point,snap,sizeof snap);memo->have=1;}
    } else if(memo && memo->have) {
        memcpy(snap,memo->point,sizeof snap);
    } else return 0;
    dg_grip_mix3(left,snap,w,left);
    return w;
}
static int dg_blade_brace_step(DG_BLADE_BRACE *g,const DG_BLADE_SAMPLE *in) {
    int k,ok=dg_blade_sample_valid(in) && in->left_valid;
    for(k=0;k<3 && ok;k++)if(!_finite(in->left[k]) || fabs(in->left[k]-in->head[k])>2.5)ok=0;
    g->pressed=0;
    if(!ok) {
        if(g->active && in->now_ms>=g->valid_ms && in->now_ms-g->valid_ms<=DG_BLADE_BRACE_HOLD_MS)return 1;
        g->active=0;return 0;
    }
    g->valid_ms=in->now_ms;
    g->dist=dg_blade_brace_distance(in,&g->along);
    if(!in->left_grip){g->active=0;g->grip_held=0;return 0;}
    if(!g->grip_held) {                    /* the press */
        g->grip_held=1;g->pressed=1;
        g->active=g->dist<=DG_BLADE_BRACE_NEAR;
    } else if(g->active && g->dist>DG_BLADE_BRACE_FAR) g->active=0;
    return g->active;
}
static const char *dg_blade_reason_name(int r) {
    static const char *names[DG_BLADE_R_COUNT]={"fired","slow","short","brief","long",
        "pull-back","not-pointing","ambiguous","cooldown","wind-up"};
    return r>=0 && r<DG_BLADE_R_COUNT ? names[r] : "?";
}
#endif
