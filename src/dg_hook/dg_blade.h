#ifndef DG_BLADE_H
#define DG_BLADE_H
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <float.h>

/* Metres, LOCAL-space samples. No native pointers or writes in recognition.
 * No button: while the blade is held, a fast stroke of the hand relative to
 * the head is an attack. A stroke is judged when it ENDS (it slows down,
 * reverses or runs too long), so its direction is the whole stroke, not the
 * first centimetres. An attack then waits DG_BLADE_PENDING_MS: if a stroke
 * the other way starts inside that time, the first was a wind-up (lifting
 * the sword before a chop) and the follow-up is judged instead. grip is sampled but no longer consulted. */
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
#define DG_BLADE_COOLDOWN_MS 350
#define DG_BLADE_PENDING_MS  150  /* an attack waits this long for an opposing follow-up */
#define DG_BLADE_LATE_MS     500  /* a held attack still fires this late after a failed follow-up */
#define DG_BLADE_MAX_SPEED   25.0  /* m/s: faster is a tracking jump, not a swing */

enum { DG_BLADE_NONE=0, DG_BLADE_SLASH=1, DG_BLADE_STAB=2 };
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
} DG_BLADE_STATE;

typedef struct {
    int attack; unsigned char x,y;
    /* Diagnostics: the stroke judged in this step (report 0 when none), and
       a replaced wind-up, if the judged stroke displaced one. */
    int report, dropped;
    DG_BLADE_STROKE stroke, windup;
} DG_BLADE_OUTPUT;

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
    if(lateral>=DG_BLADE_MIN_SLASH && lateral>fabs(t->z)) {
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
/* A stroke ended. An attack waits DG_BLADE_PENDING_MS: a stroke starting the
   other way inside that time makes it a wind-up and is judged instead. */
static void dg_blade_settle(DG_BLADE_STATE *s,DG_BLADE_OUTPUT *out,DG_BLADE_STROKE t,uint64_t now) {
    if(s->have_pending && s->contested) {
        DG_BLADE_STROKE p=s->pending;
        s->have_pending=s->contested=0;
        if(t.attack) {
            p.attack=DG_BLADE_NONE;p.reason=DG_BLADE_R_WINDUP;p.ax=p.ay=128;
            dg_blade_release(s,out,t,now);out->windup=p;out->dropped=1;
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
    if(!dg_blade_sample_valid(in)) {dg_blade_reset(s);return out;}
    if(s->have && (in->source!=s->last.source || in->context!=s->last.context ||
        in->sequence<s->last.sequence || in->now_ms<s->last.now_ms ||
        in->time_ns<s->last.time_ns || in->now_ms-s->last.now_ms>100))dg_blade_reset(s);
    if(!s->have) {s->last=*in;s->have=1;dg_blade_frame(s,in);return out;}
    /* A duplicated publication cannot gain time, speed, or another attack. */
    if(in->sequence==s->last.sequence)return out;
    dt=(double)(in->time_ns-s->last.time_ns)*1e-9;
    if(dt<.001 || dt>.1) {dg_blade_reset(s);return out;}
    for(k=0;k<3;k++) {
        head_delta[k]=in->head[k]-s->last.head[k];
        hand_delta[k]=in->hand[k]-s->last.hand[k];
        rel[k]=in->hand[k]-in->head[k];
        last_rel[k]=s->last.hand[k]-s->last.head[k];
        v[k]=(rel[k]-last_rel[k])/dt;
    }
    raw=sqrt(dg_blade_dot(v,v));
    hand_speed=sqrt(dg_blade_dot(hand_delta,hand_delta))/dt;
    if(raw>DG_BLADE_MAX_SPEED || dg_blade_dot(head_delta,head_delta)>.0064) {dg_blade_reset(s);return out;}
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
/* Block: the left hand on the blade with the left grip held (tracking-space
 * metres). The blade runs from the right grip along its pointing axis for
 * DG_BLADE_BRACE_LEN. A grip PRESS with the left hand within
 * DG_BLADE_BRACE_NEAR of it starts the block; releasing the grip, or moving the
 * hand more than DG_BLADE_BRACE_FAR off the blade, ends it. A press made away
 * from the blade never becomes a block while it is held. Tracking loss keeps
 * it DG_BLADE_BRACE_HOLD_MS (the native guard animation may take the hand). */
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





#endif
