/* Manual M9 mechanics. Pure, deterministic, metres and monotonic milliseconds.
 * Native ammunition/shot creation remains outside this module. */
#ifndef DG_M9_SLIDE_H
#define DG_M9_SLIDE_H
#include <stdint.h>
#include <string.h>
#include <math.h>
#ifdef _MSC_VER
#include <float.h>
#define M9_FINITE(x) _finite(x)
#else
#define M9_FINITE(x) isfinite(x)
#endif
#define DG_M9_GRAB_RADIUS_M 0.15

enum { M9_NEEDS_RACK, M9_GRABBED, M9_FULL_REAR, M9_RETURN_BAD,
       M9_RETURN_GOOD, M9_READY };
enum { M9_GRAB=1, M9_END=2, M9_CLICK=4, M9_COMPLETE=8, M9_CANCEL=16 };
typedef struct {
    uint64_t epoch, source, sequence, now_ms, shot_sequence;
    int valid, allowed, grip, trigger, physical_trigger, contact_valid;
    double left[3], anchor[3]; /* in weapon space; +Y is rearward */
    double stroke, return_seconds;
} DG_M9_INPUT;
typedef struct {
    uint64_t epoch, source, sequence, now_ms, shot_sequence, fresh_ms;
    uint64_t grab_deadline;
    int state, initialized, grip_prev, trigger_prev, neutral, full_samples;
    double travel, grab[3], previous[3];
} DG_M9_STATE;
typedef struct {
    int state, block_fire, attached, cancel_reason; /* 1=input/context, 2=rail */
    unsigned events;
    double progress, contact[3];
} DG_M9_OUTPUT;
static void dg_m9_reset(DG_M9_STATE *s) { memset(s,0,sizeof *s); }
static int dg_m9_vec_ok(const double v[3]) {
    int k; for(k=0;k<3;k++) if(!M9_FINITE(v[k]) || fabs(v[k])>10) return 0;
    return 1;
}
/* Raw controller positions -> weapon space using the same C basis as
 * dg_aim_target: native -Y points along XR aim -Z, native +Z along XR +Y.
 * right_position is the driven GRIP origin, right_q is the AIM orientation. */
static int dg_m9_relative(const double left[3],const double right[3],
                           const double right_q[4],double local[3]) {
    double q[3],v[3],t[3],r[3],n=0; int k;
    if(!dg_m9_vec_ok(left)||!dg_m9_vec_ok(right))return 0;
    for(k=0;k<4;k++){if(!M9_FINITE(right_q[k]))return 0;n+=right_q[k]*right_q[k];}
    if(fabs(n-1)>0.001)return 0;
    for(k=0;k<3;k++){q[k]=-right_q[k];v[k]=left[k]-right[k];}
    t[0]=2*(q[1]*v[2]-q[2]*v[1]);
    t[1]=2*(q[2]*v[0]-q[0]*v[2]);
    t[2]=2*(q[0]*v[1]-q[1]*v[0]);
    r[0]=v[0]+right_q[3]*t[0]+q[1]*t[2]-q[2]*t[1];
    r[1]=v[1]+right_q[3]*t[1]+q[2]*t[0]-q[0]*t[2];
    r[2]=v[2]+right_q[3]*t[2]+q[0]*t[1]-q[1]*t[0];
    local[0]=-r[0];local[1]=r[2];local[2]=r[1];return dg_m9_vec_ok(local);
}
static DG_M9_OUTPUT dg_m9_step(DG_M9_STATE *s,const DG_M9_INPUT *in) {
    DG_M9_OUTPUT o; double dt=0,dx,dy,dz,d,step; int fresh,trigger,k;
    memset(&o,0,sizeof o);o.block_fire=1;
    if(!s||!in)return o;
    if(!s->initialized || s->epoch!=in->epoch) {
        dg_m9_reset(s);s->initialized=1;s->epoch=in->epoch;s->source=in->source;
        s->now_ms=s->fresh_ms=in->now_ms;s->shot_sequence=in->shot_sequence;
        s->grip_prev=1;s->trigger_prev=1; /* first sample must be neutral */
    }
    if(in->shot_sequence!=s->shot_sequence) {
        s->shot_sequence=in->shot_sequence;s->state=M9_NEEDS_RACK;
        s->travel=0;s->neutral=0;s->full_samples=0;s->grip_prev=1;
        s->grab_deadline=0;
    }
    fresh=in->sequence>s->sequence;
    if(in->now_ms>=s->now_ms)dt=(double)(in->now_ms-s->now_ms)/1000.0;
    if(!in->valid || !in->allowed || !in->epoch || !in->source ||
       in->source!=s->source || in->sequence<s->sequence ||
       in->now_ms<s->now_ms || dt>0.1 ||
       (!fresh && in->now_ms>=s->fresh_ms && in->now_ms-s->fresh_ms>100) ||
       !dg_m9_vec_ok(in->left)||!dg_m9_vec_ok(in->anchor)||
       !M9_FINITE(in->stroke)||in->stroke<0.005||in->stroke>0.15||
       !M9_FINITE(in->return_seconds)||in->return_seconds<0.02||in->return_seconds>2) {
        if(s->state!=M9_READY) {s->state=M9_NEEDS_RACK;s->travel=0;}
        s->neutral=0;s->grip_prev=1;s->trigger_prev=1;s->full_samples=0;
        s->grab_deadline=0;
        s->source=in->source;s->sequence=in->sequence;s->now_ms=in->now_ms;
        o.events=M9_CANCEL;o.cancel_reason=1;goto output;
    }
    /* Repeated frames cannot release a grip, confirm an endpoint or create a
     * feedback event. Return motion uses elapsed time, not sample count. */
    if(fresh) {
        s->fresh_ms=in->now_ms;
        trigger=in->trigger||in->physical_trigger;
        if(s->state==M9_READY && !trigger)s->neutral=1;
        if(trigger && !s->trigger_prev && (s->state!=M9_READY||!s->neutral))o.events|=M9_CLICK;
        /* A slightly early squeeze remains intentional for 200 ms. This is
         * not automatic reacquisition while a grip is held indefinitely. */
        if(!in->grip)s->grab_deadline=0;
        if(s->state==M9_NEEDS_RACK && in->grip && !s->grip_prev)
            s->grab_deadline=in->now_ms+200;
        if(s->state==M9_NEEDS_RACK && in->contact_valid && in->grip && s->grab_deadline &&
           in->now_ms<=s->grab_deadline) {
            dx=in->left[0]-in->anchor[0];dy=in->left[1]-in->anchor[1];dz=in->left[2]-in->anchor[2];
            if(dx*dx+dy*dy+dz*dz<=DG_M9_GRAB_RADIUS_M*DG_M9_GRAB_RADIUS_M) {
                s->state=M9_GRABBED;s->travel=0;s->full_samples=0;
                s->grab_deadline=0;
                memcpy(s->grab,in->left,sizeof s->grab);memcpy(s->previous,in->left,sizeof s->previous);
                o.events|=M9_GRAB;
            }
        }
        if(s->state==M9_GRABBED||s->state==M9_FULL_REAR) {
            dx=in->left[0]-s->grab[0];dy=in->left[1]-s->grab[1];dz=in->left[2]-s->grab[2];
            d=0;for(k=0;k<3;k++){step=in->left[k]-s->previous[k];d+=step*step;}
            /* A confirmed endpoint is latched. Letting go, or moving away
             * from the constrained rail, releases the spring successfully.
             * Invalid tracking/context was already rejected above. */
            if(s->state==M9_FULL_REAR && (!in->grip ||
               dx*dx+dz*dz>0.06*0.06 || dy < -0.12 ||
               dy > in->stroke+0.12 || d>0.08*0.08)) {
                s->state=M9_RETURN_GOOD;
            } else if(dx*dx+dz*dz>0.06*0.06 || dy < -0.12 || dy > in->stroke+0.12 || d>0.08*0.08) {
                s->state=M9_RETURN_BAD;s->full_samples=0;o.events|=M9_CANCEL;o.cancel_reason=2;
            } else if(!in->grip) {
                s->state=s->state==M9_FULL_REAR?M9_RETURN_GOOD:M9_RETURN_BAD;
            } else {
                s->travel=dy<0?0:dy>in->stroke?in->stroke:dy;
                if(s->travel>=in->stroke-0.002) {
                    if(s->full_samples<2)s->full_samples++;
                    if(s->full_samples==2 && s->state!=M9_FULL_REAR){s->state=M9_FULL_REAR;o.events|=M9_END;}
                } else if(s->state!=M9_FULL_REAR)s->full_samples=0;
                memcpy(s->previous,in->left,sizeof s->previous);
            }
        }
        s->grip_prev=in->grip;s->trigger_prev=trigger;s->sequence=in->sequence;
    }
    if(s->state==M9_RETURN_BAD||s->state==M9_RETURN_GOOD) {
        s->travel-=in->stroke*dt/in->return_seconds;
        if(s->travel<=0) {
            s->travel=0;
            if(s->state==M9_RETURN_GOOD){s->state=M9_READY;s->neutral=0;o.events|=M9_COMPLETE;}
            else s->state=M9_NEEDS_RACK;
        }
    }
    s->now_ms=in->now_ms;
output:
    o.state=s->state;o.block_fire=s->state!=M9_READY||!s->neutral||!in->valid||!in->allowed;
    o.attached=s->state==M9_GRABBED||s->state==M9_FULL_REAR;
    o.progress=in->stroke>0?s->travel/in->stroke:0;
    /* Preserve where the wrist actually grabbed, instead of teleporting it
     * to the centre of the acceptance volume (up to 10 cm away). */
    for(k=0;k<3;k++)o.contact[k]=o.attached?s->grab[k]:in->anchor[k];
    o.contact[1]+=s->travel;
    return o;
}
#endif
