#ifndef DG_BLADE_H
#define DG_BLADE_H
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <float.h>

/* Metres, LOCAL-space samples. No native pointers or writes in recognition.
 * Grip is an explicit clutch. A fresh release and 120 ms of rest are required
 * after equip, cancellation or tracking loss; a swing must not survive them. */
typedef struct {
    uint64_t source, sequence, context, now_ms;
    int64_t time_ns;
    int valid, allowed, grip;
    double hand[3], head[3], forward[3], head_forward[3];
} DG_BLADE_SAMPLE;
typedef struct {
    DG_BLADE_SAMPLE last;
    double origin[3], raw_origin[3], right[3], forward[3];
    uint64_t rest_ms, start_ms, fired_ms;
    int have, released, armed, moving, fast;
} DG_BLADE_STATE;
typedef struct { int attack; unsigned char x,y; } DG_BLADE_OUTPUT;
enum { DG_BLADE_NONE=0, DG_BLADE_SLASH=1, DG_BLADE_STAB=2 };

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
static void dg_blade_origin(DG_BLADE_STATE *s,const DG_BLADE_SAMPLE *in) {
    double n=sqrt(in->head_forward[0]*in->head_forward[0]+in->head_forward[2]*in->head_forward[2]);
    int k;
    for(k=0;k<3;k++){s->origin[k]=in->hand[k]-in->head[k];s->raw_origin[k]=in->hand[k];}
    /* Looking straight up/down cannot define a new swing plane. */
    if(n>.2) {
        s->forward[0]=in->head_forward[0]/n;s->forward[1]=0;s->forward[2]=in->head_forward[2]/n;
        s->right[0]=-s->forward[2];s->right[1]=0;s->right[2]=s->forward[0];
    }
}
static DG_BLADE_OUTPUT dg_blade_step(DG_BLADE_STATE *s,const DG_BLADE_SAMPLE *in) {
    DG_BLADE_OUTPUT out={0,128,128};
    double delta[3],travel[3],head_delta[3],raw_delta[3],raw_travel[3],dt,speed,hand_speed,x,y,z,m; int k;
    if(!dg_blade_sample_valid(in)) {dg_blade_reset(s);return out;}
    if(s->have && (in->source!=s->last.source || in->context!=s->last.context ||
        in->sequence<s->last.sequence || in->now_ms<s->last.now_ms ||
        in->time_ns<s->last.time_ns || in->now_ms-s->last.now_ms>100))dg_blade_reset(s);
    if(!s->have) {
        s->last=*in;s->have=1;s->released=!in->grip;
        s->rest_ms=in->now_ms;dg_blade_origin(s,in);return out;
    }
    /* A duplicated publication cannot gain time, speed, or another attack. */
    if(in->sequence==s->last.sequence)return out;
    dt=(double)(in->time_ns-s->last.time_ns)*1e-9;
    if(dt<.001 || dt>.1) {dg_blade_reset(s);return out;}
    for(k=0;k<3;k++) {
        head_delta[k]=in->head[k]-s->last.head[k];
        raw_delta[k]=in->hand[k]-s->last.hand[k];
        raw_travel[k]=in->hand[k]-s->raw_origin[k];
        delta[k]=(in->hand[k]-s->last.hand[k])-head_delta[k];
        travel[k]=in->hand[k]-in->head[k]-s->origin[k];
    }
    speed=sqrt(dg_blade_dot(delta,delta))/dt;
    hand_speed=sqrt(dg_blade_dot(raw_delta,raw_delta))/dt;
    if(speed>8 || dg_blade_dot(head_delta,head_delta)>.0064) {dg_blade_reset(s);return out;}
    s->last=*in;
    if(!in->grip) {s->released=1;s->armed=0;s->moving=0;s->fast=0;}
    if(speed<.25) {
        if(!s->rest_ms)s->rest_ms=in->now_ms;
        if(in->now_ms-s->rest_ms>=120 && (!s->fired_ms || in->now_ms-s->fired_ms>=300)) {
            s->armed=s->released && dg_blade_dot(s->forward,s->forward)>.9;
            s->moving=0;s->fast=0;dg_blade_origin(s,in);
        }
    } else s->rest_ms=0;
    if(!in->grip || !s->armed)return out;
    if(speed>.4 && !s->moving) {s->moving=1;s->start_ms=in->now_ms;}
    if(speed>=.9 && hand_speed>=.7)s->fast++;else s->fast=0;
    if(s->moving && in->now_ms-s->start_ms>400) {s->armed=0;s->moving=0;return out;}
    if(!s->moving || s->fast<2 || in->now_ms-s->start_ms<32 ||
       dg_blade_dot(raw_travel,raw_travel)<.0196)return out;
    x=dg_blade_dot(travel,s->right);y=travel[1];z=dg_blade_dot(travel,s->forward);
    if(z>=.18 && z>1.5*sqrt(x*x+y*y) && dg_blade_dot(in->forward,s->forward)>.7) {
        out.attack=DG_BLADE_STAB;
    } else if(sqrt(x*x+y*y)>=.18 && sqrt(x*x+y*y)>fabs(z)) {
        out.attack=DG_BLADE_SLASH;
        /* Saturate the dominant axis past native RStickDir's 96-byte margin.
         * Preserving the ratio retains the native diagonal sectors. */
        m=fmax(fabs(x),fabs(y));
        out.x=(unsigned char)(128.5+127*x/m);
        out.y=(unsigned char)(128.5-127*y/m);
    }
    if(out.attack){s->armed=0;s->moving=0;s->fast=0;s->fired_ms=in->now_ms;s->rest_ms=0;}
    return out;
}
#endif
