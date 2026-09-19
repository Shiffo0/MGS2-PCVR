#ifndef DG_PISTOL_RELOAD_H
#define DG_PISTOL_RELOAD_H
#include <stdint.h>
#include <string.h>
#include <math.h>
/* Metres and wall-clock milliseconds; duplicated render/eye invocations do
 * not advance the gesture. First leave the socket, then return deliberately. */
#define DG_RELOAD_ENTER_M .10
#define DG_RELOAD_LEAVE_M .18
#define DG_RELOAD_DWELL_MS 100
typedef struct {
    uint64_t source, sequence, sample_ms, near_ms, permit_ms;
    int outside, permit;
} DG_RELOAD_GESTURE;
static void dg_reload_gesture_reset(DG_RELOAD_GESTURE *s) {memset(s,0,sizeof *s);}
static int dg_reload_gesture(DG_RELOAD_GESTURE *s,uint64_t source,uint64_t seq,
                            uint64_t now,int valid,double distance) {
    if(!valid || !source || !seq || !isfinite(distance) || distance<0 ||
       (s->source && source!=s->source) || (s->sequence && seq<s->sequence) ||
       now<s->sample_ms || (s->sequence && now-s->sample_ms>100)) {
        dg_reload_gesture_reset(s);return 0;
    }
    if(s->permit && now-s->permit_ms>500)dg_reload_gesture_reset(s);
    if(seq==s->sequence)return s->permit;
    s->source=source;s->sequence=seq;s->sample_ms=now;
    if(distance>=DG_RELOAD_LEAVE_M){s->outside=1;s->near_ms=0;}
    else if(distance>DG_RELOAD_ENTER_M)s->near_ms=0;
    else if(s->outside){
        if(!s->near_ms)s->near_ms=now;
        else if(now-s->near_ms>=DG_RELOAD_DWELL_MS){s->permit=1;s->permit_ms=now;}
    }
    return s->permit;
}
#endif
