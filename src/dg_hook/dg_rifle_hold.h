#ifndef DG_RIFLE_HOLD_H
#define DG_RIFLE_HOLD_H
#include <stdint.h>
#include <string.h>

#define DG_RIFLE_HOLD_WITNESS 1
#define DG_RIFLE_HOLD_WS_DRAW 2
static int dg_rifle_hold_release(volatile long *status,volatile long *press,volatile long *release,
                                 volatile unsigned char *pressure,long mask,int latched) {
    int held;
    if(!mask)return 0;
    held=(*status&mask)!=0 || latched;
    *press&=~mask;
    *status&=~mask;
    *pressure=0;
    if(held)*release|=mask;
    return held;
}

#define DG_RIFLE_HOLD_WINDOW_MS 1000
#define DG_RIFLE_HOLD_LINES 600
#define DG_RIFLE_HOLD_FLIP_LINES 24      /* per owed magazine: one line per HOLD flip */
#define DG_RIFLE_HOLD_RELEASE_LINES 3    /* per owed magazine: the first release ticks and the layer's answer */

typedef struct {
    int called,released,weapon_state,button_state;
    unsigned status_in,press_in,release_in,status_out,release_out;
} DG_RIFLE_HOLD_SEAM;
typedef struct {
    uint64_t start_ms;
    int have,last;
    unsigned ticks,flips,latched,lines,total_flips;
    unsigned draw_ticks,flip_lines,release_lines,total_released;
    DG_RIFLE_HOLD_SEAM seam;
} DG_RIFLE_HOLD_METER;

static const char *dg_rifle_hold_source(const DG_RIFLE_HOLD_SEAM *s,unsigned mask) {
    if(!s->called)return "reload_pad not called (no block or no write gate)";
    if(s->status_out&mask)return "our pad write (status left set)";
    if(s->status_in&mask)return "the player's own button (GV copy at the seam)";
    if(s->weapon_state==DG_RIFLE_HOLD_WS_DRAW && !(s->release_out&mask))return "Bluepoint latch (draw state, no release from us)";
    if(s->release_out&mask)return "none: we released, the layer must drop draw state";
    return "native only (no weapon bit anywhere at the seam)";
}
static void dg_rifle_hold_meter_reset(DG_RIFLE_HOLD_METER *m){memset(m,0,sizeof *m);}
/* One game tick. Returns 1 when a window closed (the caller logs it, then the window restarts). */
static int dg_rifle_hold_meter_step(DG_RIFLE_HOLD_METER *m,int hold,uint64_t now) {
    hold=hold!=0;
    if(!m->start_ms || now<m->start_ms){m->start_ms=now;m->ticks=m->flips=m->latched=m->draw_ticks=0;}
    if(m->have && hold!=m->last){m->flips++;m->total_flips++;}
    m->have=1;m->last=hold;m->ticks++;
    return now-m->start_ms>=DG_RIFLE_HOLD_WINDOW_MS;
}
static void dg_rifle_hold_meter_window(DG_RIFLE_HOLD_METER *m,uint64_t now){m->start_ms=now;m->ticks=m->flips=m->latched=m->draw_ticks=0;}
#endif
