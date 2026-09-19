#ifndef DG_HEALTH_H
#define DG_HEALTH_H
/* Read-only LIFE snapshot and portable display policy. No game/XR calls. */
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "dg_radial_overlay.h"
typedef struct dg_health_sample {
    uint64_t player, inventory, stamp;
    int area, value, maximum, bleeding, valid;
} dg_health_sample;
typedef struct dg_health_state {
    dg_health_sample previous;
    uint64_t show_until;
} dg_health_state;
typedef struct dg_health_view {
    dg_health_sample sample;
    uint64_t show_until;
} dg_health_view;
int dg_bridge_health_snapshot(dg_health_view *out);

static int dg_health_valid(const dg_health_sample *s) {
    return s && s->valid && s->player && s->inventory && s->maximum>0 &&
        s->maximum<=32767 && s->value>=0 && s->value<=s->maximum;
}
static dg_health_view dg_health_update(dg_health_state *s, dg_health_sample v) {
    dg_health_view out;
    memset(&out,0,sizeof out);
    if (!dg_health_valid(&v)) { memset(s,0,sizeof *s); return out; }
    if (!dg_health_valid(&s->previous) || s->previous.player!=v.player ||
        s->previous.inventory!=v.inventory || s->previous.area!=v.area ||
        s->previous.maximum!=v.maximum || v.stamp<s->previous.stamp ||
        v.stamp-s->previous.stamp>100) s->show_until=0;
    else if (v.value!=s->previous.value) s->show_until=v.stamp+3000;
    s->previous=v; out.sample=v; out.show_until=s->show_until; return out;
}
/* 0 hidden, 1 wrist, 2 view. A warning outranks the wrist, never duplicates. */
static int dg_health_mode(const dg_health_view *v,uint64_t now,int wrist,int items) {
    if (!v || !dg_health_valid(&v->sample) || now<v->sample.stamp ||
        now-v->sample.stamp>100) return 0;
    if (items || now<v->show_until || v->sample.bleeding ||
        v->sample.value*5<=v->sample.maximum) return 2;
    return wrist?1:0;
}
/* Controller grip located in VIEW space. +Y is the top of the grip; revealing
 * the display requires raising it and turning that surface towards the face.
 * Hysteresis prevents flicker near the edge. No inferred pose on tracking loss. */
static int dg_health_wrist(const double p[3],const double q[4],int held,double out[3]) {
    double x=q[0],y=q[1],z=q[2],w=q[3],n=x*x+y*y+z*z+w*w;
    double up[3],back[3],d,facing;
    int k;
    if (!isfinite(n) || fabs(n-1)>0.02) return 0;
    up[0]=2*(x*y-z*w);up[1]=1-2*(x*x+z*z);up[2]=2*(y*z+x*w);
    back[0]=2*(x*z+y*w);back[1]=2*(y*z-x*w);back[2]=1-2*(x*x+y*y);
    for(k=0;k<3;k++) { if(!isfinite(p[k])) return 0; out[k]=p[k]+.045*up[k]+.09*back[k]; }
    d=sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2]);
    if(d<.22 || d>1.0 || p[2]>-.16 || fabs(p[0])>.65 || p[1]>(held?.30:.25) ||
       p[1]<(held?-.55:-.45)) return 0;
    facing=-(up[0]*p[0]+up[1]*p[1]+up[2]*p[2])/d;
    return facing>(held?.30:.45);
}
#define DG_HEALTH_SIZE 256u
#define DG_HEALTH_PIXELS (256u*256u)
/* Top 256x64 is submitted; remaining rows transparent. RGBA, straight alpha. */
static void dg_health_raster(const dg_health_sample *s,uint32_t *pixels) {
    unsigned x,y,c,bit,row,fill;
    unsigned char *p=(unsigned char *)pixels;
    static const unsigned char glyph[4][7]={
        {16,16,16,16,16,16,31},{31,4,4,4,4,4,31},
        {31,16,16,30,16,16,16},{31,16,16,30,16,16,31}};
    unsigned char r=165,g=215,b=164;
    memset(pixels,0,DG_HEALTH_PIXELS*4u);
    if(!dg_health_valid(s)) return;
    if(s->value*5<=s->maximum || s->bleeding) {r=235;g=95;b=75;}
    else if(s->value*2<=s->maximum) {r=222;g=174;b=86;}
    for(y=0;y<64;y++) for(x=0;x<256;x++) {
        unsigned i=(y*256+x)*4;p[i]=18;p[i+1]=26;p[i+2]=25;p[i+3]=205;
        if(x==0||x==255||y==0||y==63) {p[i]=95;p[i+1]=116;p[i+2]=107;p[i+3]=230;}
    }
    for(c=0;c<4;c++) for(row=0;row<7;row++) for(bit=0;bit<5;bit++)
        if(glyph[c][row]&(16>>bit)) for(y=0;y<2;y++) for(x=0;x<2;x++) {
            unsigned i=((9+row*2+y)*256+12+c*14+bit*2+x)*4;
            p[i]=p[i+1]=p[i+2]=225;p[i+3]=255;
        }
    fill=(unsigned)(s->value*230/s->maximum);
    for(y=32;y<52;y++) for(x=12;x<244;x++) {
        unsigned i=(y*256+x)*4;
        if(y==32||y==51||x==12||x==243) {p[i]=110;p[i+1]=132;p[i+2]=120;p[i+3]=255;}
        else if(x-13<fill) {p[i]=r;p[i+1]=g;p[i+2]=b;p[i+3]=255;}
    }
}
/* Dedicated GPU lifecycle, using the same backend contract as radial.
 * A timed-out acquisition remains owned; cancellation drains it without upload. */
static int dg_health_upload(dg_radial_overlay *s,const dg_radial_overlay_ops *o,
                             int want,const uint32_t *pixels) {
    int r,uploaded=0;
    if(s->fault) return 0;
    if(want && !s->handle) {
        if(o->create(o->user,256,256,&s->handle,&s->images)!=DG_RADIAL_IO_OK ||
           !s->handle || !s->images) {s->fault=1;return 0;}
    }
    if(want && !s->acquired) {
        if(o->acquire(o->user,s->handle,&s->image)!=DG_RADIAL_IO_OK) {s->fault=1;return 0;}
        s->acquired=1;
        if(s->image>=s->images) {s->fault=1;return 0;}
    }
    if(!s->acquired) return 0;
    r=o->wait(o->user,s->handle);
    if(r==DG_RADIAL_IO_TIMEOUT) return 0;
    if(r!=DG_RADIAL_IO_OK) {s->fault=1;return 0;}
    if(want) uploaded=o->upload(o->user,s->handle,s->image,pixels,DG_HEALTH_PIXELS*4u)==DG_RADIAL_IO_OK;
    if(o->release(o->user,s->handle)!=DG_RADIAL_IO_OK) {s->fault=1;return 0;}
    s->acquired=0;
    if(want&&!uploaded) s->fault=1;
    return uploaded;
}
#endif
