#ifndef DG_PISTOL_RELOAD_H
#define DG_PISTOL_RELOAD_H
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
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

static const char *dg_reload_probe_verdict(int valid,int outside,double dist_m,uint64_t held_ms,int permit) {
    if(!valid)return "hand sample not valid (no VR hand, menu/other button, or not allowed)";
    if(permit)return "contact";
    if(!isfinite(dist_m))return "no magwell/hand distance yet (weapon model not seen this frame)";
    if(!outside)return "not armed yet: first take the left hand more than 180 mm away from the magwell";
    if(dist_m>DG_RELOAD_ENTER_M)return "armed, too far: bring the left hand within 100 mm of the magwell";
    if(held_ms<DG_RELOAD_DWELL_MS)return "armed and close: hold still";
    return "armed and close long enough: contact on the next sample";
}
/* trk_ok: magwell_trk/left_trk are tracking metres (the same space as the left controller pose). */
static int dg_reload_probe_format(char *out,size_t n,int weapon,int ejected,int trk_ok,const char *trk_why,
                                  const double magwell_trk[3],const double left_trk[3],double dist_m,int outside,
                                  uint64_t held_ms,int valid,int permit) {
    char where[220];
    if(trk_ok)
        snprintf(where,sizeof where,"magwell at tracking (%.3f %.3f %.3f) m, left grip now (%.3f %.3f %.3f) m = %.0f mm apart",
                 magwell_trk[0],magwell_trk[1],magwell_trk[2],left_trk[0],left_trk[1],left_trk[2],
                 1000.0*sqrt((magwell_trk[0]-left_trk[0])*(magwell_trk[0]-left_trk[0])+(magwell_trk[1]-left_trk[1])*(magwell_trk[1]-left_trk[1])+
                             (magwell_trk[2]-left_trk[2])*(magwell_trk[2]-left_trk[2])));
    else snprintf(where,sizeof where,"magwell in tracking space unknown (%s)",trk_why?trk_why:"?");
    return snprintf(out,n,"  reload probe: weapon %d %s | %s | model distance %s%.0f mm | armed (was > %.0f mm) %d, need < %.0f mm for %d ms, held %llu ms | %s\r\n",
                    weapon,ejected?"ejected":"empty",where,isfinite(dist_m)?"":"unknown ",isfinite(dist_m)?dist_m*1000.0:0.0,
                    DG_RELOAD_LEAVE_M*1000.0,outside?1:0,DG_RELOAD_ENTER_M*1000.0,DG_RELOAD_DWELL_MS,(unsigned long long)held_ms,
                    dg_reload_probe_verdict(valid,outside,dist_m,held_ms,permit));
}

#define DG_RELOAD_V2 1
#define DG_RELOAD_GRIP_PRESS .6
#define DG_RELOAD_GRIP_RELEASE .4
/* XR queue bits beside the M9 events (dg_m9_slide.h uses 1..16). */
enum { DG_RELOAD_EV_EJECT=0x100, DG_RELOAD_EV_FILL=0x200, DG_RELOAD_EV_CLICK=0x400, DG_RELOAD_EV_GUIDE=0x800 };
#define DG_RELOAD_EV_ALL (DG_RELOAD_EV_EJECT|DG_RELOAD_EV_FILL|DG_RELOAD_EV_CLICK|DG_RELOAD_EV_GUIDE)
/* A deliberate press: >= .6 after the grip was seen below .4. An invalid or
 * refused sample disarms, so a grip held through a menu, a codec call or a
 * tracking gap never becomes an eject when it ends. */
static int dg_reload_grip_edge(int *armed,int valid,double squeeze) {
    if(!valid || !isfinite(squeeze)){*armed=0;return 0;}
    if(squeeze<DG_RELOAD_GRIP_RELEASE){*armed=1;return 0;}
    if(*armed && squeeze>=DG_RELOAD_GRIP_PRESS){*armed=0;return 1;}
    return 0;
}

static int dg_reload_eject_mag(int mag) {return mag>1 ? 1 : mag;}
static int dg_reload_rifle(int weapon) {return weapon==15 || weapon==18;}
static int dg_reload_pattern(int weapon) {
    switch(weapon) {
        case 2:return 2;
        case 3:return 8;
        case 15:return 7;
        case 18:return 4;
        default:return -1;
    }
}
enum { DG_EJECT_OK=0, DG_EJECT_OFF, DG_EJECT_WEAPON, DG_EJECT_FPS, DG_EJECT_MENU,
       DG_EJECT_EAR, DG_EJECT_TWOHAND, DG_EJECT_BUSY, DG_EJECT_LOADED };
static const char *const dg_eject_names[]={"ok","off","weapon","first person","menu",
    "codec ear zone","two-hand grip","busy","nothing to eject"};
/* Order matters only for the log reason; every refusal is final for this press. */
static int dg_reload_eject_verdict(int on,int weapon,int fps,int allowed,int ear,
                                   int twohand,int busy,int mag) {
    if(!on)return DG_EJECT_OFF;
    if(dg_reload_pattern(weapon)<0)return DG_EJECT_WEAPON;
    if(!fps)return DG_EJECT_FPS;
    if(!allowed)return DG_EJECT_MENU;
    if(ear)return DG_EJECT_EAR;
    if(twohand)return DG_EJECT_TWOHAND;
    if(busy)return DG_EJECT_BUSY;
    if(dg_reload_eject_mag(mag)==mag)return DG_EJECT_LOADED;
    return DG_EJECT_OK;
}

#define DG_INSERT_STROKE_MM 53.0   /* well mouth below the seated origin (USP model: mouth z -51, seated -1.2, 20 deg rake) */
#define DG_INSERT_BELOW_MM 60.0    /* an offer may start this far under the mouth */
#define DG_INSERT_SLACK_MM 10.0    /* ... and up to this far into the mouth */
#define DG_INSERT_SIDE_IN_MM 25.0
#define DG_INSERT_SIDE_OUT_MM 45.0
#define DG_INSERT_DEG_IN 35.0
#define DG_INSERT_DEG_OUT 50.0
#define DG_INSERT_DROP_MM 80.0     /* pulled this far under the mouth: back into the hand */
#define DG_INSERT_SEAT_MM 8.0      /* within this of the seated place = in */
#define DG_INSERT_JUMP_MM 40.0     /* a larger step in one sample is a tracking jump: never seats */
#define DG_INSERT_OFFER_SAMPLES 2
#define DG_INSERT_STALE_MS 100
#define DG_INSERT_LOG_LINES 400
#define DG_RELOAD_PROBE_LINES 600
enum { DG_INS_HAND=0, DG_INS_GUIDED, DG_INS_SEATED };
enum { DG_INSWHY_NONE=0, DG_INSWHY_INVALID, DG_INSWHY_SIDE, DG_INSWHY_ANGLE, DG_INSWHY_ABOVE, DG_INSWHY_BELOW,
       DG_INSWHY_OFFERING, DG_INSWHY_SLIDING, DG_INSWHY_SEATED, DG_INSWHY_OUT_SIDE, DG_INSWHY_OUT_ANGLE,
       DG_INSWHY_OUT_DOWN, DG_INSWHY_JUMP, DG_INSWHY_COUNT };
static const char *const dg_insert_phase_names[]={"in hand","on the axis","seated"};
static const char *const dg_insert_why_names[]={"-","hand sample not valid","too far to the side of the well axis",
    "magazine not aligned with the well","not below the well mouth (from the side or above does not insert)",
    "too far below the well mouth","offered under the mouth","sliding up the well","seated",
    "pulled off the axis sideways","turned off the axis","pulled down out of the well","tracking jump ignored"};
typedef struct {
    uint64_t source, sequence, sample_ms;
    int phase, offers, why, guides, socom;   /* socom: the well mode, DG_INS_MODE_* (0 USP, 1 SOCOM, 2 rifle) */
    double along, side, deg, last_along, capture_along, capture_offset[3];

    int ref_ok, glitches;
    double ref_along, ref_side, ref_deg;
} DG_RELOAD_INSERT;
enum { DG_INS_MODE_USP=0, DG_INS_MODE_SOCOM=1, DG_INS_MODE_RIFLE=2 };

#define DG_RIFLE_STROKE_MM 45.0
#define DG_RIFLE_SIDE_IN_MM 45.0      /* offer in the well region (along >= -stroke) */
#define DG_RIFLE_SIDE_FAR_MM 60.0     /* offer below the mouth: a wider funnel */
#define DG_RIFLE_SIDE_OUT_MM 90.0
#define DG_RIFLE_DEG_IN 45.0
#define DG_RIFLE_DEG_OUT 60.0
#define DG_RIFLE_ABOVE_MM 60.0        /* tracking puts the held magazine up to this far past the seated place */
#define DG_RIFLE_BELOW_MM 105.0
#define DG_RIFLE_DROP_MM 140.0
#define DG_RIFLE_SEAT_MM 20.0

#define DG_RIFLE_GLITCH_MM 60.0
#define DG_RIFLE_GLITCH_DEG 35.0
#define DG_RIFLE_GLITCH_MAX 8

static double insert_side_in(int m){return m==DG_INS_MODE_RIFLE ? DG_RIFLE_SIDE_IN_MM : m ? 55.0 : DG_INSERT_SIDE_IN_MM;}
static double insert_side_out(int m){return m==DG_INS_MODE_RIFLE ? DG_RIFLE_SIDE_OUT_MM : m ? 85.0 : DG_INSERT_SIDE_OUT_MM;}
static double insert_deg_in(int m){return m==DG_INS_MODE_RIFLE ? DG_RIFLE_DEG_IN : m ? 45.0 : DG_INSERT_DEG_IN;}
static double insert_deg_out(int m){return m==DG_INS_MODE_RIFLE ? DG_RIFLE_DEG_OUT : DG_INSERT_DEG_OUT;}
static double insert_drop(int m){return m==DG_INS_MODE_RIFLE ? DG_RIFLE_DROP_MM : m ? 140.0 : DG_INSERT_DROP_MM;}
static double insert_stroke(int m){return m==DG_INS_MODE_RIFLE ? DG_RIFLE_STROKE_MM : DG_INSERT_STROKE_MM;}
static double insert_seat(int m){return m==DG_INS_MODE_RIFLE ? DG_RIFLE_SEAT_MM : DG_INSERT_SEAT_MM;}
/* Highest offer (along, mm) and the deepest one. */
static double insert_above(int m){return m==DG_INS_MODE_RIFLE ? DG_RIFLE_ABOVE_MM : -DG_INSERT_STROKE_MM+DG_INSERT_SLACK_MM;}
static double insert_below(int m){return -insert_stroke(m)-(m==DG_INS_MODE_RIFLE ? DG_RIFLE_BELOW_MM : m ? 105.0 : DG_INSERT_BELOW_MM);}
static double insert_side_offer(int m,double along){
    return m==DG_INS_MODE_RIFLE && along<-DG_RIFLE_STROKE_MM ? DG_RIFLE_SIDE_FAR_MM : insert_side_in(m);
}
/* Rifle sample filter (pure). Returns 1 when this sample is to be ignored. */
static int dg_reload_insert_glitch(DG_RELOAD_INSERT *s,double along,double side,double deg) {
    if(s->socom!=DG_INS_MODE_RIFLE)return 0;
    if(s->ref_ok && (fabs(along-s->ref_along)>DG_RIFLE_GLITCH_MM || fabs(side-s->ref_side)>DG_RIFLE_GLITCH_MM ||
                     fabs(deg-s->ref_deg)>DG_RIFLE_GLITCH_DEG) && s->glitches<DG_RIFLE_GLITCH_MAX) {
        s->glitches++;return 1;
    }
    s->glitches=0;s->ref_ok=1;s->ref_along=along;s->ref_side=side;s->ref_deg=deg;
    return 0;
}
static void dg_reload_insert_reset(DG_RELOAD_INSERT *s) {memset(s,0,sizeof *s);}
/* One coherent hand sample (duplicate render/eye passes of the same sequence do not advance). Returns the
 * phase. SEATED latches until the adapter resets it (after the fill or when the magazine is no longer owed). */
static int dg_reload_insert_step(DG_RELOAD_INSERT *s,uint64_t source,uint64_t seq,uint64_t now,int valid,
                                 double along,double side,double deg) {
    if(!valid || !source || !seq || !isfinite(along) || !isfinite(side) || !isfinite(deg) || side<0 ||
       (s->source && source!=s->source) || (s->sequence && seq<s->sequence) || now<s->sample_ms ||
       (s->sequence && now-s->sample_ms>DG_INSERT_STALE_MS)) {
        int keep=s->phase==DG_INS_SEATED;
        if(!keep){dg_reload_insert_reset(s);s->why=DG_INSWHY_INVALID;}
        return s->phase;
    }
    if(seq==s->sequence)return s->phase;
    s->source=source;s->sequence=seq;s->sample_ms=now;
    /* Rifle: an implausible jump is an invalid sample, not a pull off the axis (checked before every exit). */
    if(s->phase!=DG_INS_SEATED && dg_reload_insert_glitch(s,along,side,deg)){s->why=DG_INSWHY_JUMP;return s->phase;}
    s->along=along;s->side=side;s->deg=deg;
    if(s->phase==DG_INS_HAND) {
        if(side>insert_side_offer(s->socom,along)){s->offers=0;s->why=DG_INSWHY_SIDE;}
        else if(deg>insert_deg_in(s->socom)){s->offers=0;s->why=DG_INSWHY_ANGLE;}
        else if(along>insert_above(s->socom)){s->offers=0;s->why=DG_INSWHY_ABOVE;}
        else if(along<insert_below(s->socom)){s->offers=0;s->why=DG_INSWHY_BELOW;}
        else if(++s->offers>=DG_INSERT_OFFER_SAMPLES){s->phase=DG_INS_GUIDED;s->guides++;s->capture_along=s->last_along=along;s->why=DG_INSWHY_SLIDING;}
        else s->why=DG_INSWHY_OFFERING;
    } else if(s->phase==DG_INS_GUIDED) {
        if(along<-insert_stroke(s->socom)-insert_drop(s->socom)){s->phase=DG_INS_HAND;s->offers=0;s->why=DG_INSWHY_OUT_DOWN;}
        else if(side>insert_side_out(s->socom)){s->why=DG_INSWHY_OUT_SIDE;}
        else if(deg>insert_deg_out(s->socom)){s->why=DG_INSWHY_OUT_ANGLE;}
        else if(fabs(along-s->last_along)>DG_INSERT_JUMP_MM){s->why=DG_INSWHY_JUMP;}
        else {
            s->last_along=along;
            if(along>=-insert_seat(s->socom)){s->phase=DG_INS_SEATED;s->why=DG_INSWHY_SEATED;}
            else s->why=DG_INSWHY_SLIDING;
        }
    }
    return s->phase;
}
/* Where the guided magazine is drawn: on the axis at the hand's projection, never above the seated place and
 * never further down than where it would leave the axis. */
static double dg_reload_insert_draw_mm(double along,int socom) {
    if(!(along<0))return 0;
    if(along<-insert_stroke(socom)-insert_drop(socom))return -insert_stroke(socom)-insert_drop(socom);
    return along;
}
/* Pure geometry. a = seated magazine origin, b = held origin (world units), u = well axis (world, any length),
 * held_axis = the held magazine's long axis (world, any length); upm = world units per metre. */
static int dg_reload_insert_measure(const double a[3],const double b[3],const double u[3],const double held_axis[3],
                                    double upm,double *along_mm,double *side_mm,double *deg) {
    double nu=sqrt(u[0]*u[0]+u[1]*u[1]+u[2]*u[2]),nh=sqrt(held_axis[0]*held_axis[0]+held_axis[1]*held_axis[1]+held_axis[2]*held_axis[2]);
    double d[3],t,p[3],c;int k;
    if(!(nu>1e-9) || !(nh>1e-9) || !(upm>0) || !isfinite(upm))return 0;
    for(k=0;k<3;k++)d[k]=b[k]-a[k];
    t=(d[0]*u[0]+d[1]*u[1]+d[2]*u[2])/nu;
    for(k=0;k<3;k++)p[k]=d[k]-t*u[k]/nu;
    c=(u[0]*held_axis[0]+u[1]*held_axis[1]+u[2]*held_axis[2])/(nu*nh);
    if(c>1)c=1;if(c<-1)c=-1;
    *along_mm=t/upm*1000.0;*side_mm=sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2])/upm*1000.0;*deg=acos(c)*57.29577951308232;
    return isfinite(*along_mm) && isfinite(*side_mm) && isfinite(*deg);
}
/* Probe segment appended to the reload probe line (diagnostic builds). */
static int dg_reload_insert_probe_format(char *out,size_t n,const DG_RELOAD_INSERT *s,int touch) {
    if(touch)return snprintf(out,n," | insert: off (touch mode: contact fills)");
    return snprintf(out,n," | insert: %s, along %.0f mm (mouth at -%.0f, in at -%.0f), side %.0f mm (offer <= %.0f, off > %.0f), axis %.0f deg (<= %.0f) | %s",
        dg_insert_phase_names[s->phase<0||s->phase>2?0:s->phase],s->along,insert_stroke(s->socom),insert_seat(s->socom),
        s->side,insert_side_offer(s->socom,s->along),insert_side_out(s->socom),s->deg,insert_deg_in(s->socom),
        dg_insert_why_names[s->why<0||s->why>=DG_INSWHY_COUNT?0:s->why]);
}

#define DG_INSERT_RECIPE_MM 80.0
static int dg_reload_insert_geo_format(char *out,size_t n,int ok,const double a_trk[3],const double well_trk[3],
                                       const double mag_trk[3],const double held_trk[3],int from_ctrl,double off_mm) {
    double u[3],v[3],nu=0,nv=0,m_per_mm,mouth[3],offer[3],move[3];int k;
    const char *src=from_ctrl?"measured at the controller":"measured at the drawn hand (no IK target)";
    for(k=0;k<3 && ok;k++){u[k]=well_trk[k]-a_trk[k];v[k]=held_trk[k]-mag_trk[k];nu+=u[k]*u[k];nv+=v[k]*v[k];}
    if(!ok || !(nu>1e-12) || !(nv>1e-12) || !isfinite(nu) || !isfinite(nv))
        return snprintf(out,n," | recipe: tracking geometry not available yet (%s)",src);
    nu=sqrt(nu);nv=sqrt(nv);m_per_mm=nu/100.0;
    for(k=0;k<3;k++) {
        u[k]/=nu;v[k]/=nv;
        mouth[k]=a_trk[k]-u[k]*DG_INSERT_STROKE_MM*m_per_mm;
        offer[k]=a_trk[k]-u[k]*DG_INSERT_RECIPE_MM*m_per_mm;
        move[k]=offer[k]-mag_trk[k];
    }
    return snprintf(out,n," | recipe (tracking m): well axis up (%.3f %.3f %.3f), magazine axis (%.3f %.3f %.3f), "
        "mouth (%.3f %.3f %.3f), held magazine (%.3f %.3f %.3f), offer point %.0f mm under seated (%.3f %.3f %.3f), "
        "move the left controller by (%+.3f %+.3f %+.3f) | %s, drawn hand %.0f mm off it",
        u[0],u[1],u[2],v[0],v[1],v[2],mouth[0],mouth[1],mouth[2],mag_trk[0],mag_trk[1],mag_trk[2],
        DG_INSERT_RECIPE_MM,offer[0],offer[1],offer[2],move[0],move[1],move[2],src,from_ctrl?off_mm:0.0);
}
#endif
