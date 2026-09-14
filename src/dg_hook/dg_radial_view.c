#include "dg_radial_view.h"
#include <math.h>
#include <string.h>
/* Original five-column glyph construction, rows encoded left-to-right. */
static const unsigned char glyphs[38][7] = {
 {14,17,17,31,17,17,17},{30,17,17,30,17,17,30},
 {14,17,16,16,16,17,14},{30,17,17,17,17,17,30},
 {31,16,16,30,16,16,31},{31,16,16,30,16,16,16},
 {14,17,16,23,17,17,15},{17,17,17,31,17,17,17},
 {31,4,4,4,4,4,31},{7,2,2,2,18,18,12},
 {17,18,20,24,20,18,17},{16,16,16,16,16,16,31},
 {17,27,21,21,17,17,17},{17,25,21,19,17,17,17},
 {14,17,17,17,17,17,14},{30,17,17,30,16,16,16},
 {14,17,17,17,21,18,13},{30,17,17,30,20,18,17},
 {15,16,16,14,1,1,30},{31,4,4,4,4,4,4},
 {17,17,17,17,17,17,14},{17,17,17,17,17,10,4},
 {17,17,17,21,21,27,17},{17,17,10,4,10,17,17},
 {17,17,10,4,4,4,4},{31,1,2,4,8,16,31},
 {14,17,19,21,25,17,14},{4,12,4,4,4,4,14},
 {14,17,1,2,4,8,31},{30,1,1,14,1,1,30},
 {2,6,10,18,31,2,2},{31,16,16,30,1,1,30},
 {14,16,16,30,17,17,14},{31,1,2,4,8,8,8},
 {14,17,17,14,17,17,14},{14,17,17,15,1,1,14},
 {0,0,0,31,0,0,0}, {0,0,0,0,0,0,4}
};
static int glyph(unsigned char c) {
    if (c >= 'a' && c <= 'z') c = (unsigned char)(c-'a'+'A');
    if (c >= 'A' && c <= 'Z') return c-'A';
    if (c >= '0' && c <= '9') return c-'0'+26;
    return c == '-' ? 36 : c == '.' ? 37 : -1;
}
static void put(unsigned char *p, int x, int y, unsigned char r,
                unsigned char g, unsigned char b) {
    size_t i;
    if (x < 0 || x >= 256 || y < 0 || y >= 256) return;
    i = ((size_t)y*256u+(size_t)x)*4u;
    p[i]=r; p[i+1]=g; p[i+2]=b; p[i+3]=255;
}
static void text(unsigned char *p, const char *s, int n, int x, int y) {
    int i,r,c,k;
    for (i=0;i<n;i++) {
        k=glyph((unsigned char)s[i]);
        if (k < 0) continue;
        for (r=0;r<7;r++) for (c=0;c<5;c++)
            if (glyphs[k][r] & (1u << (4-c)))
                put(p,x+i*6+c,y+r,255,255,255);
    }
}
int dg_radial_view_raster(const dg_radial_view *v, uint32_t *pixels,
                          size_t capacity) {
    static const double tau=6.2831853071795864769;
    unsigned char *p=(unsigned char *)pixels;
    int lengths[8],x,y,i,n;
    if (!pixels || capacity < DG_RADIAL_VIEW_PIXELS) return -1;
    memset(p,0,DG_RADIAL_VIEW_PIXELS*4u);
    if (!v || !v->visible) return 0;
    if (v->count<6 || v->count>8 || v->selected < -1 ||
        v->selected >= (int)v->count || v->kind>1 ||
        (v->eligible >> v->count)) return 0;
    for (i=0;i<8;i++) {
        for (n=0;n<17 && v->labels[i][n];n++)
            if (v->labels[i][n]!=' ' && glyph((unsigned char)v->labels[i][n])<0)
                return 0;
        if (n==17) return 0;
        lengths[i]=n;
    }
    for (y=0;y<256;y++) for (x=0;x<256;x++) {
        double dx=(double)x-127.5,dy=127.5-(double)y;
        double radius=dx*dx+dy*dy,a,phase;
        unsigned slot;
        unsigned char r=38,g=83,b=112;
        if (radius<40.0*40.0 || radius>120.0*120.0) continue;
        a=atan2(dx,dy); if (a<0) a+=tau;
        phase=a*(double)v->count/tau+0.5;
        slot=(unsigned)floor(phase)%v->count;
        if (phase-floor(phase)<0.025 || phase-floor(phase)>0.975) continue;
        if (!lengths[slot]) {r=25;g=25;b=30;}
        else if (!(v->eligible & (1u<<slot))) {r=67;g=67;b=67;}
        if ((int)slot==v->selected) {
            if (lengths[slot] && (v->eligible & (1u<<slot))) {r=176;g=117;b=12;}
            else {r=112;g=38;b=38;}
        }
        put(p,x,y,r,g,b);
    }
    for (i=0;i<(int)v->count;i++) {
        double a=tau*(double)i/(double)v->count;
        int cx=(int)(127.5+82.0*sin(a)),cy=(int)(127.5-82.0*cos(a));
        /* Two eight-character lines bound labels to 47 pixels width. */
        n=lengths[i];
        if (n>8) {
            text(p,v->labels[i],8,cx-23,cy-7);
            text(p,v->labels[i]+8,n-8,cx-((n-8)*6-1)/2,cy+1);
        } else if (n) text(p,v->labels[i],n,cx-(n*6-1)/2,cy-3);
    }
    /* Title outside the center deadzone, so its center remains transparent. */
    text(p,v->kind ? "ITEMS" : "WEAPONS",v->kind ? 5 : 7,
         v->kind ? 113 : 107,2);
    text(p,v->kind ? "TRIGGER ACTION" : "TRIGGER SELECT",14,86,247);
    return 1;
}
