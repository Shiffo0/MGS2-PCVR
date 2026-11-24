#ifndef DG_LASER_UI_H
#define DG_LASER_UI_H
/* Values shared by the Present-side menu and the XR-side texture renderer. */
enum { LASER_UP, LASER_DOWN, LASER_LEFT, LASER_RIGHT, LASER_OK, LASER_BACK,
       LASER_SETTINGS, LASER_FIRST_ROW, LASER_DONE=LASER_FIRST_ROW+8 };
enum { LASER_VIEW, LASER_RELOAD, LASER_SLIDE, LASER_OBSERVER, LASER_SECOND,
       LASER_TURN, LASER_RADAR, LASER_HAND, LASER_ROWS };
static const int laser_count[LASER_ROWS]={2,2,2,4,2,2,3,2};
static const char *const laser_name[LASER_ROWS]={
    "VIEW","VR PISTOL RELOAD","M9 SLIDE","DESKTOP OBSERVER","SECOND EYE","TURN","RADAR","MENU HAND"};
static const char *const laser_choice[LASER_ROWS][4]={
    {"MONO","STEREO"},{"OFF","ON"},{"OFF","ON"},{"OFF","LEFT","RIGHT","BOTH"},
    {"OFF","ON"},{"OFF","SMOOTH"},{"HUD","FIXED","WRIST"},{"LEFT","RIGHT"}};
typedef struct {int x,y,w,h;} DG_LASER_RECT;
static int laser_rect(int id,int width,int height,int page,int options,DG_LASER_RECT *r) {
    int rows=page?9:1;
    int scale=width/600, h, w;
    if(scale<1)scale=1;
    if(scale>4)scale=4;
    h=16*scale;
    w=(page?204:options?244:174)*scale;
    if(width<w || height<rows*h+8)return 0;
    r->x=(width-w)/2;r->w=w;r->h=h;
    if(page){
        if(id<LASER_FIRST_ROW || id>LASER_DONE)return 0;
        r->y=(height-rows*h)/2+(id-LASER_FIRST_ROW)*h;
    }else{
        if(id<0 || id>(options?LASER_SETTINGS:LASER_BACK))return 0;
        r->x+=id*29*scale;r->w=(id==LASER_SETTINGS?70:29)*scale;
        r->y=height-2*h;
    }
    return 1;
}

enum { LASER_NATIVE_ROW0=LASER_DONE+1, LASER_NATIVE_ROWS=9, LASER_NATIVE_VR=LASER_NATIVE_ROW0+LASER_NATIVE_ROWS,
       LASER_NATIVE_LAST=LASER_NATIVE_VR };
#define LASER_NATIVE_V0     0.1676
#define LASER_NATIVE_PITCH  0.05764
#define LASER_NATIVE_U0     0.068
#define LASER_NATIVE_U1     0.83
/* Which native row (LASER_NATIVE_ROW0.. / LASER_NATIVE_VR) a hit at (u,v) is on, or -1. Rows touch, no gaps. */
static int laser_native_hit(double u,double v) {
    double r;
    int i;
    if(!(u>=LASER_NATIVE_U0 && u<=LASER_NATIVE_U1))return -1;
    r=(v-LASER_NATIVE_V0)/LASER_NATIVE_PITCH;
    if(!(r>=-0.5 && r<LASER_NATIVE_ROWS+0.5))return -1;
    i=(int)floor(r+0.5);
    return i>=LASER_NATIVE_ROWS ? LASER_NATIVE_VR : LASER_NATIVE_ROW0+i;
}
/* The drawn VR SETTINGS line, one pitch high, left-aligned with the game's row text. */
static int laser_native_vr_rect(int width,int height,DG_LASER_RECT *r) {
    double cy=LASER_NATIVE_V0+LASER_NATIVE_PITCH*LASER_NATIVE_ROWS;
    int h=(int)(LASER_NATIVE_PITCH*height*0.8);
    if(width<64 || height<64 || h<8)return 0;
    r->x=(int)(0.109*width);r->w=(int)(0.40*width);
    r->h=h;r->y=(int)(cy*height)-h/2;
    return r->y>=0 && r->y+r->h<=height;
}
/* Present side: the shortest single step from the native cursor to the wanted row (9 rows, both ends wrap).
   Returns +1 for UP (the game's U bit: cursor-1), -1 for DOWN, 0 when already there or out of range. */
static int laser_native_step(int cursor,int want) {
    int down;
    if(cursor<0 || cursor>=LASER_NATIVE_ROWS || want<0 || want>=LASER_NATIVE_ROWS || cursor==want)return 0;
    down=(want-cursor+LASER_NATIVE_ROWS)%LASER_NATIVE_ROWS;
    return down<=LASER_NATIVE_ROWS/2 ? -1 : 1;
}

enum { NS_NONE, NS_MAIN, NS_LOAD };
#define NS_MAIN_MAX 7
typedef struct { int kind, n, cur, ready, wrap; double v[10], u0[10], u1[10], half; } DG_NATIVE_LIST;
/* Shortest single step on an n-row list; wrap = the list wraps at both ends. +1 UP (cursor-1), -1 DOWN. */
static int laser_native_step_n(int cursor,int want,int n,int wrap) {
    int down;
    if(n<1 || cursor<0 || cursor>=n || want<0 || want>=n || cursor==want)return 0;
    if(!wrap)return want>cursor ? -1 : 1;
    down=(want-cursor+n)%n;
    return down<=n/2 ? -1 : 1;
}
/* Which row of the list (u,v) is on, or -1. */
static int laser_native_list_hit(const DG_NATIVE_LIST *l,double u,double v) {
    int i;
    for(i=0;l && i<l->n && i<10;i++)
        if(l->v[i]>=0 && fabs(v-l->v[i])<l->half && u>=l->u0[i] && u<=l->u1[i])return i;   /* v < 0: hidden item */
    return -1;
}
typedef struct { int last_want, reached, pulse_from, pending; unsigned long long pulse_ms, pending_ms; int rows, wrap; } LASER_NATIVE_NAV;   /* rows 0 = OPTIONS (9, wrapping) */
static void laser_native_nav_reset(LASER_NATIVE_NAV *n) { n->last_want=n->reached=n->pulse_from=n->pending=-1; n->pulse_ms=n->pending_ms=0; n->rows=0; n->wrap=1; }
/* Returns the step for this frame (+1 UP, -1 DOWN, 0 none); *ok=1 when OK goes out this frame. */
static int laser_native_nav(LASER_NATIVE_NAV *n,int want,int cur,int ready,int click,unsigned long long now,int *ok) {
    int waiting, st;
    *ok=0;
    if(want!=n->last_want){n->last_want=want;n->reached=-1;n->pending=-1;}
    if(click && want>=0){n->pending=want;n->pending_ms=now;n->reached=-1;}
    if(n->pending>=0 && now-n->pending_ms>1500)n->pending=-1;
    waiting=n->pulse_from>=0 && cur==n->pulse_from && now-n->pulse_ms<400;
    if(n->pulse_from>=0 && !waiting)n->pulse_from=-1;
    if(want>=0 && cur==want)n->reached=want;
    if(want>=0 && n->reached==want && cur!=want && n->pulse_ms && now-n->pulse_ms<600)n->reached=-1;
    if(want<0 || !ready)return 0;
    if(cur==want){ if(n->pending==want){*ok=1;n->pending=-1;} return 0; }
    if(n->reached==want || waiting)return 0;
    st=n->rows ? laser_native_step_n(cur,want,n->rows,n->wrap) : laser_native_step(cur,want);
    if(st){n->pulse_from=cur;n->pulse_ms=now;}
    return st;
}

static int laser_stick_dir(double x,double y,double deadzone) {
    double m=sqrt(x*x+y*y),ax,ay;
    if(!(m==m) || !(deadzone==deadzone))return 0;
    if(deadzone<0.05)deadzone=0.05;
    if(deadzone>0.9)deadzone=0.9;
    if(m>1.0){x/=m;y/=m;m=1.0;}
    if(m<=deadzone)return 0;
    ax=x<0?-x:x;ay=y<0?-y:y;
    if(ay<ax)return 0;
    return y>0?1:-1;
}
/* Stick/keys on the OPTIONS screen (controller input as menu_seam sees it). From EXIT, DOWN selects VR SETTINGS instead
   of wrapping to RADAR; there UP returns to EXIT, DOWN goes on to RADAR (the game's own wrap from EXIT), OK opens the
   VR page. Pure: returns the new selection; *y / *a are what the game gets this frame. Edges only. */
static int laser_vr_row_stick(int sel,int cur,int ready,int *y,int *a,int *prev_y,int *prev_a,int *open) {
    int y_edge=*y && *y!=*prev_y, a_edge=*a && !*prev_a;
    *prev_y=*y;*prev_a=*a;*open=0;
    if(cur!=8)sel=0;   /* the native cursor is elsewhere (laser, keyboard): not on our row */
    if(!sel) {
        if(cur==8 && ready && y_edge && *y<0){sel=1;*y=0;}
        return sel;
    }
    if(a_edge){*open=1;sel=0;*a=0;return sel;}
    *a=0;
    if(y_edge && *y>0){sel=0;*y=0;return sel;}
    if(y_edge && *y<0){sel=0;return sel;}   /* passes: EXIT -> RADAR */
    *y=0;                                     /* held stick while on our row: nothing */
    return sel;
}
#endif
