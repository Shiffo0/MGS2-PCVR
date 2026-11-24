#define M9P_C_FINGER    0x001u
#define M9P_C_PREVIEW   0x002u
#define M9P_C_WORLD     0x004u
#define M9P_C_WEIGHT    0x008u
#define M9P_C_FREE_EN   0x010u
#define M9P_C_FREE_OK   0x020u
#define M9P_C_RELOAD    0x040u
#define M9P_C_BLEND     0x080u
#define M9P_C_WRIST     0x100u
static __declspec(thread) struct {
    int site_route,site_line,site_solve;unsigned site_mask;
    double site_miss_mm,site_shoulder_mm,site_reach_mm,site_ease_ms;
} g_m9_site;
static void m9_probe_left_pair(LONG gate,double blend,int engaged,int aiming,int reload,double ease_ms){}
static void m9_probe_site(int route,int line,unsigned mask,int solve,double miss_mm,
                          double shoulder_mm,double reach_mm,double ease_ms){}
static void m9_probe_step(const DG_M9_STATE *pre,const DG_M9_INPUT *in,const DG_M9_SAMPLE *s,
                          unsigned events,int safe,int can_write,int weapon,uint64_t now){}
static void m9_probe_offtick(int route,const DG_M9_STATE *pre){}
