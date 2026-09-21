/* Included in the bridge. The game seam publishes numeric profiles under a
   short lock; only the worker performs file I/O. Actor releases never touch it. */
#include <io.h>
static SRWLOCK g_hand_profile_lock=SRWLOCK_INIT;
static DG_HAND_PROFILE g_hand_profile;
static int g_hand_profile_initialized;
static unsigned long g_hand_profile_revision;
static unsigned long g_hand_profile_saved;
static unsigned long g_hand_calibration_seen;
static struct { DG_HAND_PROFILE profile; unsigned mask; } g_hand_capture;

void dg_bridge_hand_profile_get(DG_HAND_PROFILE *out) {
    AcquireSRWLockExclusive(&g_hand_profile_lock);
    if(!g_hand_profile_initialized) {
        dg_hand_profile_default(&g_hand_profile);g_hand_profile_initialized=1;
    }
    *out=g_hand_profile;
    ReleaseSRWLockExclusive(&g_hand_profile_lock);
}
typedef struct {
    unsigned int magic,version;
    DG_HAND_PROFILE profile;
    unsigned int checksum;
} DG_HAND_PROFILE_FILE;
static unsigned int hand_profile_checksum(const DG_HAND_PROFILE_FILE *f) {
    const unsigned char *p=(const unsigned char *)f;
    size_t i;unsigned int h=2166136261u;
    for(i=0;i<offsetof(DG_HAND_PROFILE_FILE,checksum);i++)h=(h^p[i])*16777619u;
    return h;
}
static int hand_profile_read(const wchar_t *path,DG_HAND_PROFILE *out) {
    DG_HAND_PROFILE_FILE b;FILE *f;int ok;
    if(_wfopen_s(&f,path,L"rb")) {
        DWORD attr=GetFileAttributesW(path),error=GetLastError();
        return attr==INVALID_FILE_ATTRIBUTES &&
            (error==ERROR_FILE_NOT_FOUND || error==ERROR_PATH_NOT_FOUND)?0:-1;
    }
    memset(&b,0,sizeof b);
    ok=fread(&b,1,sizeof b,f)==sizeof b && fgetc(f)==EOF && !ferror(f);
    fclose(f);
    if(!ok || b.magic!=0x48504744u || b.version!=1 ||
       b.checksum!=hand_profile_checksum(&b) || !dg_hand_profile_valid(&b.profile))return -1;
    *out=b.profile;return 1;
}
static int hand_profile_write(const wchar_t *path,const DG_HAND_PROFILE *p) {
    DG_HAND_PROFILE_FILE b;wchar_t temp[MAX_PATH];FILE *f;int ok;
    if(!dg_hand_profile_valid(p) ||
       _snwprintf_s(temp,MAX_PATH,_TRUNCATE,L"%s.%lu.tmp",path,GetCurrentProcessId())<0)return 0;
    memset(&b,0,sizeof b);b.magic=0x48504744u;b.version=1;b.profile=*p;
    b.checksum=hand_profile_checksum(&b);
    if(_wfopen_s(&f,temp,L"wb"))return 0;
    ok=fwrite(&b,1,sizeof b,f)==sizeof b;
    if(fflush(f) || _commit(_fileno(f)))ok=0;
    if(fclose(f))ok=0;
    if(ok)ok=MoveFileExW(temp,path,MOVEFILE_REPLACE_EXISTING|MOVEFILE_WRITE_THROUGH)!=0;
    if(!ok)DeleteFileW(temp);
    return ok;
}
int dg_bridge_hand_profile_worker(void) {
    static int loaded;static wchar_t path[MAX_PATH];
    static DWORD retry_at;static int retry;
    DG_HAND_PROFILE p;unsigned long revision;int result=0;
    dg_bridge_hand_profile_get(&p);
    if(!loaded) {
        wchar_t *leaf;DWORD n=GetModuleFileNameW(NULL,path,MAX_PATH);
        loaded=1;
        if(!n || n>=MAX_PATH || !(leaf=wcsrchr(path,L'\\')) ||
           wcscpy_s(leaf+1,MAX_PATH-(size_t)(leaf+1-path),L"dg_hand_calibration.bin")) {
            path[0]=0;return -1;
        }
        result=hand_profile_read(path,&p);
        if(!result)result=3;
        if(result==1) {
            AcquireSRWLockExclusive(&g_hand_profile_lock);
            if(!g_hand_profile_revision)g_hand_profile=p;
            ReleaseSRWLockExclusive(&g_hand_profile_lock);
            result=2;
        }
    }
    if(!path[0])return 0;
    AcquireSRWLockShared(&g_hand_profile_lock);
    p=g_hand_profile;revision=g_hand_profile_revision;
    ReleaseSRWLockShared(&g_hand_profile_lock);
    if(revision==g_hand_profile_saved)return result;
    if(retry && (DWORD)(GetTickCount()-retry_at)<5000u)return result;
    if(!hand_profile_write(path,&p)) {retry=1;retry_at=GetTickCount();return -1;}
    retry=0;g_hand_profile_saved=revision;return 1;
}
static void hand_profile_stage(int hand,const DG_BRIDGE_ARM_TARGET *t,
                                const DG_FREE_WRIST_INPUT *in,const double neutral[4]) {
    if(!t->persistent_hands || !t->hand_calibration_request ||
       t->hand_calibration_request==g_hand_calibration_seen ||
       !g_b.arm_map_unarmed || t->weight!=1 || t->left_weight!=1)return;
    if(dg_hand_profile_capture(in,neutral,g_hand_capture.profile.rotation[hand]))
        g_hand_capture.mask|=1u<<hand;
}
static void hand_profile_commit(const DG_BRIDGE_ARM_TARGET *t,int both_committed) {
    if(!both_committed || g_hand_capture.mask!=3 ||
       !dg_hand_profile_valid(&g_hand_capture.profile))return;
    AcquireSRWLockExclusive(&g_hand_profile_lock);
    g_hand_profile=g_hand_capture.profile;g_hand_profile_initialized=1;
    ++g_hand_profile_revision;
    ReleaseSRWLockExclusive(&g_hand_profile_lock);
    g_hand_calibration_seen=t->hand_calibration_request;
}
