
void dg_xr_test_publish_status(const DG_XR_FRAME *frame, int flags);
static int test_scene_blur(void)
{
    int bad=0, mask, flags, i;
    unsigned char a[]={0xf3,0x48,0x0f,0x2c,0x5f,0x78};
    unsigned char b[]={0xc1,0xe3,0x18,0x0f,0x57,0xc9,0x81,0xcb,0x80,0x80,0x80,0};
    unsigned char p[]={0xba,1,0,0,0,0x48,0x8b,0xc8,0xe8,0xf0,0x9b,0xdd,0xff};
    unsigned char s[]={0x89,0x5c,0x24,0x48};
    unsigned char *spans[]={a,b,p,s};
    size_t sizes[]={sizeof a,sizeof b,sizeof p,sizeof s}, j;
    CONTEXT c, before, expected;
    EXCEPTION_RECORD er;
    EXCEPTION_POINTERS ep;
    ULONG64 saved_addr=g_scene_blur_addr;
    LONG saved_source=g_source, saved_hits=g_scene_blur_hits;
    LONG saved_traps=g_traps, saved_zeroed=g_scene_blur_zeroed;
    LONG saved_armed=g_armed, saved_stereo=g_stereo, saved_enabled=g_scene_blur_enabled;
    ULONG64 saved_hook=g_hook_addr;
#define BLUR_CHECK(x) do { if(!(x)){printf("  FAIL scene blur line %d: %s\n",__LINE__,#x);bad++;} } while(0)
    BLUR_CHECK(dg_scene_blur_signature(a,b,p,s));
    {
        const char *words[]={"source=xr", "source=xr vr_scene_blur_fix=0",
                             "source=xr vr_scene_blur_fix=1",
                             "source=xr vr_scene_blur_fix=2"};
        int wants[]={1,0,1,0};
        for(i=0;i<4;i++) {
            char buf[128]; POSE pose; double seconds=600;
            int source,track,map[4],hand; ARM_POS_CFG ap;
            DG_XR_CONFIG cfg; DG_BRIDGE_CONFIG bridge;
            strcpy_s(buf,sizeof buf,words[i]);
            BLUR_CHECK(parse_config(buf,&pose,&seconds,&source,&cfg,&bridge,
                                     &track,map,&hand,&ap));
            BLUR_CHECK(cfg.scene_blur_fix==wants[i]);
        }
    }
    for(i=0;i<4;i++) for(j=0;j<sizes[i];j++) {
        spans[i][j]^=1;
        BLUR_CHECK(!dg_scene_blur_signature(a,b,p,s));
        spans[i][j]^=1;
    }
    /* Every combination of six gates and pose/FOV availability. */
    for(mask=0;mask<64;mask++) for(flags=0;flags<4;flags++) {
        int yes = mask==15 && flags==3;
        int actual = dg_scene_blur_allowed(mask&1,mask&2,mask&4,mask&8,
                                            mask&16,mask&32,flags);
        BLUR_CHECK(actual==yes);
        memset(&c,0x55,sizeof c);
        c.Rip=0x12345678; c.Dr6=0x4002; c.Rbx=64;
        before=c; expected=c;
        expected.Dr6&=~2ull; expected.EFlags|=0x10000;
        if(yes) expected.Rbx=0;
        BLUR_CHECK(dg_scene_blur_context(&c,0x12345678,actual));
        BLUR_CHECK(!memcmp(&c,&expected,sizeof c));
        /* Native SHL/OR/store contract: alpha changes, RGB stays 808080. */
        BLUR_CHECK((((unsigned)c.Rbx<<24)|0x00808080u)==
                     (yes?0x00808080u:0x40808080u));
    }
    for(i=0;i<3;i++) {
        c=before;
        if(i==0)c.Rip++;
        if(i==1)c.Dr6=1;
        if(i==2)c.Dr6=3; /* do not swallow simultaneous foreign trap */
        expected=c;
        BLUR_CHECK(!dg_scene_blur_context(&c,0x12345678,1));
        BLUR_CHECK(!memcmp(&c,&expected,sizeof c));
    }
    c=before;expected=c;
    BLUR_CHECK(!dg_scene_blur_context(&c,0,1));
    BLUR_CHECK(!memcmp(&c,&expected,sizeof c));
    /* Real VEH dispatch, original alpha when not XR; not the camera path. */
    memset(&er,0,sizeof er); er.ExceptionCode=EXCEPTION_SINGLE_STEP;
    ep.ExceptionRecord=&er;ep.ContextRecord=&c;
    g_scene_blur_addr=0x12345678;g_source=SRC_SYNTHETIC;
    c=before;
    BLUR_CHECK(veh(&ep)==EXCEPTION_CONTINUE_EXECUTION);
    BLUR_CHECK(c.Rbx==64 && !(c.Dr6&2) && (c.EFlags&0x10000));
    BLUR_CHECK(g_scene_blur_hits==saved_hits+1);
    BLUR_CHECK(g_scene_blur_zeroed==saved_zeroed);
    c=before;c.Rip++;
    BLUR_CHECK(veh(&ep)==EXCEPTION_CONTINUE_SEARCH);
    /* Real CPU -> DR1 -> shipping VEH -> native SHL/OR -> pixel colour.
       Only locally generated code on our own suspended test thread, never
       game code. Callee-saved RBX is pushed/popped by this fixture. */
    {
        static const unsigned char code[]={0x53,0xbb,0x40,0,0,0,
            0xc1,0xe3,0x18,0x81,0xcb,0x80,0x80,0x80,0,0x89,0xd8,0x5b,0xc3};
        unsigned char *page=(unsigned char *)VirtualAlloc(NULL,4096,
                                     MEM_RESERVE|MEM_COMMIT,PAGE_READWRITE);
        DWORD old;
        PVOID handler=NULL;
        DG_XR_FRAME frame;
        DG_XR_FRAME saved_frame;
        int saved_flags;
        memset(&saved_frame,0,sizeof saved_frame);
        saved_flags=dg_xr_get_stereo(&saved_frame);
        BLUR_CHECK(page!=NULL);
        if(page) {
            memcpy(page,code,sizeof code);
            BLUR_CHECK(VirtualProtect(page,4096,PAGE_EXECUTE_READ,&old));
            FlushInstructionCache(GetCurrentProcess(),page,sizeof code);
            handler=AddVectoredExceptionHandler(1,veh);
            BLUR_CHECK(handler!=NULL);
            g_scene_blur_addr=(ULONG64)(page+6);g_hook_addr=(ULONG64)(page+64);
            g_armed=1;g_stereo=1;g_scene_blur_enabled=1;
            memset(&frame,0,sizeof frame);
            if(handler) for(i=0;i<2;i++) {
                HANDLE thread;DWORD result=0,waited;
                g_source=i==0?SRC_XR:SRC_SYNTHETIC;
                dg_xr_test_publish(&frame);
                thread=CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)page,NULL,
                                    CREATE_SUSPENDED,NULL);
                BLUR_CHECK(thread!=NULL);
                if(!thread)continue;
                BLUR_CHECK(set_dr(thread,1));
                ResumeThread(thread);
                waited=WaitForSingleObject(thread,5000);
                /* A hung fixture makes the test process fail immediately;
                   do not free code or remove its handler under that thread. */
                if(waited!=WAIT_OBJECT_0) ExitProcess(1);
                BLUR_CHECK(GetExitCodeThread(thread,&result));
                BLUR_CHECK(result==(i==0?0x00808080u:0x40808080u));
                CloseHandle(thread);
            }
            if(handler)RemoveVectoredExceptionHandler(handler);
            VirtualFree(page,0,MEM_RELEASE);
        }
        dg_xr_test_publish_status(&saved_frame,saved_flags);
    }
    g_scene_blur_addr=saved_addr;g_source=saved_source;
    g_armed=saved_armed;g_stereo=saved_stereo;g_scene_blur_enabled=saved_enabled;
    g_hook_addr=saved_hook;
    g_scene_blur_hits=saved_hits;g_scene_blur_zeroed=saved_zeroed;g_traps=saved_traps;
    printf("  %s scene blur: 256 gate/context cases, signatures, alpha/RGB, foreign traps, real DR1/VEH\n",bad?"FAIL":"PASS");
#undef BLUR_CHECK
    return bad;
}
