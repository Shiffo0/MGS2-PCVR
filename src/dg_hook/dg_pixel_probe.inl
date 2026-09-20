/* Opt-in diagnostic, not a rendering fix. All mutable state is under g_cap_cs.
 * Three attempts per process, one in flight, only armed after a focused quad.
 * Private staging copies perturb GPU scheduling. No Flush, wait loop, blocking
 * Map, or access to an XR image after release. A pending pre-release result is
 * evidence, not permission to force completion. Late reads use staging only.
 */
enum { PP_IDLE, PP_ARMED, PP_SOURCE, PP_GPU, PP_DONE };
typedef struct {
    unsigned count, nonblack, lo[4], hi[4];
    uint64_t hash;
    int valid;
} DG_PIXEL_STATS;
static struct {
    int enabled, phase, before_attempted, before_ready, read_before;
    unsigned attempts, width, height, format, actual_format[3], polls;
    ULONGLONG next_ms, arm_ms, capture_ms, submit_ms, read_ms;
    uint64_t capture_id, frame;
    ID3D11Texture2D *stage[3];
    ID3D11Query *event;
    HRESULT hr, before_hr, map_hr[3];
    DG_PIXEL_STATS stats[3];
    const char *outcome;
    double producer_us, submit_us, read_us, pose_us;
    int quad_valid, release_attempted, end_attempted;
    XrResult head_result, release_result, end_result;
    XrSpaceLocation head;
    XrCompositionLayerQuad quad;
    XrTime display_time;
} g_pp;
static int g_pp_initialized;

static double pixel_probe_us(LARGE_INTEGER start) {
    LARGE_INTEGER end, freq;
    QueryPerformanceCounter(&end); QueryPerformanceFrequency(&freq);
    return freq.QuadPart ? (double)(end.QuadPart-start.QuadPart)*1000000.0 /
                          (double)freq.QuadPart : 0.0;
}
static int pixel_probe_opt_in(const char *s, DWORD n) {
    return n == 1 && s[0] == '1' && s[1] == 0;
}
static void pixel_probe_free(void) {
#if DG_ENABLE_DIAGNOSTICS

    int i;
    for (i=0; i<3; ++i) if (g_pp.stage[i]) {
        g_pp.stage[i]->lpVtbl->Release(g_pp.stage[i]); g_pp.stage[i]=NULL;
    }
    if (g_pp.event) { g_pp.event->lpVtbl->Release(g_pp.event); g_pp.event=NULL; }

#else

#endif
}
/* RGB/A bytes, no gamma conversion. Hash and nonblack describe only the same
 * deterministic <=64x36 grid in each full-size staging texture, NOT all pixels.
 * RowPitch padding is excluded. Coordinate formula includes all four edges. */
static void pixel_probe_stats(DG_PIXEL_STATS *s, const D3D11_MAPPED_SUBRESOURCE *m,
                              unsigned w, unsigned h, unsigned group) {
    unsigned x, y, c, nx=w<64?w:64, ny=h<36?h:36;
    memset(s,0,sizeof(*s)); s->hash=14695981039346656037ULL;
    for(c=0;c<4;++c) s->lo[c]=255;
    for(y=0;y<ny;++y) for(x=0;x<nx;++x) {
        unsigned sx=nx>1?(unsigned)((uint64_t)x*(w-1)/(nx-1)):0;
        unsigned sy=ny>1?(unsigned)((uint64_t)y*(h-1)/(ny-1)):0;
        const unsigned char *p=(const unsigned char *)m->pData+(size_t)sy*m->RowPitch+sx*4;
        unsigned v[4]; v[0]=p[group==1?2:0]; v[1]=p[1];
        v[2]=p[group==1?0:2]; v[3]=p[3];
        ++s->count; if(v[0] || v[1] || v[2]) ++s->nonblack;
        for(c=0;c<4;++c) {
            if(v[c]<s->lo[c]) s->lo[c]=v[c];
            if(v[c]>s->hi[c]) s->hi[c]=v[c];
            s->hash=(s->hash^v[c])*1099511628211ULL;
        }
    }
    s->valid=1;
}
/* Called only for an armed mono capture, inside the producer's capture lock.
 * A later producer overwrite is never silently relabelled as this sample. */
static void pixel_probe_source(ID3D11Texture2D *back, int mono) {
#if DG_ENABLE_DIAGNOSTICS

    D3D11_TEXTURE2D_DESC d; D3D11_QUERY_DESC q; LARGE_INTEGER start; int i;
    if (!g_pp.enabled || g_pp.phase!=PP_ARMED || !mono) return;
    QueryPerformanceCounter(&start);
    back->lpVtbl->GetDesc(back,&d);
    g_pp.width=d.Width; g_pp.height=d.Height; g_pp.format=(unsigned)d.Format;
    g_pp.actual_format[0]=(unsigned)d.Format;
    { D3D11_TEXTURE2D_DESC sd; g_store[0].texture->lpVtbl->GetDesc(g_store[0].texture,&sd);
      g_pp.actual_format[1]=(unsigned)sd.Format; }
    g_pp.capture_id=g_store[0].capture_id; g_pp.capture_ms=g_store[0].capture_ms;
    if (!d.Width || !d.Height || (uint64_t)d.Width*d.Height>16777216ULL ||
        d.MipLevels!=1 || d.ArraySize!=1 || d.SampleDesc.Count!=1 ||
        d.SampleDesc.Quality || !dg_capture_format_group(d.Format)) {
        g_pp.hr=E_INVALIDARG; g_pp.outcome="unsupported-source"; g_pp.phase=PP_DONE;
    } else {
        d.Usage=D3D11_USAGE_STAGING; d.BindFlags=0; d.MiscFlags=0;
        d.CPUAccessFlags=D3D11_CPU_ACCESS_READ;
        for(i=0;i<3;++i) {
            g_pp.hr=g_dev->lpVtbl->CreateTexture2D(g_dev,&d,NULL,&g_pp.stage[i]);
            if(FAILED(g_pp.hr) || !g_pp.stage[i]) break;
        }
        memset(&q,0,sizeof(q)); q.Query=D3D11_QUERY_EVENT;
        if(i==3) g_pp.hr=g_dev->lpVtbl->CreateQuery(g_dev,&q,&g_pp.event);
        if(i!=3 || FAILED(g_pp.hr) || !g_pp.event) {
            g_pp.outcome="allocation-failed"; g_pp.phase=PP_DONE;
        } else {
            g_ctx->lpVtbl->CopyResource(g_ctx,(ID3D11Resource *)g_pp.stage[0],(ID3D11Resource *)back);
            g_ctx->lpVtbl->CopyResource(g_ctx,(ID3D11Resource *)g_pp.stage[1],(ID3D11Resource *)g_store[0].texture);
            g_pp.phase=PP_SOURCE;
        }
    }
    g_pp.producer_us=pixel_probe_us(start);

#else

#endif
}
/* Called after the normal destination copy, still in the snapshot lock. */
static void pixel_probe_destination(ID3D11Texture2D *dst, int eye) {
#if DG_ENABLE_DIAGNOSTICS

    LARGE_INTEGER start; D3D11_TEXTURE2D_DESC d;
    if(!g_pp.enabled || g_pp.phase!=PP_SOURCE || eye!=0) return;
    QueryPerformanceCounter(&start);
    if(g_state!=XR_SESSION_STATE_FOCUSED || !g_cd.screen) {
        g_pp.outcome="submission-gate-changed"; g_pp.phase=PP_DONE; return;
    }
    if(g_pp.capture_id!=g_store[0].capture_id) {
        g_pp.outcome="capture-overwritten"; g_pp.phase=PP_DONE; return;
    }
    g_pp.frame=(uint64_t)InterlockedCompareExchange64(&g_cd_count[CD_FRAME],0,0);
    g_pp.display_time=g_cd.predicted_time; g_pp.submit_ms=GetTickCount64();
    dst->lpVtbl->GetDesc(dst,&d); g_pp.actual_format[2]=(unsigned)d.Format;
    g_ctx->lpVtbl->CopyResource(g_ctx,(ID3D11Resource *)g_pp.stage[2],(ID3D11Resource *)dst);
    g_ctx->lpVtbl->End(g_ctx,(ID3D11Asynchronous *)g_pp.event);
    g_pp.phase=PP_GPU; g_pp.submit_us=pixel_probe_us(start);

#else

#endif
}
/* One query per call, never spin/flush. Caller holds g_cap_cs. */
static void pixel_probe_read(int before) {
    BOOL ready=FALSE; LARGE_INTEGER start; int i;
    if(g_pp.phase!=PP_GPU) return;
    if(g_pp.polls>=128) { g_pp.outcome="poll-limit"; g_pp.phase=PP_DONE; return; }
    ++g_pp.polls;
    QueryPerformanceCounter(&start);
    g_pp.hr=g_ctx->lpVtbl->GetData(g_ctx,(ID3D11Asynchronous *)g_pp.event,
                                  &ready,sizeof(ready),D3D11_ASYNC_GETDATA_DONOTFLUSH);
    if(before) {
        g_pp.before_attempted=1; g_pp.before_hr=g_pp.hr;
        g_pp.before_ready=g_pp.hr==S_OK && ready;
    }
    if(FAILED(g_pp.hr)) { g_pp.outcome="query-failed"; g_pp.phase=PP_DONE; }
    else if(g_pp.hr==S_OK && ready) {
        for(i=0;i<3;++i) {
            D3D11_MAPPED_SUBRESOURCE m;
            if(g_pp.stats[i].valid) continue;
            memset(&m,0,sizeof(m));
            g_pp.map_hr[i]=g_ctx->lpVtbl->Map(g_ctx,(ID3D11Resource *)g_pp.stage[i],0,
                              D3D11_MAP_READ,D3D11_MAP_FLAG_DO_NOT_WAIT,&m);
            if(g_pp.map_hr[i]==DXGI_ERROR_WAS_STILL_DRAWING) break;
            if(FAILED(g_pp.map_hr[i])) { g_pp.outcome="map-failed"; g_pp.phase=PP_DONE; break; }
            pixel_probe_stats(&g_pp.stats[i],&m,g_pp.width,g_pp.height,
                              (unsigned)dg_capture_format_group((DXGI_FORMAT)g_pp.format));
            g_ctx->lpVtbl->Unmap(g_ctx,(ID3D11Resource *)g_pp.stage[i],0);
        }
        if(i==3) {
            g_pp.read_before=before; g_pp.read_ms=GetTickCount64();
            g_pp.outcome="read-complete"; g_pp.phase=PP_DONE;
        }
    }
    g_pp.read_us+=pixel_probe_us(start);
}
static void pixel_probe_before_release(void) {
#if DG_ENABLE_DIAGNOSTICS

    if(!g_pp.enabled) return;
    EnterCriticalSection(&g_cap_cs);
    /* Never relabel a previous frame's pending staging read as pre-release
     * merely because another XR frame is now about to release its image. */
    if(!g_pp.before_attempted && g_pp.submit_ms &&
       g_pp.frame==(uint64_t)InterlockedCompareExchange64(&g_cd_count[CD_FRAME],0,0))
        pixel_probe_read(1);
    LeaveCriticalSection(&g_cap_cs);

#else

#endif
}
/* XR thread only; retain the ACTUAL built quad and a head location at the same
 * predicted time. This happens before EndFrame; no anchor is changed. */
static void pixel_probe_quad(const XrCompositionLayerQuad *quad) {
#if DG_ENABLE_DIAGNOSTICS

    LARGE_INTEGER start; int measure=0;
    XrTime time=0; XrSpaceLocation head; XrResult result;
    if(!g_pp.enabled) return;
    EnterCriticalSection(&g_cap_cs);
    if(g_pp.submit_ms && g_pp.frame==(uint64_t)InterlockedCompareExchange64(&g_cd_count[CD_FRAME],0,0)
        && !g_pp.quad_valid) {
        g_pp.quad=*quad; g_pp.quad_valid=1; time=g_pp.display_time; measure=1;
    }
    LeaveCriticalSection(&g_cap_cs);
    if(measure) {
        /* Do not hold up Present's capture lock across a runtime API call. */
        QueryPerformanceCounter(&start);
        memset(&head,0,sizeof(head)); head.type=XR_TYPE_SPACE_LOCATION;
        result=pfn_LocateSpace(g_view_space,quad->space,time,&head);
        EnterCriticalSection(&g_cap_cs);
        g_pp.head=head; g_pp.head_result=result; g_pp.pose_us=pixel_probe_us(start);
        LeaveCriticalSection(&g_cap_cs);
    }

#else

#endif
}
static void pixel_probe_emit(void) {
    int i;
    g_log("  xr pixel-probe: sample %u outcome %s intervention 1 grid-only 1 capture %llu frame %llu"
          " arm/capture/submit/read-ms %llu/%llu/%llu/%llu predicted %lld"
          " before-query %d:0x%08lX ready %d read-phase %s hr 0x%08lX"
          " release/end %d:%d/%d:%d cpu-us producer/copy/read/pose %.0f/%.0f/%.0f/%.0f"
          " staging-bytes %llu\r\n",
          g_pp.attempts,g_pp.outcome?g_pp.outcome:"none",
          (unsigned long long)g_pp.capture_id,(unsigned long long)g_pp.frame,
          (unsigned long long)g_pp.arm_ms,(unsigned long long)g_pp.capture_ms,
          (unsigned long long)g_pp.submit_ms,(unsigned long long)g_pp.read_ms,(long long)g_pp.display_time,
          g_pp.before_attempted,(unsigned long)g_pp.before_hr,g_pp.before_ready,
          !g_pp.read_ms?"unavailable":g_pp.read_before?"before-release":"after-release",
          (unsigned long)g_pp.hr,g_pp.release_attempted,(int)g_pp.release_result,
          g_pp.end_attempted,(int)g_pp.end_result,g_pp.producer_us,g_pp.submit_us,g_pp.read_us,g_pp.pose_us,
          (unsigned long long)g_pp.width*g_pp.height*12);
    for(i=0;i<3;++i) {
        const DG_PIXEL_STATS *s=&g_pp.stats[i];
        g_log("  xr pixel-probe pixels: sample %u part %s valid %d map 0x%08lX size %ux%u fmt %u"
              " stage-fmt %u grid %u nonblack %u rgba-min %u/%u/%u/%u max %u/%u/%u/%u fnv64 %016llX\r\n",
              g_pp.attempts,i==0?"back":i==1?"store":"dst",s->valid,(unsigned long)g_pp.map_hr[i],
              g_pp.width,g_pp.height,g_pp.actual_format[i],g_pp.format,s->count,s->nonblack,
              s->lo[0],s->lo[1],s->lo[2],s->lo[3],s->hi[0],s->hi[1],s->hi[2],s->hi[3],
              (unsigned long long)s->hash);
    }
    if(g_pp.quad_valid) {
        DG_XR_RAW_POSE head, qp; DG_XR_REL_POSE rel; XrPosef *h=&g_pp.head.pose,*q=&g_pp.quad.pose;
        const XrSpaceLocationFlags need=XR_SPACE_LOCATION_POSITION_VALID_BIT|XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;
        int valid=XR_SUCCEEDED(g_pp.head_result) && (g_pp.head.locationFlags&need)==need;
        double distance=0, forward=0, facing=0;
        memset(&rel,0,sizeof(rel));
        head.qx=h->orientation.x; head.qy=h->orientation.y; head.qz=h->orientation.z; head.qw=h->orientation.w;
        head.px=h->position.x; head.py=h->position.y; head.pz=h->position.z;
        qp.qx=q->orientation.x; qp.qy=q->orientation.y; qp.qz=q->orientation.z; qp.qw=q->orientation.w;
        qp.px=q->position.x; qp.py=q->position.y; qp.pz=q->position.z;
        if(valid) {
            dg_xr_pose_relative(&head,&qp,&rel);
            distance=sqrt(rel.px*rel.px+rel.py*rel.py+rel.pz*rel.pz);
            if(distance>0) {
                double nx=2*(rel.qx*rel.qz+rel.qw*rel.qy);
                double ny=2*(rel.qy*rel.qz-rel.qw*rel.qx);
                double nz=1-2*(rel.qx*rel.qx+rel.qy*rel.qy);
                forward=-rel.pz/distance;
                facing=-(nx*rel.px+ny*rel.py+nz*rel.pz)/distance;
            }
        }
        g_log("  xr pixel-probe quad: sample %u head-valid %d locate %d flags %llu space %p view %p"
              " eye %d rect %d/%d/%d/%d array %u size %.3f/%.3f"
              " head-p %.4f/%.4f/%.4f head-q %.5f/%.5f/%.5f/%.5f"
              " quad-p %.4f/%.4f/%.4f quad-q %.5f/%.5f/%.5f/%.5f"
              " relative-p %.4f/%.4f/%.4f relative-q %.5f/%.5f/%.5f/%.5f"
              " distance/head-forward-cos/quad-facing-cos %.4f/%.4f/%.4f\r\n",
              g_pp.attempts,valid,(int)g_pp.head_result,(unsigned long long)g_pp.head.locationFlags,
              (void *)g_pp.quad.space,(void *)g_view_space,(int)g_pp.quad.eyeVisibility,
              g_pp.quad.subImage.imageRect.offset.x,g_pp.quad.subImage.imageRect.offset.y,
              g_pp.quad.subImage.imageRect.extent.width,g_pp.quad.subImage.imageRect.extent.height,
              g_pp.quad.subImage.imageArrayIndex,g_pp.quad.size.width,g_pp.quad.size.height,
              head.px,head.py,head.pz,head.qx,head.qy,head.qz,head.qw,
              qp.px,qp.py,qp.pz,qp.qx,qp.qy,qp.qz,qp.qw,rel.px,rel.py,rel.pz,
              rel.qx,rel.qy,rel.qz,rel.qw,distance,forward,facing);
    }
}
/* EndFrame result is attached before servicing or arming the next sample. */
static void pixel_probe_tick(int arm, int end_attempted, XrResult end_result) {
#if DG_ENABLE_DIAGNOSTICS

    ULONGLONG now; int emit=0;
    if(!g_pp.enabled) return;
    EnterCriticalSection(&g_cap_cs); now=GetTickCount64();
    if(g_pp.submit_ms && g_pp.frame==(uint64_t)InterlockedCompareExchange64(&g_cd_count[CD_FRAME],0,0)
        && end_attempted) {
        g_pp.release_attempted=g_cd.attempted_release[0]; g_pp.release_result=g_cd.release[0];
        g_pp.end_attempted=1; g_pp.end_result=end_result;
    }
    pixel_probe_read(0);
    if(g_pp.phase!=PP_IDLE && g_pp.phase!=PP_DONE && now-g_pp.arm_ms>=2000) {
        g_pp.outcome="timeout"; g_pp.phase=PP_DONE;
    }
    if(g_pp.phase==PP_DONE) { pixel_probe_free(); emit=1; }
    LeaveCriticalSection(&g_cap_cs);
    /* Producer cannot touch DONE. Never log while holding the capture lock. */
    if(emit) pixel_probe_emit();
    EnterCriticalSection(&g_cap_cs);
    if(emit) { g_pp.phase=PP_IDLE; g_pp.next_ms=GetTickCount64()+2000; }
    if(arm && g_pp.phase==PP_IDLE && g_pp.attempts<3 && now>=g_pp.next_ms) {
        unsigned n=g_pp.attempts+1;
        memset(&g_pp,0,sizeof(g_pp)); g_pp.enabled=1; g_pp.attempts=n;
        g_pp.phase=PP_ARMED; g_pp.arm_ms=now;
    }
    LeaveCriticalSection(&g_cap_cs);

#else

#endif
}
static void pixel_probe_teardown(void) {
#if DG_ENABLE_DIAGNOSTICS

    if(!g_pp.enabled) return;
    EnterCriticalSection(&g_cap_cs);
    if(g_pp.submit_ms && g_pp.frame==(uint64_t)InterlockedCompareExchange64(&g_cd_count[CD_FRAME],0,0)) {
        g_pp.release_attempted=g_cd.attempted_release[0]; g_pp.release_result=g_cd.release[0];
        g_pp.end_attempted=g_cd.end_attempted; g_pp.end_result=g_cd.end;
    }
    if(g_pp.phase!=PP_IDLE && g_pp.phase!=PP_DONE) {
        g_pp.outcome="session-teardown"; g_pp.phase=PP_DONE;
    }
    LeaveCriticalSection(&g_cap_cs);
    pixel_probe_tick(0,0,XR_SUCCESS);

#else

#endif
}
static void pixel_probe_init(void) {
#if DG_ENABLE_DIAGNOSTICS

    char value[8]={0}; DWORD n;
    if(g_pp_initialized) return;
    g_pp_initialized=1;
    n=GetEnvironmentVariableA("DG_XR_PIXEL_PROBE",value,sizeof(value));
    /* start owns initialization before launching XR; capture also uses this lock. */
    EnterCriticalSection(&g_cap_cs);
    pixel_probe_free(); memset(&g_pp,0,sizeof(g_pp));
    g_pp.enabled=pixel_probe_opt_in(value,n);
    LeaveCriticalSection(&g_cap_cs);
    if(g_pp.enabled) g_log("  xr pixel-probe enabled: max 3 attempts, 2 s spacing/timeout,"
        " focused mono quad only; staging intervention, nonflushing query/nonblocking Map;"
        " <=64x36 grid stats are not a full-image checksum or headset proof\r\n");

#else

#endif
}
