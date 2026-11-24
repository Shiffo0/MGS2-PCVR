/* Diagnostic-only F10 snapshot after five seconds, at most four requests per process.
   Freeze both inputs and decisions under g_cap_cs. Private GPU staging, event
   query + DO_NOT_WAIT readback on later Presents; CPU-only writer thread.
   Never flush/wait for the GPU, modify eye textures, or change composition. */
#define OC_LIMIT 4
#define OC_DELAY_MS 5000
#define OC_MAX_BYTES (128u*1024u*1024u)
typedef struct {
    DG_CAPTURE_STORE eye[2];
    DG_OBSERVER_LAYOUT layout;
    unsigned width,height,format,request;
    int requested_mode,effective_mode,pose_pair,valid[2];
    uint64_t ms,delta,trigger_ms,target_ms;
    unsigned char *pixels[3];
    char root[MAX_PATH];
} OC_JOB;
static struct {
    ID3D11Texture2D *stage[3]; ID3D11Query *query;
    OC_JOB *job; HANDLE writer;
    unsigned requests; int key,armed;
    uint64_t pending_ms,trigger_ms,target_ms;
} g_oc;
#ifdef DG_OBSERVER_CAPTURE_TEST
static int oc_test_key;
static unsigned oc_test_delay_ms=20;
#endif
static void oc_number(FILE *f,double x) {if(isfinite(x))fprintf(f,"%.17g",x);else fputs("null",f);}
static void oc_values(FILE *f,const double *v,int n) {
    int i;fputc('[',f);for(i=0;i<n;i++){if(i)fputc(',',f);oc_number(f,v[i]);}fputc(']',f);
}
static void oc_floats(FILE *f,const float *v,int n) {
    int i;fputc('[',f);for(i=0;i<n;i++){if(i)fputc(',',f);oc_number(f,v[i]);}fputc(']',f);
}
static void oc_arm(FILE *f,const DG_MODEL_ARM *a) {
    fprintf(f,"{\"valid\":%d,\"ms\":%llu,\"sequence\":%llu,\"stream\":%lu,\"pair\":%lu,\"elbow\":",a->valid,a->ms,a->sequence,a->stream,a->pair);
    oc_values(f,a->elbow,3);fputs(",\"wrist\":",f);oc_values(f,a->wrist,3);fputc('}',f);
}
static void oc_job_free(OC_JOB *j) {int i;if(!j)return;for(i=0;i<3;i++)free(j->pixels[i]);free(j);}
static int oc_ppm(const char *path,const OC_JOB *j,int image) {
    FILE *f=NULL;unsigned x,y;unsigned char *row;int ok=1;
    row=(unsigned char*)malloc((size_t)j->width*3);if(!row)return 0;
    if(fopen_s(&f,path,"wb")||!f){free(row);return 0;}
    fprintf(f,"P6\n%u %u\n255\n",j->width,j->height);
    for(y=0;y<j->height;y++) {
        const unsigned char *p=j->pixels[image]+(size_t)y*j->width*4;
        for(x=0;x<j->width;x++,p+=4) {
            if(j->format==DXGI_FORMAT_R10G10B10A2_UNORM) {
                unsigned v;memcpy(&v,p,4);
                row[x*3]=(unsigned char)(((v&1023)*255+511)/1023);
                row[x*3+1]=(unsigned char)((((v>>10)&1023)*255+511)/1023);
                row[x*3+2]=(unsigned char)((((v>>20)&1023)*255+511)/1023);
            }else {
                int bgra=j->format==DXGI_FORMAT_B8G8R8A8_UNORM||j->format==DXGI_FORMAT_B8G8R8A8_UNORM_SRGB;
                row[x*3]=p[bgra?2:0];row[x*3+1]=p[1];row[x*3+2]=p[bgra?0:2];
            }
        }
        if(fwrite(row,3,j->width,f)!=j->width){ok=0;break;}
    }
    if(fclose(f))ok=0;free(row);return ok;
}
static DWORD WINAPI oc_write(void *arg) {
    OC_JOB *j=(OC_JOB*)arg;char dir[MAX_PATH],path[MAX_PATH];FILE *f=NULL;int i,ok=1;
    const char *names[3]={"left.ppm","right.ppm","desktop.ppm"};
    if(_snprintf_s(dir,sizeof dir,_TRUNCATE,"%s\\observer_capture_%lu_%llu_%u",j->root,GetCurrentProcessId(),j->ms,j->request)<0)goto fail;
    if(!CreateDirectoryA(j->root,NULL)&&GetLastError()!=ERROR_ALREADY_EXISTS)goto fail;

    if(!CreateDirectoryA(dir,NULL))goto fail;
    for(i=0;i<3;i++) {
        if(_snprintf_s(path,sizeof path,_TRUNCATE,"%s\\%s",dir,names[i])<0||!oc_ppm(path,j,i))goto fail;
        /* Lossless native storage allows exact 10-bit and sRGB reconstruction. */
        if(_snprintf_s(path,sizeof path,_TRUNCATE,"%s\\%s.bin",dir,names[i])<0||fopen_s(&f,path,"wb")||!f)goto fail;
        ok=fwrite(j->pixels[i],4,(size_t)j->width*j->height,f)==(size_t)j->width*j->height;
        if(fclose(f))ok=0;f=NULL;if(!ok)goto fail;
    }
    if(_snprintf_s(path,sizeof path,_TRUNCATE,"%s\\metadata.json",dir)<0||fopen_s(&f,path,"wb")||!f)goto fail;
    fputs("{\"schema\":1,\"capture_context\":",f);
#ifdef DG_OBSERVER_CAPTURE_TEST
    fputs("\"synthetic_WARP_test\",",f);
#else
    fputs("\"runtime_observer\",",f);
#endif
    fprintf(f,"\"width\":%u,\"height\":%u,\"format\":%u,\"requested_mode\":%d,\"effective_mode\":%d,\"trigger_ms\":%llu,\"target_ms\":%llu,\"snapshot_ms\":%llu,\"delta_ms\":%llu,\"pose_pair\":%d,\"producer_identity_note\":\"Present labels and native metadata are producer-supplied, not proof of the consumed draw generation or simultaneous simulation\",\"eye_bounds\":[",j->width,j->height,j->format,j->requested_mode,j->effective_mode,j->trigger_ms,j->target_ms,j->ms,j->delta,j->pose_pair);
    oc_floats(f,j->layout.eye[0],4);fputc(',',f);oc_floats(f,j->layout.eye[1],4);
    fputs("],\"crop\":",f);oc_floats(f,j->layout.crop,4);
    fputs(",\"options\":",f);oc_floats(f,j->layout.options,4);
    fputs(",\"arm_bounds\":[",f);oc_floats(f,j->layout.arm[0],4);fputc(',',f);oc_floats(f,j->layout.arm[1],4);
    fputs("],\"arm_mode\":",f);oc_floats(f,j->layout.arm_mode,4);fputs(",\"eyes\":[",f);
    for(i=0;i<2;i++) {
        DG_CAPTURE_STORE *s=&j->eye[i];double p[7]={s->raw.px,s->raw.py,s->raw.pz,s->raw.qx,s->raw.qy,s->raw.qz,s->raw.qw};
        double fv[4]={s->fov.left,s->fov.right,s->fov.down,s->fov.up};
        if(i)fputc(',',f);
        fprintf(f,"{\"valid\":%d,\"capture_id\":%llu,\"capture_ms\":%llu,\"producer_kind\":%d,\"producer_present\":%u,\"native_meta_valid\":%d,\"native_meta_present\":%llu,\"native_meta_qpc\":%llu,\"pose\":",j->valid[i],s->capture_id,s->capture_ms,s->observer_kind,s->observer_present,s->observer_meta_valid,s->observer_meta_present,s->observer_meta_qpc);
        oc_values(f,p,7);fputs(",\"fov_radians\":",f);oc_values(f,fv,4);
        fputs(",\"left_arm\":",f);oc_arm(f,&s->model_arm);fputs(",\"right_arm\":",f);oc_arm(f,&s->right_model_arm);fputc('}',f);
    }
    fputs("]}\n",f);ok=!ferror(f);if(fclose(f))ok=0;f=NULL;
    if(!ok)goto fail;
    if(g_log)g_log("  observer capture complete: %s\r\n",dir);
    oc_job_free(j);return 1;
fail:
    if(f)fclose(f);if(g_log)g_log("  observer capture FAILED: incomplete bundle, do not use as evidence\r\n");
    oc_job_free(j);return 0;
}
static void oc_gpu_free(void) {
    int i;for(i=0;i<3;i++)OB_RELEASE(g_oc.stage[i]);OB_RELEASE(g_oc.query);
}
static void oc_cancel(void) {oc_gpu_free();oc_job_free(g_oc.job);g_oc.job=NULL;}
static void oc_free(void) {
    g_oc.armed=0;
    oc_cancel();
    /* CPU writer never touches graphics or g_cap_cs; shutdown joins before unload. */
    if(g_oc.writer){WaitForSingleObject(g_oc.writer,INFINITE);CloseHandle(g_oc.writer);g_oc.writer=NULL;}
}
static void oc_poll(void) {
    BOOL ready=FALSE;HRESULT hr;D3D11_MAPPED_SUBRESOURCE map[3];int i,mapped=0;
    if(g_oc.writer&&WaitForSingleObject(g_oc.writer,0)==WAIT_OBJECT_0){CloseHandle(g_oc.writer);g_oc.writer=NULL;}
    if(!g_oc.job||!g_oc.query)return;
    if(GetTickCount64()-g_oc.pending_ms>2000){if(g_log)g_log("  observer capture refused: GPU readback timeout\r\n");oc_cancel();return;}
    hr=g_ctx->lpVtbl->GetData(g_ctx,(ID3D11Asynchronous*)g_oc.query,&ready,sizeof ready,D3D11_ASYNC_GETDATA_DONOTFLUSH);
    if(FAILED(hr))goto fail;
    if(hr==S_FALSE||!ready)return;
    for(i=0;i<3;i++) {
        hr=g_ctx->lpVtbl->Map(g_ctx,(ID3D11Resource*)g_oc.stage[i],0,D3D11_MAP_READ,D3D11_MAP_FLAG_DO_NOT_WAIT,&map[i]);
        if(FAILED(hr))break;mapped++;
    }
    if(mapped==3) {
        unsigned y;for(i=0;i<3;i++)for(y=0;y<g_oc.job->height;y++)
            memcpy(g_oc.job->pixels[i]+(size_t)y*g_oc.job->width*4,(char*)map[i].pData+(size_t)y*map[i].RowPitch,(size_t)g_oc.job->width*4);
    }
    for(i=0;i<mapped;i++)g_ctx->lpVtbl->Unmap(g_ctx,(ID3D11Resource*)g_oc.stage[i],0);
    if(mapped!=3){if(hr==DXGI_ERROR_WAS_STILL_DRAWING)return;goto fail;}
    g_oc.writer=CreateThread(NULL,0,oc_write,g_oc.job,0,NULL);
    if(!g_oc.writer)goto fail;
    g_oc.job=NULL;oc_gpu_free();return;
fail:
    if(g_log)g_log("  observer capture refused: readback/writer failure\r\n");oc_cancel();
}
static int oc_request(void) {
    int down,rising;uint64_t now;unsigned delay=OC_DELAY_MS;
#ifdef DG_OBSERVER_CAPTURE_TEST
    down=oc_test_key;
    delay=oc_test_delay_ms;
#else
    down=0;
#endif
    rising=down&&!g_oc.key;g_oc.key=down;
    now=GetTickCount64();
    if(rising) {
        if(g_oc.requests>=OC_LIMIT||g_oc.armed||g_oc.job||g_oc.writer) {
            if(g_log)g_log("  observer capture refused: busy or four-request limit\r\n");return 0;
        }
        g_oc.requests++;g_oc.trigger_ms=now;g_oc.target_ms=now+delay;g_oc.armed=1;
        if(g_log)g_log("  observer capture armed: request %u, delay %u ms; return hands to pose and hold\r\n",g_oc.requests,delay);
    }
    if(!g_oc.armed||now<g_oc.target_ms)return 0;
    g_oc.armed=0;
    /* Do not silently take an old request after theatre/menu/observer downtime. */
    if(now-g_oc.target_ms>1000) {
        if(g_log)g_log("  observer capture refused: delayed request expired\r\n");return 0;
    }
    return 1;
}
static int oc_begin(const D3D11_TEXTURE2D_DESC *td,const DG_OBSERVER_LAYOUT *layout,int requested,int effective,const int valid[2],uint64_t now) {
    D3D11_TEXTURE2D_DESC d=*td;D3D11_QUERY_DESC q={D3D11_QUERY_EVENT,0};OC_JOB *j;size_t bytes;int i;char *slash;
    if(g_oc.job||g_oc.writer||!valid[0]||!valid[1])goto refused;
    if(td->SampleDesc.Count!=1||td->ArraySize!=1||td->MipLevels!=1||!td->Width||!td->Height||td->Width>16384||td->Height>16384)goto refused;
    if(td->Format!=DXGI_FORMAT_B8G8R8A8_UNORM&&td->Format!=DXGI_FORMAT_B8G8R8A8_UNORM_SRGB&&td->Format!=DXGI_FORMAT_R8G8B8A8_UNORM&&td->Format!=DXGI_FORMAT_R8G8B8A8_UNORM_SRGB&&td->Format!=DXGI_FORMAT_R10G10B10A2_UNORM)goto refused;
    bytes=(size_t)td->Width*td->Height*4;if(bytes>OC_MAX_BYTES/3)goto refused;
    j=(OC_JOB*)calloc(1,sizeof *j);if(!j)goto refused;g_oc.job=j;
    j->width=td->Width;j->height=td->Height;j->format=td->Format;j->request=g_oc.requests;j->layout=*layout;
    j->requested_mode=requested;j->effective_mode=effective;j->ms=now;
    j->trigger_ms=g_oc.trigger_ms;j->target_ms=g_oc.target_ms;
    j->delta=g_store[0].capture_ms>g_store[1].capture_ms?g_store[0].capture_ms-g_store[1].capture_ms:g_store[1].capture_ms-g_store[0].capture_ms;
    j->pose_pair=observer_pose_pair(&g_store[0].raw,&g_store[1].raw);
    for(i=0;i<2;i++){j->eye[i]=g_store[i];j->eye[i].texture=NULL;j->valid[i]=valid[i];}
    {DWORD n=GetModuleFileNameA(NULL,j->root,sizeof j->root);if(!n||n>=sizeof j->root)goto failed;}
    slash=strrchr(j->root,'\\');if(!slash)goto failed;*slash=0;
    if(strcat_s(j->root,sizeof j->root,"\\logs"))goto failed;
    d.Usage=D3D11_USAGE_STAGING;d.BindFlags=d.MiscFlags=0;d.CPUAccessFlags=D3D11_CPU_ACCESS_READ;
    for(i=0;i<3;i++) {
        j->pixels[i]=(unsigned char*)malloc(bytes);if(!j->pixels[i])goto failed;
        if(FAILED(g_dev->lpVtbl->CreateTexture2D(g_dev,&d,NULL,&g_oc.stage[i])))goto failed;
    }
    if(FAILED(g_dev->lpVtbl->CreateQuery(g_dev,&q,&g_oc.query)))goto failed;
    for(i=0;i<2;i++)g_ctx->lpVtbl->CopyResource(g_ctx,(ID3D11Resource*)g_oc.stage[i],(ID3D11Resource*)g_store[i].texture);
    g_oc.pending_ms=GetTickCount64();return 1;
failed:
    oc_cancel();
refused:
    if(g_log)g_log("  observer capture refused: invalid pair, format, size or allocation\r\n");return 0;
}
static void oc_end(ID3D11Texture2D *back,int rendered) {
    if(!g_oc.job)return;
    if(!rendered){oc_cancel();return;}
    g_ctx->lpVtbl->CopyResource(g_ctx,(ID3D11Resource*)g_oc.stage[2],(ID3D11Resource*)back);
    g_ctx->lpVtbl->End(g_ctx,(ID3D11Asynchronous*)g_oc.query);
}
