

#ifndef NP_ROOT
#define NP_ROOT "logs\\pcvr_near"
#endif
#include <float.h>
#pragma warning(push)
#pragma warning(disable:4996)
static struct {
    unsigned token, last_token, mask, polls;
    ULONGLONG next_poll, start_ms;
    ID3D11Texture2D *texture[2]; ID3D11Query *query;
    D3D11_TEXTURE2D_DESC desc[2];
    DG_NEAR_META meta[2]; DG_XR_RAW_POSE raw[2]; DG_PROJ_FOV fov[2];
    uint64_t capture_id[2], qpc[2];
} g_np;
static void near_probe_free(void) {
#if DG_ENABLE_DIAGNOSTICS

    int i;for(i=0;i<2;i++)if(g_np.texture[i]){g_np.texture[i]->lpVtbl->Release(g_np.texture[i]);g_np.texture[i]=NULL;}
    if(g_np.query){g_np.query->lpVtbl->Release(g_np.query);g_np.query=NULL;}
    g_np.mask=0;g_np.token=0;g_np.polls=0;

#else

#endif
}
static void near_numbers(FILE *f,const double *a,int n){int i;fputc('[',f);for(i=0;i<n;i++){if(i)fputc(',',f);if(_finite(a[i]))fprintf(f,"%.17g",a[i]);else fputs("null",f);}fputc(']',f);}
static void near_pose(FILE *f,const DG_XR_RAW_POSE *p){double q[4]={p->qx,p->qy,p->qz,p->qw},v[3]={p->px,p->py,p->pz};fputs("{\"q\":",f);near_numbers(f,q,4);fputs(",\"p\":",f);near_numbers(f,v,3);fputc('}',f);}
static void near_mat(FILE *f,const MAT *p){double a[16];int i;for(i=0;i<16;i++)a[i]=((const float*)p)[i];near_numbers(f,a,16);}
static void near_hand(FILE *f,const DG_XR_HAND_POSE *p){fprintf(f,"{\"active\":%u,\"tracked\":%u,\"position_valid\":%u,\"orientation_valid\":%u,\"age_ms\":%u,\"sample_seq\":%llu,\"xr_time\":%lld,\"pose\":",p->active,p->tracked,p->position_valid,p->orientation_valid,p->pose_age_ms,p->sample_seq,p->xr_time);near_pose(f,&p->raw_local);fputc('}',f);}
static int near_bmp(const char *path, const D3D11_MAPPED_SUBRESOURCE *m, unsigned w,unsigned h){
    BITMAPFILEHEADER b={0};BITMAPINFOHEADER d={0};FILE *f;unsigned y;int ok=1;
    b.bfType=0x4d42;b.bfOffBits=sizeof(b)+sizeof(d);b.bfSize=b.bfOffBits+w*h*4;
    d.biSize=sizeof(d);d.biWidth=(LONG)w;d.biHeight=-(LONG)h;d.biPlanes=1;d.biBitCount=32;d.biSizeImage=w*h*4;
    f=fopen(path,"wbx");if(!f)return 0;
    if(fwrite(&b,1,sizeof(b),f)!=sizeof(b)||fwrite(&d,1,sizeof(d),f)!=sizeof(d))ok=0;
    for(y=0;ok&&y<h;y++)if(fwrite((const char*)m->pData+(size_t)y*m->RowPitch,4,w,f)!=w)ok=0;
    if(fclose(f))ok=0;return ok;
}
static void near_finish(void){
    BOOL ready=FALSE;HRESULT hr;D3D11_MAPPED_SUBRESOURCE m[2];int i,n=0,ok=1;char dir[MAX_PATH],path[MAX_PATH];FILE *f;LARGE_INTEGER freq;
    if(++g_np.polls>240||GetTickCount64()-g_np.start_ms>5000){g_log("near probe: timeout, capture invalid\r\n");near_probe_free();return;}
    hr=g_ctx->lpVtbl->GetData(g_ctx,(ID3D11Asynchronous*)g_np.query,&ready,sizeof(ready),D3D11_ASYNC_GETDATA_DONOTFLUSH);
    if(hr==S_FALSE)return;if(FAILED(hr)||!ready){near_probe_free();return;}
    for(i=0;i<2;i++){
        hr=g_ctx->lpVtbl->Map(g_ctx,(ID3D11Resource*)g_np.texture[i],0,D3D11_MAP_READ,D3D11_MAP_FLAG_DO_NOT_WAIT,&m[i]);
        if(FAILED(hr))break;n++;
    }
    if(n!=2){for(i=0;i<n;i++)g_ctx->lpVtbl->Unmap(g_ctx,(ID3D11Resource*)g_np.texture[i],0);if(hr!=DXGI_ERROR_WAS_STILL_DRAWING)near_probe_free();return;}
    sprintf_s(dir,sizeof(dir),NP_ROOT "\\sample_%03u",g_np.token);
    if(!CreateDirectoryA(dir,NULL))ok=0; /* Never overwrite a previous sample. */
    for(i=0;i<2;i++){
        sprintf_s(path,sizeof(path),"%s\\%s.bmp",dir,i?"right":"left");
        if(ok&&!near_bmp(path,&m[i],g_np.desc[i].Width,g_np.desc[i].Height))ok=0;
        g_ctx->lpVtbl->Unmap(g_ctx,(ID3D11Resource*)g_np.texture[i],0);
    }
    sprintf_s(path,sizeof(path),"%s\\capture.json",dir);
    f=ok?fopen(path,"wx"):NULL;
    if(f){
        QueryPerformanceFrequency(&freq);
        fprintf(f,"{\"format\":\"DG_NEAR1\",\"sample\":%u,\"qpf\":%lld,\"same_simulation\":false,\"weapon_model_transform_known\":false,\"final_xr_display_proven\":false,\"eyes\":[",g_np.token,freq.QuadPart);
        for(i=0;i<2;i++){
            double fv[4]={g_np.fov[i].left,g_np.fov[i].right,g_np.fov[i].up,g_np.fov[i].down};
            if(i)fputc(',',f);
            fprintf(f,"{\"eye\":%d,\"image\":\"%s.bmp\",\"width\":%u,\"height\":%u,\"dxgi_format\":%u,\"capture_id\":%llu,\"capture_qpc\":%llu,\"camera_qpc\":%llu,\"present_id\":%u,\"raw_eye\":",i,i?"right":"left",g_np.desc[i].Width,g_np.desc[i].Height,(unsigned)g_np.desc[i].Format,g_np.capture_id[i],g_np.qpc[i],g_np.meta[i].camera_qpc,g_np.meta[i].present_id);
            near_pose(f,&g_np.raw[i]);fputs(",\"fov_lrud\":",f);near_numbers(f,fv,4);
            fputs(",\"head\":",f);near_pose(f,&g_np.meta[i].head_raw);
            fputs(",\"grip\":",f);near_hand(f,&g_np.meta[i].grip);
            fputs(",\"aim\":",f);near_hand(f,&g_np.meta[i].aim);
            fputs(",\"camera_world\":",f);near_mat(f,&g_np.meta[i].camera_world);
            fputs(",\"projection\":",f);near_mat(f,&g_np.meta[i].projection);fputc('}',f);
        }fputs("]}\n",f);if(ferror(f))ok=0;if(fclose(f))ok=0;
    }else ok=0;
    g_log("near probe: sample %u %s -> %s\r\n",g_np.token,ok?"saved":"FAILED",dir);near_probe_free();
}
static void near_probe_capture(ID3D11Texture2D *back,int eye,int mono,const DG_XR_RAW_POSE *raw,const DG_PROJ_FOV *fov,const DG_NEAR_META *meta,uint64_t id){
#if DG_ENABLE_DIAGNOSTICS

    ULONGLONG now=GetTickCount64();FILE *f;unsigned token=0;char extra;HRESULT hr;D3D11_TEXTURE2D_DESC d;D3D11_QUERY_DESC q={D3D11_QUERY_EVENT,0};LARGE_INTEGER pc;
    if(g_np.token&&(mono||g_state!=XR_SESSION_STATE_FOCUSED||!meta||!meta->valid||!raw)){near_probe_free();return;}
    if(g_np.mask==3){near_finish();return;}
    if(mono||!meta||!meta->valid||!raw||g_state!=XR_SESSION_STATE_FOCUSED)return;
    if(g_np.token&&now-g_np.start_ms>5000){g_log("near probe: second eye timeout\r\n");near_probe_free();return;}
    if(!g_np.token){
        if(now<g_np.next_poll)return;g_np.next_poll=now+250;
        f=fopen(NP_ROOT "\\request.txt","r");if(!f)return;
        if(fscanf(f,"%u %c",&token,&extra)!=1)token=0;fclose(f);
        if(token<1||token>12||token<=g_np.last_token)return;
        g_np.last_token=token;g_np.token=token;g_np.start_ms=now;
    }
    if(g_np.mask&(1u<<eye))return;
    back->lpVtbl->GetDesc(back,&d);
    if(d.SampleDesc.Count!=1||d.ArraySize!=1||d.MipLevels!=1||!d.Width||!d.Height||d.Width>8192||d.Height>8192||
       (d.Format!=DXGI_FORMAT_B8G8R8A8_UNORM&&d.Format!=DXGI_FORMAT_B8G8R8A8_UNORM_SRGB)){g_log("near probe: unsupported texture\r\n");near_probe_free();return;}
    g_np.desc[eye]=d;d.Usage=D3D11_USAGE_STAGING;d.BindFlags=0;d.CPUAccessFlags=D3D11_CPU_ACCESS_READ;d.MiscFlags=0;
    hr=g_dev->lpVtbl->CreateTexture2D(g_dev,&d,NULL,&g_np.texture[eye]);
    if(FAILED(hr)){near_probe_free();return;}
    g_ctx->lpVtbl->CopyResource(g_ctx,(ID3D11Resource*)g_np.texture[eye],(ID3D11Resource*)back);
    g_np.meta[eye]=*meta;g_np.raw[eye]=*raw;g_np.fov[eye]=*fov;g_np.capture_id[eye]=id;QueryPerformanceCounter(&pc);g_np.qpc[eye]=(uint64_t)pc.QuadPart;
    g_np.mask|=1u<<eye;
    if(g_np.mask==3){
        if(g_np.desc[0].Width!=g_np.desc[1].Width||g_np.desc[0].Height!=g_np.desc[1].Height||g_np.desc[0].Format!=g_np.desc[1].Format){near_probe_free();return;}
        hr=g_dev->lpVtbl->CreateQuery(g_dev,&q,&g_np.query);if(FAILED(hr)){near_probe_free();return;}
        g_ctx->lpVtbl->End(g_ctx,(ID3D11Asynchronous*)g_np.query);
    }

#else

#endif
}
#pragma warning(pop)
