#pragma once

#include <d3d11_1.h>
#include <wrl/client.h>
#include <math.h>
#include <string.h>
#include "dg_msaa_compare.h"
#include "dg_geometry_probe_cs.h"
#include "dg_second_camera.h"
using Microsoft::WRL::ComPtr;

enum { GEO_STRIDE=34, GEO_BINS=3, GEO_EXTRA=72, GEO_WORDS=GEO_BINS*GEO_STRIDE+GEO_EXTRA, GEO_SAMPLES=4 };
static const float geoNearZ=2000.f, geoFarZ=8000.f; /* distance from the source camera, game units (~mm) */
static const unsigned geoMinIndices=600;
static const unsigned geoMinInliers=256;
static const double geoMaxError=0.5,   /* pixels, per measured axis */
                    geoMinInfo=0.05,   /* share of the gradient along an axis to measure it */
                    geoMinInlierShare=0.5;

/* Row vectors, as the engine: inverse(S), inverse(S)*T, and the source
   camera centre - the world point where the x, y and w columns of S vanish,
   whatever units the engine's projection uses. */
static bool geoMatrices(const float source[16],const float target[16],double sinv[16],double m[16],double camera[3],double minv[16]){
 float identity[16]{};for(int i=0;i<4;i++)identity[i*5]=1;
 if(!dg_second_camera_delta(source,identity,sinv)||!dg_second_camera_delta(source,target,m)||
    !dg_second_camera_delta(target,source,minv))return false;  /* inverse(T)*S = inverse(M) */
 /* sum_i C_i*S[i][j] = -S[3][j] for j = 0,1,3 */
 const int col[3]={0,1,3};double a[3][4];
 for(int r=0;r<3;r++){for(int i=0;i<3;i++)a[r][i]=source[i*4+col[r]];a[r][3]=-source[12+col[r]];}
 for(int j=0;j<3;j++){int q=j;for(int r=j+1;r<3;r++)if(fabs(a[r][j])>fabs(a[q][j]))q=r;
  if(fabs(a[q][j])<1e-20)return false;for(int k=0;k<4;k++){double t=a[j][k];a[j][k]=a[q][k];a[q][k]=t;}
  for(int r=0;r<3;r++)if(r!=j){double f=a[r][j]/a[j][j];for(int k=0;k<4;k++)a[r][k]-=f*a[j][k];}}
 for(int i=0;i<3;i++){camera[i]=a[i][3]/a[i][i];if(!isfinite(camera[i]))return false;}
 return true;
}
struct GeoSample { UINT x=0,y=0;float depth=0,world[4]{},clip[4]{}; };
struct GeoRaw {
 UINT samples=0;GeoSample sample[GEO_SAMPLES];float minDepth=0,maxDepth=0;
 /* Centroids (pixels), both sides restricted to what both eyes can see: where
    the replay drew, where the native draw drew, and where the prediction puts
    the native pixels in the other eye. Independent of the +-4 px window. */
 UINT sameWritten=0,sameEqual=0,contradictions=0,nativeOnly=0; /* same-camera replay against the real frame */
 UINT replayPixels=0,predictedPixels=0;double replayX=0,replayY=0,nativeX=0,nativeY=0,predictedX=0,predictedY=0;
 explicit GeoRaw(const UINT*raw){
  const UINT*e=raw+GEO_BINS*GEO_STRIDE;samples=e[0];
  auto u64=[&](int i){return e[i]|((unsigned long long)e[i+1]<<32);};
  auto pair=[&](int i){return ((double)u64(i)-(double)u64(i+2))/4096.0;};
  replayPixels=e[48];predictedPixels=e[53];sameWritten=e[66];sameEqual=e[67];contradictions=e[68];nativeOnly=e[69];
  if(replayPixels){replayX=(double)u64(49)/replayPixels;replayY=(double)u64(51)/replayPixels;}
  if(predictedPixels){predictedX=pair(54)/predictedPixels;predictedY=pair(58)/predictedPixels;
   nativeX=(double)u64(62)/predictedPixels;nativeY=(double)u64(64)/predictedPixels;}
  for(UINT i=0;i<GEO_SAMPLES&&i<samples;i++){const UINT*r=e+1+i*11;auto&s=sample[i];s.x=r[0];s.y=r[1];
   memcpy(&s.depth,r+2,4);memcpy(s.world,r+3,16);memcpy(s.clip,r+7,16);}
  UINT lo=~e[45];memcpy(&minDepth,&lo,4);memcpy(&maxDepth,e+46,4);
 }
};

struct GeoBin {
 unsigned long long pixels=0,inliers=0,outside=0,rejected=0,flat=0,nonplanar=0,residual=0; /* rejected = replay neighbour unwritten */
 double sxx=0,syy=0,sxy=0,bx=0,by=0,dx=0,dy=0; /* sums, pixels */
 float minZ=INFINITY,maxZ=0;
};
struct GeoTotals {
 GeoBin bin[GEO_BINS];
 static unsigned long long u64(const UINT*r,int i){return r[i]|((unsigned long long)r[i+1]<<32);}
 static double pair(const UINT*r,int i){return ((double)u64(r,i)-(double)u64(r,i+2))/4096.0;}
 void add(const UINT*raw){
  for(int b=0;b<GEO_BINS;b++){
   const UINT*r=raw+b*GEO_STRIDE;GeoBin&g=bin[b];if(!r[0])continue;
   g.pixels+=r[0];g.inliers+=r[1];g.outside+=r[2];g.rejected+=r[3];g.flat+=r[4];g.nonplanar+=r[31];g.residual+=r[32];
   g.sxx+=u64(r,5)/4096.0;g.syy+=u64(r,7)/4096.0;g.sxy+=pair(r,9);g.bx+=pair(r,13);g.by+=pair(r,17);
   g.dx+=pair(r,21);g.dy+=pair(r,25);
   UINT lo=~r[29],hi=r[30];float f;memcpy(&f,&lo,4);if(f<g.minZ)g.minZ=f;memcpy(&f,&hi,4);if(f>g.maxZ)g.maxZ=f;
  }
 }
};

struct GeometryProbe {
 ComPtr<ID3D11ComputeShader> cs;ComPtr<ID3D11Buffer> cb,counters,staging;ComPtr<ID3D11UnorderedAccessView> uav;
 bool init(ID3D11Device*d){
  if(cs)return true;
  if(FAILED(d->CreateComputeShader(g_geometry_probe,sizeof g_geometry_probe,nullptr,&cs)))return false;
  D3D11_BUFFER_DESC b{};b.ByteWidth=240;b.Usage=D3D11_USAGE_DYNAMIC;b.BindFlags=D3D11_BIND_CONSTANT_BUFFER;b.CPUAccessFlags=D3D11_CPU_ACCESS_WRITE;
  if(FAILED(d->CreateBuffer(&b,nullptr,&cb)))return false;
  b={};b.ByteWidth=GEO_WORDS*4;b.BindFlags=D3D11_BIND_UNORDERED_ACCESS;b.MiscFlags=D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS;
  if(FAILED(d->CreateBuffer(&b,nullptr,&counters)))return false;
  D3D11_UNORDERED_ACCESS_VIEW_DESC u{};u.Format=DXGI_FORMAT_R32_TYPELESS;u.ViewDimension=D3D11_UAV_DIMENSION_BUFFER;
  u.Buffer.NumElements=GEO_WORDS;u.Buffer.Flags=D3D11_BUFFER_UAV_FLAG_RAW;
  if(FAILED(d->CreateUnorderedAccessView(counters.Get(),&u,&uav)))return false;
  b={};b.ByteWidth=GEO_WORDS*4;b.Usage=D3D11_USAGE_STAGING;b.CPUAccessFlags=D3D11_CPU_ACCESS_READ;
  return SUCCEEDED(d->CreateBuffer(&b,nullptr,&staging));
 }
 /* Depth SRVs (R24_UNORM_X8_TYPELESS, multisampled) of the snapshots:
    before the native draw, after it, and the second-camera replay. */
 bool dispatch(ID3D11DeviceContext*c,ID3D11ShaderResourceView*initial,ID3D11ShaderResourceView*normal,ID3D11ShaderResourceView*copy,
               UINT w,UINT h,const double sinv[16],const double m[16],const double camera[3],const double minv[16],
               ID3D11ShaderResourceView*native=nullptr,bool lessNearer=true){
  if(!native)native=normal;   /* per-draw proof: the native result is the normal snapshot */
  if(!cs||!initial||!normal||!copy)return false;
  MsaaCsState keep(c);
  D3D11_MAPPED_SUBRESOURCE map{};if(FAILED(c->Map(cb.Get(),0,D3D11_MAP_WRITE_DISCARD,0,&map)))return false;
  float*f=(float*)map.pData;for(int i=0;i<16;i++){f[i]=(float)sinv[i];f[16+i]=(float)m[i];}
  UINT*u=(UINT*)(f+32);u[0]=w;u[1]=h;f[34]=geoNearZ;f[35]=geoFarZ;for(int i=0;i<3;i++)f[36+i]=(float)camera[i];f[39]=1;for(int i=0;i<16;i++)f[40+i]=(float)minv[i];u[24]=lessNearer;c->Unmap(cb.Get(),0);
  UINT zero[4]{};c->ClearUnorderedAccessViewUint(uav.Get(),zero);
  auto v=uav.Get();UINT none=~0u;c->CSSetUnorderedAccessViews(0,1,&v,&none);
  c->CSSetShader(cs.Get(),nullptr,0);auto b=cb.Get();c->CSSetConstantBuffers(0,1,&b);
  ID3D11ShaderResourceView*s[4]={initial,normal,copy,native};c->CSSetShaderResources(0,4,s);
  c->Dispatch((w+7)/8,(h+7)/8,1);
  ID3D11ShaderResourceView*clear[4]{};c->CSSetShaderResources(0,4,clear);
  c->CopyResource(staging.Get(),counters.Get());
  return true;
 }
 /* 0 pending, 1 read, -1 failure. */
 int read(ID3D11DeviceContext*c,UINT out[GEO_WORDS]){
  D3D11_MAPPED_SUBRESOURCE m{};HRESULT hr=c->Map(staging.Get(),0,D3D11_MAP_READ,D3D11_MAP_FLAG_DO_NOT_WAIT,&m);
  if(FAILED(hr))return hr==DXGI_ERROR_WAS_STILL_DRAWING?0:-1;
  memcpy(out,m.pData,GEO_WORDS*4);c->Unmap(staging.Get(),0);return 1;
 }
};

/* A: predicted shift (pixels) on a 5x5 screen grid at three depth values that
   were actually measured (min, middle, max of the native draw), with the
   distance of each from the source camera, and the fit dx = a + b/distance
   through the screen centre. Makes no assumption about the engine's w. */
struct GeoPrediction {
 double depth[3]{},distance[3]{},dxMean[3]{},dxMin[3]{},dxMax[3]{},dyMaxAbs[3]{},a=0,b=0,fitResidual=0;unsigned points[3]{};
};
static bool geoShift(const double sinv[16],const double m[16],const double camera[3],UINT w,UINT h,double x,double y,double z,double&dx,double&dy,double*distance=nullptr){
 double n[4]={x,y,z,1},t[4]{},p[4]{};
 for(int j=0;j<4;j++)for(int i=0;i<4;i++){t[j]+=n[i]*m[i*4+j];p[j]+=n[i]*sinv[i*4+j];}
 if(!(t[3]>0)||p[3]==0)return false;
 if(distance){double d=0;for(int i=0;i<3;i++){double v=p[i]/p[3]-camera[i];d+=v*v;}*distance=sqrt(d);}
 dx=(t[0]/t[3]-x)*0.5*w;dy=-(t[1]/t[3]-y)*0.5*h;return true;
}
static bool geoPredict(const double sinv[16],const double m[16],const double camera[3],UINT w,UINT h,double minDepth,double maxDepth,GeoPrediction&p){
 if(!(maxDepth>=minDepth))return false;
 const double zs[3]={minDepth,0.5*(minDepth+maxDepth),maxDepth};
 for(int k=0;k<3;k++){
  p.depth[k]=zs[k];p.dxMin[k]=INFINITY;p.dxMax[k]=-INFINITY;double sum=0;
  for(int i=0;i<5;i++)for(int j=0;j<5;j++){
   double dx,dy;if(!geoShift(sinv,m,camera,w,h,-0.8+0.4*i,-0.8+0.4*j,zs[k],dx,dy))continue;
   p.points[k]++;sum+=dx;if(dx<p.dxMin[k])p.dxMin[k]=dx;if(dx>p.dxMax[k])p.dxMax[k]=dx;if(fabs(dy)>p.dyMaxAbs[k])p.dyMaxAbs[k]=fabs(dy);
  }
  if(!p.points[k])return false;p.dxMean[k]=sum/p.points[k];
 }
 double c[3],unused;for(int k=0;k<3;k++)if(!geoShift(sinv,m,camera,w,h,0,0,zs[k],c[k],unused,&p.distance[k]))return false;
 if(p.distance[0]!=p.distance[2]&&p.distance[0]>0&&p.distance[2]>0){
  p.b=(c[0]-c[2])/(1/p.distance[0]-1/p.distance[2]);p.a=c[0]-p.b/p.distance[0];p.fitResidual=c[1]-(p.a+p.b/p.distance[1]);}
 return true;
}

struct GeoBinVerdict {
 bool tested=false,measuredX=false,measuredY=false,pass=false;
 double sx=0,sy=0,inlierShare=0,meanDx=0,meanDy=0,infoX=0,infoY=0;
};
struct GeoVerdict { GeoBinVerdict bin[GEO_BINS];bool complete=false,vertical=false,horizontal=false,pass=false;unsigned tested=0,measuredX=0,measuredY=0; };
static GeoVerdict geoJudge(const GeoTotals&t){
 GeoVerdict v;
 for(int b=0;b<GEO_BINS;b++){
  const GeoBin&g=t.bin[b];GeoBinVerdict&r=v.bin[b];
  if(g.pixels){r.meanDx=g.dx/g.pixels;r.meanDy=g.dy/g.pixels;}
  if(g.inliers<geoMinInliers)continue;
  r.tested=true;v.tested++;
  /* Share of consistent points among those that could be judged: points
     outside the other eye's image or on a surface without depth variation
     ("flat") are unmeasurable, not wrong; a wrong camera shows up as
     unwritten, non-planar or beyond-window points. */
  double n=(double)g.inliers,judged=n+(double)(g.rejected+g.nonplanar+g.residual);
  r.inlierShare=judged>0?n/judged:0;r.infoX=g.sxx/n;r.infoY=g.syy/n;
  r.measuredX=r.infoX>=geoMinInfo;r.measuredY=r.infoY>=geoMinInfo;
  /* Normal equations (sum u u^T) s = -(sum u r'), restricted to measurable axes. */
  if(r.measuredX&&r.measuredY){double det=g.sxx*g.syy-g.sxy*g.sxy;
   r.sx=-(g.syy*g.bx-g.sxy*g.by)/det;r.sy=-(g.sxx*g.by-g.sxy*g.bx)/det;}
  else if(r.measuredX)r.sx=-g.bx/g.sxx;
  else if(r.measuredY)r.sy=-g.by/g.syy;
  v.measuredX+=r.measuredX;v.measuredY+=r.measuredY;
  r.pass=r.inlierShare>=geoMinInlierShare&&(!r.measuredX||fabs(r.sx)<=geoMaxError)&&(!r.measuredY||fabs(r.sy)<=geoMaxError);
 }
 v.complete=v.tested==GEO_BINS;
 v.vertical=v.measuredY==GEO_BINS;
 for(auto&r:v.bin)if(r.tested&&(!r.measuredY||fabs(r.sy)>geoMaxError||fabs(r.meanDy)>=1))v.vertical=false;
 double d[GEO_BINS];int k=0;for(int b=0;b<GEO_BINS;b++)if(t.bin[b].pixels)d[k++]=v.bin[b].meanDx;
 v.horizontal=v.measuredX>=2&&k==GEO_BINS&&fabs(d[0]-d[k-1])>=1;
 for(int i=1;i<k;i++)if((d[i-1]-d[i])*(d[0]-d[k-1])<=0)v.horizontal=false;
 v.pass=v.complete&&v.vertical&&v.horizontal;
 for(auto&r:v.bin)if(r.tested&&!r.pass)v.pass=false;
 return v;
}
