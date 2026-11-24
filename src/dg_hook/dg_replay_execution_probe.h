#pragma once
#include <stdint.h>
#include <d3d11.h>
#include <wrl/client.h>
struct DgReplayExecutionProbe {
 using Factory=HRESULT(STDMETHODCALLTYPE*)(ID3D11Device*,const D3D11_QUERY_DESC*,ID3D11Query**);
 Microsoft::WRL::ComPtr<ID3D11Query> occlusion,statistics;
 bool pending=false,active=false;uint64_t next=0,started=0,id=0,bankId=0,hash=0;unsigned frame=0,n=0,first=0;int base=0;
 void(*log)(const char*,...)=nullptr;
 UINT64 samples=0;D3D11_QUERY_DATA_PIPELINE_STATISTICS stats{};unsigned completed=0;
 bool begin(ID3D11Device*d,ID3D11DeviceContext*c,Factory factory,void(*report)(const char*,...),uint64_t now,uint64_t bank,uint64_t shader,unsigned fr,unsigned count,unsigned firstIndex,int baseVertex){
  if(pending||active||!factory||!report||now<next)return false;
  next=now+100; // At most 10 observed existing draws/second; only one outstanding.
  D3D11_QUERY_DESC q{D3D11_QUERY_OCCLUSION,0};
  if(!occlusion&&FAILED(factory(d,&q,&occlusion))){next=now+1000;return false;}
  q.Query=D3D11_QUERY_PIPELINE_STATISTICS;
  if(!statistics&&FAILED(factory(d,&q,&statistics))){next=now+1000;return false;}
  log=report;started=now;bankId=bank;hash=shader;frame=fr;n=count;first=firstIndex;base=baseVertex;++id;
  c->Begin(occlusion.Get());c->Begin(statistics.Get());active=true;return true;
 }
 void end(ID3D11DeviceContext*c){if(!active)return;c->End(statistics.Get());c->End(occlusion.Get());active=false;pending=true;}
 void poll(ID3D11DeviceContext*c,uint64_t now){
  if(!pending)return;UINT64 count=0;D3D11_QUERY_DATA_PIPELINE_STATISTICS result{};
  HRESULT a=c->GetData(occlusion.Get(),&count,sizeof(count),D3D11_ASYNC_GETDATA_DONOTFLUSH);
  HRESULT b=c->GetData(statistics.Get(),&result,sizeof(result),D3D11_ASYNC_GETDATA_DONOTFLUSH);
  if(a==S_OK&&b==S_OK){
   samples=count;stats=result;pending=false;++completed;
   log("replay_execution: id=%llu bank_id=%llu frame=%u shader=%016llX draw=%u first=%u base=%d samples=%llu ia_vertices=%llu ia_primitives=%llu vs=%llu gs=%llu gs_primitives=%llu clip_in=%llu clip_out=%llu ps=%llu tick=%llu observed_existing_draw=1 not_final_pixels=1\r\n",id,bankId,frame,hash,n,first,base,samples,stats.IAVertices,stats.IAPrimitives,stats.VSInvocations,stats.GSInvocations,stats.GSPrimitives,stats.CInvocations,stats.CPrimitives,stats.PSInvocations,started);
  }else if(FAILED(a)||FAILED(b)||now-started>1000){
   log("replay_execution: id=%llu bank_id=%llu query_incomplete=1 hr=%08X,%08X timeout=%d\r\n",id,bankId,(unsigned)a,(unsigned)b,now-started>1000);
   pending=false;occlusion.Reset();statistics.Reset();
  }
 }
};
