// Inside SecondEyeLive. Preserve a native MSAA clear, never a scene/history image.

// cannot be recycled before beginFrame consumes the matching colour.
struct SceneClear {ComPtr<ID3D11Resource>resource;FLOAT rgba[4]{};unsigned epoch=~0u;const char*route=nullptr;} sceneClears[8];
const char*sceneClearRoute=nullptr;
unsigned sceneClearNext=0;unsigned long long sceneClearMatched=0,sceneClearMissing=0;

unsigned long long sceneClearByRoute[4]{};

static constexpr unsigned sceneClearHoldPresents=30;
FLOAT sceneClearHeldRgba[4]{};unsigned sceneClearHeldEpoch=0;void*sceneClearHeldRes=nullptr;bool sceneClearHeldValid=false;
unsigned long long sceneClearHeld=0;
/* Only an UNSEEN clear is held. A partial clear of the main image (forgotten)
   or a stale note means the game did clear, just not to one colour this
   frame: black, and the held colour is dropped. */
bool sceneClearMainForgot=false;
static unsigned sceneClearRouteIndex(const char*r){static const char*const names[3]={"ClearView observer","ClearRenderTargetView observer","call-site trace (fallback)"};
 for(unsigned i=0;i<3;i++)if(r&&!strcmp(r,names[i]))return i;return 3;}
void discardSceneClears(){for(auto&x:sceneClears)x=SceneClear{};sceneClearNext=0;sceneClearMainForgot=false;}
void forgetSceneClear(ID3D11Resource*r){for(auto&x:sceneClears)if(x.resource.Get()==r)x=SceneClear{};if(r&&r==mainColorRes)sceneClearMainForgot=true;}
void noteSceneClear(ID3D11RenderTargetView*v,ID3D11Resource*r,const FLOAT col[4],const char*route="observer"){
 if(!r||!v||!col||r==b.color.Get())return;
 for(auto&q:pp)if(r==q.b3.Get()||r==q.b4.Get())return;
 for(auto&q:half)if(r==q.colour.Get())return;
 D3D11_RENDER_TARGET_VIEW_DESC vd{};v->GetDesc(&vd);
 if(vd.ViewDimension!=D3D11_RTV_DIMENSION_TEXTURE2DMS)return; // whole MSAA subresource only
 for(unsigned k=0;k<4;k++)if(!std::isfinite(col[k]))return;
 SceneClear*entry=nullptr;
 for(auto&x:sceneClears)if(x.resource.Get()==r){entry=&x;break;}
 if(!entry){for(auto&x:sceneClears)if(!x.resource){entry=&x;break;}}
 if(!entry)entry=&sceneClears[sceneClearNext++%8];
 entry->resource=r;memcpy(entry->rgba,col,sizeof entry->rgba);entry->epoch=guardEpoch;entry->route=route;
}
bool takeSceneClear(FLOAT col[4]){
 bool found=false,seen=sceneClearMainForgot;
 const char*route=nullptr;
 for(auto&x:sceneClears)if(x.resource.Get()==mainColorRes){seen=true;if(x.epoch==guardEpoch){memcpy(col,x.rgba,sizeof x.rgba);route=x.route;found=true;break;}}
 discardSceneClears();
 if(found){memcpy(sceneClearHeldRgba,col,sizeof sceneClearHeldRgba);sceneClearHeldEpoch=guardEpoch;sceneClearHeldRes=mainColorRes;sceneClearHeldValid=true;}
 else if(!seen&&sceneClearHeldValid&&sceneClearHeldRes==(void*)mainColorRes&&guardEpoch-sceneClearHeldEpoch<=sceneClearHoldPresents){
  memcpy(col,sceneClearHeldRgba,sizeof sceneClearHeldRgba);sceneClearRoute="held last colour";
  if(++sceneClearHeld==1&&logger)logger("second_eye: native scene clear colour %.6g %.6g %.6g %.6g from the held last colour (no clear seen this frame, kept up to %u Presents)\r\n",col[0],col[1],col[2],col[3],sceneClearHoldPresents);
  return true;
 }
 else sceneClearHeldValid=false;
 if(found){++sceneClearMatched;sceneClearRoute=route;

  if(++sceneClearByRoute[sceneClearRouteIndex(route)]==1&&logger)logger("second_eye: native scene clear colour %.6g %.6g %.6g %.6g from the %s; no scene image copied\r\n",col[0],col[1],col[2],col[3],route?route:"?");}
 else ++sceneClearMissing;
 return found;
}
