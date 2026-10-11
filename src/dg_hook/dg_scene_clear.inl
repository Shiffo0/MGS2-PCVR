// Inside SecondEyeLive. Preserve a native MSAA clear, never a scene/history image.
// References live only through the current Present interval; resource identity
// cannot be recycled before beginFrame consumes the matching colour.
struct SceneClear {ComPtr<ID3D11Resource>resource;FLOAT rgba[4]{};unsigned epoch=~0u;} sceneClears[8];
unsigned sceneClearNext=0;unsigned long long sceneClearMatched=0,sceneClearMissing=0;
void discardSceneClears(){for(auto&x:sceneClears)x=SceneClear{};sceneClearNext=0;}
void forgetSceneClear(ID3D11Resource*r){for(auto&x:sceneClears)if(x.resource.Get()==r)x=SceneClear{};}
void noteSceneClear(ID3D11RenderTargetView*v,ID3D11Resource*r,const FLOAT col[4]){
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
 entry->resource=r;memcpy(entry->rgba,col,sizeof entry->rgba);entry->epoch=guardEpoch;
}
bool takeSceneClear(FLOAT col[4]){
 bool found=false;
 for(auto&x:sceneClears)if(x.resource.Get()==mainColorRes&&x.epoch==guardEpoch){memcpy(col,x.rgba,sizeof x.rgba);found=true;break;}
 discardSceneClears();
 if(found){++sceneClearMatched;if(sceneClearMatched==1&&logger)logger("second_eye: native scene clear colour %.6g %.6g %.6g %.6g; no scene image copied\r\n",col[0],col[1],col[2],col[3]);}
 else ++sceneClearMissing;
 return found;
}
