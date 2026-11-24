#pragma once
void postProbeArmFrame();

extern unsigned postProbeSkip,postProbeStride;
void postProbeDiscard();
void postProbePresent();
void postProbeExport(const char*dir,int ok,int slot);
int postProbeStartDraw(ID3D11DeviceContext*,ID3D11Resource*,UINT,UINT,INT,bool);
void postProbeReplayStart(int,ID3D11DeviceContext*,ID3D11Texture2D*);
void postProbeReplayDone(int,ID3D11DeviceContext*);
void postProbeRefused(int,const char*);
ID3D11ShaderResourceView*postProbeMapped(ID3D11ShaderResourceView*,ID3D11ShaderResourceView*);
struct PostProbeScope {
 int index=-1;
 PostProbeScope(ID3D11DeviceContext*c,ID3D11Resource*r,UINT n,UINT first,INT base,bool indexed){if(postProbeStride||(n==4&&!indexed))index=postProbeStartDraw(c,r,n,first,base,indexed);}
 bool selected()const{return index>=0;}
 void target(ID3D11DeviceContext*c,ID3D11Texture2D*t){postProbeReplayStart(index,c,t);}
 void done(ID3D11DeviceContext*c){postProbeReplayDone(index,c);}
 void refused(const char*r){postProbeRefused(index,r);}
 ~PostProbeScope(){postProbeRefused(index,nullptr);}
};
