unsigned postProbeSkip=0,postProbeStride=0;
void postProbeArmFrame(){}
void postProbeDiscard(){}
void postProbePresent(){}
void postProbeExport(const char*,int,int){}
int postProbeStartDraw(ID3D11DeviceContext*,ID3D11Resource*,UINT,UINT,INT,bool){return -1;}
void postProbeReplayStart(int,ID3D11DeviceContext*,ID3D11Texture2D*){}
void postProbeReplayDone(int,ID3D11DeviceContext*){}
void postProbeRefused(int,const char*){}
ID3D11ShaderResourceView*postProbeMapped(ID3D11ShaderResourceView*,ID3D11ShaderResourceView*m){return m;}
