struct DgCoverage{enum{DRAWN,ZEROED,REFUSED,NATIVE,STAGEB,KINDS};void add(UINT64,int,int){}};
DgCoverage coverage;
struct CovSnap{unsigned long long postRecorded,postRefused,draws,sideWrites,recordRefused,postMain;unsigned proj;};
static bool covActive(ID3D11DeviceContext*){return false;}
static CovSnap covTake(){return {};}
static void covRecord(ID3D11DeviceContext*,const CovSnap&,int){}
