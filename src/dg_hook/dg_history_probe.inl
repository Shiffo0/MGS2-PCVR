#include "dg_history_probe_core.h"
struct HistoryDrawScope{HistoryDrawScope(ID3D11DeviceContext*,UINT,UINT){}void used(){}void forwarded(){}};
struct HistoryProbe {
 DgHistoryArm arm;bool captured=false;
 void release(){captured=false;}
 void publish(int,int){}
} historyProbe;
void historyProbePresent(){}
