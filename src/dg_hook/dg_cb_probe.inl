UINT64 vsHashOf(void *ptr, const BYTE **bytes = nullptr, UINT32 *size = nullptr);
#pragma pack(push, 1)
#pragma pack(pop)
void cbOnDraw(ID3D11DeviceContext *c, unsigned kind, UINT count, UINT first, INT base){}
void cbOnPresent(){}
void cbPoll(const char *markerPath, const char *text, size_t n){}
