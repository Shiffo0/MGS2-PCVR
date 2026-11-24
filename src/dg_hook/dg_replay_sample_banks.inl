/* Single render-thread writer. Source shadow only: no added GPU readback. */
struct SampleBankSlot {
 volatile LONG64 stamp;
 uint64_t meta[15];
 float matrices[68]; /* delta, inverse source, source camera, target camera, rounding */
 uint8_t bank[65536];
};
struct SampleBankRing {
 uint64_t magic,version,stride,capacity;
 volatile LONG64 written;
 uint64_t reserved[3];
 SampleBankSlot slots[32];
};
extern "C" __declspec(dllexport) SampleBankRing dg_replay_sample_banks = {
 0x525042414E4B5331ULL,1,sizeof(SampleBankSlot),32,0,{},{}
};
static_assert(sizeof(SampleBankSlot)==65936,"sample ABI");
static_assert(offsetof(SampleBankRing,slots)==64,"sample header ABI");
static void captureSampleBank(uint64_t epoch,uint64_t frame,int eye,int seq,uint64_t hash,
 UINT n,UINT first,INT base,UINT pers,UINT width,UINT begin,UINT bytes,UINT cbCount,const uint8_t*data,
 const float*delta,const float*sinv,const float*S,const float*T,const float*rounding){
 auto&b=dg_replay_sample_banks;LONG64 id=b.written+1;auto&s=b.slots[(id-1)%32];
 InterlockedExchange64(&s.stamp,id*2-1);
 uint64_t m[15]={epoch,frame,(uint64_t)(int64_t)eye,(uint64_t)(int64_t)seq,GetTickCount64(),hash,n,first,(uint64_t)(int64_t)base,pers,width,begin,bytes,0,cbCount};
 memcpy(s.meta,m,sizeof(m));
 memcpy(s.matrices,delta,64);memcpy(s.matrices+16,sinv,64);
 memcpy(s.matrices+32,S,64);memcpy(s.matrices+48,T,64);memcpy(s.matrices+64,rounding,16);
 /* Invalid/missing shadows remain explicit empty records; no partial-bank claim. */
 if(data&&bytes&&bytes<=65536&&begin<=width&&bytes<=width-begin){
  memcpy(s.bank,data,bytes);s.meta[13]=1;
 }
 InterlockedExchange64(&s.stamp,id*2);InterlockedExchange64(&b.written,id);
}
