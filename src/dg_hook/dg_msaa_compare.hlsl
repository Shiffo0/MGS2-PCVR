// Per-sample comparison of two multisampled snapshots, for the draw trial's
// integrity and contribution checks. Staging textures cannot be multisampled
// and a resolve averages samples, so equality is decided here, sample by
// sample and bit-exact (asuint), and only two counters per pair leave the GPU:
//   Counters[Slot*8+0] += samples that differ
//   Counters[Slot*8+4] += pixels with at least one differing sample
// Compiled twice by gen_msaa_compare.bat: colour, and /D DEPTH=1 (depth from
// R24_UNORM_X8_TYPELESS plus stencil from X24_TYPELESS_G8_UINT, .g).
cbuffer P : register(b0) { uint W; uint H; uint S; uint Slot; };
RWByteAddressBuffer Counters : register(u0);
#ifdef DEPTH
Texture2DMS<float> DA : register(t0);
Texture2DMS<uint2> SA : register(t1);
Texture2DMS<float> DB : register(t2);
Texture2DMS<uint2> SB : register(t3);
#else
Texture2DMS<float4> A : register(t0);
Texture2DMS<float4> B : register(t1);
#endif
[numthreads(8, 8, 1)]
void main(uint3 id : SV_DispatchThreadID)
{
    if (id.x >= W || id.y >= H) return;
    int2 p = int2(id.xy);
    uint d = 0;
    for (uint s = 0; s < S; s++) {
#ifdef DEPTH
        if (asuint(DA.Load(p, s)) != asuint(DB.Load(p, s)) || SA.Load(p, s).g != SB.Load(p, s).g) d++;
#else
        if (any(asuint(A.Load(p, s)) != asuint(B.Load(p, s)))) d++;
#endif
    }
    if (d) {
        uint ignored;
        Counters.InterlockedAdd(Slot * 8, d, ignored);
        Counters.InterlockedAdd(Slot * 8 + 4, 1, ignored);
    }
}
