// Original 5109FCC9 tangent mapping and framing. No pixel displacement.
cbuffer C:register(b0) {float4 L,R,crop,opt,arm0,arm1,armMode;};
Texture2D A:register(t0),B:register(t1);SamplerState S:register(s0);
struct V {float4 p:SV_Position;float2 uv:TEXCOORD0;};
V vs(uint id:SV_VertexID) {
    V o;float2 u=float2((id<<1)&2,id&2);o.uv=u;
    o.p=float4(u.x*2-1,1-u.y*2,0,1);return o;
}
float2 uv(float2 p,float4 f){return float2((p.x-f.x)/(f.y-f.x),(f.w-p.y)/(f.w-f.z));}
float protection(float2 p,float4 b) {
    float2 outside=max(float2(b.x,b.z)-p,p-float2(b.y,b.w));
    return 1-smoothstep(0,max(opt.y,.02),max(outside.x,outside.y));
}
float4 ps(V i):SV_Target {
    float2 p=float2(lerp(crop.x,crop.y,i.uv.x),lerp(crop.w,crop.z,i.uv.y));
    float3 l=A.SampleLevel(S,uv(p,L),0).rgb,r=B.SampleLevel(S,uv(p,R),0).rgb;
    if(opt.x<.5)return float4(l,1);
    if(opt.x<1.5)return float4(r,1);
    float w=1;
    if(p.x<R.x||p.x>R.y)w=0;
    else {
        if(L.x<R.x)w=min(w,saturate((p.x-R.x)/max(opt.y,.0001)));
        if(L.y>R.y)w=min(w,saturate((R.y-p.x)/max(opt.y,.0001)));
    }
    if(armMode.x>0)w=lerp(w,armMode.x-1,protection(p,arm0));
    if(armMode.y>0)w=lerp(w,armMode.y-1,protection(p,arm1));
    return float4(lerp(l,r,w),1);
}
