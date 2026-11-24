// Geometry proof for the second camera (ONTWERP_GEOMETRIEPROEF.md).
// For every pixel whose sample 0 the native draw wrote (depth: normal !=
// initial) the point is reprojected into the other eye with M = inverse(S)*T
// (row vectors, as the engine): predicted location and depth. The replay
// target (copy, the second-camera draw) is read around that location: the
// four sample-0 neighbours must all be written by the replay and coplanar
// (no edge, no occluder). Depth of a plane is linear in screen space, so the
// bilinear depth there is exact; its residual r against the predicted depth
// satisfies r = -(gx*sx + gy*sy) for an image shifted by (sx,sy) pixels.
// Per distance bin the normal equations of that least-squares problem are
// accumulated, each pixel normalised by its gradient length (so r' is in
// pixels); the CPU solves for (sx,sy). An axis without depth variation (a
// floor seen horizontally, a wall seen head-on) is reported unmeasurable,
// never as agreeing. Counters per bin (GEO_STRIDE uints, 64-bit = lo,hi):
//   [0] pixels [1] inliers [2] behind/outside [3] replay neighbour unwritten
//   [4] too flat to measure  [5] Sxx [7] Syy [9] Sxy+ [11] Sxy- [13] Bx+ [15] Bx-
//   [17] By+ [19] By- (B = sum g*r')  [21] dx+ [23] dx- [25] dy+ [27] dy-
//   (predicted shift, all pixels in front of the other eye)  [29] ~min distance
//   bits [30] max distance bits  [31] neighbours not coplanar (edge, occluder)
//   [32] residual beyond the window (image elsewhere).  Fixed point: 1/4096.
// Distance = |world point - source camera centre|: independent of how the
// engine's projection scales w (its own screen units). After the bins:
//   [0] sample count, [1..44] four samples (x, y, depth bits, n*Sinv xyzw,
//   n*M xyzw), [45] ~min depth bits, [46] max depth bits - raw evidence of
//   the engine's clip conventions. Centroids, independent of any window:
//   [48] replay-written pixels whose back-projection (inverse M) lies inside
//   the source image, [49] sum x, [51] sum y;  [53] native pixels whose
//   prediction lies inside the target image, [54] predicted x +/- [58]
//   predicted y +/- (1/4096), [62] native sum x, [64] native sum y.
//   Both sides filtered alike, so the screen edges do not bias the centroids.
//   Same camera against the real frame (frame capture: Normal is the
//   same-camera replay A, Native the game's own depth at Present):
//   [66] A-written pixels, [67] of those Native bit-equal, [68] of those
//   Native FARTHER than A - A drew a surface the game does not have: a
//   contradiction (nearer is legitimate: geometry not replayed covers it),
//   [69] Native written where A is not (writes outside the replayed set).
cbuffer P : register(b0) {
    row_major float4x4 Sinv;   // clip -> world, source eye
    row_major float4x4 M;      // source clip -> target clip
    uint W; uint H; float NearZ; float FarZ;
    float4 Camera;             // source camera centre, world
    row_major float4x4 Minv;   // target clip -> source clip
    uint LessNearer;           // 1: smaller depth is nearer (LESS family)
};
RWByteAddressBuffer Counters : register(u0);
Texture2DMS<float> Initial : register(t0);
Texture2DMS<float> Normal  : register(t1);
Texture2DMS<float> Copy    : register(t2);
Texture2DMS<float> Native  : register(t3);
#define GEO_STRIDE 34
#define GEO_EXTRA (3 * GEO_STRIDE * 4)
#define SCALE 4096.0
#define MAX_SHIFT 4.0
#define MIN_GRADIENT 2.4e-7   // four D24 steps per pixel

void add64(uint o, uint v) {
    uint old;
    Counters.InterlockedAdd(o, v, old);
    if (old + v < old) Counters.InterlockedAdd(o + 4, 1);
}
void addSigned(uint o, float x) {
    int v = int(round(x * SCALE));
    if (v >= 0) add64(o, uint(v)); else add64(o + 8, uint(-v));
}
bool replayWritten(int2 c) {
    return c.x >= 0 && c.y >= 0 && c.x < int(W) && c.y < int(H) &&
           asuint(Copy.Load(c, 0)) != asuint(Initial.Load(c, 0));
}
[numthreads(8, 8, 1)]
void main(uint3 id : SV_DispatchThreadID)
{
    if (id.x >= W || id.y >= H) return;
    int2 p = int2(id.xy);
    if (replayWritten(p)) {
        float2 rp = Copy.GetSamplePosition(0);
        float4 back = mul(float4((p.x + 0.5 + rp.x) / W * 2 - 1, 1 - (p.y + 0.5 + rp.y) / H * 2, Copy.Load(p, 0), 1), Minv);
        if (back.w > 0) {
            float2 b = float2((back.x / back.w + 1) * 0.5 * W, (1 - back.y / back.w) * 0.5 * H);
            if (b.x >= 0 && b.y >= 0 && b.x < float(W) && b.y < float(H)) {
                Counters.InterlockedAdd(GEO_EXTRA + 48 * 4, 1);
                add64(GEO_EXTRA + 49 * 4, uint(p.x));
                add64(GEO_EXTRA + 51 * 4, uint(p.y));
            }
        }
    }
    float z = Normal.Load(p, 0), i0 = Initial.Load(p, 0), n0 = Native.Load(p, 0);
    if (asuint(z) != asuint(i0)) {
        Counters.InterlockedAdd(GEO_EXTRA + 66 * 4, 1);
        if (asuint(n0) == asuint(z)) Counters.InterlockedAdd(GEO_EXTRA + 67 * 4, 1);
        else if (LessNearer ? n0 > z : n0 < z) Counters.InterlockedAdd(GEO_EXTRA + 68 * 4, 1);
    } else if (asuint(n0) != asuint(i0)) Counters.InterlockedAdd(GEO_EXTRA + 69 * 4, 1);
    if (asuint(z) == asuint(i0)) return;
    float2 sp = Normal.GetSamplePosition(0);
    float4 n = float4((p.x + 0.5 + sp.x) / W * 2 - 1, 1 - (p.y + 0.5 + sp.y) / H * 2, z, 1);
    float4 world = mul(n, Sinv);
    float4 h = mul(n, M);
    Counters.InterlockedMax(GEO_EXTRA + 45 * 4, ~asuint(z));
    Counters.InterlockedMax(GEO_EXTRA + 46 * 4, asuint(z));
    uint slot;
    Counters.InterlockedAdd(GEO_EXTRA, 1, slot);
    if (slot < 4) {
        uint o = GEO_EXTRA + (1 + slot * 11) * 4;
        Counters.Store3(o, uint3(p.x, p.y, asuint(z)));
        Counters.Store4(o + 12, asuint(world));
        Counters.Store4(o + 28, asuint(h));
    }
    if (world.w == 0) return;
    float view = length(world.xyz / world.w - Camera.xyz);
    if (!(view > 0)) return;
    uint base = (view < NearZ ? 0 : (view < FarZ ? 1 : 2)) * GEO_STRIDE * 4;
    Counters.InterlockedAdd(base, 1);
    Counters.InterlockedMax(base + 29 * 4, ~asuint(view));
    Counters.InterlockedMax(base + 30 * 4, asuint(view));
    if (!(h.w > 0)) { Counters.InterlockedAdd(base + 2 * 4, 1); return; }
    float3 t = h.xyz / h.w;
    // Predicted location in sample-0 grid coordinates of the other eye.
    float2 g = float2((t.x + 1) * 0.5 * W, (1 - t.y) * 0.5 * H) - 0.5 - sp;
    addSigned(base + 21 * 4, g.x - p.x);
    addSigned(base + 25 * 4, g.y - p.y);
    if (g.x >= -0.5 && g.y >= -0.5 && g.x < float(W) - 0.5 && g.y < float(H) - 0.5) {
        Counters.InterlockedAdd(GEO_EXTRA + 53 * 4, 1);
        addSigned(GEO_EXTRA + 54 * 4, g.x);
        addSigned(GEO_EXTRA + 58 * 4, g.y);
        add64(GEO_EXTRA + 62 * 4, uint(p.x));
        add64(GEO_EXTRA + 64 * 4, uint(p.y));
    }
    int2 c = int2(floor(g));
    float2 f = g - c;
    if (!replayWritten(c) || !replayWritten(c + int2(1, 0)) || !replayWritten(c + int2(0, 1)) || !replayWritten(c + int2(1, 1))) {
        Counters.InterlockedAdd(base + (c.x < -1 || c.y < -1 || c.x >= int(W) || c.y >= int(H) ? 2 : 3) * 4, 1); return;
    }
    float d00 = Copy.Load(c, 0), d10 = Copy.Load(c + int2(1, 0), 0);
    float d01 = Copy.Load(c + int2(0, 1), 0), d11 = Copy.Load(c + int2(1, 1), 0);
    float gx = 0.5 * ((d10 - d00) + (d11 - d01)), gy = 0.5 * ((d01 - d00) + (d11 - d10));
    float len = sqrt(gx * gx + gy * gy);
    if (len < MIN_GRADIENT) { Counters.InterlockedAdd(base + 4 * 4, 1); return; }
    // Coplanar neighbours only: an edge or an occluder breaks the plane.
    if (abs(d00 + d11 - d01 - d10) > 0.25 * (abs(gx) + abs(gy)) + 1.2e-7) { Counters.InterlockedAdd(base + 31 * 4, 1); return; }
    float d = lerp(lerp(d00, d10, f.x), lerp(d01, d11, f.x), f.y);
    float ux = gx / len, uy = gy / len, r = (d - t.z) / len;   // r in pixels along the gradient
    if (abs(r) > MAX_SHIFT) { Counters.InterlockedAdd(base + 32 * 4, 1); return; }
    Counters.InterlockedAdd(base + 1 * 4, 1);
    add64(base + 5 * 4, uint(round(ux * ux * SCALE)));
    add64(base + 7 * 4, uint(round(uy * uy * SCALE)));
    addSigned(base + 9 * 4, ux * uy);
    addSigned(base + 13 * 4, ux * r);
    addSigned(base + 17 * 4, uy * r);
}
