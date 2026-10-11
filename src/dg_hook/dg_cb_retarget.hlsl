// GPU retargeting of a world shader's constant bank for the second camera,
// without a CPU readback: the same arithmetic as dg_second_camera_bank()
// (dg_second_camera.h). The shader's gVS_Pers rows (constant Pers..Pers+3 of
// the bound range: c20 for most shaders, c16 or c24 for some, per the
// 18 Sep shader table) become transpose(D) * rows; every other word is copied.
//   output[i][j] = sum_k D[k][i] * input[k][j]      (i,j,k = 0..3)
// The rows hold P with P^T = X * S * Q: the frame's source camera S, the port's
// clip conversion Q (folded into S and D by the caller) and a per-draw prefix X.
// Right-multiplying by D gives X * T - exact when X is affine (a model or world
// transform). A projective X (another projection) is not retargeted: its rows
// become zero (w = 0, nothing drawn) in both passes, and the draw is counted.
cbuffer P : register(b0) {
    row_major float4x4 D;     // inverse(S) * T (identity: same camera)
    uint Begin;               // first word of the bound range
    uint Words;               // words in the buffer
    float Tolerance;          // relative
    uint Count;               // 1: add this draw to Flags (once per draw)
    row_major float4x4 Sinv;  // inverse of the frame's source camera
    uint Pers;                // constant index of gVS_Pers within the range
    uint Slot;                // shader slot for the per-shader counters
    float4 SourceRounding;    // source-camera clip-z operand rounding, computed once per frame
};
Buffer<uint> In : register(t0);
RWBuffer<uint> Out : register(u0);
// Flags: [0] draws checked, [1] projective, [2] banks captured, [3 + 32n ..]
// the 8 constants from gVS_Pers-4 of the first four checked draws (evidence),
// [131 + 2*slot] draws checked per shader slot, [132 + 2*slot] projective.
RWBuffer<uint> Flags : register(u1);
bool affinePrefix(uint row0)
{
    // X[i][3] = sum_k P^T[i][k] * Sinv[k][3],  P^T[i][k] = c[Pers+k][i]
    [unroll] for (uint i = 0; i < 4; i++) {
        float x = 0, scale = 0;
        [unroll] for (uint k = 0; k < 4; k++) {
            float t = asfloat(In[row0 + k * 4 + i]) * Sinv[k][3];
            x += t; scale += abs(t);
        }
        float want = i == 3 ? 1.0 : 0.0;
        // Match the CPU bound for rounding of legacy z/w before z'=(z+w)/2.
        float z = asfloat(In[row0 + 8 + i]), w = asfloat(In[row0 + 12 + i]);
        float rounding = (5.960464477539063e-8 * 0.5) *
                         (abs(2 * z - w) + abs(w)) * abs(Sinv[2][3]);
        // Most banks already pass; only pay for camera-error propagation on refusal.
        if (!(abs(x - want) <= Tolerance * (scale + 1) + rounding)) {
            float sourceError = 0;
            [unroll] for (uint j = 0; j < 4; j++) {
                float prefix = 0;
                [unroll] for (uint k = 0; k < 4; k++)
                    prefix += asfloat(In[row0 + k * 4 + i]) * Sinv[k][j];
                sourceError += abs(prefix) * SourceRounding[j];
            }
            rounding += sourceError * abs(Sinv[2][3]);
        }
        float bound = Tolerance * (scale + 1) + rounding;
        if (!isfinite(bound) || !(abs(x - want) <= bound)) return false;
    }
    return true;
}
[numthreads(64, 1, 1)]
void main(uint3 id : SV_DispatchThreadID)
{
    uint w = id.x;
    if (w >= Words) return;
    uint row0 = Begin + Pers * 4;
    uint v = In[w];
    if (w >= row0 && w < row0 + 16) {
        bool ok = affinePrefix(row0);
        if (w == row0 && Count) {
            InterlockedAdd(Flags[0], 1);
            InterlockedAdd(Flags[131 + 2 * Slot], 1);
            if (!ok) { InterlockedAdd(Flags[1], 1); InterlockedAdd(Flags[132 + 2 * Slot], 1); }
            uint n;
            InterlockedAdd(Flags[2], 1, n);
            if (n < 4) for (uint q = 0; q < 32; q++) Flags[3 + n * 32 + q] = In[row0 - 16 + q];
        }
        uint e = w - row0, i = e / 4, j = e % 4;
        float s = 0;
        [unroll] for (uint k = 0; k < 4; k++) s += D[k][i] * asfloat(In[row0 + k * 4 + j]);
        v = ok ? asuint(s) : 0;
    }
    Out[w] = v;
}
