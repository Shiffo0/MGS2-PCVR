#pragma once

#define DG_REPLAY_NEAR_NATIVE_MM 50.0f
static inline float dgReplayNearScale(long mm) { return mm > 0 ? (float)mm / DG_REPLAY_NEAR_NATIVE_MM : 1.0f; }
/* Scales the z row (r = 2) of a column-major game-clip camera; no-op when off or not deeper than native. */
static inline void dgReplayNearApply(float T[16], long mm) {
    float s = dgReplayNearScale(mm);
    if (!(s > 1.0001f)) return;
    for (int c = 0; c < 4; c++) T[c * 4 + 2] *= s;
}
