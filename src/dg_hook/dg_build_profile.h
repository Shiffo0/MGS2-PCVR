#ifndef DG_BUILD_PROFILE_H
#define DG_BUILD_PROFILE_H
/* Release is the default. Diagnostic builds must opt in at compile time. */
#ifndef DG_ENABLE_DIAGNOSTICS
#define DG_ENABLE_DIAGNOSTICS 0
#endif
#if DG_ENABLE_DIAGNOSTICS != 0 && DG_ENABLE_DIAGNOSTICS != 1
#error DG_ENABLE_DIAGNOSTICS must be 0 or 1
#endif
#if !DG_ENABLE_DIAGNOSTICS
#define DG_DIAGNOSTIC_CAPACITY(n) 1
#else
#define DG_DIAGNOSTIC_CAPACITY(n) (n)
#endif
#endif
