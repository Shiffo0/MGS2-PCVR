/* dg_present.h - D3D11 Present capture for S4a. */

#ifndef DG_PRESENT_H
#define DG_PRESENT_H

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <d3d11.h>
#include <dxgi.h>

/* Starts discovery only. No XR work or GPU submission is done here. */
int dg_present_start(void (*log)(const char *fmt, ...));
void dg_present_stop(void);

/* Returned interfaces are borrowed; the module owns its retained references. */
int dg_present_get_device(ID3D11Device **dev, IDXGISwapChain **sc);

/* The callback runs on the game's Present thread, before the chained Present. */
void dg_present_set_callback(void (*on_present)(IDXGISwapChain *sc));
void dg_present_stats(long *presents, long *errors);
/* Worker-only marker polling; exactly one opt-in attempt per DLL lifetime. */
void dg_present_poll_state_probe(const char *marker);

#endif
