/* Read-only, bounded CPU witnesses at the native consumer entry/return.
 * No restore API: these bytes alone cannot authorize native render replay. */
#ifndef DG_NATIVE_STATE_PROBE_H
#define DG_NATIVE_STATE_PROBE_H
#include <stdint.h>
#include <string.h>
#include <stdio.h>
typedef int (*NS_READ)(void*,uint64_t,void*,unsigned);
typedef struct { const char *name; unsigned offset,size; } NS_FIELD;
static const NS_FIELD ns_fields[]={
 {"draw_count",0x174,4},{"descriptor_header",0x6c0,16},
 {"descriptor_state",0x6d0,0x138},{"binding_keys",0x808,16},
 {"index_base",0x818,4},{"binding_source_count_dirty",0x908,13}
};
typedef struct {
 uint64_t root,pools[2];
 unsigned char state[365],pool_state[2][20],global_count[4];
 int complete,error;
} NS_SAMPLE;
/* Explicit widths: pool+8 GPU identity; +1c cursor; +38 descriptor cursor
 * and +3c discard flag. Pointers are identities, never followed to GPU data. */












































#endif
