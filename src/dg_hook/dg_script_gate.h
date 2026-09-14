#ifndef DG_SCRIPT_GATE_H
#define DG_SCRIPT_GATE_H

#include <stddef.h>
#include <stdint.h>

#define DG_SCRIPT_MARKER_MAX_BYTES 1023u

typedef struct {
    int source_script;
    int token_present;
    int token_valid;
    int token;
} DG_SCRIPT_MARKER_INFO;

typedef struct {
    int baseline;
    int high_water;
    int active;
    int requested;
    int busy;
    int initialized;
} DG_SCRIPT_GATE;

/* Parse only the bounded control surface. Unknown marker keys are ignored. */
int dg_script_marker_parse(const char *text, size_t len,
                           DG_SCRIPT_MARKER_INFO *out);
void dg_script_gate_init(DG_SCRIPT_GATE *gate, int persisted_baseline);
int dg_script_gate_offer(DG_SCRIPT_GATE *gate, int source_script,
                         int token_valid, int token);
void dg_script_gate_consume(DG_SCRIPT_GATE *gate);
/* Returns nonzero while the active request remains exactly the same. */
int dg_script_gate_observe(DG_SCRIPT_GATE *gate, int source_script,
                           int token_valid, int token);
void dg_script_gate_finish(DG_SCRIPT_GATE *gate);
/* -1 seeds the first valid marker as a baseline; 0 waits; 1 offers once. */
int dg_script_gate_poll(DG_SCRIPT_GATE *gate, int read_ok,
                        const DG_SCRIPT_MARKER_INFO *info);
int dg_script_gate_self_test(void);

#endif
