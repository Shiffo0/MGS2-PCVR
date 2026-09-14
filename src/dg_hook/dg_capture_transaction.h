/* Shared by the runtime and the deterministic producer/consumer desk test.
 * Acquire/wait/release belong OUTSIDE this transaction. Callbacks must not
 * block on XR, and copy queues work on the same immediate context as capture.
 * This makes one coherent snapshot of AFR stores, not a same-time stereo pair.
 */
#ifndef DG_CAPTURE_TRANSACTION_H
#define DG_CAPTURE_TRANSACTION_H
typedef struct {
    void (*lock)(void *);
    void (*unlock)(void *);
    int (*valid)(void *, int);
    void (*copy)(void *, int);
} DG_CAPTURE_SNAPSHOT_OPS;

static int dg_capture_snapshot(const DG_CAPTURE_SNAPSHOT_OPS *ops,
                               void *ctx, int eyes) {
    int eye, ok = 1;
    if (eyes != 1 && eyes != 2) return 0;
    ops->lock(ctx);
    for (eye = 0; eye < eyes; ++eye)
        if (!ops->valid(ctx, eye)) { ok = 0; break; }
    if (ok)
        for (eye = 0; eye < eyes; ++eye) ops->copy(ctx, eye);
    ops->unlock(ctx);
    return ok;
}
#endif
