#ifndef DG_COOLANT_INPUT_H
#define DG_COOLANT_INPUT_H
#include <stdint.h>


typedef struct {
    uint64_t player,pad,press;
    unsigned tick,stream,mask;
    int pressure,ready;
} DG_COOLANT_LEASE;
typedef struct {
    uint64_t player,pad,press,release;
    unsigned tick,stream,mask,status;
    int safe,weapon,fresh;
} DG_COOLANT_CONSUMER;
static int dg_coolant_take(DG_COOLANT_LEASE *lease,
                           const DG_COOLANT_CONSUMER *in) {
    DG_COOLANT_LEASE old=*lease;
    lease->ready=0; /* Every attempt consumes the lease, even a refusal. */
    return old.ready && in->safe && in->fresh && in->weapon==14 &&
        old.player==in->player && old.pad==in->pad && old.mask &&
        old.mask==in->mask && old.stream==in->stream &&
        old.press==in->press && in->press>in->release &&
        (unsigned)(in->tick-old.tick)<=1 && !(in->status&in->mask);
}
static int dg_coolant_merge_pad(DG_COOLANT_LEASE *lease,
    const DG_COOLANT_CONSUMER *in,volatile unsigned *status,
    volatile unsigned char *pressure) {
    int wanted=lease->pressure;
    if(!dg_coolant_take(lease,in))return 0;
    *status|=in->mask;
    if(wanted>0 && wanted<=255 && *pressure<wanted)*pressure=(unsigned char)wanted;
    return 1;
}
#endif
