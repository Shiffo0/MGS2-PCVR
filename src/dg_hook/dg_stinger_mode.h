#ifndef DG_STINGER_MODE_H
#define DG_STINGER_MODE_H
#include <stdint.h>
#include <string.h>
typedef struct {
    uint64_t source,context,sequence,now_ms;
    unsigned age_ms;
    int valid,allowed,down;
} DG_STINGER_INPUT;
typedef struct {
    uint64_t actor,source,context,sequence,stamp;
    int armed,scope,active,down;
} DG_STINGER_MODE;
static void dg_stinger_mode_reset(DG_STINGER_MODE *s) {
    memset(s,0,sizeof *s);
}
/* A held grip at entry never becomes an implicit request for the sight. */
static void dg_stinger_mode_step(DG_STINGER_MODE *s,const DG_STINGER_INPUT *in,uint64_t actor) {
    if(!actor || !in->valid || !in->allowed || !in->sequence || in->age_ms>100 ||
       in->now_ms<in->age_ms) {dg_stinger_mode_reset(s);return;}
    if(s->actor!=actor || s->source!=in->source || s->context!=in->context ||
       in->sequence<s->sequence || in->now_ms<s->stamp || in->now_ms-s->stamp>100)
        dg_stinger_mode_reset(s);
    if(s->sequence==in->sequence) {
        if(s->down!=in->down)dg_stinger_mode_reset(s);
        return; /* Duplicate samples cannot renew a lease or rearm. */
    }
    s->actor=actor;s->source=in->source;s->context=in->context;
    s->sequence=in->sequence;s->stamp=in->now_ms-in->age_ms;
    s->down=in->down;s->active=1;
    if(!in->down)s->armed=1;
    s->scope=s->armed && in->down;
}
#endif
