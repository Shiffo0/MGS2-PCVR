#include "dg_radial_overlay.h"
#include <string.h>
static int ops_ok(const dg_radial_overlay_ops *o) {
    return o && o->create && o->acquire && o->wait && o->upload && o->release && o->destroy;
}
void dg_radial_overlay_init(dg_radial_overlay *s, uint64_t epoch) {
    if (s) { memset(s,0,sizeof(*s)); s->session_epoch=epoch; }
}
int dg_radial_overlay_teardown(dg_radial_overlay *s,
    const dg_radial_overlay_ops *o, int stopped) {
    if (!s || !stopped || !ops_ok(o)) return 0;
    if (s->handle && o->destroy(o->user,s->handle)!=DG_RADIAL_IO_OK) {
        s->fault=1; return 0;
    }
    memset(s,0,sizeof(*s)); return 1;
}
dg_radial_overlay_layer dg_radial_overlay_step(dg_radial_overlay *s,
    const dg_radial_overlay_ops *o, const dg_radial_overlay_frame *f,
    const dg_radial_overlay_model *m, uint32_t *scratch, size_t capacity) {
    dg_radial_overlay_layer layer;
    int want, result, uploaded=0;
    memset(&layer,0,sizeof(layer));
    if (!s || !ops_ok(o) || !f || s->fault || !s->session_epoch) return layer;
    if (f->session_epoch!=s->session_epoch) { s->fault=1; return layer; }
    /* Publish cancellation even if the caller revisits the same frame. */
    if (m && m->session_epoch==s->session_epoch && m->sequence>s->last_model)
        s->last_model=m->sequence;
    if (!f->running) { s->blocked_model=s->last_model; return layer; }
    if (s->seen_frame && (f->sequence<s->last_frame || f->now_ms<s->last_ms)) {
        s->fault=1; return layer;
    }
    /* Sequence belongs to the session, including hidden/cancel publications.
     * A late older model must not resurrect a menu closed by a newer one. */
    want=f->focused && f->should_render && f->projection_ready && !f->theater_active &&
        f->base_layers>0 && f->base_layers<f->max_layers && m && m->sequence &&
        m->sequence==s->last_model && m->sequence>s->blocked_model && m->view.visible &&
        m->session_epoch==f->session_epoch && m->context_epoch==f->context_epoch &&
        m->reference_epoch==f->reference_epoch && m->catalog_version==f->catalog_version &&
        m->sample_ms<=f->now_ms && f->now_ms-m->sample_ms<=100u;
    if (!want) s->blocked_model=s->last_model;
    if (s->seen_frame && f->sequence==s->last_frame) return layer;
    s->seen_frame=1; s->last_frame=f->sequence; s->last_ms=f->now_ms;
    if (want) {
        want=dg_radial_view_raster(&m->view,scratch,capacity)==1;
        if (!want) s->blocked_model=s->last_model;
    }
    if (want && !s->handle) {
        if (o->create(o->user,DG_RADIAL_VIEW_SIZE,DG_RADIAL_VIEW_SIZE,&s->handle,&s->images)!=DG_RADIAL_IO_OK ||
            !s->handle || !s->images) { s->fault=1; return layer; }
    }
    if (want && !s->acquired) {
        if (o->acquire(o->user,s->handle,&s->image)!=DG_RADIAL_IO_OK) {
            s->fault=1; return layer;
        }
        s->acquired=1;
        if (s->image>=s->images) { s->fault=1; return layer; }
    }
    if (!s->acquired) return layer;
    /* Even an expired/hidden model drains an already acquired image, without
     * upload/submission. A timed-out wait is retried on the SAME image next
     * frame. Never release an image that has not successfully waited. */
    result=o->wait(o->user,s->handle);
    if (result==DG_RADIAL_IO_TIMEOUT) return layer;
    if (result!=DG_RADIAL_IO_OK) { s->fault=1; return layer; }
    if (want) uploaded=o->upload(o->user,s->handle,s->image,scratch,
                                DG_RADIAL_VIEW_PIXELS*4u)==DG_RADIAL_IO_OK;
    if (o->release(o->user,s->handle)!=DG_RADIAL_IO_OK) {
        s->fault=1; return layer;
    }
    s->acquired=0;
    if (want && !uploaded) { s->fault=1; return layer; }
    if (!uploaded) return layer;
    layer.submit=1; layer.handle=s->handle;
    layer.rect_w=layer.rect_h=DG_RADIAL_VIEW_SIZE;
    layer.source_alpha=layer.unpremultiplied_alpha=1;
    layer.both_eyes=layer.view_space=1;
    layer.position_z=-1.2f; layer.orientation_w=1.0f;
    layer.width_m=layer.height_m=0.65f;
    return layer;
}
