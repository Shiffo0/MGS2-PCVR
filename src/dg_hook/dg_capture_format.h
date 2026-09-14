/* CopyResource compatibility for the color textures used by this bridge.
 * Concrete XR swapchain format is not the resource's GetDesc().Format.
 * Microsoft permits copies within a DXGI typeless family. Keep unrelated
 * layouts (in particular BGRA versus RGBA/BGRX) distinct.
 */
#ifndef DG_CAPTURE_FORMAT_H
#define DG_CAPTURE_FORMAT_H
static int dg_capture_format_group(DXGI_FORMAT f) {
    switch (f) {
    case DXGI_FORMAT_B8G8R8A8_TYPELESS:
    case DXGI_FORMAT_B8G8R8A8_UNORM:
    case DXGI_FORMAT_B8G8R8A8_UNORM_SRGB:
        return 1;
    case DXGI_FORMAT_R8G8B8A8_TYPELESS:
    case DXGI_FORMAT_R8G8B8A8_UNORM:
    case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB:
        return 2;
    default: return 0;
    }
}
static int dg_capture_format_compatible(DXGI_FORMAT src, DXGI_FORMAT dst) {
    int group;
    if (src == DXGI_FORMAT_UNKNOWN || dst == DXGI_FORMAT_UNKNOWN) return 0;
    if (src == dst) return 1;
    group = dg_capture_format_group(src);
    return group != 0 && group == dg_capture_format_group(dst);
}
#endif
