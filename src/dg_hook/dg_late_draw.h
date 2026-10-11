#pragma once
/* Selection is not visibility proof: only output readback establishes contribution. */
static const char* lateTarget(ID3D11DeviceContext* c, UINT count, bool indexed, bool gameplay, bool plain=false) {
 if(indexed==plain || count<3) return plain?"not_plain_geometry":"not_indexed_geometry";
 if(!gameplay) return "not_fresh_gameplay";
 UINT nv=1; D3D11_VIEWPORT v{}; c->RSGetViewports(&nv,&v);
 if(nv!=1 || v.TopLeftX!=0 || v.TopLeftY!=0 || v.Width<64 || v.Height<64) return "viewport";
 ComPtr<ID3D11RenderTargetView> rt; ComPtr<ID3D11DepthStencilView> ds;
 c->OMGetRenderTargets(1,&rt,&ds); if(!rt || !ds) return "missing_color_depth";
 D3D11_RENDER_TARGET_VIEW_DESC rv{};rt->GetDesc(&rv);
 /* Single-sample 2D or multisampled 2D (the game's main pass is MSAA4). */
 if(!(rv.ViewDimension==D3D11_RTV_DIMENSION_TEXTURE2DMS || (rv.ViewDimension==D3D11_RTV_DIMENSION_TEXTURE2D && !rv.Texture2D.MipSlice)))return "rtv_subresource";
 D3D11_DEPTH_STENCIL_VIEW_DESC dv{};ds->GetDesc(&dv);
 if(dv.Flags || !(dv.ViewDimension==D3D11_DSV_DIMENSION_TEXTURE2DMS || (dv.ViewDimension==D3D11_DSV_DIMENSION_TEXTURE2D && !dv.Texture2D.MipSlice)))return "dsv_subresource";
 ComPtr<ID3D11Resource> res; ComPtr<ID3D11Texture2D> tex;
 rt->GetResource(&res); if(FAILED(res.As(&tex))) return "not_texture";
 D3D11_TEXTURE2D_DESC td{};tex->GetDesc(&td);
 if(v.Width!=td.Width || v.Height!=td.Height) return "partial_viewport";

 ComPtr<ID3D11DepthStencilState> depth;UINT stencil=0;c->OMGetDepthStencilState(&depth,&stencil);
 if(depth){D3D11_DEPTH_STENCIL_DESC d{};depth->GetDesc(&d);if(!d.DepthEnable||d.DepthWriteMask!=D3D11_DEPTH_WRITE_MASK_ALL||d.DepthFunc==D3D11_COMPARISON_NEVER)return "depth_disabled";}
 ComPtr<ID3D11BlendState> blend;FLOAT factor[4];UINT mask=0;c->OMGetBlendState(&blend,factor,&mask);
 if(!mask)return "sample_mask_zero";
 if(blend){D3D11_BLEND_DESC d{};blend->GetDesc(&d);if(!d.RenderTarget[0].RenderTargetWriteMask)return "color_writes_zero";}
 ComPtr<ID3D11RasterizerState> raster;c->RSGetState(&raster);
 if(raster){D3D11_RASTERIZER_DESC d{};raster->GetDesc(&d);if(d.ScissorEnable){UINT nr=1;D3D11_RECT rect{};c->RSGetScissorRects(&nr,&rect);if(nr!=1||rect.left>=rect.right||rect.top>=rect.bottom||rect.right<=0||rect.bottom<=0||rect.left>=v.Width||rect.top>=v.Height)return "empty_scissor";}}
 return nullptr;
}

// Called only for the first occurrence of a selection refusal (bounded by reason buckets).








