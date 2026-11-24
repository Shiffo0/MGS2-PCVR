// Read-only binding metadata. Descriptor equality does NOT imply equal contents.
#pragma once
struct TrialVertexBindings {
 ComPtr<ID3D11Buffer> buffers[32];UINT strides[32]{},offsets[32]{};
 D3D11_BUFFER_DESC descriptions[32]{};
 void reset(){for(auto&b:buffers)b.Reset();memset(strides,0,sizeof strides);memset(offsets,0,sizeof offsets);memset(descriptions,0,sizeof descriptions);}
 void capture(ID3D11DeviceContext*c){
  reset();ID3D11Buffer* raw[32]{};c->IAGetVertexBuffers(0,32,raw,strides,offsets);
  for(unsigned i=0;i<32;i++){buffers[i].Attach(raw[i]);if(raw[i])raw[i]->GetDesc(&descriptions[i]);}
 }
 unsigned differences(const TrialVertexBindings&b,unsigned i)const {
  return (buffers[i].Get()!=b.buffers[i].Get()?1u:0u)|(strides[i]!=b.strides[i]?2u:0u)|(offsets[i]!=b.offsets[i]?4u:0u);
 }
 void report(const TrialVertexBindings&b,void(*out)(const char*,...))const {
  if(!out)return;
  for(unsigned i=0;i<32;i++){unsigned mask=differences(b,i);if(!mask)continue;auto&a=descriptions[i];auto&z=b.descriptions[i];
   out("draw_trial: vertex_binding slot=%u difference_mask=%u buffer=%p/%p stride=%u/%u offset=%u/%u byte_width=%u/%u usage=%u/%u bind=0x%X/0x%X cpu=0x%X/0x%X misc=0x%X/0x%X structure_stride=%u/%u contents=unmeasured\r\n",i,mask,buffers[i].Get(),b.buffers[i].Get(),strides[i],b.strides[i],offsets[i],b.offsets[i],a.ByteWidth,z.ByteWidth,(unsigned)a.Usage,(unsigned)z.Usage,a.BindFlags,z.BindFlags,a.CPUAccessFlags,z.CPUAccessFlags,a.MiscFlags,z.MiscFlags,a.StructureByteStride,z.StructureByteStride);
  }
 }
};
