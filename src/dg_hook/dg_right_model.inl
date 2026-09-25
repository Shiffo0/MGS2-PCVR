/* Read the observed right elbow/wrist, not the next commanded solve.
 * Joint 5 is forearm, joint 6 wrist. No writes to either native matrix. */
static void right_model_publish(ULONGLONG base,int stride,const DG_ADJ_FRAME *frame,
 const DG_BRIDGE_ARM_TARGET *t,const double qa[4],const double qb[4],
 int had_active,const double predicted_elbow[3],const double predicted_wrist[3]) {
 DG_MODEL_ARM out={0};double rows[3][3],live[4],mapping[4],inv[4],normal[3],along[3],axis[3]={0,0,1};
 double points[2][3],v[3],length=0,dot=0;int r,k;
 (void)frame;(void)qa;(void)qb;(void)had_active;(void)predicted_elbow;(void)predicted_wrist;
 if(t->weight<.999||!t->position.enabled||!dg_position_frame(&t->position,mapping))goto refuse;
 for(r=0;r<3;r++)for(k=0;k<3;k++)rows[r][k]=((volatile float *)(ULONG_PTR)(base+5*(ULONGLONG)stride))[r*4+k];
 if(!dg_ik_basis_quat(rows,live))goto refuse;
 for(r=0;r<2;r++)for(k=0;k<3;k++)points[r][k]=((volatile float *)(ULONG_PTR)(base+(5+r)*(ULONGLONG)stride))[12+k];
 for(k=0;k<3;k++){along[k]=points[1][k]-points[0][k];length+=along[k]*along[k];}
 length=sqrt(length);if(!isfinite(length)||length<1||length>1000)goto refuse;
 arm_quat_rotate(live,axis,normal);
 for(k=0;k<3;k++)dot+=normal[k]*along[k]/length;
 if(fabs(dot)>.9){axis[2]=0;axis[1]=1;arm_quat_rotate(live,axis,normal);}
 dg_ik_quat_conj(mapping,inv);
 for(r=0;r<2;r++){
  double local[3];for(k=0;k<3;k++)v[k]=(points[r][k]-t->position.camera[12+k])/t->position.units;
  arm_quat_rotate(inv,v,local);for(k=0;k<3;k++)(r?out.wrist:out.elbow)[k]=local[k]+t->position.view[4+k];
 }
 arm_quat_rotate(inv,normal,out.normal);
 out.valid=1;out.ms=GetTickCount64();out.sequence=t->aim_sample_seq;out.stream=t->stream_id;out.pair=t->pair_id;
 dg_bridge_right_model_arm_publish(&out);return;
refuse:dg_bridge_right_model_arm_clear();
}
