/* Prediction uses exactly this solve's chain and actual clamped joints,
   rather than the raw controller target. Arm q8/q9 affect the forearm;
   independent q10 wrist rotation does not change its roll. */
static void left_model_publish(ULONGLONG base,int stride,const DG_ADJ_FRAME *frame,
    const DG_BRIDGE_ARM_TARGET *t,const double qa[4],const double qb[4],
    int had_active,const double elbow[3],const double wrist[3])
{
    DG_MODEL_ARM out={0};
    double rows[3][3],live[4],old_up[4],old_fore[4],old_chain[4],inv[4];
    double up[4],fore[4],chain[4],native[4],solved[4],mapping[4],normal[3],v[3];
    double a[4],b[4],axis[3]={0,1,0},direction[3],length,dot;int r,k;
    if(t->left_weight<.999 || !t->left_position.enabled ||
       !dg_position_frame(&t->left_position,mapping))goto refuse;
    for(r=0;r<3;r++)for(k=0;k<3;k++)
        rows[r][k]=((volatile float *)(ULONG_PTR)(base+9*(ULONGLONG)stride))[r*4+k];
    if(!dg_ik_basis_quat(rows,live))goto refuse;
    for(k=0;k<4;k++) {
        a[k]=had_active?g_left.cached[k]:(k==3);
        b[k]=had_active?g_left.cached[k+4]:(k==3);
    }
    if(!adjust_quat_to_world(frame,a,old_up) || !adjust_quat_to_world(frame,b,old_fore) ||
       !adjust_quat_to_world(frame,qa,up) || !adjust_quat_to_world(frame,qb,fore))goto refuse;
    dg_ik_quat_mul(old_fore,old_up,old_chain);dg_ik_quat_conj(old_chain,inv);
    dg_ik_quat_mul(inv,live,native);dg_ik_quat_mul(fore,up,chain);
    dg_ik_quat_mul(chain,native,solved);arm_quat_rotate(solved,axis,normal);
    for(k=0;k<3;k++)direction[k]=wrist[k]-elbow[k];
    length=sqrt(direction[0]*direction[0]+direction[1]*direction[1]+direction[2]*direction[2]);
    if(!isfinite(length)||length<1)goto refuse;
    dot=0;for(k=0;k<3;k++)dot+=normal[k]*direction[k]/length;
    /* Some native rigs use Y as the longitudinal bone axis. Their Z axis
       supplies the surface reference instead; never synthesize a head-facing
       normal or lose roll by using a fixed world-up vector. */
    if(fabs(dot)>.9) {axis[1]=0;axis[2]=1;arm_quat_rotate(solved,axis,normal);}
    dg_ik_quat_conj(mapping,inv);
    for(r=0;r<2;r++) {
        double local[3];const double *point=r?wrist:elbow;
        for(k=0;k<3;k++)v[k]=(point[k]-t->left_position.camera[12+k])/t->left_position.units;
        arm_quat_rotate(inv,v,local);
        for(k=0;k<3;k++)(r?out.wrist:out.elbow)[k]=local[k]+t->left_position.view[4+k];
    }
    arm_quat_rotate(inv,normal,out.normal);
    out.valid=1;out.ms=GetTickCount64();out.sequence=t->left_sample_seq;
    out.stream=left_calibration_stream(t);out.pair=t->pair_id;
    dg_bridge_model_arm_publish(&out);return;
refuse:
    dg_bridge_model_arm_clear();
}
