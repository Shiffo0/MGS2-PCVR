static int t_left_model_transport(void)
{
    unsigned char blob[11*64]={0};DG_LEFT_STATE saved=g_left;
    DG_BRIDGE_ARM_TARGET t={0};DG_ADJ_FRAME frame=ADJ_FRAME_LEGACY;
    DG_MODEL_ARM out;double identity[4]={0,0,0,1},elbow[3]={3700,3300,2600},wrist[3]={3700,3100,2600};
    double q[4],basis[3][3];int r,k,bad=0;
#define LM_CHECK(x) do {if(!(x)){bad++;printf("FAIL model arm %d: %s\n",__LINE__,#x);}}while(0)
    memset(&g_left,0,sizeof g_left);
    frame.live=1;frame.valid=arm_frame_yaw(0,frame.q);
    t.left_weight=1;t.stream_id=7;t.pair_id=3;t.left_sample_seq=99;
    t.left_position.enabled=t.left_position.valid=1;t.left_position.units=1000;
    t.left_position.camera[0]=t.left_position.camera[5]=t.left_position.camera[10]=t.left_position.camera[15]=1;
    t.left_position.camera[12]=4000;t.left_position.camera[13]=3000;t.left_position.camera[14]=2000;
    t.left_position.view[3]=1;t.left_position.view[4]=.1;t.left_position.view[5]=1.6;t.left_position.view[6]=.2;
    t.left_position.grip[0]=9; /* unreachable controller: must NOT be radar origin */
    for(r=0;r<2;r++) {
        /* Rotated native bone basis, Y longitudinal in the first pose and
           roll about that bone in the second; fallback surface must survive. */
        th_axis(0,1,0,r?90:0,q);th_write_basis((float *)(blob+9*64),q);
        left_model_publish((ULONGLONG)(ULONG_PTR)blob,64,&frame,&t,identity,identity,0,elbow,wrist);
        LM_CHECK(dg_bridge_model_arm_snapshot(&out));
        LM_CHECK(fabs(out.wrist[0]+.2)<1e-6 && fabs(out.wrist[1]-1.5)<1e-6 && fabs(out.wrist[2]+.4)<1e-6);
        LM_CHECK(fabs(out.elbow[1]-1.3)<1e-6 && out.sequence==99 && out.stream==7);
        LM_CHECK(r?fabs(out.normal[0]-1)<1e-6:fabs(out.normal[2]+1)<1e-6);
    }
    /* Observed hierarchy contains the previous adjustment. Remove that
       before composing this solve, rather than accumulating old arm roll. */
    th_axis(0,1,0,90,q);th_write_basis((float *)(blob+9*64),q);
    for(k=0;k<4;k++){g_left.cached[k]=(float)q[k];g_left.cached[k+4]=(float)identity[k];}
    left_model_publish((ULONGLONG)(ULONG_PTR)blob,64,&frame,&t,identity,identity,1,elbow,wrist);
    LM_CHECK(dg_bridge_model_arm_snapshot(&out) && fabs(out.normal[2]+1)<1e-6);
    /* A nonidentity raw-eye orientation and camera heading use the actual
       rendering inverse, preserving the same world points on round trip. */
    th_axis(0,1,0,75,q);
    {float m[16]={0};th_write_basis(m,q);for(r=0;r<3;r++)for(k=0;k<3;k++)t.left_position.camera[r*4+k]=m[r*4+k];}
    th_axis(1,0,0,25,t.left_position.view);
    left_model_publish((ULONGLONG)(ULONG_PTR)blob,64,&frame,&t,identity,identity,0,elbow,wrist);
    LM_CHECK(dg_bridge_model_arm_snapshot(&out));
    LM_CHECK(dg_position_frame(&t.left_position,q));
    {double delta[3],world[3];for(k=0;k<3;k++)delta[k]=(out.wrist[k]-t.left_position.view[4+k])*1000;
     arm_quat_rotate(q,delta,world);for(k=0;k<3;k++)LM_CHECK(fabs(world[k]+t.left_position.camera[12+k]-wrist[k])<1e-6);}
    t.left_weight=.5;left_model_publish((ULONGLONG)(ULONG_PTR)blob,64,&frame,&t,identity,identity,0,elbow,wrist);
    LM_CHECK(!dg_bridge_model_arm_snapshot(&out));
    t.left_weight=1;t.left_position.valid=0;
    left_model_publish((ULONGLONG)(ULONG_PTR)blob,64,&frame,&t,identity,identity,0,elbow,wrist);
    LM_CHECK(!dg_bridge_model_arm_snapshot(&out));
    (void)basis;g_left=saved;dg_bridge_model_arm_clear();
    printf("  %s model arm: rendered/clamped endpoint inverse, translated/yawed camera and raw head, roll and longitudinal-axis fallback, invalidation\n",bad?"FAIL":"ok");
#undef LM_CHECK
    return bad!=0;
}
