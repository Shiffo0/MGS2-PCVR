static int t_unarmed_prone_tables(void)
{
    const DWORD size=0x981000;
    unsigned char *bytes=(unsigned char *)VirtualAlloc(NULL,size,MEM_RESERVE|MEM_COMMIT,PAGE_READWRITE);
    unsigned char *valid=(unsigned char *)malloc(size),*original=(unsigned char *)malloc(size);
    LiveImage im;
    ULONGLONG saved_anchor=g_b.a.arm_cam_rotate_shift;
    int bad=0,i,hand;
    DWORD j;
#define UP_CHECK(x) do {if(!(x)){bad++;printf("FAIL unarmed prone %d: %s\n",__LINE__,#x);}}while(0)
    if(!bytes || !valid || !original) {if(bytes)VirtualFree(bytes,0,MEM_RELEASE);free(valid);free(original);return 1;}
    memset(&im,0,sizeof im);im.base=(ULONGLONG)(ULONG_PTR)bytes;
    im.bytes=bytes;im.valid=valid;im.size=size;memset(valid,1,size);
    for(i=0;i<4;i++)memcpy(bytes+(i<2?unarmed_prone_standing[i]:unarmed_prone_ground[i-2]),unarmed_prone_witness[i],32);
    memcpy(original,bytes,size);g_b.a.arm_cam_rotate_shift=im.base+0x100;
    unarmed_prone_resolve(&im);UP_CHECK(g_unarmed_prone.resolved);
    unarmed_prone_install(0);UP_CHECK(!memcmp(bytes,original,size));
    unarmed_prone_install(1);
    UP_CHECK(g_unarmed_prone.owned[0] && g_unarmed_prone.owned[1]);
    for(i=0;i<2;i++)UP_CHECK(!memcmp(bytes+unarmed_prone_ground[i],bytes+unarmed_prone_standing[i],16));
    for(j=0;j<size;j++)if(!((j>=unarmed_prone_ground[0]&&j<unarmed_prone_ground[0]+16)||
                          (j>=unarmed_prone_ground[1]&&j<unarmed_prone_ground[1]+16)))
        if(bytes[j]!=original[j]){UP_CHECK(0);break;}

    for(i=0;i<2;i++)for(hand=-1;hand<=1;hand+=2) {
        DG_ARM_MAP_STATE state={0};DG_ARM_MAP_IN in={0};DG_ARM_MAP_OUT out;
        const float *standing=(const float *)(bytes+unarmed_prone_standing[i]);
        const float *ground=(const float *)(bytes+unarmed_prone_ground[i]);
        double desired[3]={hand*220.0,-1140,35};
        in.anchor_shoulder=1;in.player_reach=330;in.upper=180;in.fore=150;
        in.controller_view[0]=hand*220.0;in.controller_view[1]=-140;in.controller_view[2]=35;
        in.native_wrist_view[0]=hand*270.0;in.native_wrist_view[1]=-120;
        UP_CHECK(!dg_arm_map_step(&state,&in,&out) && (out.flags&DG_ARM_MAP_F_CALIBRATED));
        in.explicit_target=1;memcpy(in.desired_view,desired,sizeof desired);
        in.shoulder_view[1]=-1000+standing[1];in.shoulder_view[2]=standing[2];
        UP_CHECK(dg_arm_map_step(&state,&in,&out));
        UP_CHECK((out.flags&DG_ARM_MAP_F_CLAMP_REACH) && out.target_view[1]>desired[1]+100);
        in.shoulder_view[1]=-1000+standing[1]-ground[1];
        in.shoulder_view[2]=standing[2]-ground[2];
        UP_CHECK(dg_arm_map_step(&state,&in,&out));
        UP_CHECK(vdist3(out.target_view,desired)<.001);
    }
    unarmed_prone_stop();UP_CHECK(!memcmp(bytes,original,size));
    for(i=0;i<4;i++) {
        DWORD rva=i<2?unarmed_prone_standing[i]:unarmed_prone_ground[i-2];
        bytes[rva+20]^=1;unarmed_prone_resolve(&im);UP_CHECK(!g_unarmed_prone.resolved);
        unarmed_prone_install(1);bytes[rva+20]^=1;UP_CHECK(!memcmp(bytes,original,size));
    }
    g_b.a.arm_cam_rotate_shift=0;unarmed_prone_resolve(&im);UP_CHECK(!g_unarmed_prone.resolved);
    g_b.a.arm_cam_rotate_shift=im.base+0x100;
    valid[unarmed_prone_ground[1]]=0;unarmed_prone_resolve(&im);UP_CHECK(!g_unarmed_prone.resolved);
    valid[unarmed_prone_ground[1]]=1;
    /* A foreign edit before install cannot leave a partial first patch. */
    unarmed_prone_resolve(&im);((float *)(bytes+unarmed_prone_ground[1]))[1]=17;
    unarmed_prone_install(1);UP_CHECK(!g_unarmed_prone.owned[0]&&!g_unarmed_prone.owned[1]);
    UP_CHECK(!memcmp(bytes+unarmed_prone_ground[0],original+unarmed_prone_ground[0],16));
    UP_CHECK(((float *)(bytes+unarmed_prone_ground[1]))[1]==17);
    memcpy(bytes,original,size);unarmed_prone_resolve(&im);unarmed_prone_install(1);
    ((float *)(bytes+unarmed_prone_ground[0]))[1]=23;
    unarmed_prone_stop();UP_CHECK(((float *)(bytes+unarmed_prone_ground[0]))[1]==23);
    UP_CHECK(!memcmp(bytes+unarmed_prone_ground[1],original+unarmed_prone_ground[1],16));
    g_b.a.arm_cam_rotate_shift=saved_anchor;memset(&g_unarmed_prone,0,sizeof g_unarmed_prone);
    VirtualFree(bytes,0,MEM_RELEASE);free(valid);free(original);
    printf("  %s unarmed prone: Snake/Raiden full-height/depth geometry, exact None slots, signature refusal and ownership restore\n",bad?"FAIL":"ok");
#undef UP_CHECK
    return bad!=0;
}
