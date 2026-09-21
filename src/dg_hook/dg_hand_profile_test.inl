static int t_persistent_hand_profile(void) {
    DG_HAND_PROFILE p,saved_profile,loaded;
    DG_FREE_WRIST_INPUT in={0};DG_FREE_WRIST_STATE state={0};
    DG_POSITION_INPUT pos={0};DG_BRIDGE_ARM_TARGET t={0};
    double rest[4],out[4],want[4],turn[4],world[4],point[3],before[3];
    unsigned long saved_revision=g_hand_profile_revision,saved_seen=g_hand_calibration_seen;
    int saved_initialized=g_hand_profile_initialized,saved_unarmed=g_b.arm_map_unarmed;
    int bad=0,h,i,j;wchar_t path[MAX_PATH],dir[MAX_PATH];
#define HP_CHECK(x) do {if(!(x)){bad++;printf("FAIL persistent hands %d: %s\n",__LINE__,#x);}}while(0)
    saved_profile=g_hand_profile;
    { /* Actual gesture admission: no queued calibration after a scene or
         tracking gap; one press stays pending only in the same safe stream. */
        DG_HAND_REQUEST request={0};
        HP_CHECK(!dg_hand_request_step(&request,5,1,1,0));
        HP_CHECK(dg_hand_request_step(&request,6,1,1,10)==6);
        HP_CHECK(dg_hand_request_step(&request,6,1,1,50)==6);
        HP_CHECK(!dg_hand_request_step(&request,6,0,1,60));
        HP_CHECK(!dg_hand_request_step(&request,7,1,1,70));
        HP_CHECK(!dg_hand_request_step(&request,8,1,1,1000));
        HP_CHECK(!dg_hand_request_step(&request,9,1,2,1010));
        HP_CHECK(dg_hand_request_step(&request,10,1,2,1020)==10);
        HP_CHECK(!dg_hand_request_step(&request,10,1,2,3021));
    }
    dg_hand_profile_default(&p);HP_CHECK(dg_hand_profile_valid(&p));
    in.enabled=in.valid=in.persistent=1;in.world[3]=1;
    for(h=0;h<2;h++) {
        memcpy(in.alignment,p.rotation[h],sizeof in.alignment);
        for(i=0;i<24;i++) {
            /* Disposable cache/animation changes cannot affect the default. */
            memset(&state,0,sizeof state);
            th_axis(1,0,0,i*13,rest);th_axis(0,1,0,i*11,in.world);
            dg_ik_quat_mul(in.world,in.alignment,want);
            HP_CHECK(dg_free_wrist_step(&state,&in,rest,out));
            HP_CHECK(th_angle_between(out,want)<.001);
            in.valid=0;HP_CHECK(!dg_free_wrist_step(&state,&in,rest,out));in.valid=1;
            HP_CHECK(dg_free_wrist_step(&state,&in,rest,out));
            HP_CHECK(th_angle_between(out,want)<.001);
        }
        /* Saved grip-local offset follows changed camera/room frames. */
        th_axis(1,0,0,23,rest);th_axis(0,0,1,17,in.world);
        HP_CHECK(dg_hand_profile_capture(&in,rest,p.rotation[h]));
        memcpy(in.alignment,p.rotation[h],sizeof in.alignment);
        memcpy(world,in.world,sizeof world);
        for(i=0;i<12;i++) {
            th_axis(0,1,0,i*30,turn);dg_ik_quat_mul(turn,world,in.world);
            dg_ik_quat_mul(turn,rest,want);memset(&state,0,sizeof state);
            HP_CHECK(dg_free_wrist_step(&state,&in,rest,out));
            HP_CHECK(th_angle_between(out,want)<.001);
        }
    }
    pos.enabled=pos.valid=1;pos.units=1000;pos.view[3]=1;
    for(i=0;i<4;i++)pos.camera[i*5]=1;
    pos.grip[0]=.15;pos.grip[1]=-.2;pos.grip[2]=-.35;
    HP_CHECK(dg_hand_profile_position(&pos,before));
    HP_CHECK(fabs(before[0]-150)<.001 && fabs(before[1]-200)<.001 && fabs(before[2]-350)<.001);
    /* New scene translation, head/eye translation, invalid frame. */
    for(i=0;i<3;i++)pos.camera[12+i]=100*(i+1);
    HP_CHECK(dg_hand_profile_position(&pos,point));
    for(i=0;i<3;i++)HP_CHECK(fabs(point[i]-before[i]-100*(i+1))<.001);
    pos.view[4]=.032;pos.camera[12]+=.032*1000;
    HP_CHECK(dg_hand_profile_position(&pos,point));HP_CHECK(fabs(point[0]-before[0]-100)<.001);
    pos.valid=0;HP_CHECK(!dg_hand_profile_position(&pos,point));
    /* Real staging/commit: neither partial, armed nor refused samples save. */
    dg_bridge_hand_profile_get(&loaded);
    t.persistent_hands=1;t.hand_calibration_request=0x12345678;t.weight=t.left_weight=1;
    g_hand_calibration_seen=0;g_b.arm_map_unarmed=1;
    memset(&g_hand_capture,0,sizeof g_hand_capture);
    hand_profile_stage(0,&t,&in,rest);hand_profile_commit(&t,1);
    HP_CHECK(g_hand_profile_revision==saved_revision);
    hand_profile_stage(1,&t,&in,rest);hand_profile_commit(&t,0);
    HP_CHECK(g_hand_profile_revision==saved_revision);
    hand_profile_commit(&t,1);HP_CHECK(g_hand_profile_revision==saved_revision+1);
    memset(&g_hand_capture,0,sizeof g_hand_capture);
    for(h=0;h<2;h++)hand_profile_stage(h,&t,&in,rest);
    hand_profile_commit(&t,1);HP_CHECK(g_hand_profile_revision==saved_revision+1);
    t.hand_calibration_request++;g_b.arm_map_unarmed=0;
    for(h=0;h<2;h++)hand_profile_stage(h,&t,&in,rest);
    hand_profile_commit(&t,1);HP_CHECK(g_hand_profile_revision==saved_revision+1);
    /* File round trip, replacement, truncation, corruption, unknown version.
       Only an explicitly created temp file is touched by the desk test. */
    HP_CHECK(GetTempPathW(MAX_PATH,dir)>0);
    HP_CHECK(GetTempFileNameW(dir,L"dgh",0,path)!=0);
    HP_CHECK(hand_profile_write(path,&p));
    memset(&loaded,0,sizeof loaded);HP_CHECK(hand_profile_read(path,&loaded)==1);
    HP_CHECK(!memcmp(&p,&loaded,sizeof p));
    dg_hand_profile_default(&p);HP_CHECK(hand_profile_write(path,&p));
    HP_CHECK(hand_profile_read(path,&loaded)==1 && !memcmp(&p,&loaded,sizeof p));
    for(j=0;j<3;j++) {
        FILE *f;DG_HAND_PROFILE_FILE disk={0};
        disk.magic=0x48504744u;disk.version=j==2?2:1;disk.profile=p;
        disk.checksum=hand_profile_checksum(&disk)+(j==1?1:0);
        HP_CHECK(!_wfopen_s(&f,path,L"wb"));
        if(f){fwrite(&disk,1,j==0?7:sizeof disk,f);fclose(f);}
        loaded=p;HP_CHECK(hand_profile_read(path,&loaded)==-1);
        HP_CHECK(!memcmp(&p,&loaded,sizeof p));
    }
    HP_CHECK(DeleteFileW(path));
    p.rotation[0][0]=NAN;HP_CHECK(!dg_hand_profile_valid(&p));
    g_hand_profile=saved_profile;g_hand_profile_revision=saved_revision;
    g_hand_profile_initialized=saved_initialized;g_hand_calibration_seen=saved_seen;
    g_b.arm_map_unarmed=saved_unarmed;memset(&g_hand_capture,0,sizeof g_hand_capture);
    printf("  %s persistent hands: defaults, tracking/scene frames, paired manual commit, file round trip/corruption\n",bad?"FAIL":"ok");
#undef HP_CHECK
    return bad;
}
