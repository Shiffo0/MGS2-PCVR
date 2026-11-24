/* Read-only retail EVM witness. A palette translation is NOT a joint position:
 * undo the inverse bind by applying the palette to the bind-space joint. */
static int evm_joint_point(const float *matrix,const float *bind,double out[3]) {
    double point[3];int k,r;
    if(!matrix||!bind||!out)return 0;
    for(k=0;k<16;k++)if(!_finite(matrix[k])||fabs(matrix[k])>1e9)return 0;
    for(k=0;k<3;k++)if(!_finite(bind[k])||fabs(bind[k])>100000)return 0;
    for(k=0;k<3;k++) {
        point[k]=matrix[12+k];
        for(r=0;r<3;r++)point[k]+=bind[r]*matrix[4*r+k];
        if(!_finite(point[k])||fabs(point[k])>1e9)return 0;
    }
    memcpy(out,point,sizeof point);return 1;
}
static int evm_witness_layout(void) {
    ULONGLONG image=(ULONGLONG)(ULONG_PTR)GetModuleHandleW(NULL);

    return image && region_end(image+0x6820c2)>=image+0x68214c &&
        !memcmp((void *)(ULONG_PTR)(image+0x6820c2),"\x48\x63\x82\x80\0\0\0",7) &&
        !memcmp((void *)(ULONG_PTR)(image+0x6820da),"\x48\x8b\x9c\xc2\x88\0\0\0",8) &&
        !memcmp((void *)(ULONG_PTR)(image+0x68211e),"\x49\x8b\x76\x50",4) &&
        !memcmp((void *)(ULONG_PTR)(image+0x68212a),"\x41\x39\x6e\x78",4) &&
        !memcmp((void *)(ULONG_PTR)(image+0x682148),"\x48\x83\xc6\x58",4) &&
        region_end(image+0x682286)>=image+0x68228a &&
        !memcmp((void *)(ULONG_PTR)(image+0x682286),"\x48\x83\xc6\x40",4);
}
static void left_evm_observe(ULONGLONG arm,const DG_BRIDGE_ARM_TARGET *t) {
    static uint64_t last[2];static unsigned count[2];
    ULONGLONG evm,objs,def,palette,base;int weapon,slot,n,buffer,j,k,stride=g_b.skel_stride;
    const int joints[2]={6,10};double position[2][3],gap[2];uint64_t now;
    if(!g_b.log||!t)return;
    if(t->aim_weapon_id!=0&&t->aim_weapon_id!=13)return;
    weapon=(int)t->aim_weapon_id;slot=weapon==13;
    now=GetTickCount64();if(count[slot]>=80||now-last[slot]<500)return;
    if(!plausible_ptr(arm)||region_end(arm)<arm+0x38||!evm_witness_layout())return;
    evm=*(ULONGLONG *)(ULONG_PTR)(arm+0x30);objs=*(ULONGLONG *)(ULONG_PTR)arm;
    if(!plausible_ptr(evm)||region_end(evm)<evm+0x98||!plausible_ptr(objs)||stride<64||stride>4096)return;
    n=RD32(evm+0x78);buffer=RD32(evm+0x80);
    if(n<11||n>255||(buffer!=0&&buffer!=1))return;
    def=*(ULONGLONG *)(ULONG_PTR)(evm+0x50);
    palette=*(ULONGLONG *)(ULONG_PTR)(evm+0x88+buffer*8);base=objs+DG_OBJS_ARRAY;
    if(!plausible_ptr(def)||region_end(def)<def+0x40+n*0x40ULL ||
       !plausible_ptr(palette)||region_end(palette)<palette+n*64ULL ||
       region_end(objs)<base+10*(ULONGLONG)stride+64)return;
    for(j=0;j<2;j++) {
        float *m=(float *)(ULONG_PTR)(palette+joints[j]*64ULL);
        float *bind=(float *)(ULONG_PTR)(def+0x54+joints[j]*0x40ULL);
        float *bone=(float *)(ULONG_PTR)(base+joints[j]*(ULONGLONG)stride);
        double bone_point[3];
        if(!looks_like_matrix((ULONGLONG)(ULONG_PTR)bone)||!evm_joint_point(m,bind,position[j]))return;
        for(k=0;k<3;k++)bone_point[k]=bone[12+k];
        gap[j]=vdist3(position[j],bone_point);
    }
    last[slot]=now;count[slot]++;
    g_b.log("  hand evm witness: weapon %d pair %lu brace %d active %d buffer %d joints %d flags %08X root %llX right_gap_mm %.2f left_gap_mm %.2f left_skin [%.2f %.2f %.2f]\r\n",
        weapon,t->pair_id,dg_bridge_blade_braced(),g_left.active,buffer,n,(unsigned)RD32(evm+0x58),
        *(ULONGLONG *)(ULONG_PTR)(evm+0x40),gap[0],gap[1],position[1][0],position[1][1],position[1][2]);
}
