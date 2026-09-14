/* Native-layout and refusal checks; no game process is used. */
static int t_camera_standing_height(void)
{
    unsigned char *image = (unsigned char *)VirtualAlloc(NULL,0x540000,
        MEM_RESERVE|MEM_COMMIT,PAGE_READWRITE);
    unsigned char player[0xE00];
    ULONGLONG base = (ULONGLONG)(ULONG_PTR)image;
    ULONGLONG p = (ULONGLONG)(ULONG_PTR)player;
    static const unsigned char actions[32] = {
        0x48,0x8d,0x0d,0x75,0x0e,0xff,0xff,0x48,0x3b,0xc1,
        0x0f,0x84,0x29,0x03,0,0,0x48,0x8d,0x0d,0xf5,0x3c,1,0,
        0x48,0x3b,0xc1,0x0f,0x84,0x19,3,0,0
    };
    static const unsigned char floors[16] = {
        0xf3,0x0f,0x11,0x9b,0xb0,0,0,0,0xf3,0x0f,0x11,0x83,0xb4,0,0,0
    };
    float height;
    int i, bad=0;
    if (!image) return 1;
    memset(player,0,sizeof player);
    memcpy(image+0x529914,actions,sizeof actions);
    memcpy(image+0x72A88,floors,sizeof floors);
    *(ULONGLONG *)(player+0xC60)=base+0x53D620;
    player[0x11A]=1;
    *(float *)(player+0x110)=120.0f;
    *(float *)(player+0x114)=5000.0f;
    if (!camera_standing_height_read(base,p,&height) || height!=1620.0f) bad++;
    /* Native model/base heights may bob independently of the collision floor. */
    for (i=0;i<120;i++) {
        *(float *)(player+0x9C)=120.0f+(float)(40.0*sin(i*0.4));
        *(float *)(player+0xD14)=1600.0f+(float)(60.0*sin(i*0.4));
        if (!camera_standing_height_read(base,p,&height) || height!=1620.0f) bad++;
    }
    *(ULONGLONG *)(player+0xC60)=base+0x51A790; /* still */
    *(float *)(player+0x110)=420.0f; /* new floor/elevator height */
    if (!camera_standing_height_read(base,p,&height) || height!=1920.0f) bad++;
    for (i=0;i<8;i++) {
        *(ULONGLONG *)(player+0xC60)=base+0x53D620;
        player[0x11A]=1;
        *(float *)(player+0x110)=120.0f;
        *(float *)(player+0x114)=5000.0f;
        switch(i) {
        case 0: *(ULONGLONG *)(player+0xC60)=base+0x51AC20; break; /* squat */
        case 1: player[0x11A]=0; break; /* airborne */
        case 2: player[0x11A]=2; break; /* ceiling contact */
        case 3: *(float *)(player+0x114)=1800.0f; break;
        case 4: *(float *)(player+0x110)=-1000000.0f; break;
        case 5: *(unsigned int *)(player+0x110)=0x7fc00000; break;
        case 6: image[0x529914]^=1; break;
        case 7: image[0x72A88]^=1; break;
        }
        height=123.0f;
        if (camera_standing_height_read(base,p,&height) || height!=123.0f) bad++;
        memcpy(image+0x529914,actions,sizeof actions);
        memcpy(image+0x72A88,floors,sizeof floors);
    }
    if (camera_standing_height_read(base,0,&height) ||
        camera_standing_height_read(base,p,NULL)) bad++;
    VirtualFree(image,0,MEM_RELEASE);
    printf("  %-6s camera step height: floor follows terrain; animation Y ignored; squat, air, low ceiling, sentinel, NaN and layout refusals\n",bad?"FAIL":"ok");
    return bad;
}
