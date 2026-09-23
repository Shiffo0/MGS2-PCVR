/* VR standing-camera height. Included after the bridge's guarded player walk.
 * Retail 2.1.0.0: CheckSubjectMoveCamera compares StandStill/StandRun at
 * 529914/529924 and uses control.hzx_base + 1500 at 529C4D. CONTROL starts
 * at player+60. CheckLevelHazard stores floor/ceiling at control+B0/B4 and
 * grounded at BA (72A50, 72A88). Use the collision floor, not animation Y.
 * No native player/camera memory is written and no height survives a frame.
 */
static int camera_standing_height_read(ULONGLONG base, ULONGLONG player,
                                       float *height)
{
    static const unsigned char actions[] = {
        0x48,0x8d,0x0d,0x75,0x0e,0xff,0xff, /* StandStill = 51A790 */
        0x48,0x3b,0xc1,0x0f,0x84,0x29,0x03,0,0,
        0x48,0x8d,0x0d,0xf5,0x3c,0x01,0, /* StandRun = 53D620 */
        0x48,0x3b,0xc1,0x0f,0x84,0x19,0x03,0,0
    };
    static const unsigned char floors[] = {
        0xf3,0x0f,0x11,0x9b,0xb0,0,0,0,
        0xf3,0x0f,0x11,0x83,0xb4,0,0,0
    };
    ULONGLONG action;
    float floor, ceiling, wanted;
    if (!height || !plausible_ptr(base) || !plausible_ptr(player) ||
        region_end(base+0x529914) < base+0x529914+sizeof actions ||
        region_end(base+0x72A88) < base+0x72A88+sizeof floors ||
        region_end(player) < player+0xC68 ||
        memcmp((const void *)(ULONG_PTR)(base+0x529914),actions,sizeof actions) ||
        memcmp((const void *)(ULONG_PTR)(base+0x72A88),floors,sizeof floors))
        return 0;
    action = *(const ULONGLONG *)(ULONG_PTR)(player+0xC60);
    if (action != base+0x51A790 && action != base+0x53D620) {
        /* JetSpray/SetMic retain standing stance but own another action.
           Match both action and weapon; demo mic and stance transitions
           must retain their native camera collision/height behavior. */
        LONG weapon;
        static const unsigned char spray[] = {0x4c,0x8b,0xdc,0x57,0x41,0x54,0x41,0x55,
            0x48,0x81,0xec,0x90,0,0,0};
        static const unsigned char mic[] = {0x48,0x89,0x5c,0x24,0x10,0x48,0x89,0x6c,
            0x24,0x18,0x48,0x89,0x74,0x24,0x20,0x57,0x48,0x83,0xec,0x60};
        if(region_end(player)<player+0x13D2 ||
           *(const short *)(ULONG_PTR)(player+0x13D0)!=0)return 0;
        weapon=RD32(player+0xB90);
        if(action==base+0x51F880 && weapon==14) {
            if(region_end(action)<action+sizeof spray ||
               memcmp((const void *)(ULONG_PTR)action,spray,sizeof spray))return 0;
        } else if(action==base+0x51FD40 && weapon==12) {
            if(region_end(action)<action+sizeof mic ||
               memcmp((const void *)(ULONG_PTR)action,mic,sizeof mic))return 0;
        } else return 0;
    }
    if (*(const unsigned char *)(ULONG_PTR)(player+0x11A) != 1) return 0;
    floor = *(const float *)(ULONG_PTR)(player+0x110);
    ceiling = *(const float *)(ULONG_PTR)(player+0x114);
    wanted = floor + 1500.0f;
    /* Low ceilings retain the native camera's collision correction. Also
       refuse missing-floor sentinels, airborne frames and corrupt values. */
    if (!_finite(floor) || !_finite(ceiling) || fabs(floor) >= 1000000.0f ||
        ceiling < wanted+400.0f) return 0;
    *height = wanted;
    return 1;
}

int dg_bridge_camera_standing_height_now(float *height)
{
    DG_CAMERA_GATE gate;
    ULONGLONG arm, player;
    LONG weapon;
    if (!height || !dg_bridge_camera_gate_now(&gate) ||
        !g_b.a.pl_subject_move ||
        region_end(g_b.a.pl_subject_move) < g_b.a.pl_subject_move+4 ||
        !RD32(g_b.a.pl_subject_move) ||
        resolve_motion_player(&arm,&player,&weapon) != DG_RESOLVE_OK ||
        arm != gate.arm_body) return 0;
    return camera_standing_height_read((ULONGLONG)(ULONG_PTR)GetModuleHandleW(NULL),
                                       player,height);
}
