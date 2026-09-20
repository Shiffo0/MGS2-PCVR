static int hanging_camera_apply(MAT *world, MAT *inverse, int valid, double heading)
{
    MAT corrected;
    int have=1;
    if (!valid || !_finite(heading)) return 0;
    corrected=*world;
    if (!camera_yaw_anchor_apply(&corrected,&heading,&have)) return 0;
    *world=corrected;
    camera_inverse_from_world(inverse,world);
    return 1;
}
