static int test_hanging_camera(void)
{
    MAT world,inv,raw,product,head,head_inv,eye;
    POSE p;
    int i,j,k,bad=0;
    double heading;
    for(i=0;i<16;i++) {
        rot_y(&world,0.37);world.m[3][0]=120;world.m[3][1]=350;world.m[3][2]=-50;
        raw=world;camera_inverse_from_world(&inv,&world);
        heading=i*6.283185307179586/16;
        if(!hanging_camera_apply(&world,&inv,1,heading)) bad++;
        if(fabs(world.m[2][0]-sin(heading))>1e-5 ||
           fabs(world.m[2][2]-cos(heading))>1e-5 ||
           memcmp(world.m[3],raw.m[3],sizeof world.m[3])) bad++;
        mat_mul(&product,&inv,&world);
        for(j=0;j<4;j++)for(k=0;k<4;k++)
            if(fabs(product.m[j][k]-(j==k?1:0))>1e-4) bad++;
        /* The physical HMD yaw remains relative to the corrected native base. */
        memset(&p,0,sizeof p);p.yaw=0.2;
        build_delta(&head,&head_inv,&p);mat_mul(&eye,&head_inv,&world);
        if(fabs(eye.m[2][0]-sin(heading-0.2))>1e-5 ||
           fabs(eye.m[2][2]-cos(heading-0.2))>1e-5) bad++;
        raw=world;
        if(hanging_camera_apply(&world,&inv,0,0) || memcmp(&world,&raw,sizeof raw)) bad++;
        if(hanging_camera_apply(&world,&inv,1,NAN) || memcmp(&world,&raw,sizeof raw)) bad++;
    }
    printf("  %-6s hanging camera: 16 actor headings, head yaw, position, inverse and refusal\n",bad?"FAIL":"ok");
    return bad;
}
