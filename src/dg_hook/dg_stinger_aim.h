#ifndef DG_STINGER_AIM_H
#define DG_STINGER_AIM_H

static int dg_stinger_delta(const float root[4][4], const float projection[4][4],
                            const float target[4], const float display[2], float delta[2]) {
    double origin[3],forward[3],point[4],clip[4]={0},target_w=0,depth=0,norm=0;
    int i,j;
    delta[0]=delta[1]=1000000.f;
    for(i=0;i<3;i++) {
        origin[i]=root[3][i]+17.5*root[0][i]-641.0*root[1][i]+112.7*root[2][i];
        forward[i]=-root[1][i];norm+=forward[i]*forward[i];
        if(!_finite(origin[i]) || !_finite(target[i]))return 0;
    }
    if(!_finite(norm) || norm<.9 || norm>1.1)return 0;
    norm=sqrt(norm);
    for(i=0;i<3;i++){forward[i]/=norm;depth+=(target[i]-origin[i])*forward[i];}
    if(!_finite(depth) || depth<=1)return 0;
    for(i=0;i<3;i++)point[i]=origin[i]+depth*forward[i];
    point[3]=1;
    for(i=0;i<4;i++)for(j=0;j<4;j++)clip[j]+=point[i]*projection[i][j];
    for(i=0;i<4;i++)target_w+=(i==3 ? 1:target[i])*projection[i][3];
    if(!_finite(clip[3]) || fabs(clip[3])<.001 || clip[3]*target_w<=0)return 0;
    delta[0]=(float)(256.0*(clip[0]/fabs(clip[3])+1)-display[0]);
    delta[1]=(float)(192.0*(clip[1]/fabs(clip[3])+1)-display[1]);
    if(!_finite(delta[0]) || !_finite(delta[1])){delta[0]=delta[1]=1000000.f;return 0;}
    return 1;
}
#endif
