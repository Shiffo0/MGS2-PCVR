static int test_reload_config(void) {
    const char *words[]={"on","ON","off","onjunk","","1"};int i,bad=0;
    for(i=0;i<6;i++) {
        char text[128];POSE pose;double seconds;int source,track,map[4],hand;
        DG_XR_CONFIG xc;DG_BRIDGE_CONFIG bc;ARM_POS_CFG ap;
        sprintf_s(text,sizeof text,"source=xr\nvr_m9_slide=on\nvr_pistol_reload=%s\n",words[i]);
        if(!parse_config(text,&pose,&seconds,&source,&xc,&bc,&track,map,&hand,&ap) ||
           bc.pistol_reload!=(i<2) || !bc.m9_slide) {
            printf("FAIL reload config case %d\n",i);bad++;
        }
    }
    printf("Reload config: 6 cases, %d failures\n",bad);return bad!=0;
}
