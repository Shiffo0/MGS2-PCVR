



int dg_bridge_hanging_heading_now(double *heading)
{
    uint64_t player;
    int weapon;
    short yaw;
    if (!heading || dg_bridge_controller_special_now()!=DG_CONTROLS_BEYOND ||
        !g_b.a.gbp_active || !g_b.a.pl_subject_move ||
        region_end(g_b.a.gbp_active)<g_b.a.gbp_active+4 ||
        region_end(g_b.a.pl_subject_move)<g_b.a.pl_subject_move+4 ||
        !RD32(g_b.a.gbp_active) || !RD32(g_b.a.pl_subject_move) ||
        !interact_player_now(&player,&weapon) ||
        !interact_read(NULL,player+0x82,&yaw,sizeof yaw)) return 0;
    *heading=(double)((unsigned short)yaw&4095)*
        (6.28318530717958647692/4096.0);
    return 1;
}
