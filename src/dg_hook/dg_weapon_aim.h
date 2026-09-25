#ifndef DG_WEAPON_AIM_H
#define DG_WEAPON_AIM_H






static int dg_weapon_hand_aim(int weapon)
{
    return weapon == 1 || weapon == 2 || weapon == 3 || weapon == 5 ||
           weapon == 6 || weapon == 7 || weapon == 12 || weapon == 13 || weapon == 14 || weapon == 15 || weapon == 18;
}

/* Existing proximity/anchor tuning is for pistol support. Do not silently
   apply it to rifle foregrips, RGB6 reloads or a spray can. */
static int dg_weapon_pistol_support(int weapon)
{
    return weapon == 1 || weapon == 2 || weapon == 3;
}
#endif
