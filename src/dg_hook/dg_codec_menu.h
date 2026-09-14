

#ifndef DG_CODEC_MENU_H
#define DG_CODEC_MENU_H
#include "dg_menu.h"
#define DG_CODEC_MENU_REQUEST 0x80000000u
#define DG_CODEC_MENU_ALLOW_EXIT 2
#define DG_CODEC_MENU_NATIVE_CANCEL 0x00040040u
static unsigned int dg_codec_menu_route(unsigned int pending,
                                        unsigned int menu,
                                        unsigned int physical)
{
    if (!(pending & DG_CODEC_MENU_REQUEST)) return pending & DG_MENU_PAD_ALLOWED;
    /* RADIO_ON, no weapon/item panel, and no native START+SELECT chord. */
    if (!(menu & 0x400u) || (menu & 0x300u) ||
        (physical & DG_MENU_PAD_STA)) return 0;
    return DG_CODEC_MENU_NATIVE_CANCEL | DG_MENU_PAD_SEL;
}
#endif
