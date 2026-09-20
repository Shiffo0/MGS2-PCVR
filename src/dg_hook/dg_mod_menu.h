#ifndef DG_MOD_MENU_H
#define DG_MOD_MENU_H
/* Portable Home menu policy. Keys are held-state bits, never OS repeat events. */
#include <string.h>
enum { DG_MM_HOME=1, DG_MM_UP=2, DG_MM_DOWN=4, DG_MM_ENTER=8,
       DG_MM_LEFT=16, DG_MM_RIGHT=32 };
typedef struct {
    unsigned keys, requested, overridden;
    int open, selected, capture, neutral_ticks;
} DG_MOD_MENU;
static unsigned dg_mod_menu_step(DG_MOD_MENU *m,unsigned keys,int allowed,
                                int neutral,unsigned actual,unsigned available) {
    unsigned edge=keys & ~m->keys, changed=0, bit;
    m->keys=keys;
    m->requested=(m->requested&m->overridden)|(actual&~m->overridden);
    if(!allowed) m->open=0;
    else if(edge&DG_MM_HOME) m->open=!m->open;
    if(m->open) {
        m->capture=1;m->neutral_ticks=0;
        if(edge&DG_MM_UP)m->selected=(m->selected+2)%3;
        if(edge&DG_MM_DOWN)m->selected=(m->selected+1)%3;
        bit=1u<<m->selected;
        if((edge&(DG_MM_ENTER|DG_MM_LEFT|DG_MM_RIGHT)) && (available&bit)) {
            m->requested^=bit;m->overridden|=bit;changed=bit;
        }
    } else if(m->capture) {
        if(neutral && !keys) m->neutral_ticks++; else m->neutral_ticks=0;
        if(m->neutral_ticks>=3)m->capture=0;
    }
    return changed;
}
#endif
