

static unsigned ia_pad(unsigned levels, unsigned capture) {
    return ((levels&DG_IA_ACTION)?0x10u:0) |
           ((levels&DG_IA_POSTURE)?0x40u:0) |
           ((levels&DG_IA_MELEE)?0x20u:0) |
           ((levels&DG_IA_CAPTURE)?capture:0) |
           ((levels&DG_IA_UP)?0x1000u:0) | ((levels&DG_IA_DOWN)?0x4000u:0) |
           ((levels&DG_IA_PEEP_LEFT)?1u:0) | ((levels&DG_IA_PEEP_RIGHT)?2u:0);
}
static void ia_step(DG_INTERACT_ADAPTER *s,
    const DG_INTERACT_NATIVE_INPUT *in, DG_INTERACT_NATIVE_OUTPUT *out) {
    const DG_INTERACT_SAMPLE *p=&in->input;
    unsigned levels=p->levels, deny=p->suppressed, physical=0, bit, next;
    int context=p->special ? p->special : (p->ladder?DG_IA_LADDER_CONTEXT:0);
    int fresh=!s->seen || p->sample>s->sample;
    memset(out,0,sizeof *out);
    if (!s->seen || p->epoch!=s->epoch) {
        memset(s,0,sizeof *s);
        s->epoch=p->epoch; s->blocked=DG_IA_ALL;
        fresh=1;
    }
    if (s->special!=context) {
        s->held=0;s->blocked=DG_IA_ALL;s->special=context;
    }
    if (!in->safe || !p->valid || !p->epoch || !p->sample ||
        !in->tick || p->age_ms>100 || (levels&~DG_IA_ALL) ||
        (deny&~DG_IA_ALL) || (s->seen && (p->sample<s->sample ||
        in->tick<s->tick || p->choke_seq<s->choke_seq ||
        (p->sample==s->sample && (levels!=s->raw || p->choke_seq!=s->choke_seq))))) {
        s->held=0; s->blocked=DG_IA_ALL;
        /* Keep ordering floors; invalid samples can never provide neutral. */
        if (p->sample>s->sample) s->sample=p->sample;
        if (p->choke_seq>s->choke_seq) s->choke_seq=p->choke_seq;
        if (in->tick>s->tick) s->tick=in->tick;
        s->raw=levels; s->seen=1;
        return;
    }
    if (s->player_identity!=in->player_identity) {
        s->held&=~DG_IA_CAPTURE;s->blocked|=DG_IA_CAPTURE;
        if (context) { s->held=0;s->blocked=DG_IA_ALL; }
        s->player_identity=in->player_identity;
    }
    if (!in->unarmed || !in->player_identity || !in->capture_mask ||
        (in->capture_mask&(in->capture_mask-1u)) || (in->capture_mask&0x70u))
        deny|=DG_IA_CAPTURE;
    if (context==DG_IA_LADDER_CONTEXT) deny|=DG_IA_ALL&~(DG_IA_UP|DG_IA_DOWN);
    else if (context==DG_IA_BEYOND_CONTEXT) deny|=DG_IA_ALL&~
        (DG_IA_ACTION|DG_IA_POSTURE|DG_IA_PEEP_LEFT|DG_IA_PEEP_RIGHT);
    else if (context==DG_IA_LOCKER_CONTEXT) deny|=DG_IA_ALL&~
        (DG_IA_ACTION|DG_IA_POSTURE|DG_IA_MELEE|DG_IA_PEEP_LEFT|DG_IA_PEEP_RIGHT);
    else if (context) deny|=DG_IA_ALL;
    else deny|=DG_IA_UP|DG_IA_DOWN|DG_IA_PEEP_LEFT|DG_IA_PEEP_RIGHT;
    if (context && !in->player_identity) deny|=DG_IA_ALL;
    if (((s->held&DG_IA_UP) && (levels&DG_IA_DOWN)) ||
        ((s->held&DG_IA_DOWN) && (levels&DG_IA_UP))) deny|=DG_IA_UP|DG_IA_DOWN;
    if ((levels&(DG_IA_UP|DG_IA_DOWN))==(DG_IA_UP|DG_IA_DOWN)) deny|=DG_IA_UP|DG_IA_DOWN;
    for (bit=1;bit<=DG_IA_PEEP_RIGHT;bit<<=1)
        if ((in->native_status|in->native_press|in->native_release)&
            ia_pad(bit,in->capture_mask)) physical|=bit;
    deny|=physical;
    if (in->native_peep_left) deny|=DG_IA_PEEP_LEFT;
    if (in->native_peep_right) deny|=DG_IA_PEEP_RIGHT;
    /* A trigger while capture is held is a choke request, never a punch.
     * Its physical claim stays blocked until trigger neutral after capture. */
    if (!context && ((levels|s->held)&DG_IA_CAPTURE)) deny|=DG_IA_MELEE;
    s->blocked|=deny;
    s->held&=~deny;
    if (fresh) s->blocked&=levels|deny;
    next=levels&~s->blocked;
    if (!fresh) next&=s->held;
    if (in->tick!=s->tick) {
        out->status=ia_pad(next,in->capture_mask);
        if (fresh) {
            out->codec_press=(next&~s->held&DG_IA_CODEC)!=0;
            out->press=ia_pad(next&~s->held,in->capture_mask);
            out->release=ia_pad(s->held&~next,in->capture_mask);
            if ((s->held&next&DG_IA_CAPTURE) &&
                (levels&DG_IA_MELEE) && !(p->suppressed&DG_IA_MELEE) &&
                p->choke_seq>s->choke_seq)
                out->press|=in->capture_mask;
        }
        out->capture_pressure=(next&DG_IA_CAPTURE)!=0;
    }
    s->held=next; s->raw=levels; s->sample=p->sample;
    s->choke_seq=p->choke_seq; s->tick=in->tick; s->seen=1;
}
