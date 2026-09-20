/* CPU tests embedded in the shipping hook translation unit. */
#include "dg_interact_adapter.inl"
static int test_controls_producer(void) {
    DG_XR_FRAME frame;
    DG_BRIDGE_CONTROLS_FRAME out;
    LONG saved_fire=g_fire_mode, saved_move=g_move_mode, saved_turn=g_turn_mode;
    LONG saved_track=g_arm_track, saved_source=g_source;
    LONG saved_dz=g_turn_deadzone_mils_live;
    LONG saved_armed=g_armed;
    int saved_fallback=g_controls_turn_fallback;
    unsigned long saved_stream=g_arm_pose_stream_id;
    uint64_t now=1000;
    int bad=0, checks=0, h;
#define CONTROL_CHECK(x) do { checks++; if (!(x)) { bad++; \
    printf("FAIL controls producer line %d: %s\n",__LINE__,#x); } } while(0)
    {
        DG_ACTION_OWNER owner={0};
        DG_ACTION_INPUT input={0};
        int i;
        input.physical.valid=1;
        input.physical.source=input.physical.actor=1;
        input.physical.sample=input.physical.now_ms=1;
        input.context=1; input.pad_allowed=1;
        CONTROL_CHECK(dg_action_step(&owner,&input)==0);
        input.physical.down=1;
        ++input.physical.sample; ++input.physical.now_ms;
        CONTROL_CHECK(dg_action_step(&owner,&input)==0x10);
        input.context=2;
        for (i=0;i<DG_FLAT_SILENT_FRAMES+90;i++) {
            int allowed=action_present_allowed(1,i>=DG_FLAT_SILENT_FRAMES,0,0,2);
            ++input.physical.sample; ++input.physical.now_ms;
            input.physical.denied=!allowed;
            CONTROL_CHECK(dg_action_step(&owner,&input)==0x10);
        }
        CONTROL_CHECK(!action_present_allowed(0,1,0,0,2));
        CONTROL_CHECK(!action_present_allowed(1,1,1,0,2));
        CONTROL_CHECK(!action_present_allowed(1,1,0,1,0));
        CONTROL_CHECK(!action_present_allowed(1,1,0,0,3));
        CONTROL_CHECK(action_present_allowed(1,0,0,1,0));
        /* A real revocation still requires physical release before rearming. */
        input.physical.denied=1;
        CONTROL_CHECK(dg_action_step(&owner,&input)==0);
        input.physical.denied=0; ++input.physical.sample;
        CONTROL_CHECK(dg_action_step(&owner,&input)==0);
        input.physical.down=0; ++input.physical.sample;
        CONTROL_CHECK(dg_action_step(&owner,&input)==0);
        input.physical.down=1; ++input.physical.sample;
        CONTROL_CHECK(dg_action_step(&owner,&input)==0x10);
        input.physical.down=0; ++input.physical.sample;
        CONTROL_CHECK(dg_action_step(&owner,&input)==0);
    }
    memset(&frame,0,sizeof frame);
    g_fire_mode=g_move_mode=g_turn_mode=1;
    g_turn_deadzone_mils_live=150;
    g_arm_track=ARM_TRACK_OFF;
    g_source=SRC_SCRIPT;
    frame.left_hand.grip.active=frame.right_hand.grip.active=1;
    frame.left_hand.grip.tracked=frame.right_hand.grip.tracked=1;
    frame.left_hand.grip.orientation_valid=frame.right_hand.grip.orientation_valid=1;
    frame.left_hand.grip.sample_seq=frame.right_hand.grip.sample_seq=1;
    frame.right_hand.trigger_press_seq=7;
    frame.right_hand.trigger_release_seq=6;
    frame.right_hand.trigger_value=0.8f;
    frame.left_hand.thumbstick_x=0.5f;
    frame.left_hand.thumbstick_y=-0.25f;
    controls_init();
    controls_route_frame(&frame,1,1,now,0,&out);
    CONTROL_CHECK(out.fire_available && out.fire.valid && out.fire.press_seq==7);
    CONTROL_CHECK(out.move_available && out.move.valid && out.move.x==0.5 && out.move.y==-0.25);
    controls_route_frame(&frame,0,1,++now,0,&out);
    CONTROL_CHECK(out.fire_available && !out.fire.valid && !out.move_available);
    controls_route_frame(&frame,0,0,++now,0,&out);
    CONTROL_CHECK(out.fire.valid && out.fire.press_seq==7 && out.fire.release_seq==6);
    controls_route_frame(NULL,1,0,++now,0,&out);
    CONTROL_CHECK(!out.fire_available && !out.move_available);

    frame.left_hand.squeeze_click=frame.right_hand.squeeze_click=1;
    for (h=DG_CONTROLS_BEYOND;h<=DG_CONTROLS_LOCKER;h++) {
        controls_route_frame(&frame,h,1,++now,0,&out);
        CONTROL_CHECK(out.interact.valid && out.interact.special==h &&
            (out.interact.levels&(DG_IA_PEEP_LEFT|DG_IA_PEEP_RIGHT))==(DG_IA_PEEP_LEFT|DG_IA_PEEP_RIGHT));
        CONTROL_CHECK(!out.move_available && !out.fire.valid && g_controls.ladder_move_claim);
    }
    frame.left_hand.squeeze_click=frame.right_hand.squeeze_click=0;
    frame.left_hand.thumbstick_x=-1;
    controls_route_frame(&frame,DG_CONTROLS_BEYOND,1,++now,0,&out);
    CONTROL_CHECK(out.interact.valid && (out.interact.levels&DG_IA_PEEP_LEFT));
    CONTROL_CHECK(!out.move_available && !out.fire.valid && g_controls.ladder_move_claim);
    frame.left_hand.thumbstick_x=1;
    controls_route_frame(&frame,DG_CONTROLS_BEYOND,1,++now,0,&out);
    CONTROL_CHECK(out.interact.valid && (out.interact.levels&DG_IA_PEEP_RIGHT));
    controls_route_frame(&frame,DG_CONTROLS_GAMEPLAY,1,++now,0,&out);
    CONTROL_CHECK(g_controls.ladder_move_claim && out.move.x==0 && out.move.y==0);
    frame.left_hand.thumbstick_x=0.5f;
    /* Test-only catalog: production has no catalog writer or selectable IDs. */
    for (h=0;h<2;h++) {
        int k;
        controls_init();
        g_controls.catalog.available=1; g_controls.catalog.version=1;
        for (k=0;k<2;k++) {
            int slot;
            g_controls.catalog.hand[k].count=6;
            g_controls.catalog.hand[k].eligible=63;
            g_controls.catalog.hand[k].kind=k==0 ? 1u : 0u;
            for (slot=0;slot<6;slot++) strcpy_s(g_controls.catalog.hand[k].labels[slot],17,"TEST");
        }
        frame.left_hand.thumbstick_x=frame.left_hand.thumbstick_y=0;
        frame.right_hand.thumbstick_x=frame.right_hand.thumbstick_y=0;
        frame.left_hand.thumbstick_click=frame.right_hand.thumbstick_click=0;
        frame.right_hand.trigger_value=0;
        frame.left_hand.grip.sample_seq=frame.right_hand.grip.sample_seq=8;
        frame.left_hand.thumbstick_y=0.8f;
        controls_route_frame(&frame,DG_CONTROLS_LADDER,1,++now,1,&out);
        CONTROL_CHECK(out.interact.valid && out.interact.ladder &&
            (out.interact.levels&DG_IA_UP) && !out.interact.suppressed);
        CONTROL_CHECK(!out.move_available && !out.fire.valid);
        CONTROL_CHECK(DG_CONTROLS_LADDER!=DG_CONTROLS_RADIAL_ONLY);
        CONTROL_CHECK(g_controls.owner.select.hand<0);
        controls_route_frame(&frame,DG_CONTROLS_RADIAL_ONLY,1,++now,1,&out);
        CONTROL_CHECK(!out.interact.valid && !out.interact.ladder);
        CONTROL_CHECK(!out.move_available && !out.fire.valid);
        frame.left_hand.grip.sample_seq=frame.right_hand.grip.sample_seq=9;
        controls_route_frame(&frame,DG_CONTROLS_GAMEPLAY,1,++now,1,&out);
        CONTROL_CHECK(g_controls.ladder_move_claim && out.move_available && out.move.y==0);
        frame.left_hand.thumbstick_y=0;
        controls_route_frame(&frame,DG_CONTROLS_GAMEPLAY,1,++now,1,&out);
        CONTROL_CHECK(g_controls.ladder_move_claim);
        frame.left_hand.thumbstick_y=0.8f;
        g_controls.owner.axes=1;
        controls_route_frame(&frame,DG_CONTROLS_LADDER,1,++now,1,&out);
        CONTROL_CHECK(out.interact.suppressed==DG_IA_ALL);
        g_controls.owner.axes=0;
        frame.left_hand.thumbstick_y=0;
        frame.left_hand.grip.sample_seq=frame.right_hand.grip.sample_seq=10;
        controls_route_frame(&frame,1,1,++now,1,&out);
        CONTROL_CHECK(g_controls.owner.select.armed);
        CONTROL_CHECK(!g_controls.ladder_move_claim);
        if (h) frame.right_hand.thumbstick_click=1; else frame.left_hand.thumbstick_click=1;
        frame.left_hand.grip.sample_seq=frame.right_hand.grip.sample_seq=11;
        controls_route_frame(&frame,1,1,++now,1,&out);
        CONTROL_CHECK(g_controls.owner.select.hand==h && !out.fire.valid);
        if (h) frame.right_hand.thumbstick_x=0.8f; else frame.left_hand.thumbstick_x=0.8f;
        frame.left_hand.grip.sample_seq=frame.right_hand.grip.sample_seq=12;
        controls_route_frame(&frame,1,1,++now,1,&out);
        CONTROL_CHECK(out.move_available && out.move.x==0 && g_controls.last_output.turn_x==0);
        CONTROL_CHECK(g_controls.owner.select.selected>=0);
        controls_route_frame(&frame,1,1,++now,1,&out);
        CONTROL_CHECK(out.move_available && out.move.x==0 && !g_controls.last_output.selection.intent);
        if (h) frame.right_hand.thumbstick_click=0; else frame.left_hand.thumbstick_click=0;
        frame.left_hand.grip.sample_seq=frame.right_hand.grip.sample_seq=13;
        controls_route_frame(&frame,1,1,++now,1,&out);
        CONTROL_CHECK(g_controls.last_output.selection.intent && g_controls.owner.select.hand==-1);
        CONTROL_CHECK(g_controls.last_output.consume_axes==(1u<<h));
        controls_route_frame(&frame,1,1,++now,1,&out);
        CONTROL_CHECK(out.move_available && out.move.x==0);
        if (h) frame.right_hand.thumbstick_x=0.2f; else frame.left_hand.thumbstick_x=0.2f;
        frame.left_hand.grip.sample_seq=frame.right_hand.grip.sample_seq=14;
        controls_route_frame(&frame,1,1,++now,1,&out);
        CONTROL_CHECK(g_controls.last_output.consume_axes==(1u<<h) && out.move.x==0);
        if (h) frame.right_hand.thumbstick_x=0; else frame.left_hand.thumbstick_x=0;
        frame.left_hand.grip.sample_seq=frame.right_hand.grip.sample_seq=15;
        controls_route_frame(&frame,1,1,++now,1,&out);
        CONTROL_CHECK(g_controls.last_output.consume_axes==0 && out.move.x==0);
        /* Same sequence with revoked context cannot use cached routing. */
        controls_route_frame(&frame,0,1,++now,1,&out);
        CONTROL_CHECK(!out.move_available && !out.fire.valid && !g_controls.cached);
        frame.right_hand.trigger_value=0.8f;
        controls_route_frame(&frame,0,0,++now,1,&out);
        CONTROL_CHECK(out.fire.valid && out.fire.value==(double)0.8f);
        frame.right_hand.trigger_value=0;
        frame.left_hand.grip.pose_age_ms=101;
        controls_route_frame(&frame,1,1,++now,1,&out);
        CONTROL_CHECK(!out.move_available && !out.fire.valid);
        frame.left_hand.grip.pose_age_ms=0;
        /* Confirm category, keep the same click held, then select its item. */
        {
            DG_XR_HAND *hand=h ? &frame.right_hand : &frame.left_hand;
            dg_radial_owner_init(&g_controls.owner);g_controls.cached=0;
            g_controls.catalog.ids[h][0]=-3;
            hand->thumbstick_x=hand->thumbstick_y=0;
            hand->thumbstick_click=hand->trigger_click=0;
            ++frame.left_hand.grip.sample_seq;++frame.right_hand.grip.sample_seq;
            controls_route_frame(&frame,1,1,++now,1,&out);
            hand->thumbstick_click=1;
            ++frame.left_hand.grip.sample_seq;++frame.right_hand.grip.sample_seq;
            controls_route_frame(&frame,1,1,++now,1,&out);
            hand->thumbstick_y=0.8f;hand->trigger_click=1;
            ++frame.left_hand.grip.sample_seq;++frame.right_hand.grip.sample_seq;
            controls_route_frame(&frame,1,1,++now,1,&out);
            CONTROL_CHECK(g_controls.last_output.selection.trigger_confirm && g_controls.navigate_hand==h+1);
            CONTROL_CHECK(g_controls.group[h!=1]==2 && !out.fire.valid);
            hand->thumbstick_y=0;hand->trigger_click=0;
            ++frame.left_hand.grip.sample_seq;++frame.right_hand.grip.sample_seq;
            controls_route_frame(&frame,1,1,++now,1,&out);
            CONTROL_CHECK(g_controls.navigate_hand==h+1 && g_controls.owner.select.hand==h);
            CONTROL_CHECK(!out.fire.valid && !g_controls.last_output.selection.intent);
            g_controls.catalog.ids[h][0]=0;
            hand->thumbstick_y=0.8f;hand->trigger_click=1;
            ++frame.left_hand.grip.sample_seq;++frame.right_hand.grip.sample_seq;
            controls_route_frame(&frame,1,1,++now,1,&out);
            CONTROL_CHECK(g_controls.last_output.selection.intent && g_controls.last_output.selection.trigger_confirm);
            CONTROL_CHECK(g_controls.navigate_hand==h+1 && !out.fire.valid);
            /* Native ACK/stock refresh and held trigger preserve the gesture
               without producing another use. Then release/repress in place. */
            {
                int repeat;
                g_controls.catalog.busy=1;
                for(repeat=0;repeat<8;repeat++) {
                    ++frame.left_hand.grip.sample_seq;++frame.right_hand.grip.sample_seq;
                    controls_route_frame(&frame,1,1,++now,1,&out);
                    CONTROL_CHECK(g_controls.navigate_hand==h+1 && !out.fire.valid &&
                        !g_controls.last_output.selection.intent);
                }
                g_controls.catalog.busy=0;g_controls.catalog.version++;
                hand->trigger_click=0;
                ++frame.left_hand.grip.sample_seq;++frame.right_hand.grip.sample_seq;
                controls_route_frame(&frame,1,1,++now,1,&out);
                CONTROL_CHECK(g_controls.owner.select.hand==h && !g_controls.last_output.selection.intent);
                g_controls.catalog.version++; /* delayed stock refresh */
                ++frame.left_hand.grip.sample_seq;++frame.right_hand.grip.sample_seq;
                controls_route_frame(&frame,1,1,++now,1,&out);
                CONTROL_CHECK(g_controls.owner.select.hand==h && !g_controls.last_output.selection.intent);
                hand->trigger_click=1;
                ++frame.left_hand.grip.sample_seq;++frame.right_hand.grip.sample_seq;
                controls_route_frame(&frame,1,1,++now,1,&out);
                CONTROL_CHECK(g_controls.last_output.selection.intent && !out.fire.valid);
                hand->trigger_click=0;
                ++frame.left_hand.grip.sample_seq;++frame.right_hand.grip.sample_seq;
                controls_route_frame(&frame,1,1,++now,1,&out);
                hand->thumbstick_click=0; /* still pointing at item: close only */
                ++frame.left_hand.grip.sample_seq;++frame.right_hand.grip.sample_seq;
                controls_route_frame(&frame,1,1,++now,1,&out);
                CONTROL_CHECK(!g_controls.navigate_hand && !g_controls.last_output.selection.intent &&
                    g_controls.owner.select.hand<0);
            }
            hand->thumbstick_y=0;hand->thumbstick_click=hand->trigger_click=0;
        }
    }
    controls_init();
    {
        DG_CONTROLS_CATALOG valid;
        uint64_t context;
        int k;
        valid=g_controls.catalog;
        valid.available=1; valid.version=8;
        for (h=0;h<2;h++) {
            valid.hand[h].count=6; valid.hand[h].eligible=63;
            valid.hand[h].kind=h==0 ? 1u : 0u;
            for (k=0;k<6;k++) strcpy_s(valid.hand[h].labels[k],17,"TEST");
        }
        g_controls.catalog=valid;
        CONTROL_CHECK(controls_catalog_valid(1) && !controls_catalog_valid(0));
        strcpy_s(g_controls.catalog.hand[0].labels[0],17,"B.ARMOR");
        strcpy_s(g_controls.catalog.hand[0].labels[1],17,"MINE.D");
        CONTROL_CHECK(controls_catalog_valid(1));
        for (k=0;k<5;k++) {
            g_controls.catalog=valid;
            if (k==0) g_controls.catalog.hand[1].labels[0][0]='?';
            if (k==1) memset(g_controls.catalog.hand[1].labels[0],'A',17);
            if (k==2) g_controls.catalog.hand[1].labels[0][0]=0;
            if (k==3) g_controls.catalog.hand[1].eligible=0;
            if (k==4) g_controls.catalog.hand[1].eligible=128;
            CONTROL_CHECK(controls_catalog_valid(1)==(k==3));
        }
        g_controls.catalog=valid;
        memset(&frame,0,sizeof frame);
        frame.left_hand.grip.active=frame.right_hand.grip.active=1;
        frame.left_hand.grip.tracked=frame.right_hand.grip.tracked=1;
        frame.left_hand.grip.orientation_valid=frame.right_hand.grip.orientation_valid=1;
        frame.left_hand.grip.sample_seq=frame.right_hand.grip.sample_seq=100;
        controls_route_frame(&frame,1,1,++now,2,&out);
        frame.right_hand.thumbstick_click=1;
        frame.left_hand.grip.sample_seq=frame.right_hand.grip.sample_seq=101;
        controls_route_frame(&frame,1,1,++now,2,&out);
        CONTROL_CHECK(g_controls.owner.select.hand==1);
        context=g_controls.context;
        ++g_arm_pose_stream_id;
        frame.right_hand.thumbstick_x=0.7f;
        frame.left_hand.grip.sample_seq=frame.right_hand.grip.sample_seq=1;
        controls_route_frame(&frame,1,1,++now,2,&out);
        CONTROL_CHECK(g_controls.context==context+1 && g_controls.owner.select.hand==-1);
        CONTROL_CHECK(g_controls.owner.axes==2 && out.move_available && out.move.x==0);
        CONTROL_CHECK(out.fire.stream_id==g_arm_pose_stream_id);
        frame.left_hand.grip.sample_seq=frame.right_hand.grip.sample_seq=2;
        controls_route_frame(&frame,1,1,++now,3,&out);
        CONTROL_CHECK(g_controls.owner.select.hand==-1 && g_controls.owner.axes==2);
        /* Losing the catalog cancels selection, but cannot release a stick
         * that was claimed while deflected. */
        g_controls.catalog.available=0;
        frame.right_hand.thumbstick_click=0;
        frame.left_hand.grip.sample_seq=frame.right_hand.grip.sample_seq=3;
        controls_route_frame(&frame,1,1,++now,3,&out);
        CONTROL_CHECK(g_controls.owner.axes==2 && g_controls.last_output.turn_x==0);
        frame.right_hand.thumbstick_x=0;
        frame.left_hand.grip.sample_seq=frame.right_hand.grip.sample_seq=4;
        controls_route_frame(&frame,1,1,++now,3,&out);
        CONTROL_CHECK(g_controls.owner.axes==0 && out.move_available);
    }
    controls_init();
    {
        dg_radial_game_catalog native;
        int id,page,slot,kind,group,seen[64]={0};
        memset(&native,0,sizeof native);native.valid=1;native.version=7;
        native.player=11;native.inventory=22;
        for (id=0;id<64;id++) {
            strcpy_s(native.labels[0][id],17,"TEST");
            strcpy_s(native.labels[1][id],17,"ITEM");
            if (id&1) native.eligible[0]|=UINT64_C(1)<<id;
        }
        controls_catalog_build(&native,1);
        CONTROL_CHECK(controls_catalog_valid(1));
        CONTROL_CHECK(g_controls.catalog.hand[1].count==6 &&
            g_controls.catalog.ids[1][5]==-6);
        for (kind=0;kind<2;kind++) {
            int hand=1-kind;
            memset(seen,0,sizeof seen);
            for (group=2;group<6;group++) for (page=0;page<11;page++) {
                g_controls.group[kind]=group;g_controls.page[kind]=page;
                controls_catalog_build(&native,1);
                if (g_controls.page[kind]!=page) break;
                CONTROL_CHECK(g_controls.catalog.hand[hand].count==8 && g_controls.catalog.ids[hand][6]==-100);
                for (slot=0;slot<6;slot++) {
                    if (!g_controls.catalog.hand[hand].labels[slot][0]) continue;
                    id=g_controls.catalog.ids[hand][slot];
                    CONTROL_CHECK(id>=0 && id<64 && !seen[id]);seen[id]=1;
                    CONTROL_CHECK(((g_controls.catalog.hand[hand].eligible>>slot)&1u)==
                        (unsigned)((native.eligible[kind]>>id)&1u));
                }
            }
            for (id=0;id<64;id++) CONTROL_CHECK(seen[id]);
        }
        g_controls.group[0]=1;g_controls.page[0]=0;
        g_controls.favorites[0]=(UINT64_C(1)<<3)|(UINT64_C(1)<<18);
        controls_catalog_build(&native,1);
        CONTROL_CHECK(g_controls.catalog.ids[1][0]==3 && g_controls.catalog.ids[1][1]==18 &&
            g_controls.catalog.ids[1][5]==-102);
        g_controls.favorites_edit[0]=1;controls_catalog_build(&native,1);
        CONTROL_CHECK(!strncmp(g_controls.catalog.hand[1].labels[0],"ADD ",4) &&
            (g_controls.catalog.hand[1].eligible&63)==63);
        g_controls.favorites[0]|=1;controls_catalog_build(&native,1);
        CONTROL_CHECK(!strncmp(g_controls.catalog.hand[1].labels[0],"DROP ",5));
        native.recent_count[0]=3;native.recent[0][0]=18;native.recent[0][1]=0;native.recent[0][2]=3;
        g_controls.group[0]=0;controls_catalog_build(&native,1);
        CONTROL_CHECK(g_controls.catalog.ids[1][0]==18 && g_controls.catalog.ids[1][1]==0 &&
            g_controls.catalog.ids[1][2]==3 && (g_controls.catalog.hand[1].eligible&7)==4);
        controls_init();controls_catalog_build(&native,1);
        memset(&frame,0,sizeof frame);
        frame.left_hand.grip.active=frame.right_hand.grip.active=1;
        frame.left_hand.grip.tracked=frame.right_hand.grip.tracked=1;
        frame.left_hand.grip.orientation_valid=frame.right_hand.grip.orientation_valid=1;
        frame.left_hand.grip.sample_seq=frame.right_hand.grip.sample_seq=200;
        controls_route_frame(&frame,DG_CONTROLS_TURN_ONLY,1,++now,2,&out);
        frame.right_hand.thumbstick_click=1;
        frame.left_hand.grip.sample_seq=frame.right_hand.grip.sample_seq=201;
        controls_route_frame(&frame,DG_CONTROLS_TURN_ONLY,1,++now,2,&out);
        CONTROL_CHECK(g_controls.owner.select.hand==1 && !out.move_available && !out.fire.valid);
        frame.right_hand.thumbstick_x=0.8660254f;frame.right_hand.thumbstick_y=-0.5f;frame.right_hand.thumbstick_click=0;
        frame.left_hand.grip.sample_seq=frame.right_hand.grip.sample_seq=202;
        controls_route_frame(&frame,DG_CONTROLS_TURN_ONLY,1,++now,2,&out);
        CONTROL_CHECK(g_controls.group[0]==2 && g_controls.context==2);
        controls_catalog_build(&native,1);
        CONTROL_CHECK(g_controls.catalog.ids[1][0]==1 && g_controls.catalog.ids[1][2]==3);
        controls_init();g_controls.group[0]=1;g_controls.favorites_edit[0]=1;
        controls_catalog_build(&native,1);
        for (id=0;id<2;id++) {
            unsigned seq=300u+(unsigned)id*5;
            int neutral_sample;
            frame.right_hand.thumbstick_x=frame.right_hand.thumbstick_y=0;frame.right_hand.thumbstick_click=0;
            for (neutral_sample=0;neutral_sample<3;neutral_sample++) {
                frame.left_hand.grip.sample_seq=frame.right_hand.grip.sample_seq=seq+(unsigned)neutral_sample;
                controls_route_frame(&frame,DG_CONTROLS_TURN_ONLY,1,++now,2,&out);
            }
            frame.right_hand.thumbstick_click=1;
            frame.left_hand.grip.sample_seq=frame.right_hand.grip.sample_seq=seq+3;
            controls_route_frame(&frame,DG_CONTROLS_TURN_ONLY,1,++now,2,&out);
            frame.right_hand.thumbstick_click=0;frame.right_hand.thumbstick_y=1;
            frame.left_hand.grip.sample_seq=frame.right_hand.grip.sample_seq=seq+4;
            controls_route_frame(&frame,DG_CONTROLS_TURN_ONLY,1,++now,2,&out);
            CONTROL_CHECK((g_controls.favorites[0]&1u)==(uint64_t)(id==0));
            CONTROL_CHECK(g_controls.favorites_edit[0] && g_controls.group[0]==1);
            controls_catalog_build(&native,1);
        }
    }
    controls_init();
    /* Observe the actual selected script turn integrator. Stopping a route
     * must stop the independent software-turn side effect too. */
    {
        static const char script[]="DGXR_SCRIPT 1 10\n"
            "pose left grip 1 -0.2 -0.3 -0.4 0 0 0 1\n"
            "pose right grip 1 0.2 -0.3 -0.4 0 0 0 1\n"
            "input right 0 0 0.8 0 0\n"
            "hold 20\n";
        DG_XR_SCRIPT_PROGRAM program;
        DG_XR_CONFIG cfg;
        DG_XR_FRAME sample;
        char error[160];
        double before;
        unsigned reads;
        memset(&cfg,0,sizeof cfg);
        cfg.yaw_sign=cfg.pitch_sign=cfg.roll_sign=1;
        cfg.x_sign=cfg.y_sign=cfg.z_sign=1; cfg.scale=1000;
        if (dg_xr_script_program_parse_text(&program,script,sizeof script-1,error,sizeof error)) {
            CONTROL_CHECK(dg_xr_script_test_begin(&program,&cfg));
            CONTROL_CHECK(dg_xr_script_test_step(&sample));
            reads=g_controls_input_reads;
            controls_provide(NULL,1,1,++now,1,&out);
            CONTROL_CHECK(g_controls_input_reads==reads+1 && out.move_available);
            before=input_turn_offset_rad();
            CONTROL_CHECK(dg_xr_script_test_step(&sample));
            CONTROL_CHECK(input_turn_offset_rad()>before);
            controls_route_frame(NULL,1,1,++now,0,&out);
            before=input_turn_offset_rad();
            CONTROL_CHECK(dg_xr_script_test_step(&sample));
            CONTROL_CHECK(input_turn_offset_rad()==before);
            controls_route_frame(&sample,1,1,++now,0,&out);
            controls_route_frame(&sample,0,1,++now,0,&out);
            before=input_turn_offset_rad();
            CONTROL_CHECK(dg_xr_script_test_step(&sample));
            CONTROL_CHECK(input_turn_offset_rad()==before);
            controls_route_frame(&sample,1,1,++now,0,&out);
            sample.right_hand.grip.pose_age_ms=101;
            controls_route_frame(&sample,1,1,++now,0,&out);
            before=input_turn_offset_rad();
            CONTROL_CHECK(dg_xr_script_test_step(&sample));
            CONTROL_CHECK(input_turn_offset_rad()==before);
            controls_route_frame(&sample,DG_CONTROLS_TURN_ONLY,1,++now,0,&out);
            CONTROL_CHECK(!out.move_available && !out.fire.valid);
            before=input_turn_offset_rad();
            CONTROL_CHECK(dg_xr_script_test_step(&sample));
            CONTROL_CHECK(input_turn_offset_rad()>before);
            controls_route_frame(&sample,DG_CONTROLS_RADIAL_ONLY,1,++now,0,&out);
            CONTROL_CHECK(!out.move_available && !out.fire.valid);
            before=input_turn_offset_rad();
            CONTROL_CHECK(dg_xr_script_test_step(&sample));
            CONTROL_CHECK(input_turn_offset_rad()==before);
            /* No registered bridge: camera fallback owns tracking turn only. */
            g_armed=1;g_controls_turn_fallback=1;
            reads=g_controls_input_reads;
            controls_fallback_turn();
            CONTROL_CHECK(g_controls_input_reads==reads+1);
            before=input_turn_offset_rad();
            CONTROL_CHECK(dg_xr_script_test_step(&sample));
            CONTROL_CHECK(input_turn_offset_rad()>before);
            g_controls_turn_fallback=0; /* a bridge session cannot also fallback */
            reads=g_controls_input_reads;controls_fallback_turn();
            CONTROL_CHECK(g_controls_input_reads==reads);
            before=input_turn_offset_rad();
            CONTROL_CHECK(dg_xr_script_test_step(&sample));
            CONTROL_CHECK(input_turn_offset_rad()>before);
            g_controls_turn_fallback=1;
            controls_session_end(0);
            reads=g_controls_input_reads;controls_fallback_turn();
            CONTROL_CHECK(!g_armed && g_controls_input_reads==reads);
            before=input_turn_offset_rad();
            CONTROL_CHECK(dg_xr_script_test_step(&sample));
            CONTROL_CHECK(input_turn_offset_rad()==before);
            CONTROL_CHECK(dg_xr_script_test_abort(&sample));
            dg_xr_script_program_free(&program);
        } else CONTROL_CHECK(0);
    }
    /* A catalog retained from gameplay must not open on a ledge/in a locker.
     * Exercise both hands with fresh neutral and click samples. */
    for (h=DG_CONTROLS_BEYOND;h<=DG_CONTROLS_DOWNED;h++) {
        int hand,phase;
        controls_init();
        g_controls.source=g_source;g_controls.stream=g_arm_pose_stream_id;
        g_controls.catalog.available=1;g_controls.catalog.version=1;
        memset(&frame,0,sizeof frame);
        for (hand=0;hand<2;hand++) {
            DG_XR_HAND *hp=hand?&frame.right_hand:&frame.left_hand;
            hp->grip.active=hp->grip.tracked=hp->grip.orientation_valid=1;
            g_controls.catalog.hand[hand].count=1;
            g_controls.catalog.hand[hand].eligible=1;
            strcpy_s(g_controls.catalog.hand[hand].labels[0],17,"TEST");
        }
        for (phase=0;phase<3;phase++) {
            frame.left_hand.grip.sample_seq=frame.right_hand.grip.sample_seq=(uint64_t)phase+1;
            frame.left_hand.thumbstick_click=phase==1;
            frame.right_hand.thumbstick_click=phase==2;
            controls_route_frame(&frame,h,1,++now,1,&out);
            CONTROL_CHECK(!g_controls.last_input.select.safe && g_controls.owner.select.hand<0);
            CONTROL_CHECK(!g_controls.last_output.selection.intent && !out.move_available && !out.fire.valid);
        }
    }
    /* Downed input still reaches interactions while walking/fire/radial stay
       refused, even with a catalog retained from normal gameplay. */
    frame.left_hand.thumbstick_click=frame.right_hand.thumbstick_click=0;
    frame.right_hand.primary_button=1;
    ++frame.left_hand.grip.sample_seq;++frame.right_hand.grip.sample_seq;
    controls_route_frame(&frame,DG_CONTROLS_DOWNED,1,++now,1,&out);
    CONTROL_CHECK(out.interact.valid && out.interact.special==DG_IA_DOWNED_CONTEXT &&
        (out.interact.levels&DG_IA_ACTION) && !(out.interact.suppressed&DG_IA_ACTION));
    CONTROL_CHECK(!out.move_available && !out.fire.valid && !g_controls.last_output.selection.intent);
    frame.right_hand.primary_button=0;
    {
        DG_MOD_MENU saved_menu=g_mod_menu;
        DG_INTERACT_ADAPTER adapter={0};
        DG_INTERACT_NATIVE_INPUT native={0};
        DG_INTERACT_NATIVE_OUTPUT pad;
        int tick,recovered=0,progress=0,context;
        memset(&g_mod_menu,0,sizeof g_mod_menu);
        dg_mod_menu_step(&g_mod_menu,DG_MM_HOME,1,1,7,7);
        CONTROL_CHECK(g_mod_menu.open && controls_menu_capture_allowed(DG_CONTROLS_GAMEPLAY));
        /* Knocked down while Home was open and left stick remains held.
           Raw menu rearm and radial axes stay claimed, but cannot steal
           fresh face recovery input. This uses production routing+adapter. */
        dg_mod_menu_step(&g_mod_menu,0,0,0,7,7);
        frame.left_hand.thumbstick_x=.8f;
        CONTROL_CHECK(!g_mod_menu.open && g_mod_menu.capture &&
            !controls_menu_capture_allowed(DG_CONTROLS_DOWNED));
        CONTROL_CHECK(g_controls.owner.axes!=0);
        native.safe=1;native.player_identity=1;
        for(tick=0;tick<80;tick++) {
            frame.right_hand.primary_button=(tick%4)==1;
            ++frame.left_hand.grip.sample_seq;++frame.right_hand.grip.sample_seq;
            context=controls_menu_capture_allowed(DG_CONTROLS_DOWNED)?DG_CONTROLS_NONE:DG_CONTROLS_DOWNED;
            controls_route_frame(&frame,context,1,++now,1,&out);
            native.input=out.interact;native.tick=(uint64_t)tick+1;
            ia_step(&adapter,&native,&pad);
            progress++;if(pad.press&0xf0)progress+=4;
            if(progress>60 && pad.press)recovered=1;
            CONTROL_CHECK(!out.move_available && !out.fire.valid && !(pad.status&~0x10u));
        }
        CONTROL_CHECK(recovered && g_controls.owner.axes!=0 && g_mod_menu.capture);
        frame.left_hand.thumbstick_x=0;frame.right_hand.primary_button=0;
        g_mod_menu=saved_menu;
    }
    /* Catalog toggle targets are actor-backed and eligibility checked. */
    {
        dg_radial_game_catalog n;
        memset(&n,0,sizeof n);n.version=1;n.player=11;n.inventory=22;
        n.current[0]=2;n.current[1]=1;
        n.eligible[0]=5;n.eligible[1]=3;
        controls_catalog_build(&n,1);
        CONTROL_CHECK(g_controls.catalog.quick_id[0]==0 && g_controls.catalog.quick_id[1]==0);
        n.current[0]=n.current[1]=0;n.previous[0]=2;n.previous[1]=1;
        controls_catalog_build(&n,1);
        CONTROL_CHECK(g_controls.catalog.quick_id[0]==2 && g_controls.catalog.quick_id[1]==1);
        n.eligible[0]=1;n.previous[1]=0;
        controls_catalog_build(&n,1);
        CONTROL_CHECK(g_controls.catalog.quick_id[0]==-1 && g_controls.catalog.quick_id[1]==-1);
    }
    {
        LONG saved_third=g_move_third_live;
        int mode;
        /* Actual DOWNED -> standing producer transitions, in both camera
           modes, without A/FPS-toggle input. Held/invalid sticks stay refused. */
        for(mode=0;mode<3;mode++) {
            int context=mode==0?DG_CONTROLS_GAMEPLAY:DG_CONTROLS_TURN_ONLY;
            controls_init();memset(&frame,0,sizeof frame);
            g_move_third_live=mode!=2;
            frame.left_hand.grip.active=frame.right_hand.grip.active=1;
            frame.left_hand.grip.tracked=frame.right_hand.grip.tracked=1;
            frame.left_hand.grip.orientation_valid=frame.right_hand.grip.orientation_valid=1;
            frame.left_hand.grip.sample_seq=frame.right_hand.grip.sample_seq=1;
            frame.left_hand.thumbstick_y=.8f;
            controls_route_frame(&frame,DG_CONTROLS_DOWNED,1,++now,0,&out);
            CONTROL_CHECK(g_controls.ladder_move_claim && !out.move_available);
            ++frame.left_hand.grip.sample_seq;++frame.right_hand.grip.sample_seq;
            controls_route_frame(&frame,context,1,++now,0,&out);
            CONTROL_CHECK(g_controls.ladder_move_claim && out.move.y==0);
            frame.left_hand.thumbstick_y=0;frame.left_hand.grip.tracked=0;
            ++frame.left_hand.grip.sample_seq;++frame.right_hand.grip.sample_seq;
            controls_route_frame(&frame,context,1,++now,0,&out);
            CONTROL_CHECK(g_controls.ladder_move_claim);
            frame.left_hand.grip.tracked=1;
            ++frame.left_hand.grip.sample_seq;++frame.right_hand.grip.sample_seq;
            controls_route_frame(&frame,context,1,++now,0,&out);
            CONTROL_CHECK(g_controls.ladder_move_claim==(mode==2));
            frame.left_hand.thumbstick_y=.8f;
            ++frame.left_hand.grip.sample_seq;++frame.right_hand.grip.sample_seq;
            controls_route_frame(&frame,context,1,++now,0,&out);
            CONTROL_CHECK(mode==2?!out.move_available:
                (out.move_available && out.move.valid && out.move.y>.79));
        }
        g_move_third_live=saved_third;
    }
    input_turn_rate(0);
    g_fire_mode=saved_fire; g_move_mode=saved_move; g_turn_mode=saved_turn;
    g_arm_track=saved_track; g_source=saved_source; g_turn_deadzone_mils_live=saved_dz;
    g_arm_pose_stream_id=saved_stream;
    g_armed=saved_armed;g_controls_turn_fallback=saved_fallback;
    printf("controls producer: %d checks, %d failures\n",checks,bad);
#undef CONTROL_CHECK
    return bad ? 1 : 0;
}
