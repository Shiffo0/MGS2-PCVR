#ifndef DG_CONTROLLER_BUTTONS_TEST_H
#define DG_CONTROLLER_BUTTONS_TEST_H

static int test_controller_button_context(void)
{
    DG_XR_BUTTON_GATE gate;
    unsigned events, total;
    uint32_t t;
    int bad=0;
#define CB_CHECK(x) do { if (!(x)) { printf("    button context line %d\n",__LINE__); bad++; } } while(0)
    memset(&gate,0,sizeof gate);
    /* Menu -> gameplay with both buttons still held. Every menu sample is
       real input, but neither it nor the new context owns a gameplay press. */
    for(t=0;t<=1500;t+=50) CB_CHECK(!dg_xr_button_gate_step(&gate,1,0,1,1,t));
    for(;t<=3000;t+=50) CB_CHECK(!dg_xr_button_gate_step(&gate,2,1,1,1,t));
    CB_CHECK(!dg_xr_button_gate_step(&gate,2,1,0,0,t)); t+=50;
    CB_CHECK(dg_xr_button_gate_step(&gate,2,1,1,0,t)==DG_XR_BUTTON_TOGGLE);
    for(t+=50;t<=4000;t+=50) CB_CHECK(!dg_xr_button_gate_step(&gate,2,1,1,0,t));
    CB_CHECK(!dg_xr_button_gate_step(&gate,2,1,0,0,t)); t+=50;
    total=0;
    { uint32_t start=t;
      for(;t<=start+1500;t+=50) {
        events=dg_xr_button_gate_step(&gate,2,1,0,1,t);
        CB_CHECK(events==(t==start+1000 ? DG_XR_BUTTON_RECENTER : 0));
        total+=events;
      }
    }
    CB_CHECK(total==DG_XR_BUTTON_RECENTER);
    /* Interrupt a 700ms hold; it must not complete after a menu or focus gap. */
    CB_CHECK(!dg_xr_button_gate_step(&gate,2,1,0,0,t)); t+=50;
    { uint32_t start=t;
      for(;t<=start+700;t+=50) CB_CHECK(!dg_xr_button_gate_step(&gate,2,1,0,1,t));
      CB_CHECK(!dg_xr_button_gate_step(&gate,3,0,0,1,t)); t+=50;
      for(;t<=start+2400;t+=50) CB_CHECK(!dg_xr_button_gate_step(&gate,4,1,0,1,t));
    }
    /* An input stall is a boundary even if the context epoch did not change. */
    CB_CHECK(!dg_xr_button_gate_step(&gate,4,1,0,0,t)); t+=50;
    CB_CHECK(!dg_xr_button_gate_step(&gate,4,1,0,1,t)); t+=1000;
    CB_CHECK(!dg_xr_button_gate_step(&gate,4,1,1,1,t));
    /* Unsigned tick wrap keeps a continuously sampled one-second hold. */
    memset(&gate,0,sizeof gate); t=UINT32_MAX-600u;
    CB_CHECK(!dg_xr_button_gate_step(&gate,8,1,0,0,t)); t+=50;
    total=0;
    { unsigned i;
      for(i=0;i<=30;i++,t+=50) total+=dg_xr_button_gate_step(&gate,8,1,0,1,t);
    }
    CB_CHECK(total==DG_XR_BUTTON_RECENTER);

    /* Actual producer/publication/consumer helpers, including events queued
       before Present learns that a menu has opened. Synthetic clock only. */
    dg_xr_test_controller_context(0,100);
    CB_CHECK(!dg_xr_test_controller_buttons(1,1,1,100));
    dg_xr_test_controller_context(1,150);
    CB_CHECK(!dg_xr_test_controller_buttons(1,1,1,150));
    CB_CHECK(!dg_xr_test_controller_toggle_take(150));
    CB_CHECK(!dg_xr_test_controller_buttons(1,0,0,160));
    CB_CHECK(dg_xr_test_controller_buttons(1,1,0,170)==DG_XR_BUTTON_TOGGLE);
    CB_CHECK(dg_xr_test_controller_toggle_take(170));
    CB_CHECK(!dg_xr_test_controller_toggle_take(170));
    CB_CHECK(!dg_xr_test_controller_buttons(1,0,0,180));
    CB_CHECK(dg_xr_test_controller_buttons(1,1,0,190)==DG_XR_BUTTON_TOGGLE);
    /* An entire menu interval may occur between two XR samples. */
    dg_xr_test_controller_context(0,191);
    dg_xr_test_controller_context(1,192);
    CB_CHECK(!dg_xr_test_controller_toggle_take(192));
    CB_CHECK(!dg_xr_test_controller_buttons(1,1,0,192));
    CB_CHECK(!dg_xr_test_controller_buttons(1,0,0,200));
    CB_CHECK(dg_xr_test_controller_buttons(1,1,0,210)==DG_XR_BUTTON_TOGGLE);
    dg_xr_test_controller_context(1,400); /* refresh cannot revive an old event */
    CB_CHECK(!dg_xr_test_controller_toggle_take(400));
    CB_CHECK(!dg_xr_test_controller_buttons(1,1,1,400));
    CB_CHECK(!dg_xr_test_controller_buttons(1,0,0,410));
    total=0;
    for(t=420;t<=1420;t+=50) {
        dg_xr_test_controller_context(1,t);
        total+=dg_xr_test_controller_buttons(1,0,1,t);
    }
    CB_CHECK(total==DG_XR_BUTTON_RECENTER);
    CB_CHECK(dg_xr_test_controller_recenter_current(1420));
    /* Focus loss cancels even a recenter that was already requested. */
    CB_CHECK(!dg_xr_test_controller_buttons(0,0,0,1430));
    CB_CHECK(!dg_xr_test_controller_recenter_current(1430));
    dg_xr_test_controller_context(1,1440);
    CB_CHECK(!dg_xr_test_controller_buttons(1,1,1,1440));
    /* Missing Present heartbeat refuses held buttons and their timers. */
    CB_CHECK(!dg_xr_test_controller_buttons(1,1,1,1600));
    dg_xr_test_controller_context(1,1610);
    CB_CHECK(!dg_xr_test_controller_buttons(1,1,1,1610));
    dg_xr_test_controller_context(0,1620);
    CB_CHECK(!dg_xr_test_controller_buttons(0,0,0,1620));
    /* An inactive action is not an observed physical release. Test the
       actual runtime validity mask independently for A and recenter. */
    dg_xr_test_controller_context(1,2000);
    CB_CHECK(!dg_xr_test_controller_actions(3,0,0,2000));
    CB_CHECK(!dg_xr_test_controller_actions(2,0,0,2010)); /* A inactive */
    CB_CHECK(!dg_xr_test_controller_actions(3,1,0,2020)); /* A returns held */
    CB_CHECK(!dg_xr_test_controller_toggle_take(2020));
    CB_CHECK(!dg_xr_test_controller_actions(3,0,0,2030));
    CB_CHECK(dg_xr_test_controller_actions(3,1,0,2040)==DG_XR_BUTTON_TOGGLE);
    CB_CHECK(dg_xr_test_controller_toggle_take(2040));
    CB_CHECK(!dg_xr_test_controller_actions(1,0,0,2050)); /* Y inactive */
    for(t=2060;t<3500;t+=50) {
        dg_xr_test_controller_context(1,t);
        CB_CHECK(!dg_xr_test_controller_actions(3,0,1,t)); /* returns held */
    }
    CB_CHECK(!dg_xr_test_controller_actions(3,0,0,t)); t+=50;
    total=0;
    { uint32_t start=t;
      for(;t<=start+1000;t+=50) {
        dg_xr_test_controller_context(1,t);
        total+=dg_xr_test_controller_actions(2,0,1,t); /* Y works without A binding */
      }
    }
    CB_CHECK(total==DG_XR_BUTTON_RECENTER);
    dg_xr_test_controller_context(0,t);
    CB_CHECK(!dg_xr_test_controller_recenter_current(t));
    CB_CHECK(!dg_xr_test_controller_actions(0,0,0,t));
    /* Actual Present-side context decision at 1 camera : 2 Presents, not
       just a hand-invented allow boolean. Known UI denies immediately. */
    {
        LONG saved_camera=g_buttons_camera_seen;
        DWORD saved_ms=g_buttons_camera_ms, saved_start_ms=g_buttons_start_ms;
        int saved_have=g_buttons_have_camera, saved_start=g_buttons_start_block;
        unsigned epoch=10;
        int last_allowed=0;
        g_buttons_camera_seen=0; g_buttons_have_camera=g_buttons_start_block=0;
        memset(&gate,0,sizeof gate); total=0;
        for(t=0;t<=1500;t+=10) {
            int allowed=controller_camera_context((LONG)(1+t/20),1,t);
            if (allowed!=last_allowed) epoch++;
            last_allowed=allowed;
            CB_CHECK(allowed);
            total+=dg_xr_button_gate_step(&gate,epoch,allowed,0,t>=10,t);
        }
        CB_CHECK(total==DG_XR_BUTTON_RECENTER);
        CB_CHECK(!controller_camera_context(g_buttons_camera_seen,0,1510));
        CB_CHECK(!controller_camera_context(g_buttons_camera_seen,1,1700));
        g_buttons_start_ms=1800; g_buttons_start_block=1;
        CB_CHECK(!controller_camera_context(200,1,1800));
        CB_CHECK(!controller_camera_context(201,1,1900));
        CB_CHECK(controller_camera_context(202,1,1901));
        g_buttons_camera_seen=saved_camera; g_buttons_camera_ms=saved_ms;
        g_buttons_start_ms=saved_start_ms; g_buttons_start_block=saved_start;
        g_buttons_have_camera=saved_have;
    }
    printf("  %s controller context: held menu A/Y, fresh-press rearm, one-second Y, "
           "epoch/focus/stall/clock-wrap and real queued-event expiry (%d failures)\n",
           bad?"FAIL":"ok",bad);
#undef CB_CHECK
    return bad ? 1 : 0;
}
#endif
