/* Serialized provider barrier precedes every session teardown operation. */
static void controls_session_end(int bridge_on) {
    AcquireSRWLockExclusive(&g_controls_fallback_lock);
    g_controls_turn_fallback=0;
    if (bridge_on) dg_bridge_controls_register(NULL,NULL,NULL);
    InterlockedExchange(&g_armed,0);
    input_turn_rate(0);
    ReleaseSRWLockExclusive(&g_controls_fallback_lock);
}
