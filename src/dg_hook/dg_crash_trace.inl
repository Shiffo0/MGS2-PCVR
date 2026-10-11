/* Crash trace (diagnostic build). Live 27 Sep: two sessions with the second eye
   on ended mid-log with no WER report, no dump and no DLL-detach recorder dump.
   This handler sits LAST in the vectored chain and only reads: for fatal-class
   exceptions it logs code, faulting module+offset, registers and the code
   addresses found on the stack. First chance - the game or another module may
   still handle it - so each distinct faulting address is logged once (32 max)
   and repeats are only counted. logf_ writes unbuffered, so the lines survive
   whatever follows. */














































































static void crash_trace_start(void) {}

