/* dg_xr_script.c - deterministic, fail-closed desktop controller playback. */
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dg_xr_script.h"

#define SCRIPT_MAX_BYTES  (1024u * 1024u)
#define SCRIPT_MAX_STEPS  1024u
#define SCRIPT_MAX_TICKS  360000u
#define SCRIPT_DEFAULT_TICK_MS 10u
#define SCRIPT_STALE_MS   250u
#define SCRIPT_TURN_STALE_MS 100u
#define SCRIPT_FOCUS_WAIT_MS 30000u

typedef struct {
    unsigned int start_tick;
    unsigned int ticks;
    unsigned int events;
    DG_XR_FRAME frame;
} DG_XR_SCRIPT_STEP;

static void set_error(char *dst, size_t cap, unsigned int line,
                      const char *message)
{
    if (!dst || !cap) return;
    if (line)
        _snprintf_s(dst, cap, _TRUNCATE, "line %u: %s", line, message);
    else
        _snprintf_s(dst, cap, _TRUNCATE, "%s", message);
}

static int parse_double_strict(const char *s, double *out)
{
    char *end;
    double v;
    if (!s || !*s) return 0;
    v = strtod(s, &end);
    if (end == s || *end || !_finite(v)) return 0;
    *out = v;
    return 1;
}

static int parse_uint_strict(const char *s, int base, unsigned int *out)
{
    char *end;
    unsigned long v;
    if (!s || !*s || *s == '-') return 0;
    v = strtoul(s, &end, base);
    if (end == s || *end || v > 0xffffffffUL) return 0;
    *out = (unsigned int)v;
    return 1;
}

static int split_tokens(char *line, char **tok, int cap)
{
    char *ctx = NULL;
    char *p = strtok_s(line, " \t\r", &ctx);
    int n = 0;
    while (p) {
        if (n == cap) return -1;
        tok[n++] = p;
        p = strtok_s(NULL, " \t\r", &ctx);
    }
    return n;
}

static DG_XR_HAND *select_hand(DG_XR_FRAME *frame, const char *name)
{
    if (_stricmp(name, "left") == 0) return &frame->left_hand;
    if (_stricmp(name, "right") == 0) return &frame->right_hand;
    return NULL;
}

static DG_XR_HAND_POSE *select_pose(DG_XR_HAND *hand, const char *name)
{
    if (_stricmp(name, "grip") == 0) return &hand->grip;
    if (_stricmp(name, "aim") == 0) return &hand->aim;
    return NULL;
}

static void frame_identity(DG_XR_FRAME *frame)
{
    memset(frame, 0, sizeof(*frame));
    frame->head_raw.qw = 1.0;
    frame->eye[0].raw.qw = 1.0;
    frame->eye[1].raw.qw = 1.0;
    frame->left_hand.hand = DG_XR_HAND_LEFT;
    frame->right_hand.hand = DG_XR_HAND_RIGHT;
    frame->left_hand.grip.hand = frame->left_hand.aim.hand = DG_XR_HAND_LEFT;
    frame->right_hand.grip.hand = frame->right_hand.aim.hand = DG_XR_HAND_RIGHT;
    frame->left_hand.grip.kind = frame->right_hand.grip.kind = DG_XR_POSE_GRIP;
    frame->left_hand.aim.kind = frame->right_hand.aim.kind = DG_XR_POSE_AIM;
    frame->left_hand.grip.raw_local.qw = 1.0;
    frame->left_hand.aim.raw_local.qw = 1.0;
    frame->right_hand.grip.raw_local.qw = 1.0;
    frame->right_hand.aim.raw_local.qw = 1.0;
}

static void finalize_pose(DG_XR_HAND_POSE *pose, unsigned int hand,
                          unsigned int kind, uint64_t seq, int64_t time)
{
    pose->hand = hand;
    pose->kind = kind;
    pose->sample_seq = seq;
    pose->xr_time = time;
    pose->pose_age_ms = 0;
    if (pose->active) {
        pose->position_valid = 1;
        pose->orientation_valid = 1;
        pose->tracked = 1;
    } else {
        pose->position_valid = 0;
        pose->orientation_valid = 0;
        pose->tracked = 0;
    }
}

int dg_xr_script_program_parse_text(DG_XR_SCRIPT_PROGRAM *out,
                                    const char *text, size_t text_len,
                                    char *error, size_t error_cap)
{
    DG_XR_SCRIPT_STEP *steps = NULL;
    DG_XR_FRAME state;
    char *copy, *line, *cursor;
    unsigned int line_no = 0, count = 0, total = 0, tick_ms = SCRIPT_DEFAULT_TICK_MS;
    unsigned int pending_events = 0;
    int have_header = 0, dirty_since_hold = 0, ok = 0;

    if (out) memset(out, 0, sizeof(*out));
    if (error && error_cap) error[0] = 0;
    if (!out || !text || !text_len || text_len > SCRIPT_MAX_BYTES) {
        set_error(error, error_cap, 0, "empty or oversized script");
        return 0;
    }
    if (memchr(text, 0, text_len)) {
        set_error(error, error_cap, 0, "embedded NUL byte");
        return 0;
    }
    copy = (char *)malloc(text_len + 1);
    steps = (DG_XR_SCRIPT_STEP *)calloc(SCRIPT_MAX_STEPS, sizeof(*steps));
    if (!copy || !steps) {
        set_error(error, error_cap, 0, "out of memory");
        goto done;
    }
    memcpy(copy, text, text_len);
    copy[text_len] = 0;
    frame_identity(&state);

    cursor = copy;
    while (*cursor) {
        char *tok[16], *comment;
        char *newline;
        int n;
        line = cursor;
        newline = strchr(cursor, '\n');
        if (newline) { *newline = 0; cursor = newline + 1; }
        else cursor += strlen(cursor);
        comment = strchr(line, '#');
        line_no++;
        if (comment) *comment = 0;
        n = split_tokens(line, tok, (int)(sizeof(tok) / sizeof(tok[0])));
        if (n < 0) { set_error(error, error_cap, line_no, "too many fields"); goto done; }
        if (!n) continue;

        if (!have_header) {
            unsigned int version;
            if ((n != 2 && n != 3) || _stricmp(tok[0], "DGXR_SCRIPT") != 0 ||
                !parse_uint_strict(tok[1], 10, &version) || version != 1) {
                set_error(error, error_cap, line_no, "expected DGXR_SCRIPT 1 [tick_ms]");
                goto done;
            }
            if (n == 3 && (!parse_uint_strict(tok[2], 10, &tick_ms) ||
                           tick_ms < 5 || tick_ms > 100)) {
                set_error(error, error_cap, line_no, "tick_ms must be 5..100");
                goto done;
            }
            have_header = 1;
            continue;
        }

        if (_stricmp(tok[0], "pose") == 0) {
            DG_XR_HAND *hand;
            DG_XR_HAND_POSE *pose;
            unsigned int active;
            double v[7], norm;
            int i;
            if (n != 11 || !(hand = select_hand(&state, tok[1])) ||
                !(pose = select_pose(hand, tok[2])) ||
                !parse_uint_strict(tok[3], 10, &active) || active > 1) {
                set_error(error, error_cap, line_no, "bad pose command");
                goto done;
            }
            for (i = 0; i < 7; i++) if (!parse_double_strict(tok[4 + i], &v[i])) {
                set_error(error, error_cap, line_no, "pose contains a non-number");
                goto done;
            }
            if (fabs(v[0]) > 10.0 || fabs(v[1]) > 10.0 || fabs(v[2]) > 10.0) {
                set_error(error, error_cap, line_no, "pose position exceeds 10 metres");
                goto done;
            }
            norm = sqrt(v[3]*v[3] + v[4]*v[4] + v[5]*v[5] + v[6]*v[6]);
            if (!(norm > 0.99 && norm < 1.01)) {
                set_error(error, error_cap, line_no, "pose quaternion is not unit length");
                goto done;
            }
            pose->active = active;
            pose->raw_local.px = v[0]; pose->raw_local.py = v[1]; pose->raw_local.pz = v[2];
            pose->raw_local.qx = v[3] / norm; pose->raw_local.qy = v[4] / norm;
            pose->raw_local.qz = v[5] / norm; pose->raw_local.qw = v[6] / norm;
            dirty_since_hold = 1;
        } else if (_stricmp(tok[0], "head") == 0) {
            double v[7], norm;
            int i;
            if (n != 8) { set_error(error,error_cap,line_no,"head needs px py pz qx qy qz qw"); goto done; }
            for (i=0;i<7;i++) if (!parse_double_strict(tok[i+1],&v[i])) {
                set_error(error,error_cap,line_no,"head contains a non-number"); goto done;
            }
            norm=sqrt(v[3]*v[3]+v[4]*v[4]+v[5]*v[5]+v[6]*v[6]);
            if (fabs(v[0])>10 || fabs(v[1])>10 || fabs(v[2])>10 ||
                !(norm>.99 && norm<1.01)) {
                set_error(error,error_cap,line_no,"head position/quaternion out of range"); goto done;
            }
            state.head_raw.px=v[0]; state.head_raw.py=v[1]; state.head_raw.pz=v[2];
            state.head_raw.qx=v[3]/norm; state.head_raw.qy=v[4]/norm;
            state.head_raw.qz=v[5]/norm; state.head_raw.qw=v[6]/norm;
            dirty_since_hold=1;
        } else if (_stricmp(tok[0], "input") == 0) {
            DG_XR_HAND *hand;
            double trigger, squeeze, x, y;
            unsigned int buttons;
            if (n != 7 || !(hand = select_hand(&state, tok[1])) ||
                !parse_double_strict(tok[2], &trigger) ||
                !parse_double_strict(tok[3], &squeeze) ||
                !parse_double_strict(tok[4], &x) ||
                !parse_double_strict(tok[5], &y) ||
                !parse_uint_strict(tok[6], 0, &buttons) || buttons > 31u ||
                trigger < 0.0 || trigger > 1.0 || squeeze < 0.0 || squeeze > 1.0 ||
                x < -1.0 || x > 1.0 || y < -1.0 || y > 1.0) {
                set_error(error, error_cap, line_no, "bad input command");
                goto done;
            }
            hand->trigger_value = (float)trigger;
            hand->squeeze_value = (float)squeeze;
            hand->thumbstick_x = (float)x;
            hand->thumbstick_y = (float)y;
            hand->squeeze_click = (buttons & 1u) != 0;
            hand->thumbstick_click = (buttons & 2u) != 0;
            hand->primary_button = (buttons & 4u) != 0;
            hand->secondary_button = (buttons & 8u) != 0;
            hand->menu_button = (buttons & 16u) != 0;
            dirty_since_hold = 1;
        } else if (_stricmp(tok[0], "recenter") == 0) {
            if (n != 1) { set_error(error, error_cap, line_no, "recenter takes no fields"); goto done; }
            pending_events |= DG_XR_SCRIPT_EVENT_RECENTER;
            dirty_since_hold = 1;
        } else if (_stricmp(tok[0], "hold") == 0) {
            unsigned int ticks;
            if (n != 2 || !parse_uint_strict(tok[1], 10, &ticks) || !ticks ||
                ticks > SCRIPT_MAX_TICKS || total > SCRIPT_MAX_TICKS - ticks) {
                set_error(error, error_cap, line_no, "bad or excessive hold length");
                goto done;
            }
            if (count == SCRIPT_MAX_STEPS) {
                set_error(error, error_cap, line_no, "too many hold steps");
                goto done;
            }
            steps[count].start_tick = total;
            steps[count].ticks = ticks;
            steps[count].events = pending_events;
            steps[count].frame = state;
            pending_events = 0;
            total += ticks;
            count++;
            dirty_since_hold = 0;
        } else {
            set_error(error, error_cap, line_no, "unknown command");
            goto done;
        }
    }

    if (!have_header || !count) {
        set_error(error, error_cap, line_no, "script has no hold steps");
        goto done;
    }
    if (pending_events || dirty_since_hold) {
        set_error(error, error_cap, line_no, "state change has no following hold");
        goto done;
    }
    out->steps = steps;
    out->step_count = count;
    out->total_ticks = total;
    out->tick_ms = tick_ms;
    steps = NULL;
    ok = 1;
done:
    free(steps);
    free(copy);
    return ok;
}

void dg_xr_script_program_free(DG_XR_SCRIPT_PROGRAM *program)
{
    if (!program) return;
    free(program->steps);
    memset(program, 0, sizeof(*program));
}

int dg_xr_script_program_sample(const DG_XR_SCRIPT_PROGRAM *program,
                                uint64_t tick, DG_XR_FRAME *frame,
                                unsigned int *events)
{
    const DG_XR_SCRIPT_STEP *steps;
    unsigned int i;
    if (events) *events = 0;
    if (frame) frame_identity(frame);
    if (!program || !program->steps || !frame || tick >= program->total_ticks)
        return 0;
    steps = (const DG_XR_SCRIPT_STEP *)program->steps;
    for (i = 0; i < program->step_count; i++) {
        uint64_t end = (uint64_t)steps[i].start_tick + steps[i].ticks;
        if (tick < end) {
            *frame = steps[i].frame;
            if (events && tick == steps[i].start_tick) *events = steps[i].events;
            return 1;
        }
    }
    return 0;
}

/* ----------------------------------------------------------- live player */
static DG_XR_SCRIPT_PROGRAM g_program;
static DG_XR_CONFIG g_cfg;
static void (*g_log)(const char *fmt, ...);
static HANDLE g_thread, g_stop_event, g_go_event;
static volatile LONG g_started, g_finished;
static volatile LONG g_seq, g_valid;
static volatile LONG64 g_pub_ms;
static DG_XR_FRAME g_pub;
static volatile LONG g_turn_rate_mdeg, g_turn_offset_bits;
static volatile LONG64 g_turn_rate_ms;
static volatile LONG g_recenter_req;
static volatile LONG g_recenter_count;
static volatile LONG64 g_sample_seq, g_action_edge_seq;
static volatile LONG64 g_trigger_press[2], g_trigger_release[2];
static volatile LONG64 g_primary_press[2], g_menu_press;
static volatile LONG g_secondary_press[2], g_secondary_ignored[2];

static int hand_index(unsigned int hand)
{
    return hand == DG_XR_HAND_LEFT ? 0 : hand == DG_XR_HAND_RIGHT ? 1 : -1;
}

static void turn_offset_publish(double rad)
{
    float degrees = (float)(rad * 180.0 / 3.14159265358979323846);
    LONG bits;
    memcpy(&bits, &degrees, sizeof(bits));
    InterlockedExchange(&g_turn_offset_bits, bits);
}

void dg_xr_script_turn_rate(double deg_per_s)
{
    LONG rate;
    if (!(deg_per_s > -720.0 && deg_per_s < 720.0)) deg_per_s = 0.0;
    rate = (LONG)(deg_per_s * 1000.0);
    if (!rate) {
        InterlockedExchange(&g_turn_rate_mdeg, 0);
        InterlockedExchange64(&g_turn_rate_ms, (LONG64)GetTickCount64());
    } else {
        InterlockedExchange64(&g_turn_rate_ms, (LONG64)GetTickCount64());
        InterlockedExchange(&g_turn_rate_mdeg, rate);
    }
}

double dg_xr_script_turn_offset_rad(void)
{
    LONG bits = InterlockedCompareExchange(&g_turn_offset_bits, 0, 0);
    float degrees;
    memcpy(&degrees, &bits, sizeof(degrees));
    return (double)degrees * 3.14159265358979323846 / 180.0;
}

void dg_xr_script_recenter(void)
{
    InterlockedIncrement(&g_recenter_req);
}

long dg_xr_script_recenter_count(void)
{
    return InterlockedCompareExchange(&g_recenter_count, 0, 0);
}

uint64_t dg_xr_script_primary_press_seq(unsigned int hand)
{
    int i = hand_index(hand);
    return i < 0 ? 0 : (uint64_t)InterlockedCompareExchange64(&g_primary_press[i], 0, 0);
}

uint64_t dg_xr_script_menu_press_seq(void)
{
    return (uint64_t)InterlockedCompareExchange64(&g_menu_press, 0, 0);
}

unsigned long dg_xr_script_secondary_presses(unsigned int hand)
{
    int i = hand_index(hand);
    return i < 0 ? 0 : (unsigned long)InterlockedCompareExchange(&g_secondary_press[i], 0, 0);
}

unsigned long dg_xr_script_secondary_ignored(unsigned int hand)
{
    int i = hand_index(hand);
    return i < 0 ? 0 : (unsigned long)InterlockedCompareExchange(&g_secondary_ignored[i], 0, 0);
}

static void publish_frame(const DG_XR_FRAME *frame, int valid)
{
    InterlockedIncrement(&g_seq);
    MemoryBarrier();
    g_pub = *frame;
    InterlockedExchange(&g_valid, valid);
    InterlockedExchange64(&g_pub_ms, (LONG64)GetTickCount64());
    MemoryBarrier();
    InterlockedIncrement(&g_seq);
}

static void stamp_trigger_sequences(DG_XR_FRAME *frame)
{
    frame->left_hand.trigger_press_seq =
        (uint64_t)InterlockedCompareExchange64(&g_trigger_press[0], 0, 0);
    frame->left_hand.trigger_release_seq =
        (uint64_t)InterlockedCompareExchange64(&g_trigger_release[0], 0, 0);
    frame->right_hand.trigger_press_seq =
        (uint64_t)InterlockedCompareExchange64(&g_trigger_press[1], 0, 0);
    frame->right_hand.trigger_release_seq =
        (uint64_t)InterlockedCompareExchange64(&g_trigger_release[1], 0, 0);
}

int dg_xr_script_get_frame(DG_XR_FRAME *out)
{
    int tries;
    for (tries = 0; tries < 8; tries++) {
        LONG before = InterlockedCompareExchange(&g_seq, 0, 0), after, valid;
        LONG64 ms;
        DG_XR_FRAME frame;
        uint64_t age;
        if (before & 1) continue;
        MemoryBarrier();
        frame = g_pub;
        valid = InterlockedCompareExchange(&g_valid, 0, 0);
        ms = InterlockedCompareExchange64(&g_pub_ms, 0, 0);
        MemoryBarrier();
        after = InterlockedCompareExchange(&g_seq, 0, 0);
        if (before != after || (after & 1)) continue;
        age = (uint64_t)((LONG64)GetTickCount64() - ms);
        if (!valid || age > SCRIPT_STALE_MS) return 0;
#define AGE_POSE(p_) do { \
            (p_).pose_age_ms = age > 0xffffffffu ? 0xffffffffu : (unsigned int)age; \
            if (age > 100u) { (p_).position_valid = 0; (p_).orientation_valid = 0; \
                              (p_).tracked = 0; } \
        } while (0)
        AGE_POSE(frame.left_hand.grip);
        AGE_POSE(frame.left_hand.aim);
        AGE_POSE(frame.right_hand.grip);
        AGE_POSE(frame.right_hand.aim);
#undef AGE_POSE
        if (out) *out = frame;
        return 1;
    }
    return 0;
}

static void reset_reference(DG_XR_RAW_POSE *reference, double *offset)
{
    memset(reference, 0, sizeof(*reference));
    reference->qw = 1.0;
    *offset = 0.0;
    turn_offset_publish(0.0);
    InterlockedIncrement(&g_recenter_count);
}

static void transform_pose(DG_XR_HAND_POSE *pose,
                           const DG_XR_RAW_POSE *reference,
                           uint64_t seq, int64_t time)
{
    if (pose->active)
        dg_xr_pose_relative(reference, &pose->raw_local,
                            &pose->reference_relative);
    else
        memset(&pose->reference_relative, 0, sizeof(pose->reference_relative));
    finalize_pose(pose, pose->hand, pose->kind, seq, time);
}

static void transform_frame(DG_XR_FRAME *frame,
                            const DG_XR_RAW_POSE *reference,
                            const DG_XR_CONFIG *cfg,
                            uint64_t seq)
{
    DG_XR_REL_POSE head_rel;
    int64_t time = (int64_t)GetTickCount64() * 1000000;
    dg_xr_pose_relative(reference, &frame->head_raw, &head_rel);
    dg_xr_head_to_pose(head_rel.qx, head_rel.qy, head_rel.qz, head_rel.qw,
                       head_rel.px, head_rel.py, head_rel.pz, cfg,
                       &frame->head);
    frame->eye[0].pose = frame->eye[1].pose = frame->head;
    frame->eye[0].raw = frame->eye[1].raw = frame->head_raw;
    transform_pose(&frame->left_hand.grip, reference, seq, time);
    transform_pose(&frame->left_hand.aim, reference, seq, time);
    transform_pose(&frame->right_hand.grip, reference, seq, time);
    transform_pose(&frame->right_hand.aim, reference, seq, time);
}

static void process_edges(DG_XR_FRAME *frame, const DG_XR_FRAME *previous,
                          float deadzone, float fire)
{
    DG_XR_HAND *cur[2] = { &frame->left_hand, &frame->right_hand };
    const DG_XR_HAND *old[2] = { &previous->left_hand, &previous->right_hand };
    int i;
    for (i = 0; i < 2; i++) {
        unsigned int old_click = old[i]->trigger_click;
        unsigned int click = dg_xr_trigger_hysteresis(cur[i]->trigger_value,
                                                       deadzone, fire, old_click);
        cur[i]->trigger_click = click;
        if (!old_click && click)
            InterlockedExchange64(&g_trigger_press[i],
                                  InterlockedIncrement64(&g_action_edge_seq));
        else if (old_click && !click)
            InterlockedExchange64(&g_trigger_release[i],
                                  InterlockedIncrement64(&g_action_edge_seq));
        if (!old[i]->primary_button && cur[i]->primary_button)
            InterlockedIncrement64(&g_primary_press[i]);
        if (i == 0 && !old[i]->menu_button && cur[i]->menu_button)
            InterlockedIncrement64(&g_menu_press);
        if (!old[i]->secondary_button && cur[i]->secondary_button) {
            if (click) InterlockedIncrement(&g_secondary_press[i]);
            else InterlockedIncrement(&g_secondary_ignored[i]);
        }
    }
    stamp_trigger_sequences(frame);
}

static int foreground_is_ours(void)
{
    DWORD pid = 0;
    HWND window = GetForegroundWindow();
    if (window) GetWindowThreadProcessId(window, &pid);
    return pid == GetCurrentProcessId();
}

typedef struct {
    uint64_t tick;
    unsigned int recenter_seen;
    double turn_offset;
    DG_XR_FRAME previous;
    DG_XR_RAW_POSE reference;
} DG_XR_SCRIPT_PLAYER;

static void player_init(DG_XR_SCRIPT_PLAYER *player)
{
    memset(player, 0, sizeof(*player));
    frame_identity(&player->previous);
    reset_reference(&player->reference, &player->turn_offset);
    player->recenter_seen =
        (unsigned int)InterlockedCompareExchange(&g_recenter_req, 0, 0);
}

static int player_step(DG_XR_SCRIPT_PLAYER *player,
                       const DG_XR_SCRIPT_PROGRAM *program,
                       const DG_XR_CONFIG *cfg, DG_XR_FRAME *frame)
{
    unsigned int events = 0;
    unsigned int requested;
    LONG rate;
    float deadzone = (float)cfg->trigger_deadzone;
    float fire = (float)cfg->trigger_fire;
    if (!dg_xr_script_program_sample(program, player->tick, frame, &events))
        return 0;
    requested =
        (unsigned int)InterlockedCompareExchange(&g_recenter_req, 0, 0);
    if ((events & DG_XR_SCRIPT_EVENT_RECENTER) ||
        player->recenter_seen != requested) {
        player->recenter_seen = requested;
        reset_reference(&player->reference, &player->turn_offset);
    }
    rate = InterlockedCompareExchange(&g_turn_rate_mdeg, 0, 0);
    if ((LONG64)GetTickCount64() -
            InterlockedCompareExchange64(&g_turn_rate_ms, 0, 0) >
        SCRIPT_TURN_STALE_MS) {
        /* The demand comes from the camera seam. If that seam pauses, a held
           last value is not a controller any more and must not keep turning. */
        rate = 0;
        InterlockedExchange(&g_turn_rate_mdeg, 0);
    }
    if (rate) {
        double delta = (double)rate / 1000.0 *
                       ((double)program->tick_ms / 1000.0) *
                       3.14159265358979323846 / 180.0;
        double q[2] = { player->reference.qy, player->reference.qw };
        double p[2] = { player->reference.px, player->reference.pz };
        dg_xr_turn_step(q, p, frame->head_raw.px, frame->head_raw.pz, delta);
        player->reference.qy = q[0]; player->reference.qw = q[1];
        player->reference.px = p[0]; player->reference.pz = p[1];
        player->turn_offset += delta;
        turn_offset_publish(player->turn_offset);
    }
    process_edges(frame, &player->previous, deadzone, fire);
    transform_frame(frame, &player->reference, cfg,
                    (uint64_t)InterlockedIncrement64(&g_sample_seq));
    player->previous = *frame;
    player->tick++;
    return 1;
}

static void player_neutral(DG_XR_SCRIPT_PLAYER *player,
                           const DG_XR_CONFIG *cfg, DG_XR_FRAME *frame)
{
    float deadzone = (float)cfg->trigger_deadzone;
    float fire = (float)cfg->trigger_fire;
    *frame = player->previous;
    frame->left_hand.trigger_value = frame->right_hand.trigger_value = 0.0f;
    frame->left_hand.squeeze_value = frame->right_hand.squeeze_value = 0.0f;
    frame->left_hand.thumbstick_x = frame->left_hand.thumbstick_y = 0.0f;
    frame->right_hand.thumbstick_x = frame->right_hand.thumbstick_y = 0.0f;
    frame->left_hand.squeeze_click = frame->right_hand.squeeze_click = 0;
    frame->left_hand.thumbstick_click = frame->right_hand.thumbstick_click = 0;
    frame->left_hand.primary_button = frame->right_hand.primary_button = 0;
    frame->left_hand.secondary_button = frame->right_hand.secondary_button = 0;
    frame->left_hand.menu_button = frame->right_hand.menu_button = 0;
    process_edges(frame, &player->previous, deadzone, fire);
    transform_frame(frame, &player->reference, cfg,
                    (uint64_t)InterlockedIncrement64(&g_sample_seq));
    player->previous = *frame;
}

#ifdef DG_HOOK_TEST
static DG_XR_SCRIPT_PLAYER g_test_player;
static const DG_XR_SCRIPT_PROGRAM *g_test_program;
static DG_XR_CONFIG g_test_cfg;
static int g_test_active;

int dg_xr_script_test_begin(const DG_XR_SCRIPT_PROGRAM *program,
                            const DG_XR_CONFIG *cfg)
{
    if (!program || !program->steps || !program->step_count || !cfg ||
        InterlockedCompareExchange(&g_started, 0, 0))
        return 0;
    g_test_cfg = *cfg;
    if (!(g_test_cfg.trigger_deadzone >= 0.0 &&
          g_test_cfg.trigger_deadzone < 1.0 &&
          g_test_cfg.trigger_fire > g_test_cfg.trigger_deadzone &&
          g_test_cfg.trigger_fire <= 1.0)) {
        g_test_cfg.trigger_deadzone = 0.10;
        g_test_cfg.trigger_fire = 0.55;
    }
    dg_xr_script_turn_rate(0.0);
    InterlockedExchange(&g_recenter_req, 0);
    player_init(&g_test_player);
    g_test_program = program;
    g_test_active = 1;
    return 1;
}

int dg_xr_script_test_step(DG_XR_FRAME *frame)
{
    DG_XR_FRAME empty;
    if (!g_test_active || !frame) return 0;
    if (!player_step(&g_test_player, g_test_program, &g_test_cfg, frame)) {
        dg_xr_script_turn_rate(0.0);
        player_neutral(&g_test_player, &g_test_cfg, frame);
        g_test_active = 0;
        frame_identity(&empty);
        publish_frame(&empty, 0);
        return 0;
    }
    publish_frame(frame, 1);
    return 1;
}

int dg_xr_script_test_abort(DG_XR_FRAME *neutral_frame)
{
    DG_XR_FRAME empty;
    if (!g_test_active || !neutral_frame) return 0;
    dg_xr_script_turn_rate(0.0);
    player_neutral(&g_test_player, &g_test_cfg, neutral_frame);
    /* Return the release frame for assertions, then model the worker's final
       state without sleeps: provider invalid and no physical fallback. */
    frame_identity(&empty);
    publish_frame(&empty, 0);
    g_test_active = 0;
    return 1;
}
#endif

static DWORD WINAPI script_thread(LPVOID unused)
{
    HANDLE waits[2] = { g_stop_event, g_go_event };
    DG_XR_SCRIPT_PLAYER player;
    DG_XR_FRAME frame;
    DWORD wait;
    (void)unused;

    player_init(&player);
    wait = WaitForMultipleObjects(2, waits, FALSE, INFINITE);
    if (wait != WAIT_OBJECT_0 + 1) goto release;

    /* Marker activation commonly happens from Codex or an editor. Give the
       user a bounded window to put the game in front without consuming tick
       zero. Keep republishing a neutral identity frame so the camera/provider
       does not age stale while waiting. Once playback starts, the stricter
       focus-loss rule in the main loop aborts permanently. */
    if (!foreground_is_ours()) {
        ULONGLONG deadline = GetTickCount64() + SCRIPT_FOCUS_WAIT_MS;
        if (g_log) g_log("  xr-script: waiting up to 30 s for game focus\r\n");
        for (;;) {
            if (GetAsyncKeyState(VK_ESCAPE) & 0x8000) {
                if (g_log) g_log("  xr-script: aborted by Escape while waiting for focus\r\n");
                goto release;
            }
            if (foreground_is_ours()) {
                if (g_log) g_log("  xr-script: game focused; starting tick 0\r\n");
                break;
            }
            if (GetTickCount64() >= deadline) {
                if (g_log) g_log("  xr-script: aborted after 30 s without game focus\r\n");
                goto release;
            }
            player_neutral(&player, &g_cfg, &frame);
            publish_frame(&frame, 1);
            if (WaitForSingleObject(g_stop_event, g_program.tick_ms) ==
                WAIT_OBJECT_0)
                goto release;
        }
    } else if (g_log) {
        g_log("  xr-script: game focused; starting tick 0\r\n");
    }

    for (;;) {
        ULONGLONG before, elapsed;
        if (WaitForSingleObject(g_stop_event, 0) == WAIT_OBJECT_0) break;
        if ((GetAsyncKeyState(VK_ESCAPE) & 0x8000) || !foreground_is_ours()) {
            if (g_log) g_log("  xr-script: aborted (Escape or game lost foreground)\r\n");
            break;
        }
        before = GetTickCount64();
        if (!player_step(&player, &g_program, &g_cfg, &frame)) {
            if (g_log) g_log("  xr-script: completed %llu ticks\r\n",
                             (unsigned long long)player.tick);
            break;
        }
        publish_frame(&frame, 1);

        wait = WaitForSingleObject(g_stop_event, g_program.tick_ms);
        if (wait == WAIT_OBJECT_0) break;
        elapsed = GetTickCount64() - before;
        if (elapsed > SCRIPT_STALE_MS) {
            if (g_log) g_log("  xr-script: aborted after %llu ms scheduler stall\r\n",
                             (unsigned long long)elapsed);
            break;
        }
    }

release:
    {
    unsigned int neutral_ticks, i;
    dg_xr_script_turn_rate(0.0);
    /* Keep neutral valid for at least 100 ms and at least two producer ticks.
       Two 10 ms samples can both fit between 30 Hz game seams; a time floor
       makes the release observable while retaining deterministic values. */
    neutral_ticks = (100u + g_program.tick_ms - 1u) / g_program.tick_ms;
    if (neutral_ticks < 2u) neutral_ticks = 2u;
    for (i = 0; i < neutral_ticks; i++) {
        player_neutral(&player, &g_cfg, &frame);
        publish_frame(&frame, 1);
        Sleep(g_program.tick_ms);
    }
    frame_identity(&frame);
    publish_frame(&frame, 0);
    InterlockedExchange(&g_finished, 1);
    return 0;
    }
}

static int read_program_file(const char *path, char **text_out, size_t *size_out)
{
    HANDLE file;
    LARGE_INTEGER size;
    DWORD got;
    char *text;
    *text_out = NULL; *size_out = 0;
    file = CreateFileA(path, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING,
                       FILE_ATTRIBUTE_NORMAL, NULL);
    if (file == INVALID_HANDLE_VALUE) return 0;
    if (!GetFileSizeEx(file, &size) || size.QuadPart <= 0 ||
        size.QuadPart > SCRIPT_MAX_BYTES) { CloseHandle(file); return 0; }
    text = (char *)malloc((size_t)size.QuadPart + 1);
    if (!text) { CloseHandle(file); return 0; }
    if (!ReadFile(file, text, (DWORD)size.QuadPart, &got, NULL) ||
        got != (DWORD)size.QuadPart) {
        free(text); CloseHandle(file); return 0;
    }
    CloseHandle(file);
    text[got] = 0;
    *text_out = text; *size_out = got;
    return 1;
}

int dg_xr_script_start(const char *path, const DG_XR_CONFIG *cfg,
                       void (*log)(const char *fmt, ...))
{
    char *text = NULL, error[160] = { 0 };
    size_t size = 0;
    DG_XR_FRAME neutral;
    if (!path || !cfg || InterlockedCompareExchange(&g_started, 1, 0) != 0)
        return 0;
    if (!read_program_file(path, &text, &size) ||
        !dg_xr_script_program_parse_text(&g_program, text, size,
                                         error, sizeof(error))) {
        if (log) log("  xr-script: cannot load %s%s%s\r\n", path,
                     error[0] ? " - " : "", error);
        free(text);
        InterlockedExchange(&g_started, 0);
        return 0;
    }
    free(text);
    g_cfg = *cfg;
    if (!(g_cfg.trigger_deadzone >= 0.0 && g_cfg.trigger_deadzone < 1.0 &&
          g_cfg.trigger_fire > g_cfg.trigger_deadzone && g_cfg.trigger_fire <= 1.0)) {
        g_cfg.trigger_deadzone = 0.10;
        g_cfg.trigger_fire = 0.55;
    }
    g_log = log;
    InterlockedExchange(&g_finished, 0);
    InterlockedExchange(&g_turn_rate_mdeg, 0);
    InterlockedExchange64(&g_turn_rate_ms, (LONG64)GetTickCount64());
    InterlockedExchange(&g_recenter_req, 0);
    g_stop_event = CreateEventA(NULL, TRUE, FALSE, NULL);
    g_go_event = CreateEventA(NULL, TRUE, FALSE, NULL);
    if (!g_stop_event || !g_go_event) goto fail;
    frame_identity(&neutral);
    stamp_trigger_sequences(&neutral);
    transform_frame(&neutral, &neutral.head_raw, &g_cfg,
                    (uint64_t)InterlockedIncrement64(&g_sample_seq));
    publish_frame(&neutral, 1);
    g_thread = CreateThread(NULL, 0, script_thread, NULL, 0, NULL);
    if (!g_thread) goto fail;
    if (g_log) g_log("  xr-script: loaded %u steps, %u ticks at %u ms\r\n",
                     g_program.step_count, g_program.total_ticks,
                     g_program.tick_ms);
    return 1;
fail:
    if (g_stop_event) { CloseHandle(g_stop_event); g_stop_event = NULL; }
    if (g_go_event) { CloseHandle(g_go_event); g_go_event = NULL; }
    dg_xr_script_program_free(&g_program);
    frame_identity(&neutral);
    publish_frame(&neutral, 0);
    InterlockedExchange(&g_started, 0);
    return 0;
}

void dg_xr_script_go(void)
{
    if (InterlockedCompareExchange(&g_started, 0, 0) && g_go_event)
        SetEvent(g_go_event);
}

void dg_xr_script_stop(void)
{
    DG_XR_FRAME empty;
    DWORD wait;
    if (!InterlockedCompareExchange(&g_started, 0, 0)) return;
    if (g_stop_event) SetEvent(g_stop_event);
    if (g_thread) {
        wait = WaitForSingleObject(g_thread, 3000);
        if (wait != WAIT_OBJECT_0) {
            /* The worker may still read g_program and publish to g_pub. Keep
               every handle and allocation alive, keep g_started owned, and
               permit a later stop call to reap it. */
            if (g_log) g_log("  xr-script: stop timed out; worker still owns resources\r\n");
            InterlockedExchange(&g_finished, -1);
            return;
        }
        CloseHandle(g_thread);
        g_thread = NULL;
    }
    if (g_stop_event) { CloseHandle(g_stop_event); g_stop_event = NULL; }
    if (g_go_event) { CloseHandle(g_go_event); g_go_event = NULL; }
    dg_xr_script_program_free(&g_program);
    frame_identity(&empty);
    publish_frame(&empty, 0);
    InterlockedExchange(&g_started, 0);
}

int dg_xr_script_finished(void)
{
    return (int)InterlockedCompareExchange(&g_finished, 0, 0);
}
