#include <limits.h>
#include <string.h>

#include "dg_script_gate.h"

static int eq(const char *a, size_t n, const char *b)
{
    size_t i, m = strlen(b);
    if (n != m) return 0;
    for (i = 0; i < n; i++) {
        char x = a[i], y = b[i];
        if (x >= 'A' && x <= 'Z') x = (char)(x + ('a' - 'A'));
        if (y >= 'A' && y <= 'Z') y = (char)(y + ('a' - 'A'));
        if (x != y) return 0;
    }
    return 1;
}

static int decimal(const char *s, size_t n, int *out)
{
    size_t i;
    unsigned long long v = 0;
    if (!n) return 0;
    for (i = 0; i < n; i++) {
        unsigned int d;
        if (s[i] < '0' || s[i] > '9') return 0;
        d = (unsigned int)(s[i] - '0');
        if (v > ((unsigned long long)INT_MAX - d) / 10ull) return 0;
        v = v * 10ull + d;
    }
    *out = (int)v;
    return 1;
}

int dg_script_marker_parse(const char *text, size_t len,
                           DG_SCRIPT_MARKER_INFO *out)
{
    size_t pos = 0;
    DG_SCRIPT_MARKER_INFO v;
    int token_seen = 0, source_seen = 0;
    if (out) memset(out, 0, sizeof(*out));
    if (!text || !out || !len || len > DG_SCRIPT_MARKER_MAX_BYTES || memchr(text, 0, len)) return 0;
    memset(&v, 0, sizeof(v));
    while (pos < len) {
        size_t e = pos, key_end, k0, k1, v0, v1;
        while (e < len && text[e] != '\n') e++;
        k0 = pos;
        while (k0 < e && (text[k0] == ' ' || text[k0] == '\t' || text[k0] == '\r')) k0++;
        k1 = k0;
        while (k1 < e && text[k1] != '=' && text[k1] != ' ' && text[k1] != '\t' && text[k1] != '\r') k1++;
        key_end = k1;
        while (k1 < e && (text[k1] == ' ' || text[k1] == '\t')) k1++;
        if (k1 < e && text[k1] == '=') {
            k1++;
            v0 = k1;
            while (v0 < e && (text[v0] == ' ' || text[v0] == '\t')) v0++;
            v1 = e;
            while (v1 > v0 && (text[v1 - 1] == ' ' || text[v1 - 1] == '\t' || text[v1 - 1] == '\r')) v1--;
            if (eq(text + k0, key_end - k0, "source")) {
                if (source_seen) return 0;
                source_seen = 1;
                v.source_script = (v1 - v0 == 6 && eq(text + v0, 6, "script"));
            } else if (eq(text + k0, key_end - k0, "vr_script_start")) {
                if (token_seen) return 0;
                token_seen = 1;
                v.token_present = 1;
                v.token_valid = decimal(text + v0, v1 - v0, &v.token);
                if (!v.token_valid) return 0;
            }
        } else if (eq(text + k0, key_end - k0, "source") ||
                   eq(text + k0, key_end - k0, "vr_script_start")) {
            return 0;
        }
        pos = e < len ? e + 1 : len;
    }
    if (v.source_script && !v.token_present) {
        v.token_valid = 1;
        v.token = 0;
    }
    *out = v;
    return 1;
}

void dg_script_gate_init(DG_SCRIPT_GATE *g, int baseline)
{
    if (!g) return;
    g->initialized = baseline >= 0;
    if (baseline < 0) baseline = 0;
    g->baseline = baseline;
    g->high_water = baseline;
    g->active = 0;
    g->requested = 0;
    g->busy = 0;
}

int dg_script_gate_offer(DG_SCRIPT_GATE *g, int source_script,
                         int token_valid, int token)
{
    if (!g || !g->initialized || g->busy || !source_script || !token_valid || token <= 0 ||
        token <= g->high_water) return 0;
    g->high_water = token;
    g->active = token;
    g->requested = 1;
    g->busy = 1;
    return 1;
}

void dg_script_gate_consume(DG_SCRIPT_GATE *g)
{
    if (g) g->requested = 0;
}

int dg_script_gate_observe(DG_SCRIPT_GATE *g, int source_script,
                           int token_valid, int token)
{
    if (!g || !g->busy) return 0;
    if (token_valid && token > g->high_water) g->high_water = token;
    if (!source_script || !token_valid || token <= 0 || token != g->active) {
        /* Remain busy until the caller completes release/cleanup. Later
           observations during cleanup must still discard newer commands. */
        return 0;
    }
    return 1;
}

void dg_script_gate_finish(DG_SCRIPT_GATE *g)
{
    if (!g) return;
    g->active = 0;
    g->requested = 0;
    g->busy = 0;
}

int dg_script_gate_poll(DG_SCRIPT_GATE *g, int read_ok,
                        const DG_SCRIPT_MARKER_INFO *info)
{
    if (!g || !read_ok || !info || !info->source_script || !info->token_valid)
        return 0;
    if (!g->initialized) {
        g->baseline = info->token > 0 ? info->token : 0;
        g->high_water = g->baseline;
        g->initialized = 1;
        return -1;
    }
    if (!dg_script_gate_offer(g, 1, 1, info->token)) return 0;
    dg_script_gate_consume(g);
    return 1;
}
