/*
 * dg_anchors.h - the F1B anchor tables and the resolver both halves share.
 *
 * The external scanner (scanner\dg_f1b.c) and the in-process bridge
 * (asi\dg_hook\dg_bridge.c) must agree, byte for byte, on what an anchor IS.
 * Two copies of a signature table drift the moment one of them is edited, and
 * the drift is invisible: both keep passing their own tests while proving
 * different things about the same executable. So there is exactly one table,
 * one fingerprint, one set of semantic cross-checks, and one resolver here.
 *
 * Everything is a static function in a header on purpose: each consumer is a
 * single translation unit, there is no library to link, and the resolver has no
 * global state of its own. It works on a LiveImage - a byte view of a loaded
 * PE, plus a per-byte validity map - which the scanner fills with
 * ReadProcessMemory and the bridge fills from its own module. The resolver
 * itself never touches a process handle, never writes, and never assumes it is
 * looking at a remote process.
 *
 * Fail-closed is the contract: every lookup requires exactly one hit, every
 * cross-check must pass, and a partial result is reported as no result.
 */

#ifndef DG_ANCHORS_H
#define DG_ANCHORS_H

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <bcrypt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#pragma comment(lib, "bcrypt.lib")

/* ------------------------------------------------ approved retail identity --- */

#define GAME_EXE "METAL GEAR SOLID2.exe"
#define EXPECTED_FILE_SIZE 10811976ULL
#define EXPECTED_SHA256 \
    "9C8575E6B2D6449636D1A04B63EF8C9E44C3AECF84CADDDC27B4874C62688F2B"
#define EXPECTED_TIMESTAMP 0x696EFC83UL
#define EXPECTED_ENTRY_RVA 0x0186B310UL
#define EXPECTED_IMAGE_BASE 0x0000000140000000ULL
#define EXPECTED_IMAGE_SIZE 0x018A5000UL
#define MAX_PATTERN_BYTES 64
#define MAX_HITS_SAVED 4
#define READ_CHUNK 0x10000UL
#define MAX_FUNCTION_REFS 256
#define MAX_CLUSTER_FUNCTIONS 8
#define MAX_DIRECT_CALLS 128

/* ------------------------------------------------------------- types --- */

/* leaf_accessor marks the handful of anchors that legitimately have no .pdata
   record. See leaf_accessor_bounds: the exemption is per-anchor and still costs
   a second, independent structural constraint. */
typedef struct PatternDef {
    const char *name;
    const char *text;
    int ref_count;
    int disp_offset[2];
    int instruction_end[2];
    int require_writable_target;
    int leaf_accessor;
} PatternDef;

typedef struct ParsedPattern {
    unsigned char bytes[MAX_PATTERN_BYTES];
    unsigned char exact[MAX_PATTERN_BYTES];
    size_t length;
} ParsedPattern;

typedef struct PatternResult {
    size_t count;
    DWORD hits[MAX_HITS_SAVED];
    ULONGLONG targets[2];
    DWORD function_begin;
    DWORD function_end;
    int function_found;
    int leaf_function;
    int ok;
} PatternResult;

typedef struct LiveImage {
    unsigned char *bytes;
    unsigned char *valid;
    DWORD size;
    ULONGLONG base;
    IMAGE_NT_HEADERS64 *nt;
    IMAGE_SECTION_HEADER *sections;
    WORD section_count;
} LiveImage;

typedef struct DiskPe {
    ULONGLONG file_size;
    DWORD timestamp;
    DWORD entry_rva;
    ULONGLONG image_base;
    DWORD image_size;
    WORD machine;
    WORD section_count;
} DiskPe;

typedef enum RefKind {
    REF_LOAD,
    REF_STORE,
    REF_READWRITE,
    REF_LEA
} RefKind;

typedef struct RipRef {
    DWORD instruction;
    DWORD instruction_end;
    ULONGLONG target;
    RefKind kind;
    int width;
    int immediate_valid;
    ULONGLONG immediate;
} RipRef;

typedef struct FunctionRefs {
    DWORD begin;
    DWORD end;
    RipRef refs[MAX_FUNCTION_REFS];
    size_t count;
} FunctionRefs;

/* pad_stop_aim, not pad_subject_toggle: PL_PAD_SUBJECT_TOGGLE is compiled out
   of this build along with its readers - see find_pad_masks. */
typedef struct PadMaskResult {
    ULONGLONG pad_subject;
    ULONGLONG pad_stop_aim;
    ULONGLONG pad_weapon;
    ULONGLONG pad_press_weapon;
    ULONGLONG subject_move;
    ULONGLONG subject_toggle;
    DWORD cluster_base;
    DWORD cluster_slots;
    DWORD pattern_function;
    DWORD dispatcher_function;
    int ok;
} PadMaskResult;

typedef struct PlayerPadResult {
    DWORD function_begin;
    DWORD function_end;
    DWORD copy_begin;
    DWORD merge_seam;

    DWORD copy_seam;
    DWORD copy_function_begin;
    DWORD copy_function_end;
    ULONGLONG player_pad;
    ULONGLONG gv_pad_data;
    int ok;
} PlayerPadResult;

#define DG_GV_PAD_STRIDE 40
#define DG_GV_PAD_FLAG_OFFSET 36

#define DG_GV_PAD_STATUS_OFFSET 4
#define DG_GV_PAD_PRESS_OFFSET 8
#define DG_GV_PAD_COUNT 4

/* GV_UpdatePadSystem's scenario-press machinery. gv_pad_press is the anchor;
   everything else is carried because it is what a later detour on this path
   would need and it costs nothing to report now. See find_pad_press. */
typedef struct PadPressResult {
    DWORD function_begin;       /* GV_UpdatePadSystem's unwind bounds */
    DWORD function_end;
    DWORD press_read;           /* the first `test flag,0x20`-guarded read */
    DWORD press_clear;          /* `and reg, ~GV_PAD_PRESS_SCN` */
    DWORD direct_seam;          /* just past UpdatePad(&GV_PadDataDirect[0]) */
    ULONGLONG gv_pad_press;
    ULONGLONG gv_pad_data_direct;   /* 0 unless verified, never assumed */
    int ok;
} PadPressResult;

typedef struct WristResult {
    DWORD function_begin;
    DWORD function_end;
    DWORD adjust_store;
    DWORD adjust_or;
    DWORD clamp;
    DWORD seam;
    int ok;
} WristResult;

/* No NewBullet fields: GM_SetWeaponFire has many call sites in retail, so there
   is no unique producer to anchor - see find_fire_publication. */
typedef struct FireResult {
    DWORD publisher_begin;
    DWORD publisher_end;
    DWORD setter;
    DWORD publication;
    DWORD setter_callers;
    ULONGLONG weapon_fire_private;
    ULONGLONG gm_weapon_fire;
    ULONGLONG player_body;
    ULONGLONG player_arm_body;
    int ok;
} FireResult;

typedef struct ExtensionResult {
    PadMaskResult masks;
    PlayerPadResult player_pad;
    PadPressResult pad_press;
    WristResult wrist;
    FireResult fire;
    int relationships_ok;
} ExtensionResult;

/*
 * The first six patterns are the exact game-image patterns used by the
 * official MGSHDFix 4.0.1 FPS feature. The last two are official GameVars
 * anchors useful for subject-state and aiming validation.
 *
 * disp_offset is relative to the match and points at a signed RIP disp32.
 * instruction_end is also relative to the match.
 */
static const PatternDef g_patterns[] = {
    {
        "FPS Toggle -> PL_SubjectToggle",
        "8B 05 ?? ?? ?? ?? 89 05 ?? ?? ?? ?? 48 8B 97",
        2, { 2, 8 }, { 6, 12 }, 1
    },
    {
        "FPS Override + Active relation",
        "8B 15 ?? ?? ?? ?? 8B 0D ?? ?? ?? ?? 85 D2",
        2, { 2, 8 }, { 6, 12 }, 1
    },
    {
        "FPS Active branch",
        "8B 0D ?? ?? ?? ?? 85 D2 0F 85",
        1, { 2, 0 }, { 6, 0 }, 1
    },
    {
        "FPS Move -> PL_SubjectMove",
        "8B 05 ?? ?? ?? ?? 89 05 ?? ?? ?? ?? 85 C0",
        2, { 2, 8 }, { 6, 12 }, 1
    },
    {
        "NewDivingGoggles act branch",
        "0F 84 ?? ?? ?? ?? 33 C9 E8 ?? ?? ?? ?? 39 6F",
        0, { 0, 0 }, { 0, 0 }, 0
    },
    {
        "CheckSubjectMoveCamera aim height",
        "F3 0F 11 2D ?? ?? ?? ?? F7 83",
        1, { 4, 0 }, { 8, 0 }, 1
    },
    {
        "GameVars aimingState",
        "8B 2D ?? ?? ?? ?? F3 0F 10 1D",
        1, { 2, 0 }, { 6, 0 }, 1
    },
    {
        /* A leaf getter with no .pdata record of its own - see
           leaf_accessor_bounds for what stands in for the unwind bounds. */
        "GameVars GM_PlayerStatus",
        "48 8B 05 ?? ?? ?? ?? 48 23 C1 C3",
        1, { 3, 0 }, { 7, 0 }, 1, 1
    },
    {

        "GM_GameStatus | GM_GameStatusScn demo test",
        "8B 05 ?? ?? ?? ?? 0B 05 ?? ?? ?? ?? A9 00 00 00 F8",
        2, { 2, 8 }, { 6, 12 }, 1
    },
    {

        "GM_MenuStatus | GM_MenuStatusScn radio test",
        "8B 15 ?? ?? ?? ?? 0B 15 ?? ?? ?? ?? F7 C2 04 07 00 00",
        2, { 2, 8 }, { 6, 12 }, 1
    },
    {

        "GM_PlayerArmBody read (weapon_body switch, subjective half)",
        "48 8B 05 ?? ?? ?? ?? 8B 93 28 14 00 00 48 8B 8B 90 02 00 00 "
        "48 89 83 A8 0B 00 00 C7 83 B0 0B 00 00 06 00 00 00",
        1, { 3, 0 }, { 7, 0 }, 1
    },
    {

        "GM_PlayerArmBody write (GetResources)",
        "48 8D 8F D0 00 00 00 48 89 1D ?? ?? ?? ?? 48 8D 05",
        1, { 10, 0 }, { 14, 0 }, 1
    },
    {

        "SetPos ArmCamRotateShift branch",
        "66 83 3D ?? ?? ?? ?? 00 48 8D 54 24 ?? 48 8D 0D ?? ?? ?? ?? 74",
        2, { 3, 16 }, { 8, 20 }, 1
    }
};

#define PATTERN_COUNT (sizeof(g_patterns) / sizeof(g_patterns[0]))

/* --------------------------------------------------------- resolution --- */

static int is_hex_digit_char(char c)
{
    return isxdigit((unsigned char)c) != 0;
}

static int hex_value(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    c = (char)toupper((unsigned char)c);
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static int parse_pattern(const char *text, ParsedPattern *out)
{
    const char *p = text;
    memset(out, 0, sizeof(*out));

    while (*p) {
        int hi;
        int lo;
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) break;
        if (out->length >= MAX_PATTERN_BYTES) return 0;

        if (p[0] == '?' && p[1] == '?') {
            out->bytes[out->length] = 0;
            out->exact[out->length] = 0;
            out->length++;
            p += 2;
        } else {
            if (!is_hex_digit_char(p[0]) || !is_hex_digit_char(p[1]))
                return 0;
            hi = hex_value(p[0]);
            lo = hex_value(p[1]);
            out->bytes[out->length] = (unsigned char)((hi << 4) | lo);
            out->exact[out->length] = 1;
            out->length++;
            p += 2;
        }
        if (*p && *p != ' ' && *p != '\t') return 0;
    }
    return out->length != 0;
}

static int all_valid(const unsigned char *valid, size_t offset, size_t length,
                     size_t limit)
{
    size_t i;
    if (offset > limit || length > limit - offset) return 0;
    for (i = 0; i < length; i++) {
        if (!valid[offset + i]) return 0;
    }
    return 1;
}

static int pattern_matches(const unsigned char *bytes,
                           const unsigned char *valid,
                           size_t limit,
                           size_t offset,
                           const ParsedPattern *pattern)
{
    size_t i;
    if (!all_valid(valid, offset, pattern->length, limit)) return 0;
    for (i = 0; i < pattern->length; i++) {
        if (pattern->exact[i] && bytes[offset + i] != pattern->bytes[i])
            return 0;
    }
    return 1;
}

static ULONGLONG resolve_rip(const LiveImage *image, DWORD hit,
                            int disp_offset, int instruction_end, int *ok)
{
    LONGLONG target;
    LONG displacement;
    size_t disp = (size_t)hit + (size_t)disp_offset;

    *ok = 0;
    if (!all_valid(image->valid, disp, sizeof(displacement), image->size))
        return 0;
    memcpy(&displacement, image->bytes + disp, sizeof(displacement));
    target = (LONGLONG)image->base + (LONGLONG)hit +
             (LONGLONG)instruction_end + (LONGLONG)displacement;
    if (target < (LONGLONG)image->base ||
        target >= (LONGLONG)(image->base + image->size))
        return 0;
    *ok = 1;
    return (ULONGLONG)target;
}

static int protection_readable(DWORD protection)
{
    DWORD basic;
    if ((protection & PAGE_GUARD) || (protection & PAGE_NOACCESS))
        return 0;
    basic = protection & 0xFF;
    return basic == PAGE_READONLY ||
           basic == PAGE_READWRITE ||
           basic == PAGE_WRITECOPY ||
           basic == PAGE_EXECUTE ||
           basic == PAGE_EXECUTE_READ ||
           basic == PAGE_EXECUTE_READWRITE ||
           basic == PAGE_EXECUTE_WRITECOPY;
}

static const IMAGE_SECTION_HEADER *section_for_rva(const LiveImage *image,
                                                    DWORD rva)
{
    WORD i;
    for (i = 0; i < image->section_count; i++) {
        const IMAGE_SECTION_HEADER *section = &image->sections[i];
        DWORD size = section->Misc.VirtualSize;
        if (size < section->SizeOfRawData) size = section->SizeOfRawData;
        if (rva >= section->VirtualAddress &&
            rva - section->VirtualAddress < size)
            return section;
    }
    return NULL;
}











static int target_is_writable_data(const LiveImage *image, ULONGLONG target)
{
    DWORD rva;
    const IMAGE_SECTION_HEADER *section;
    if (target < image->base || target >= image->base + image->size) return 0;
    rva = (DWORD)(target - image->base);
    section = section_for_rva(image, rva);
    if (!section) return 0;
    return (section->Characteristics & IMAGE_SCN_MEM_WRITE) != 0 &&
           (section->Characteristics & IMAGE_SCN_MEM_EXECUTE) == 0;
}

static int find_runtime_function(const LiveImage *image, DWORD rva,
                                 DWORD *begin, DWORD *end)
{
    IMAGE_DATA_DIRECTORY dir;
    const RUNTIME_FUNCTION *table;
    DWORD count;
    DWORD lo;
    DWORD hi;

    dir = image->nt->OptionalHeader
        .DataDirectory[IMAGE_DIRECTORY_ENTRY_EXCEPTION];
    if (!dir.VirtualAddress || dir.Size < sizeof(RUNTIME_FUNCTION)) return 0;
    if (!all_valid(image->valid, dir.VirtualAddress, dir.Size, image->size))
        return 0;
    table = (const RUNTIME_FUNCTION *)(image->bytes + dir.VirtualAddress);
    count = dir.Size / sizeof(RUNTIME_FUNCTION);
    lo = 0;
    hi = count;

    while (lo < hi) {
        DWORD mid = lo + (hi - lo) / 2;
        if (rva < table[mid].BeginAddress) {
            hi = mid;
        } else if (rva >= table[mid].EndAddress) {
            lo = mid + 1;
        } else {
            *begin = table[mid].BeginAddress;
            *end = table[mid].EndAddress;
            return 1;
        }
    }
    return 0;
}

static int get_runtime_table(const LiveImage *image,
                             const RUNTIME_FUNCTION **table, DWORD *count)
{
    IMAGE_DATA_DIRECTORY dir;
    dir = image->nt->OptionalHeader
        .DataDirectory[IMAGE_DIRECTORY_ENTRY_EXCEPTION];
    if (!dir.VirtualAddress || dir.Size < sizeof(RUNTIME_FUNCTION) ||
        !all_valid(image->valid, dir.VirtualAddress, dir.Size, image->size))
        return 0;
    *table = (const RUNTIME_FUNCTION *)(image->bytes + dir.VirtualAddress);
    *count = dir.Size / sizeof(RUNTIME_FUNCTION);
    return *count != 0;
}

static int find_runtime_index(const LiveImage *image, DWORD rva, DWORD *index)
{
    const RUNTIME_FUNCTION *table;
    DWORD count;
    DWORD lo;
    DWORD hi;
    if (!get_runtime_table(image, &table, &count)) return 0;
    lo = 0;
    hi = count;
    while (lo < hi) {
        DWORD mid = lo + (hi - lo) / 2;
        if (rva < table[mid].BeginAddress) {
            hi = mid;
        } else if (rva >= table[mid].EndAddress) {
            lo = mid + 1;
        } else {
            *index = mid;
            return 1;
        }
    }
    return 0;
}

static int function_run_bounds(const LiveImage *image, DWORD rva,
                               DWORD *begin, DWORD *end)
{
    const RUNTIME_FUNCTION *table;
    DWORD count;
    DWORD index;
    DWORD low;
    DWORD high;

    if (!get_runtime_table(image, &table, &count) ||
        !find_runtime_index(image, rva, &index))
        return 0;
    low = index;
    high = index;
    while (low > 0 && table[low - 1].EndAddress == table[low].BeginAddress)
        low--;
    while (high + 1 < count &&
           table[high].EndAddress == table[high + 1].BeginAddress)
        high++;
    *begin = table[low].BeginAddress;
    *end = table[high].EndAddress;
    return 1;
}

/* Two RVAs belong to the same body: same contiguous run, and the second is not
   before the first. */











static int decode_rip_reference(const LiveImage *image, DWORD at, DWORD limit,
                                RipRef *out)
{
    DWORD p = at;
    DWORD modrm_at;
    DWORD disp_at;
    DWORD end;
    unsigned char opcode;
    unsigned char opcode2 = 0;
    unsigned char modrm;
    unsigned char prefix = 0;
    unsigned char rex = 0;
    int two_byte = 0;
    int immediate_bytes = 0;
    int width = 4;
    RefKind kind = REF_LOAD;
    LONG displacement;
    LONGLONG target;

    memset(out, 0, sizeof(*out));
    while (p < limit && p - at < 4) {
        unsigned char byte;
        if (!all_valid(image->valid, p, 1, image->size)) return 0;
        byte = image->bytes[p];
        if (byte == 0x66 || byte == 0xF2 || byte == 0xF3) {
            prefix = byte;
            p++;
            continue;
        }
        if (byte >= 0x40 && byte <= 0x4F) {
            rex = byte;
            p++;
            continue;
        }
        break;
    }
    if (p >= limit || !all_valid(image->valid, p, 1, image->size)) return 0;
    opcode = image->bytes[p++];
    if (opcode == 0x0F) {
        if (p >= limit || !all_valid(image->valid, p, 1, image->size))
            return 0;
        two_byte = 1;
        opcode2 = image->bytes[p++];
    }

    if (two_byte) {
        if (opcode2 == 0x10 || opcode2 == 0x6F) {
            kind = REF_LOAD;
        } else if (opcode2 == 0x11 || opcode2 == 0x7F) {
            kind = REF_STORE;
        } else if (opcode2 == 0xB6 || opcode2 == 0xBE ||
                   opcode2 == 0xB7 || opcode2 == 0xBF) {
            kind = REF_LOAD;
        } else {
            return 0;
        }
        if (opcode2 == 0xB6 || opcode2 == 0xBE) {
            width = 1;
        } else if (opcode2 == 0xB7 || opcode2 == 0xBF) {
            width = 2;
        } else if (prefix == 0xF2) {
            width = 8;
        } else if (prefix == 0xF3) {
            width = 4;
        } else {
            width = 16;
        }
    } else {
        switch (opcode) {
        case 0x8A: kind = REF_LOAD; width = 1; break;
        case 0x88: kind = REF_STORE; width = 1; break;
        case 0x8B: kind = REF_LOAD; width = (rex & 8) ? 8 : 4; break;
        case 0x89: kind = REF_STORE; width = (rex & 8) ? 8 : 4; break;
        case 0x8D: kind = REF_LEA; width = (rex & 8) ? 8 : 4; break;
        case 0xC6:
            kind = REF_STORE;
            width = 1;
            immediate_bytes = 1;
            break;
        case 0xC7:
            kind = REF_STORE;
            width = (rex & 8) ? 8 : 4;
            immediate_bytes = 4;
            break;
        case 0x80:
            kind = REF_READWRITE;
            width = 1;
            immediate_bytes = 1;
            break;
        case 0x81:
            kind = REF_READWRITE;
            width = (rex & 8) ? 8 : 4;
            immediate_bytes = 4;
            break;
        case 0x83:
            kind = REF_READWRITE;
            width = (rex & 8) ? 8 : 4;
            immediate_bytes = 1;
            break;
        case 0x84: kind = REF_LOAD; width = 1; break;
        case 0x85:
        case 0x39:
        case 0x3B:
            kind = REF_LOAD;
            width = (rex & 8) ? 8 : 4;
            break;
        default:
            return 0;
        }
    }

    modrm_at = p;
    if (modrm_at >= limit ||
        !all_valid(image->valid, modrm_at, 1, image->size))
        return 0;
    modrm = image->bytes[modrm_at];
    if ((modrm & 0xC7) != 0x05) return 0;
    if (!two_byte && (opcode == 0xC6 || opcode == 0xC7) &&
        ((modrm >> 3) & 7) != 0)
        return 0;
    if (!two_byte && (opcode == 0x80 || opcode == 0x81 ||
                      opcode == 0x83) &&
        ((modrm >> 3) & 7) == 7)
        kind = REF_LOAD;

    disp_at = modrm_at + 1;
    end = disp_at + 4 + (DWORD)immediate_bytes;
    if (end > limit || !all_valid(image->valid, disp_at,
                                  4 + immediate_bytes, image->size))
        return 0;
    memcpy(&displacement, image->bytes + disp_at, sizeof(displacement));
    target = (LONGLONG)image->base + (LONGLONG)end +
             (LONGLONG)displacement;
    if (target < (LONGLONG)image->base ||
        target >= (LONGLONG)(image->base + image->size))
        return 0;

    out->instruction = at;
    out->instruction_end = end;
    out->target = (ULONGLONG)target;
    out->kind = kind;
    out->width = width;
    if (immediate_bytes) {
        out->immediate_valid = 1;
        if (immediate_bytes == 1) {
            out->immediate = image->bytes[end - 1];
        } else {
            DWORD immediate;
            memcpy(&immediate, image->bytes + end - 4, sizeof(immediate));
            out->immediate = immediate;
        }
    }
    return 1;
}

/* A leaf getter - load a global, mask it, return - allocates no stack frame and
   calls nothing, so the x64 ABI lets the compiler emit it with no .pdata record
   at all. Demanding unwind bounds there rejects a correct anchor for a reason
   that has nothing to do with whether it is the right code. The exemption is
   therefore per-anchor (PatternDef.leaf_accessor) and it is not free: the bytes
   must still independently prove the accessor idiom, which is a second semantic
   constraint on the same match rather than a relaxation of the first.
     - the match must BEGIN a function: the preceding byte is a ret or int3 pad,
       so this cannot be a fragment spliced out of a larger routine;
     - the first instruction must be a REX.W RIP-relative LOAD of a global that
       lives in writable data - a pointer-width read of game state;
     - the second must be a register-to-register REX.W AND - the mask that makes
       this a status accessor and not an arbitrary load;
     - a ret must follow within 16 bytes of the entry, and int3 padding must
       follow the ret, which is what an isolated leaf function looks like.
   Anything else - including a leaf_accessor pattern that matched somewhere the
   idiom does not hold - still fails closed. */
static int leaf_accessor_bounds(const LiveImage *image, DWORD hit,
                                DWORD *begin, DWORD *end)
{
    RipRef load;
    DWORD limit = hit + 16;
    DWORD at;
    unsigned char opcode;

    if (!hit || limit > image->size) return 0;
    if (!all_valid(image->valid, hit - 1, 1, image->size)) return 0;
    if (image->bytes[hit - 1] != 0xC3 && image->bytes[hit - 1] != 0xCC)
        return 0;

    if (!decode_rip_reference(image, hit, limit, &load) ||
        load.kind != REF_LOAD || load.width != 8 ||
        !target_is_writable_data(image, load.target))
        return 0;

    at = load.instruction_end;
    if (at + 3 > limit || !all_valid(image->valid, at, 3, image->size))
        return 0;
    if (image->bytes[at] < 0x48 || image->bytes[at] > 0x4F ||
        !(image->bytes[at] & 8))
        return 0;
    opcode = image->bytes[at + 1];
    if (opcode != 0x21 && opcode != 0x23) return 0;
    if ((image->bytes[at + 2] & 0xC0) != 0xC0) return 0;
    at += 3;

    if (at >= limit || !all_valid(image->valid, at, 2, image->size) ||
        image->bytes[at] != 0xC3 || image->bytes[at + 1] != 0xCC)
        return 0;

    *begin = hit;
    *end = at + 1;
    return 1;
}

static void collect_function_refs(const LiveImage *image, DWORD begin,
                                  DWORD end, FunctionRefs *out)
{
    DWORD at;
    memset(out, 0, sizeof(*out));
    out->begin = begin;
    out->end = end;
    if (end > image->size) end = image->size;
    for (at = begin; at < end;) {
        RipRef ref;
        if (decode_rip_reference(image, at, end, &ref)) {
            if (out->count < MAX_FUNCTION_REFS)
                out->refs[out->count] = ref;
            out->count++;
            at = ref.instruction_end;
        } else {
            at++;
        }
    }
    if (out->count > MAX_FUNCTION_REFS) out->count = MAX_FUNCTION_REFS;
}

static int function_has_target(const FunctionRefs *refs, ULONGLONG target,
                               int require_store)
{
    size_t i;
    for (i = 0; i < refs->count; i++) {
        if (refs->refs[i].target == target &&
            (!require_store || refs->refs[i].kind == REF_STORE ||
             refs->refs[i].kind == REF_READWRITE))
            return 1;
    }
    return 0;
}






















static int direct_call_target(const LiveImage *image, DWORD at, DWORD limit,
                              DWORD *target)
{
    LONG displacement;
    LONGLONG resolved;
    if (at > limit || limit - at < 5 ||
        !all_valid(image->valid, at, 5, image->size) ||
        image->bytes[at] != 0xE8)
        return 0;
    memcpy(&displacement, image->bytes + at + 1, sizeof(displacement));
    resolved = (LONGLONG)at + 5 + (LONGLONG)displacement;
    if (resolved < 0 || resolved >= image->size) return 0;
    *target = (DWORD)resolved;
    return section_for_rva(image, *target) != NULL;
}









































static void scan_pattern(const LiveImage *image, const PatternDef *definition,
                         PatternResult *result)
{
    ParsedPattern pattern;
    WORD i;
    memset(result, 0, sizeof(*result));
    if (!parse_pattern(definition->text, &pattern)) return;

    for (i = 0; i < image->section_count; i++) {
        const IMAGE_SECTION_HEADER *section = &image->sections[i];
        DWORD start;
        DWORD size;
        DWORD end;
        DWORD offset;

        if (!(section->Characteristics & IMAGE_SCN_MEM_EXECUTE)) continue;
        start = section->VirtualAddress;
        size = section->Misc.VirtualSize;
        if (size < section->SizeOfRawData) size = section->SizeOfRawData;
        if (start >= image->size) continue;
        if (size > image->size - start) size = image->size - start;
        end = start + size;
        if (pattern.length > size) continue;

        for (offset = start;
             offset <= end - (DWORD)pattern.length;
             offset++) {
            if (pattern_matches(image->bytes, image->valid, image->size,
                                offset, &pattern)) {
                if (result->count < MAX_HITS_SAVED)
                    result->hits[result->count] = offset;
                result->count++;
            }
        }
    }

    if (result->count == 1) {
        int ref;
        int targets_ok = 1;
        DWORD hit = result->hits[0];
        result->function_found = find_runtime_function(
            image, hit, &result->function_begin, &result->function_end);
        if (!result->function_found && definition->leaf_accessor)
            result->leaf_function = leaf_accessor_bounds(
                image, hit, &result->function_begin, &result->function_end);
        if (!result->function_found && !result->leaf_function)
            targets_ok = 0;

        for (ref = 0; ref < definition->ref_count; ref++) {
            int resolved = 0;
            result->targets[ref] = resolve_rip(
                image, hit, definition->disp_offset[ref],
                definition->instruction_end[ref], &resolved);
            if (!resolved) {
                targets_ok = 0;
            } else if (definition->require_writable_target &&
                       !target_is_writable_data(image, result->targets[ref])) {
                targets_ok = 0;
            }
        }
        result->ok = targets_ok;
    }
}

#define MAX_CLUSTER_SLOTS 64
#define MAX_STORE_VALUES 8

#define PAD_SLOT_SUBJECT 1
#define PAD_SLOT_STOP_AIM 2
#define PAD_SLOT_WEAPON 4
#define PAD_SLOT_PRESS_WEAPON 5
#define PAD_SLOT_CAPUTRE 13
#define PAD_SLOT_HANG 16
#define PAD_CLUSTER_MIN_SLOTS 20

static size_t collect_store_immediates(const FunctionRefs *function,
                                       ULONGLONG target, ULONGLONG *values,
                                       size_t limit)
{
    size_t i;
    size_t count = 0;
    for (i = 0; i < function->count; i++) {
        const RipRef *ref = &function->refs[i];
        if (ref->kind != REF_STORE || ref->target != target) continue;
        if (!ref->immediate_valid) return (size_t)-1;
        if (count >= limit) return (size_t)-1;
        values[count++] = ref->immediate;
    }
    return count;
}

static void sort_values(ULONGLONG *values, size_t count)
{
    size_t i;
    for (i = 1; i < count; i++) {
        ULONGLONG key = values[i];
        size_t j = i;
        while (j && values[j - 1] > key) {
            values[j] = values[j - 1];
            j--;
        }
        values[j] = key;
    }
}

static int store_values_match(const FunctionRefs *function, ULONGLONG a,
                              ULONGLONG b)
{
    ULONGLONG left[MAX_STORE_VALUES];
    ULONGLONG right[MAX_STORE_VALUES];
    size_t left_count = collect_store_immediates(function, a, left,
                                                 MAX_STORE_VALUES);
    size_t right_count = collect_store_immediates(function, b, right,
                                                  MAX_STORE_VALUES);
    size_t i;
    if (left_count == (size_t)-1 || right_count == (size_t)-1) return 0;
    if (!left_count || left_count != right_count) return 0;
    sort_values(left, left_count);
    sort_values(right, right_count);
    for (i = 0; i < left_count; i++) {
        if (left[i] != right[i]) return 0;
    }
    return 1;
}

static int function_stores_target(const FunctionRefs *function,
                                  ULONGLONG target)
{
    size_t i;
    for (i = 0; i < function->count; i++) {
        if (function->refs[i].target == target &&
            function->refs[i].kind == REF_STORE)
            return 1;
    }
    return 0;
}

static int function_loads_target(const FunctionRefs *function,
                                 ULONGLONG target)
{
    size_t i;
    for (i = 0; i < function->count; i++) {
        if (function->refs[i].target == target &&
            function->refs[i].kind != REF_STORE)
            return 1;
    }
    return 0;
}

static size_t longest_dword_cluster(const LiveImage *image,
                                    const FunctionRefs *function,
                                    ULONGLONG *base)
{
    ULONGLONG targets[MAX_FUNCTION_REFS];
    size_t count = 0;
    size_t i;
    size_t best = 0;
    size_t run;
    for (i = 0; i < function->count; i++) {
        const RipRef *ref = &function->refs[i];
        size_t j;
        if (ref->kind != REF_STORE || ref->width != 4 ||
            !target_is_writable_data(image, ref->target))
            continue;
        for (j = 0; j < count; j++) {
            if (targets[j] == ref->target) break;
        }
        if (j == count && count < MAX_FUNCTION_REFS)
            targets[count++] = ref->target;
    }
    sort_values(targets, count);
    for (i = 0; i < count; i = i + run) {
        size_t k = i;
        while (k + 1 < count && targets[k + 1] == targets[k] + 4) k++;
        run = k - i + 1;
        if (run > best) {
            best = run;
            *base = targets[i];
        }
        if (!run) run = 1;
    }
    return best;
}

static void find_pad_masks(const LiveImage *image,
                           const PatternResult patterns[PATTERN_COUNT],
                           PadMaskResult *result)
{
    const RUNTIME_FUNCTION *table;
    DWORD count;
    DWORD i;
    FunctionRefs candidates[MAX_CLUSTER_FUNCTIONS];
    size_t candidate_count = 0;
    FunctionRefs fps_refs;
    DWORD fps_begin;
    DWORD fps_end;
    ULONGLONG subject_toggle;
    ULONGLONG subject_move;
    ULONGLONG base = 0;
    ULONGLONG probe = 0;
    size_t slots;
    size_t dispatcher;
    size_t setter;
    size_t missing = 0;
    size_t missing_count = 0;
    size_t s;

    memset(result, 0, sizeof(*result));
    if (!patterns[0].ok || !patterns[3].ok) return;
    subject_toggle = patterns[0].targets[1];
    subject_move = patterns[3].targets[1];
    if (!target_is_writable_data(image, subject_toggle) ||
        !target_is_writable_data(image, subject_move) ||
        !get_runtime_table(image, &table, &count) ||
        !function_run_bounds(image, patterns[0].hits[0], &fps_begin, &fps_end))
        return;
    collect_function_refs(image, fps_begin, fps_end, &fps_refs);

    for (i = 0; i < count; i++) {
        FunctionRefs refs;
        DWORD begin;
        DWORD end;
        if (!function_run_bounds(image, table[i].BeginAddress, &begin, &end) ||
            begin != table[i].BeginAddress)
            continue;
        collect_function_refs(image, begin, end, &refs);
        /* Writing PL_SubjectToggle is not on its own rare enough - Action()'s
           run does it too. What only a pattern setter does is write one long
           stride-4 block of int globals in a single body. */
        if (function_has_target(&refs, subject_toggle, 1) &&
            longest_dword_cluster(image, &refs, &probe) >=
                PAD_CLUSTER_MIN_SLOTS) {
            if (candidate_count < MAX_CLUSTER_FUNCTIONS)
                candidates[candidate_count] = refs;
            candidate_count++;
        }
    }
    if (candidate_count != 2) return;

    /* Both candidates read PL_SubjectMove - retail inlined PL_PadSetPatternA
       into PL_SetPadType(), whose first act is the `if (PL_SubjectMove) return`
       early-out, and the merged SubjectMove body switches on it. So that read
       ties both bodies to the FPS anchors but cannot tell them apart.
       What does: only the SubjectMove body assigns PL_PAD_STOP_AIM, so only its
       cluster is unbroken. PL_PadSetPatternA skips that one slot, which splits
       its longest run short and strictly inside the other's. */
    if (!function_loads_target(&candidates[0], subject_move) ||
        !function_loads_target(&candidates[1], subject_move))
        return;
    {
        ULONGLONG base0 = 0;
        ULONGLONG base1 = 0;
        size_t slots0 = longest_dword_cluster(image, &candidates[0], &base0);
        size_t slots1 = longest_dword_cluster(image, &candidates[1], &base1);
        ULONGLONG inner_base;
        size_t inner_slots;
        if (slots0 == slots1) return;
        dispatcher = slots0 > slots1 ? 0 : 1;
        setter = dispatcher ? 0 : 1;
        base = dispatcher == 0 ? base0 : base1;
        slots = dispatcher == 0 ? slots0 : slots1;
        inner_base = dispatcher == 0 ? base1 : base0;
        inner_slots = dispatcher == 0 ? slots1 : slots0;
        if (inner_base < base ||
            inner_base + (ULONGLONG)inner_slots * 4 >
                base + (ULONGLONG)slots * 4)
            return;
    }
    if (slots < PAD_CLUSTER_MIN_SLOTS || slots > MAX_CLUSTER_SLOTS ||
        slots <= PAD_SLOT_HANG)
        return;

    for (s = 0; s < slots; s++) {
        if (!function_stores_target(&candidates[setter],
                                    base + (ULONGLONG)s * 4)) {
            missing = s;
            missing_count++;
        }
    }
    if (missing_count != 1 || missing != PAD_SLOT_STOP_AIM) return;

    if (!function_loads_target(&fps_refs,
                               base + PAD_SLOT_SUBJECT * 4))
        return;

    for (i = 0; i < 2; i++) {
        if (!store_values_match(&candidates[i],
                                base + PAD_SLOT_WEAPON * 4,
                                base + PAD_SLOT_CAPUTRE * 4) ||
            !store_values_match(&candidates[i],
                                base + PAD_SLOT_WEAPON * 4,
                                base + PAD_SLOT_HANG * 4))
            return;
    }

    result->pad_subject = base + PAD_SLOT_SUBJECT * 4;
    result->pad_stop_aim = base + PAD_SLOT_STOP_AIM * 4;
    result->pad_weapon = base + PAD_SLOT_WEAPON * 4;
    result->pad_press_weapon = base + PAD_SLOT_PRESS_WEAPON * 4;
    result->subject_toggle = subject_toggle;
    result->subject_move = subject_move;
    result->cluster_base = (DWORD)(base - image->base);
    result->cluster_slots = (DWORD)slots;
    result->pattern_function = candidates[setter].begin;
    result->dispatcher_function = candidates[dispatcher].begin;

    if (result->pad_subject == subject_toggle ||
        result->pad_subject == subject_move ||
        result->pad_stop_aim == subject_toggle ||
        result->pad_stop_aim == subject_move ||
        !target_is_writable_data(image, result->pad_subject) ||
        !target_is_writable_data(image, result->pad_stop_aim) ||
        !target_is_writable_data(image, result->pad_weapon) ||
        !target_is_writable_data(image, result->pad_press_weapon))
        return;
    result->ok = 1;
}

static int ref_is_copy_access(const RipRef *ref, RefKind kind, int width)
{
    return ref->kind == kind && ref->width == width;
}

/* The destination half of the copy: three RIP-relative stores covering +0, +16
   and +32 of one base, i.e. exactly 40 bytes - sizeof(GV_PAD). */
static int find_40byte_span(const FunctionRefs *function, RefKind kind,
                            ULONGLONG *base, DWORD *first, DWORD *last)
{
    size_t i;
    int matches = 0;
    for (i = 0; i < function->count; i++) {
        const RipRef *a = &function->refs[i];
        size_t j;
        const RipRef *b = NULL;
        const RipRef *c = NULL;
        DWORD min_instruction;
        DWORD max_instruction;
        if (!ref_is_copy_access(a, kind, 16)) continue;
        for (j = 0; j < function->count; j++) {
            const RipRef *candidate = &function->refs[j];
            if (ref_is_copy_access(candidate, kind, 16) &&
                candidate->target == a->target + 16)
                b = candidate;
            if (ref_is_copy_access(candidate, kind, 8) &&
                candidate->target == a->target + 32)
                c = candidate;
        }
        if (!b || !c) continue;
        min_instruction = a->instruction;
        if (b->instruction < min_instruction) min_instruction = b->instruction;
        if (c->instruction < min_instruction) min_instruction = c->instruction;
        max_instruction = a->instruction;
        if (b->instruction > max_instruction) max_instruction = b->instruction;
        if (c->instruction > max_instruction) max_instruction = c->instruction;
        if (max_instruction - min_instruction > 96) continue;
        *base = a->target;
        *first = min_instruction;
        *last = max_instruction;
        matches++;
    }
    return matches == 1;
}

static int has_non_rip_qword_store(const LiveImage *image, DWORD begin,
                                   DWORD end)
{
    DWORD at;
    if (end > image->size) end = image->size;
    for (at = begin; at + 3 <= end; at++) {
        unsigned char modrm;
        if (!all_valid(image->valid, at, 3, image->size) ||
            image->bytes[at] < 0x48 || image->bytes[at] > 0x4F ||
            !(image->bytes[at] & 8) || image->bytes[at + 1] != 0x89)
            continue;
        modrm = image->bytes[at + 2];
        if ((modrm & 0xC0) != 0xC0 && (modrm & 0xC7) != 0x05)
            return 1;
    }
    return 0;
}

static int decode_scaled_load_base(const LiveImage *image, DWORD at,
                                   DWORD limit, DWORD *disp,
                                   DWORD *instruction_end, int *width)
{
    DWORD p = at;
    unsigned char modrm;
    unsigned char sib;
    unsigned char prefix = 0;
    LONG displacement;

    if (p < limit && all_valid(image->valid, p, 1, image->size) &&
        (image->bytes[p] == 0xF2 || image->bytes[p] == 0xF3 ||
         image->bytes[p] == 0x66)) {
        prefix = image->bytes[p];
        p++;
    }
    if (p < limit && all_valid(image->valid, p, 1, image->size) &&
        image->bytes[p] >= 0x40 && image->bytes[p] <= 0x4F)
        p++;
    if (p + 2 > limit || !all_valid(image->valid, p, 2, image->size) ||
        image->bytes[p] != 0x0F || image->bytes[p + 1] != 0x10)
        return 0;
    p += 2;
    if (p + 6 > limit || !all_valid(image->valid, p, 6, image->size))
        return 0;
    modrm = image->bytes[p];
    /* mod=10 (disp32), rm=100 (SIB follows) */
    if ((modrm & 0xC7) != 0x84) return 0;
    sib = image->bytes[p + 1];
    /* scale=11 (x8); index and base must both be real registers */
    if ((sib >> 6) != 3 || ((sib >> 3) & 7) == 4) return 0;
    memcpy(&displacement, image->bytes + p + 2, sizeof(displacement));
    if (displacement <= 0) return 0;
    *disp = (DWORD)displacement;
    *instruction_end = p + 6;
    *width = prefix == 0xF2 ? 8 : (prefix == 0xF3 ? 4 : 16);
    return 1;
}

/* Three scaled loads at +0, +16, +32 of one disp32 base, all with the same
   index scale - the 40-byte GV_PAD record the copy reads. */
static int find_scaled_40byte_source(const LiveImage *image, DWORD begin,
                                     DWORD end, ULONGLONG *source)
{
    DWORD at;
    int matches = 0;
    if (end > image->size) end = image->size;
    for (at = begin; at < end; at++) {
        DWORD disp;
        DWORD instruction_end;
        int width;
        DWORD scan;
        int have_mid = 0;
        int have_tail = 0;
        if (!decode_scaled_load_base(image, at, end, &disp, &instruction_end,
                                     &width) ||
            width != 16)
            continue;
        for (scan = begin; scan < end; scan++) {
            DWORD other;
            DWORD other_end;
            int other_width;
            if (!decode_scaled_load_base(image, scan, end, &other, &other_end,
                                         &other_width))
                continue;
            if (other == disp + 16 && other_width == 16) have_mid = 1;
            if (other == disp + 32 && other_width == 8) have_tail = 1;
        }
        if (!have_mid || !have_tail) continue;
        if (!target_is_writable_data(image, image->base + disp)) continue;
        *source = image->base + disp;
        matches++;
    }
    return matches == 1;
}

static int find_merge_lea(const LiveImage *image,
                          const FunctionRefs *function,
                          ULONGLONG player_pad, DWORD after,
                          DWORD *seam)
{
    size_t i;
    int matches = 0;
    for (i = 0; i < function->count; i++) {
        const RipRef *ref = &function->refs[i];
        if (ref->kind == REF_LEA && ref->target == player_pad &&
            ref->instruction > after &&
            has_non_rip_qword_store(image, ref->instruction_end,
                                    ref->instruction_end + 24)) {
            *seam = ref->instruction_end;
            matches++;
        }
    }
    return matches == 1;
}

static size_t count_ref_functions(const LiveImage *image, ULONGLONG target,
                                  RefKind kind)
{
    const RUNTIME_FUNCTION *table;
    DWORD count;
    DWORD i;
    size_t functions = 0;
    if (!get_runtime_table(image, &table, &count)) return 0;
    for (i = 0; i < count; i++) {
        FunctionRefs refs;
        size_t j;
        collect_function_refs(image, table[i].BeginAddress,
                              table[i].EndAddress, &refs);
        for (j = 0; j < refs.count; j++) {
            if (refs.refs[j].target == target &&
                refs.refs[j].kind == kind) {
                functions++;
                break;
            }
        }
    }
    return functions;
}

static void find_player_pad(const LiveImage *image,
                            const PadMaskResult *masks,
                            PlayerPadResult *result)
{
    const RUNTIME_FUNCTION *table;
    DWORD count;
    DWORD i;
    size_t matches = 0;
    memset(result, 0, sizeof(*result));
    if (!masks->ok || !get_runtime_table(image, &table, &count)) return;

    for (i = 0; i < count; i++) {
        FunctionRefs copy_refs;
        FunctionRefs search_refs;
        ULONGLONG player_pad;
        ULONGLONG gv_pad_data;
        DWORD copy_begin = table[i].BeginAddress;
        DWORD copy_end = table[i].EndAddress;
        DWORD run_begin;
        DWORD run_end;
        DWORD seam_begin;
        DWORD seam_end;
        DWORD first;
        DWORD last;
        DWORD seam;

        collect_function_refs(image, copy_begin, copy_end, &copy_refs);
        if (!find_40byte_span(&copy_refs, REF_STORE, &player_pad, &first,
                              &last))
            continue;
        if (!function_run_bounds(image, copy_begin, &run_begin, &run_end))
            continue;
        collect_function_refs(image, run_begin, run_end, &search_refs);
        if (!function_has_target(&search_refs, masks->pad_weapon, 0) ||
            !find_merge_lea(image, &search_refs, player_pad, last, &seam) ||
            !find_scaled_40byte_source(image, run_begin, run_end,
                                       &gv_pad_data) ||
            gv_pad_data == player_pad ||
            count_ref_functions(image, gv_pad_data, REF_LEA) < 2)
            continue;
        /* The seam is where F2 detours, so its unwind record must be this
           entry or the one immediately after it with no gap - never further,
           even though the run continues past both. */
        if (seam >= copy_begin && seam < copy_end) {
            seam_begin = copy_begin;
            seam_end = copy_end;
        } else if (i + 1 < count && table[i + 1].BeginAddress == copy_end &&
                   seam >= copy_end && seam < table[i + 1].EndAddress) {
            seam_begin = table[i + 1].BeginAddress;
            seam_end = table[i + 1].EndAddress;
        } else {
            continue;
        }
        result->function_begin = seam_begin;
        result->function_end = seam_end;
        result->copy_begin = first;
        result->merge_seam = seam;
        {

            DWORD last_end = 0;
            size_t k;
            for (k = 0; k < copy_refs.count; k++)
                if (copy_refs.refs[k].instruction == last)
                    last_end = copy_refs.refs[k].instruction_end;
            if (last_end && last_end + 6 <= copy_end &&
                all_valid(image->valid, last_end, 6, image->size) &&
                image->bytes[last_end] == 0x0F && image->bytes[last_end + 1] == 0xBA &&
                image->bytes[last_end + 2] == 0xE0 && image->bytes[last_end + 4] == 0x73) {
                DWORD cs = (DWORD)((LONG)(last_end + 6) +
                                   (LONG)(signed char)image->bytes[last_end + 5]);
                if (cs > last_end && cs < copy_end) {
                    result->copy_seam = cs;
                    result->copy_function_begin = copy_begin;
                    result->copy_function_end = copy_end;
                }
            }
        }
        result->player_pad = player_pad;
        result->gv_pad_data = gv_pad_data;
        matches++;
    }
    result->ok = matches == 1;
}

static int modrm_memory_end(const LiveImage *image, DWORD modrm_at,
                            DWORD limit, DWORD *end)
{
    unsigned char modrm;
    unsigned char mod;
    unsigned char rm;
    DWORD p;
    if (modrm_at >= limit ||
        !all_valid(image->valid, modrm_at, 1, image->size))
        return 0;
    modrm = image->bytes[modrm_at];
    mod = modrm >> 6;
    rm = modrm & 7;
    if (mod == 3) return 0;
    p = modrm_at + 1;
    if (rm == 4) {
        unsigned char sib;
        if (p >= limit || !all_valid(image->valid, p, 1, image->size))
            return 0;
        sib = image->bytes[p++];
        if (mod == 0 && (sib & 7) == 5) p += 4;
    } else if (mod == 0 && rm == 5) {
        p += 4;
    }
    if (mod == 1) p++;
    if (mod == 2) p += 4;
    if (p > limit || !all_valid(image->valid, modrm_at, p - modrm_at,
                                image->size))
        return 0;
    *end = p;
    return 1;
}













































































































































/* ------------------------------------------------------- GV_PadPress[] --- */

/* `test <r/m>, imm` - F6 /0 ib on a byte operand, F7 /0 id on a dword one.
   The pad update tests its flag word both ways in the same function
   (`test r10d, 0x103` for GV_PAD_RELEASE, `test r10b, 0x20` for
   GV_PAD_PRESS_SCN), so a decoder that only knew one of them would read half
   the branch structure and call it the whole. Register and memory operands
   are both accepted; the immediate comes back zero-extended. */
static int decode_test_immediate(const LiveImage *image, DWORD at, DWORD limit,
                                 ULONGLONG *immediate, DWORD *end)
{
    DWORD p = at;
    DWORD operand_end;
    unsigned char opcode;
    unsigned char modrm;
    int immediate_bytes;

    if (p < limit && all_valid(image->valid, p, 1, image->size) &&
        image->bytes[p] == 0x66)
        p++;
    if (p < limit && all_valid(image->valid, p, 1, image->size) &&
        image->bytes[p] >= 0x40 && image->bytes[p] <= 0x4F)
        p++;
    if (p + 2 > limit || !all_valid(image->valid, p, 2, image->size))
        return 0;
    opcode = image->bytes[p];
    if (opcode != 0xF6 && opcode != 0xF7) return 0;
    modrm = image->bytes[p + 1];
    /* /0 is TEST. /2 is NOT, /3 NEG, /4 MUL and so on - none of them carries
       an immediate at all, so reading one would be reading the next
       instruction's bytes. */
    if (((modrm >> 3) & 7) != 0) return 0;
    if ((modrm >> 6) == 3) {
        operand_end = p + 2;
    } else if (!modrm_memory_end(image, p + 1, limit, &operand_end)) {
        return 0;
    }
    immediate_bytes = (opcode == 0xF6) ? 1 : 4;
    if (operand_end + (DWORD)immediate_bytes > limit ||
        !all_valid(image->valid, operand_end, immediate_bytes, image->size))
        return 0;
    if (immediate_bytes == 1) {
        *immediate = image->bytes[operand_end];
    } else {
        DWORD value;
        memcpy(&value, image->bytes + operand_end, sizeof(value));
        *immediate = value;
    }
    *end = operand_end + (DWORD)immediate_bytes;
    return 1;
}

static int decode_rip_dword_read(const LiveImage *image, DWORD at, DWORD limit,
                                 ULONGLONG *target, DWORD *end)
{
    DWORD p = at;
    DWORD instruction_end;
    LONG displacement;
    LONGLONG resolved;

    if (p < limit && all_valid(image->valid, p, 1, image->size) &&
        image->bytes[p] >= 0x40 && image->bytes[p] <= 0x4F)
        p++;
    if (p + 2 > limit || !all_valid(image->valid, p, 2, image->size))
        return 0;
    if (image->bytes[p] != 0x8B && image->bytes[p] != 0x0B) return 0;
    if ((image->bytes[p + 1] & 0xC7) != 0x05) return 0;
    instruction_end = p + 6;
    if (instruction_end > limit ||
        !all_valid(image->valid, p + 2, 4, image->size))
        return 0;
    memcpy(&displacement, image->bytes + p + 2, sizeof(displacement));
    resolved = (LONGLONG)image->base + (LONGLONG)instruction_end +
               (LONGLONG)displacement;
    if (resolved < (LONGLONG)image->base ||
        resolved >= (LONGLONG)(image->base + image->size))
        return 0;
    *target = (ULONGLONG)resolved;
    *end = instruction_end;
    return 1;
}

/* `and r32, imm8` with the /4 reg field on a REGISTER operand - 83 /4 ib.
   0xDF sign-extends to 0xFFFFFFDF, which is ~GV_PAD_PRESS_SCN, so this is how
   `pad->flag &= ~GV_PAD_PRESS_SCN` looks once the flag is already in a
   register. Register-only on purpose: an AND against a memory operand is a
   different statement shape and not the one being identified. */
static int decode_and_register_imm8(const LiveImage *image, DWORD at,
                                    DWORD limit, ULONGLONG *immediate,
                                    DWORD *end)
{
    DWORD p = at;
    unsigned char modrm;

    if (p < limit && all_valid(image->valid, p, 1, image->size) &&
        image->bytes[p] >= 0x40 && image->bytes[p] <= 0x4F)
        p++;
    if (p + 3 > limit || !all_valid(image->valid, p, 3, image->size))
        return 0;
    if (image->bytes[p] != 0x83) return 0;
    modrm = image->bytes[p + 1];
    if ((modrm >> 6) != 3 || ((modrm >> 3) & 7) != 4) return 0;
    *immediate = image->bytes[p + 2];
    *end = p + 3;
    return 1;
}

static int function_has_lea(const FunctionRefs *refs, ULONGLONG target)
{
    size_t i;
    for (i = 0; i < refs->count; i++) {
        if (refs->refs[i].target == target && refs->refs[i].kind == REF_LEA)
            return 1;
    }
    return 0;
}

/* The LAST LEA of `target` in a body, and the first direct call that starts
   within `window` bytes after it. Used to name the instruction boundary just
   past `UpdatePad( &GV_PadDataDirect[0], up )` - the one point in the frame
   where that copy holds this frame's buttons and no menu actor has read them
   yet. Reported, never required. */
static int lea_then_call_seam(const LiveImage *image, DWORD begin, DWORD end,
                              ULONGLONG target, DWORD window, DWORD *seam)
{
    FunctionRefs refs;
    DWORD last_lea_end = 0;
    DWORD at;
    size_t i;
    collect_function_refs(image, begin, end, &refs);
    for (i = 0; i < refs.count; i++) {
        if (refs.refs[i].target == target && refs.refs[i].kind == REF_LEA)
            last_lea_end = refs.refs[i].instruction_end;
    }
    if (!last_lea_end) return 0;
    for (at = last_lea_end; at < end && at < last_lea_end + window; at++) {
        DWORD call_target;
        if (!direct_call_target(image, at, end, &call_target)) continue;
        *seam = at + 5;
        return 1;
    }
    return 0;
}

static void find_pad_press(const LiveImage *image,
                           const PlayerPadResult *player_pad,
                           PadPressResult *result)
{
    const RUNTIME_FUNCTION *table;
    DWORD count;
    DWORD i;
    size_t matches = 0;
    ULONGLONG flag_word;
    memset(result, 0, sizeof(*result));
    if (!player_pad || !player_pad->ok || !player_pad->gv_pad_data) return;
    if (!get_runtime_table(image, &table, &count)) return;
    flag_word = player_pad->gv_pad_data + DG_GV_PAD_FLAG_OFFSET;

    for (i = 0; i < count; i++) {
        FunctionRefs refs;
        DWORD begin = table[i].BeginAddress;
        DWORD end = table[i].EndAddress;
        DWORD at;
        DWORD press_read = 0;
        DWORD press_clear = 0;
        DWORD direct_seam = 0;
        ULONGLONG press = 0;
        ULONGLONG direct = player_pad->gv_pad_data +
                           DG_GV_PAD_STRIDE * DG_GV_PAD_COUNT;
        size_t j;
        size_t guarded = 0;
        int flag_loaded = 0;
        int flag_stored = 0;
        int disagreed = 0;

        if (end <= begin) continue;
        collect_function_refs(image, begin, end, &refs);
        for (j = 0; j < refs.count; j++) {
            if (refs.refs[j].target != flag_word) continue;
            if (refs.refs[j].kind == REF_LOAD ||
                refs.refs[j].kind == REF_READWRITE)
                flag_loaded = 1;
            if (refs.refs[j].kind == REF_STORE ||
                refs.refs[j].kind == REF_READWRITE)
                flag_stored = 1;
        }
        if (!flag_loaded || !flag_stored) continue;

        for (at = begin; at < end; at++) {
            ULONGLONG immediate;
            DWORD instruction_end;
            DWORD and_at = at;
            if (!decode_and_register_imm8(image, at, end, &immediate,
                                          &instruction_end))
                continue;
            /* Step past the whole instruction - the optional REX makes the
               same bytes decode again one in - but report where it STARTS. */
            at = instruction_end - 1;
            if (immediate != 0xDF) continue;
            press_clear = and_at;
        }
        if (!press_clear) continue;

        for (at = begin; at < end; at++) {
            ULONGLONG immediate;
            ULONGLONG target;
            DWORD test_end;
            DWORD read_at;
            DWORD read_end;
            DWORD test_at = at;
            if (!decode_test_immediate(image, at, end, &immediate, &test_end))
                continue;
            at = test_end - 1;
            if (immediate != 0x20) continue;
            /* The guarded read, past whatever conditional jump the compiler
               put between them. Eight bytes covers a short jz and nothing
               large enough to be a different statement. */
            for (read_at = test_end;
                 read_at < end && read_at < test_end + 8; read_at++) {
                if (decode_rip_dword_read(image, read_at, end, &target,
                                          &read_end))
                    break;
            }
            if (read_at >= end || read_at >= test_end + 8) continue;
            if (!press) {
                press = target;
                press_read = read_at;
            } else if (press != target) {
                disagreed = 1;
            }
            guarded++;
            (void)test_at;
        }
        if (disagreed || guarded < 2 || !press) continue;
        if (press == player_pad->gv_pad_data || press == flag_word ||
            !target_is_writable_data(image, press))
            continue;

        if (!function_has_lea(&refs, direct)) direct = 0;
        if (direct)
            lea_then_call_seam(image, begin, end, direct, 24, &direct_seam);

        result->function_begin = begin;
        result->function_end = end;
        result->press_read = press_read;
        result->press_clear = press_clear;
        result->direct_seam = direct_seam;
        result->gv_pad_press = press;
        result->gv_pad_data_direct = direct;
        matches++;
    }
    result->ok = matches == 1;
}























































































/* Does any function assign this exact byte global the immediate `value`? For
   the private WeaponFire that is `WeaponFire = -1`, which is what separates it
   from the image's other leaf byte setters - they write globals nothing ever
   resets to -1. */























/* GM_SetWeaponFire: mov byte ptr [rip+disp], reg8 ; ret. A leaf with no .pdata
   record of its own, so it is found by shape. The private global it writes must
   also take an immediate -1 byte store elsewhere in the image - the
   `WeaponFire = -1` in the publisher - which is what makes the match unique
   among the image's leaf byte setters. */







































































































































































































/* ------------------------------------------- base FPS cross-anchor gate --- */

/* The four FPS anchors only mean what their names say if they also relate to
   each other correctly: Active must be reachable from two independent code
   sites, and no two of the six globals may be the same address. print = 0 makes
   this usable from the in-process bridge, which has no console. */
static int check_relationships(const PatternResult results[PATTERN_COUNT],
                               int print)
{
    int ok = 1;
    if (!results[0].ok || !results[1].ok ||
        !results[2].ok || !results[3].ok) {
        if (print)
            printf("  FAIL relation: prerequisite FPS anchors did not all pass\n");
        return 0;
    }
    if (results[0].targets[0] == results[0].targets[1]) {
        if (print)
            printf("  FAIL relation: gBP Toggle aliases PL_SubjectToggle\n");
        ok = 0;
    } else if (print) {
        printf("  PASS relation: gBP Toggle -> distinct PL_SubjectToggle\n");
    }
    if (results[3].targets[0] == results[3].targets[1]) {
        if (print)
            printf("  FAIL relation: gBP Move aliases PL_SubjectMove\n");
        ok = 0;
    } else if (print) {
        printf("  PASS relation: gBP Move -> distinct PL_SubjectMove\n");
    }
    if (results[1].targets[1] != results[2].targets[0]) {
        if (print)
            printf("  FAIL relation: Override companion Active != Active branch\n");
        ok = 0;
    } else if (print) {
        printf("  PASS relation: Active has two matching code anchors\n");
    }
    if (results[0].targets[0] == results[1].targets[0] ||
        results[0].targets[0] == results[1].targets[1] ||
        results[0].targets[0] == results[3].targets[0] ||
        results[1].targets[0] == results[1].targets[1] ||
        results[1].targets[0] == results[3].targets[0] ||
        results[1].targets[1] == results[3].targets[0]) {
        if (print)
            printf("  FAIL relation: distinct FPS globals alias each other\n");
        ok = 0;
    } else if (print) {
        printf("  PASS relation: Toggle, Override, Active and Move are "
               "distinct\n");
    }
    if (results[0].targets[1] == results[3].targets[1] ||
        results[0].targets[1] == results[0].targets[0] ||
        results[3].targets[1] == results[3].targets[0]) {
        if (print)
            printf("  FAIL relation: native subject state targets alias\n");
        ok = 0;
    } else if (print) {
        printf("  PASS relation: PL_SubjectToggle and PL_SubjectMove are "
               "distinct targets\n");
    }

    if (!results[8].ok) {
        if (print) printf("  FAIL relation: game-status anchor did not pass\n");
        return 0;
    }
    if (results[8].targets[1] + 4 != results[8].targets[0]) {
        if (print)
            printf("  FAIL relation: game-status halves are not adjacent "
                   "(0x%llX, 0x%llX)\n",
                   results[8].targets[1], results[8].targets[0]);
        ok = 0;
    } else if (print) {
        printf("  PASS relation: game-status pair is two adjacent ints\n");
    }
    {   /* And it must be a pair nobody else already owns. */
        int j;
        ULONGLONG others[6];
        others[0] = results[0].targets[0]; others[1] = results[0].targets[1];
        others[2] = results[1].targets[0]; others[3] = results[1].targets[1];
        others[4] = results[3].targets[0]; others[5] = results[3].targets[1];
        for (j = 0; j < 6; j++) {
            if (others[j] == results[8].targets[0] ||
                others[j] == results[8].targets[1]) {
                if (print)
                    printf("  FAIL relation: game-status aliases an FPS "
                           "global\n");
                ok = 0;
                break;
            }
        }
        if (print && j == 6)
            printf("  PASS relation: game-status is distinct from the FPS "
                   "globals\n");
    }
    /* The menu-status pair, held to the same two constraints - adjacent, and
       the `or` operand the lower of the two. The two pairs must also be four
       distinct addresses: they sit in the same run of globals four bytes apart,
       which is exactly the arrangement a mis-resolved displacement would land
       in without anything else looking wrong. */
    if (!results[9].ok) {
        if (print) printf("  FAIL relation: menu-status anchor did not pass\n");
        return 0;
    }
    if (results[9].targets[1] + 4 != results[9].targets[0]) {
        if (print)
            printf("  FAIL relation: menu-status halves are not adjacent "
                   "(0x%llX, 0x%llX)\n",
                   results[9].targets[1], results[9].targets[0]);
        ok = 0;
    } else if (print) {
        printf("  PASS relation: menu-status pair is two adjacent ints\n");
    }
    if (results[9].targets[0] == results[8].targets[0] ||
        results[9].targets[0] == results[8].targets[1] ||
        results[9].targets[1] == results[8].targets[0] ||
        results[9].targets[1] == results[8].targets[1]) {
        if (print)
            printf("  FAIL relation: menu-status overlaps game-status\n");
        ok = 0;
    } else if (print) {
        printf("  PASS relation: menu-status and game-status are four distinct "
               "globals\n");
    }

    if (!results[10].ok || !results[11].ok) {
        if (print)
            printf("  FAIL relation: arm-body anchors did not both pass\n");
        return 0;
    }
    if (results[10].targets[0] != results[11].targets[0]) {
        if (print)
            printf("  FAIL relation: arm-body read and write disagree "
                   "(0x%llX, 0x%llX)\n",
                   results[10].targets[0], results[11].targets[0]);
        ok = 0;
    } else if (print) {
        printf("  PASS relation: arm-body has two matching code anchors\n");
    }
    {   /* And it is a pointer of its own, not one of the words we already hold.
           They all live in the same writable data, so an off-by-a-displacement
           landing on a neighbour is the failure to guard against. */
        int j;
        ULONGLONG others[11];
        others[0] = results[0].targets[0]; others[1] = results[0].targets[1];
        others[2] = results[1].targets[0]; others[3] = results[1].targets[1];
        others[4] = results[3].targets[0]; others[5] = results[3].targets[1];
        others[6] = results[7].targets[0];
        others[7] = results[8].targets[0]; others[8] = results[8].targets[1];
        others[9] = results[9].targets[0]; others[10] = results[9].targets[1];
        for (j = 0; j < 11; j++) {
            if (others[j] == results[10].targets[0]) {
                if (print)
                    printf("  FAIL relation: arm-body aliases an existing "
                           "anchor\n");
                ok = 0;
                break;
            }
        }
        if (print && j == 11)
            printf("  PASS relation: arm-body is distinct from every other "
                   "anchor\n");
    }
    /* ArmCamRotateShift. The two displacements in one branch must describe one
       SVECTOR: the address the lea takes, and .vy two bytes into it. Getting
       that wrong by any amount other than exactly 2 means the pattern is not
       looking at what its name says. */
    if (!results[12].ok) {
        if (print)
            printf("  FAIL relation: ArmCamRotateShift anchor did not pass\n");
        return 0;
    }
    if (results[12].targets[1] + 2 != results[12].targets[0]) {
        if (print)
            printf("  FAIL relation: ArmCamRotateShift .vy is not 2 bytes into "
                   "the struct the lea takes (0x%llX, 0x%llX)\n",
                   results[12].targets[1], results[12].targets[0]);
        ok = 0;
    } else if (print) {
        printf("  PASS relation: ArmCamRotateShift address and .vy describe "
               "one SVECTOR\n");
    }
    {   /* It is an SVECTOR of its own, in the same writable run as everything
           else here, so the neighbour test applies to it too. */
        int j;
        ULONGLONG others[12];
        others[0] = results[0].targets[0]; others[1] = results[0].targets[1];
        others[2] = results[1].targets[0]; others[3] = results[1].targets[1];
        others[4] = results[3].targets[0]; others[5] = results[3].targets[1];
        others[6] = results[7].targets[0];
        others[7] = results[8].targets[0]; others[8] = results[8].targets[1];
        others[9] = results[9].targets[0]; others[10] = results[9].targets[1];
        others[11] = results[10].targets[0];
        for (j = 0; j < 12; j++) {
            if (others[j] == results[12].targets[0] ||
                others[j] == results[12].targets[1]) {
                if (print)
                    printf("  FAIL relation: ArmCamRotateShift aliases an "
                           "existing anchor\n");
                ok = 0;
                break;
            }
        }
        if (print && j == 12)
            printf("  PASS relation: ArmCamRotateShift is distinct from every "
                   "other anchor\n");
    }
    return ok;
}

/* ------------------------------------------------ identity and PE view --- */

static int sha256_file(const char *path, char output[65])
{
    BCRYPT_ALG_HANDLE algorithm = NULL;
    BCRYPT_HASH_HANDLE hash = NULL;
    PUCHAR object = NULL;
    PUCHAR digest = NULL;
    DWORD object_size = 0;
    DWORD digest_size = 0;
    DWORD got = 0;
    DWORD read = 0;
    HANDLE file = INVALID_HANDLE_VALUE;
    unsigned char buffer[READ_CHUNK];
    NTSTATUS status;
    int ok = 0;
    DWORD i;

    status = BCryptOpenAlgorithmProvider(
        &algorithm, BCRYPT_SHA256_ALGORITHM, NULL, 0);
    if (status < 0) goto done;
    status = BCryptGetProperty(algorithm, BCRYPT_OBJECT_LENGTH,
        (PUCHAR)&object_size, sizeof(object_size), &got, 0);
    if (status < 0) goto done;
    status = BCryptGetProperty(algorithm, BCRYPT_HASH_LENGTH,
        (PUCHAR)&digest_size, sizeof(digest_size), &got, 0);
    if (status < 0 || digest_size != 32) goto done;
    object = (PUCHAR)HeapAlloc(GetProcessHeap(), 0, object_size);
    digest = (PUCHAR)HeapAlloc(GetProcessHeap(), 0, digest_size);
    if (!object || !digest) goto done;
    status = BCryptCreateHash(
        algorithm, &hash, object, object_size, NULL, 0, 0);
    if (status < 0) goto done;

    file = CreateFileA(path, GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE |
                       FILE_SHARE_DELETE, NULL, OPEN_EXISTING,
                       FILE_ATTRIBUTE_NORMAL, NULL);
    if (file == INVALID_HANDLE_VALUE) goto done;
    for (;;) {
        if (!ReadFile(file, buffer, sizeof(buffer), &read, NULL)) goto done;
        if (!read) break;
        status = BCryptHashData(hash, buffer, read, 0);
        if (status < 0) goto done;
    }
    status = BCryptFinishHash(hash, digest, digest_size, 0);
    if (status < 0) goto done;
    for (i = 0; i < digest_size; i++)
        sprintf_s(output + i * 2, 65 - i * 2, "%02X", digest[i]);
    output[64] = 0;
    ok = 1;

done:
    if (file != INVALID_HANDLE_VALUE) CloseHandle(file);
    if (hash) BCryptDestroyHash(hash);
    if (object) HeapFree(GetProcessHeap(), 0, object);
    if (digest) HeapFree(GetProcessHeap(), 0, digest);
    if (algorithm) BCryptCloseAlgorithmProvider(algorithm, 0);
    return ok;
}

static int inspect_disk_pe(const char *path, DiskPe *out)
{
    HANDLE file;
    LARGE_INTEGER size;
    IMAGE_DOS_HEADER dos;
    IMAGE_NT_HEADERS64 nt;
    DWORD read;
    LARGE_INTEGER position;

    memset(out, 0, sizeof(*out));
    file = CreateFileA(path, GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE |
                       FILE_SHARE_DELETE, NULL, OPEN_EXISTING,
                       FILE_ATTRIBUTE_NORMAL, NULL);
    if (file == INVALID_HANDLE_VALUE) return 0;
    if (!GetFileSizeEx(file, &size) ||
        !ReadFile(file, &dos, sizeof(dos), &read, NULL) ||
        read != sizeof(dos) || dos.e_magic != IMAGE_DOS_SIGNATURE) {
        CloseHandle(file);
        return 0;
    }
    position.QuadPart = dos.e_lfanew;
    if (!SetFilePointerEx(file, position, NULL, FILE_BEGIN) ||
        !ReadFile(file, &nt, sizeof(nt), &read, NULL) ||
        read != sizeof(nt) || nt.Signature != IMAGE_NT_SIGNATURE ||
        nt.OptionalHeader.Magic != IMAGE_NT_OPTIONAL_HDR64_MAGIC) {
        CloseHandle(file);
        return 0;
    }
    CloseHandle(file);
    out->file_size = (ULONGLONG)size.QuadPart;
    out->timestamp = nt.FileHeader.TimeDateStamp;
    out->entry_rva = nt.OptionalHeader.AddressOfEntryPoint;
    out->image_base = nt.OptionalHeader.ImageBase;
    out->image_size = nt.OptionalHeader.SizeOfImage;
    out->machine = nt.FileHeader.Machine;
    out->section_count = nt.FileHeader.NumberOfSections;
    return 1;
}

static void free_live_image(LiveImage *image)
{
    free(image->bytes);
    free(image->valid);
    memset(image, 0, sizeof(*image));
}

static int fingerprint_ok(const DiskPe *disk, const LiveImage *image,
                          const char *sha)
{
    int ok = 1;
    if (disk->file_size != EXPECTED_FILE_SIZE) ok = 0;
    if (_stricmp(sha, EXPECTED_SHA256) != 0) ok = 0;
    if (disk->machine != IMAGE_FILE_MACHINE_AMD64) ok = 0;
    if (disk->timestamp != EXPECTED_TIMESTAMP) ok = 0;
    if (disk->entry_rva != EXPECTED_ENTRY_RVA) ok = 0;
    if (disk->image_base != EXPECTED_IMAGE_BASE) ok = 0;
    if (disk->image_size != EXPECTED_IMAGE_SIZE) ok = 0;

    if (image->nt->FileHeader.Machine != disk->machine) ok = 0;
    if (image->nt->FileHeader.TimeDateStamp != disk->timestamp) ok = 0;
    if (image->nt->FileHeader.NumberOfSections != disk->section_count) ok = 0;
    if (image->nt->OptionalHeader.AddressOfEntryPoint != disk->entry_rva) ok = 0;
    /* ImageBase is the ONE header field the loader rewrites. When ASLR moves
       the module, the in-memory OptionalHeader.ImageBase becomes the address it
       was actually mapped at, not the 0x140000000 the file asks for - so an
       exact match against the disk value fails on every normal retail run,
       externally and in-process alike. Both readings are legitimate identity;
       anything else means the header has been tampered with. The disk-side
       check above stays exact. */
    if (image->nt->OptionalHeader.ImageBase != disk->image_base &&
        image->nt->OptionalHeader.ImageBase != image->base) ok = 0;
    if (image->nt->OptionalHeader.SizeOfImage != disk->image_size) ok = 0;
    if (image->size != disk->image_size) ok = 0;
    return ok;
}

/* The PE-header half of building a LiveImage, shared because it is the part
   that decides whether the byte view can be trusted at all. How the bytes were
   obtained - ReadProcessMemory from outside, or a copy of our own module from
   inside - is the caller's business. */
static int validate_image_headers(LiveImage *image)
{
    IMAGE_DOS_HEADER *dos;
    size_t section_headers_end;

    if (!all_valid(image->valid, 0, sizeof(IMAGE_DOS_HEADER), image->size))
        return 0;
    dos = (IMAGE_DOS_HEADER *)image->bytes;
    if (dos->e_magic != IMAGE_DOS_SIGNATURE || dos->e_lfanew < 0 ||
        (DWORD)dos->e_lfanew > image->size - sizeof(IMAGE_NT_HEADERS64))
        return 0;
    if (!all_valid(image->valid, (DWORD)dos->e_lfanew,
                   sizeof(IMAGE_NT_HEADERS64), image->size))
        return 0;
    image->nt = (IMAGE_NT_HEADERS64 *)(image->bytes + dos->e_lfanew);
    if (image->nt->Signature != IMAGE_NT_SIGNATURE ||
        image->nt->FileHeader.Machine != IMAGE_FILE_MACHINE_AMD64 ||
        image->nt->OptionalHeader.Magic != IMAGE_NT_OPTIONAL_HDR64_MAGIC)
        return 0;
    image->section_count = image->nt->FileHeader.NumberOfSections;
    if (!image->section_count || image->section_count > 96) return 0;
    image->sections = IMAGE_FIRST_SECTION(image->nt);
    section_headers_end = (size_t)((unsigned char *)image->sections -
                                   image->bytes) +
                          image->section_count *
                              sizeof(IMAGE_SECTION_HEADER);
    if (section_headers_end > image->size ||
        !all_valid(image->valid, 0, section_headers_end, image->size))
        return 0;
    return 1;
}

/* ------------------------------------------------- synthetic test images --- *
 *
 * Shared so that both halves can exercise the SAME resolver against a hand
 * built image. Nothing here ever touches a real process; the scanner's
 * --self-test and the bridge's desk tests both stand on these.
 */






























































































/* REX.W form: 48 8B 05 disp32 = mov rax,[rip+disp32]. */












































/* movups/movsd xmm,[rsi+rdx*8+disp32] - the indexed form retail uses to read
   GV_PadData, where the disp32 is the array's RVA. */


























/* `test r/m8, imm8` on a register - 41 F6 C2 <imm> in retail, the guard in
   front of every scenario-press read. */









/* `jz +2` then `mov r32,[rip+d32]` (8B) or `or r32,[rip+d32]` (0B) - the
   guarded read itself, in the two spellings the two branches compile to. */










/* `and r32, imm8` on a register - 41 83 E2 <imm>, the one-shot's own clear. */









/*
 * The GV_PadPress anchor, exercised against a hand-built image by BOTH halves
 * - the offline scanner and the in-process bridge's desk suite - so the thing
 * under test is the shipping resolver and not a copy of it.
 *
 * The image is modelled on the retail bytes, decoys included. The pad update's
 * normal branch reads THREE guarded globals four bytes apart from each other -
 * GV_PadMask[0] and GV_PadMask[1] under `test flag,4` and `test flag,8`, and
 * GV_PadPress under `test flag,0x20` - and the masks are the neighbours an
 * address-arithmetic guess would land on. If the 0x20 guard ever stops being
 * load-bearing, this image resolves to a MASK and the anchor would arm the
 * word that SUPPRESSES buttons while claiming to press one.
 *
 * Returns 1 on pass. Every mutation listed inside must turn it into a 0.
 */
























































































































































/* ------------------------------------------------------ one-call resolve --- *
 *
 * What the in-process bridge needs, resolved in one go and reported as a single
 * pass/fail. The scanner keeps calling the individual finders because it prints
 * a per-anchor verdict; the bridge has nothing to print and only one decision to
 * make, so it asks once and stays inert on anything short of a full pass.
 */

typedef struct DG_ANCHORS {
    int ok;
    /* Bluepoint's hidden first-person camera state. */
    ULONGLONG gbp_override;
    ULONGLONG gbp_move;
    ULONGLONG gbp_toggle;
    ULONGLONG gbp_active;
    /* The native subject state CheckWatch drives from them. */
    ULONGLONG pl_subject_move;
    ULONGLONG pl_subject_toggle;
    /* Runtime pad masks. Never hardcoded: the SubjectMove patterns set SUBJECT
       to 0, and pattern A reads a user-configurable assignment.
       pad_stop_aim is PL_PAD_STOP_AIM, not PL_PAD_SUBJECT_TOGGLE: retail never
       assigns the latter, so there is nothing to anchor - see find_pad_masks. */
    ULONGLONG pad_subject;
    ULONGLONG pad_stop_aim;
    ULONGLONG pad_weapon;
    ULONGLONG pad_press_weapon;
    /* Action()'s PlayerPad merge point: after the GV_PadData copy, before the
       Bluepoint weapon/button state rewrites it. */
    ULONGLONG player_pad;
    ULONGLONG gv_pad_data;
    /* One more identity anchor, carried through so the bridge can snapshot the
       raw player-status qword. Its exact bit semantics are NOT settled by two
       independent anchors, so it gates fail-closed and never fail-open. */
    ULONGLONG gm_player_status;
    /* Both halves of the game-status word, because there is no such thing as
       reading one of them: GM_CheckGameStatus is
       `(GM_GameStatus | GM_GameStatusScn) & state`. Resolving one and testing
       it alone would call every scenario-set state safe. The names record the
       offline evidence for which is which; no decision depends on them, since
       the bridge ORs the pair the way the game does. */
    ULONGLONG gm_game_status;       /* the `or` operand, the lower address */
    ULONGLONG gm_game_status_scn;   /* the `mov` operand, 4 bytes above it */
    /* The same again for the menu word, which is where a codec call shows up.
       MENU_RADIO_ON is not a game state and never appears in GM_GameStatus. */
    ULONGLONG gm_menu_status;
    ULONGLONG gm_menu_status_scn;
    /* The subjective arm object, held by two independent anchors. F4 needs it
       to make the arm visible; F5 and F6 need it because its right-hand joint
       is what the bullet matrix is taken from. Nothing writes through this yet
       - it is resolved so the first live run has something to observe. */
    ULONGLONG gm_player_arm_body;

    ULONGLONG arm_cam_rotate_shift;
    DWORD merge_seam;               /* RVA of the seam itself */
    DWORD merge_function_begin;     /* enclosing .pdata bounds, for the detour */
    DWORD merge_function_end;
    DWORD copy_seam;                /* optional early seam, 0 = not found */
    DWORD copy_function_begin;
    DWORD copy_function_end;

    ULONGLONG gv_pad_press;
    ULONGLONG gv_pad_data_direct;   /* 0 unless independently verified */
    DWORD pad_update_begin;         /* GV_UpdatePadSystem's unwind bounds */
    DWORD pad_update_end;
    DWORD pad_press_read;           /* the guarded read of GV_PadPress[0] */
    DWORD pad_press_clear;          /* the one-shot's own clear */
    DWORD pad_direct_seam;          /* past UpdatePad(&GV_PadDataDirect[0]) */
    int pad_press_ok;
} DG_ANCHORS;

static int dg_anchors_resolve(const LiveImage *image, DG_ANCHORS *out)
{
    PatternResult results[PATTERN_COUNT];
    PadMaskResult masks;
    PlayerPadResult player_pad;
    size_t i;

    memset(out, 0, sizeof(*out));
    for (i = 0; i < PATTERN_COUNT; i++) {
        scan_pattern(image, &g_patterns[i], &results[i]);
        /* Exactly one hit, resolvable RIP targets, all in writable data. */
        if (!results[i].ok) return 0;
    }
    if (!check_relationships(results, 0)) return 0;

    find_pad_masks(image, results, &masks);
    if (!masks.ok) return 0;
    find_player_pad(image, &masks, &player_pad);
    if (!player_pad.ok) return 0;

    /* find_pad_masks is seeded from the pattern targets, so these must agree.
       Checking anyway costs one compare and turns a future refactor that breaks
       the seeding into a fail-closed rather than a silent mismatch. */
    if (masks.subject_toggle != results[0].targets[1] ||
        masks.subject_move != results[3].targets[1])
        return 0;
    if (player_pad.merge_seam <= player_pad.function_begin ||
        player_pad.merge_seam >= player_pad.function_end)
        return 0;

    out->gbp_toggle = results[0].targets[0];
    out->pl_subject_toggle = results[0].targets[1];
    out->gbp_override = results[1].targets[0];
    out->gbp_active = results[1].targets[1];
    out->gbp_move = results[3].targets[0];
    out->pl_subject_move = results[3].targets[1];
    out->pad_subject = masks.pad_subject;
    out->pad_stop_aim = masks.pad_stop_aim;
    out->pad_weapon = masks.pad_weapon;
    out->pad_press_weapon = masks.pad_press_weapon;
    out->gm_player_status = results[7].targets[0];
    out->gm_game_status = results[8].targets[1];
    out->gm_game_status_scn = results[8].targets[0];
    out->gm_menu_status = results[9].targets[1];
    out->gm_menu_status_scn = results[9].targets[0];
    /* Either anchor would do; check_relationships has already required them to
       be equal, so taking the read side is arbitrary and safe. */
    out->gm_player_arm_body = results[10].targets[0];
    /* targets[1] is the lea - the struct itself. targets[0] is .vy two bytes
       in, and check_relationships has already required that. */
    out->arm_cam_rotate_shift = results[12].targets[1];
    out->player_pad = player_pad.player_pad;
    out->gv_pad_data = player_pad.gv_pad_data;
    out->merge_seam = player_pad.merge_seam;
    out->merge_function_begin = player_pad.function_begin;
    out->merge_function_end = player_pad.function_end;
    out->copy_seam = player_pad.copy_seam;
    out->copy_function_begin = player_pad.copy_function_begin;
    out->copy_function_end = player_pad.copy_function_end;

    /* Optional, and the ONLY optional anchor here. It is filled after `ok` has
       already been decided by the mandatory set, and its own failure is
       recorded rather than propagated - see the field comments. Callers must
       test pad_press_ok, never gv_pad_press against zero: a resolver that
       silently handed back address 0 is exactly the failure mode the rest of
       this file is built to make impossible. */
    {
        PadPressResult pad_press;
        find_pad_press(image, &player_pad, &pad_press);
        if (pad_press.ok) {
            out->gv_pad_press = pad_press.gv_pad_press;
            out->gv_pad_data_direct = pad_press.gv_pad_data_direct;
            out->pad_update_begin = pad_press.function_begin;
            out->pad_update_end = pad_press.function_end;
            out->pad_press_read = pad_press.press_read;
            out->pad_press_clear = pad_press.press_clear;
            out->pad_direct_seam = pad_press.direct_seam;
            out->pad_press_ok = 1;
        }
    }

    out->ok = 1;
    return 1;
}

#endif /* DG_ANCHORS_H */
