#ifndef DG_RADIAL_PHASE_H
#define DG_RADIAL_PHASE_H
#include "dg_radial_inventory.h"
/* Resolver only: no calls, hooks, writes or readiness decisions. All addresses
   are extracted from a caller-approved loaded PE image, never fixed RVAs.
   entry is a five-byte complete MOV [RSP+8],RBX (no relocation operand).
   A detour still needs its own allocation/reachability/lifetime proof. */
typedef struct dg_radial_phase_anchors {
    int valid;
    uint64_t module_base, weapon_entry, weapon_end, item_entry, item_end;
    uint64_t caller_begin, caller_end, weapon_call, item_call, return_address;
    uint64_t player_slot, status_slot, changed_weapon, changed_item;
    uint64_t scenario_weapon, scenario_item, no_use_item_slot;
    uint64_t flags_helper, flags2_helper, status_helper, clear_flags_helper;
    uint32_t flags_offset, flags2_offset;
    unsigned patch_bytes;
    unsigned char expected_entry[5];
} dg_radial_phase_anchors;
int dg_radial_phase_resolve(const dg_radial_inventory_image *image,
    const dg_radial_inventory_anchors *inventory,dg_radial_phase_anchors *out);
#endif
