#ifndef DG_RADIAL_NATIVE_READ_H
#define DG_RADIAL_NATIVE_READ_H
#include "dg_radial_inventory.h"
/* Read-only retail no-use/catalog observer. Never calls native functions.
 * kind index: 0 weapon, 1 item. These are semantic enum bounds, cross-checked
 * against the supported retail name/type profiles, NOT physical storage lengths.
 * Caller supplies approved image identity and serialized, fault-contained reads.
 * phase_fresh must mean AFTER this tick's SetNoUse and before equip checks.
 * NONE (ID 0) skips stock checks but retains exact type and no-use checks.
 * Eligibility is inventory+no-use only, never the complete engine-ready gate.
 * No output authorizes a write; root phase/ownership/commit policy is separate.
 */
enum { DG_RN_WEAPONS=22, DG_RN_ITEMS=41 };
typedef struct dg_radial_native_anchors {
    int valid;
    uint64_t module_base;
    size_t module_size;
    uint64_t no_use_function[2], default_function[2];
    uint64_t dynamic_mask[2], scenario_mask[2], type_mask[2], types[2], names[2];
    uint64_t cross_table[2][2];
} dg_radial_native_anchors;
typedef struct dg_radial_native_snapshot {
    int valid, eligibility_valid;
    dg_radial_inventory_snapshot inventory;
    uint32_t weapon_types[DG_RN_WEAPONS], item_types[DG_RN_ITEMS];
    uint64_t dynamic_mask[2], scenario_mask[2];
    uint32_t type_mask[2];
    uint64_t eligible_weapons, eligible_items;
} dg_radial_native_snapshot;
int dg_radial_native_resolve(const dg_radial_inventory_image *image,
                            const dg_radial_inventory_anchors *inventory,
                            dg_radial_native_anchors *out);
int dg_radial_native_sample(const dg_radial_inventory_image *image,
                           const dg_radial_inventory_anchors *inventory,
                           const dg_radial_native_anchors *anchors,
                           int phase_fresh, dg_radial_native_snapshot *out);
/* NULL for every excluded microphone/placeholder ID. ID zero names native unequip (None).
 * Merely naming a profiled ID says nothing about current eligibility. */
const char *dg_radial_native_label(int kind, int id);
#endif
