#ifndef DG_RADIAL_INVENTORY_H
#define DG_RADIAL_INVENTORY_H
#include <stddef.h>
#include <stdint.h>

enum { DG_RINV_INVENTORY=1u, DG_RINV_DESIRED=2u, DG_RINV_ACTOR=4u };
enum { DG_RINV_MAX_SLOTS=64 };
typedef int (*dg_radial_inventory_read)(void *ctx, uint64_t address, void *dst, size_t n);
typedef struct dg_radial_inventory_image {
    uint64_t base;
    size_t size;
    int approved_identity;
    dg_radial_inventory_read read;
    void *ctx;
} dg_radial_inventory_image;
typedef struct dg_radial_inventory_anchors {
    unsigned valid_bits;
    uint64_t module_base;
    size_t module_size;
    uint64_t linkvar_slot, weapons_slot, items_slot, player_slot;
    uint32_t set_offset[2][4]; /* weapons,maxweapons,items,maxitems */
    uint32_t actor_weapon_offset, actor_item_offset;
    unsigned weapon_slots, item_slots;
} dg_radial_inventory_anchors;
typedef struct dg_radial_inventory_snapshot {
    unsigned valid_bits;
    int no_use_ready, equip_ready; /* unconditionally zero */
    uint64_t player_identity, inventory_identity;
    uint64_t weapons_identity, items_identity;
    int desired_weapon, desired_item, actor_weapon, actor_item;
    unsigned weapon_slots, item_slots;
    int16_t weapons[DG_RINV_MAX_SLOTS], items[DG_RINV_MAX_SLOTS];
} dg_radial_inventory_snapshot;
unsigned dg_radial_inventory_resolve(const dg_radial_inventory_image *image,
                                    dg_radial_inventory_anchors *out);
unsigned dg_radial_inventory_sample(const dg_radial_inventory_image *image,
                                   const dg_radial_inventory_anchors *anchors,
                                   dg_radial_inventory_snapshot *out);
#endif
