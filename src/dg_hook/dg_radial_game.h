#ifndef DG_RADIAL_GAME_H
#define DG_RADIAL_GAME_H
#include "dg_radial_inventory.h"
#include "dg_radial_commit.h"
/* Called only by the registered producer during its serialized game tick.
 * Snapshot eligibility comes from the preceding native change phase. It is
 * presentation data; every proposal is revalidated at the next phase entry.
 * No arbitrary caller may use these functions to bypass control ownership. */
typedef struct dg_radial_game_catalog {
    int valid, busy;
    int current[2], previous[2]; /* actor-confirmed; None is zero */
    int actor_item, thermal_owned;
    uint64_t player, inventory, version, sampled_ms;
    uint64_t eligible[2];
    unsigned recent_count[2];
    int quantities[DG_RINV_MAX_SLOTS];
    int recent[2][6]; /* confirmed native actor selections, most recent first */
    char labels[2][DG_RINV_MAX_SLOTS][17];
} dg_radial_game_catalog;
int dg_bridge_radial_catalog(uint64_t now_ms, dg_radial_game_catalog *out);
void dg_bridge_radial_offer(const dg_radial_commit_intent *intent,
    uint64_t player, uint64_t inventory, int controls_ready, uint64_t now_ms);
/* An explicit context loss cancels pending work and catalog publication. */
void dg_bridge_radial_cancel(void);
int dg_bridge_radial_paused(void);
int dg_bridge_radial_context_held(void);
void dg_bridge_radial_feedback(int visible,int selected,int kind,uint64_t context,int accept);
void dg_bridge_item_use_offer(const dg_radial_commit_intent *intent,
    uint64_t player,uint64_t inventory,int ready,uint64_t now_ms);
#endif
