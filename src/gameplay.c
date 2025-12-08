#include <raylib.h>
#include <raymath.h>

#include "gameplay.h"

void update_gameplay(GameWorld* world, Camera3D camera, float dt) {
    // TODO: Everything
}

static void update_player_orientation(GameWorld* world, int player_idx, Camera3D camera) {
    // TODO: Cube always looking to cursor 
    // (get mouse ray -> intersect with plane -> direction vector (y = 0) -> new quaternion)
}

static void update_player_movement(GameWorld* world, int player_idx) {
    // TODO: Move player (get local vectors -> get input -> just apply speed)
}

static void update_player_jump(GameWorld* world, int player_idx) {
    // TODO: Vertical impulse so just velocity if it's grounded (check if it collided with something)
}

void constrain_player_upright(GameWorld* world, int player_idx) {
    // TODO: Rotate ex (0,0,1) and rotate it with current orient -> y = 0 -> normalize -> new quaternion 
    // (after update_physics)
}

static void apply_magnetic_force(GameWorld* world, int player_idx) {
    // TODO: I guess G + & H - (check all entities and add force to its physics_state)
    // normalize distance between player & entity -> force = that * MAGNET_FORCE * mass -> if H then invert
}

static int find_player_index(GameWorld* world) {
    // TODO: Add bool is_player idk where dxdxdx
}