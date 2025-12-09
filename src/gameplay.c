#include <raylib.h>
#include <raymath.h>

#include "gameplay.h"

static void update_player_orientation(GameWorld* world, int player_idx, float yaw) {
    Quaternion target_rotation = QuaternionFromAxisAngle((Vector3){0, 1, 0}, yaw);
    world->transform[player_idx].orientation = target_rotation;
}

static void update_player_movement(GameWorld* world, int player_idx, PlayerInput input) {
    PhysicsStateComponent* phys = &world->physics_state[player_idx];
    
    if(!phys->in_ground) return; 

    Quaternion q = world->transform[player_idx].orientation;
    Vector3 forward = Vector3RotateByQuaternion((Vector3){0, 0, 1}, q);
    Vector3 right   = Vector3RotateByQuaternion((Vector3){1, 0, 0}, q);
    forward.y = 0; forward = Vector3Normalize(forward);
    right.y = 0;   right = Vector3Normalize(right);

    Vector3 target_vel = {0};

    if (input.buttons & BUTTON_UP)    target_vel = Vector3Add(target_vel, forward);
    if (input.buttons & BUTTON_DOWN)  target_vel = Vector3Subtract(target_vel, forward);
    if (input.buttons & BUTTON_RIGHT) target_vel = Vector3Add(target_vel, right);
    if (input.buttons & BUTTON_LEFT)  target_vel = Vector3Subtract(target_vel, right);

    if (Vector3LengthSqr(target_vel) > 0.01f) {
        target_vel = Vector3Normalize(target_vel);
        target_vel = Vector3Scale(target_vel, PLAYER_MOVE_SPEED);
        phys->linear_velocity.x = target_vel.x;
        phys->linear_velocity.z = target_vel.z;
    } else {
        phys->linear_velocity.x = 0;
        phys->linear_velocity.z = 0;
    }
}

static void update_player_jump(GameWorld* world, int player_idx, PlayerInput input) {
    PhysicsStateComponent* phys = &world->physics_state[player_idx];

    if ((input.buttons & BUTTON_JUMP) && phys->in_ground) {
        phys->linear_velocity.y = PLAYER_JUMP_FORCE;
        phys->in_ground = false;
    }
}

void constrain_player_upright(GameWorld* world, int player_idx) {
    if (player_idx == -1) return;

    Quaternion q = world->transform[player_idx].orientation;
    Vector3 current_forward = Vector3RotateByQuaternion((Vector3){0, 0, 1}, q);
    current_forward.y = 0;
    
    if (Vector3LengthSqr(current_forward) < 0.001f) return;

    current_forward = Vector3Normalize(current_forward);
    Quaternion upright_q = QuaternionFromVector3ToVector3((Vector3){0, 0, 1}, current_forward);
    world->transform[player_idx].orientation = upright_q;
}

static void apply_magnetic_force(GameWorld* world, int player_idx, int polarity) {
    if (polarity == 0) return;

    Vector3 player_pos = world->transform[player_idx].position;

    for (int i = 0; i < world->entity_count; i++) {
        if (!world->entity_active[i]) continue;
        if (i == player_idx) continue;
        if (world->physics_prop[i].inverse_mass == 0.0f) continue;

        Vector3 target_pos = world->transform[i].position;
        Vector3 diff = Vector3Subtract(player_pos, target_pos);
        float dist_sq = Vector3LengthSqr(diff);

        if (dist_sq < MAGNET_RADIUS * MAGNET_RADIUS && dist_sq > 0.1f) {
            float dist = sqrtf(dist_sq);
            Vector3 dir = Vector3Scale(diff, 1.0f / dist);
            float force_mag = MAGNET_FORCE_MAGNITUDE * world->physics_prop[i].mass;
            
            if (polarity == -1) dir = Vector3Negate(dir); 

            world->physics_state[i].force_accumulator = Vector3Add(
                world->physics_state[i].force_accumulator, 
                Vector3Scale(dir, force_mag)
            );
        }
    }
}

static bool update_energy_tank(float* energy, bool* overheated, bool is_using, float dt) {
    if (is_using && !(*overheated)) {
        *energy -= dt;

        if (*energy <= 0.0f) {
            *energy = 0.0f;
            *overheated = true;
            return false;
        }

        return true;
    } else {
        *energy += RECHARGE_RATE * dt;
        if (*energy > MAX_ENERGY) *energy = MAX_ENERGY;

        if (*overheated && *energy >= UNLOCK_THRESHOLD) {
            *overheated = false;
        }

        return false;
    }
}

static void handle_power(GameWorld* world, int player_idx, PlayerInput input, float dt) {
    PlayerLogicComponent* logic = &world->player_logic[player_idx];
    
    bool want_attract = (input.buttons & BUTTON_ATTRACT);
    bool want_repel   = (input.buttons & BUTTON_REPEL);

    if (want_attract) want_repel = false;

    bool can_attract = update_energy_tank(&logic->energy_attract, &logic->attract_overheat, want_attract, dt);
    bool can_repel = update_energy_tank(&logic->energy_repel, &logic->repel_overheat, want_repel, dt);
    Color final_color = DARKPURPLE;

    if (can_attract && want_attract) {
        apply_magnetic_force(world, player_idx, 1);
        final_color = RED;
    } 
    else if (can_repel && want_repel) {
        apply_magnetic_force(world, player_idx, -1);
        final_color = BLUE;
    }

    if ((want_attract && logic->attract_overheat) || (want_repel && logic->repel_overheat)) {
        final_color = DARKGRAY; 
    }

    world->rendering[player_idx].model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = final_color;
}

static void update_player_dash(GameWorld* world, int player_idx, PlayerInput input, float dt) {
    PlayerLogicComponent* logic = &world->player_logic[player_idx];
    PhysicsStateComponent* phys = &world->physics_state[player_idx];

    if (logic->dash_cooldown > 0.0f) {
        logic->dash_cooldown -= dt;
    }

    if ((input.buttons & BUTTON_DASH) && logic->dash_cooldown <= 0.0f) {
        Quaternion q = world->transform[player_idx].orientation;
        Vector3 forward = Vector3RotateByQuaternion((Vector3){0, 0, 1}, q);
        forward.y = 0;
        forward = Vector3Normalize(forward);
        phys->linear_velocity = Vector3Add(phys->linear_velocity, Vector3Scale(forward, DASH_FORCE));
        phys->linear_velocity.y = 2.0f; 
        phys->in_ground = false;
        logic->dash_cooldown = DASH_COOLDOWN;
    }
}

void update_gameplay(GameWorld* world, int player_idx, PlayerInput input, float dt) {
    if (player_idx == -1) return;

    update_player_orientation(world, player_idx, input.yaw);
    update_player_movement(world, player_idx, input);
    update_player_jump(world, player_idx, input);
    handle_power(world, player_idx, input, dt);
    update_player_dash(world, player_idx, input, dt);
}
