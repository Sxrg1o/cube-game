#include <raylib.h>
#include <raymath.h>

#include "gameplay.h"

static int find_player_index(GameWorld* world) {
    for(int i = 0; i < world->entity_count; i++) {
        if(world->player_logic[i].is_player) return i;
    }
    return -1;
}

static Vector3 get_mouse_ray_position(Camera3D camera) {
    Ray mouse_ray = GetScreenToWorldRay(GetMousePosition(), camera);
    
    if (fabs(mouse_ray.direction.y) < 0.00001f) return (Vector3){0};

    float t = -(mouse_ray.position.y) / mouse_ray.direction.y;

    if(t < 0) return (Vector3){0};

    Vector3 point = Vector3Add(mouse_ray.position, Vector3Scale(mouse_ray.direction, t));
    return point;
}

static void update_player_orientation(GameWorld* world, int player_idx, Camera3D camera) {
    Vector3 target = get_mouse_ray_position(camera);
    Vector3 player_pos = world->transform[player_idx].position;
    Vector3 direction = Vector3Subtract(target, player_pos);
    direction.y = 0; 
    
    if (Vector3LengthSqr(direction) < 0.01f) return;

    direction = Vector3Normalize(direction);
    
    float angle = atan2f(direction.x, direction.z);
    Quaternion target_rotation = QuaternionFromAxisAngle((Vector3){0, 1, 0}, angle);
    world->transform[player_idx].orientation = target_rotation;
}

static void update_player_movement(GameWorld* world, int player_idx) {
    PhysicsStateComponent* phys = &world->physics_state[player_idx];
    
    if(!phys->in_ground) return; 

    Quaternion q = world->transform[player_idx].orientation;
    Vector3 forward = Vector3RotateByQuaternion((Vector3){0, 0, 1}, q);
    Vector3 right   = Vector3RotateByQuaternion((Vector3){1, 0, 0}, q);
    forward.y = 0; forward = Vector3Normalize(forward);
    right.y = 0;   right = Vector3Normalize(right);

    Vector3 target_vel = {0};
    if (IsKeyDown(KEY_W)) target_vel = Vector3Add(target_vel, forward);
    if (IsKeyDown(KEY_S)) target_vel = Vector3Subtract(target_vel, forward);
    if (IsKeyDown(KEY_D)) target_vel = Vector3Add(target_vel, right);
    if (IsKeyDown(KEY_A)) target_vel = Vector3Subtract(target_vel, right);

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

static void update_player_jump(GameWorld* world, int player_idx) {
    PhysicsStateComponent* phys = &world->physics_state[player_idx];
    
    if (IsKeyPressed(KEY_SPACE) && phys->in_ground) {
        phys->linear_velocity.y = PLAYER_JUMP_FORCE;
        phys->in_ground = false;
    }
}

void constrain_player_upright(GameWorld* world) {
    int player_idx = find_player_index(world);
    if (player_idx == -1) return;

    Quaternion q = world->transform[player_idx].orientation;
    Vector3 current_forward = Vector3RotateByQuaternion((Vector3){0, 0, 1}, q);
    current_forward.y = 0;
    
    if (Vector3LengthSqr(current_forward) < 0.001f) return;

    current_forward = Vector3Normalize(current_forward);
    Quaternion upright_q = QuaternionFromVector3ToVector3((Vector3){0, 0, 1}, current_forward);
    world->transform[player_idx].orientation = upright_q;
}

static void apply_magnetic_force(GameWorld* world, int player_idx) {
    int polarity = 0;
    if (IsKeyDown(KEY_G)) polarity = 1;
    if (IsKeyDown(KEY_H)) polarity = -1;

    if (polarity == 0) return;

    Vector3 player_pos = world->transform[player_idx].position;

    for (int i = 0; i < world->entity_count; i++) {
        if (i == player_idx) continue;
        if (world->physics_prop[i].inverse_mass == 0.0f) continue;

        Vector3 target_pos = world->transform[i].position;
        Vector3 diff = Vector3Subtract(player_pos, target_pos);
        float dist_sq = Vector3LengthSqr(diff);

        if (dist_sq < MAGNET_RADIUS * MAGNET_RADIUS && dist_sq > 0.1f) {
            float dist = sqrtf(dist_sq);
            Vector3 dir = Vector3Scale(diff, 1.0f / dist);
            float force_mag = MAGNET_FORCE_MAGNITUDE * world->physics_prop[i].mass * (float)polarity;
            
            if (polarity == -1) dir = Vector3Negate(dir); 

            world->physics_state[i].force_accumulator = Vector3Add(
                world->physics_state[i].force_accumulator, 
                Vector3Scale(dir, fabsf(force_mag))
            );
        }
    }
}

void update_gameplay(GameWorld* world, Camera3D camera, float dt) {
    int player_idx = find_player_index(world);
    if (player_idx == -1) return;

    update_player_orientation(world, player_idx, camera);
    update_player_movement(world, player_idx);
    update_player_jump(world, player_idx);
    apply_magnetic_force(world, player_idx);
}