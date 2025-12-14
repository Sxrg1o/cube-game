#include <raylib.h>
#include <raymath.h>
#include <stdlib.h>

#include "state.h"
#include "gameplay.h"
#include "world.h"

static void update_player_orientation(GameWorld* world, int player_idx, float yaw) {
    Quaternion target_rotation = QuaternionFromAxisAngle((Vector3){0, 1, 0}, yaw);
    world->transform[player_idx].orientation = target_rotation;
}

static void update_player_movement(GameWorld* world, int player_idx, PlayerInput input, GameConfig* config) {
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

    float move_speed = config ? config->player_move_speed : PLAYER_MOVE_SPEED_DEFAULT;

    if (Vector3LengthSqr(target_vel) > 0.01f) {
        target_vel = Vector3Normalize(target_vel);
        target_vel = Vector3Scale(target_vel, move_speed);
        phys->linear_velocity.x = target_vel.x;
        phys->linear_velocity.z = target_vel.z;
    } else {
        phys->linear_velocity.x = 0;
        phys->linear_velocity.z = 0;
    }
}

static void update_player_jump(GameWorld* world, int player_idx, PlayerInput input, GameConfig* config) {
    PhysicsStateComponent* phys = &world->physics_state[player_idx];

    if ((input.buttons & BUTTON_JUMP) && phys->in_ground) {
        float jump_force = config ? config->player_jump_force : PLAYER_JUMP_FORCE_DEFAULT;
        phys->linear_velocity.y = jump_force;
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

static void apply_magnetic_force(GameWorld* world, int player_idx, int polarity, GameConfig* config) {
    if (polarity == 0) return;

    Vector3 player_pos = world->transform[player_idx].position;
    
    float mag_radius = config ? config->magnet_radius : MAGNET_RADIUS_DEFAULT;
    float mag_force = config ? config->magnet_force_magnitude : MAGNET_FORCE_DEFAULT;

    for (int i = 0; i < world->entity_count; i++) {
        if (!world->entity_active[i]) continue;
        if (i == player_idx) continue;
        if (world->physics_prop[i].inverse_mass == 0.0f) continue;

        Vector3 target_pos = world->transform[i].position;
        Vector3 diff = Vector3Subtract(player_pos, target_pos);
        float dist_sq = Vector3LengthSqr(diff);

        if (dist_sq < mag_radius * mag_radius && dist_sq > 0.1f) {
            float dist = sqrtf(dist_sq);
            Vector3 dir = Vector3Scale(diff, 1.0f / dist);
            float force_mag = mag_force * world->physics_prop[i].mass;
            
            if (polarity == -1) dir = Vector3Negate(dir); 

            world->physics_state[i].force_accumulator = Vector3Add(
                world->physics_state[i].force_accumulator, 
                Vector3Scale(dir, force_mag)
            );
        }
    }
}

static bool update_energy_tank(float* energy, bool* overheated, bool is_using, float dt, GameConfig* config) {
    float max_energy = config ? config->max_energy : MAX_ENERGY_DEFAULT;
    float recharge_time = config ? config->recharge_time : RECHARGE_TIME_DEFAULT;
    float recharge_rate = (recharge_time > 0.0f) ? (max_energy / recharge_time) : max_energy;
    float unlock_threshold = max_energy * 0.5f;

    if (is_using && !(*overheated)) {
        *energy -= dt;

        if (*energy <= 0.0f) {
            *energy = 0.0f;
            *overheated = true;
            return false;
        }

        return true;
    } else {
        *energy += recharge_rate * dt;
        if (*energy > max_energy) *energy = max_energy;

        if (*overheated && *energy >= unlock_threshold) {
            *overheated = false;
        }

        return false;
    }
}

static void handle_power(GameWorld* world, int player_idx, PlayerInput input, float dt, GameConfig* config) {
    PlayerLogicComponent* logic = &world->player_logic[player_idx];
    
    bool want_attract = (input.buttons & BUTTON_ATTRACT);
    bool want_repel   = (input.buttons & BUTTON_REPEL);

    if (want_attract) want_repel = false;

    bool can_attract = update_energy_tank(&logic->energy_attract, &logic->attract_overheat, want_attract, dt, config);
    bool can_repel = update_energy_tank(&logic->energy_repel, &logic->repel_overheat, want_repel, dt, config);
    Color final_color = DARKPURPLE;

    if (can_attract && want_attract) {
        apply_magnetic_force(world, player_idx, 1, config);
        final_color = RED;
    } 
    else if (can_repel && want_repel) {
        apply_magnetic_force(world, player_idx, -1, config);
        final_color = BLUE;
    }

    if ((want_attract && logic->attract_overheat) || (want_repel && logic->repel_overheat)) {
        final_color = DARKGRAY; 
    }

    world->rendering[player_idx].model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = final_color;
}

static void update_player_dash(GameWorld* world, int player_idx, PlayerInput input, float dt, GameConfig* config) {
    PlayerLogicComponent* logic = &world->player_logic[player_idx];
    PhysicsStateComponent* phys = &world->physics_state[player_idx];

    if (logic->dash_cooldown > 0.0f) {
        logic->dash_cooldown -= dt;
    }

    float dash_force = config ? config->dash_force : DASH_FORCE_DEFAULT;
    float dash_cd = config ? config->dash_cooldown : DASH_COOLDOWN_DEFAULT;

    if ((input.buttons & BUTTON_DASH) && logic->dash_cooldown <= 0.0f) {
        Quaternion q = world->transform[player_idx].orientation;
        Vector3 forward = Vector3RotateByQuaternion((Vector3){0, 0, 1}, q);
        forward.y = 0;
        forward = Vector3Normalize(forward);
        phys->linear_velocity = Vector3Add(phys->linear_velocity, Vector3Scale(forward, dash_force));
        phys->linear_velocity.y = 2.0f; 
        phys->in_ground = false;
        logic->dash_cooldown = dash_cd;
    }
}

static void handle_collision_damage(GameWorld* world) {
    for (int i = 0; i < world->collision_event_count; i++) {
        CollisionEvent* event = &world->collision_events[i];
        
        if (event->force > DAMAGE_THRESHOLD) {
            float damage = (event->force - DAMAGE_THRESHOLD) * DAMAGE_FACTOR;
            world->player_logic[event->entity_a].health -= damage;
            world->player_logic[event->entity_b].health -= damage;
            if (world->player_logic[event->entity_a].is_player) TraceLog(LOG_INFO, "OUCH! Damage: %.2f", damage);
        }
    }
}

int check_round_winner(GameWorld* world) {
    int players_alive = 0;
    int last_alive_id = -1;

    for (int i = 0; i < world->entity_count; i++) {
        if (!world->entity_active[i]) continue;
        if (!world->player_logic[i].is_player) continue;

        if (world->player_logic[i].health > 0) {
            players_alive++;
            last_alive_id = i;
        }
    }

    if (players_alive <= 1) {
        return last_alive_id;
    }
    return -1;
}

void update_gameplay(GameWorld* world, int player_idx, PlayerInput input, float dt, GameConfig* config) {
    if (player_idx == -1) return;

    update_player_orientation(world, player_idx, input.yaw);
    update_player_movement(world, player_idx, input, config);
    update_player_jump(world, player_idx, input, config);
    handle_power(world, player_idx, input, dt, config);
    update_player_dash(world, player_idx, input, dt, config);
    handle_collision_damage(world);
}

void check_fallen_entities(GameWorld* world) {
    const float DEATH_Y_LIMIT = -20.0f;

    for (int i = 0; i < world->entity_count; i++) {
        if (!world->entity_active[i]) continue;

        if (world->transform[i].position.y < DEATH_Y_LIMIT) {
            if (world->player_logic[i].is_player) {
                world->player_logic[i].health = 0.0f;
                world->entity_active[i] = false; 
            } else {
                destroy_entity(world, i);
            }
        }
    }
}

void trigger_random_event(GameWorld* world, GameConfig* config) {
    float x = (float)((int)config->main_platform_size.x - rand() % (int)config->main_platform_size.x*2);
    float z = (float)((int)config->main_platform_size.y - rand() % (int)config->main_platform_size.y*2);
    Vector3 pos = { x, 10.0f, z };

    // TODO: Randomize this even more
    EntityDesc obj = {0};
    obj.position = pos;
    obj.orientation = (Quaternion){0,0,0,1};
    obj.mass = 5.0f;
    obj.restitution = 0.5f;
    obj.is_player = false;
    
    if (rand() % 2 == 0) {
        obj.shape_type = SHAPE_CUBE;
        obj.dimentions = (Vector3){ 2.0f, 2.0f, 2.0f };
        obj.color = ORANGE;
    } else {
        obj.shape_type = SHAPE_SPHERE;
        obj.dimentions = (Vector3){ 1.5f, 0, 0 };
        obj.color = PINK;
    }

    create_entity(world, obj, config);
    TraceLog(LOG_INFO, "Evento: Objeto generado en (%.1f, %.1f)", x, z);
}
