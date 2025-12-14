#pragma once

#include "components.h"

#define PLAYER_MOVE_SPEED_MIN 4.0f
#define PLAYER_MOVE_SPEED_DEFAULT 7.0f
#define PLAYER_MOVE_SPEED_MAX 15.0f

#define PLAYER_JUMP_FORCE_MIN 5.0f
#define PLAYER_JUMP_FORCE_DEFAULT 10.0f
#define PLAYER_JUMP_FORCE_MAX 20.0f

#define MAGNET_FORCE_MIN 100.0f
#define MAGNET_FORCE_DEFAULT 200.0f
#define MAGNET_FORCE_MAX 400.0f

#define MAGNET_RADIUS_MIN 5.0f
#define MAGNET_RADIUS_DEFAULT 20.0f
#define MAGNET_RADIUS_MAX 50.0f

#define MAX_ENERGY_MIN 2.0f
#define MAX_ENERGY_DEFAULT 5.0f
#define MAX_ENERGY_MAX 10.0f

#define RECHARGE_TIME_MIN 2.0f
#define RECHARGE_TIME_DEFAULT 5.0f
#define RECHARGE_TIME_MAX 10.0f

#define DASH_FORCE_MIN 10.0f
#define DASH_FORCE_DEFAULT 20.0f
#define DASH_FORCE_MAX 30.0f

#define DASH_COOLDOWN_MIN 1.0f
#define DASH_COOLDOWN_DEFAULT 1.0f
#define DASH_COOLDOWN_MAX 5.0f

#define TOTAL_HEALTH_MIN 100.0f
#define TOTAL_HEALTH_DEFAULT 200.0f
#define TOTAL_HEALTH_MAX 1000.0f

#define GRAVITY_FORCE_MIN 5.0f
#define GRAVITY_FORCE_DEFAULT 18.0f
#define GRAVITY_FORCE_MAX 30.0f

#define PLATFORM_X_MIN 20.0f
#define PLATFORM_X_DEFAULT 20.0f
#define PLATFORM_X_MAX 50.0f

#define PLATFORM_Y_MIN 20.0f
#define PLATFORM_Y_DEFAULT 20.0f
#define PLATFORM_Y_MAX 50.0f

#define PLAYER_SIZE_MIN 1.0f
#define PLAYER_SIZE_DEFAULT 2.0f
#define PLAYER_SIZE_MAX 4.0f

#define MATCH_DURATION_MIN 4.0f
#define MATCH_DURATION_DEFAULT 4.0f
#define MATCH_DURATION_MAX 8.0f

#define TIME_EVENTS_MIN 0.5f
#define TIME_EVENTS_DEFAULT 1.0f
#define TIME_EVENTS_MAX 2.0f

#define ROUNDS_MIN 1
#define ROUNDS_DEFAULT 3
#define ROUNDS_MAX 10

typedef struct {
    Vector3 position;
    Quaternion orientation;
    float mass;
    float restitution;
    ShapeType shape_type;
    Vector3 dimentions;
    Color color;
    bool is_player;
} EntityDesc;

typedef struct {
    int entity_a;
    int entity_b;
    float force;
} CollisionEvent;

typedef struct {
    int entity_count;
    int max_entities;
    bool* entity_active;
    int* free_idx;
    int free_idx_top;

    TransformComponent* transform;
    PhysicsStateComponent* physics_state;
    PhysicsPropertiesComponent* physics_prop;
    CollisionShapeComponent* collision;
    PlayerLogicComponent* player_logic;
    RenderComponent* rendering;

    CollisionEvent* collision_events;
    int collision_event_count;
    int max_collision_events;
} GameWorld;

typedef enum {
    MENU,
    LOBBY,
    PLAYING
} GameState;

typedef struct {
    float player_move_speed;
    float player_jump_force;
    float magnet_force_magnitude;
    float magnet_radius;
    float max_energy;
    float recharge_time;
    float dash_force;
    float dash_cooldown;
    float total_health;
    float gravity_force;
    Vector2 main_platform_size;
    float time_between_events;
    float player_size;
    float match_duration;
    int rounds;
} GameConfig;

typedef struct {
    int current_round;
    int max_rounds;
    float time_remaining;
    float next_event_in;
    bool round_active;
    int scores[4];
    float next_round_in;
} Match;

typedef struct {
    GameConfig config;
    GameWorld world;
    GameState state;
    Match match;
} Game;