#pragma once

#include "components.h"

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
    float player_size;
} GameConfig;

typedef struct {
    GameConfig config;
    GameWorld world;
    GameState state;
} Game;