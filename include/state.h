#pragma once

#include "components.h"

// Just for creating entities
typedef struct {
    Vector3 position;
    Quaternion orientation;
    float mass;
    ShapeType shape_type;
    Vector3 dimensions;
    Color color;
    bool is_player;
} EntityDesc;

typedef struct {
    int entity_count;
    int max_entities;

    TransformComponent* transform;
    PhysicsStateComponent* physics_state;
    PhysicsPropertiesComponent* physics_prop;
    CollisionShapeComponent* collision;
    PlayerLogicComponent* player_logic;
    RenderComponent* rendering;
} GameWorld;
