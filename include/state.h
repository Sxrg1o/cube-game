#pragma once

#include "components.h"
#include "aabb.h"

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
    int entity_count;
    int max_entities;
    DynamicTree collision_tree;

    TransformComponent* transform;
    PhysicsStateComponent* physics_state;
    PhysicsPropertiesComponent* physics_prop;
    CollisionShapeComponent* collision;
    PlayerLogicComponent* player_logic;
    RenderComponent* rendering;
} GameWorld;
