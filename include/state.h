#pragma once

#include "components.h"

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
