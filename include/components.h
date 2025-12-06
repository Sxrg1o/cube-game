#pragma once

#include <raylib.h>

typedef struct {
    Vector3 position;
    Quaternion orientation;
} TransformComponent;

typedef struct {
    Vector3 linear_velocity;
    Vector3 angular_velocity;
    Vector3 force_accumulator;
    Vector3 torque_accumulator;
    Matrix inverse_inertia_tensor_world;
} PhysicsStateComponent;

typedef struct {
    float mass;
    float inverse_mass;
    Matrix inertia_tensor;
    Matrix inverse_inertia_tensor;
    float restitution;
} PhysicsPropertiesComponent;

// Geometric differenciation (just basic for now i dont want to die)
typedef enum {
    SHAPE_CUBE,
    SHAPE_SPHERE,
    SHAPE_PLANE
} ShapeType;

typedef struct {
    ShapeType type;
    union {
        Vector3 cube_extents;
        float sphere_radius;
        Vector2 plane_extents;
    } params;
} CollisionShapeComponent;

typedef enum {
    STATE_NEUTRAL,
    STATE_POSITIVE,
    STATE_NEGATIVE,
    STATE_COOLDOWN
} FSMState;

typedef struct {
    FSMState current_state;
    float cooldown_timer;
} PlayerLogicComponent;

typedef struct {
    Model model;
    Material material_normal;
    Material material_dumb;
} RenderComponent;
