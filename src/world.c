#include <stdlib.h>
#include <raylib.h>
#include <raymath.h>

#include "state.h"
#include "world.h"

void init_world(GameWorld* world, int max_entities) {
    world->max_entities = max_entities;
    world->entity_count = 0;

    world->transform = (TransformComponent*)calloc(max_entities, sizeof(TransformComponent));
    world->physics_state = (PhysicsStateComponent*)calloc(max_entities, sizeof(PhysicsStateComponent));
    world->physics_prop = (PhysicsPropertiesComponent*)calloc(max_entities, sizeof(PhysicsPropertiesComponent));
    world->collision = (CollisionShapeComponent*)calloc(max_entities, sizeof(CollisionShapeComponent));
    world->rendering = (RenderComponent*)calloc(max_entities, sizeof(RenderComponent));
    world->player_logic = (PlayerLogicComponent*)calloc(max_entities, sizeof(PlayerLogicComponent));
}

Matrix calc_inertia_tensor(ShapeType shape_type, float mass, Vector3 dimentions) {
    Matrix it = MatrixIdentity();
    switch(shape_type) {
        case SHAPE_CUBE:
            float sc = (1.0f/12.0f) * mass;
            float w2 = dimentions.x * dimentions.x;
            float h2 = dimentions.y * dimentions.y;
            float d2 = dimentions.z * dimentions.z;

            it.m0 = sc * (h2 + d2);
            it.m5 = sc * (w2 + d2);
            it.m10 = sc * (w2 + h2);
            break;
        case SHAPE_SPHERE:
            sc = (2.0f/5.0f) * mass * dimentions.x * dimentions.x;
            it.m0 = sc;
            it.m5 = sc;
            it.m10 = sc;
            break;
        default:
            break;
    }

    return it;
}

int create_entity(GameWorld* world, EntityDesc desc) {
    if(world->entity_count >= world->max_entities) return -1;
    
    int idx = world->entity_count++;

    world->transform[idx].position = desc.position;
    world->transform[idx].orientation = desc.orientation;
    world->physics_prop[idx].mass = desc.mass;
    world->physics_prop[idx].restitution = desc.restitution;

    if(desc.mass > 0.0f) {
        world->physics_prop[idx].inverse_mass = 1.0f / desc.mass;
        world->physics_prop[idx].inertia_tensor = calc_inertia_tensor(desc.shape_type, desc.mass, desc.dimentions);
        world->physics_prop[idx].inverse_inertia_tensor = MatrixInvert(world->physics_prop[idx].inertia_tensor);
    } else {
        world->physics_prop[idx].inverse_mass = 0.0f;
        world->physics_prop[idx].inertia_tensor = MatrixIdentity();
    }

    Mesh mesh = { 0 };
    
    switch (desc.shape_type) {
        case SHAPE_CUBE:
            mesh = GenMeshCube(desc.dimentions.x, desc.dimentions.y, desc.dimentions.z);
            world->collision[idx].type = SHAPE_CUBE;
            world->collision[idx].params.cube_extents = Vector3Scale(desc.dimentions, 0.5f);
            break;
        case SHAPE_SPHERE:
            mesh = GenMeshSphere(desc.dimentions.x, 16, 16);
            world->collision[idx].type = SHAPE_SPHERE;
            world->collision[idx].params.sphere_radius = desc.dimentions.x;
            world->physics_state[idx].linear_velocity = (Vector3){0.0f, 0.0f, -13.0f};
            break;
        case SHAPE_PLANE:
            mesh = GenMeshPlane(desc.dimentions.x, desc.dimentions.z, 1, 1);
            world->collision[idx].type = SHAPE_PLANE;
            world->collision[idx].params.plane_extents = (Vector2) {desc.dimentions.x, desc.dimentions.z};
            break;
    }

    world->rendering[idx].model = LoadModelFromMesh(mesh);
    world->rendering[idx].model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = desc.color;

    if (desc.is_player) {
        world->player_logic[idx].current_state = STATE_NEUTRAL;
    }

    return idx;
}

void create_scene(GameWorld* world) {
    // Floor
    EntityDesc ground = {0};
    ground.position = (Vector3){ 0.0f, -0.5f, 0.0f };
    ground.orientation = (Quaternion){ 0.0f, 0.0f, 0.0f, 1.0f };
    ground.mass = 0.0f;
    ground.restitution = 0.9f;
    ground.shape_type = SHAPE_CUBE;
    ground.dimentions = (Vector3){ 20.0f, 1.0f, 20.0f };
    ground.color = GRAY;
    ground.is_player = false;

    create_entity(world, ground);

    // Player
    EntityDesc player = {0};
    player.position = (Vector3){ 0.0f, 10.0f, 0.0f };
    player.orientation = (Quaternion){ 0.0f, 0.0f, 0.0f, 1.0f };
    player.mass = 10.0f;
    player.restitution = 0.3f;
    player.shape_type = SHAPE_CUBE;
    player.dimentions = (Vector3){ 2.0f, 2.0f, 2.0f };
    player.color = RED;
    player.is_player = true;

    create_entity(world, player);
    
    // Sphere
    EntityDesc ball = {0};
    ball.position = (Vector3){ 5.0f, 10.0f, 0.0f };
    ball.orientation = (Quaternion){ 0.0f, 0.0f, 0.0f, 1.0f };
    ball.mass = 5.0f;
    ball.restitution = 0.7f;
    ball.shape_type = SHAPE_SPHERE;
    ball.dimentions = (Vector3){ 1.0f, 0.0f, 0.0f };
    ball.color = BLUE;
    ball.is_player = false;
    create_entity(world, ball);
    
}

void free_world(GameWorld* world) {
    if(!world) return;

    for (int i = 0; i < world->entity_count; i++) {
        UnloadModel(world->rendering[i].model); 
    }

    free(world->transform);
    free(world->physics_prop);
    free(world->physics_state);
    free(world->collision);
    free(world->rendering);
    free(world->player_logic);
}