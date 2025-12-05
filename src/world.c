#include <stdlib.h>
#include <raylib.h>
#include <raymath.h>

#include "state.h"
#include "world.h"

void init_world(GameWorld* world, int max_entities) {
    world->max_entities = max_entities;
    world->entity_count = 0;

    world->transform = (TransformComponent*)malloc(sizeof(TransformComponent) * max_entities);
    world->physics_state = (PhysicsStateComponent*)malloc(sizeof(PhysicsStateComponent) * max_entities);
    world->physics_prop = (PhysicsPropertiesComponent*)malloc(sizeof(PhysicsPropertiesComponent) * max_entities);
    world->collision = (CollisionShapeComponent*)malloc(sizeof(CollisionShapeComponent) * max_entities);
    world->rendering = (RenderComponent*)malloc(sizeof(RenderComponent) * max_entities);
    
    world->player_logic = (PlayerLogicComponent*)malloc(sizeof(PlayerLogicComponent) * max_entities);
}

Matrix calc_inertia_tensor(ShapeType shape_type, float mass, Vector3 dimensions) {
    Matrix it = MatrixIdentity();
    switch(shape_type) {
        case SHAPE_CUBE:
            float sc = (1.0f/12.0f) * mass;
            float w2 = dimensions.x * dimensions.x;
            float h2 = dimensions.y * dimensions.y;
            float d2 = dimensions.z * dimensions.z;

            it.m0 = sc * (h2 + d2);
            it.m5 = sc * (w2 + d2);
            it.m10 = sc * (w2 + h2);
            break;
        case SHAPE_SPHERE:
            sc = (2.0f/5.0f) * mass * dimensions.x * dimensions.x;
            it.m0 = sc;
            it.m5 = sc;
            it.m10 = sc;
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

    if(desc.mass > 0.0f) {
        world->physics_prop[idx].inverse_mass = 1.0f / desc.mass;
        world->physics_prop[idx].inertia_tensor = calc_inertia_tensor(desc.shape_type, desc.mass, desc.dimensions);
        world->physics_prop[idx].inverse_inertia_tensor = MatrixInvert(world->physics_prop[idx].inertia_tensor);
    } else {
        world->physics_prop[idx].inverse_mass = 0.0f;
        world->physics_prop[idx].inertia_tensor = MatrixIdentity();
    }

    Mesh mesh = { 0 };
    
    switch (desc.shape_type) {
        case SHAPE_CUBE:
            mesh = GenMeshCube(desc.dimensions.x, desc.dimensions.y, desc.dimensions.z);
            world->collision[idx].type = SHAPE_CUBE;
            world->collision[idx].params.cube_extents = Vector3Scale(desc.dimensions, 0.5f);
            break;
        case SHAPE_SPHERE:
            mesh = GenMeshSphere(desc.dimensions.x, 16, 16);
            world->collision[idx].type = SHAPE_SPHERE;
            world->collision[idx].params.sphere_radius = desc.dimensions.x;
            break;
    }

    world->rendering[idx].model = LoadModelFromMesh(mesh);
    world->rendering[idx].model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = desc.color;

    if (desc.is_player) {
        world->player_logic[idx].current_state = STATE_NEUTRAL;

        // JUST TESTING
        world->physics_state[idx].angular_velocity = (Vector3) {10.0f, 0.0f, 0.0f};
    }

    return idx;
}

void create_scene(GameWorld* world) {
    // Floor
    EntityDesc ground = {0};
    ground.position = (Vector3){ 0.0f, -0.5f, 0.0f };
    ground.orientation = (Quaternion){ 0.0f, 0.0f, 0.0f, 1.0f };
    ground.mass = 0.0f;
    ground.shape_type = SHAPE_CUBE;
    ground.dimensions = (Vector3){ 20.0f, 1.0f, 20.0f };
    ground.color = GRAY;
    ground.is_player = false;

    create_entity(world, ground);

    // Player
    EntityDesc player = {0};
    player.position = (Vector3){ 0.0f, 10.0f, 0.0f };
    player.orientation = (Quaternion){ 0.0f, 0.0f, 0.0f, 1.0f };
    player.mass = 10.0f;
    player.shape_type = SHAPE_CUBE;
    player.dimensions = (Vector3){ 2.0f, 2.0f, 2.0f };
    player.color = RED;
    player.is_player = true;

    create_entity(world, player);
    
    // Sphere
    EntityDesc ball = {0};
    ball.position = (Vector3){ 5.0f, 10.0f, 0.0f };
    ball.orientation = (Quaternion){ 0.0f, 0.0f, 0.0f, 1.0f };
    ball.mass = 5.0f;
    ball.shape_type = SHAPE_SPHERE;
    ball.dimensions = (Vector3){ 1.0f, 0.0f, 0.0f };
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