#include <stdlib.h>

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

void create_scene(GameWorld* world) {
    int p_idx = world->entity_count++;

    world->transform[p_idx].position = (Vector3){ 0.0f, -0.5f, 0.0f };
    world->transform[p_idx].orientation = (Quaternion) {0.0f, 0.0f, 0.0f, 1.0f};

    world->rendering[p_idx].model = LoadModelFromMesh(GenMeshCube(20.0f, 1.0f, 20.0f));
    world->rendering[p_idx].material_normal = LoadMaterialDefault();
    
    world->physics_prop[p_idx].inverse_mass = 0.0f;
    world->collision[p_idx].type = SHAPE_CUBE;
    world->collision[p_idx].params.cube_extents = (Vector3){ 10.0f, 0.5f, 10.0f };

    int j_idx = world->entity_count++;

    world->transform[j_idx].position = (Vector3){ 0.0f, 10.0f, 0.0f };
    world->transform[j_idx].orientation = (Quaternion) {0.0f, 0.0f, 0.0f, 1.0f};

    world->rendering[j_idx].model = LoadModelFromMesh(GenMeshCube(2.0f, 2.0f, 2.0f));
    
    world->physics_prop[j_idx].mass = 10.0f;
    world->physics_prop[j_idx].inverse_mass = 1.0f / 10.0f;
    world->collision[j_idx].type = SHAPE_CUBE;
    world->collision[j_idx].params.cube_extents = (Vector3){ 1.0f, 1.0f, 1.0f };
    
    world->player_logic[j_idx].current_state = STATE_NEUTRAL;
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