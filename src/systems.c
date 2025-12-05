#include <raymath.h>

#include "systems.h"

static const Vector3 GRAVITY = {0.0f, 9.81f, 0.0f};

void update_physics(GameWorld* world, float delta_time) {
    for(int i = 0; i < world->entity_count; i++) {
        if(world->physics_prop[i].inverse_mass == 0.0f) continue;

        Vector3 acc = Vector3Scale(world->physics_state[i].force_accumulator ,world->physics_prop[i].inverse_mass);
        acc.y -= GRAVITY.y;

        world->physics_state[i].linear_velocity = Vector3Add(
            world->physics_state[i].linear_velocity,
            Vector3Scale(acc, delta_time)
        );

        world->physics_state[i].linear_velocity = Vector3Scale(
            world->physics_state[i].linear_velocity, 0.98f);
        
        world->transform[i].position = Vector3Add(
            world->transform[i].position, 
            Vector3Scale(world->physics_state[i].linear_velocity, delta_time)
        );

        // TODO: Orientation... 

    }
}

void update_render(GameWorld* world) {
    for (int i = 0; i < world->entity_count; i++) {
        Matrix orientation = QuaternionToMatrix(world->transform[i].orientation);
        
    }
}