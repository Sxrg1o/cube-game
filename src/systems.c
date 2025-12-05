#include <raylib.h>
#include <raymath.h>

#include "systems.h"

static const Vector3 GRAVITY = {0.0f, -9.81f, 0.0f};

static Vector3 Vector3TransformRotate(Vector3 v, Matrix mat) {
    Vector3 result;
    result.x = v.x * mat.m0 + v.y * mat.m4 + v.z * mat.m8;
    result.y = v.x * mat.m1 + v.y * mat.m5 + v.z * mat.m9;
    result.z = v.x * mat.m2 + v.y * mat.m6 + v.z * mat.m10;
    return result;
}

void update_physics(GameWorld* world, float delta_time) {
    for(int i = 0; i < world->entity_count; i++) {
        if(world->physics_prop[i].inverse_mass == 0.0f) continue;

        Vector3 gravity_force = Vector3Scale(GRAVITY, world->physics_prop[i].mass);
        world->physics_state[i].force_accumulator = Vector3Add(world->physics_state[i].force_accumulator, gravity_force);
        Vector3 lin_acc = Vector3Scale(world->physics_state[i].force_accumulator, world->physics_prop[i].inverse_mass);
        
        world->physics_state[i].linear_velocity = Vector3Add(
            world->physics_state[i].linear_velocity,
            Vector3Scale(lin_acc, delta_time));

        world->physics_state[i].linear_velocity = Vector3Scale(
            world->physics_state[i].linear_velocity, 0.98f);
        
        world->transform[i].position = Vector3Add(
            world->transform[i].position, 
            Vector3Scale(world->physics_state[i].linear_velocity, delta_time)
        );

        Matrix rot_matrix = QuaternionToMatrix(world->transform[i].orientation);

        Matrix world_inv_it = MatrixMultiply(rot_matrix, 
            MatrixMultiply(world->physics_prop[i].inverse_inertia_tensor, MatrixTranspose(rot_matrix)));

        Vector3 ang_acc = Vector3TransformRotate(world->physics_state[i].torque_accumulator, world_inv_it);
        
        world->physics_state[i].angular_velocity = Vector3Add(
            world->physics_state[i].angular_velocity,
            Vector3Scale(ang_acc, delta_time));

        world->physics_state[i].angular_velocity = Vector3Scale(
            world->physics_state[i].angular_velocity, 0.98f);

        Quaternion temp = (Quaternion) {world->physics_state[i].angular_velocity.x,
                                        world->physics_state[i].angular_velocity.y,
                                        world->physics_state[i].angular_velocity.z,
                                        0.0f};
        temp = QuaternionScale(QuaternionMultiply(temp, world->transform[i].orientation), 0.5f * delta_time);
        world->transform[i].orientation = QuaternionAdd(world->transform[i].orientation, temp);

        world->transform[i].orientation = QuaternionNormalize(world->transform[i].orientation);

        world->physics_state[i].force_accumulator = Vector3Zero();
        world->physics_state[i].torque_accumulator = Vector3Zero();
    }
}

void update_render(GameWorld* world) {
    for (int i = 0; i < world->entity_count; i++) {
        Matrix orientation = QuaternionToMatrix(world->transform[i].orientation);
        Matrix translation = MatrixTranslate(world->transform[i].position.x, 
                                            world->transform[i].position.y,
                                            world->transform[i].position.z);
        Matrix res = MatrixMultiply(orientation, translation);
        world->rendering[i].model.transform = res;
        Color col = (i == 0) ? GRAY : RED;
        DrawModel(world->rendering[i].model, (Vector3){0.0f, 0.0f, 0.0f}, 1.0f, col);
        DrawModelWires(world->rendering[i].model, (Vector3){0.0f, 0.0f, 0.0f}, 1.0f, BLACK);
    }
}