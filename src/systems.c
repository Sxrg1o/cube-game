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

        world->physics_state[i].inverse_inertia_tensor_world = world_inv_it;

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

static bool check_sphere_sphere(GameWorld* world, int i, int j, Contact* contact) {
    bool collides;
    Vector3 sphere1_pos = world->transform[i].position;
    Vector3 sphere2_pos = world->transform[j].position;
    Vector3 delta = Vector3Subtract(sphere1_pos, sphere2_pos);
    float distance = Vector3Length(delta);
    float sphere1_r = world->collision[i].params.sphere_radius;
    float sphere2_r = world->collision[j].params.sphere_radius;
    collides = sphere1_r + sphere2_r > distance;

    if(collides) {
        contact->a_idx = i;
        contact->b_idx = j;
        if(distance < 0.0001f) delta = (Vector3) {0.0f, 1.0f, 0.0f};
        contact->normal = Vector3Normalize(delta);
        contact->penetration = sphere1_r + sphere2_r - distance;
        contact->impact_point = Vector3Add(sphere2_pos, Vector3Scale(contact->normal, sphere2_r));
    }
    return collides;
}

static bool check_plane_sphere(GameWorld* world, int plane_idx, int sph_idx, Contact* contact) {
    bool collides;
    Vector3 plane_pos = world->transform[plane_idx].position;
    Quaternion plane_rot = world->transform[plane_idx].orientation;
    Vector2 plane_size = world->collision[plane_idx].params.plane_extents;
    float half_width = plane_size.x * 0.5f;
    float half_depth = plane_size.y * 0.5f; 
    Vector3 sphere_pos = world->transform[sph_idx].position;
    float sphere_r = world->collision[sph_idx].params.sphere_radius;
    Vector3 rel_pos = Vector3Subtract(sphere_pos, plane_pos);
    Vector3 local_pos = Vector3RotateByQuaternion(rel_pos, QuaternionInvert(plane_rot));

    Vector3 closest_local_point = {
        Clamp(local_pos.x, -half_width, half_width),
        0.0f, 
        Clamp(local_pos.z, -half_depth, half_depth)
    };

    Vector3 local_delta = Vector3Subtract(local_pos, closest_local_point);
    float distance = Vector3Length(local_delta);
    collides = distance < sphere_r;

    if (collides) {
        contact->a_idx = plane_idx;
        contact->b_idx = sph_idx;

        Vector3 local_normal;

        if (distance < 0.0001f) {
            // TODO: Fix this, check local_delta smallest comp and use it as normal
            local_normal = (Vector3){0.0f, 1.0f, 0.0f};
            if (local_pos.y < 0) local_normal.y = -1.0f;
            contact->penetration = sphere_r; 
        } else {
            local_normal = Vector3Normalize(local_delta);
            contact->penetration = sphere_r - distance;
        }

        contact->normal = Vector3RotateByQuaternion(local_normal, plane_rot);
        
        Vector3 closest_world_point = Vector3Add(Vector3RotateByQuaternion(closest_local_point, plane_rot), plane_pos);
        contact->impact_point = closest_world_point;
    }

    return collides;
}

static bool check_plane_box(GameWorld* world, int i, int j, Contact* contact) {
    return false;
}

static bool check_box_box(GameWorld* world, int i, int j, Contact* contact) {
    return false;
}

static bool check_box_sphere(GameWorld* world, int box_idx, int sph_idx, Contact* contact) {
    bool collides;
    Vector3 box_pos = world->transform[box_idx].position;
    Quaternion box_rot = world->transform[box_idx].orientation;
    Vector3 box_half_ext = world->collision[box_idx].params.cube_extents;
    Vector3 sphere_pos = world->transform[sph_idx].position;
    float sphere_r = world->collision[sph_idx].params.sphere_radius;

    Vector3 rel_pos = Vector3Subtract(sphere_pos, box_pos);
    Vector3 local_pos = Vector3RotateByQuaternion(rel_pos, QuaternionInvert(box_rot));

    Vector3 closest_local_point = {
        Clamp(local_pos.x, -box_half_ext.x, box_half_ext.x),
        Clamp(local_pos.y, -box_half_ext.y, box_half_ext.y),
        Clamp(local_pos.z, -box_half_ext.z, box_half_ext.z)
    };

    Vector3 local_delta = Vector3Subtract(local_pos, closest_local_point);
    float distance = Vector3Length(local_delta);
    collides = distance < sphere_r;

    if (collides) {
        contact->a_idx = box_idx;
        contact->b_idx = sph_idx;

        Vector3 local_normal;
        
        if (distance < 0.0001f) {
            local_normal = (Vector3){0, 1, 0};
            contact->penetration = sphere_r + Vector3Length(closest_local_point);
        } else {
            local_normal = Vector3Normalize(local_delta);
            contact->penetration = sphere_r - distance;
        }

        contact->normal = Vector3RotateByQuaternion(local_normal, box_rot);
        Vector3 closest_world_point = Vector3Add(Vector3RotateByQuaternion(closest_local_point, box_rot), box_pos);
        contact->impact_point = closest_world_point;
    }

    return collides;
}

static CollisionFunc dispatch_table[3][3] = {
    {check_box_box, check_box_sphere, 0},
    {0, check_sphere_sphere, 0},
    {check_plane_box, check_plane_sphere, 0}
};

bool dispatch_collision(GameWorld* world, int i, int j, Contact* c) {
    int tA = world->collision[i].type;
    int tB = world->collision[j].type;

    if (dispatch_table[tA][tB]) {
        return dispatch_table[tA][tB](world, i, j, c);
    }
    
    if (dispatch_table[tB][tA]) {
        bool hit = dispatch_table[tB][tA](world, j, i, c);
        if (hit) {
            c->normal = Vector3Negate(c->normal); 
        }
        return hit;
    }
    
    return false;
}

static void solve_contact(GameWorld* world, Contact* contacts, int contact_count) {
    const float slop = 0.01f;
    const float percent = 0.5f;
    for(int i = 0; i < contact_count; i++) {
        Vector3 r_a = Vector3Subtract(contacts[i].impact_point, world->transform[contacts[i].a_idx].position);
        Vector3 r_b = Vector3Subtract(contacts[i].impact_point, world->transform[contacts[i].b_idx].position);
        Vector3 vel_a = Vector3Add(world->physics_state[contacts[i].a_idx].linear_velocity, 
                        Vector3CrossProduct(world->physics_state[contacts[i].a_idx].angular_velocity, r_a));
        Vector3 vel_b = Vector3Add(world->physics_state[contacts[i].b_idx].linear_velocity, 
                        Vector3CrossProduct(world->physics_state[contacts[i].b_idx].angular_velocity, r_b));
        Vector3 vel_rel = Vector3Subtract(vel_b, vel_a);
        float vel_imp = Vector3DotProduct(vel_rel, contacts[i].normal);

        if(vel_imp > 0) continue;

        float e = world->physics_prop[contacts[i].a_idx].restitution * world->physics_prop[contacts[i].b_idx].restitution;
        float num = (-1) * (1 + e) * vel_imp;
        
        Vector3 utorq_a = Vector3CrossProduct(r_a, contacts[i].normal);
        Vector3 uiner_a = Vector3TransformRotate(utorq_a, world->physics_state[contacts[i].a_idx].inverse_inertia_tensor_world);
        float ang_a = Vector3DotProduct(Vector3CrossProduct(uiner_a, r_a), contacts[i].normal);
        Vector3 utorq_b = Vector3CrossProduct(r_b, contacts[i].normal);
        Vector3 uiner_b = Vector3TransformRotate(utorq_b, world->physics_state[contacts[i].b_idx].inverse_inertia_tensor_world);
        float ang_b = Vector3DotProduct(Vector3CrossProduct(uiner_b, r_b), contacts[i].normal);
        float den = ang_a + ang_b + world->physics_prop[contacts[i].a_idx].inverse_mass
                    + world->physics_prop[contacts[i].b_idx].inverse_mass;
        float impulse = num / den;
        
        Vector3 impulse_vector = Vector3Scale(contacts[i].normal, impulse);
        world->physics_state[contacts[i].a_idx].linear_velocity = Vector3Subtract(world->physics_state[contacts[i].a_idx].linear_velocity, 
            Vector3Scale(impulse_vector, world->physics_prop[contacts[i].a_idx].inverse_mass));
        world->physics_state[contacts[i].b_idx].linear_velocity = Vector3Add(world->physics_state[contacts[i].b_idx].linear_velocity, 
            Vector3Scale(impulse_vector, world->physics_prop[contacts[i].b_idx].inverse_mass));
        
        Vector3 torq_a = Vector3CrossProduct(r_a, impulse_vector);
        world->physics_state[contacts[i].a_idx].angular_velocity = Vector3Subtract(world->physics_state[contacts[i].a_idx].angular_velocity, 
            Vector3TransformRotate(torq_a, world->physics_state[contacts[i].a_idx].inverse_inertia_tensor_world));
        Vector3 torq_b = Vector3CrossProduct(r_b, impulse_vector);
        world->physics_state[contacts[i].b_idx].angular_velocity = Vector3Add(world->physics_state[contacts[i].b_idx].angular_velocity,
            Vector3TransformRotate(torq_b, world->physics_state[contacts[i].b_idx].inverse_inertia_tensor_world));

        // TODO: Friction

        if(contacts[i].penetration > slop) {
            float scl = (contacts[i].penetration / (world->physics_prop[contacts[i].a_idx].inverse_mass + world->physics_prop[contacts[i].b_idx].inverse_mass));
            scl *= percent;
            Vector3 correction = Vector3Scale(contacts[i].normal, scl);
            world->transform[contacts[i].a_idx].position = Vector3Subtract(world->transform[contacts[i].a_idx].position, 
                Vector3Scale(correction, world->physics_prop[contacts[i].a_idx].inverse_mass));
            world->transform[contacts[i].b_idx].position = Vector3Add(world->transform[contacts[i].b_idx].position,
                Vector3Scale(correction, world->physics_prop[contacts[i].b_idx].inverse_mass));
        }
    }
}

void detect_collisions(GameWorld* world) {
    Contact contacts[100];
    int contact_count = 0;
    // For now, O(n2) but later implement Dynamic AABB tree - BVH (https://robotic.tistory.com/11)
    for(int i = 0; i < world->entity_count; i++) {
        for(int j = i + 1; j < world->entity_count; j++) {
            if(world->physics_prop[i].inverse_mass == 0.0f && 
               world->physics_prop[j].inverse_mass == 0.0f) {
                continue;
            }

            Contact c;
            if(dispatch_collision(world, i, j, &c) && contact_count < 100) {
                contacts[contact_count++] = c;
            }
        }
    }
    
    for(int k = 0; k < 5; k++) solve_contact(world, contacts, contact_count);
    
}

void update_render(GameWorld* world) {
    for (int i = 0; i < world->entity_count; i++) {
        Matrix orientation = QuaternionToMatrix(world->transform[i].orientation);
        Matrix translation = MatrixTranslate(world->transform[i].position.x, 
                                            world->transform[i].position.y,
                                            world->transform[i].position.z);
        Matrix res = MatrixMultiply(orientation, translation);
        world->rendering[i].model.transform = res;
        Color col = world->rendering[i].model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color;
        DrawModel(world->rendering[i].model, (Vector3){0.0f, 0.0f, 0.0f}, 1.0f, col);
        DrawModelWires(world->rendering[i].model, (Vector3){0.0f, 0.0f, 0.0f}, 1.0f, BLACK);
    }
}