#include <raylib.h>
#include <raymath.h>
#include <math.h>
#include <float.h>

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

static int clip_poly(Vector3* in_poly, int in_count, Vector3* out_poly, int axis, float sign, float limit) {
    int out_count = 0;

    for (int i = 0; i < in_count; i++) {
        int prev_idx = (i - 1 + in_count) % in_count;
        Vector3 curr = in_poly[i];
        Vector3 prev = in_poly[prev_idx];
        float curr_val = (axis == 0) ? curr.x : ((axis == 1) ? curr.y : curr.z);
        float prev_val = (axis == 0) ? prev.x : ((axis == 1) ? prev.y : prev.z);
        bool curr_inside = (curr_val * sign) <= limit;
        bool prev_inside = (prev_val * sign) <= limit;

        if (curr_inside) {
            if (!prev_inside) {
                float t = (limit - (prev_val * sign)) / ((curr_val * sign) - (prev_val * sign));
                t = (limit * sign - prev_val) / (curr_val - prev_val);

                Vector3 diff = Vector3Subtract(curr, prev);
                out_poly[out_count] = Vector3Add(prev, Vector3Scale(diff, t));
                out_count++;
            }
            out_poly[out_count] = curr;
            out_count++;
        } else if (prev_inside) {
            float t = (limit * sign - prev_val) / (curr_val - prev_val);
            
            Vector3 diff = Vector3Subtract(curr, prev);
            out_poly[out_count] = Vector3Add(prev, Vector3Scale(diff, t));
            out_count++;
        }
    }
    return out_count;
}

static void compute_contact_points(GameWorld* world, int i, int j, Contact* contact, 
                                   int axis_idx_a, int axis_idx_b, Vector3* list, 
                                   Matrix rot_a, Matrix rot_b, Vector3 pos_a, float min_s) {
    float extent_inc = world->collision[j].params.cube_extents.z;
    float e_u = world->collision[j].params.cube_extents.x;
    float e_v = world->collision[j].params.cube_extents.y;
    int u_idx = 3, v_idx = 4;

    if(axis_idx_b % 3 == 0) {
        extent_inc = world->collision[j].params.cube_extents.x;
        e_u = world->collision[j].params.cube_extents.y;
        e_v = world->collision[j].params.cube_extents.z;
        u_idx = 4; v_idx = 5;
    } else if(axis_idx_b % 3 == 1) {
        extent_inc = world->collision[j].params.cube_extents.y;
        e_u = world->collision[j].params.cube_extents.x;
        e_v = world->collision[j].params.cube_extents.z;
        u_idx = 3; v_idx = 5;
    }

    Vector3 inc_normal = list[axis_idx_b];
    Vector3 c_face = Vector3Add(world->transform[j].position, Vector3Scale(inc_normal, extent_inc));
    Vector3 vector_u = Vector3Scale(list[u_idx], e_u);
    Vector3 vector_v = Vector3Scale(list[v_idx], e_v);

    Vector3 points[4];
    points[0] = Vector3Add(c_face, Vector3Add(vector_u, vector_v));
    points[1] = Vector3Add(c_face, Vector3Subtract(vector_u, vector_v));
    points[2] = Vector3Subtract(c_face, Vector3Subtract(vector_u, vector_v));
    points[3] = Vector3Subtract(c_face, Vector3Add(vector_u, vector_v));

    Quaternion q_a = QuaternionFromMatrix(rot_a);
    Quaternion q_a_inv = QuaternionInvert(q_a);
    Vector3 ext_a = (Vector3)world->collision[i].params.cube_extents;

    Vector3 poly_buffer_1[16];
    Vector3 poly_buffer_2[16];
    int poly_count = 4;

    for(int p = 0; p < 4; p++) {
        Vector3 local_p = Vector3Subtract(points[p], pos_a);
        poly_buffer_1[p] = Vector3RotateByQuaternion(local_p, q_a_inv);
    }

    int ref_axis = axis_idx_a; 
    int clip_axis_1 = (ref_axis + 1) % 3;
    int clip_axis_2 = (ref_axis + 2) % 3;
    float limit_1 = (clip_axis_1 == 0) ? ext_a.x : ((clip_axis_1 == 1) ? ext_a.y : ext_a.z);
    float limit_2 = (clip_axis_2 == 0) ? ext_a.x : ((clip_axis_2 == 1) ? ext_a.y : ext_a.z);

    Vector3* in_poly = poly_buffer_1;
    Vector3* out_poly = poly_buffer_2;
    Vector3* temp;

    poly_count = clip_poly(in_poly, poly_count, out_poly, clip_axis_1, 1.0f, limit_1);
    temp = in_poly; in_poly = out_poly; out_poly = temp;
    poly_count = clip_poly(in_poly, poly_count, out_poly, clip_axis_1, -1.0f, limit_1);
    temp = in_poly; in_poly = out_poly; out_poly = temp;
    poly_count = clip_poly(in_poly, poly_count, out_poly, clip_axis_2, 1.0f, limit_2);
    temp = in_poly; in_poly = out_poly; out_poly = temp;
    poly_count = clip_poly(in_poly, poly_count, out_poly, clip_axis_2, -1.0f, limit_2);
    in_poly = out_poly;

    Vector3 local_normal = Vector3RotateByQuaternion(contact->normal, q_a_inv);
    float n_val = (ref_axis == 0) ? local_normal.x : ((ref_axis == 1) ? local_normal.y : local_normal.z);
    float ref_limit = (ref_axis == 0) ? ext_a.x : ((ref_axis == 1) ? ext_a.y : ext_a.z);
    
    Vector3 accum_point = {0};
    int valid_points = 0;

    for(int p = 0; p < poly_count; p++) {
        Vector3 curr = in_poly[p];
        float val_p = (ref_axis == 0) ? curr.x : ((ref_axis == 1) ? curr.y : curr.z);
        bool is_inside = false;

        if (n_val > 0) {
            if (val_p <= ref_limit + 0.05f) is_inside = true;
        } else {
            if (val_p >= -ref_limit - 0.05f) is_inside = true;
        }

        if (is_inside) {
            accum_point = Vector3Add(accum_point, curr);
            valid_points++;
        }
    }

    if (valid_points > 0) {
        Vector3 local_impact = Vector3Scale(accum_point, 1.0f / (float)valid_points);
        Vector3 world_impact = Vector3Add(Vector3RotateByQuaternion(local_impact, q_a), pos_a);
        contact->impact_point = world_impact;
        contact->penetration = min_s;
    } else {
        contact->impact_point = Vector3Add(world->transform[j].position, Vector3Scale(contact->normal, -min_s * 0.5f));
        contact->penetration = min_s;
    }
}

static bool check_box_box(GameWorld* world, int i, int j, Contact* contact) {
    Matrix orien_a = QuaternionToMatrix(world->transform[i].orientation);
    Matrix orien_b = QuaternionToMatrix(world->transform[j].orientation);
    Vector3 dist = Vector3Subtract(world->transform[j].position, world->transform[i].position);

    Vector3 list[15] = {
        {orien_a.m0, orien_a.m1, orien_a.m2}, {orien_a.m4, orien_a.m5, orien_a.m6}, {orien_a.m8, orien_a.m9, orien_a.m10},
        {orien_b.m0, orien_b.m1, orien_b.m2}, {orien_b.m4, orien_b.m5, orien_b.m6}, {orien_b.m8, orien_b.m9, orien_b.m10}
    };
    int k = 6;
    for(int n = 0; n < 3; n++) {
        for(int m = 3; m < 6; m++) {
            Vector3 cross = Vector3CrossProduct(list[n], list[m]);
            if (Vector3LengthSqr(cross) < 0.001f) {
                list[k++] = (Vector3){0}; 
            } else {
                list[k++] = Vector3Normalize(cross);
            }
        }
    }

    int min_idx_pn = -1;
    float min_s = FLT_MAX;
    float min_s_biased = FLT_MAX;

    for(k = 0; k < 15; k++) {
        if(Vector3LengthSqr(list[k]) < 0.001f) continue;

        Vector3 a_ext = (Vector3) world->collision[i].params.cube_extents;
        Vector3 b_ext = (Vector3) world->collision[j].params.cube_extents;

        float r_a = a_ext.x * fabsf(Vector3DotProduct(list[0], list[k])) +
                    a_ext.y * fabsf(Vector3DotProduct(list[1], list[k])) +
                    a_ext.z * fabsf(Vector3DotProduct(list[2], list[k]));
        float r_b = b_ext.x * fabsf(Vector3DotProduct(list[3], list[k])) +
                    b_ext.y * fabsf(Vector3DotProduct(list[4], list[k])) +
                    b_ext.z * fabsf(Vector3DotProduct(list[5], list[k]));
        
        float dist_proy = fabsf(Vector3DotProduct(dist, list[k]));
        float s = r_a + r_b - dist_proy;

        if(s < 0.0f) return false;
        float bias = (k >= 6) ? 1.05f : 1.0f;
        
        if((s * bias) < min_s_biased) {
            min_s_biased = s * bias;
            min_s = s;
            min_idx_pn = k;
        }
    }

    contact->a_idx = i;
    contact->b_idx = j;
    if(Vector3DotProduct(list[min_idx_pn], dist) < 0) Vector3Invert(list[min_idx_pn]);
    contact->normal = list[min_idx_pn];
    
    int axis_idx_a = 0;
    int axis_idx_b = 0;
    float max_dot_a = 0.0f;
    float min_dot_b = FLT_MAX;

    for(int n = 0; n < 3; n++) {
        float dot_a = fabsf(Vector3DotProduct(contact->normal, list[n]));
        if(dot_a > max_dot_a) { max_dot_a = dot_a; axis_idx_a = n; }

        float dot_b = Vector3DotProduct(contact->normal, list[n+3]);
        if(dot_b < min_dot_b) { min_dot_b = dot_b; axis_idx_b = n+3; }
    }

    compute_contact_points(world, i, j, contact, axis_idx_a, axis_idx_b, list, orien_a, orien_b, world->transform[i].position, min_s);

    return true;
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

void solve_velocity(GameWorld* world, Contact* contacts, int contact_count) {
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
        float num = (-1.0f) * (1.0f + e) * vel_imp;
        
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
    }
}

void solve_position(GameWorld* world, Contact* contacts, int contact_count) {
    const float slop = 0.01f;
    const float percent = 0.5f; 

    for(int i = 0; i < contact_count; i++) {
        if(contacts[i].penetration > slop) {
            float total_inverse_mass = world->physics_prop[contacts[i].a_idx].inverse_mass + world->physics_prop[contacts[i].b_idx].inverse_mass;
            
            if (total_inverse_mass <= 0.0f) continue;

            float scl = (contacts[i].penetration / total_inverse_mass) * percent;
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
    
    for(int k = 0; k < 10; k++) solve_velocity(world, contacts, contact_count);
    solve_position(world, contacts, contact_count);   
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