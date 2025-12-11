#include <raylib.h>
#include <raymath.h>
#include <float.h>
#include <math.h>

#include "state.h"
#include "collisions.h"
#include "systems.h"

#define SAT_EDGE_BIAS 1.15f

static int check_sphere_sphere(GameWorld* world, int i, int j, Contact* contact) {
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
        return 1;
    }
    return 0;
}

static int check_plane_sphere(GameWorld* world, int plane_idx, int sph_idx, Contact* contact) {
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
        contact->acc_normal_impulse = 0.0f;
        contact->acc_tangent_impulse_1 = 0.0f;
        contact->acc_tangent_impulse_2 = 0.0f;
        return 1;
    }

    return 0;
}

static int check_plane_box(GameWorld* world, int plane_idx, int box_idx, Contact* contacts) {
    Vector3 plane_pos = world->transform[plane_idx].position;
    Quaternion plane_rot = world->transform[plane_idx].orientation;
    Vector2 plane_size = world->collision[plane_idx].params.plane_extents;
    float plane_half_x = plane_size.x * 0.5f;
    float plane_half_z = plane_size.y * 0.5f;
    Vector3 plane_normal = Vector3RotateByQuaternion((Vector3){0, 1, 0}, plane_rot);
    Vector3 box_pos = world->transform[box_idx].position;
    Quaternion box_rot = world->transform[box_idx].orientation;
    Vector3 box_extents = world->collision[box_idx].params.cube_extents;
    int contacts_added = 0;

    for (int i = 0; i < 8; i++) {
        float sx = (i & 1) ? 1.0f : -1.0f;
        float sy = (i & 2) ? 1.0f : -1.0f;
        float sz = (i & 4) ? 1.0f : -1.0f;

        Vector3 vertex_local = { 
            box_extents.x * sx, 
            box_extents.y * sy, 
            box_extents.z * sz 
        };

        Vector3 vertex_world = Vector3Add(box_pos, Vector3RotateByQuaternion(vertex_local, box_rot));
        Vector3 rel_pos = Vector3Subtract(vertex_world, plane_pos);
        Vector3 point_in_plane_space = Vector3RotateByQuaternion(rel_pos, QuaternionInvert(plane_rot));

        if (point_in_plane_space.y <= 0.0f) {
            bool inside_x = (point_in_plane_space.x >= -plane_half_x && point_in_plane_space.x <= plane_half_x);
            bool inside_z = (point_in_plane_space.z >= -plane_half_z && point_in_plane_space.z <= plane_half_z);

            if (inside_x && inside_z) {
                if (contacts_added < 4) {
                    Contact* c = &contacts[contacts_added++];
                    
                    c->a_idx = plane_idx;
                    c->b_idx = box_idx;
                    c->normal = plane_normal;
                    c->penetration = -point_in_plane_space.y;
                    c->impact_point = vertex_world;
                    c->acc_normal_impulse = 0.0f;
                    c->acc_tangent_impulse_1 = 0.0f;
                    c->acc_tangent_impulse_2 = 0.0f;
                }
            }
        }
    }

    return contacts_added;
}

static Vector3 get_support_point(GameWorld* world, int idx, Vector3 dir) {
    Quaternion q = world->transform[idx].orientation;
    Quaternion q_inv = QuaternionInvert(q);
    Vector3 d_local = Vector3RotateByQuaternion(dir, q_inv);
    
    Vector3 extents = world->collision[idx].params.cube_extents;
    
    Vector3 support_local = {
        (d_local.x > 0) ? extents.x : -extents.x,
        (d_local.y > 0) ? extents.y : -extents.y,
        (d_local.z > 0) ? extents.z : -extents.z
    };
    
    return Vector3Add(world->transform[idx].position, Vector3RotateByQuaternion(support_local, q));
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

static int compute_contact_points(GameWorld* world, int i, int j, Contact* contacts_out, 
                                   int axis_idx_a, int axis_idx_b, Vector3* list, 
                                   Matrix rot_a, Matrix rot_b, Vector3 pos_a, float min_s, Vector3 normal) {
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
    Vector3 ext_a = world->collision[i].params.cube_extents;

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

    Vector3 local_normal = Vector3RotateByQuaternion(normal, q_a_inv);
    float n_val = (ref_axis == 0) ? local_normal.x : ((ref_axis == 1) ? local_normal.y : local_normal.z);
    float ref_limit = (ref_axis == 0) ? ext_a.x : ((ref_axis == 1) ? ext_a.y : ext_a.z);
    
    int contacts_added = 0;

    for(int p = 0; p < poly_count && contacts_added < 4; p++) {
        Vector3 curr = in_poly[p];
        float val_p = (ref_axis == 0) ? curr.x : ((ref_axis == 1) ? curr.y : curr.z);
        bool is_inside = false;

        if (n_val > 0) {
            if (val_p <= ref_limit + 0.03f) is_inside = true;
        } else {
            if (val_p >= -ref_limit - 0.03f) is_inside = true;
        }

        if (is_inside) {
            Vector3 world_impact = Vector3Add(Vector3RotateByQuaternion(curr, q_a), pos_a);
            Contact* c = &contacts_out[contacts_added];
            c->a_idx = i;
            c->b_idx = j;
            c->normal = normal;
            c->impact_point = world_impact;
            c->penetration = min_s;
            c->acc_normal_impulse = 0.0f;
            c->acc_tangent_impulse_1 = 0.0f; 
            c->acc_tangent_impulse_2 = 0.0f;
            contacts_added++;
        }
    }

    if (contacts_added == 0) {
        Vector3 deepest_point = get_support_point(world, j, Vector3Negate(normal));
        
        Contact* c = &contacts_out[0];
        c->a_idx = i;
        c->b_idx = j;
        c->normal = normal;
        c->impact_point = deepest_point;
        c->penetration = min_s;
        c->acc_normal_impulse = 0.0f;
        c->acc_tangent_impulse_1 = 0.0f;
        c->acc_tangent_impulse_2 = 0.0f;
        
        return 1;
    }

    return contacts_added;
}

static int check_box_box(GameWorld* world, int i, int j, Contact* contacts) {
    TransformComponent* tA = &world->transform[i];
    TransformComponent* tB = &world->transform[j];
    
    Matrix matA = QuaternionToMatrix(tA->orientation);
    Matrix matB = QuaternionToMatrix(tB->orientation);
    
    Vector3 axes[15] = {
        {matA.m0, matA.m1, matA.m2}, {matA.m4, matA.m5, matA.m6}, {matA.m8, matA.m9, matA.m10},
        {matB.m0, matB.m1, matB.m2}, {matB.m4, matB.m5, matB.m6}, {matB.m8, matB.m9, matB.m10}
    };

    int k = 6;
    for (int a = 0; a < 3; a++) {
        for (int b = 3; b < 6; b++) {
            Vector3 cross = Vector3CrossProduct(axes[a], axes[b]);
            float lenSq = Vector3LengthSqr(cross);
            if (lenSq < 0.001f) {
                axes[k++] = (Vector3){0}; 
            } else {
                axes[k++] = Vector3Scale(cross, 1.0f / sqrtf(lenSq));
            }
        }
    }

    Vector3 dist = Vector3Subtract(tB->position, tA->position);
    Vector3 extA = world->collision[i].params.cube_extents;
    Vector3 extB = world->collision[j].params.cube_extents;

    float min_penetration_biased = FLT_MAX;
    int best_axis_idx = -1;

    for (int axis = 0; axis < 15; axis++) {
        if (Vector3LengthSqr(axes[axis]) < 0.001f) continue;

        float rA = extA.x * fabsf(Vector3DotProduct(axes[0], axes[axis])) +
                   extA.y * fabsf(Vector3DotProduct(axes[1], axes[axis])) +
                   extA.z * fabsf(Vector3DotProduct(axes[2], axes[axis]));
        
        float rB = extB.x * fabsf(Vector3DotProduct(axes[3], axes[axis])) +
                   extB.y * fabsf(Vector3DotProduct(axes[4], axes[axis])) +
                   extB.z * fabsf(Vector3DotProduct(axes[5], axes[axis]));

        float dist_proj = fabsf(Vector3DotProduct(dist, axes[axis]));
        float penetration = rA + rB - dist_proj;

        if (penetration < 0.0f) return 0;

        float biased_penetration = penetration;
        if (axis >= 6) {
            biased_penetration *= SAT_EDGE_BIAS;
        }

        if (biased_penetration < min_penetration_biased) {
            min_penetration_biased = biased_penetration;
            best_axis_idx = axis;
        }
    }

    if (best_axis_idx == -1) return 0;

    Vector3 final_normal = axes[best_axis_idx];
    if (Vector3DotProduct(final_normal, dist) < 0) {
        final_normal = Vector3Negate(final_normal);
    }

    float real_penetration = 0.0f;
    {
        float rA = extA.x * fabsf(Vector3DotProduct(axes[0], final_normal)) +
                   extA.y * fabsf(Vector3DotProduct(axes[1], final_normal)) +
                   extA.z * fabsf(Vector3DotProduct(axes[2], final_normal));
        float rB = extB.x * fabsf(Vector3DotProduct(axes[3], final_normal)) +
                   extB.y * fabsf(Vector3DotProduct(axes[4], final_normal)) +
                   extB.z * fabsf(Vector3DotProduct(axes[5], final_normal));
        float dist_proj = fabsf(Vector3DotProduct(dist, final_normal));
        real_penetration = rA + rB - dist_proj;
    }

    int axis_idx_a = 0;
    int axis_idx_b = 0;
    float max_dot_a = 0.0f;
    float min_dot_b = FLT_MAX;

    for(int n = 0; n < 3; n++) {
        float dot_a = fabsf(Vector3DotProduct(final_normal, axes[n]));
        if(dot_a > max_dot_a) { max_dot_a = dot_a; axis_idx_a = n; }

        float dot_b = Vector3DotProduct(final_normal, axes[n+3]);
        if(dot_b < min_dot_b) { min_dot_b = dot_b; axis_idx_b = n+3; }
    }

    return compute_contact_points(world, i, j, contacts, axis_idx_a, axis_idx_b, axes, matA, matB, tA->position, real_penetration, final_normal);
}

static int check_box_sphere(GameWorld* world, int box_idx, int sph_idx, Contact* contact) {
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
        Vector3 local_normal = {0};

        if (distance < 0.0001f) {
            float absX = fabsf(local_pos.x) - box_half_ext.x;
            float absY = fabsf(local_pos.y) - box_half_ext.y;
            float absZ = fabsf(local_pos.z) - box_half_ext.z;
            float max_axis = absX;
            int axis_idx = 0;

            if (absY > max_axis) { max_axis = absY; axis_idx = 1; }
            if (absZ > max_axis) { max_axis = absZ; axis_idx = 2; }

            if (axis_idx == 0) {
                local_normal = (Vector3){ (local_pos.x > 0) ? 1.0f : -1.0f, 0, 0 };
            } else if (axis_idx == 1) {
                local_normal = (Vector3){ 0, (local_pos.y > 0) ? 1.0f : -1.0f, 0 };
            } else {
                local_normal = (Vector3){ 0, 0, (local_pos.z > 0) ? 1.0f : -1.0f };
            }

            contact->penetration = sphere_r - max_axis; 
        } else {
            local_normal = Vector3Normalize(local_delta);
            contact->penetration = sphere_r - distance;
        }

        contact->normal = Vector3RotateByQuaternion(local_normal, box_rot);
        Vector3 closest_world_point = Vector3Add(Vector3RotateByQuaternion(closest_local_point, box_rot), box_pos);
        contact->impact_point = closest_world_point;
        contact->acc_normal_impulse = 0.0f;
        contact->acc_tangent_impulse_1 = 0.0f;
        contact->acc_tangent_impulse_2 = 0.0f;
        return 1;
    }

    return 0;
}

void solve_velocity(GameWorld* world, Contact* contacts, int contact_count) {
    for(int i = 0; i < contact_count; i++) {
        int idxA = contacts[i].a_idx;
        int idxB = contacts[i].b_idx;

        Vector3 r_a = Vector3Subtract(contacts[i].impact_point, world->transform[idxA].position);
        Vector3 r_b = Vector3Subtract(contacts[i].impact_point, world->transform[idxB].position);

        Vector3 vel_a = Vector3Add(world->physics_state[idxA].linear_velocity, 
                        Vector3CrossProduct(world->physics_state[idxA].angular_velocity, r_a));
        Vector3 vel_b = Vector3Add(world->physics_state[idxB].linear_velocity, 
                        Vector3CrossProduct(world->physics_state[idxB].angular_velocity, r_b));
        Vector3 vel_rel = Vector3Subtract(vel_b, vel_a);
        
        float vel_normal = Vector3DotProduct(vel_rel, contacts[i].normal);

        float normal_impulse_mag = 0.0f;

        if(vel_normal < 0.0f) {
            float e = world->physics_prop[idxA].restitution * world->physics_prop[idxB].restitution;
            if (fabsf(vel_normal) < 1.0f) e = 0.0f;

            float num = -(1.0f + e) * vel_normal;
            
            Vector3 ra_n = Vector3CrossProduct(r_a, contacts[i].normal);
            Vector3 rb_n = Vector3CrossProduct(r_b, contacts[i].normal);
            Vector3 term_a = Vector3TransformRotate(ra_n, world->physics_state[idxA].inverse_inertia_tensor_world);
            Vector3 term_b = Vector3TransformRotate(rb_n, world->physics_state[idxB].inverse_inertia_tensor_world);
            
            float den = world->physics_prop[idxA].inverse_mass + world->physics_prop[idxB].inverse_mass +
                        Vector3DotProduct(Vector3CrossProduct(term_a, r_a), contacts[i].normal) +
                        Vector3DotProduct(Vector3CrossProduct(term_b, r_b), contacts[i].normal);

            normal_impulse_mag = num / den;
            contacts[i].acc_normal_impulse = normal_impulse_mag;
            Vector3 impulse_vec = Vector3Scale(contacts[i].normal, normal_impulse_mag);

            world->physics_state[idxA].linear_velocity = Vector3Subtract(world->physics_state[idxA].linear_velocity, 
                Vector3Scale(impulse_vec, world->physics_prop[idxA].inverse_mass));
            world->physics_state[idxA].angular_velocity = Vector3Subtract(world->physics_state[idxA].angular_velocity, 
                Vector3TransformRotate(Vector3CrossProduct(r_a, impulse_vec), world->physics_state[idxA].inverse_inertia_tensor_world));
            
            world->physics_state[idxB].linear_velocity = Vector3Add(world->physics_state[idxB].linear_velocity, 
                Vector3Scale(impulse_vec, world->physics_prop[idxB].inverse_mass));
            world->physics_state[idxB].angular_velocity = Vector3Add(world->physics_state[idxB].angular_velocity, 
                Vector3TransformRotate(Vector3CrossProduct(r_b, impulse_vec), world->physics_state[idxB].inverse_inertia_tensor_world));
        }

        vel_a = Vector3Add(world->physics_state[idxA].linear_velocity, Vector3CrossProduct(world->physics_state[idxA].angular_velocity, r_a));
        vel_b = Vector3Add(world->physics_state[idxB].linear_velocity, Vector3CrossProduct(world->physics_state[idxB].angular_velocity, r_b));
        vel_rel = Vector3Subtract(vel_b, vel_a);

        Vector3 tangent = Vector3Subtract(vel_rel, Vector3Scale(contacts[i].normal, Vector3DotProduct(vel_rel, contacts[i].normal)));
        float tangent_len = Vector3Length(tangent);

        if (tangent_len > 0.001f) {
            tangent = Vector3Scale(tangent, 1.0f / tangent_len);
            
            Vector3 ra_t = Vector3CrossProduct(r_a, tangent);
            Vector3 rb_t = Vector3CrossProduct(r_b, tangent);
            Vector3 term_a_t = Vector3TransformRotate(ra_t, world->physics_state[idxA].inverse_inertia_tensor_world);
            Vector3 term_b_t = Vector3TransformRotate(rb_t, world->physics_state[idxB].inverse_inertia_tensor_world);
            
            float den_t = world->physics_prop[idxA].inverse_mass + world->physics_prop[idxB].inverse_mass +
                          Vector3DotProduct(Vector3CrossProduct(term_a_t, r_a), tangent) +
                          Vector3DotProduct(Vector3CrossProduct(term_b_t, r_b), tangent);
            
            float jt = -Vector3DotProduct(vel_rel, tangent) / den_t;

            float friction_coef = 0.4f;
            float max_friction = normal_impulse_mag * friction_coef;

            if (jt > max_friction) jt = max_friction;
            if (jt < -max_friction) jt = -max_friction;
            
            if (fabsf(jt) > 0.0001f) {
                Vector3 impulse_fric = Vector3Scale(tangent, jt);
                
                world->physics_state[idxA].linear_velocity = Vector3Subtract(world->physics_state[idxA].linear_velocity, 
                    Vector3Scale(impulse_fric, world->physics_prop[idxA].inverse_mass));
                world->physics_state[idxA].angular_velocity = Vector3Subtract(world->physics_state[idxA].angular_velocity, 
                    Vector3TransformRotate(Vector3CrossProduct(r_a, impulse_fric), world->physics_state[idxA].inverse_inertia_tensor_world));

                world->physics_state[idxB].linear_velocity = Vector3Add(world->physics_state[idxB].linear_velocity, 
                    Vector3Scale(impulse_fric, world->physics_prop[idxB].inverse_mass));
                world->physics_state[idxB].angular_velocity = Vector3Add(world->physics_state[idxB].angular_velocity, 
                    Vector3TransformRotate(Vector3CrossProduct(r_b, impulse_fric), world->physics_state[idxB].inverse_inertia_tensor_world));
            }
        }
    }
}

void solve_position(GameWorld* world, Contact* contacts, int contact_count) {
    const float slop = 0.01f;
    const float percent = 0.8f;

    for(int i = 0; i < contact_count; i++) {
        float penetration = contacts[i].penetration;

        if(penetration > slop) {
            float imA = world->physics_prop[contacts[i].a_idx].inverse_mass;
            float imB = world->physics_prop[contacts[i].b_idx].inverse_mass;
            float total_inverse_mass = imA + imB;
            
            if (total_inverse_mass <= 0.0f) continue;

            float correction_magnitude = (penetration - slop) / total_inverse_mass * percent;
            if (correction_magnitude > 0.2f) correction_magnitude = 0.2f;
            
            Vector3 correction = Vector3Scale(contacts[i].normal, correction_magnitude);
            
            world->transform[contacts[i].a_idx].position = Vector3Subtract(
                world->transform[contacts[i].a_idx].position, 
                Vector3Scale(correction, imA));
                
            world->transform[contacts[i].b_idx].position = Vector3Add(
                world->transform[contacts[i].b_idx].position,
                Vector3Scale(correction, imB));
        }
    }
}

static CollisionFunc dispatch_table[3][3] = {
    {check_box_box, check_box_sphere, 0},
    {0, check_sphere_sphere, 0},
    {check_plane_box, check_plane_sphere, 0}
};

int dispatch_collision(GameWorld* world, int i, int j, Contact* c) {
    int tA = world->collision[i].type;
    int tB = world->collision[j].type;

    if (dispatch_table[tA][tB]) {
        return dispatch_table[tA][tB](world, i, j, c);
    }
    
    if (dispatch_table[tB][tA]) {
        return dispatch_table[tB][tA](world, j, i, c);
    }
    
    return 0;
}
