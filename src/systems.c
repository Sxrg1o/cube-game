#include <raylib.h>
#include <raymath.h>

#include "systems.h"
#include "collisions.h"
#include "gameplay.h"

static const Vector3 GRAVITY = {0.0f, -18.0f, 0.0f};

Vector3 Vector3TransformRotate(Vector3 v, Matrix mat) {
    Vector3 result;
    result.x = v.x * mat.m0 + v.y * mat.m4 + v.z * mat.m8;
    result.y = v.x * mat.m1 + v.y * mat.m5 + v.z * mat.m9;
    result.z = v.x * mat.m2 + v.y * mat.m6 + v.z * mat.m10;
    return result;
}

static void physics_step(GameWorld* world, float dt) {
    for(int i = 0; i < world->entity_count; i++) {
        if (!world->entity_active[i]) continue;
        if(world->physics_prop[i].inverse_mass == 0.0f) continue;

        Vector3 gravity_force = Vector3Scale(GRAVITY, world->physics_prop[i].mass);
        world->physics_state[i].force_accumulator = Vector3Add(world->physics_state[i].force_accumulator, gravity_force);

        Vector3 lin_acc = Vector3Scale(world->physics_state[i].force_accumulator, world->physics_prop[i].inverse_mass);
        world->physics_state[i].linear_velocity = Vector3Add(
            world->physics_state[i].linear_velocity,
            Vector3Scale(lin_acc, dt)
        );

        Matrix rot_matrix = QuaternionToMatrix(world->transform[i].orientation);
        Matrix world_inv_it = MatrixMultiply(rot_matrix, 
            MatrixMultiply(world->physics_prop[i].inverse_inertia_tensor, MatrixTranspose(rot_matrix)));
        world->physics_state[i].inverse_inertia_tensor_world = world_inv_it;

        Vector3 ang_acc = Vector3TransformRotate(world->physics_state[i].torque_accumulator, world_inv_it);
        world->physics_state[i].angular_velocity = Vector3Add(
            world->physics_state[i].angular_velocity,
            Vector3Scale(ang_acc, dt)
        );

        float damping_multiplier = 1.0f / (1.0f + DAMPING_FACTOR * dt);
        world->physics_state[i].linear_velocity = Vector3Scale(world->physics_state[i].linear_velocity, damping_multiplier);
        world->physics_state[i].angular_velocity = Vector3Scale(world->physics_state[i].angular_velocity, damping_multiplier);

        world->physics_state[i].force_accumulator = Vector3Zero();
        world->physics_state[i].torque_accumulator = Vector3Zero();
    }

    for(int i = 0; i < world->entity_count; i++) {
        if (!world->entity_active[i]) continue;
        if(world->physics_prop[i].inverse_mass == 0.0f) continue;

        world->transform[i].position = Vector3Add(
            world->transform[i].position, 
            Vector3Scale(world->physics_state[i].linear_velocity, dt)
        );

        Quaternion q = world->transform[i].orientation;
        Vector3 w = world->physics_state[i].angular_velocity;
        
        Quaternion q_vel = {
            w.x * q.w + w.y * q.z - w.z * q.y,
            w.y * q.w + w.z * q.x - w.x * q.z,
            w.z * q.w + w.x * q.y - w.y * q.x,
            -w.x * q.x - w.y * q.y - w.z * q.z
        };
        
        q = QuaternionAdd(q, QuaternionScale(q_vel, 0.5f * dt));
        world->transform[i].orientation = QuaternionNormalize(q);
    }

    detect_collisions(world); 
}

void update_physics(GameWorld* world, float delta_time) {
    float sub_dt = delta_time / (float)SUB_STEPS;
    
    for(int s = 0; s < SUB_STEPS; s++) {
        physics_step(world, sub_dt);
    }
}

void detect_collisions(GameWorld* world) {
    for (int i = 0; i < world->entity_count; i++) {
        if (!world->entity_active[i]) continue;
        world->physics_state[i].in_ground = false;
    }

    for (int i = 0; i < world->entity_count; i++) {
        if (!world->entity_active[i]) continue;
        int proxy_id = world->physics_state[i].broadphase_proxy;
        BoundingBox fat_aabb = get_aabb(world->transform[i].position, world->transform[i].orientation,
            world->collision[i]);

        if (proxy_id == -1) {
            world->physics_state[i].broadphase_proxy = insert_leaf(&world->collision_tree, i, fat_aabb);
        } else {
            BoundingBox current_node_aabb = world->collision_tree.nodes[proxy_id].aabb;
            if (!contains_box(current_node_aabb, fat_aabb)) {
                remove_leaf(&world->collision_tree, proxy_id);
                world->physics_state[i].broadphase_proxy = insert_leaf(&world->collision_tree, i, fat_aabb);
            }
        }
    }

    Contact contacts[1024];
    int contact_count = 0;
    int candidates[128];
    for (int i = 0; i < world->entity_count; i++) {
        if (!world->entity_active[i]) continue;
        if (world->physics_prop[i].inverse_mass == 0.0f) continue;

        int candidate_count = 0;
        BoundingBox query_box = get_aabb(world->transform[i].position, world->transform[i].orientation,
            world->collision[i]);
        
        query_tree(&world->collision_tree, world->collision_tree.root, query_box, candidates, &candidate_count);

        for (int k = 0; k < candidate_count; k++) {
            int j = candidates[k];

            if (i == j) continue;
            if (world->physics_prop[j].inverse_mass != 0.0f) {
                if (i > j) continue; 
            }

            Contact batch[4]; 
            int points_found = dispatch_collision(world, i, j, batch);

            for (int p = 0; p < points_found; p++) {
                if (contact_count < 1024) {
                    contacts[contact_count++] = batch[p];
                } else {
                    TraceLog(LOG_WARNING, "MAX CONTACTS REACHED! Physics may glitch.");
                }
            }
        }
    }

    for (int k = 0; k < 8; k++) solve_velocity(world, contacts, contact_count);
    solve_position(world, contacts, contact_count);

    for (int j = 0; j < contact_count; j++) {
        Contact* c = &contacts[j];
    
        if(c->normal.y > 0.7f) {
            world->physics_state[c->b_idx].in_ground = true;
        }

        if(c->normal.y < -0.7f) {
            world->physics_state[c->a_idx].in_ground = true;
        }
    }
}

void update_render(GameWorld* world) {
    for (int i = 0; i < world->entity_count; i++) {
        if (!world->entity_active[i]) continue;
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

void draw_ui(GameWorld* world, int player_idx) {
    if (player_idx == -1) return;

    PlayerLogicComponent* logic = &world->player_logic[player_idx];
    const int screen_width = GetScreenWidth();
    const int bar_width = 30;
    const int bar_height = 150;
    const int margin = 20;
    const int padding = 10;
    int posX_Attract = screen_width - margin - bar_width;
    int posX_Repel   = posX_Attract - bar_width - padding;
    int posX_Dash    = posX_Repel - bar_width - padding;
    int posY = margin;

    float pct_attract = logic->energy_attract / MAX_ENERGY;
    Color color_attract = RED;
    if (logic->attract_overheat) color_attract = GRAY;

    DrawRectangle(posX_Attract, posY, bar_width, bar_height, Fade(DARKGRAY, 0.5f));
    int fill_height_A = (int)(bar_height * pct_attract);
    int fillY_A = posY + (bar_height - fill_height_A);
    DrawRectangle(posX_Attract, fillY_A, bar_width, fill_height_A, color_attract);
    DrawRectangleLines(posX_Attract, posY, bar_width, bar_height, BLACK);
    DrawText("+", posX_Attract + 8, posY + bar_height + 5, 20, BLACK);
    if (logic->attract_overheat) DrawText("!", posX_Attract + 10, posY - 20, 20, RED);

    float pct_repel = logic->energy_repel / MAX_ENERGY;
    Color color_repel = BLUE;
    if (logic->repel_overheat) color_repel = GRAY;

    DrawRectangle(posX_Repel, posY, bar_width, bar_height, Fade(DARKGRAY, 0.5f));
    int fill_height_R = (int)(bar_height * pct_repel);
    int fillY_R = posY + (bar_height - fill_height_R);
    DrawRectangle(posX_Repel, fillY_R, bar_width, fill_height_R, color_repel);
    DrawRectangleLines(posX_Repel, posY, bar_width, bar_height, BLACK);
    DrawText("-", posX_Repel + 8, posY + bar_height + 5, 20, BLACK);
    if (logic->repel_overheat) DrawText("!", posX_Repel + 10, posY - 20, 20, RED);

    float pct_dash = 1.0f - (logic->dash_cooldown / DASH_COOLDOWN);

    if (pct_dash < 0.0f) pct_dash = 0.0f;
    if (pct_dash > 1.0f) pct_dash = 1.0f;

    Color color_dash = LIME; 
    if (pct_dash < 1.0f) color_dash = ORANGE;

    DrawRectangle(posX_Dash, posY, bar_width, bar_height, Fade(DARKGRAY, 0.5f));
    int fill_height_D = (int)(bar_height * pct_dash);
    int fillY_D = posY + (bar_height - fill_height_D);
    
    DrawRectangle(posX_Dash, fillY_D, bar_width, fill_height_D, color_dash);
    DrawRectangleLines(posX_Dash, posY, bar_width, bar_height, BLACK);
    DrawText("D", posX_Dash + 8, posY + bar_height + 5, 20, BLACK);
}