#include <raylib.h>
#include <raymath.h>
#include <math.h>

#include "camera.h"
#include "state.h"
#include "world.h"
#include "systems.h"
#include "gameplay.h"

#define PHYSICS_TICK_RATE 60.0
#define DELTA_TIME (1.0 / PHYSICS_TICK_RATE)

static float get_player_yaw(GameWorld* world, Camera3D cam, int player_idx) {    
    if (player_idx == -1) return 0.0f;

    Vector3 player_pos = world->transform[player_idx].position;
    Ray ray = GetScreenToWorldRay(GetMousePosition(), cam);
    
    if (fabs(ray.direction.y) < 0.0001f) return 0.0f;

    float t = (player_pos.y - ray.position.y) / ray.direction.y;
    if (t < 0) return 0.0f;

    Vector3 hit_point = Vector3Add(ray.position, Vector3Scale(ray.direction, t));
    Vector3 diff = Vector3Subtract(hit_point, player_pos);
    
    return atan2f(diff.x, diff.z);
}

int main(void) {
    const int screen_width = 1600;
    const int screen_height = 900;

    InitWindow(screen_width, screen_height, "cube");
    
    init_camera();
    
    GameWorld world;
    init_world(&world, 100);
    create_scene(&world);

    int local_player_id = -1;
    // TODO: For now, later will be defined by network
    for(int i = 0; i < world.entity_count; i++) {
        if (!world.entity_active[i]) continue;
        if(world.player_logic[i].is_player) {
            local_player_id = i;
            break;
        }
    }

    if (local_player_id == -1) {
        TraceLog(LOG_FATAL, "No Local Player Found!");
        CloseWindow();
        return -1;
    }

    SetTargetFPS(144);

    double current_time = GetTime();
    double accumulator = 0.0;

    while (!WindowShouldClose()) {
        double new_time = GetTime();
        double frame_time = new_time - current_time;
        current_time = new_time;

        if (frame_time > 0.25) frame_time = 0.25;

        accumulator += frame_time;

        PlayerInput current_input = {0};
        
        if (IsKeyDown(KEY_W)) current_input.buttons |= BUTTON_UP;
        if (IsKeyDown(KEY_S)) current_input.buttons |= BUTTON_DOWN;
        if (IsKeyDown(KEY_A)) current_input.buttons |= BUTTON_LEFT;
        if (IsKeyDown(KEY_D)) current_input.buttons |= BUTTON_RIGHT;
        if (IsKeyDown(KEY_SPACE)) current_input.buttons |= BUTTON_JUMP;
        if (IsKeyDown(KEY_G)) current_input.buttons |= BUTTON_ATTRACT;
        if (IsKeyDown(KEY_H)) current_input.buttons |= BUTTON_REPEL;
        if (IsKeyDown(KEY_LEFT_SHIFT)) current_input.buttons |= BUTTON_DASH;
        
        current_input.yaw = get_player_yaw(&world, camera, local_player_id);

        update_camera();

        while (accumulator >= DELTA_TIME) {
            update_gameplay(&world, local_player_id, current_input, (float)DELTA_TIME);
            update_physics(&world, (float)DELTA_TIME);
            constrain_player_upright(&world, local_player_id);

            accumulator -= DELTA_TIME;
        }

        BeginDrawing();
            ClearBackground(RAYWHITE);
            BeginMode3D(camera);
                update_render(&world);
            EndMode3D();
            draw_ui(&world, local_player_id);
            DrawFPS(10, 10);
        EndDrawing();
    }

    free_world(&world);
    CloseWindow();

    return 0;
}