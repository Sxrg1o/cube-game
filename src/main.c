#include <raylib.h>
#include <raymath.h>
#include <math.h>
#define RAYGUI_IMPLEMENTATION
#include <raygui.h>

#include "camera.h"
#include "state.h"
#include "world.h"
#include "systems.h"
#include "gameplay.h"
#include "gui.h"

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

void reset_game_config(GameConfig* config) {
    config->player_move_speed = PLAYER_MOVE_SPEED_DEFAULT;
    config->player_jump_force = PLAYER_JUMP_FORCE_DEFAULT;
    config->magnet_force_magnitude = MAGNET_FORCE_DEFAULT;
    config->magnet_radius = MAGNET_RADIUS_DEFAULT;
    config->max_energy = MAX_ENERGY_DEFAULT;
    config->recharge_time = RECHARGE_TIME_DEFAULT;
    config->dash_force = DASH_FORCE_DEFAULT;
    config->dash_cooldown = DASH_COOLDOWN_DEFAULT;
    config->total_health = TOTAL_HEALTH_DEFAULT;
    config->gravity_force = GRAVITY_FORCE_DEFAULT;
    config->player_size = PLAYER_SIZE_DEFAULT;
    config->match_duration = MATCH_DURATION_DEFAULT;
    config->rounds = ROUNDS_DEFAULT;
    config->main_platform_size = (Vector2){ PLATFORM_X_DEFAULT, PLATFORM_Y_DEFAULT };
}

int main(void) {
    const int screen_width = 1600;
    const int screen_height = 900;

    InitWindow(screen_width, screen_height, "cube");
    init_camera();
    GuiSetStyle(DEFAULT, TEXT_SIZE, 18);

    Game game = {0};
    game.state = MENU;
    reset_game_config(&game.config);

    int local_player_id = -1;

    SetTargetFPS(144);

    double current_time = GetTime();
    double accumulator = 0.0;

    while (!WindowShouldClose()) {
        if (game.state == PLAYING && IsKeyPressed(KEY_ESCAPE)) {
            free_world(&game.world);
            game.state = MENU;
            local_player_id = -1;
        }

        switch (game.state) {
            case MENU:
            case LOBBY:
                break;

            case PLAYING: {
                double new_time = GetTime();
                double frame_time = new_time - current_time;
                current_time = new_time;

                if (frame_time > 0.25) frame_time = 0.25;
                accumulator += frame_time;

                if (local_player_id == -1) {
                    for(int i = 0; i < game.world.entity_count; i++) {
                        if (!game.world.entity_active[i]) continue;
                        if(game.world.player_logic[i].is_player) {
                            local_player_id = i;
                            break;
                        }
                    }
                }

                if (local_player_id != -1) {
                    PlayerInput current_input = {0};
                    if (IsKeyDown(KEY_W)) current_input.buttons |= BUTTON_UP;
                    if (IsKeyDown(KEY_S)) current_input.buttons |= BUTTON_DOWN;
                    if (IsKeyDown(KEY_A)) current_input.buttons |= BUTTON_LEFT;
                    if (IsKeyDown(KEY_D)) current_input.buttons |= BUTTON_RIGHT;
                    if (IsKeyDown(KEY_SPACE)) current_input.buttons |= BUTTON_JUMP;
                    if (IsKeyDown(KEY_G)) current_input.buttons |= BUTTON_ATTRACT;
                    if (IsKeyDown(KEY_H)) current_input.buttons |= BUTTON_REPEL;
                    if (IsKeyDown(KEY_LEFT_SHIFT)) current_input.buttons |= BUTTON_DASH;
                    
                    current_input.yaw = get_player_yaw(&game.world, camera, local_player_id);

                    update_camera();

                    while (accumulator >= DELTA_TIME) {
                        update_gameplay(&game.world, local_player_id, current_input, (float)DELTA_TIME, &game.config);
                        update_physics(&game.world, (float)DELTA_TIME, &game.config);
                        constrain_player_upright(&game.world, local_player_id);
                        accumulator -= DELTA_TIME;
                    }
                }
            } break;
        }

        BeginDrawing();
            ClearBackground(RAYWHITE);

            switch (game.state) {
                case MENU:
                    draw_main_menu(&game, screen_width, screen_height);
                    break;
                
                case LOBBY:
                    draw_config_screen(&game, screen_width, screen_height);
                    break;

                case PLAYING:
                    BeginMode3D(camera);
                        update_render(&game.world);
                    EndMode3D();
                    draw_hud(&game.world, local_player_id, &game.config);
                    DrawFPS(10, 10);
                    break;
            }

        EndDrawing();
    }

    if (game.state == PLAYING) {
        free_world(&game.world);
    }

    CloseWindow();
    return 0;
}