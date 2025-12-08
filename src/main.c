#include <raylib.h>
#include <raymath.h>

#include "camera.h"
#include "state.h"
#include "world.h"
#include "systems.h"
#include "gameplay.h"

int main(void) {
    const int screenWidth = 1600;
    const int screenHeight = 900;

    InitWindow(screenWidth, screenHeight, "cube");
    
    init_camera();
    
    GameWorld world;
    init_world(&world, 100);
    create_scene(&world);

    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        float delta_time = GetFrameTime();

        update_camera();
        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
            if (IsCursorHidden()) EnableCursor();
            else DisableCursor();
        }
        
        update_gameplay(&world, camera, delta_time);
        update_physics(&world, delta_time);

        BeginDrawing();
            ClearBackground(RAYWHITE);
            BeginMode3D(camera);

            update_render(&world);

            EndMode3D();
            DrawFPS(10, 10);
        EndDrawing();
    }

    free_world(&world);
    CloseWindow();

    return 0;
}