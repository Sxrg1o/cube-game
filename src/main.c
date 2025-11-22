#include <raylib.h>
#include <raymath.h>

#include "camera.h"
#include "state.h"
#include "world.h"

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

        if (IsCursorHidden()) update_camera();
        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
            if (IsCursorHidden()) EnableCursor();
            else DisableCursor();
        }
        // TODO: Update Systems

        BeginDrawing();
            ClearBackground(RAYWHITE);
            BeginMode3D(camera);
                
                DrawGrid(20, 1.0f);

                for (int i = 0; i < world.entity_count; i++) {
                    Vector3 pos = world.transform[i].position;
                    
                    if (world.collision[i].type == SHAPE_CUBE) {
                        Vector3 half_size = world.collision[i].params.cube_extents;
                        Vector3 size = Vector3Scale(half_size, 2.0f);
                        Color col = (i == 0) ? GRAY : RED;
                        DrawCube(pos, size.x, size.y, size.z, col);
                        DrawCubeWires(pos, size.x, size.y, size.z, MAROON);
                    }
                }

            EndMode3D();
            DrawFPS(10, 10);
        EndDrawing();
    }

    free_world(&world);
    CloseWindow();

    return 0;
}