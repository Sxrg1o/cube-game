#include <raylib.h>

#include "camera.h"

int main(void) {
    const int screenWidth = 1600;
    const int screenHeight = 900;

    InitWindow(screenWidth, screenHeight, "cube");

    init_camera();

    Vector3 cubePosition = { 0.0f, 1.0f, 0.0f };
    Vector3 cubeSize = { 2.0f, 2.0f, 2.0f };

    SetTargetFPS(60);
    while (!WindowShouldClose()) {
        if (IsCursorHidden()) update_camera();

        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT))
        {
            if (IsCursorHidden()) EnableCursor();
            else DisableCursor();
        }

        BeginDrawing();

            ClearBackground(RAYWHITE);

            BeginMode3D(camera);

                DrawCube(cubePosition, cubeSize.x, cubeSize.y, cubeSize.z, RED);
                DrawCubeWires(cubePosition, cubeSize.x, cubeSize.y, cubeSize.z, MAROON);

                DrawGrid(10, 1.0f);

            EndMode3D();

            DrawFPS(10, 10);

        EndDrawing();
    }

    CloseWindow();

    return 0;
}
