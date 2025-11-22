#include <raylib.h>
#include <math.h>

#include "camera.h"

Camera3D camera = {0};
float camera_radius = 30.0f;
float camera_height = 30.0f;

static float camera_angle = 0.0f;

void init_camera() {
    camera_angle = 0.0f;
    camera.position = (Vector3){ camera_radius, camera_height, 0.0f };
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;
}

void update_camera() {
    float delta_time = GetFrameTime();
    if(IsKeyDown(KEY_LEFT)) {
        camera_angle -= CAM_AV * delta_time;
    } else if(IsKeyDown(KEY_RIGHT)) {
        camera_angle += CAM_AV * delta_time;
    }
    if(IsKeyDown(KEY_UP)) {
        if(MAX_DISTANCE - camera_radius > 0.000001 &&
            MAX_DISTANCE - camera_height > 0.000001) {
            camera_radius += CAM_LV * delta_time;
            camera_height += CAM_LV * delta_time;
        }
    } else if(IsKeyDown(KEY_DOWN)) {
        if(camera_radius - MIN_DISTANCE > 0.000001 &&
            camera_height - MIN_DISTANCE > 0.000001) {
            camera_radius -= CAM_LV * delta_time;
            camera_height -= CAM_LV * delta_time;
        }
    }
    camera.position.x = cosf(camera_angle) * camera_radius;
    camera.position.z = sinf(camera_angle) * camera_radius;
    camera.position.y = camera_height;
}
