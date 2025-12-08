#pragma once

#include <raylib.h>

#define CAM_AV 1.5f
#define CAM_LV 50.0f
#define MAX_DISTANCE 50.0f
#define MIN_DISTANCE 10.0f

extern Camera3D camera;
extern float camera_radius;
extern float camera_height;

void init_camera();
void update_camera();
