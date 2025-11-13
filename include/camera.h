#pragma once

#include <raylib.h>

extern Camera3D camera;
extern float camera_radius;
extern float camera_height;

void init_camera();
void update_camera();
