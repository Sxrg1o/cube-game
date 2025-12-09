#pragma once

#include "state.h"

#define DAMPING_FACTOR 0.5f
#define SUB_STEPS 8

Vector3 Vector3TransformRotate(Vector3, Matrix);
void update_physics(GameWorld*, float);
void detect_collisions(GameWorld*);
void update_render(GameWorld*);
void draw_ui(GameWorld*, int);