#pragma once

#include "state.h"

Vector3 Vector3TransformRotate(Vector3, Matrix);
void update_physics(GameWorld*, float);
void detect_collisions(GameWorld*);
void update_render(GameWorld*);