#pragma once

#include "state.h"

void init_world(GameWorld*, int);
Matrix calc_inertia_tensor(ShapeType, float, Vector3);
int create_entity(GameWorld*, EntityDesc);
void create_scene(GameWorld*);
void free_world(GameWorld*);