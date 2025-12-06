#pragma once

#include "state.h"

typedef struct {
    int a_idx;
    int b_idx;
    Vector3 impact_point;
    Vector3 normal;
    float penetration;
} Contact;

typedef bool (*CollisionFunc)(GameWorld*, int, int, Contact*);

void update_physics(GameWorld*, float);
void detect_collisions(GameWorld*);
void update_render(GameWorld*);