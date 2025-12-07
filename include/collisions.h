#pragma once

#include <raylib.h>

#include "state.h"

typedef struct {
    int a_idx;
    int b_idx;
    Vector3 impact_point;
    Vector3 normal;
    float penetration;
    float acc_normal_impulse;
    float acc_tangent_impulse_1;
    float acc_tangent_impulse_2;
} Contact;

typedef int (*CollisionFunc)(GameWorld*, int, int, Contact*);

void solve_position(GameWorld*, Contact*, int);
void solve_velocity(GameWorld*, Contact*, int);
int dispatch_collision(GameWorld*, int, int, Contact*);