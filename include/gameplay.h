#pragma once

#include "state.h"
#include "components.h"

#define PLAYER_MOVE_SPEED 10.0f
#define PLAYER_JUMP_FORCE 15.0f
#define MAGNET_FORCE_MAGNITUDE 50.0f
#define MAGNET_RADIUS 20.0f

void update_gameplay(GameWorld*, Camera3D, float);
void constrain_player_upright(GameWorld*, int);