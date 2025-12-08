#pragma once

#include "state.h"
#include "components.h"

#define PLAYER_MOVE_SPEED 7.0f
#define PLAYER_JUMP_FORCE 10.0f
#define MAGNET_FORCE_MAGNITUDE 100.0f
#define MAGNET_RADIUS 20.0f
#define MAX_ENERGY 5.0f
#define RECHARGE_TIME 5.0f
#define RECHARGE_RATE (MAX_ENERGY / RECHARGE_TIME)
#define UNLOCK_THRESHOLD (MAX_ENERGY * 0.5f)

void update_gameplay(GameWorld*, Camera3D, float);
void constrain_player_upright(GameWorld*);