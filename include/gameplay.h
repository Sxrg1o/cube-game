#pragma once

#include <stdint.h>

#include "state.h"
#include "components.h"

#define DAMAGE_THRESHOLD 20.0f
#define DAMAGE_FACTOR 0.1f

#define BUTTON_UP (1 << 0)
#define BUTTON_DOWN (1 << 1)
#define BUTTON_LEFT (1 << 2)
#define BUTTON_RIGHT (1 << 3)
#define BUTTON_JUMP (1 << 4)
#define BUTTON_ATTRACT (1 << 5)
#define BUTTON_REPEL (1 << 6)
#define BUTTON_DASH (1 << 7)

typedef struct {
    uint8_t buttons;
    float yaw;
} PlayerInput;

void update_gameplay(GameWorld*, int, PlayerInput, float, GameConfig*);
void constrain_player_upright(GameWorld*, int);