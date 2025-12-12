#pragma once

#include "state.h"

void init_world(GameWorld*, int);
int create_entity(GameWorld*, EntityDesc, GameConfig*);
void create_scene(GameWorld*, GameConfig*);
void destroy_entity(GameWorld*, int);
void free_world(GameWorld*);