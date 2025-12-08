#pragma once

#include "state.h"

void init_world(GameWorld*, int);
int create_entity(GameWorld*, EntityDesc);
void create_scene(GameWorld*);
void free_world(GameWorld*);