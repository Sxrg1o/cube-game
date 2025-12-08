#pragma once

#include <raylib.h>

#include "components.h"

#define AABB_MARGIN 0.2f
#define NULL_NODE -1

typedef struct {
    BoundingBox aabb;
    int entity_idx;
    union {
        int parent;
        int next_free;
    };
    int left;
    int right;
    int height;
} TreeNode;

typedef struct {
    TreeNode* nodes;
    int root;
    int node_capacity;
    int node_count;
    int free_idx;
} DynamicTree;

void init_tree(DynamicTree*, int);
BoundingBox get_aabb(Vector3, Quaternion, CollisionShapeComponent);
bool contains_box(BoundingBox, BoundingBox);
int insert_leaf(DynamicTree*, int, BoundingBox);
void remove_leaf(DynamicTree*, int);
void query_tree(DynamicTree*, int, BoundingBox, int*, int*);
void free_tree(DynamicTree*);