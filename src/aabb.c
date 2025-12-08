#include <stdlib.h>
#include <math.h>
#include <raylib.h>
#include <raymath.h>

#include "aabb.h"
#include "systems.h"

void init_tree(DynamicTree* tree, int size) {
    tree->root = NULL_NODE;
    tree->node_capacity = size;
    tree->node_count = 0;
    tree->nodes = (TreeNode*)calloc(size, sizeof(TreeNode));

    for (int i = 0; i < size - 1; i++) {
        tree->nodes[i].next_free = i + 1;
        tree->nodes[i].height = -1;
    }
    
    tree->nodes[size - 1].next_free = NULL_NODE;
    tree->nodes[size - 1].height = -1;
    tree->free_idx = 0;
}

static Vector3 calc_extents(Matrix rot_matrix, Vector3 loc_extents) {
    Vector3 rot_extents;
    rot_extents.x = fabsf(rot_matrix.m0) * loc_extents.x + 
        fabsf(rot_matrix.m1) * loc_extents.y + fabsf(rot_matrix.m2) * loc_extents.z;
    rot_extents.y = fabsf(rot_matrix.m4) * loc_extents.x +
        fabsf(rot_matrix.m5) * loc_extents.y + fabsf(rot_matrix.m6) * loc_extents.z;
    rot_extents.z = fabsf(rot_matrix.m8) * loc_extents.x +
        fabsf(rot_matrix.m9) * loc_extents.y + fabsf(rot_matrix.m10) * loc_extents.z;
    return rot_extents;
}

BoundingBox get_aabb(Vector3 position, Quaternion orientation, CollisionShapeComponent sh_ex) {
    BoundingBox aabb;
    Vector3 extents;
    Vector3 margin = {AABB_MARGIN, AABB_MARGIN, AABB_MARGIN};
    switch(sh_ex.type) {
        case SHAPE_SPHERE:
            float r = sh_ex.params.sphere_radius;
            extents = (Vector3){r, r, r};
            break;
        case SHAPE_CUBE:
            Matrix rot_matrix_c = QuaternionToMatrix(orientation);
            Vector3 loc_extents_c = sh_ex.params.cube_extents;
            extents = calc_extents(rot_matrix_c, loc_extents_c);
            break;
        case SHAPE_PLANE:
            Matrix rot_matrix_p = QuaternionToMatrix(orientation);
            Vector3 loc_extents_p = (Vector3) {500.0f, 0.0f, 500.0f};
            extents = calc_extents(rot_matrix_p, loc_extents_p);
            break;
    }
    Vector3 fat_extents = Vector3Add(extents, margin);
    aabb.max = Vector3Add(position, fat_extents);
    aabb.min = Vector3Subtract(position, fat_extents);
    return aabb;
}

bool contains_box(BoundingBox proxy_box, BoundingBox new_box) {
    return (proxy_box.max.x >= new_box.max.x && proxy_box.max.y >= new_box.max.y && proxy_box.max.z >= new_box.max.z) &&
           (proxy_box.min.x <= new_box.min.x && proxy_box.min.y <= new_box.min.y && proxy_box.min.z <= new_box.min.z);
}

static float get_area(BoundingBox b) {
    float w = b.max.x - b.min.x;
    float h = b.max.y - b.min.y;
    float d = b.max.z - b.min.z;
    return 2.0f * (w * h + h * d + d * w);
}

static BoundingBox union_box(BoundingBox a, BoundingBox b) {
    BoundingBox res;
    res.min.x = fminf(a.min.x, b.min.x);
    res.min.y = fminf(a.min.y, b.min.y);
    res.min.z = fminf(a.min.z, b.min.z);
    res.max.x = fmaxf(a.max.x, b.max.x);
    res.max.y = fmaxf(a.max.y, b.max.y);
    res.max.z = fmaxf(a.max.z, b.max.z);
    return res;
}

static int alloc_node(DynamicTree* tree) {
    if (tree->free_idx == NULL_NODE) return NULL_NODE;
    int node = tree->free_idx;
    tree->free_idx = tree->nodes[node].next_free;
    tree->nodes[node].parent = NULL_NODE;
    tree->nodes[node].height = 0;
    tree->node_count++;
    return node;
}

static void free_node(DynamicTree* tree, int node) {
    tree->nodes[node].next_free = tree->free_idx;
    tree->nodes[node].height = -1;
    tree->free_idx = node;
    tree->node_count--;
}

int insert_leaf(DynamicTree* tree, int entity_idx, BoundingBox box) {
    int leaf = alloc_node(tree);
    tree->nodes[leaf].aabb = box;
    tree->nodes[leaf].entity_idx = entity_idx;
    tree->nodes[leaf].left = NULL_NODE;
    tree->nodes[leaf].right = NULL_NODE;
    tree->nodes[leaf].height = 0;

    if (tree->root == NULL_NODE) {
        tree->root = leaf;
        tree->nodes[leaf].parent = NULL_NODE;
        return leaf;
    }

    int idx = tree->root;
    while (tree->nodes[idx].left != NULL_NODE) {
        int left = tree->nodes[idx].left;
        int right = tree->nodes[idx].right;

        float area = get_area(tree->nodes[idx].aabb);
        BoundingBox combined = union_box(tree->nodes[idx].aabb, box);
        float combined_area = get_area(combined);

        float cost = 2.0f * combined_area;
        float inheritance_cost = 2.0f * (combined_area - area);

        float cost_l;
        BoundingBox union_l = union_box(tree->nodes[left].aabb, box);
        if (tree->nodes[left].left == NULL_NODE) {
            cost_l = get_area(union_l) + inheritance_cost;
        } else {
            cost_l = (get_area(union_l) - get_area(tree->nodes[left].aabb)) + inheritance_cost;
        }

        float cost_r;
        BoundingBox union_r = union_box(tree->nodes[right].aabb, box);
        if (tree->nodes[right].left == NULL_NODE) {
            cost_r = get_area(union_r) + inheritance_cost;
        } else {
            cost_r = (get_area(union_r) - get_area(tree->nodes[right].aabb)) + inheritance_cost;
        }

        if (cost < cost_l && cost < cost_r) break;

        if (cost_l < cost_r) idx = left;
        else idx = right;
    }

    int sibling = idx;
    int old_parent = tree->nodes[sibling].parent;
    int new_parent = alloc_node(tree);

    tree->nodes[new_parent].parent = old_parent;
    tree->nodes[new_parent].aabb = union_box(box, tree->nodes[sibling].aabb);
    tree->nodes[new_parent].height = tree->nodes[sibling].height + 1;
    tree->nodes[new_parent].left = sibling;
    tree->nodes[new_parent].right = leaf;
    tree->nodes[new_parent].entity_idx = -1;

    tree->nodes[sibling].parent = new_parent;
    tree->nodes[leaf].parent = new_parent;

    if (old_parent != NULL_NODE) {
        if (tree->nodes[old_parent].left == sibling) tree->nodes[old_parent].left = new_parent;
        else tree->nodes[old_parent].right = new_parent;
    } else {
        tree->root = new_parent;
    }

    int walk = tree->nodes[leaf].parent;
    while (walk != NULL_NODE) {
        int l = tree->nodes[walk].left;
        int r = tree->nodes[walk].right;
        
        tree->nodes[walk].height = 1 + (tree->nodes[l].height > tree->nodes[r].height ? tree->nodes[l].height : tree->nodes[r].height);
        tree->nodes[walk].aabb = union_box(tree->nodes[l].aabb, tree->nodes[r].aabb);
        
        walk = tree->nodes[walk].parent;
    }

    return leaf;
}

void remove_leaf(DynamicTree* tree, int leaf) {
    if (leaf == tree->root) {
        tree->root = NULL_NODE;
        free_node(tree, leaf);
        return;
    }

    int parent = tree->nodes[leaf].parent;
    int grand_parent = tree->nodes[parent].parent;
    int sibling;
    
    if (tree->nodes[parent].left == leaf) sibling = tree->nodes[parent].right;
    else sibling = tree->nodes[parent].left;

    if (grand_parent != NULL_NODE) {
        if (tree->nodes[grand_parent].left == parent) tree->nodes[grand_parent].left = sibling;
        else tree->nodes[grand_parent].right = sibling;
        
        tree->nodes[sibling].parent = grand_parent;
        free_node(tree, parent);

        int walk = grand_parent;
        while (walk != NULL_NODE) {
            int l = tree->nodes[walk].left;
            int r = tree->nodes[walk].right;
            
            tree->nodes[walk].height = 1 + (tree->nodes[l].height > tree->nodes[r].height ? tree->nodes[l].height : tree->nodes[r].height);
            tree->nodes[walk].aabb = union_box(tree->nodes[l].aabb, tree->nodes[r].aabb);
            
            walk = tree->nodes[walk].parent;
        }
    } else {
        tree->root = sibling;
        tree->nodes[sibling].parent = NULL_NODE;
        free_node(tree, parent);
    }

    free_node(tree, leaf);
}

void query_tree(DynamicTree* tree, int node, BoundingBox query_box, int* candidates, int* count) {
    if (node == NULL_NODE) return;

    bool overlap = (tree->nodes[node].aabb.max.x >= query_box.min.x && tree->nodes[node].aabb.min.x <= query_box.max.x) &&
               (tree->nodes[node].aabb.max.y >= query_box.min.y && tree->nodes[node].aabb.min.y <= query_box.max.y) &&
               (tree->nodes[node].aabb.max.z >= query_box.min.z && tree->nodes[node].aabb.min.z <= query_box.max.z);

    if (!overlap) return;

    if (tree->nodes[node].left == NULL_NODE) {
        candidates[*count] = tree->nodes[node].entity_idx;
        (*count)++;
        return;
    }

    query_tree(tree, tree->nodes[node].left, query_box, candidates, count);
    query_tree(tree, tree->nodes[node].right, query_box, candidates, count);
}

void free_tree(DynamicTree* tree) {
    if (tree->nodes) {
        free(tree->nodes);
        tree->nodes = NULL;
    }
    
    tree->root = NULL_NODE;
    tree->node_count = 0;
    tree->node_capacity = 0;
    tree->free_idx = NULL_NODE;
}