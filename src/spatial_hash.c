#include "spatial_hash.h"
#include <stdlib.h>
#include <math.h>

#include "boids.h"

int length(BoidNode* node) {
    int count = 0;
    while (node) {
        count++;
        node = node->next;
    }
    return count;
}

void free_boid_node(BoidNode* node) {
    while(node) {
        BoidNode* next = node->next;
        free(node);
        node = next;
    }
}

static BoidNode* hash_table[HASH_SIZE] = {0};

static unsigned int hash_cell(int cell_x, int cell_y) {
    unsigned int hash = (unsigned)(cell_x * 73856093) ^ (cell_y * 19349663);
    return hash % HASH_SIZE;
}

void init_spatial_hash(void) {
    for (int i = 0; i < HASH_SIZE; ++i) hash_table[i] = NULL;
}

void clear_spatial_hash(void) {
    for (int i = 0; i < HASH_SIZE; ++i) {
        BoidNode* node = hash_table[i];
        while (node) {
            BoidNode* next = node->next;
            free(node);
            node = next;
        }
        hash_table[i] = NULL;
    }
}

void insert_boid(Boid* p) {
    int cell_x = (int)(p->position.x / CELL_SIZE);
    int cell_y = (int)(p->position.y / CELL_SIZE);
    unsigned int index = hash_cell(cell_x, cell_y);

    BoidNode* node = malloc(sizeof(BoidNode));
    node->boid = p;
    node->next = hash_table[index];
    hash_table[index] = node;
}

BoidNode* find_neighbors(Boid* p) {
    BoidNode* neighbors = NULL;

    int cell_x = (int)(p->position.x / CELL_SIZE);
    int cell_y = (int)(p->position.y / CELL_SIZE);

    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            int nx = cell_x + dx;
            int ny = cell_y + dy;
            unsigned int index = hash_cell(nx, ny);

            BoidNode* node = hash_table[index];
            while (node) {
                Boid* other = node->boid;
                if (other != p) {
                    BoidNode* new_node = malloc(sizeof(BoidNode));
                    new_node->boid = other;
                    new_node->next = neighbors;
                    neighbors = new_node;
                }
                node = node->next;
            }
        }
    }

    return neighbors;
}
