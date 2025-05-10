#include "spatial_hash.h"
#include <stdlib.h>
#include <stdio.h>
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

HashCell hash_table[HASH_SIZE];

unsigned int hash_cell(int cell_x, int cell_y) {
    unsigned int hash = (unsigned)(cell_x * 73856093) ^ (cell_y * 19349669);
    return hash % HASH_SIZE;
}

void init_spatial_hash(void) {
    for (int i = 0; i < HASH_SIZE; ++i) {
        hash_table[i].length = 0;
        hash_table[i].max_length = INITIAL_MAX_BOIDS_PER_CELL;
        hash_table[i].boids = malloc(INITIAL_MAX_BOIDS_PER_CELL * sizeof(Boid*));
        if (!hash_table[i].boids) {
            fprintf(stderr, "Failed to allocate boid array!\n");
            exit(1);
        }
    }
}

void clear_spatial_hash(void) {
    for (int i = 0; i < HASH_SIZE; ++i) {
        hash_table[i].length = 0;
    }
}

void insert_boid(Boid* p) {
    int cell_x = (int)(p->position.x / CELL_SIZE);
    int cell_y = (int)(p->position.y / CELL_SIZE);
    unsigned int index = hash_cell(cell_x, cell_y);

    HashCell* cell = &hash_table[index];

    if (cell->length < cell->max_length) {
        cell->boids[cell->length++] = p;
    } else {
        printf("Cell (%d, %d) full current max %d, reallocating...\n", cell_x, cell_y, cell->max_length);
        cell->max_length *= 2;
        Boid** new_boids = realloc(cell->boids, cell->max_length * sizeof(Boid*));
        if (!new_boids) {
            fprintf(stderr, "Failed to realloc boid array!\n");
            exit(1);
        }
        cell->boids = new_boids;
        cell->boids[cell->length++] = p;
    
        printf("Cell (%d, %d) new max %d\n", cell_x, cell_y, cell->max_length);
    }
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

            HashCell* cell = &hash_table[index];
            for(int i = 0; i < cell->length; ++i) {
                Boid* neighbor = cell->boids[i];
                if (neighbor != p) {
                    BoidNode* new_node = malloc(sizeof(BoidNode));
                    new_node->boid = neighbor;
                    new_node->next = neighbors;
                    neighbors = new_node;
                }
            }
        }
    }

    return neighbors;
}
