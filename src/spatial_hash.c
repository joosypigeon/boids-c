#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "spatial_hash.h"
#include "boids.h"

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

FlockForces ComputeFlockForces(Boid *boid) {
    FlockForces forces = {0};

    int cell_x = (int)(boid->position.x / CELL_SIZE);
    int cell_y = (int)(boid->position.y / CELL_SIZE);

    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            int nx = cell_x + dx;
            int ny = cell_y + dy;
            unsigned int index = hash_cell(nx, ny);

            HashCell* cell = &hash_table[index];
            for (int j = 0; j < cell->length; ++j) {
                Boid* neighbor = cell->boids[j];
                if (neighbor != boid) {
                    float dist = Vector2Distance(boid->position, neighbor->position);
                    if (dist < PROTECTED_RADIUS) {
                        Vector2 diff = Vector2Subtract(boid->position, neighbor->position);
                        if (dist != 0) diff = Vector2Scale(diff, 1.0f / (dist*dist)) ;
                        forces.separation = Vector2Add(forces.separation, diff);
                        forces.nearNeighborCount++;
                    } else if (dist < NEIGHBOR_RADIUS) {
                        forces.alignment = Vector2Add(forces.alignment, neighbor->velocity);
                        forces.cohesion = Vector2Add(forces.cohesion, neighbor->position);
                        forces.neighborCount++;
                    }
                }
            }
        }
    }
    if (forces.neighborCount > 0) {
        forces.alignment = Vector2Scale(forces.alignment, 1.0f / forces.neighborCount);
        forces.cohesion = Vector2Scale(forces.cohesion, 1.0f / forces.neighborCount);
    }
    return forces;
}

void DrawNearestNeighbor(Boid *boid){
    int cell_x = (int)(boid->position.x / CELL_SIZE);
    int cell_y = (int)(boid->position.y / CELL_SIZE);

    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            int nx = cell_x + dx;
            int ny = cell_y + dy;
            unsigned int index = hash_cell(nx, ny);

            HashCell* cell = &hash_table[index];
            for (int j = 0; j < cell->length; ++j) {
                Boid* neighbor = cell->boids[j];
                if (neighbor != boid) {
                    float dist = Vector2Distance(boid->position, neighbor->position);
                    if (dist < NEIGHBOR_RADIUS) {
                        //printf("boid->position: (%.2f, %.2f), neighbor->position: (%.2f, %.2f)\n",boid->position.x, boid->position.y, neighbor->position.x, neighbor->position.y);
                        DrawLineV(boid->position, neighbor->position, GREEN);
                    }
                }
            }
        }
    }  
}
