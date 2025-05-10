#ifndef SPATIAL_HASH_H
#define SPATIAL_HASH_H

#include <stdbool.h>
#include "boids.h"

#define HASH_SIZE 10007
#define CELL_SIZE 50.0f


#define INITIAL_MAX_BOIDS_PER_CELL 256 // Tweak as needed

typedef struct {
    Vector2 alignment;
    Vector2 cohesion;
    Vector2 separation;
    int neighborCount;
} FlockForces;

typedef struct {
    int length;
    int max_length;
    Boid** boids;  // dynamically allocated array
} HashCell;

extern HashCell hash_table[HASH_SIZE];

void init_spatial_hash(void);
void clear_spatial_hash(void);
unsigned int hash_cell(int cell_x, int cell_y);
void free_boid_node(BoidNode* node);

FlockForces ComputeFlockForces(Boid *boid);

int length(BoidNode* node);

#endif // SPATIAL_HASH_H

