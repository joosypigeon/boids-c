#ifndef SPATIAL_HASH_H
#define SPATIAL_HASH_H

#include <stdbool.h>
#include "boids.h"

#define HASH_SIZE 10007
#define CELL_SIZE 50.0f

void init_spatial_hash(void);
void clear_spatial_hash(void);
BoidNode* find_neighbors(Boid* p);
void free_boid_node(BoidNode* node);

int length(BoidNode* node);

#endif // SPATIAL_HASH_H

