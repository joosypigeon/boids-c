#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "spatial_hash.h"
#include "boids.h"

#include <assert.h>

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

    assert(cell_x >= 0 && cell_x < CELL_WIDTH);
    assert(cell_y >= 0 && cell_y < CELL_HEIGHT);

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
            unsigned int index = hash_cell(WRAP_MOD(nx, CELL_WIDTH), WRAP_MOD(ny, CELL_HEIGHT));
            HashCell* cell = &hash_table[index];
            for (int j = 0; j < cell->length; ++j) {
                Boid* neighbor = cell->boids[j];
                if (neighbor != boid) {
                    float dist = DistanceOnTorus(boid->position, neighbor->position);
                    if (dist < PROTECTED_RADIUS) {
                        Vector2 diff = Vector2SubtractTorus(boid->position, neighbor->position);
                        if (dist != 0) diff = Vector2Scale(diff, 1.0f / (dist*dist)) ;
                        forces.separation = Vector2Add(forces.separation, diff);
                        forces.nearNeighborCount++;
                    } else if (dist < NEIGHBOR_RADIUS) {
                        forces.alignment = Vector2Add(forces.alignment, neighbor->velocity);
                        Vector2 diff = Vector2SubtractTorus(neighbor->position, boid->position);
                        forces.cohesion = Vector2Add(forces.cohesion, Vector2Add(diff, boid->position));
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

Boid *FindNearestBoid(Vector2 position) {
    int cell_x = (int)(position.x / CELL_SIZE);
    int cell_y = (int)(position.y / CELL_SIZE);

    Boid *nearest_boid = NULL;
    float nearest_distance = 10000.0f;

    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            int nx = cell_x + dx;
            int ny = cell_y + dy;
            unsigned int index = hash_cell(WRAP_MOD(nx, CELL_WIDTH), WRAP_MOD(ny, CELL_HEIGHT));
            HashCell* cell = &hash_table[index];
            for (int j = 0; j < cell->length; ++j) {
                Boid* neighbor = cell->boids[j];
                if (neighbor != &boids[MOUSE_INDEX]) {
                    float dist = DistanceOnTorus(position, neighbor->position);
                    if (dist < nearest_distance) {
                        nearest_distance = dist;
                        nearest_boid = neighbor;
                    }
                }
            }
        }
    }
    return nearest_boid;
}


int ceil_div(int a, int b) {
    // Expect both a and b to be positive
    return (a + b - 1) / b;
}

Vector2 PreditorAjustment(){
    Vector2 preditor_adjustment = {0.0f, 0.0f};

    Vector2 predator_dir = Vector2Normalize(boids[PREDATOR_INDEX].velocity);

    int width = ceil_div(PREDATOR_VISUAL_RADIUS, CELL_SIZE);
    if (width < 1) width = 1;

    Boid* predator = &boids[PREDATOR_INDEX];
    int cell_x = (int)(predator->position.x / CELL_SIZE);
    int cell_y = (int)(predator->position.y / CELL_SIZE);

    int count = 0;
    for (int dx = -width; dx <= width; ++dx) {
        for (int dy = -width; dy <= width; ++dy) {
            int nx = cell_x + dx;
            int ny = cell_y + dy;
            unsigned int index = hash_cell(WRAP_MOD(nx, CELL_WIDTH), WRAP_MOD(ny, CELL_HEIGHT));
            HashCell* cell = &hash_table[index];
            for (int j = 0; j < cell->length; ++j) {
                Boid* neighbor = cell->boids[j];
                if (neighbor != predator) {
                    float dist = DistanceOnTorus(predator->position, neighbor->position);
                    if (dist < PREDATOR_VISUAL_RADIUS) {
                        count++;
                        Vector2 diff = Vector2SubtractTorus(neighbor->position, predator->position);
                        Vector2 to_neighbor = Vector2Normalize(diff);
                        float alignment = Vector2DotProduct(predator_dir, to_neighbor);  // ranges from -1.0 to 1.0
                        float scale = (alignment + 1.0f) * 0.5f;
                        Vector2 scaled_diff = Vector2Scale(diff, scale*scale*scale);
                        preditor_adjustment = Vector2Add(preditor_adjustment, scaled_diff);
                    }
                }
            }
        }
    }

    if (count > 0) {
        preditor_adjustment = Vector2Scale(preditor_adjustment, 1.0f / count);
    }

    return preditor_adjustment;
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
