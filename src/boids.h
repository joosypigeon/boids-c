#ifndef BOIDS_H
#define BOIDS_H

#include "raylib.h"
#include "raymath.h"

#define MAX_BOIDS 10000



#define NEIGHBOR_RADIUS 40.0f
#define PROTECTED_RADIUS 8.0f
#define PREDATOR_RADIUS 90.0f

#define AVOID_FACTOR 0.05f
#define MATCH_FACTOR 0.05f
#define CENTER_FACTOR 0.0005f
#define TURN_FACTOR 0.2f
#define PREDATOR_AVOID_FACTOR 10.0f

#define MAX_SPEED 1.0f
#define MIN_SPEED 0.5f

#define BOID_RADIUS 2.0f

extern int SCREEN_WIDTH;
extern int SCREEN_HEIGHT;

// Boid structure
typedef struct Boid {
    Vector2 position;
    Vector2 velocity;
    Vector2 position_update;
    Vector2 velocity_update;
    bool predated;
    bool isPredator;
} Boid;

extern Boid boids[MAX_BOIDS+1];

typedef struct BoidNode {
    Boid* boid;
    struct BoidNode* next;
} BoidNode;

void init_spatial_hash(void);
void clear_spatial_hash(void);
void insert_boid(Boid* p);
BoidNode* find_neighbors(Boid* p);

void InitBoids(void);
void UpdateBoids(void);
void DrawBoids(void);

extern int number_drawn;
#endif // BOIDS_H
