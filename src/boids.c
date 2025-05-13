
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <omp.h>

#include "boids.h"
#include "spatial_hash.h"
#include "normal_random.h"

Boid boids[MAX_BOIDS + 2]; // +1 for predator, +1 for mouse

void InitBoids() {
    // Initialize spatial hash
    init_spatial_hash();

    // Initialize boids
    for (int i = 0; i < MAX_BOIDS; i++) {
        boids[i].position = (Vector2){ GetRandomValue(0, SCREEN_WIDTH), GetRandomValue(0, SCREEN_HEIGHT) };
        float angle = GetRandomValue(0, 360) * DEG2RAD;
        float speed = random_normal(4.0f, 3.0f);
        boids[i].velocity = Vector2Scale((Vector2){ cosf(angle), sinf(angle) }, speed);
        boids[i].isPredator = false;
        boids[i].neighborCount = -1;
        boids[i].nearNeighborCount = -1;
        insert_boid(&boids[i]);
    }
    // Predator
    boids[PREDITOR_INDEX].position = (Vector2){ SCREEN_WIDTH/2, SCREEN_HEIGHT/2 };
    boids[PREDITOR_INDEX].velocity = (Vector2){ PREDATOR_SPEED, PREDATOR_SPEED };
    boids[PREDITOR_INDEX].isPredator = true;

    // Mouse
    boids[MOUSE_INDEX].position = (Vector2){ -1.0f, -1.0f };
    boids[MOUSE_INDEX].velocity = (Vector2){ 0.0f, 0.0f };
    boids[MOUSE_INDEX].isPredator = false;
    insert_boid(&boids[MAX_BOIDS]);
}

Vector2 Vector2Wrap(Vector2 v, float width, float height)
{
    if (v.x < 0) v.x += width;
    else if (v.x >= width) v.x -= width;

    if (v.y < 0) v.y += height;
    else if (v.y >= height) v.y -= height;

    return v;
}

void UpdateBoids(float alignmentWeight, float cohesionWeight, float separationWeight)
{
    // Parallel update stage
    #pragma omp parallel for schedule(static)
    for (int boid_index = 0; boid_index < MAX_BOIDS; boid_index++) {
        Boid* self = &boids[boid_index];

        // Initialize updates
        self->velocity_update = self->velocity;
        self->position_update = self->position;

        // Compute flocking forces
        // ComputeFlockForces() is a function that computes the alignment, cohesion, and separation forces
        FlockForces forces = ComputeFlockForces(self);
        self->neighborCount = forces.neighborCount;
        self->nearNeighborCount = forces.nearNeighborCount;

        // Apply flocking behaviour
        if (forces.neighborCount > 0) {
            Vector2 align_force = Vector2Subtract(forces.alignment, self->velocity);
            self->velocity_update = Vector2Add(self->velocity_update, Vector2Scale(align_force, MATCH_FACTOR * alignmentWeight));

            Vector2 cohesion_force = Vector2Subtract(forces.cohesion, self->position);
            self->velocity_update = Vector2Add(self->velocity_update, Vector2Scale(cohesion_force, CENTER_FACTOR * cohesionWeight));
        }
        self->velocity_update = Vector2Add(self->velocity_update, Vector2Scale(forces.separation, AVOID_FACTOR * separationWeight));

        // Predator avoidance
        Vector2 predatorVec = Vector2Subtract(self->position, boids[MAX_BOIDS].position);
        float distToPredator = Vector2Length(predatorVec);
        if (distToPredator < PREDATOR_RADIUS) {
            self->predated = true;
            if (distToPredator != 0)
                predatorVec = Vector2Scale(predatorVec, PREDATOR_AVOID_FACTOR / distToPredator);
            self->velocity_update = Vector2Add(self->velocity_update, predatorVec);
        }
        else {
            self->predated = false;
        }

        // Mouse
        if (mousePressed) {
            Vector2 mouseVec = Vector2Subtract(self->position, boids[MOUSE_INDEX].position);
            float distToMouse = Vector2Length(mouseVec);
            if (distToMouse < MOUSE_RADIUS) {
                self->predated = true;
                if (distToMouse != 0) mouseVec = Vector2Scale(mouseVec, - MOUSE_ATTRACTION_FACTOR / distToMouse);
                self->velocity_update = Vector2Add(self->velocity_update, mouseVec);
            }
        }

        // Speed limiting
        self->velocity_update = Vector2ClampValue(self->velocity_update, MIN_SPEED, MAX_SPEED);

        // Predict next position
        self->position_update = Vector2Add(self->position, Vector2Scale(self->velocity_update, GetFrameTime() * 60.0f));

        // Screen wrap
        self->position_update = Vector2Wrap(self->position_update, SCREEN_WIDTH, SCREEN_HEIGHT);
    }

    // Move predator (serial)
    boids[PREDITOR_INDEX].position = Vector2Add(boids[PREDITOR_INDEX].position, Vector2Scale(boids[PREDITOR_INDEX].velocity, GetFrameTime() * 60.0f));
    //if (boids[PREDITOR_INDEX].position.x < 0 || boids[PREDITOR_INDEX].position.x > SCREEN_WIDTH) boids[PREDITOR_INDEX].velocity.x *= -1;
    //if (boids[PREDITOR_INDEX].position.y < 0 || boids[PREDITOR_INDEX].position.y > SCREEN_HEIGHT) boids[PREDITOR_INDEX].velocity.y *= -1;
    boids[PREDITOR_INDEX].position = Vector2Wrap(boids[PREDITOR_INDEX].position, SCREEN_WIDTH, SCREEN_HEIGHT);

    // Commit updates and rebuild spatial hash (serial)
    clear_spatial_hash();
    for (int i = 0; i < MAX_BOIDS; i++) {
        boids[i].velocity = boids[i].velocity_update;
        boids[i].position = boids[i].position_update;
        insert_boid(&boids[i]);
    }
    insert_boid(&boids[MAX_BOIDS]);
}

static Color colors[11] = {
    (Color){  0,  40,  82, 255},  // Deep Blue (20% darker)
    (Color){  0,  60, 122, 255},
    (Color){  0,  81, 163, 255},
    (Color){ 40, 122, 204, 255},  // Light Blue
    (Color){ 81, 163, 204, 255},  // Cyanish
    (Color){102, 184, 184, 255},  // Light greenish-cyan
    (Color){122, 204, 163, 255},  // Minty green
    (Color){204, 204,  81, 255},  // Yellow
    (Color){204, 163,  40, 255},  // Orange-Yellow
    (Color){204,  81,  40, 255},  // Orange
    (Color){163,   0,   0, 255}   // Deep Red (hottest)
};

int int_log2(int x) {
    if (x < 0) { exit(0); } // Error: log2(0) is undefined}
    if (x <= 0) {
        return 0;
    }
    int log = 0;
    while (x >>= 1) {
        log++;
        if (log >= 10) return 10;
        }
    return log;
}

int number_drawn = 0;

void DrawBoid(Boid boid) {
    number_drawn++;
    float size = BOID_RADIUS;
    Vector2 topLeft = { boid.position.x - size / 2, boid.position.y - size / 2 };
    Color color =  drawDensity ? colors[int_log2(boid.neighborCount + boid.nearNeighborCount)] : DARKGRAY;
    DrawRectangleV(topLeft, (Vector2){size, size}, color);
    if (drawFullGlyph) {
        // Normalize velocity to get direction
        Vector2 dir = Vector2Normalize(boid.velocity);

        // Draw main circle
        DrawCircleLinesV(boid.position, PROTECTED_RADIUS/2.0, boid.predated ? GREEN : color);

        // Compute tail endpoint (outside of the circle)
        Vector2 tailDir = Vector2Scale(dir, -(10.0f + 10)); // 10 pixels past edge
        Vector2 tailEnd = Vector2Add(boid.position, tailDir);

        // Draw tail line
        DrawLineV(boid.position, tailEnd, boid.predated ? GREEN : color);
    }
}

void DrawPreditor(Boid boid) {
    number_drawn++;
    // Normalize velocity to get direction
    Vector2 dir = Vector2Normalize(boid.velocity);

    // Draw main circle
    DrawCircleLinesV(boid.position, PREDATOR_RADIUS, RED);

    // Draw center dot
    DrawCircleV(boid.position, 2.0f, DARKGRAY);

    // Compute tail endpoint (outside of the circle)
    Vector2 tailDir = Vector2Scale(dir, -(10.0f + 10)); // 10 pixels past edge
    Vector2 tailEnd = Vector2Add(boid.position, tailDir);

    // Draw tail line
    DrawLineV(boid.position, tailEnd, BLUE);
}

void DrawMouse(Boid boid) {
    // Draw main circle
    DrawCircleLinesV(boid.position, MOUSE_RADIUS, BLUE);

    // Draw center dot
    DrawCircleV(boid.position, BOID_RADIUS, RED);
}

void DrawBoids() {
    number_drawn = 0;
    for (int i = 0; i < MAX_BOIDS; i++) DrawBoid(boids[i]);
    DrawPreditor(boids[PREDITOR_INDEX]);
    if (mousePressed) DrawMouse(boids[MOUSE_INDEX]);
}

void DrawNearestNeighborNetwork(){
    for (int i = 0; i < MAX_BOIDS; i++) DrawNearestNeighbor(&boids[i]);
}

