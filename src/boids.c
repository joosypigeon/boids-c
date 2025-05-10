
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <omp.h>

#include "boids.h"
#include "spatial_hash.h"
#include "normal_random.h"

int MAX_NEIGHBORS = -1;

int get_max_neighbors(void) {
    return MAX_NEIGHBORS;
}

Boid boids[MAX_BOIDS + 1]; // last one is predator

// Helper function to limit vector length
Vector2 Vector2Limit(Vector2 v, float max)
{
    if (Vector2Length(v) > max) {
        v = Vector2Scale(Vector2Normalize(v), max);
    }
    return v;
}

void InitBoids()
{
    for (int i = 0; i < MAX_BOIDS; i++) {
        boids[i].position = (Vector2){ GetRandomValue(0, SCREEN_WIDTH), GetRandomValue(0, SCREEN_HEIGHT) };
        float angle = GetRandomValue(0, 360) * DEG2RAD;
        float speed = random_normal(4.0f, 3.0f);
        boids[i].velocity = Vector2Scale((Vector2){ cosf(angle), sinf(angle) }, speed);
        boids[i].isPredator = false;
        insert_boid(&boids[i]);
    }
    // Predator
    boids[MAX_BOIDS].position = (Vector2){ SCREEN_WIDTH/2, SCREEN_HEIGHT/2 };
    boids[MAX_BOIDS].velocity = (Vector2){ 2.0f, 2.0f };
    boids[MAX_BOIDS].isPredator = true;
    insert_boid(&boids[MAX_BOIDS]);
}

void XXUpdateBoids()
{
    // Update all normal boids first
    for (int i = 0; i < MAX_BOIDS; i++) {
        boids[i].predated = false;
        boids[i].velocity_update = boids[i].velocity;
        boids[i].position_update = boids[i].position;

        Vector2 alignment = {0};
        Vector2 cohesion = {0};
        Vector2 separation = {0};

        int neighborCount = 0;

        BoidNode* neighbors = find_neighbors(&boids[i]);
        BoidNode* iter = neighbors;
        //int nCount = length(neighbors);
        //printf("Boid %d has %d neighbors\n", i, nCount);
        //int count = 0;
        while(iter) {
            //count++;
            //if (count > 100) break; // Limit to 10 neighbors for performance
            Boid* neighbor = iter->boid;
            float dist = Vector2Distance(boids[i].position, neighbor->position);

            if (dist < PROTECTED_RADIUS) {
                Vector2 diff = Vector2Subtract(boids[i].position, neighbor->position);
                if (dist != 0) diff = Vector2Scale(diff, 1.0f/dist);
                separation = Vector2Add(separation, diff);
            } else if (dist < NEIGHBOR_RADIUS) {
                alignment = Vector2Add(alignment, neighbor->velocity);
                cohesion = Vector2Add(cohesion, neighbor->position);
                neighborCount++;
            }
            iter = iter->next;
        }

        free_boid_node(neighbors);

        // Predator avoidance
        Vector2 predatorVec = Vector2Subtract(boids[i].position, boids[MAX_BOIDS].position);
        float distToPredator = Vector2Length(predatorVec);
        if (distToPredator < PREDATOR_RADIUS) {
            boids[i].predated = true;
            if (distToPredator != 0) predatorVec = Vector2Scale(predatorVec, PREDATOR_AVOID_FACTOR/distToPredator);
            boids[i].velocity_update = Vector2Add(boids[i].velocity_update, predatorVec);
        }

        if (neighborCount > 0) {
            alignment = Vector2Scale(alignment, 1.0f/neighborCount);
            Vector2 align_force = Vector2Subtract(alignment, boids[i].velocity);
            boids[i].velocity_update = Vector2Add(boids[i].velocity_update, Vector2Scale(align_force, MATCH_FACTOR));

            cohesion = Vector2Scale(cohesion, 1.0f/neighborCount);
            Vector2 cohesion_force = Vector2Subtract(cohesion, boids[i].position);
            boids[i].velocity_update = Vector2Add(boids[i].velocity_update, Vector2Scale(cohesion_force, CENTER_FACTOR));
        }

        boids[i].velocity_update = Vector2Add(boids[i].velocity_update, Vector2Scale(separation, AVOID_FACTOR));

        // Wall avoidance
        //if (boids[i].position.x < SCREEN_WIDTH*0.05) boids[i].velocity_update.x += TURN_FACTOR;
        //if (boids[i].position.x > SCREEN_WIDTH*0.95) boids[i].velocity_update.x -= TURN_FACTOR;
        //if (boids[i].position.y < SCREEN_HEIGHT*0.05) boids[i].velocity_update.y += TURN_FACTOR;
        //if (boids[i].position.y > SCREEN_HEIGHT*0.95) boids[i].velocity_update.y -= TURN_FACTOR;

        // Speed limiting
        float speed = Vector2Length(boids[i].velocity_update);
        if (speed > MAX_SPEED) boids[i].velocity_update = Vector2Scale(Vector2Normalize(boids[i].velocity_update), MAX_SPEED);
        if (speed < MIN_SPEED) boids[i].velocity_update = Vector2Scale(Vector2Normalize(boids[i].velocity_update), MIN_SPEED);

        boids[i].position_update = Vector2Add(boids[i].position, Vector2Scale(boids[i].velocity_update, GetFrameTime() * 60.0f));

        // Wrap around screen
        if (boids[i].position_update.x < 0) boids[i].position_update.x += SCREEN_WIDTH;
        if (boids[i].position_update.x > SCREEN_WIDTH) boids[i].position_update.x -= SCREEN_WIDTH;
        if (boids[i].position_update.y < 0) boids[i].position_update.y += SCREEN_HEIGHT;
        if (boids[i].position_update.y > SCREEN_HEIGHT) boids[i].position_update.y -= SCREEN_HEIGHT;
    }

    // Move predator
    int pred = MAX_BOIDS;
    boids[pred].position = Vector2Add(boids[pred].position, Vector2Scale(boids[pred].velocity, GetFrameTime() * 60.0f));
    if (boids[pred].position.x < 0 || boids[pred].position.x > SCREEN_WIDTH) boids[pred].velocity.x *= -1;
    if (boids[pred].position.y < 0 || boids[pred].position.y > SCREEN_HEIGHT) boids[pred].velocity.y *= -1;

    // Commit updates
    for (int i = 0; i < MAX_BOIDS; i++) {
        boids[i].velocity = boids[i].velocity_update;
        boids[i].position = boids[i].position_update;
    }

    clear_spatial_hash();
    for (int i = 0; i < MAX_BOIDS; i++) {
        insert_boid(&boids[i]);
    }
    //insert_boid(&boids[MAX_BOIDS]);
}

void UpdateBoids()
{
    // Parallel update stage
    #pragma omp parallel for schedule(static)
    for (int i = 0; i < MAX_BOIDS; i++) {
        boids[i].predated = false;
        boids[i].velocity_update = boids[i].velocity;
        boids[i].position_update = boids[i].position;

        Vector2 alignment = {0};
        Vector2 cohesion = {0};
        Vector2 separation = {0};

        int neighborCount = 0;

        // You must copy the neighbor list locally because it's dynamically allocated
        BoidNode* neighbors = find_neighbors(&boids[i]);
        int nLength = length(neighbors);
        if (nLength > MAX_NEIGHBORS) {
            MAX_NEIGHBORS = nLength;
        }
        BoidNode* iter = neighbors;
        while(iter) {
            Boid* neighbor = iter->boid;
            float dist = Vector2Distance(boids[i].position, neighbor->position);

            if (dist < PROTECTED_RADIUS) {
                Vector2 diff = Vector2Subtract(boids[i].position, neighbor->position);
                if (dist != 0) diff = Vector2Scale(diff, 1.0f/dist);
                separation = Vector2Add(separation, diff);
            } else if (dist < NEIGHBOR_RADIUS) {
                alignment = Vector2Add(alignment, neighbor->velocity);
                cohesion = Vector2Add(cohesion, neighbor->position);
                neighborCount++;
            }
            iter = iter->next;
        }
        free_boid_node(neighbors);

        // Predator avoidance
        Vector2 predatorVec = Vector2Subtract(boids[i].position, boids[MAX_BOIDS].position);
        float distToPredator = Vector2Length(predatorVec);
        if (distToPredator < PREDATOR_RADIUS) {
            boids[i].predated = true;
            if (distToPredator != 0) predatorVec = Vector2Scale(predatorVec, PREDATOR_AVOID_FACTOR/distToPredator);
            boids[i].velocity_update = Vector2Add(boids[i].velocity_update, predatorVec);
        }

        if (neighborCount > 0) {
            alignment = Vector2Scale(alignment, 1.0f/neighborCount);
            Vector2 align_force = Vector2Subtract(alignment, boids[i].velocity);
            boids[i].velocity_update = Vector2Add(boids[i].velocity_update, Vector2Scale(align_force, MATCH_FACTOR));

            cohesion = Vector2Scale(cohesion, 1.0f/neighborCount);
            Vector2 cohesion_force = Vector2Subtract(cohesion, boids[i].position);
            boids[i].velocity_update = Vector2Add(boids[i].velocity_update, Vector2Scale(cohesion_force, CENTER_FACTOR));
        }

        boids[i].velocity_update = Vector2Add(boids[i].velocity_update, Vector2Scale(separation, AVOID_FACTOR));

        // Speed limiting
        float speed = Vector2Length(boids[i].velocity_update);
        if (speed > MAX_SPEED) boids[i].velocity_update = Vector2Scale(Vector2Normalize(boids[i].velocity_update), MAX_SPEED);
        if (speed < MIN_SPEED) boids[i].velocity_update = Vector2Scale(Vector2Normalize(boids[i].velocity_update), MIN_SPEED);

        boids[i].position_update = Vector2Add(boids[i].position, Vector2Scale(boids[i].velocity_update, GetFrameTime() * 60.0f));

        // Wrap around screen
        if (boids[i].position_update.x < 0) boids[i].position_update.x += SCREEN_WIDTH;
        if (boids[i].position_update.x > SCREEN_WIDTH) boids[i].position_update.x -= SCREEN_WIDTH;
        if (boids[i].position_update.y < 0) boids[i].position_update.y += SCREEN_HEIGHT;
        if (boids[i].position_update.y > SCREEN_HEIGHT) boids[i].position_update.y -= SCREEN_HEIGHT;
    }

    // Serial predator update
    int pred = MAX_BOIDS;
    boids[pred].position = Vector2Add(boids[pred].position, Vector2Scale(boids[pred].velocity, GetFrameTime() * 60.0f));
    if (boids[pred].position.x < 0 || boids[pred].position.x > SCREEN_WIDTH) boids[pred].velocity.x *= -1;
    if (boids[pred].position.y < 0 || boids[pred].position.y > SCREEN_HEIGHT) boids[pred].velocity.y *= -1;

    // Serial commit and spatial hash rebuild
    clear_spatial_hash();
    for (int i = 0; i < MAX_BOIDS; i++) {
        boids[i].velocity = boids[i].velocity_update;
        boids[i].position = boids[i].position_update;
        insert_boid(&boids[i]);
    }
    insert_boid(&boids[MAX_BOIDS]);
}


int number_drawn = 0;
void DrawBoid(Boid boid) {
    number_drawn++;
    // Normalize velocity to get direction
    Vector2 dir = Vector2Normalize(boid.velocity);

    // Draw main circle
    DrawCircleLinesV(boid.position, PROTECTED_RADIUS/2.0, boid.predated ? GREEN : RED);

    // Draw center dot
    DrawCircleV(boid.position, 2.0f, DARKGRAY);

    // Compute tail endpoint (outside of the circle)
    Vector2 tailDir = Vector2Scale(dir, -(10.0f + 10)); // 10 pixels past edge
    Vector2 tailEnd = Vector2Add(boid.position, tailDir);

    // Draw tail line
    DrawLineV(boid.position, tailEnd, RED);
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

void DrawBoids()
{
    number_drawn = 0;
    for (int i = 0; i <= MAX_BOIDS; i++) {
        if (boids[i].isPredator) {
            DrawPreditor(boids[i]);
        } else {
            //DrawCircleV(boids[i].position, BOID_RADIUS, boids[i].predated ? GREEN : RED);
            DrawBoid(boids[i]);
            //DrawText(TextFormat("%d", i), boids[i].position.x, boids[i].position.y - 10, 10, DARKGRAY);
        }
    }
}



