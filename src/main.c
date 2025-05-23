#include <stdlib.h>
#include <omp.h>
#include "boids.h"
#include "spatial_hash.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

int SCREEN_WIDTH;
int SCREEN_HEIGHT;
bool drawFullGlyph = false;
bool drawDensity = false;
bool mousePressed = false;
bool nearestNeighboursNetwork = false;
bool pauseSimulation = false;
Boid *debugBoid = NULL;

int main(void)
{
    

    SetConfigFlags(FLAG_FULLSCREEN_MODE);
    InitWindow(0, 0, "Fullscreen at Desktop Resolution");

    // Get the primary monitor's resolution before window creation
    int monitor = GetCurrentMonitor();
    SCREEN_HEIGHT = GetMonitorHeight(monitor);
    SCREEN_WIDTH = GetMonitorWidth(monitor);
    printf("Monitor %d: %d x %d\n", monitor, SCREEN_WIDTH, SCREEN_HEIGHT);
    SCREEN_WIDTH = (SCREEN_WIDTH/CELL_SIZE)*CELL_SIZE;
    SCREEN_HEIGHT = (SCREEN_HEIGHT/CELL_SIZE)*CELL_SIZE;
    printf("Monitor %d: %d x %d\n", monitor, SCREEN_WIDTH, SCREEN_HEIGHT);


    SetTargetFPS(60);

    InitBoids();

    static float alignmentWeight = 1.0f;
    static float cohesionWeight = 1.0f;
    static float separationWeight = 1.0f;

    while (!WindowShouldClose())
    {
        if (IsKeyPressed(KEY_SPACE)) pauseSimulation = !pauseSimulation;
        if (!pauseSimulation) UpdateBoids(alignmentWeight, cohesionWeight, separationWeight);

        if(IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)){
            if(debugBoid) debugBoid = NULL;
            else debugBoid = FindNearestBoid(GetMousePosition());
        }

        BeginDrawing();
            if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
                mousePressed = true;
                boids[MOUSE_INDEX].position = GetMousePosition();
            } else {
                mousePressed = false;
                boids[MOUSE_INDEX].position = (Vector2){ -1.0f, -1.0f };
            }
            ClearBackground(RAYWHITE);
            DrawBoids();
            if(debugBoid) DrawCells(debugBoid->position);
            if(nearestNeighboursNetwork) DrawNearestNeighborNetwork();
            DrawText("Boids with Predator Simulation", 20, 10, 20, DARKGRAY);
            DrawText("Current Resolution:", 20, 30, 20, DARKGRAY);
            DrawText(TextFormat("%d x %d", SCREEN_WIDTH, SCREEN_HEIGHT), 20, 50, 30, BLUE);
            DrawText(TextFormat("Boids drawn: %d", number_drawn), 20, 80, 30, BLUE);
            DrawText(TextFormat("Frame Time: %0.2f ms", GetFrameTime() * 1000), 20, 110, 30, BLUE);
            DrawText(TextFormat("OpenMP threads: %d", omp_get_max_threads()), 20, 140, 30, BLUE);

            int oldTextSize = GuiGetStyle(DEFAULT, TEXT_SIZE);
            GuiSetStyle(DEFAULT, TEXT_SIZE, 24);
            GuiCheckBox((Rectangle){ 500, 10, 28, 28 }, "Draw Full Boid Glyph", &drawFullGlyph);
            GuiCheckBox((Rectangle){ 500, 40, 28, 28 }, "Show density", &drawDensity);
            GuiCheckBox((Rectangle){ 500, 70, 28, 28 }, "Show nearest neighbours", &nearestNeighboursNetwork);

            GuiSetStyle(DEFAULT, TEXT_SIZE, oldTextSize);  // Restore to avoid breaking other widgets
            
            DrawFPS(SCREEN_WIDTH - 100, 10);

            // Start the sliders below the text stats
            Rectangle sliderBounds = { 500, 140, 300, 30 };
            float sliderSpacing = 50;

            // Optional: Draw a heading in larger font
            DrawText("Boid Behaviour Weights", sliderBounds.x, sliderBounds.y - 40, 28, DARKGRAY);

            // Make font size larger manually
            int labelFontSize = 22;
            int valueFontSize = 22;

            GuiSlider(sliderBounds,
                TextFormat("Alignment (%.2f)", alignmentWeight),
                NULL,
                &alignmentWeight, 0.0f, 10.0f);
            sliderBounds.y += sliderSpacing;

            GuiSlider(sliderBounds,
                TextFormat("Cohesion (%.2f)", cohesionWeight),
                NULL,
                &cohesionWeight, 0.0f, 10.0f);
            sliderBounds.y += sliderSpacing;

            GuiSlider(sliderBounds,
                TextFormat("Separation (%.2f)", separationWeight),
                NULL,
                &separationWeight, 0.0f, 10.0f);

        

        EndDrawing();
    }

    CloseWindow();

    return 0;
}