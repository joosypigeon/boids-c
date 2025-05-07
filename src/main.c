
#include <stdlib.h>

#include "boids.h"

int SCREEN_WIDTH;
int SCREEN_HEIGHT;

int main(void)
{


    SetConfigFlags(FLAG_FULLSCREEN_MODE);
    InitWindow(0, 0, "Fullscreen at Desktop Resolution");

    // Get the primary monitor's resolution before window creation
    int monitor = GetCurrentMonitor();
    SCREEN_WIDTH = GetMonitorWidth(monitor);
    SCREEN_HEIGHT = GetMonitorHeight(monitor);


    SetTargetFPS(60);

    InitBoids();

    while (!WindowShouldClose())
    {
        UpdateBoids();

        BeginDrawing();
            ClearBackground(RAYWHITE);
            DrawBoids();
            DrawText("Boids with Predator Simulation", 20, 10, 20, DARKGRAY);
            DrawText("Current Resolution:", 20, 30, 20, DARKGRAY);
            DrawText(TextFormat("%d x %d", SCREEN_WIDTH, SCREEN_HEIGHT), 20, 50, 30, BLUE);
            DrawText(TextFormat("Boids drawn: %d", number_drawn), 20, 80, 30, BLUE);
        EndDrawing();
    }

    CloseWindow();

    return 0;
}