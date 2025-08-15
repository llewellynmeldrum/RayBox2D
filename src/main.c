#include <stdbool.h>
#include <stdio.h>
#include "raylib.h"
//#include "debug.h"
//
//
//
//
//NOW BEGINS THE BOX 2D IMPORT.

Font PROTO_NERD_REG;
char debug_text[512];
int FrameCount = 1;
int DebugUpdateRate = 10; // every ten frames, update debug menu. if i was bothered id make it so we average out over this many frames but I really dont care that much
float FrameTimeMS = -1.0f; 
float FrameRate = -1.0f; 

void DrawDebugMenu(float ox, float oy);
int main(){

    const int screenWidth = 800;
    const int screenHeight = 450;

    InitWindow(screenWidth, screenHeight, "raylib [core] example - basic window");
    ToggleBorderlessWindowed();

    PROTO_NERD_REG = LoadFont("fonts/0xProtoNerdFont-Regular.ttf");
    if (IsFontValid(PROTO_NERD_REG)==false){
        fprintf(stderr,"\tError! Unable to load custom font, falling back to default.\n"); 
    }


    SetTargetFPS(120);
    while (!WindowShouldClose()) {
        BeginDrawing();
        DrawDebugMenu(10,10);

        DrawCircle(GetMouseX(), GetMouseY(), 20.0f, WHITE);
        ClearBackground(BLACK);


        if (FrameCount%DebugUpdateRate==0){
            FrameTimeMS = GetFrameTime()*1000.0f;
            FrameRate = 1000.0f/FrameTimeMS;
            sprintf(debug_text, "frametime:%.3fms\nframerate:%.1ffps", FrameTimeMS, FrameRate);
        }
        FrameCount++;
        EndDrawing();
    }

    CloseWindow(); 
    return 0;
}

void DrawDebugMenu(float ox, float oy){
    const float fontsize = 24.0f;
    const float spacing = 1.0f;
    Vector2 pos = (Vector2){ox, oy};


    DrawTextEx(PROTO_NERD_REG, debug_text, pos, fontsize, spacing, WHITE); 
}
