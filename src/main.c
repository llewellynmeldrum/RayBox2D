#include <stdbool.h>
#include <stdio.h>
#include "raylib.h"
#include "box2d/box2d.h"
#include "box2d/math_functions.h"
#include "box2d/id.h"
#include "box2d/collision.h"
#include "box2d/types.h"
#include "body.h"
#include "physics_world.h"


// Raylib_Helper.c
char debug_text[512];
int FrameCount = 1;
int DebugUpdateRate = 10; 
float FrameTimeMS = -1.0f; 
float FrameRate = -1.0f; 
void DrawDebugMenu(float ox, float oy);


// SharedHelper.h 
b2WorldId worldId;
float timeStep;
int subStepCount;

Font debugFont;
Vector2 windowRes;

struct r2d_rect{
    b2BodyId id;
    b2Vec2 scale;
};
typedef struct r2d_rect Rect;

void DrawRect(Rect rect){
    b2Transform t = b2Body_GetTransform(rect.id);
    Rectangle rectangle = (Rectangle){
        .x = t.p.x,
        .y = t.p.y,
        .width = rect.scale.x,
        .height = rect.scale.y,
    };
    DrawRectanglePro(rectangle, (Vector2){0,0}, b2Rot_GetAngle(t.q), WHITE);

}

// B2D_Helper.h
b2WorldId InitWorld(float grav_y);
// etc

// B2D_Helper.c 

b2WorldId InitWorld(float grav_y){
    b2WorldDef worldDef = b2DefaultWorldDef();
    worldDef.gravity = (b2Vec2){0.0f, grav_y};
    return b2CreateWorld(&worldDef);
}

Rect CreateRect(b2Vec2 pos, b2Vec2 scale, float density, float friction){
    b2BodyDef bodyDef = b2DefaultBodyDef();
    bodyDef.type = b2_dynamicBody;
    bodyDef.position = (b2Vec2){pos.x, pos.y};
    b2BodyId bodyId = b2CreateBody(worldId, &bodyDef);

    b2Polygon dynamicBox = b2MakeBox(scale.x, scale.y);
    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.density = density;
    shapeDef.material.friction = friction; 
    return (Rect){.id = bodyId, .scale=scale};
}

// ----------------------
// ------MAIN FILE-------
// ----------------------

int main(){
//b2setup()
    timeStep = 1.0f / 60.0f;
    subStepCount = 4;

    worldId = InitWorld(-10.0f);
    Rect rect = CreateRect(
        (b2Vec2){400.0f, 200.0f},
        (b2Vec2){100.0f, 100.0f},
        1.0f,
        0.3f
    );


//raysetup()
    InitWindow(800, 400, "RayBox2D");
    SetTargetFPS(120);
    ToggleBorderlessWindowed();
    
    windowRes = (Vector2){GetScreenWidth(), GetScreenHeight()};
    debugFont = LoadFont("fonts/0xProtoNerdFont-Regular.ttf");

    while (!WindowShouldClose()) {
        // do physics updates
        b2World_Step(worldId, timeStep, subStepCount);

        // do draw updates
        BeginDrawing();
        DrawDebugMenu(10,10);

        DrawCircle(GetMouseX(), GetMouseY(), 20.0f, WHITE);
        DrawRect(rect);
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
}

void DrawDebugMenu(float ox, float oy){
    const float fontsize = 24.0f;
    const float spacing = 1.0f;
    Vector2 pos = (Vector2){ox, oy};


    DrawTextEx(debugFont, debug_text, pos, fontsize, spacing, WHITE); 
}
