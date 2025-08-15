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


#define PPM 100.0f
// Raylib_Helper.c
char debug_text[512];
int FrameCount = 1;
int DebugUpdateRate = 10; 
float FrameTimeMS = -1.0f; 
float FrameRate = -1.0f; 

void DrawDebugMenu(float ox, float oy);


// SharedHelper.h 
Vector2 origin = (Vector2){0,0};
b2WorldId worldId;
float timeStep = 1.0f / 60.0f;
int subStepCount = 4;
float gravity_y = -10.f;

Font debugFont;
Vector2 windowRes;

struct r2d_rect{
    b2BodyId id;
    b2Vec2 halfExtents;
};
typedef struct r2d_rect Rect;

Vector2 worldPosToScreenPos(b2Vec2 worldPos){
    Vector2 screenPos;
    screenPos.x = worldPos.x*PPM;
    screenPos.y = windowRes.y - (worldPos.y*PPM);
    return screenPos;
}
// sy = W - wy*P
// wy*P = W - sy
// wy = (W-sy)/P
b2Vec2 screenPosToWorldPos(float sx, float sy){
    b2Vec2 worldPos = {
        worldPos.x = sx/PPM,
        worldPos.y = (windowRes.y - sy)/PPM,
    };
    return worldPos;
}

void DrawRect(Rect rect){
    // box2d expects top left position.
    // rect.p is the middle
    //
    b2Transform tr = b2Body_GetTransform(rect.id);
    // center = x,y
    // top left = x-w, y-h
    b2Vec2 worldPosTopLeft = (b2Vec2){
        .x = tr.p.x - rect.halfExtents.x,
        .y = worldPosTopLeft.y = tr.p.y + rect.halfExtents.y
    };
    Vector2 rScreenPos = worldPosToScreenPos(worldPosTopLeft);
    Rectangle screenRec = (Rectangle){
        .x = rScreenPos.x,
        .y = rScreenPos.y,
        .width = (rect.halfExtents.x*2) * PPM,
        .height = (rect.halfExtents.y*2) * PPM,
    };
    printf("    DRAWING RECT [w:%02f, h:%02f] @ (x:%02f, y:%02f)\n",screenRec.width, screenRec.height, screenRec.x, screenRec.y);
    printf("    REAL RECTPOS [w:%02f, h:%02f] @ (x:%02f, y:%02f)\n",rect.halfExtents.x*2, rect.halfExtents.y*2, tr.p.x, tr.p.y);
    DrawRectanglePro(screenRec, origin, b2Rot_GetAngle(tr.q), WHITE);
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

Rect CreateRect(b2Vec2 pos, float width, float height, float density, float friction, bool isDynamic){
    b2BodyDef bodyDef = b2DefaultBodyDef();
    b2Vec2 halfExtents = (b2Vec2){width/2.0f, height/2.0f};

    if (isDynamic)
        bodyDef.type = b2_dynamicBody;
    else 
        bodyDef.type = b2_staticBody;

    bodyDef.position = (b2Vec2){pos.x, pos.y};
    b2BodyId bodyId = b2CreateBody(worldId, &bodyDef);

    b2Polygon dynamicBox = b2MakeBox(halfExtents.x, halfExtents.y);
    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.density = density;
    shapeDef.material.friction = friction; 
    b2CreatePolygonShape(bodyId, &shapeDef, &dynamicBox);
    return (Rect){.id = bodyId, .halfExtents=halfExtents};
}

// ----------------------
// ------MAIN FILE-------
// ----------------------

int main(){
//b2setup()

    worldId = InitWorld(gravity_y);
    float side_len = 1.0f;
    Rect rectangles [100]; 
    rectangles[0] = CreateRect((b2Vec2){10.0f, 10.0f}, side_len, side_len, 1.0f, 0.3f, true); // isDynamic);
    rectangles[1]= CreateRect((b2Vec2){0.0f, 2.0f}, 50.0f, 2.0f, 1.0f, 0.3f, false); // isDynamic);
    int rectCount = 2;



//raysetup()
    InitWindow(800, 400, "RayBox2D");
    SetTargetFPS(120);
    ToggleBorderlessWindowed();
    
    windowRes = (Vector2){GetScreenWidth(), GetScreenHeight()};
    debugFont = LoadFont("fonts/0xProtoNerdFont-Regular.ttf");

    while (!WindowShouldClose()) {
        // do physics updates
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            b2Vec2 pos = screenPosToWorldPos(GetMouseX(), GetMouseY());
            rectangles[rectCount] = CreateRect(pos, side_len, side_len, 1.0f, 0.3f, true); // isDynamic);
            rectCount++;
        }
        b2World_Step(worldId, timeStep, subStepCount);

        // do draw updates
        BeginDrawing();
        DrawDebugMenu(10,10);

        //DrawCircle(GetMouseX(), GetMouseY(), 20.0f, WHITE);

        for (int i = 0; i<rectCount; i++)
            DrawRect(rectangles[i]);

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
