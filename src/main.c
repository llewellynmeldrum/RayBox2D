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
#include <sys/time.h>


#define PPM 50.0f
#define MAX_BOXES 1000
#define BOX_SIZE (b2Vec2){0.5f, 0.5f} 
#define SPAWN_COOLDOWN_MS 100.0f

#define recordTime(t) gettimeofday(&t, NULL);

typedef struct timeval timeval;
timeval t_LastSpawn, t_SpawnAttempt;

unsigned long timeDiff(timeval start, timeval stop){
    unsigned long secDiff = stop.tv_sec - start.tv_sec;
    unsigned long usecDiff = stop.tv_usec - start.tv_usec;
    unsigned long ms= (secDiff*1000) + (usecDiff/1000);
    return ms;
}

// Raylib_Helper.c
char debug_text[512];
int FrameCount = 1;
int DebugUpdateRate = 10; 
float FrameTimeMS = -1.0f; 
float FrameRate = -1.0f; 

void DrawDebugMenu(float ox, float oy);


// SharedHelper.h 
b2WorldId worldId;
float timeStep = 1.0f / 60.0f;
int subStepCount = 4;
//float gravity_y = -10.f;

Font debugFont;
Vector2 windowSize;

struct r2d_rect{
    b2BodyId id;
    b2Vec2 halfExtents;
};
typedef struct r2d_rect Rect;

Vector2 worldPosToScreenPos(b2Vec2 worldPos){
    Vector2 screenPos;
    screenPos.x = worldPos.x*PPM;
    screenPos.y = windowSize.y - (worldPos.y*PPM);
    return screenPos;
}
// sy = W - wy*P
// wy*P = W - sy
// wy = (W-sy)/P
b2Vec2 screenPosToWorldPos(float sx, float sy){
    b2Vec2 worldPos = {
        worldPos.x = sx/PPM,
        worldPos.y = (windowSize.y - sy)/PPM,
    };
    return worldPos;
}

float radToDeg(float rad){
    return rad * (180.0f/PI);
}

void DrawRect(Rect rect){
    b2Transform tr = b2Body_GetTransform(rect.id);
    float b2Rad = b2Rot_GetAngle(tr.q);

    // for screen space=down, also rl likes degrees not rad
    float rl_Deg = -(radToDeg(b2Rad));

    
    // rotate about center
    float w = 2.0f * rect.halfExtents.x * PPM;
    float h = 2.0f * rect.halfExtents.y * PPM;

    Vector2 pos = worldPosToScreenPos(tr.p);
    Rectangle rl_Rec = (Rectangle){
        .x = pos.x,
        .y = pos.y, 
        .width = w, 
        .height = h
    };
    Vector2 rl_RotOrigin = (Vector2){w * 0.5f, h*0.5f};

    DrawRectanglePro(rl_Rec, rl_RotOrigin, rl_Deg, WHITE);
}

void DrawRectLines(Rect rect){

    b2Vec2 tl_offset = (b2Vec2){-rect.halfExtents.x, -rect.halfExtents.y};
    b2Vec2 br_offset = (b2Vec2){rect.halfExtents.x, rect.halfExtents.y};
    b2Vec2 tr_offset = (b2Vec2){rect.halfExtents.x, -rect.halfExtents.y};
    b2Vec2 bl_offset = (b2Vec2){-rect.halfExtents.x, rect.halfExtents.y};


    b2Vec2 tl = b2Body_GetWorldPoint(rect.id, tl_offset);
    b2Vec2 br = b2Body_GetWorldPoint(rect.id, br_offset);
    b2Vec2 tr = b2Body_GetWorldPoint(rect.id, tr_offset);
    b2Vec2 bl = b2Body_GetWorldPoint(rect.id, bl_offset);

    Vector2 bl_pos = worldPosToScreenPos(tl); // TOP LEFT IN B2D = BOTTOM LEFT IN RLIB
    Vector2 tr_pos = worldPosToScreenPos(br);
    Vector2 br_pos = worldPosToScreenPos(tr);
    Vector2 tl_pos = worldPosToScreenPos(bl);

    DrawLineEx(bl_pos, tr_pos, 5.0f, BLACK); // bot left --> top right
    DrawLineEx(br_pos, tl_pos, 5.0f, BLACK); // bot right --> top left

}
void DrawPointWS(b2Vec2 p, float rad, Color c){
    DrawCircleV(worldPosToScreenPos(p),rad, c);
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
//raysetup()
    InitWindow(800, 400, "RayBox2D");
    SetTargetFPS(120);
    ToggleBorderlessWindowed();
    
    windowSize = (Vector2){GetScreenWidth(), GetScreenHeight()};
    debugFont = LoadFont("fonts/0xProtoNerdFont-Regular.ttf");

//b2setup()

    b2Vec2 worldSize = (b2Vec2){windowSize.x/PPM, windowSize.y/PPM};
    printf("WorldSize:%f,%f\n",worldSize.x, worldSize.y);
    float gravity_y = -10.f;
    worldId = InitWorld(gravity_y);
    float side_len = 1.0f;
    Rect rectangles [MAX_BOXES]; 

    float floorWidth = worldSize.x*2;
    float floorHeight = 0.5f;
    rectangles[0]= CreateRect((b2Vec2){0.0f , 0.0f}, floorWidth , floorHeight, 1.0f, 0.3f, false); // isDynamic);
    // floor 
    rectangles[1]= CreateRect((b2Vec2){0.0f, worldSize.y+floorHeight}, floorWidth, floorHeight, 1.0f, 0.3f, false); // isDynamic);
    // roof 
    int boxCount = 2;



    while (!WindowShouldClose()) {
        // handle input
        b2Vec2 WorldMousePos = screenPosToWorldPos(GetMouseX(), GetMouseY());
        Vector2 mousePos = {GetMouseX(), GetMouseY()};
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            recordTime(t_SpawnAttempt);
            bool spawnCooldownElapsed = timeDiff(t_LastSpawn, t_SpawnAttempt)>SPAWN_COOLDOWN_MS; 
            if (spawnCooldownElapsed && boxCount<MAX_BOXES){
                recordTime(t_LastSpawn);
                rectangles[boxCount] = CreateRect(WorldMousePos, BOX_SIZE.x, BOX_SIZE.y, 1.0f, 0.3f, true); // isDynamic);
                boxCount++;
            }
        }

        // do physics updates
        b2World_Step(worldId, timeStep, subStepCount);

        // do draw updates
        BeginDrawing();
        ClearBackground(BLACK);
        DrawDebugMenu(10,10);

        for (int i = 0; i<boxCount; i++){
            DrawRect(rectangles[i]);
            //DrawRectLines(rectangles[i]);
        }

        if (FrameCount%DebugUpdateRate==0){
            FrameTimeMS = GetFrameTime()*1000.0f;
            FrameRate = 1000.0f/FrameTimeMS;
            sprintf(debug_text, "frametime:%.3fms\nframerate:%.1ffps\nboxcount:%d", FrameTimeMS, FrameRate,boxCount);
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
