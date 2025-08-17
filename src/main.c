#include <stdbool.h>
#include <assert.h>
#include <stdio.h>
#include "raylib.h"
#include "box2d/box2d.h"
#include "box2d/math_functions.h"
#include "box2d/id.h"
#include "box2d/collision.h"
#include "box2d/types.h"
#include <sys/time.h>

#define RED_TRANSLUCENT (Color){0xFF, 0x00, 0x00, 0x40}


// d = r*180/pi
// d*pi = r*180
// (d*pi)/180 = r
#define RAD_TO_DEG (180.0f/B2_PI)
#define DEG_TO_RAD (B2_PI/180.0f)
#define PPM 50.0f
#define MAX_BOXES 1000
#define MAX_JOINTS 100
#define MAX_LAYOUT_BOXES 100
#define SPAWN_COOLDOWN_MS 100.0f

#define SPAWNABLE_BOX_SIZE (b2BoxScale){0.1f, 0.1f} 
#define BOX_DENSITY 1.0f
#define BOX_FRICTION 0.3f

#define IS_DYNAMIC 1
#define IS_STATIC 0  

#define BALL_COUNT 0

#define recordTime(t) gettimeofday(&t, NULL);

#define AUTOPAUSE 0
bool SimulationPaused = AUTOPAUSE;

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

typedef struct box{
    b2BodyId id;
    b2Vec2 hExtent;
}Box;

typedef struct ball{
    b2BodyId id;
    float radius;
}Ball;

typedef struct revJoint{
    b2JointId id;

}Joint;

Vector2 worldToScreen(b2Vec2 worldPos){
    Vector2 screenPos;
    screenPos.x = worldPos.x*PPM;
    screenPos.y = windowSize.y - (worldPos.y*PPM);
    return screenPos;
}
// sy = W - wy*P
// wy*P = W - sy
// wy = (W-sy)/P
b2Vec2 screenToWorld(float sx, float sy){
    b2Vec2 worldPos = {
        worldPos.x = sx/PPM,
        worldPos.y = (windowSize.y - sy)/PPM,
    };
    return worldPos;
}
b2Vec2 screenToWorldV(Vector2 s){
    b2Vec2 worldPos = {
        worldPos.x = s.x/PPM,
        worldPos.y = (windowSize.y - s.y)/PPM,
    };
    return worldPos;
}

void DrawBBoxLines(Box rect){

    b2Vec2 tl_offset = (b2Vec2){-rect.hExtent.x, -rect.hExtent.y};
    b2Vec2 br_offset = (b2Vec2){rect.hExtent.x, rect.hExtent.y};
    b2Vec2 tr_offset = (b2Vec2){rect.hExtent.x, -rect.hExtent.y};
    b2Vec2 bl_offset = (b2Vec2){-rect.hExtent.x, rect.hExtent.y};


    b2Vec2 tl = b2Body_GetWorldPoint(rect.id, tl_offset);
    b2Vec2 br = b2Body_GetWorldPoint(rect.id, br_offset);
    b2Vec2 tr = b2Body_GetWorldPoint(rect.id, tr_offset);
    b2Vec2 bl = b2Body_GetWorldPoint(rect.id, bl_offset);

    Vector2 bl_pos = worldToScreen(tl); // TOP LEFT IN B2D = BOTTOM LEFT IN RLIB
    Vector2 tr_pos = worldToScreen(br);
    Vector2 br_pos = worldToScreen(tr);
    Vector2 tl_pos = worldToScreen(bl);

    DrawLineEx(bl_pos, tr_pos, 2.0f, BLACK); // bot left --> top right
    DrawLineEx(br_pos, tl_pos, 2.0f, BLACK); // bot right --> top left

}

void DrawBox(Box rect){
    b2Transform tr = b2Body_GetTransform(rect.id);
    float b2Rad = b2Rot_GetAngle(tr.q);

    // for screen space=down, also rl likes degrees not rad
    float rl_Deg = -(b2Rad * RAD_TO_DEG);

    
    // rotate about center
    float w = 2.0f * rect.hExtent.x * PPM;
    float h = 2.0f * rect.hExtent.y * PPM;

    Vector2 pos = worldToScreen(tr.p);
    Rectangle rl_Rec = (Rectangle){
        .x = pos.x,
        .y = pos.y, 
        .width = w, 
        .height = h
    };
    Vector2 rl_RotOrigin = (Vector2){w * 0.5f, h*0.5f};

    DrawRectanglePro(rl_Rec, rl_RotOrigin, rl_Deg, WHITE);
    //DrawBBoxLines(rect);
}

void DrawBall(Ball b){
    b2Transform tr = b2Body_GetTransform(b.id);
    Vector2 pos = worldToScreen(tr.p);
    DrawCircleV(pos,b.radius, RED);
}

void DrawJoint(Joint j){
    b2Transform tr_localFrameA = b2Joint_GetLocalFrameA(j.id);
    b2Vec2 ws_bodyA = b2Body_GetTransform(b2Joint_GetBodyA(j.id)).p;
//    printf("\t ws_localFrameA:  %02f,%02f\n", ws_localFrameA.x, ws_localFrameA.y);
 //   printf("\t ws_bodyA:  %02f,%02f\n", tr_bodyA.p.x, tr_bodyA.p.y);
    

    b2Vec2 ws_AnchorPointA = b2TransformPoint(tr_localFrameA, ws_bodyA);
    
    Vector2 pos = worldToScreen(ws_AnchorPointA);
    //printf("\t pos: %02f,%02f\n",pos.x,pos.y);
    DrawCircleV(pos, 10.0f, RED_TRANSLUCENT); 
}

void DrawPointWS(b2Vec2 p, float rad, Color c){
    DrawCircleV(worldToScreen(p),rad, c);
}

// B2D_Helper.c 

b2WorldId InitWorld(float grav_y){
    b2WorldDef worldDef = b2DefaultWorldDef();
    worldDef.gravity = (b2Vec2){0.0f, grav_y};
    return b2CreateWorld(&worldDef);
}

typedef struct scaleHelper{
    // make function calls a bit less cumbersome
    float width;
    float height;
}b2BoxScale;

Box CreateBox(b2Vec2 pos, b2BoxScale scale, bool isDynamic){
    b2BodyDef bodyDef = b2DefaultBodyDef();
    b2Vec2 hExtent = (b2Vec2){scale.width/2.0f, scale.height/2.0f};

    if (isDynamic)
        bodyDef.type = b2_dynamicBody;
    else 
        bodyDef.type = b2_staticBody;

    bodyDef.position = (b2Vec2){pos.x, pos.y};
    b2BodyId bodyId = b2CreateBody(worldId, &bodyDef);

    b2Polygon dynamicBox = b2MakeBox(hExtent.x, hExtent.y);
    b2ShapeDef shapeDef = b2DefaultShapeDef();

    if (scale.width==SPAWNABLE_BOX_SIZE.width
    &&  scale.height==SPAWNABLE_BOX_SIZE.height) shapeDef.density = BOX_DENSITY;
    else shapeDef.density = 1.0f;
    

    shapeDef.material.friction = BOX_FRICTION; 
    b2CreatePolygonShape(bodyId, &shapeDef, &dynamicBox);
    return (Box){.id = bodyId, .hExtent=hExtent};
}

Box Boxes[MAX_BOXES]; int BoxCount = 0;
Box LayoutBoxes[MAX_LAYOUT_BOXES]; int LayoutBoxCount = 0;
Ball Balls[BALL_COUNT];
Joint Joints[MAX_JOINTS]; int JointCount= 0;

Ball CreateBall(b2Vec2 pos, float radius, bool isDynamic){
    b2BodyDef bodyDef = b2DefaultBodyDef();

    if (isDynamic)
        bodyDef.type = b2_dynamicBody;
    else 
        bodyDef.type = b2_staticBody;

    bodyDef.position = (b2Vec2){pos.x, pos.y};
    b2BodyId bodyId = b2CreateBody(worldId, &bodyDef);

    b2Circle circle = {
        .center= {0.0, 0.0f}, 
        .radius=radius
    };
    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.density = BOX_DENSITY;
    shapeDef.material.friction = BOX_FRICTION; 
    b2CreateCircleShape(bodyId, &shapeDef, &circle);
    return (Ball){.id = bodyId, .radius=radius};
}


void AttemptSpawnBox(Vector2 mousePos){
        b2Vec2 WorldMousePos = screenToWorldV(mousePos);
        recordTime(t_SpawnAttempt);
        if (BoxCount<MAX_BOXES){
            bool cooldownElapsed = timeDiff(t_LastSpawn, t_SpawnAttempt)>SPAWN_COOLDOWN_MS; 
            if (cooldownElapsed){ 
                Boxes[BoxCount] = CreateBox(WorldMousePos, SPAWNABLE_BOX_SIZE, IS_DYNAMIC); // isDynamic);
                BoxCount++;
                recordTime(t_LastSpawn);
            }
        }   
}

b2JointId CreateDefaultJointBetween(b2BodyId id_a, b2BodyId id_b, b2Vec2 pivot){
    b2Transform atr = b2Body_GetTransform(id_a); 
    b2RevoluteJointDef def = b2DefaultRevoluteJointDef();
    def.base.bodyIdA = id_a;
    def.base.bodyIdB = id_b;
    def.base.localFrameA.p = b2Body_GetLocalPoint(id_a, pivot);
    def.base.localFrameB.p = b2Body_GetLocalPoint(id_b, pivot);
    def.lowerAngle = -45.0f * DEG_TO_RAD;
    def.upperAngle = 90.0f * DEG_TO_RAD;
    def.enableLimit = true;
    b2JointId jointId = b2CreateRevoluteJoint(worldId, &def);
    Joints[JointCount++] = (Joint){.id=jointId};
    return jointId;
}
// ----------------------
// ------MAIN FILE-------
// ----------------------

void HandleInput(){
        Vector2 mousePos = {GetMouseX(), GetMouseY()};
        b2Vec2 wmouse = screenToWorldV(mousePos);
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) AttemptSpawnBox(mousePos);

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))  printf("WMOUSE: %f,%f\n",wmouse.x,wmouse.y);

        if(IsKeyPressed(KEY_P)) SimulationPaused=!SimulationPaused;
    
}
void HandleDrawing(){
        ClearBackground(BLACK);
        DrawDebugMenu(15,15);


        for (int i = 0; i<LayoutBoxCount; i++) DrawBox(LayoutBoxes[i]);

        for (int i = 0; i<BoxCount; i++) DrawBox(Boxes[i]);

        for (int i = 0; i<BALL_COUNT; i++) DrawBall(Balls[i]);

        for (int i = 0; i<JointCount; i++) DrawJoint(Joints[i]);

        if (FrameCount%DebugUpdateRate==0){
            FrameTimeMS = GetFrameTime()*1000.0f;
            FrameRate = 1000.0f/FrameTimeMS;
            sprintf(debug_text, "frames:%d\nframetime:%.3fms\nframerate:%.1ffps\nboxcount:%d/%d\nsimpaused:%d",\
                    FrameCount,\
                    FrameTimeMS,\
                    FrameRate,\
                    BoxCount,MAX_BOXES,
                    SimulationPaused);
        }

        FrameCount++;
}


void AddLayoutBox(b2Vec2 pos, b2BoxScale scale, bool isStatic){
    if (LayoutBoxCount < MAX_LAYOUT_BOXES)
        LayoutBoxes[LayoutBoxCount++] = CreateBox(pos, scale, isStatic);
}
void AddLayoutGeometry(b2Vec2 worldSize){
    b2BoxScale floorCeil_Scale = (b2BoxScale){worldSize.x, 0.5f};
    // floor
    AddLayoutBox((b2Vec2)   {worldSize.x/2.0f , 1.0f},
                (b2BoxScale){worldSize.x, 0.5f},
                IS_STATIC);
    //roof
    AddLayoutBox((b2Vec2)   {worldSize.x/2.0f, worldSize.y},
                (b2BoxScale){worldSize.x, 0.5f},
                 IS_STATIC);


    // left wall
    AddLayoutBox((b2Vec2)    {0.0f, worldSize.y/2.0f},
                 (b2BoxScale){0.5f, worldSize.y},
                 IS_STATIC);
    // right wall
    AddLayoutBox((b2Vec2)    {worldSize.x, worldSize.y/2.0f},
                 (b2BoxScale){0.5f, worldSize.y},
                 IS_STATIC);

    // domino platform
    AddLayoutBox((b2Vec2)    {24.7f, 4.3f},
                 (b2BoxScale){22.0f, 6.0f},
                 IS_STATIC);


    b2Vec2 seesawPivot = (b2Vec2){7.2f, 10.0f};

    // Seesaw pillar
    AddLayoutBox((b2Vec2)    {seesawPivot.x, 5.0f}, 
                 (b2BoxScale){1.0f,10.0f},
                 IS_STATIC);

    // seesaw platform
    AddLayoutBox((b2Vec2)    {seesawPivot.x, seesawPivot.y},
                 (b2BoxScale){13.5f,1.0f},
                 IS_DYNAMIC);

    CreateDefaultJointBetween(LayoutBoxes[LayoutBoxCount-2].id, LayoutBoxes[LayoutBoxCount-1].id, seesawPivot);
}

int main(){
//raysetup()
    InitWindow(800, 400, "RayBox2D");
    SetTargetFPS(120);
    ToggleBorderlessWindowed();
    
    windowSize = (Vector2){GetScreenWidth(), GetScreenHeight()};
    debugFont = LoadFont("fonts/0xProtoNerdFont-Regular.ttf");

//b2setup()
    b2Vec2 worldSize = (b2Vec2){windowSize.x/PPM, windowSize.y/PPM};
    float gravity_y = -10.f;
    worldId = InitWorld(gravity_y);
    AddLayoutGeometry(worldSize);

    while (!WindowShouldClose()) {
        HandleInput();
        if (!SimulationPaused) b2World_Step(worldId, timeStep, subStepCount);
        BeginDrawing();
        HandleDrawing();
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
