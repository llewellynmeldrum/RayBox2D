#include <stdbool.h>
#include <stdlib.h>
#include <assert.h>
#include <stdio.h>
#include "raylib.h"
#include "box2d/box2d.h"
#include "box2d/math_functions.h"
#include "box2d/id.h"
#include "box2d/collision.h"
#include "box2d/types.h"
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include "Timing.h"

#define RED_TRANSLUCENT (Color){0xFF, 0x00, 0x00, 0x40}


// d = r*180/pi
// d*pi = r*180
// (d*pi)/180 = r
#define RAD_TO_DEG (180.0f/B2_PI)
#define DEG_TO_RAD (B2_PI/180.0f)
#define PPM 25.0f
#define MAX_BOXES 10000
#define MAX_JOINTS 100
#define MAX_LAYOUT_BOXES 100
#define SPAWN_COOLDOWN_MS 000.0f

#define LAYOUT_BOX_DENSITY 1.0f
#define LAYOUT_BOX_FRICTION 0.3f

#define DOMINO_DENSITY 10.0f
#define DOMINO_FRICTION 0.3f

#define SPAWNABLE_BOX_SIZE (b2BoxScale){0.5f, 0.5f}
#define SPAWNABLE_BOX_DENSITY 2.0f
#define BALL_DENSITY 1.8f
#define BOX_FRICTION 0.3f

#define IS_DYNAMIC 1
#define IS_STATIC 0

#define YES_LINES 1
#define NO_LINES 0
#define OUTLINE_THICK 1.5f

#define BALL_COUNT 1

#define recordTime(t) gettimeofday(&t, NULL);

#define AUTOPAUSE 1

Timespan GetTimespan(wc_timeval before, wc_timeval after) {
	time_t s_diff = labs(after.tv_sec - before.tv_sec);
	suseconds_t us_diff = abs(after.tv_usec - before.tv_usec);
	//printf("BEFORE: us=%07d s=%ld\n AFTER: us=%07d s=%ld\n", before.tv_usec, before.tv_sec, after.tv_usec, after.tv_sec);
	return (Timespan) {
		.m = ((time_t) s_diff) / SECS_PER_MIN,
		.s = ((time_t) s_diff) % SECS_PER_MIN,
		.ms = ((time_t) us_diff) / USECS_PER_MSEC,
		.us = ((time_t) us_diff) % USECS_PER_MSEC,
	};
}

double TimespanSumMS(Timespan t) {
	double ms = t.ms;
	ms += t.s * 1000;
	ms += t.m * (1000 * 60);
	ms += t.us / 1000.0f;
	return ms;
}

void PrintTimespan(Timespan t) {
	// 03 m 27 s 123 ms 456 µs OR 03m27s (123 ms, 456 µs) OR 03:27:123:456
	printf("%02ld m %02ld s %03ld ms %03ld µs\n", t.m, t.s, t.ms, t.us);
}
bool SimulationPaused = AUTOPAUSE;

typedef struct timeval timeval;
timeval t_LastSpawn, t_SpawnAttempt;

float randf(float min, float max) {
	int imin = (int)min * 100;
	int imax = (int)max * 100;
	return (float)(imin + (rand() % imax - imin)) / 100.0f;
}

unsigned long timeDiff(timeval start, timeval stop) {
	unsigned long secDiff = stop.tv_sec - start.tv_sec;
	unsigned long usecDiff = stop.tv_usec - start.tv_usec;
	unsigned long ms = (secDiff * 1000) + (usecDiff / 1000);
	return ms;
}

// Raylib_Helper.c
char debug_text[512];
int StepCount = 1;
int FrameCount = 1;
int DebugUpdateRate = 10;
float FrameTimeMS = -1.0f;
float FrameRate = -1.0f;

void DrawDebugMenu(float ox, float oy);


// SharedHelper.h
b2WorldId worldId;
float timeStep = 1.0f / 60.0f;
int subStepCount = 2;
//float gravity_y = -10.f;

Font debugFont;
Vector2 windowSize;

typedef struct box {
	b2BodyId id;
	b2Vec2 hExtent;
} Box;

typedef struct ball {
	b2BodyId id;
	float radius;
} Ball;

typedef struct revJoint {
	b2JointId id;
} Joint;


Vector2 worldToScreen(b2Vec2 worldPos) {
	Vector2 screenPos;
	screenPos.x = worldPos.x * PPM;
	screenPos.y = windowSize.y - (worldPos.y * PPM);
	return screenPos;
}

b2Vec2 screenToWorld(float sx, float sy) {
	b2Vec2 worldPos = {
		worldPos.x = sx / PPM,
		worldPos.y = (windowSize.y - sy) / PPM,
	};
	return worldPos;
}

b2Vec2 screenToWorldV(Vector2 s) {
	b2Vec2 worldPos = {
		worldPos.x = s.x / PPM,
		worldPos.y = (windowSize.y - s.y) / PPM,
	};
	return worldPos;
}

void DrawBBoxLines(Box rect) {

	b2Vec2 tl_offset = (b2Vec2) {
		-rect.hExtent.x, -rect.hExtent.y
	};
	b2Vec2 br_offset = (b2Vec2) {
		rect.hExtent.x, rect.hExtent.y
	};
	b2Vec2 tr_offset = (b2Vec2) {
		rect.hExtent.x, -rect.hExtent.y
	};
	b2Vec2 bl_offset = (b2Vec2) {
		-rect.hExtent.x, rect.hExtent.y
	};


	b2Vec2 tl = b2Body_GetWorldPoint(rect.id, tl_offset);
	b2Vec2 br = b2Body_GetWorldPoint(rect.id, br_offset);
	b2Vec2 tr = b2Body_GetWorldPoint(rect.id, tr_offset);
	b2Vec2 bl = b2Body_GetWorldPoint(rect.id, bl_offset);

	Vector2 bl_pos = worldToScreen(tl); // TOP LEFT IN B2D = BOTTOM LEFT IN RLIB
	Vector2 tr_pos = worldToScreen(br);
	Vector2 br_pos = worldToScreen(tr);
	Vector2 tl_pos = worldToScreen(bl);

	DrawLineEx(tl_pos, tr_pos, OUTLINE_THICK, BLACK); // bot left --> top right
	DrawLineEx(tr_pos, br_pos, OUTLINE_THICK, BLACK); // bot left --> top right
	DrawLineEx(br_pos, bl_pos, OUTLINE_THICK, BLACK); // bot right --> top left
	DrawLineEx(bl_pos, tl_pos, OUTLINE_THICK, BLACK); // bot right --> top left

}

void DrawBox(Box rect, Color c, bool drawLines) {
	b2Transform tr = b2Body_GetTransform(rect.id);
	float b2Rad = b2Rot_GetAngle(tr.q);

	// for screen space=down, also rl likes degrees not rad
	float rl_Deg = -(b2Rad * RAD_TO_DEG);


	// rotate about center
	float w = 2.0f * rect.hExtent.x * PPM;
	float h = 2.0f * rect.hExtent.y * PPM;

	Vector2 pos = worldToScreen(tr.p);
	Rectangle rl_Rec = (Rectangle) {
		.x = pos.x,
		.y = pos.y,
		.width = w,
		.height = h
	};
	Vector2 rl_RotOrigin = (Vector2) {
		w * 0.5f, h * 0.5f
	};

	DrawRectanglePro(rl_Rec, rl_RotOrigin, rl_Deg, c);
	DrawBBoxLines(rect);
}

void DrawBall(Ball b) {
	b2Transform tr = b2Body_GetTransform(b.id);
	Vector2 pos = worldToScreen(tr.p);
	DrawCircleV(pos, b.radius * PPM, RED);
}

void DrawJoint(Joint j) {
	b2Transform tr_localFrameA = b2Joint_GetLocalFrameA(j.id);
	b2Vec2 ws_bodyA = b2Body_GetTransform(b2Joint_GetBodyA(j.id)).p;
//    printf("\t ws_localFrameA:  %02f,%02f\n", ws_localFrameA.x, ws_localFrameA.y);
//   printf("\t ws_bodyA:  %02f,%02f\n", tr_bodyA.p.x, tr_bodyA.p.y);


	b2Vec2 ws_AnchorPointA = b2TransformPoint(tr_localFrameA, ws_bodyA);

	Vector2 pos = worldToScreen(ws_AnchorPointA);
	//printf("\t pos: %02f,%02f\n",pos.x,pos.y);
	DrawCircleV(pos, 10.0f, RED_TRANSLUCENT);
}

void DrawPointWS(b2Vec2 p, float rad, Color c) {
	DrawCircleV(worldToScreen(p), rad, c);
}

// B2D_Helper.c

typedef struct scaleHelper {
	// make function calls a bit less cumbersome
	float width;
	float height;
} b2BoxScale;

b2WorldId InitWorld(float grav_y) {
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = (b2Vec2) {
		0.0f, grav_y
	};
	return b2CreateWorld(&worldDef);
}



Box CreateBox(b2Vec2 pos, b2BoxScale scale, float density, float friction, bool isDynamic) {
	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2Vec2 hExtent = (b2Vec2) {
		scale.width / 2.0f, scale.height / 2.0f
	};

	if (isDynamic)
		bodyDef.type = b2_dynamicBody;
	else
		bodyDef.type = b2_staticBody;

	bodyDef.position = (b2Vec2) {
		pos.x, pos.y
	};
	b2BodyId bodyId = b2CreateBody(worldId, &bodyDef);

	b2Polygon dynamicBox = b2MakeBox(hExtent.x, hExtent.y);
	b2ShapeDef shapeDef = b2DefaultShapeDef();

	shapeDef.density = density;
	shapeDef.material.friction = friction;

	b2CreatePolygonShape(bodyId, &shapeDef, &dynamicBox);
	return (Box) {
		.id = bodyId, .hExtent = hExtent
	};
}

Box CreateBoxBot(b2Vec2 pos, b2BoxScale scale, float density, float friction, bool isDynamic) {
	b2Vec2 bot = (b2Vec2) {
		pos.x, pos.y + (scale.height / 2.0f)
	};

	return CreateBox(bot, scale, isDynamic, density, friction);
}

Box Boxes[MAX_BOXES];
int BoxCount = 0;
Box LayoutBoxes[MAX_LAYOUT_BOXES];
int LayoutBoxCount = 0;
Ball Balls[BALL_COUNT];
Joint Joints[MAX_JOINTS];
int JointCount = 0;

Ball CreateBall(b2Vec2 pos, float radius, bool isDynamic) {
	b2BodyDef bodyDef = b2DefaultBodyDef();

	if (isDynamic)
		bodyDef.type = b2_dynamicBody;
	else
		bodyDef.type = b2_staticBody;

	bodyDef.position = (b2Vec2) {
		pos.x, pos.y
	};
	b2BodyId bodyId = b2CreateBody(worldId, &bodyDef);

	b2Circle circle = {
		.center = {0.0, 0.0f},
		.radius = radius
	};
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = BALL_DENSITY;
	shapeDef.material.friction = BOX_FRICTION;
	b2CreateCircleShape(bodyId, &shapeDef, &circle);
	return (Ball) {
		.id = bodyId, .radius = radius
	};
}



void AttemptSpawnBox(b2Vec2 worldPos) {
	const int spawnperclick = 2;
	recordTime(t_SpawnAttempt);
	if (BoxCount < MAX_BOXES) {
		bool cooldownElapsed = timeDiff(t_LastSpawn, t_SpawnAttempt) > SPAWN_COOLDOWN_MS;
		if (cooldownElapsed) {
			for (int i = 0; i < spawnperclick; i++) {
				Boxes[BoxCount] = CreateBox(worldPos, SPAWNABLE_BOX_SIZE, SPAWNABLE_BOX_DENSITY, BOX_FRICTION, IS_DYNAMIC);
				BoxCount++;
			}
			recordTime(t_LastSpawn);
		}
	}
}

void SpawnBoxAtScreenPos(Vector2 screenPos) {
	b2Vec2 worldPos = screenToWorldV(screenPos);
	AttemptSpawnBox(worldPos);
}

b2JointId CreateDefaultJointBetween(b2BodyId id_a, b2BodyId id_b, b2Vec2 pivot) {
	b2Transform atr = b2Body_GetTransform(id_a);
	b2RevoluteJointDef def = b2DefaultRevoluteJointDef();
	def.base.bodyIdA = id_a;
	def.base.bodyIdB = id_b;
	def.base.localFrameA.p = b2Body_GetLocalPoint(id_a, pivot);
	def.base.localFrameB.p = b2Body_GetLocalPoint(id_b, pivot);
	def.lowerAngle = -26.0f * DEG_TO_RAD;
	def.upperAngle = 45.0f * DEG_TO_RAD;
	def.enableLimit = true;
	b2JointId jointId = b2CreateRevoluteJoint(worldId, &def);
	Joints[JointCount++] = (Joint) {
		.id = jointId
	};
	return jointId;
}

b2JointId WeldBodies(b2BodyId id_a, b2BodyId id_b, b2Vec2 pivot) {
	b2Transform atr = b2Body_GetTransform(id_a);
	b2RevoluteJointDef def = b2DefaultRevoluteJointDef();
	def.base.bodyIdA = id_a;
	def.base.bodyIdB = id_b;
	def.base.localFrameA.p = b2Body_GetLocalPoint(id_a, pivot);
	def.base.localFrameB.p = b2Body_GetLocalPoint(id_b, pivot);
	def.lowerAngle = 0.0f;
	def.upperAngle = 0.0f;
	def.enableLimit = true;
	b2JointId jointId = b2CreateRevoluteJoint(worldId, &def);
	Joints[JointCount++] = (Joint) {
		.id = jointId
	};
	return jointId;
}
// ----------------------
// ------MAIN FILE-------
// ----------------------

bool QueueRestart = false;

//void HandleInput() {
//	Vector2 mousePos = {GetMouseX(), GetMouseY()};
//	b2Vec2 wmouse = screenToWorldV(mousePos);
//	if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) SpawnBoxAtScreenPos(mousePos);
//	if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))  printf("WMOUSE: %f,%f\n", wmouse.x, wmouse.y);
//
//	if(IsKeyPressed(KEY_P)) SimulationPaused = !SimulationPaused;
//	if(IsKeyPressed(KEY_R)) QueueRestart = true;
//
//}
void HandleInput() {
	Vector2 mousePos = {GetMouseX(), GetMouseY()};
	if(IsKeyPressed(KEY_SPACE)) {
		SimulationPaused = !SimulationPaused;
		printf("paused:%d\n", SimulationPaused);
	}
	if(IsKeyPressed(KEY_R)) QueueRestart = true;

}
void HandleDrawing() {
	ClearBackground(BLACK);

	for (int i = 0; i < LayoutBoxCount; i++) DrawBox(LayoutBoxes[i], RAYWHITE, NO_LINES);
	for (int i = 0; i < BoxCount; i++) DrawBox(Boxes[i], WHITE, NO_LINES);
	for (int i = 0; i < BALL_COUNT; i++) DrawBall(Balls[i]);
	for (int i = 0; i < JointCount; i++) DrawJoint(Joints[i]);

	FrameCount++;
	//DrawDebugMenu(15, 15);
}

void updateDebugMenu(Timespan inpTime, Timespan simTime, Timespan drawTime) {
	// idea: create a structure for debugLine, make it so they can have colors based on the value
	// for example, framerate<60 = red, otherwise white
	// idea 2:
	// create some visualisation of the proportion of time taken by the drawtime, update time, and inputtime
	double inputMS = TimespanSumMS(inpTime);
	double simMS = TimespanSumMS(simTime);
	double drawMS = TimespanSumMS(drawTime);
	double frametime = inputMS + simMS + drawMS;
	if (StepCount % DebugUpdateRate == 0) {
		FrameTimeMS = GetFrameTime() * 1000.0f;
		FrameRate = 1000.0f / FrameTimeMS;
		sprintf(debug_text, "inputtime: %0.2lfms\nsimtime:   %0.2lfms\ndrawtime:  %0.2lfms\nframetime: %0.2fms\nframerate: %0.1f\nboxcount:%d/%d\nsimpaused:%d", \
		        inputMS, \
		        simMS, \
		        drawMS, \
		        FrameTimeMS, \
		        FrameRate, \
		        BoxCount, MAX_BOXES,
		        SimulationPaused);
	}
}


void AddLayoutBox(b2Vec2 pos, b2BoxScale scale, bool isStatic) {
	if (LayoutBoxCount < MAX_LAYOUT_BOXES)
		LayoutBoxes[LayoutBoxCount++] = CreateBox(pos, scale, LAYOUT_BOX_DENSITY, LAYOUT_BOX_FRICTION, isStatic);
}
void AddLayoutDomino(b2Vec2 bot, b2BoxScale scale, bool isStatic) {
	if (LayoutBoxCount < MAX_LAYOUT_BOXES)
		LayoutBoxes[LayoutBoxCount++] = CreateBoxBot(bot, scale, DOMINO_DENSITY, DOMINO_FRICTION, isStatic);
}
void AddLayoutGeometry(b2Vec2 worldSize) {
	// floor
	AddLayoutBox((b2Vec2)   {
		worldSize.x / 2.0f, 2.0f
	},
	(b2BoxScale) {
		worldSize.x * 2, 0.5f
	},
	IS_STATIC);
	//roof
//    AddLayoutBox((b2Vec2)   {worldSize.x/2.0f, worldSize.y},
//                (b2BoxScale){worldSize.x, 0.5f},
//                 IS_STATIC);


	// left wall
	AddLayoutBox((b2Vec2)    {
		0.0f, worldSize.y / 2.0f
	},
	(b2BoxScale) {
		0.5f, worldSize.y
	},
	IS_STATIC);
	// right wall
//    AddLayoutBox((b2Vec2)    {worldSize.x, worldSize.y/2.0f},
//                 (b2BoxScale){0.5f, worldSize.y},
//                 IS_STATIC);

	// domino platform
//    AddLayoutBox((b2Vec2)    {24.8f, 4.2f},
//                 (b2BoxScale){22.0f, 2.0f},
//                 IS_STATIC);


	b2Vec2 seesawPivot = (b2Vec2) {
		4.2f, 4.0f
	};
	float platformLength = 6.0f;

	// Seesaw pillar
	int pillarIDX = LayoutBoxCount;
	AddLayoutBox((b2Vec2)    {
		seesawPivot.x, 1.0f
	},
	(b2BoxScale) {
		1.0f, 4.0f
	},
	IS_STATIC);

	// seesaw platform
	int platformIDX = LayoutBoxCount;
	AddLayoutBox((b2Vec2)    {
		seesawPivot.x, seesawPivot.y
	},
	(b2BoxScale) {
		platformLength, 1.0f
	},
	IS_DYNAMIC);

	// ball holder.
	int ballHolderIDX = LayoutBoxCount;
	AddLayoutBox((b2Vec2)    {
		seesawPivot.x - (platformLength / 2.0f), seesawPivot.y + 0.5f
	},
	(b2BoxScale) {
		0.2f, 2.0f
	},
	IS_DYNAMIC);

	WeldBodies(LayoutBoxes[platformIDX].id, LayoutBoxes[ballHolderIDX].id, (b2Vec2) {
		0.5f, 10.5f
	});

	CreateDefaultJointBetween(LayoutBoxes[pillarIDX].id, LayoutBoxes[platformIDX].id, seesawPivot);

	// DOMINOS

//	float dominoGap = 4.0f;
//	float dominoOffset = 10.0f;
//	int dominoCount = 15;
//	for (int i = 0; i < dominoCount; i++) {
//		b2BoxScale dominoSize = (b2BoxScale) {
//			0.7f, 15.0f+i
//		};
//		float x = dominoOffset + ( dominoGap * i);
//		AddLayoutDomino((b2Vec2) {
//			x, 1.2f
//		}, dominoSize, IS_DYNAMIC);
//	}
	b2BoxScale dominoSize = (b2BoxScale) {
		5.0f, 35.0f
	};
	AddLayoutDomino((b2Vec2) {
		40.0f, 1.3f
	}, dominoSize, IS_DYNAMIC);

	Balls[0] = CreateBall((b2Vec2) {
		2.25f, 5.0f
	}, 0.5f, IS_DYNAMIC);

}

void RestartSimulation() {
	BoxCount = 0;
	JointCount = 0;
	LayoutBoxCount = 0;
	StepCount = 0;
	FrameCount = 0;
	b2DestroyWorld(worldId);
}

void HandleUpdates() {
	b2World_Step(worldId, timeStep, subStepCount);
	if (StepCount < 350) {
		SpawnBoxAtScreenPos((Vector2) {
			0.02f, 32.5f
		});
	}
	StepCount++;
}
int main() {
//raysetup()
	InitWindow(800, 400, "RayBox2D");
	SetTargetFPS(120);
	ToggleBorderlessWindowed();

	windowSize = (Vector2) {
		GetScreenWidth(), GetScreenHeight()
	};
	debugFont = LoadFont("fonts/0xProtoNerdFont-Regular.ttf");

//b2setup()
	do {
		QueueRestart = false;
		b2Vec2 worldSize = (b2Vec2) {
			windowSize.x / PPM, windowSize.y / PPM
		};
		float gravity_y = -10.f;
		worldId = InitWorld(gravity_y);
		AddLayoutGeometry(worldSize);

		wc_timeval before_input, after_input, before_sim, after_sim, before_draw, after_draw;
		while (!WindowShouldClose() && !QueueRestart) {

			RecordTime(before_input);
			HandleInput();
			RecordTime(after_input);

			if (!SimulationPaused) {
				RecordTime(before_sim);
				HandleUpdates();
				RecordTime(after_sim);
			}

			RecordTime(before_draw);
			BeginDrawing();
			HandleDrawing();
			EndDrawing();
			RecordTime(after_draw);

			Timespan inputTime = GetTimespan(before_input, after_input);
			Timespan simTime = GetTimespan(before_sim, after_sim);
			Timespan drawTime = GetTimespan(before_draw, after_draw);
			updateDebugMenu(inputTime, simTime, drawTime);
		}

		if (QueueRestart) RestartSimulation();
	} while(QueueRestart == true);

	b2DestroyWorld(worldId);
	CloseWindow();
}

void DrawDebugMenu(float ox, float oy) {
	const float fontsize = 24.0f;
	const float spacing = 1.0f;
	Vector2 pos = (Vector2) {
		ox, oy
	};

	DrawTextEx(debugFont, debug_text, pos, fontsize, spacing, RAYWHITE);
}
