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
#include "log.h"
#include "ring_buffer.h"
#include "debug_overlay.h"

/* math helpers */
const double RAD_TO_DEG = (180.0f / B2_PI);
const double DEG_TO_RAD = (B2_PI / 180.0f);

static bool close_window = false;
/* colors */
const Color RED_TRANSLUCENT = (Color) {
	0xFF, 0x00, 0x00, 0x40
};

/* structs? these kinda suck */
typedef struct b2BoxScale {
	double width;
	double height;
} b2BoxScale;

typedef struct box {
	b2BodyId id;
	b2Vec2 hExtent;
} Box;

typedef struct ball {
	b2BodyId id;
	double radius;
} Ball;

typedef struct revJoint {
	b2JointId id;
} Joint;

const double PPM = 50.0f; // PIXELS PER (B2D)METRE

// *INDENT-OFF*
#define MAX_BOXES ((size_t)1000)
#define MAX_JOINTS ((size_t)100)
#define MAX_LAYOUT_BOXES ((size_t)100)
#define MAX_BALLS  ((size_t)1)

const double SPAWN_COOLDOWN_MS = 500.0;

const double LAYOUT_BOX_DENSITY = 1.0;
const double LAYOUT_BOX_FRICTION = 0.3;

const double DOMINO_DENSITY = 1.0;
const double DOMINO_FRICTION = 0.3;

const b2BoxScale SPAWNABLE_BOX_SIZE =  (b2BoxScale) { 2.0, 2.0 };
const double SPAWNABLE_BOX_DENSITY =  10.0;
const double BALL_DENSITY = 0.4;
const double BOX_FRICTION = 0.3;



#define recordTime(t) gettimeofday(&t, NULL);

static bool pause_sim = false;

typedef struct timeval timeval;
timeval t_LastSpawn, t_SpawnAttempt;


unsigned long timeDiff_ms(timeval start, timeval stop) {
	unsigned long secDiff = stop.tv_sec - start.tv_sec;
	unsigned long usecDiff = stop.tv_usec - start.tv_usec;
	unsigned long ms = (secDiff * 1000) + (usecDiff / 1000);
	return ms;
}

// Raylib_Helper.c

DebugOverlay topleft = (DebugOverlay){
	.fontsize = 24.0f,
	.spacing = 1.0f,
};
size_t FrameCount = 1;
size_t DebugUpdateRate = 1;
double FrameTimeMS = -1.0f;
double FrameRate = -1.0f;



// SharedHelper.h
b2WorldId worldId;
double timeStep = 1.0f / 60.0f;
size_t subStepCount = 4;
//double gravity_y = -10.f;

Vector2 windowSize;



Vector2 worldToScreen(b2Vec2 worldPos) {
	Vector2 screenPos;
	screenPos.x = worldPos.x * PPM;
	screenPos.y = windowSize.y - (worldPos.y * PPM);
	return screenPos;
}
// sy = W - wy*P
// wy*P = W - sy
// wy = (W-sy)/P
b2Vec2 screenToWorld(double sx, float sy) {
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

	DrawLineEx(bl_pos, tr_pos, 2.0f, BLACK); // bot left --> top right
	DrawLineEx(br_pos, tl_pos, 2.0f, BLACK); // bot right --> top left

}

void DrawBox(Box rect, Color c, bool drawLines) {
	b2Transform tr = b2Body_GetTransform(rect.id);
	double b2Rad = b2Rot_GetAngle(tr.q);

	// for screen space=down, also rl likes degrees not rad
	double rl_Deg = -(b2Rad * RAD_TO_DEG);


	// rotate about center
	double w = 2.0f * rect.hExtent.x * PPM;
	double h = 2.0f * rect.hExtent.y * PPM;

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
	//DrawBBoxLines(rect);
}

void DrawBall(Ball b) {
	b2Transform tr = b2Body_GetTransform(b.id);
	Vector2 pos = worldToScreen(tr.p);
	DrawCircleV(pos, b.radius * PPM, RED);
}

void DrawPointWS(b2Vec2 p, double rad, Color c) {
	DrawCircleV(worldToScreen(p), rad, c);
}

// B2D_Helper.c

b2WorldId InitWorld(double grav_y) {
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = (b2Vec2) {
		0.0f, grav_y
	};
	return b2CreateWorld(&worldDef);
}



Box CreateBox(b2Vec2 pos, b2BoxScale scale, double density, float friction, b2BodyType body_type) {
	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2Vec2 hExtent = (b2Vec2) {
		scale.width / 2.0f, scale.height / 2.0f
	};


	bodyDef.type = body_type;

	bodyDef.position = pos;
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
Box CreateBoxBot(b2Vec2 pos, b2BoxScale scale, double density, float friction, b2BodyType body_type) {
	b2Vec2 bot = (b2Vec2) {
		pos.x, pos.y + (scale.height / 2.0f)
	};

	return CreateBox(bot, scale, b2_dynamicBody, density, friction);
}

Box Boxes[MAX_BOXES];
int BoxCount = 0;
size_t BallCount = 1;
Box LayoutBoxes[MAX_LAYOUT_BOXES];
int LayoutBoxCount = 0;
Ball Balls[MAX_BALLS];
Joint Joints[MAX_JOINTS];
int JointCount = 0;

Ball CreateBall(b2Vec2 pos, double radius, b2BodyType body_type) {
	b2BodyDef bodyDef = b2DefaultBodyDef();

	if (body_type==b2_dynamicBody)
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
	recordTime(t_SpawnAttempt);
	if (BoxCount < MAX_BOXES) {
		bool cooldownElapsed = timeDiff_ms(t_LastSpawn, t_SpawnAttempt) > SPAWN_COOLDOWN_MS;
		if (cooldownElapsed) {
			Boxes[BoxCount] = CreateBox(worldPos, SPAWNABLE_BOX_SIZE, SPAWNABLE_BOX_DENSITY, BOX_FRICTION, b2_dynamicBody);
			BoxCount++;
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
	def.bodyIdA = id_a;
	def.bodyIdB = id_b;
	def.localAnchorA = b2Body_GetLocalPoint(id_a, pivot);
	def.localAnchorB = b2Body_GetLocalPoint(id_b, pivot);
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
	def.bodyIdA = id_a;
	def.bodyIdB = id_b;
	def.localAnchorA = b2Body_GetLocalPoint(id_a, pivot);
	def.localAnchorB = b2Body_GetLocalPoint(id_b, pivot);
	def.lowerAngle = 0.0f;
	def.upperAngle = 0.0f;
	def.enableLimit = true;
	b2JointId jointId = b2CreateRevoluteJoint(worldId, &def);
	Joints[JointCount++] = (Joint) {
		.id = jointId
	};
	return jointId;
}
bool restart_sim = false;

void HandleInputs() {
	Vector2 mousePos = {GetMouseX(), GetMouseY()};
	b2Vec2 wmouse = screenToWorldV(mousePos);
	if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)){ 
		SpawnBoxAtScreenPos(mousePos);
	 	printf("Clicked (b2d): %f,%f\n", wmouse.x, wmouse.y);
	}

	if (IsKeyDown(KEY_C) && (IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL))){
		close_window = true;
	}
	if(IsKeyPressed(KEY_P)){ 
		pause_sim = !pause_sim;
	}
	if(IsKeyPressed(KEY_R)){
		restart_sim = true;
	}

}

void HandleDrawing() {
	ClearBackground(BLACK);
	DrawDebugMenu(topleft, 15, 15);


	for (int i = 0; i < LayoutBoxCount; i++) DrawBox(LayoutBoxes[i], RAYWHITE, false);

	for (int i = 0; i < BoxCount; i++) DrawBox(Boxes[i], WHITE, false);

	for (int i = 0; i < BallCount; i++) DrawBall(Balls[i]);


	if (FrameCount % DebugUpdateRate == 0) {
		FrameTimeMS = GetFrameTime() * 1000.0f;
		FrameRate = 1000.0f / FrameTimeMS;
		sprintf(topleft.text, "frames:%zu\nframetime:%.3lfms\nframerate:%.1lffps\nboxcount:%d/%zu\nsimpaused:%d", \
		        FrameCount, \
		        FrameTimeMS, \
		        FrameRate, \
		        BoxCount, MAX_BOXES,
		        pause_sim);
	}
	FrameCount++;
}


void CreateLayoutBox(b2Vec2 pos, b2BoxScale scale, bool b2_staticBody) {
	if (LayoutBoxCount < MAX_LAYOUT_BOXES)
		LayoutBoxes[LayoutBoxCount++] = CreateBox(pos, scale, LAYOUT_BOX_DENSITY, LAYOUT_BOX_FRICTION, b2_staticBody);
}
void CreateLayoutDomino(b2Vec2 bot, b2BoxScale scale, bool b2_staticBody) {
	if (LayoutBoxCount < MAX_LAYOUT_BOXES)
		LayoutBoxes[LayoutBoxCount++] = CreateBoxBot(bot, scale, DOMINO_DENSITY, DOMINO_FRICTION, b2_staticBody);
}
void InitLayout(b2Vec2 worldSize) {
	// floor
	CreateLayoutBox((b2Vec2)   {
		worldSize.x / 2.0f, 1.0f
	},
	(b2BoxScale) {
		worldSize.x * 2, 0.5f
	},
	b2_staticBody);
	//roof
//    AddLayoutBox((b2Vec2)   {worldSize.x/2.0f, worldSize.y},
//                (b2BoxScale){worldSize.x, 0.5f},
//                 b2_staticBody);


	// left wall
	CreateLayoutBox((b2Vec2)    {
		0.0f, worldSize.y / 2.0f
	},
	(b2BoxScale) {
		0.5f, worldSize.y
	},
	b2_staticBody);
	// right wall
//    AddLayoutBox((b2Vec2)    {worldSize.x, worldSize.y/2.0f},
//                 (b2BoxScale){0.5f, worldSize.y},
//                 b2_staticBody);

	// domino platform
//    AddLayoutBox((b2Vec2)    {24.8f, 4.2f},
//                 (b2BoxScale){22.0f, 2.0f},
//                 b2_staticBody);


	b2Vec2 seesawPivot = (b2Vec2) {
		4.2f, 4.0f
	};
	double platformLength = 6.0f;

	// Seesaw pillar
	int pillarIDX = LayoutBoxCount;
	CreateLayoutBox((b2Vec2)    {
		seesawPivot.x, 1.0f
	},
	(b2BoxScale) {
		1.0f, 4.0f
	},
	b2_staticBody);

	// seesaw platform
	int platformIDX = LayoutBoxCount;
	CreateLayoutBox((b2Vec2)    {
		seesawPivot.x, seesawPivot.y
	},
	(b2BoxScale) {
		platformLength, 1.0f
	},
	b2_dynamicBody);

	// ball holder.
	int ballHolderIDX = LayoutBoxCount;
	CreateLayoutBox((b2Vec2)    {
		seesawPivot.x - (platformLength / 2.0f), seesawPivot.y + 0.5f
	},
	(b2BoxScale) {
		0.2f, 2.0f
	},
	b2_dynamicBody);

	WeldBodies(LayoutBoxes[platformIDX].id, LayoutBoxes[ballHolderIDX].id, (b2Vec2) {
		0.5f, 10.5f
	});


	// DOMINOS

	double dominoGap = 0.7f;
	double dominoOffset = 16.0f;
	b2BoxScale dominoSize = (b2BoxScale) {
		0.35f, 2.0f
	};
	int dominoCount = 10;
	for (int i = 0; i < dominoCount; i++) {
		double x = dominoOffset + ( (dominoSize.width + dominoGap) * i);
		dominoSize.width *= 1.2;
		dominoSize.height += ((1.05 + (i / 10.0f)));
		CreateLayoutDomino((b2Vec2) {
			x, 1.2f
		}, dominoSize, b2_dynamicBody);
	}

	Balls[0] = CreateBall((b2Vec2) {
		2.25f, 5.0f
	}, 0.5f, b2_dynamicBody);

}

void RestartSimulation() {
	BoxCount = 0;
	JointCount = 0;
	LayoutBoxCount = 0;
	b2DestroyWorld(worldId);
}

int main() {
    SetTraceLogLevel(LOG_WARNING); 
	InitWindow(800, 400, "RayBox2D");
	SetTargetFPS(120);
	ToggleBorderlessWindowed();

	windowSize = (Vector2) {
		GetScreenWidth(), GetScreenHeight()
	};

	topleft.font = LoadFont("fonts/0xProtoNerdFont-Regular.ttf");

	do {
		restart_sim = false;
		b2Vec2 worldSize = (b2Vec2) {
			windowSize.x / PPM, windowSize.y / PPM
		};
		double gravity_y = -10.0;
		worldId = InitWorld(gravity_y);
		InitLayout(worldSize);
		AttemptSpawnBox((b2Vec2) {
			6.0, 18.0
		});

		while (!WindowShouldClose() && !restart_sim) {
			HandleInputs();
			if (close_window) break;
			if (!pause_sim) b2World_Step(worldId, timeStep, subStepCount);
			BeginDrawing();
			HandleDrawing();
			EndDrawing();
		}

		if (restart_sim) RestartSimulation();
	} while(restart_sim == true);

	b2DestroyWorld(worldId);
	CloseWindow();
}

