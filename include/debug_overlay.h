#ifndef DEBUG_OVERLAY_H
#define DEBUG_OVERLAY_H
#include "raylib.h"
typedef struct DebugOverlay {
	const double fontsize;
	const double spacing;
	Font font;
	char text[512];

} DebugOverlay;
void DrawDebugMenu(DebugOverlay debug_overlay, double ox, float oy);
#endif //DEBUG_OVERLAY_H
