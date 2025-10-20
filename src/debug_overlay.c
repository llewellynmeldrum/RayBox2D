#include "debug_overlay.h"
void DrawDebugMenu(DebugOverlay debug_overlay, double ox, float oy) {
	Vector2 pos = (Vector2) {
		ox, oy
	};

	DrawTextEx(debug_overlay.font, debug_overlay.text, pos, debug_overlay.fontsize, debug_overlay.spacing, WHITE);
}
