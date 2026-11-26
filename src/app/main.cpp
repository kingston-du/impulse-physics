#include <raylib.h>

int main() {
    const int screenW = 1280;
    const int screenH = 720;

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
    InitWindow(screenW, screenH, "impulse");
    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(Color{24, 26, 32, 255});
        DrawText("impulse", 20, 20, 20, RAYWHITE);
        DrawFPS(screenW - 100, 20);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
