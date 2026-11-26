#include <raylib.h>

#include "physics/World.hpp"

using namespace impulse;

namespace {

constexpr int kScreenW = 1280;
constexpr int kScreenH = 720;
constexpr float kPixelsPerMeter = 20.0f;

constexpr float kPhysicsHz = 120.0f;
constexpr float kDt = 1.0f / kPhysicsHz;

// physics is y up in meters, raylib is y down in pixels
Vector2 toScreen(Vec2 p) {
    return {p.x * kPixelsPerMeter, kScreenH - p.y * kPixelsPerMeter};
}

} // namespace

int main() {
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
    InitWindow(kScreenW, kScreenH, "impulse");
    SetTargetFPS(60);

    World world;
    world.boundsMax = {kScreenW / kPixelsPerMeter, kScreenH / kPixelsPerMeter};

    for (int i = 0; i < 10; ++i) {
        Body b(Circle{0.5f + 0.1f * i}, Vec2{6.0f + 5.0f * i, 30.0f});
        b.velocity = {(i % 2 ? 1.0f : -1.0f) * 4.0f, 0.0f};
        b.restitution = 0.8f;
        world.addBody(b);
    }

    float accumulator = 0.0f;

    while (!WindowShouldClose()) {
        // clamp so a long frame (window drag, breakpoint) doesn't cause a
        // spiral of death
        float frameTime = GetFrameTime();
        if (frameTime > 0.25f) frameTime = 0.25f;
        accumulator += frameTime;

        while (accumulator >= kDt) {
            world.step(kDt);
            accumulator -= kDt;
        }

        BeginDrawing();
        ClearBackground(Color{24, 26, 32, 255});
        for (const Body& b : world.bodies()) {
            const auto& c = std::get<Circle>(b.shape);
            DrawCircleV(toScreen(b.position), c.radius * kPixelsPerMeter, Color{120, 180, 255, 255});
        }
        DrawText("impulse", 20, 20, 20, RAYWHITE);
        DrawFPS(kScreenW - 100, 20);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
