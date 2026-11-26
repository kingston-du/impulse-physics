#include <raylib.h>

#include <vector>

#include "physics/Body.hpp"

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

void integrate(std::vector<Body>& bodies, Vec2 gravity, float dt) {
    for (Body& b : bodies) {
        if (b.isStatic()) continue;
        // semi implicit euler: update velocity first, then use the new
        // velocity for position
        b.velocity += (gravity + b.invMass * b.force) * dt;
        b.position += b.velocity * dt;
        b.force = {};
    }
}

} // namespace

int main() {
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
    InitWindow(kScreenW, kScreenH, "impulse");
    SetTargetFPS(60);

    const Vec2 gravity{0.0f, -9.81f};
    std::vector<Body> bodies;
    for (int i = 0; i < 10; ++i) {
        bodies.emplace_back(Circle{0.5f + 0.1f * i}, Vec2{6.0f + 5.0f * i, 30.0f});
    }

    float accumulator = 0.0f;

    while (!WindowShouldClose()) {
        // clamp so a long frame (window drag, breakpoint) doesn't cause a
        // spiral of death
        float frameTime = GetFrameTime();
        if (frameTime > 0.25f) frameTime = 0.25f;
        accumulator += frameTime;

        while (accumulator >= kDt) {
            integrate(bodies, gravity, kDt);
            accumulator -= kDt;
        }

        BeginDrawing();
        ClearBackground(Color{24, 26, 32, 255});
        for (const Body& b : bodies) {
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
