#include <raylib.h>

#include <cstdlib>
#include <cstring>
#include <string>

#include "app/Scenes.hpp"
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

Vec2 toWorld(Vector2 p) {
    return {p.x / kPixelsPerMeter, (kScreenH - p.y) / kPixelsPerMeter};
}

float randRange(float lo, float hi) {
    return lo + (hi - lo) * (static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX));
}

// drag to throw: press sets the spawn point, release sets the velocity
struct ThrowState {
    bool dragging = false;
    Vec2 start;
};

constexpr float kThrowScale = 3.0f;

void handleMouse(World& world, ThrowState& t) {
    Vec2 mouse = toWorld(GetMousePosition());
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        t.dragging = true;
        t.start = mouse;
    }
    if (t.dragging && IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
        t.dragging = false;
        Body b(Circle{randRange(0.4f, 1.0f)}, t.start);
        b.velocity = (mouse - t.start) * kThrowScale;
        b.restitution = 0.5f;
        world.addBody(b);
    }
}

struct DebugFlags {
    bool showGrid = false;
};

void drawGrid(const World& world) {
    float cs = world.broadPhase.lastCellSize();
    float px = cs * kPixelsPerMeter;
    if (px < 4.0f) return; // too dense to be useful
    Color c{255, 255, 255, 25};
    for (float x = 0.0f; x < kScreenW; x += px) DrawLineV({x, 0}, {x, kScreenH}, c);
    // cells are anchored at world y = 0, which is the bottom of the screen
    for (float y = kScreenH; y > 0.0f; y -= px) DrawLineV({0, y}, {kScreenW, y}, c);
}

void drawHud(const World& world, const Scenes& scenes) {
    const StepStats& st = world.stats();
    DrawRectangle(10, 10, 260, 164, Color{0, 0, 0, 150});
    DrawText(TextFormat("FPS: %d", GetFPS()), 20, 20, 18, RAYWHITE);
    DrawText(TextFormat("bodies: %d", static_cast<int>(world.bodies().size())), 20, 42, 18, RAYWHITE);
    DrawText(TextFormat("contacts: %d", st.contactCount), 20, 64, 18, RAYWHITE);
    DrawText(TextFormat("step: %.3f ms", st.stepMs), 20, 86, 18, RAYWHITE);
    DrawText(TextFormat("pairs tested: %lld", st.pairsTested), 20, 108, 18, RAYWHITE);
    bool grid = world.broadPhase.mode == BroadPhaseMode::Grid;
    DrawText(grid ? "broad phase: grid" : "broad phase: brute force", 20, 130, 18,
             grid ? GREEN : ORANGE);
    DrawText(TextFormat("scene %d: %s", scenes.current(), scenes.name()), 20, 152, 18, RAYWHITE);
}

void handleKeys(World& world, Scenes& scenes, DebugFlags& flags) {
    for (int k = 1; k <= 5; ++k) {
        if (IsKeyPressed(KEY_ONE + k - 1)) scenes.load(k, world);
    }
    if (IsKeyPressed(KEY_R)) scenes.load(scenes.current(), world);
    if (IsKeyPressed(KEY_F2)) flags.showGrid = !flags.showGrid;
    if (IsKeyPressed(KEY_F3)) {
        auto& m = world.broadPhase.mode;
        m = (m == BroadPhaseMode::Grid) ? BroadPhaseMode::BruteForce : BroadPhaseMode::Grid;
    }
}

// command line options, mostly so I can record the README gif without
// fighting with screen capture software
struct Options {
    int scene = 2;
    const char* recordDir = nullptr; // save every frame as a png here
    int maxFrames = 0;               // quit after this many frames (0 = never)
};

Options parseArgs(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        bool hasNext = i + 1 < argc;
        if (!std::strcmp(argv[i], "--scene") && hasNext) o.scene = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--record") && hasNext) o.recordDir = argv[++i];
        else if (!std::strcmp(argv[i], "--frames") && hasNext) o.maxFrames = std::atoi(argv[++i]);
    }
    return o;
}

} // namespace

int main(int argc, char** argv) {
    Options opts = parseArgs(argc, argv);

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
    InitWindow(kScreenW, kScreenH, "impulse");
    SetTargetFPS(60);

    const Vec2 worldSize{kScreenW / kPixelsPerMeter, kScreenH / kPixelsPerMeter};
    World world;
    Scenes scenes(worldSize);
    scenes.load(opts.scene, world);

    float accumulator = 0.0f;
    ThrowState throwState;
    DebugFlags flags;
    int frame = 0;

    while (!WindowShouldClose()) {
        handleMouse(world, throwState);
        handleKeys(world, scenes, flags);

        // clamp so a long frame (window drag, breakpoint) doesn't cause a
        // spiral of death
        float frameTime = GetFrameTime();
        if (frameTime > 0.25f) frameTime = 0.25f;
        // when recording, pretend every frame took exactly 1/60 s so the
        // video plays back at the right speed even if saving pngs is slow
        if (opts.recordDir) frameTime = 1.0f / 60.0f;
        accumulator += frameTime;

        while (accumulator >= kDt) {
            scenes.update(world);
            world.step(kDt);
            accumulator -= kDt;
        }

        BeginDrawing();
        ClearBackground(Color{24, 26, 32, 255});
        if (flags.showGrid) drawGrid(world);
        for (const Body& b : world.bodies()) {
            const auto& c = std::get<Circle>(b.shape);
            DrawCircleV(toScreen(b.position), c.radius * kPixelsPerMeter, Color{120, 180, 255, 255});
        }
        for (const Manifold& m : world.contacts()) {
            for (int i = 0; i < m.count; ++i) {
                DrawCircleV(toScreen(m.contacts[i].position), 3.0f, RED);
            }
        }
        if (throwState.dragging) {
            DrawLineEx(toScreen(throwState.start), GetMousePosition(), 2.0f, YELLOW);
        }
        drawHud(world, scenes);
        EndDrawing();

        ++frame;
        if (opts.recordDir) {
            Image img = LoadImageFromScreen();
            std::string path = std::string(opts.recordDir) + TextFormat("/frame_%05d.png", frame);
            ExportImage(img, path.c_str());
            UnloadImage(img);
        }
        if (opts.maxFrames > 0 && frame >= opts.maxFrames) break;
    }

    CloseWindow();
    return 0;
}
