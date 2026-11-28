#include <raylib.h>

#include <cstdio>
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
    bool showContacts = false;
    bool showGrid = false;
};

// a few colors to pick from so piles don't look like one blue blob
Color bodyColor(int index) {
    static const Color palette[] = {
        {102, 178, 255, 255}, {255, 159, 67, 255}, {120, 224, 143, 255},
        {255, 107, 129, 255}, {200, 160, 255, 255}, {255, 221, 89, 255},
    };
    return palette[static_cast<unsigned>(index) * 2654435761u % 6];
}

void drawContacts(const World& world) {
    for (const Manifold& m : world.contacts()) {
        for (int i = 0; i < m.count; ++i) {
            Vector2 p = toScreen(m.contacts[i].position);
            Vector2 tip = toScreen(m.contacts[i].position + m.normal * 0.6f);
            DrawLineV(p, tip, YELLOW);
            DrawCircleV(p, 2.5f, RED);
        }
    }
}

void drawGrid(const World& world) {
    float cs = world.broadPhase.lastCellSize();
    float px = cs * kPixelsPerMeter;
    if (px < 4.0f) return; // too dense to be useful
    Color c{255, 255, 255, 25};
    for (float x = 0.0f; x < kScreenW; x += px) DrawLineV({x, 0}, {x, kScreenH}, c);
    // cells are anchored at world y = 0, which is the bottom of the screen
    for (float y = kScreenH; y > 0.0f; y -= px) DrawLineV({0, y}, {kScreenW, y}, c);
}

void drawHud(const World& world, const Scenes& scenes, bool recording) {
    const StepStats& st = world.stats();
    DrawRectangle(10, 10, 260, 164, Color{0, 0, 0, 150});
    // real fps is meaningless while every frame gets saved to disk
    if (recording) DrawText("FPS: (recording)", 20, 20, 18, RAYWHITE);
    else DrawText(TextFormat("FPS: %d", GetFPS()), 20, 20, 18, RAYWHITE);
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
    if (IsKeyPressed(KEY_F1)) flags.showContacts = !flags.showContacts;
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
    bool findMax = false;            // keep adding balls until we drop below 60 fps
};

Options parseArgs(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        bool hasNext = i + 1 < argc;
        if (!std::strcmp(argv[i], "--scene") && hasNext) o.scene = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--record") && hasNext) o.recordDir = argv[++i];
        else if (!std::strcmp(argv[i], "--frames") && hasNext) o.maxFrames = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--find-max")) o.findMax = true;
    }
    return o;
}

// Finds the biggest body count that still holds 60 fps with physics at
// 120 Hz. Vsync is off so we can see how long a frame really takes. Adds a
// row of balls every half second and stops once the average frame time over
// the last second goes past 16.7 ms.
struct MaxFinder {
    float rowTimer = 0.0f;
    double frameSum = 0.0;
    int frameCount = 0;
    int lastGood = 0;

    // returns true when done
    bool update(World& world, Vec2 worldSize, float frameTime) {
        frameSum += frameTime;
        ++frameCount;
        if (frameCount == 60) {
            double avg = frameSum / frameCount;
            int n = static_cast<int>(world.bodies().size());
            std::printf("bodies %d  avg frame %.2f ms\n", n, avg * 1000.0);
            if (avg > 1.0 / 60.0) {
                std::printf("max bodies at 60 fps: %d\n", lastGood);
                return true;
            }
            lastGood = n;
            frameSum = 0.0;
            frameCount = 0;
        }

        rowTimer += frameTime;
        if (rowTimer > 0.5f) {
            rowTimer = 0.0f;
            const float r = 0.15f;
            for (float x = 0.5f; x < worldSize.x - 0.5f; x += 2.2f * r) {
                world.addBody(Body(Circle{r}, Vec2{x, worldSize.y - 1.0f}));
            }
        }
        return false;
    }
};

} // namespace

int main(int argc, char** argv) {
    Options opts = parseArgs(argc, argv);

    SetConfigFlags(opts.findMax ? FLAG_MSAA_4X_HINT : (FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT));
    InitWindow(kScreenW, kScreenH, "impulse");
    SetTargetFPS(opts.findMax ? 0 : 60);

    const Vec2 worldSize{kScreenW / kPixelsPerMeter, kScreenH / kPixelsPerMeter};
    World world;
    Scenes scenes(worldSize);
    scenes.load(opts.scene, world);
    MaxFinder maxFinder;
    if (opts.findMax) world.clear();

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
        if (opts.findMax && maxFinder.update(world, worldSize, GetFrameTime())) break;
        accumulator += frameTime;

        while (accumulator >= kDt) {
            scenes.update(world);
            world.step(kDt);
            accumulator -= kDt;
        }

        BeginDrawing();
        ClearBackground(Color{24, 26, 32, 255});
        if (flags.showGrid) drawGrid(world);
        const auto& bodies = world.bodies();
        for (size_t i = 0; i < bodies.size(); ++i) {
            const Body& b = bodies[i];
            const auto& c = std::get<Circle>(b.shape);
            DrawCircleV(toScreen(b.position), c.radius * kPixelsPerMeter, bodyColor(static_cast<int>(i)));
        }
        if (flags.showContacts) drawContacts(world);
        if (throwState.dragging) {
            DrawLineEx(toScreen(throwState.start), GetMousePosition(), 2.0f, YELLOW);
        }
        drawHud(world, scenes, opts.recordDir != nullptr);
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
