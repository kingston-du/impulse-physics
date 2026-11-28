// Headless benchmark. Prints CSV to stdout:
//   bodies,mode,step_ms,pairs_tested,contacts
// Build in Release or the numbers are meaningless.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "physics/World.hpp"

using namespace impulse;

namespace {

constexpr float kDt = 1.0f / 120.0f;

void buildScene(World& w, int count) {
    w.clear();
    w.gravity = {0.0f, -9.81f};
    w.useBounds = true;
    w.boundsMin = {0.0f, 0.0f};
    w.boundsMax = {120.0f, 200.0f};

    // stack circles in rows starting near the floor so there's a real pile
    // with lots of contacts, not just a cloud of falling balls
    unsigned seed = 5;
    auto rnd = [&seed](float lo, float hi) {
        seed = seed * 1664525u + 1013904223u;
        return lo + (hi - lo) * (static_cast<float>(seed >> 8) / static_cast<float>(1u << 24));
    };
    const int cols = 110;
    for (int i = 0; i < count; ++i) {
        float x = 1.0f + 1.08f * static_cast<float>(i % cols) + rnd(-0.03f, 0.03f);
        float y = 1.0f + 1.08f * static_cast<float>(i / cols);
        Body b(Circle{rnd(0.3f, 0.5f)}, Vec2{x, y});
        b.restitution = 0.2f;
        w.addBody(b);
    }
}

struct Result {
    double stepMs;
    double pairs;
    double contacts;
};

Result run(int count, BroadPhaseMode mode, int warmup, int steps) {
    World w;
    w.broadPhase.mode = mode;
    buildScene(w, count);
    for (int i = 0; i < warmup; ++i) w.step(kDt);

    Result r{0.0, 0.0, 0.0};
    for (int i = 0; i < steps; ++i) {
        w.step(kDt);
        r.stepMs += w.stats().stepMs;
        r.pairs += static_cast<double>(w.stats().pairsTested);
        r.contacts += w.stats().contactCount;
    }
    r.stepMs /= steps;
    r.pairs /= steps;
    r.contacts /= steps;
    return r;
}

} // namespace

int main(int argc, char** argv) {
    std::vector<int> counts = {500, 1000, 2000, 5000};
    int steps = 300;
    bool skipBrute = false;

    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--steps") && i + 1 < argc) steps = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--grid-only")) skipBrute = true;
        else if (!std::strcmp(argv[i], "--counts") && i + 1 < argc) {
            // comma separated, like --counts 1000,10000
            counts.clear();
            for (char* tok = std::strtok(argv[++i], ","); tok; tok = std::strtok(nullptr, ","))
                counts.push_back(std::atoi(tok));
        }
    }

    std::printf("bodies,mode,step_ms,pairs_tested,contacts\n");
    for (int n : counts) {
        for (BroadPhaseMode mode : {BroadPhaseMode::BruteForce, BroadPhaseMode::Grid}) {
            if (skipBrute && mode == BroadPhaseMode::BruteForce) continue;
            Result r = run(n, mode, 120, steps);
            std::printf("%d,%s,%.4f,%.0f,%.0f\n", n,
                        mode == BroadPhaseMode::Grid ? "grid" : "brute",
                        r.stepMs, r.pairs, r.contacts);
            std::fflush(stdout);
        }
    }
    return 0;
}
