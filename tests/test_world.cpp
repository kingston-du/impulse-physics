#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cstdint>
#include <cstring>

#include "physics/World.hpp"

using namespace impulse;
using Catch::Matchers::WithinAbs;

namespace {

constexpr float kDt = 1.0f / 120.0f;

// tiny LCG so the tests don't depend on how a given standard library
// implements the <random> distributions
struct Rng {
    uint32_t state;
    explicit Rng(uint32_t seed) : state(seed) {}
    float next(float lo, float hi) {
        state = state * 1664525u + 1013904223u;
        float t = static_cast<float>(state >> 8) / static_cast<float>(1u << 24);
        return lo + (hi - lo) * t;
    }
};

// grid of circles with random velocities, no overlaps at the start
void fillWithCircles(World& w, int count, uint32_t seed, Vec2 drift = {}) {
    Rng rng(seed);
    int cols = 10;
    for (int i = 0; i < count; ++i) {
        float x = 4.0f + 4.0f * static_cast<float>(i % cols);
        float y = 4.0f + 4.0f * static_cast<float>(i / cols);
        Body b(Circle{rng.next(0.5f, 1.2f)}, Vec2{x, y});
        b.velocity = Vec2{rng.next(-5.0f, 5.0f), rng.next(-5.0f, 5.0f)} + drift;
        b.restitution = 1.0f;
        w.addBody(b);
    }
}

Vec2 totalMomentum(const World& w) {
    Vec2 p;
    for (const Body& b : w.bodies()) p += b.velocity * b.mass;
    return p;
}

double kineticEnergy(const World& w) {
    double ke = 0.0;
    for (const Body& b : w.bodies()) {
        ke += 0.5 * b.mass * b.velocity.lengthSq();
        ke += 0.5 * b.inertia * b.angularVelocity * b.angularVelocity;
    }
    return ke;
}

World zeroGravityWorld() {
    World w;
    w.gravity = {0.0f, 0.0f};
    w.useBounds = false;
    w.restitutionThreshold = 0.0f;
    return w;
}

} // namespace

TEST_CASE("equal circles in an elastic head on collision swap velocities", "[world][conservation]") {
    World w = zeroGravityWorld();
    Body a(Circle{1.0f}, Vec2{0.0f, 0.0f});
    Body b(Circle{1.0f}, Vec2{3.0f, 0.0f});
    a.velocity = {2.0f, 0.0f};
    b.velocity = {-2.0f, 0.0f};
    a.restitution = b.restitution = 1.0f;
    w.addBody(a);
    w.addBody(b);

    for (int i = 0; i < 120; ++i) w.step(kDt);

    const auto& bodies = w.bodies();
    REQUIRE_THAT(bodies[0].velocity.x, WithinAbs(-2.0f, 1e-5));
    REQUIRE_THAT(bodies[1].velocity.x, WithinAbs(2.0f, 1e-5));
    REQUIRE_THAT(bodies[0].velocity.y, WithinAbs(0.0f, 1e-5));
    REQUIRE_THAT(bodies[1].velocity.y, WithinAbs(0.0f, 1e-5));
}

TEST_CASE("linear momentum is conserved with no gravity or walls", "[world][conservation]") {
    World w = zeroGravityWorld();
    // drift keeps total momentum well away from zero so a relative check
    // actually means something
    fillWithCircles(w, 60, 1234, Vec2{3.0f, 1.0f});

    Vec2 p0 = totalMomentum(w);
    for (int i = 0; i < 1000; ++i) w.step(kDt);
    Vec2 p1 = totalMomentum(w);

    float err = length(p1 - p0) / length(p0);
    INFO("relative momentum error " << err);
    REQUIRE(err < 0.001f);
}

TEST_CASE("kinetic energy barely drifts with restitution 1", "[world][conservation]") {
    World w = zeroGravityWorld();
    // keep them in a box so there are lots of collisions the whole time
    w.useBounds = true;
    w.boundsMin = {0.0f, 0.0f};
    w.boundsMax = {44.0f, 30.0f};
    fillWithCircles(w, 40, 99);

    double e0 = kineticEnergy(w);
    for (int i = 0; i < 10000; ++i) w.step(kDt);
    double e1 = kineticEnergy(w);

    double drift = std::abs(e1 - e0) / e0;
    INFO("relative energy drift " << drift);
    REQUIRE(drift < 0.01);
}

TEST_CASE("same scene run twice is bit identical", "[world][determinism]") {
    auto run = [] {
        World w;
        w.boundsMax = {44.0f, 40.0f};
        fillWithCircles(w, 80, 42);
        for (int i = 0; i < 1000; ++i) w.step(kDt);
        return w;
    };

    World w1 = run();
    World w2 = run();
    REQUIRE(w1.bodies().size() == w2.bodies().size());
    for (size_t i = 0; i < w1.bodies().size(); ++i) {
        const Body& a = w1.bodies()[i];
        const Body& b = w2.bodies()[i];
        // compare raw bits, not with a tolerance
        REQUIRE(std::memcmp(&a.position, &b.position, sizeof(Vec2)) == 0);
        REQUIRE(std::memcmp(&a.velocity, &b.velocity, sizeof(Vec2)) == 0);
    }
}
