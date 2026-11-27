#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <cstring>

#include "physics/World.hpp"

using namespace impulse;

namespace {

std::vector<Body> randomCircles(int count, float size, uint32_t seed) {
    uint32_t s = seed;
    auto next = [&s](float lo, float hi) {
        s = s * 1664525u + 1013904223u;
        return lo + (hi - lo) * (static_cast<float>(s >> 8) / static_cast<float>(1u << 24));
    };
    std::vector<Body> out;
    for (int i = 0; i < count; ++i) {
        // range goes negative on purpose to check the floor() cell math
        out.emplace_back(Circle{next(0.2f, 1.0f)}, Vec2{next(-size, size), next(-size, size)});
    }
    return out;
}

bool samePairs(const std::vector<BodyPair>& x, const std::vector<BodyPair>& y) {
    if (x.size() != y.size()) return false;
    for (size_t i = 0; i < x.size(); ++i) {
        if (x[i].a != y[i].a || x[i].b != y[i].b) return false;
    }
    return true;
}

} // namespace

TEST_CASE("grid finds exactly the same pairs as brute force", "[broadphase]") {
    auto bodies = randomCircles(800, 30.0f, 7);

    BroadPhase brute;
    brute.mode = BroadPhaseMode::BruteForce;
    BroadPhase grid;
    grid.mode = BroadPhaseMode::Grid;

    std::vector<BodyPair> p1, p2;
    brute.findPairs(bodies, p1);
    grid.findPairs(bodies, p2);

    REQUIRE(!p1.empty());
    REQUIRE(samePairs(p1, p2));
}

TEST_CASE("grid reports each pair once even when it spans many cells", "[broadphase]") {
    std::vector<Body> bodies;
    bodies.emplace_back(Circle{5.0f}, Vec2{0.0f, 0.0f});
    bodies.emplace_back(Circle{5.0f}, Vec2{3.0f, 1.0f});

    BroadPhase grid;
    grid.cellSize = 1.0f; // both circles cover ~100 cells
    std::vector<BodyPair> pairs;
    grid.findPairs(bodies, pairs);

    REQUIRE(pairs.size() == 1);
    REQUIRE(pairs[0].a == 0);
    REQUIRE(pairs[0].b == 1);
}

TEST_CASE("grid tests far fewer pairs than brute force", "[broadphase]") {
    auto bodies = randomCircles(2000, 60.0f, 11);

    BroadPhase brute;
    brute.mode = BroadPhaseMode::BruteForce;
    BroadPhase grid;
    std::vector<BodyPair> pairs;

    brute.findPairs(bodies, pairs);
    grid.findPairs(bodies, pairs);

    REQUIRE(brute.pairsTested() == 2000LL * 1999 / 2);
    // should be way under 1% of the brute force count
    REQUIRE(grid.pairsTested() * 100 < brute.pairsTested());
}

TEST_CASE("simulation is identical with grid and brute force", "[broadphase][determinism]") {
    auto run = [](BroadPhaseMode mode) {
        World w;
        w.broadPhase.mode = mode;
        w.boundsMin = {-30.0f, -30.0f};
        w.boundsMax = {30.0f, 30.0f};
        for (const Body& b : randomCircles(300, 25.0f, 3)) w.addBody(b);
        for (int i = 0; i < 300; ++i) w.step(1.0f / 120.0f);
        return w;
    };

    World a = run(BroadPhaseMode::BruteForce);
    World b = run(BroadPhaseMode::Grid);
    for (size_t i = 0; i < a.bodies().size(); ++i) {
        REQUIRE(std::memcmp(&a.bodies()[i].position, &b.bodies()[i].position, sizeof(Vec2)) == 0);
    }
}
