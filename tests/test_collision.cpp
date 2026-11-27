#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "physics/Collision.hpp"

using namespace impulse;
using Catch::Matchers::WithinAbs;

namespace {
Body circleAt(float x, float y, float r) {
    return Body(Circle{r}, Vec2{x, y});
}
} // namespace

TEST_CASE("circle vs circle overlapping", "[collision][circle]") {
    Body a = circleAt(0.0f, 0.0f, 1.0f);
    Body b = circleAt(1.5f, 0.0f, 1.0f);
    Manifold m;
    REQUIRE(collide(a, b, m));
    REQUIRE(m.count == 1);
    REQUIRE_THAT(m.contacts[0].penetration, WithinAbs(0.5f, 1e-6));
    // normal should point from a to b
    REQUIRE_THAT(m.normal.x, WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(m.normal.y, WithinAbs(0.0f, 1e-6));
}

TEST_CASE("circle vs circle touching counts as contact", "[collision][circle]") {
    Body a = circleAt(0.0f, 0.0f, 1.0f);
    Body b = circleAt(0.0f, 2.0f, 1.0f);
    Manifold m;
    REQUIRE(collide(a, b, m));
    REQUIRE_THAT(m.contacts[0].penetration, WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(m.contacts[0].position.y, WithinAbs(1.0f, 1e-6));
}

TEST_CASE("circle vs circle separated", "[collision][circle]") {
    Body a = circleAt(0.0f, 0.0f, 1.0f);
    Body b = circleAt(3.0f, 1.0f, 1.0f);
    Manifold m;
    REQUIRE_FALSE(collide(a, b, m));
}

TEST_CASE("circle vs circle with same center does not produce NaN", "[collision][circle]") {
    Body a = circleAt(2.0f, 2.0f, 1.0f);
    Body b = circleAt(2.0f, 2.0f, 0.5f);
    Manifold m;
    REQUIRE(collide(a, b, m));
    REQUIRE_THAT(length(m.normal), WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(m.contacts[0].penetration, WithinAbs(1.5f, 1e-6));
}
