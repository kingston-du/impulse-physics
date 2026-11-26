#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "math/Vec2.hpp"

using namespace impulse;
using Catch::Matchers::WithinAbs;

TEST_CASE("Vec2 basic arithmetic", "[vec2]") {
    constexpr Vec2 a{1.0f, 2.0f};
    constexpr Vec2 b{3.0f, -4.0f};

    // constexpr on purpose, if this compiles the ops really are constexpr
    static_assert(a + b == Vec2{4.0f, -2.0f});
    static_assert(a - b == Vec2{-2.0f, 6.0f});
    static_assert(a * 2.0f == Vec2{2.0f, 4.0f});
    static_assert(2.0f * a == a * 2.0f);
    static_assert(-a == Vec2{-1.0f, -2.0f});

    Vec2 c = a;
    c += b;
    c -= a;
    REQUIRE(c == b);
}

TEST_CASE("Vec2 dot product identities", "[vec2]") {
    Vec2 a{3.0f, 4.0f};
    Vec2 b{-2.0f, 7.5f};

    REQUIRE(dot(a, b) == dot(b, a));
    REQUIRE_THAT(dot(a, a), WithinAbs(lengthSq(a), 1e-6));
    // perpendicular vectors have zero dot product
    REQUIRE(dot(a, perp(a)) == 0.0f);
}

TEST_CASE("Vec2 cross product identities", "[vec2]") {
    Vec2 x{1.0f, 0.0f};
    Vec2 y{0.0f, 1.0f};
    Vec2 a{2.5f, -1.0f};

    REQUIRE(cross(x, y) == 1.0f);
    REQUIRE(cross(y, x) == -1.0f);
    REQUIRE(cross(a, a) == 0.0f);

    // w x r should be perpendicular to r and scale with w
    Vec2 r = cross(2.0f, a);
    REQUIRE(dot(r, a) == 0.0f);
    REQUIRE_THAT(length(r), WithinAbs(2.0f * length(a), 1e-5));

    // the two scalar forms are negatives of each other
    REQUIRE(cross(a, 3.0f) == -cross(3.0f, a));
}

TEST_CASE("Vec2 length and normalize", "[vec2]") {
    Vec2 a{3.0f, 4.0f};
    REQUIRE_THAT(length(a), WithinAbs(5.0f, 1e-6));

    Vec2 n = normalize(a);
    REQUIRE_THAT(length(n), WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(n.x, WithinAbs(0.6f, 1e-6));
    REQUIRE_THAT(n.y, WithinAbs(0.8f, 1e-6));
}

TEST_CASE("normalize of zero vector is zero, not NaN", "[vec2]") {
    Vec2 n = normalize(Vec2{0.0f, 0.0f});
    REQUIRE(n.x == 0.0f);
    REQUIRE(n.y == 0.0f);

    Vec2 tiny = normalize(Vec2{1e-12f, 0.0f});
    REQUIRE(tiny == Vec2{0.0f, 0.0f});
}
