#pragma once

#include "math/Vec2.hpp"

namespace impulse {

struct ContactPoint {
    Vec2 position;           // world space
    float penetration = 0.0f;
};

// Result of the narrow phase for one pair. Bodies are stored by index into
// World's body vector, not by pointer, so the vector can grow freely.
struct Manifold {
    int a = -1;
    int b = -1;
    Vec2 normal;             // points from a to b
    int count = 0;
    ContactPoint contacts[2];
};

} // namespace impulse
