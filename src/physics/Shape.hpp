#pragma once

#include <variant>

#include "math/Vec2.hpp"

namespace impulse {

struct Circle {
    float radius = 0.5f;
};

using Shape = std::variant<Circle>;

} // namespace impulse
