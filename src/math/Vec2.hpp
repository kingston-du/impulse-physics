#pragma once

#include <cmath>

namespace impulse {

struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;

    constexpr Vec2() = default;
    constexpr Vec2(float x_, float y_) : x(x_), y(y_) {}

    constexpr Vec2 operator-() const { return {-x, -y}; }

    constexpr Vec2& operator+=(Vec2 o) { x += o.x; y += o.y; return *this; }
    constexpr Vec2& operator-=(Vec2 o) { x -= o.x; y -= o.y; return *this; }
    constexpr Vec2& operator*=(float s) { x *= s; y *= s; return *this; }

    constexpr bool operator==(const Vec2&) const = default;

    constexpr float lengthSq() const { return x * x + y * y; }
    float length() const { return std::sqrt(lengthSq()); }
};

constexpr Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
constexpr Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
constexpr Vec2 operator*(Vec2 v, float s) { return {v.x * s, v.y * s}; }
constexpr Vec2 operator*(float s, Vec2 v) { return {v.x * s, v.y * s}; }
constexpr Vec2 operator/(Vec2 v, float s) { return {v.x / s, v.y / s}; }

constexpr float dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }

// 2D cross product comes in three flavors. vec x vec gives the z component
// of the 3D cross product (a scalar). The scalar versions treat the scalar as
// a z axis vector, which is what you need for w x r style terms.
constexpr float cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
constexpr Vec2 cross(Vec2 v, float s) { return {s * v.y, -s * v.x}; }
constexpr Vec2 cross(float s, Vec2 v) { return {-s * v.y, s * v.x}; }

inline float length(Vec2 v) { return v.length(); }
constexpr float lengthSq(Vec2 v) { return v.lengthSq(); }

// Returns the zero vector for zero (or nearly zero) input instead of NaNs.
inline Vec2 normalize(Vec2 v) {
    float len = v.length();
    if (len < 1e-8f) return {0.0f, 0.0f};
    return v / len;
}

constexpr Vec2 perp(Vec2 v) { return {-v.y, v.x}; }

constexpr Vec2 vabs(Vec2 v) { return {v.x < 0 ? -v.x : v.x, v.y < 0 ? -v.y : v.y}; }

inline float distance(Vec2 a, Vec2 b) { return length(a - b); }

} // namespace impulse
