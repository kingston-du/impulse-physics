#pragma once

#include <cstdint>
#include <vector>

#include "physics/Body.hpp"

namespace impulse {

struct AABB {
    Vec2 min;
    Vec2 max;
};

inline bool overlaps(const AABB& a, const AABB& b) {
    return a.min.x <= b.max.x && b.min.x <= a.max.x &&
           a.min.y <= b.max.y && b.min.y <= a.max.y;
}

AABB computeAABB(const Body& b);

struct BodyPair {
    int a;
    int b;
};

enum class BroadPhaseMode { BruteForce, Grid };

// Finds pairs of bodies whose bounding boxes overlap. Output is sorted by
// (a, b) so both modes feed the solver in the same order, which keeps the
// simulation identical no matter which one you pick.
class BroadPhase {
public:
    BroadPhaseMode mode = BroadPhaseMode::Grid;

    // 0 means pick automatically: 2x the average body size
    float cellSize = 0.0f;

    void findPairs(const std::vector<Body>& bodies, std::vector<BodyPair>& out);

    long long pairsTested() const { return pairsTested_; }
    float lastCellSize() const { return lastCellSize_; }
    const std::vector<AABB>& aabbs() const { return aabbs_; }

private:
    void bruteForce(const std::vector<Body>& bodies, std::vector<BodyPair>& out);
    void grid(const std::vector<Body>& bodies, std::vector<BodyPair>& out);

    struct Entry {
        int32_t cx;
        int32_t cy;
        int32_t body;
    };

    std::vector<AABB> aabbs_;
    std::vector<Entry> entries_;
    std::vector<Entry> sorted_;
    std::vector<uint32_t> bucketStart_;
    long long pairsTested_ = 0;
    float lastCellSize_ = 1.0f;
};

} // namespace impulse
