#include "physics/BroadPhase.hpp"

#include <algorithm>
#include <cmath>

namespace impulse {

AABB computeAABB(const Body& b) {
    if (const auto* c = std::get_if<Circle>(&b.shape)) {
        Vec2 r{c->radius, c->radius};
        return {b.position - r, b.position + r};
    }
    return {b.position, b.position};
}

void BroadPhase::findPairs(const std::vector<Body>& bodies, std::vector<BodyPair>& out) {
    out.clear();
    pairsTested_ = 0;

    aabbs_.resize(bodies.size());
    for (size_t i = 0; i < bodies.size(); ++i) aabbs_[i] = computeAABB(bodies[i]);

    if (mode == BroadPhaseMode::BruteForce) {
        bruteForce(bodies, out);
    } else {
        grid(bodies, out);
    }

    std::sort(out.begin(), out.end(), [](const BodyPair& l, const BodyPair& r) {
        return l.a != r.a ? l.a < r.a : l.b < r.b;
    });
}

void BroadPhase::bruteForce(const std::vector<Body>& bodies, std::vector<BodyPair>& out) {
    const int n = static_cast<int>(bodies.size());
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            ++pairsTested_;
            if (bodies[i].isStatic() && bodies[j].isStatic()) continue;
            if (overlaps(aabbs_[i], aabbs_[j])) out.push_back({i, j});
        }
    }
}

namespace {

int32_t cellCoord(float v, float invCell) {
    return static_cast<int32_t>(std::floor(v * invCell));
}

uint32_t hashCell(int32_t x, int32_t y) {
    // two big primes, the usual spatial hash trick
    return static_cast<uint32_t>(x) * 73856093u ^ static_cast<uint32_t>(y) * 19349663u;
}

} // namespace

void BroadPhase::grid(const std::vector<Body>& bodies, std::vector<BodyPair>& out) {
    const int n = static_cast<int>(bodies.size());
    if (n < 2) return;

    float cs = cellSize;
    if (cs <= 0.0f) {
        // 2x the average body size (based on dynamic bodies only, otherwise
        // one huge static floor would blow up the cell size)
        double total = 0.0;
        int count = 0;
        for (int i = 0; i < n; ++i) {
            if (bodies[i].isStatic()) continue;
            Vec2 ext = aabbs_[i].max - aabbs_[i].min;
            total += std::max(ext.x, ext.y);
            ++count;
        }
        cs = count > 0 ? static_cast<float>(2.0 * total / count) : 1.0f;
    }
    lastCellSize_ = cs;
    const float inv = 1.0f / cs;

    // 1. figure out every cell each body touches
    entries_.clear();
    for (int i = 0; i < n; ++i) {
        const AABB& box = aabbs_[i];
        int32_t x0 = cellCoord(box.min.x, inv), x1 = cellCoord(box.max.x, inv);
        int32_t y0 = cellCoord(box.min.y, inv), y1 = cellCoord(box.max.y, inv);
        for (int32_t y = y0; y <= y1; ++y)
            for (int32_t x = x0; x <= x1; ++x)
                entries_.push_back({x, y, i});
    }

    // 2. counting sort the entries into hash buckets. this is basically a
    // hash table laid out flat in one array, which is a lot friendlier to
    // the cache than a std::unordered_map of vectors.
    uint32_t bucketCount = 1;
    while (bucketCount < entries_.size() * 2) bucketCount <<= 1;
    const uint32_t mask = bucketCount - 1;

    bucketStart_.assign(bucketCount + 1, 0);
    for (const Entry& e : entries_) ++bucketStart_[(hashCell(e.cx, e.cy) & mask) + 1];
    for (uint32_t b = 0; b < bucketCount; ++b) bucketStart_[b + 1] += bucketStart_[b];

    sorted_.resize(entries_.size());
    {
        std::vector<uint32_t>& cursor = bucketStart_; // reuse, fixed up below
        for (const Entry& e : entries_) sorted_[cursor[hashCell(e.cx, e.cy) & mask]++] = e;
        // after the scatter each start points at the next bucket's start, so
        // shift everything back by one
        for (uint32_t b = bucketCount; b > 0; --b) cursor[b] = cursor[b - 1];
        cursor[0] = 0;
    }

    // 3. test pairs inside each bucket
    for (uint32_t b = 0; b < bucketCount; ++b) {
        uint32_t begin = bucketStart_[b];
        uint32_t end = bucketStart_[b + 1];
        for (uint32_t p = begin; p < end; ++p) {
            const Entry& ep = sorted_[p];
            for (uint32_t q = p + 1; q < end; ++q) {
                const Entry& eq = sorted_[q];
                // different cells can land in the same bucket
                if (ep.cx != eq.cx || ep.cy != eq.cy) continue;

                int i = std::min(ep.body, eq.body);
                int j = std::max(ep.body, eq.body);
                ++pairsTested_;
                if (bodies[i].isStatic() && bodies[j].isStatic()) continue;

                const AABB& A = aabbs_[i];
                const AABB& B = aabbs_[j];
                if (!overlaps(A, B)) continue;

                // a pair can share several cells. only report it from the
                // cell holding the min corner of the overlap region, so no
                // hash set is needed to remove duplicates.
                float ox = std::max(A.min.x, B.min.x);
                float oy = std::max(A.min.y, B.min.y);
                if (cellCoord(ox, inv) != ep.cx || cellCoord(oy, inv) != ep.cy) continue;

                out.push_back({i, j});
            }
        }
    }
}

} // namespace impulse
