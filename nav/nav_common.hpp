#pragma once

// Shared grid/costmap core for the gaz_nav radapter plugin.
// Ported from bigbang/include/common/costmap.hpp, stripped of ROS/OpenCV/Qt5.
//
// A Grid's entire state is ONE raw QByteArray: a 16-byte GridHeader
// (magic, width, height, resolution) followed by one cost byte (0..100) per
// cell, row-major. The buffer itself is the wire format: emitting a costmap
// message is `grid.bytes()` — a COW share, no copy — and it crosses into Lua
// as an immutable `bytes` userdata. Consumers adopt the same buffer and only
// ever read through constData(), so they never detach either.

#include <QByteArray>
#include <QVariantMap>
#include <cmath>
#include <cstring>
#include <vector>

#include "radapter/config.hpp"

namespace nav {

struct Coord {
    int x = 0;
    int y = 0;

    friend bool operator==(Coord a, Coord b) noexcept { return a.x == b.x && a.y == b.y; }
    friend bool operator!=(Coord a, Coord b) noexcept { return !(a == b); }
};

struct CoordHash {
    size_t operator()(Coord c) const noexcept {
        return std::hash<qulonglong>{}(qulonglong(quint32(c.x)) << 32 | quint32(c.y));
    }
};

struct Vec2 {
    double x = 0;
    double y = 0;
};

struct DPoint {
    int dx;
    int dy;
    float dist;
};

inline double normalizedTheta(double theta) noexcept {
    theta = std::remainder(theta, 2 * M_PI);
    if (theta > M_PI) theta -= 2 * M_PI;
    else if (theta < -M_PI) theta += 2 * M_PI;
    return theta;
}

struct GridHeader {
    qint32 magic = Magic;
    qint32 width = 0;
    qint32 height = 0;
    float resolution = 0; // meters per cell side

    static constexpr qint32 Magic = 0x47414D50; // "GAMP"
};
static_assert(sizeof(GridHeader) == 16, "wire format");

class Grid {
    QByteArray buf; // GridHeader + width*height cost bytes

public:
    static constexpr int MaxCost = 100;
    static constexpr int HeaderSize = int(sizeof(GridHeader));

    Grid() = default;

    void reset(int width, int height, double resolution) {
        buf = QByteArray(HeaderSize + width * height, '\0');
        GridHeader h{GridHeader::Magic, width, height, float(resolution)};
        std::memcpy(buf.data(), &h, sizeof(h));
    }

    // Adopts a received buffer (COW share). Raises on malformed input.
    static Grid fromBytes(QByteArray bytes) {
        if (bytes.size() < HeaderSize) {
            radapter::Raise("costmap: buffer too small ({} bytes)", bytes.size());
        }
        GridHeader h;
        std::memcpy(&h, bytes.constData(), sizeof(h));
        if (h.magic != GridHeader::Magic) {
            radapter::Raise("costmap: bad magic {:#x}", quint32(h.magic));
        }
        if (bytes.size() != HeaderSize + qsizetype(h.width) * h.height) {
            radapter::Raise("costmap: expected {}x{}+{} = {} bytes, got {}",
                            h.width, h.height, HeaderSize,
                            HeaderSize + qsizetype(h.width) * h.height, bytes.size());
        }
        Grid g;
        g.buf = std::move(bytes);
        return g;
    }

    QByteArray const& bytes() const noexcept { return buf; }
    bool isEmpty() const noexcept { return buf.isEmpty(); }

    GridHeader const& header() const noexcept {
        return *reinterpret_cast<GridHeader const*>(buf.constData());
    }
    int width() const noexcept { return header().width; }
    int height() const noexcept { return header().height; }
    double resolution() const noexcept { return header().resolution; }
    int cellCount() const noexcept { return int(buf.size()) - HeaderSize; }

    const char* cells() const noexcept { return buf.constData() + HeaderSize; }
    char* cells() { return buf.data() + HeaderSize; } // detaches — writers only

    void clear() {
        if (!buf.isEmpty()) std::memset(cells(), 0, size_t(cellCount()));
    }

    bool valid(int x, int y) const noexcept {
        return x >= 0 && x < width() && y >= 0 && y < height();
    }
    bool valid(Coord c) const noexcept { return valid(c.x, c.y); }

    int at(int x, int y) const noexcept { return quint8(cells()[width() * y + x]); }
    int at(Coord c) const noexcept { return at(c.x, c.y); }
    void set(int x, int y, int value) {
        cells()[width() * y + x] = char(qBound(0, value, MaxCost));
    }
    void set(Coord c, int value) { set(c.x, c.y, value); }

    int radiusCells(double meters) const noexcept { return int(meters / resolution()); }
    Coord metersToCells(double x, double y) const noexcept {
        return {int(x / resolution()), int(y / resolution())};
    }
    Vec2 cellsToMeters(Coord c) const noexcept {
        return {c.x * resolution(), c.y * resolution()};
    }

    std::vector<DPoint> makeDPoints(double radiusMeters) const {
        std::vector<DPoint> result;
        auto radius = radiusCells(radiusMeters);
        result.reserve(size_t(4 * (radius + 1) * (radius + 1)));
        for (int dx = 0; dx <= radius; ++dx) {
            for (int dy = 0; dy <= radius; ++dy) {
                auto dist = float(std::sqrt(dx * dx + dy * dy));
                if (dist <= float(radius)) {
                    result.push_back({dx, dy, dist});
                    result.push_back({-dx, dy, dist});
                    result.push_back({dx, -dy, dist});
                    result.push_back({-dx, -dy, dist});
                }
            }
        }
        return result;
    }

    // Saturating per-cell add of another grid of the same size.
    Grid& addCosts(const Grid& src) {
        auto n = std::min(cellCount(), src.cellCount());
        auto* out = cells();
        const auto* in = src.cells();
        for (int i = 0; i < n; ++i) {
            int sum = quint8(out[i]) + quint8(in[i]);
            out[i] = char(sum > MaxCost ? MaxCost : sum);
        }
        return *this;
    }

    void setAll(Coord center, const std::vector<DPoint>& dpoints, int value) {
        for (const auto& d : dpoints) {
            Coord c{center.x + d.dx, center.y + d.dy};
            if (valid(c)) set(c, value);
        }
    }

    // Linear inflation (bigbang's Costmap::inflateInto): every occupied cell
    // radiates cost (R - d) / R * 100 over `dpoints` (precomputed for radius R
    // in cells). Cells fully surrounded by occupancy are skipped.
    void inflateInto(Grid& target, const std::vector<DPoint>& dpoints, double radiusCells) const {
        if (target.cellCount() != cellCount()) {
            target = *this;
        }
        for (int y = 0; y < height(); ++y) {
            for (int x = 0; x < width(); ++x) {
                if (!at(x, y) || isSurrounded(x, y)) continue;
                for (const auto& d : dpoints) {
                    if (d.dist > radiusCells) continue;
                    Coord c{x + d.dx, y + d.dy};
                    if (!target.valid(c)) continue;
                    int wanted = int((radiusCells - d.dist) / radiusCells * MaxCost);
                    if (wanted > target.at(c)) target.set(c, wanted);
                }
            }
        }
    }

    Grid inflated(const std::vector<DPoint>& dpoints, double radiusCells) const {
        Grid copy = *this;
        inflateInto(copy, dpoints, radiusCells);
        return copy;
    }

private:
    bool isFilled(int x, int y) const noexcept { return valid(x, y) ? at(x, y) != 0 : true; }
    bool isSurrounded(int x, int y) const noexcept {
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
                if (!isFilled(x + dx, y + dy)) return false;
        return true;
    }
};

// ---- The `position` / `target` / pose message fields --------------------------

struct Position {
    radapter::WithDefault<double> x = 0.0;
    radapter::WithDefault<double> y = 0.0;
    radapter::WithDefault<double> theta = 0.0;
};
RAD_DESCRIBE(Position) {
    RAD_MEMBER(x);
    RAD_MEMBER(y);
    RAD_MEMBER(theta);
}

} // namespace nav
