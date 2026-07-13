#pragma once

// Shared grid/costmap core for the gaz_nav radapter plugin.
// Ported from bigbang/include/common/costmap.hpp, stripped of ROS/OpenCV/Qt5.
//
// A Grid's entire state is ONE raw QByteArray: a 24-byte GridHeader
// (magic, width, height, resolution, world origin x/y) followed by one byte per
// cell, row-major. Costs are 0..100; 255 means unknown. Legacy 16-byte GAMP
// buffers are accepted with an implicit (0, 0) origin. The buffer itself is the
// wire format: emitting a costmap message is `grid.bytes()` — a COW share, no
// copy — and it crosses into Lua as an immutable `bytes` userdata.

#include <QByteArray>
#include <QVariantMap>
#include <cmath>
#include <cstring>
#include <limits>
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
    float originX = 0;    // world coordinate of grid cell (0, 0)
    float originY = 0;

    static constexpr qint32 Magic = 0x47414D50; // "GAMP"
};
static_assert(sizeof(GridHeader) == 24, "wire format");

struct LegacyGridHeader {
    qint32 magic;
    qint32 width;
    qint32 height;
    float resolution;
};
static_assert(sizeof(LegacyGridHeader) == 16, "legacy wire format");

class Grid {
    QByteArray buf; // GridHeader + width*height cost bytes

public:
    static constexpr int MaxCost = 100;
    static constexpr int UnknownCost = 255;
    static constexpr int HeaderSize = int(sizeof(GridHeader));
    static constexpr int LegacyHeaderSize = int(sizeof(LegacyGridHeader));

    Grid() = default;

    void reset(int width, int height, double resolution,
               double originX = 0, double originY = 0, int fill = 0) {
        if (width <= 0 || height <= 0 || !std::isfinite(resolution) || resolution <= 0 ||
            !std::isfinite(originX) || !std::isfinite(originY)) {
            radapter::Raise("costmap: invalid geometry {}x{} @ {}, origin {},{}",
                            width, height, resolution, originX, originY);
        }
        const auto maxCells = qsizetype(std::numeric_limits<int>::max());
        if (qsizetype(width) > maxCells / height) {
            radapter::Raise("costmap: grid {}x{} is too large", width, height);
        }
        const auto cells = qsizetype(width) * height;
        buf = QByteArray(HeaderSize + cells, char(quint8(fill)));
        GridHeader h{GridHeader::Magic, width, height, float(resolution),
                     float(originX), float(originY)};
        std::memcpy(buf.data(), &h, sizeof(h));
    }

    void resetLike(const Grid& other, int fill = 0) {
        reset(other.width(), other.height(), other.resolution(),
              other.originX(), other.originY(), fill);
    }

    // Adopts a received buffer (COW share). Raises on malformed input.
    static Grid fromBytes(QByteArray bytes) {
        if (bytes.size() < LegacyHeaderSize) {
            radapter::Raise("costmap: buffer too small ({} bytes)", bytes.size());
        }
        LegacyGridHeader h;
        std::memcpy(&h, bytes.constData(), sizeof(h));
        if (h.magic != GridHeader::Magic) {
            radapter::Raise("costmap: bad magic {:#x}", quint32(h.magic));
        }
        if (h.width <= 0 || h.height <= 0 || !std::isfinite(h.resolution) || h.resolution <= 0) {
            radapter::Raise("costmap: invalid geometry {}x{} @ {}", h.width, h.height, h.resolution);
        }
        const auto maxCells = qsizetype(std::numeric_limits<int>::max());
        if (qsizetype(h.width) > maxCells / h.height) {
            radapter::Raise("costmap: grid {}x{} is too large", h.width, h.height);
        }
        const auto cells = qsizetype(h.width) * h.height;
        const auto legacySize = LegacyHeaderSize + cells;
        const auto currentSize = HeaderSize + cells;
        if (bytes.size() != legacySize && bytes.size() != currentSize) {
            radapter::Raise("costmap: expected {} or {} bytes for {}x{}, got {}",
                            legacySize, currentSize, h.width, h.height, bytes.size());
        }
        if (bytes.size() == currentSize) {
            GridHeader current;
            std::memcpy(&current, bytes.constData(), sizeof(current));
            if (!std::isfinite(current.originX) || !std::isfinite(current.originY)) {
                radapter::Raise("costmap: invalid origin {},{}", current.originX, current.originY);
            }
        }
        Grid g;
        g.buf = std::move(bytes);
        return g;
    }

    QByteArray const& bytes() const noexcept { return buf; }
    bool isEmpty() const noexcept { return buf.isEmpty(); }

    LegacyGridHeader const& baseHeader() const noexcept {
        return *reinterpret_cast<LegacyGridHeader const*>(buf.constData());
    }
    int width() const noexcept { return baseHeader().width; }
    int height() const noexcept { return baseHeader().height; }
    double resolution() const noexcept { return baseHeader().resolution; }
    int cellCount() const noexcept { return width() * height(); }
    int headerSize() const noexcept {
        return buf.size() == LegacyHeaderSize + cellCount() ? LegacyHeaderSize : HeaderSize;
    }
    double originX() const noexcept {
        return headerSize() == HeaderSize
            ? reinterpret_cast<GridHeader const*>(buf.constData())->originX : 0.0;
    }
    double originY() const noexcept {
        return headerSize() == HeaderSize
            ? reinterpret_cast<GridHeader const*>(buf.constData())->originY : 0.0;
    }
    bool sameGeometry(const Grid& other) const noexcept {
        return width() == other.width() && height() == other.height() &&
            std::abs(resolution() - other.resolution()) <= 1e-6 &&
            std::abs(originX() - other.originX()) <= 1e-6 &&
            std::abs(originY() - other.originY()) <= 1e-6;
    }

    const char* cells() const noexcept { return buf.constData() + headerSize(); }
    char* cells() { const auto offset = headerSize(); return buf.data() + offset; }

    void clear(int value = 0) {
        if (!buf.isEmpty()) std::memset(cells(), quint8(value), size_t(cellCount()));
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
    void setUnknown(int x, int y) { cells()[width() * y + x] = char(UnknownCost); }
    void setUnknown(Coord c) { setUnknown(c.x, c.y); }
    bool isUnknown(int x, int y) const noexcept { return at(x, y) == UnknownCost; }
    bool isUnknown(Coord c) const noexcept { return isUnknown(c.x, c.y); }

    int radiusCells(double meters) const noexcept { return int(meters / resolution()); }
    Coord metersToCells(double x, double y) const noexcept {
        return {int(std::lround((x - originX()) / resolution())),
                int(std::lround((y - originY()) / resolution()))};
    }
    Coord metersDeltaToCells(double x, double y) const noexcept {
        return {int(std::lround(x / resolution())), int(std::lround(y / resolution()))};
    }
    Vec2 cellsToMeters(Coord c) const noexcept {
        return {originX() + c.x * resolution(), originY() + c.y * resolution()};
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

    // Saturating per-cell add. Unknown dominates known values so an obstacle
    // layer cannot accidentally make unexplored space traversable.
    Grid& addCosts(const Grid& src) {
        if (!sameGeometry(src)) {
            radapter::Raise("costmap: cannot combine different grid geometries");
        }
        auto n = cellCount();
        auto* out = cells();
        const auto* in = src.cells();
        for (int i = 0; i < n; ++i) {
            const int a = quint8(out[i]);
            const int b = quint8(in[i]);
            if (a == UnknownCost || b == UnknownCost) {
                out[i] = char(UnknownCost);
            } else {
                const int sum = a + b;
                out[i] = char(sum > MaxCost ? MaxCost : sum);
            }
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
        if (!target.sameGeometry(*this)) {
            radapter::Raise("costmap: cannot inflate into different grid geometry");
        }
        for (int y = 0; y < height(); ++y) {
            for (int x = 0; x < width(); ++x) {
                if (!isObstacleCost(at(x, y)) || isSurrounded(x, y)) continue;
                for (const auto& d : dpoints) {
                    if (d.dist > radiusCells) continue;
                    Coord c{x + d.dx, y + d.dy};
                    if (!target.valid(c) || target.isUnknown(c)) continue;
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
    static bool isObstacleCost(int value) noexcept {
        return value > 0 && value <= MaxCost;
    }
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
    double x = 0.0;
    double y = 0.0;
    radapter::WithDefault<double> theta = 0.0;
};
RAD_DESCRIBE(Position) {
    RAD_MEMBER(x);
    RAD_MEMBER(y);
    RAD_MEMBER(theta);
}

} // namespace nav
