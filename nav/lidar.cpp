// Lidar — radapter port of bigbang's rplidarnode. Turns a 360° range scan into
// costmap obstacles: parse each beam to a world point (using the robot pose),
// segment consecutive close-together points into objects, and emit them as the
// `objects` MapObject list CostmapServer already consumes. The eurobot-specific
// beacon/monte-carlo localization is dropped — the cart has wheel odometry.
//
// Input message fields:
//   position     — { x, y, theta } robot pose in meters (former `filtered_pos`)
//   scan         — inject an external raw scan: a list of ranges (meters), or a
//                  list of { angle, range } (angle rad, relative to the sensor).
//                  Lets an out-of-process emulator feed scans (former test topic).
//   sim_obstacle — { x, y, radius } add a ground-truth circle the built-in
//                  simulator raycasts against (sim mode only)
//   clear_sim    — drop all ground-truth obstacles (sim mode only)
//
// Output (data channel), once per scan:
//   objects — list of { x, y, size, ttl, id, source_id } detected obstacles
//   scan    — { pose, points:[{x,y}...] } world-frame hits, for visualization
//
// Two acquisition backends, selected by config: `sim` present -> the built-in
// raycasting simulator (headless, no hardware); otherwise the RPLidar SDK opens
// the device and grabs scans on the same mainLoop timer (like the original —
// grabScanDataHq blocks a revolution; fine for now, real hardware tuned later).

#include <QTimer>
#include <QDateTime>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <vector>

#include "radapter/radapter.hpp"
#include "radapter/worker.hpp"
#include "nav_common.hpp"

#include "sl_lidar.h"

using namespace radapter;

namespace gaz_nav {

// ---- small planar-vector helpers (bigbang's CoordF, trimmed) ------------------

struct V2 {
    double x = 0, y = 0;
    V2 operator+(V2 o) const { return {x + o.x, y + o.y}; }
    V2 operator-(V2 o) const { return {x - o.x, y - o.y}; }
    V2 operator*(double s) const { return {x * s, y * s}; }
    double len() const { return std::sqrt(x * x + y * y); }
    double distTo(V2 o) const { return (*this - o).len(); }
    V2 normalized() const { auto l = len(); return l > 1e-9 ? V2{x / l, y / l} : V2{}; }
};

// ---- config -------------------------------------------------------------------

struct ObjectDetectionConfig {
    WithDefault<int> min_points = 3;
    WithDefault<int> split_each = 60;
    WithDefault<double> max_dist = 3.5;               // ignore hits farther than this from robot
    WithDefault<double> max_dist_between_dots = 0.05; // segment break threshold
    WithDefault<double> start_ttl = 800.0;            // ms an object lives after last seen
    WithDefault<double> map_ttl_coeff = 0.8;          // ttl scaling when handed to costmap
    WithDefault<double> max_deviation = 0.02;         // dedup radius between scans, m
    WithDefault<double> min_x = -3.0, max_x = 5.0;
    WithDefault<double> min_y = -3.0, max_y = 6.0;
};
RAD_DESCRIBE(ObjectDetectionConfig) {
    RAD_MEMBER(min_points);
    RAD_MEMBER(split_each);
    RAD_MEMBER(max_dist);
    RAD_MEMBER(max_dist_between_dots);
    RAD_MEMBER(start_ttl);
    RAD_MEMBER(map_ttl_coeff);
    RAD_MEMBER(max_deviation);
    RAD_MEMBER(min_x);
    RAD_MEMBER(max_x);
    RAD_MEMBER(min_y);
    RAD_MEMBER(max_y);
}

struct SimObstacle {
    WithDefault<double> x = 0.0;
    WithDefault<double> y = 0.0;
    WithDefault<double> radius = 0.1;
};
RAD_DESCRIBE(SimObstacle) {
    RAD_MEMBER(x);
    RAD_MEMBER(y);
    RAD_MEMBER(radius);
}

struct SimConfig {
    WithDefault<int> beams = 360;
    WithDefault<double> noise = 0.005; // range noise amplitude, m
    std::vector<SimObstacle> obstacles;
};
RAD_DESCRIBE(SimConfig) {
    RAD_MEMBER(beams);
    RAD_MEMBER(noise);
    RAD_MEMBER(obstacles);
}

struct SerialConfig {
    WithDefault<QString> port = "/dev/ttyUSB0";
    WithDefault<unsigned> baud = 256000u;
};
RAD_DESCRIBE(SerialConfig) {
    RAD_MEMBER(port);
    RAD_MEMBER(baud);
}

struct NetworkConfig {
    WithDefault<QString> host = "192.168.1.25";
    WithDefault<int> port = 20108;
    WithDefault<bool> use_tcp = true;
};
RAD_DESCRIBE(NetworkConfig) {
    RAD_MEMBER(host);
    RAD_MEMBER(port);
    RAD_MEMBER(use_tcp);
}

struct LidarConfig : WorkerConfig {
    WithDefault<double> range_min = 0.15;
    WithDefault<double> range_max = 12.0;
    WithDefault<bool> reversed = true;          // scan comes clockwise
    WithDefault<double> lidar_offset = 0.0;     // sensor yaw mount offset, rad
    WithDefault<double> lidar_x_offset = 0.0;   // sensor mount offset, m
    WithDefault<double> lidar_y_offset = 0.0;
    WithDefault<double> range_correction = 0.0;
    WithDefault<int> source_id = 0;
    WithDefault<double> scan_frequency = 10.0;  // Hz (sim tick / motor RPM)
    WithDefault<QString> scan_mode = "";        // "" -> driver's first mode
    WithDefault<bool> use_serial = true;
    WithDefault<bool> grab_with_interval = false;
    WithDefault<ObjectDetectionConfig> objects{};
    WithDefault<SerialConfig> serial{};
    WithDefault<NetworkConfig> network{};
    std::optional<SimConfig> sim; // present -> simulate; absent -> real hardware
};
RAD_DESCRIBE(LidarConfig) {
    PARENT(WorkerConfig);
    RAD_MEMBER(range_min);
    RAD_MEMBER(range_max);
    RAD_MEMBER(reversed);
    RAD_MEMBER(lidar_offset);
    RAD_MEMBER(lidar_x_offset);
    RAD_MEMBER(lidar_y_offset);
    RAD_MEMBER(range_correction);
    RAD_MEMBER(source_id);
    RAD_MEMBER(scan_frequency);
    RAD_MEMBER(scan_mode);
    RAD_MEMBER(use_serial);
    RAD_MEMBER(grab_with_interval);
    RAD_MEMBER(objects);
    RAD_MEMBER(serial);
    RAD_MEMBER(network);
    RAD_MEMBER(sim);
}

// ---- parsed scan + detected objects ------------------------------------------

struct ParsedNode {
    double range = 0;   // meters; +inf when out of range (invalid)
    V2 position;        // world-frame hit point
};

struct DetectedObject {
    V2 position;
    double size = 0;
    double ttl = 0;
    double distTo = 0;
    int id = -1;
};

// Persists detected objects across scans: dedups by proximity so a stable object
// keeps its id (and thus refreshes rather than re-adds in the costmap), and ages
// out on ttl. (bigbang's ObjectsMap, minus beacon shimmering tracking.)
class ObjectsMap {
    std::map<int, DetectedObject> objs;
    int lastId = 0;
public:
    void clear() { objs.clear(); lastId = 0; }
    void insertFound(DetectedObject obj, double maxDeviation) {
        for (auto& [id, existing] : objs) {
            if (obj.position.distTo(existing.position) < maxDeviation) {
                existing.ttl = obj.ttl;
                existing.position = obj.position;
                existing.size = obj.size;
                existing.distTo = obj.distTo;
                return;
            }
        }
        obj.id = lastId++;
        objs.emplace(obj.id, obj);
    }
    void age(double passedMs) {
        for (auto it = objs.begin(); it != objs.end();) {
            it->second.ttl -= passedMs;
            if (it->second.ttl <= 0) it = objs.erase(it);
            else ++it;
        }
    }
    auto begin() const { return objs.begin(); }
    auto end() const { return objs.end(); }
    size_t size() const { return objs.size(); }
};

// ---- worker ------------------------------------------------------------------

class Lidar final : public Worker {
    LidarConfig config;
    nav::Position pose;
    ObjectsMap objects;
    std::vector<ParsedNode> parsed;
    std::vector<SimObstacle> simObstacles;

    QTimer* mainLoop;
    qint64 lastScanMs = 0;
    quint32 rngState = 0x9e3779b9u;

    sl::ILidarDriver* driver = nullptr; // hardware backend, connected lazily
    sl::IChannel* channel = nullptr;
    bool hwReady = false;

public:
    Lidar(LidarConfig conf, Instance* inst) :
        Worker(inst, conf, "lidar"),
        mainLoop(new QTimer(this))
    {
        mainLoop->callOnTimeout(this, &Lidar::tick);
        apply(std::move(conf));
    }

    ~Lidar() override {
        if (driver) {
            driver->setMotorSpeed(0);
            driver->stop();
            delete driver;
        }
        delete channel;
    }

    QVariant Reload(LidarConfig conf) {
        apply(std::move(conf));
        return true;
    }

    void OnMsg(QVariant const& msg) override {
        auto map = msg.toMap();
        if (auto pos = map.value("position"); !pos.isNull()) {
            pose = ParseAs<nav::Position>(pos);
        }
        if (auto s = map.value("scan"); !s.isNull()) {
            processInjectedScan(s);
        }
        if (auto o = map.value("sim_obstacle"); !o.isNull()) {
            simObstacles.push_back(ParseAs<SimObstacle>(o));
        }
        if (!map.value("clear_sim").isNull()) {
            simObstacles.clear();
        }
    }

private:
    ObjectDetectionConfig const& od() const { return config.objects.value; }

    void apply(LidarConfig conf) {
        config = std::move(conf);
        objects.clear();
        if (config.sim) {
            simObstacles = config.sim->obstacles;
            Info("lidar: simulation ({} beams, {} obstacles)",
                 config.sim->beams.value, int(simObstacles.size()));
        } else if (!hwReady) {
            connectHardware();
        }
        // sim ticks at scan_frequency; hardware grab blocks a full revolution, so
        // fire as fast as possible and let grabScanDataHq pace it (bigbang did 0).
        int interval = config.sim ? int(1000.0 / std::max(1.0, config.scan_frequency.value)) : 0;
        mainLoop->start(interval);
    }

    // ---- scan acquisition -----------------------------------------------------

    void tick() {
        if (config.sim) simScan();
        else grabHardwareScan();
    }

    void simScan() {
        int beams = std::max(1, config.sim->beams.value);
        parsed.clear();
        parsed.reserve(size_t(beams));
        for (int k = 0; k < beams; ++k) {
            double relTheta = 2 * M_PI * k / beams;
            double r = raycast(worldAngle(relTheta)) + noise();
            appendParsed(relTheta, r);
        }
        finishScan();
    }

    void processInjectedScan(QVariant const& v) {
        auto list = v.toList();
        int n = list.size();
        parsed.clear();
        parsed.reserve(size_t(n));
        for (int i = 0; i < n; ++i) {
            auto const& e = list[i];
            double relTheta, range;
            if (e.metaType().id() == QMetaType::QVariantMap) {
                auto m = e.toMap();
                relTheta = m.value("angle").toDouble();
                range = m.value("range").toDouble();
            } else {
                relTheta = 2 * M_PI * i / std::max(1, n);
                range = e.toDouble();
            }
            appendParsed(relTheta, range);
        }
        finishScan();
    }

    void grabHardwareScan() {
        if (!hwReady) return;
        static constexpr size_t MAX_NODES = 8192;
        static std::vector<sl_lidar_response_measurement_node_hq_t> nodes(MAX_NODES);
        size_t count = nodes.size();
        auto res = config.grab_with_interval
            ? driver->getScanDataWithIntervalHq(nodes.data(), count)
            : driver->grabScanDataHq(nodes.data(), count);
        if (!SL_IS_OK(res)) { Warn("lidar grab failed"); return; }
        driver->ascendScanData(nodes.data(), count);
        parsed.clear();
        parsed.reserve(count);
        for (size_t i = 0; i < count; ++i) {
            size_t idx = config.reversed.value ? count - 1 - i : i;
            double angle = nodes[idx].angle_z_q14 * 90.0 / 16384.0 * M_PI / 180.0;
            double relTheta = angle * (config.reversed.value ? -1 : 1);
            double range = nodes[idx].dist_mm_q2 / 4000.0;
            appendParsed(relTheta, range);
        }
        finishScan();
    }

    // ---- parse + detect -------------------------------------------------------

    float noise() {
        if (!config.sim || config.sim->noise.value <= 0) return 0;
        rngState ^= rngState << 13; rngState ^= rngState >> 17; rngState ^= rngState << 5;
        double u = rngState / double(std::numeric_limits<quint32>::max()) * 2 - 1;
        return float(u * config.sim->noise.value);
    }

    V2 lidarPos() const {
        return {pose.x + config.lidar_x_offset.value, pose.y + config.lidar_y_offset.value};
    }
    double worldAngle(double relTheta) const {
        return pose.theta.value + config.lidar_offset.value + relTheta;
    }
    V2 robotPos() const { return {pose.x, pose.y}; }

    // Nearest ground-truth circle hit along a world-frame ray, else range_max.
    double raycast(double ang) const {
        V2 o = lidarPos();
        V2 d{std::cos(ang), std::sin(ang)};
        double best = config.range_max.value;
        for (auto const& ob : simObstacles) {
            V2 oc = V2{ob.x.value, ob.y.value} - o;
            double proj = oc.x * d.x + oc.y * d.y;
            if (proj < 0) continue;
            double perp2 = oc.x * oc.x + oc.y * oc.y - proj * proj;
            double r2 = ob.radius.value * ob.radius.value;
            if (perp2 > r2) continue;
            double hit = proj - std::sqrt(r2 - perp2);
            if (hit > 0 && hit < best) best = hit;
        }
        return best;
    }

    void appendParsed(double relTheta, double range) {
        range += config.range_correction.value;
        ParsedNode node;
        if (range < config.range_min.value || range >= config.range_max.value) {
            node.range = std::numeric_limits<double>::infinity();
            parsed.push_back(node);
            return;
        }
        double a = worldAngle(relTheta);
        node.range = range;
        node.position = lidarPos() + V2{std::cos(a), std::sin(a)} * range;
        parsed.push_back(node);
    }

    void finishScan() {
        detectObjects();
        auto now = QDateTime::currentMSecsSinceEpoch();
        double passed = lastScanMs ? double(now - lastScanMs)
                                   : 1000.0 / config.scan_frequency.value;
        lastScanMs = now;
        objects.age(passed);
        emit SendMsg(QVariantMap{
            {"objects", objectsMsg()},
            {"scan", scanMsg()},
        });
    }

    // bigbang's segment-growing detector: walk beams in angular order, grow a run
    // of consecutive close-together valid hits into one object at its midpoint.
    void detectObjects() {
        auto const& p = od();
        V2 minB{p.min_x.value, p.min_y.value};
        V2 maxB{p.max_x.value, p.max_y.value};
        int start = -1;
        bool wasClose = false;
        for (size_t i = 1; i < parsed.size(); ++i) {
            auto& last = parsed[i - 1];
            auto& cur = parsed[i];
            bool lastValid = std::isfinite(last.range) && robotPos().distTo(last.position) < p.max_dist.value;
            bool curValid = std::isfinite(cur.range) && robotPos().distTo(cur.position) < p.max_dist.value;
            double dist = (std::isfinite(last.range) && std::isfinite(cur.range))
                ? cur.position.distTo(last.position) : std::numeric_limits<double>::infinity();
            bool close = dist <= p.max_dist_between_dots.value;
            if (lastValid && start == -1 && close) start = int(i - 1);
            bool jumped = (!close && wasClose) || !curValid;
            int dots = int(i) - start + 1;
            bool tooMany = dots > p.split_each.value;
            wasClose = close;
            if ((jumped || tooMany || i == parsed.size() - 1) && start != -1) {
                if (dots < p.min_points.value) { start = -1; continue; }
                V2 startPos = parsed[size_t(start)].position;
                V2 stopPos = parsed[i - 1].position;
                V2 middle = startPos + (stopPos - startPos) * 0.5;
                double size = stopPos.distTo(startPos);
                // push the object centre onto its far surface (bigbang correction)
                V2 corr = (middle - robotPos()).normalized() * (size / 1.9);
                DetectedObject obj;
                obj.position = middle + corr;
                obj.size = size;
                obj.ttl = p.start_ttl.value;
                obj.distTo = robotPos().distTo(middle);
                bool inBounds = minB.x < obj.position.x && obj.position.x < maxB.x &&
                                minB.y < obj.position.y && obj.position.y < maxB.y;
                if (robotPos().distTo(obj.position) <= p.max_dist.value && inBounds) {
                    objects.insertFound(obj, p.max_deviation.value);
                }
                start = -1;
            }
        }
    }

    QVariantList objectsMsg() const {
        QVariantList out;
        out.reserve(int(objects.size()));
        for (auto const& [id, obj] : objects) {
            out.append(QVariantMap{
                {"id", id},
                {"source_id", config.source_id.value},
                {"x", obj.position.x},
                {"y", obj.position.y},
                {"size", obj.size},
                {"ttl", obj.ttl * od().map_ttl_coeff.value},
            });
        }
        return out;
    }

    QVariantMap scanMsg() const {
        QVariantList points;
        for (auto const& n : parsed) {
            if (!std::isfinite(n.range)) continue;
            points.append(QVariantMap{{"x", n.position.x}, {"y", n.position.y}});
        }
        return {
            {"pose", QVariantMap{{"x", pose.x}, {"y", pose.y}, {"theta", pose.theta.value}}},
            {"points", points},
        };
    }

    void connectHardware() {
        driver = *sl::createLidarDriver();
        if (config.use_serial.value) {
            channel = *sl::createSerialPortChannel(
                config.serial.value.port.value.toStdString(), config.serial.value.baud);
        } else if (config.network.value.use_tcp.value) {
            channel = *sl::createTcpChannel(
                config.network.value.host.value.toStdString(), config.network.value.port);
        } else {
            channel = *sl::createUdpChannel(
                config.network.value.host.value.toStdString(), config.network.value.port);
        }
        if (!channel || !SL_IS_OK(driver->connect(channel))) {
            Raise("Lidar: could not connect to device");
        }
        sl_lidar_response_device_info_t info;
        if (!SL_IS_OK(driver->getDeviceInfo(info))) {
            Raise("Lidar: could not read device info");
        }
        driver->setMotorSpeed(sl_u16(config.scan_frequency.value * 60));
        std::vector<sl::LidarScanMode> modes;
        driver->getAllSupportedScanModes(modes);
        sl::LidarScanMode chosen{};
        bool found = false;
        for (auto& m : modes) {
            if (config.scan_mode.value.isEmpty() || config.scan_mode.value == m.scan_mode) {
                chosen = m;
                found = true;
                break;
            }
        }
        if (!found) Raise("Lidar: scan mode '{}' unsupported", config.scan_mode.value);
        driver->startScan(false, chosen.id, 0, &chosen);
        hwReady = true;
        Info("lidar: hardware connected ({})",
             config.use_serial.value ? config.serial.value.port.value : config.network.value.host.value);
    }
};

void registerLidar(Instance* inst) {
    inst->RegisterWorker<Lidar>("Lidar", {
        {"Reload", AsExtraMethod<&Lidar::Reload>},
    });
    inst->RegisterSchema<LidarConfig>("Lidar");
}

} // namespace gaz_nav
