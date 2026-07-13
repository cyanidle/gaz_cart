// CostmapServer — radapter port of bigbang's costmap_server node.
//
// Input message fields (former ROS topics):
//   static_map — a Grid buffer from SLAM, merged with the configured image
//                and inflated like other live obstacles
//   objects — one { x, y, size, ttl, id, source_id } or a list of them
//             (was `obstacles`, bigbang_eurobot/MapObject; ttl in ms)
//   point   — { x, y } in meters (was `/clicked_point`): a manual obstacle,
//             cleared keep_points_ms after the last one arrives
//
// Output (data channel), every update_rate_ms:
//   costmap — one immutable bytes buffer: GridHeader (magic, width, height,
//             resolution, origin) + one cost/unknown byte per cell.
//             Emitting shares the internal buffer — no copy; any consumer
//             (GlobalPlanner, LocalPlanner, QML) reinterprets it in place.
//
// Former ROS params are the Lua constructor config; costmap:Reload{...}
// re-applies a full config table (same shape, defaults re-applied).

#include <QImage>
#include <QTimer>
#include <QDir>
#include <map>

#include "radapter/radapter.hpp"
#include "radapter/worker.hpp"
#include "nav_common.hpp"

using namespace radapter;

namespace gaz_nav {

struct InflateConfig {
    WithDefault<double> robot_safe_radius = 0.20;
};
RAD_DESCRIBE(InflateConfig) {
    RAD_MEMBER(robot_safe_radius);
}

struct CostmapServerConfig : WorkerConfig {
    WithDefault<int> update_rate_ms = 80;
    WithDefault<int> keep_points_ms = 15000;
    WithDefault<bool> ignore_all_outside = true;
    WithDefault<QString> image = "";
    WithDefault<int> width = 101;
    WithDefault<int> height = 151;
    WithDefault<double> resolution = 0.02;
    WithDefault<InflateConfig> inflate{};
    WithDefault<InflateConfig> inflate_static{};
};
RAD_DESCRIBE(CostmapServerConfig) {
    PARENT(WorkerConfig);
    RAD_MEMBER(update_rate_ms);
    RAD_MEMBER(keep_points_ms);
    RAD_MEMBER(ignore_all_outside);
    RAD_MEMBER(image);
    RAD_MEMBER(width);
    RAD_MEMBER(height);
    RAD_MEMBER(resolution);
    RAD_MEMBER(inflate);
    RAD_MEMBER(inflate_static);
}

struct MapObject {
    WithDefault<double> x = 0.0;
    WithDefault<double> y = 0.0;
    WithDefault<double> size = 0.0;
    WithDefault<double> ttl = 0.0; // ms
    WithDefault<int> id = 0;
    WithDefault<int> source_id = 0;
};
RAD_DESCRIBE(MapObject) {
    RAD_MEMBER(x);
    RAD_MEMBER(y);
    RAD_MEMBER(size);
    RAD_MEMBER(ttl);
    RAD_MEMBER(id);
    RAD_MEMBER(source_id);
}

class CostmapServer final : public Worker {
    CostmapServerConfig config;

    nav::Grid staticInflated; // static map, pre-inflated with inflate_static
    nav::Grid clickedPoints;  // manual `point` obstacles
    nav::Grid objectPoints;   // rasterized `objects`
    nav::Grid slamMap;        // latest occupancy grid from the SLAM worker
    nav::Grid combined;
    nav::Grid output;

    std::vector<nav::DPoint> inflateDPoints;
    double inflateRadiusCells = 0;
    std::map<double, std::vector<nav::DPoint>> dpointsForSizes;
    std::map<int, std::map<int, MapObject>> objects; // source_id -> id -> object

    QTimer* updateTimer;
    QTimer* resetClicked;

public:
    CostmapServer(CostmapServerConfig conf, Instance* inst) :
        Worker(inst, "costmap"),
        updateTimer(new QTimer(this)),
        resetClicked(new QTimer(this))
    {
        resetClicked->setSingleShot(true);
        resetClicked->callOnTimeout(this, [this] { clickedPoints.clear(); });
        updateTimer->callOnTimeout(this, &CostmapServer::update);
        apply(std::move(conf));
    }

    QVariant Reload(CostmapServerConfig conf) {
        apply(std::move(conf));
        return true;
    }

    void OnMsg(QVariant const& msg) override {
        auto map = msg.toMap();
        if (auto bytes = map.value("static_map"); !bytes.isNull()) {
            onStaticMap(bytes.toByteArray());
        }
        if (auto objs = map.value("objects"); !objs.isNull()) {
            if (objs.metaType().id() == QMetaType::QVariantList) {
                for (auto const& o : objs.toList()) onObject(ParseAs<MapObject>(o));
            } else {
                onObject(ParseAs<MapObject>(objs));
            }
        }
        if (auto pt = map.value("point"); !pt.isNull()) {
            onPoint(pt);
        }
    }

private:
    void apply(CostmapServerConfig conf) {
        config = std::move(conf);
        objects.clear();
        resetGeometry(config.width, config.height, config.resolution, 0, 0, true);

        resetClicked->setInterval(config.keep_points_ms);
        updateTimer->start(config.update_rate_ms);
        Info("costmap {}x{} @ {} m/cell, static image: {}",
             config.width.value, config.height.value, config.resolution.value,
             config.image.value.isEmpty() ? "<none>" : config.image.value);
    }

    void resetGeometry(int width, int height, double resolution,
                       double originX, double originY, bool configuredImage) {
        nav::Grid staticMap;
        staticMap.reset(width, height, resolution, originX, originY);
        if (configuredImage && !config.image.value.isEmpty()) loadImage(staticMap);

        clickedPoints.resetLike(staticMap);
        objectPoints.resetLike(staticMap);
        slamMap.resetLike(staticMap);
        combined.resetLike(staticMap);
        output.resetLike(staticMap);

        inflateRadiusCells = staticMap.radiusCells(config.inflate.value.robot_safe_radius);
        inflateDPoints = staticMap.makeDPoints(config.inflate.value.robot_safe_radius);
        dpointsForSizes.clear();

        const auto staticRadius =
            double(staticMap.radiusCells(config.inflate_static.value.robot_safe_radius));
        staticInflated = staticRadius > 0
            ? staticMap.inflated(
                staticMap.makeDPoints(config.inflate_static.value.robot_safe_radius), staticRadius)
            : staticMap;
    }

    void loadImage(nav::Grid& into) {
        auto path = config.image.value;
        if (path.startsWith("~/")) path = QDir::homePath() + path.mid(1);
        QImage img(path);
        if (img.isNull()) {
            Raise("CostmapServer: could not read image: {}", path);
        }
        if (img.width() != into.width() || img.height() != into.height()) {
            Raise("CostmapServer: image is {}x{}, grid is {}x{}",
                  img.width(), img.height(), into.width(), into.height());
        }
        img = img.convertToFormat(QImage::Format_Grayscale8).mirrored();
        for (int y = 0; y < into.height(); ++y) {
            const auto* line = img.constScanLine(y);
            for (int x = 0; x < into.width(); ++x) {
                into.set(x, y, line[x] * nav::Grid::MaxCost / 255);
            }
        }
    }

    void onObject(MapObject obj) {
        if (config.ignore_all_outside && !output.valid(output.metersToCells(obj.x, obj.y))) {
            return;
        }
        if (dpointsForSizes.find(obj.size) == dpointsForSizes.end()) {
            dpointsForSizes[obj.size] = objectPoints.makeDPoints(obj.size.value / 2);
        }
        objects[obj.source_id][obj.id] = std::move(obj);
    }

    void onStaticMap(QByteArray const& bytes) {
        auto incoming = nav::Grid::fromBytes(bytes);
        if (!incoming.sameGeometry(output)) {
            if (!config.image.value.isEmpty()) {
                Raise("CostmapServer: variable SLAM geometry cannot be combined with fixed image '{}'",
                      config.image.value);
            }
            resetGeometry(incoming.width(), incoming.height(), incoming.resolution(),
                          incoming.originX(), incoming.originY(), false);
            Debug("costmap adopted SLAM grid {}x{} @ {}, origin {},{}",
                  incoming.width(), incoming.height(), incoming.resolution(),
                  incoming.originX(), incoming.originY());
        }
        slamMap = std::move(incoming);
    }

    void onPoint(QVariant const& msg) {
        auto p = ParseAs<nav::Position>(msg);
        auto coord = clickedPoints.metersToCells(p.x, p.y);
        if (!clickedPoints.valid(coord)) return;
        clickedPoints.set(coord, nav::Grid::MaxCost);
        resetClicked->start();
    }

    void updateObjects() {
        objectPoints.clear();
        for (auto& [source, bySource] : objects) {
            for (auto it = bySource.begin(); it != bySource.end();) {
                it->second.ttl.value -= config.update_rate_ms;
                if (it->second.ttl.value <= 0) {
                    it = bySource.erase(it);
                    continue;
                }
                auto coord = objectPoints.metersToCells(it->second.x, it->second.y);
                objectPoints.setAll(coord, dpointsForSizes[it->second.size], nav::Grid::MaxCost);
                ++it;
            }
        }
    }

    void update() {
        output = staticInflated;
        output.addCosts(slamMap);
        updateObjects();
        combined.clear();
        combined.addCosts(clickedPoints).addCosts(objectPoints);
        if (inflateRadiusCells > 0) {
            slamMap.inflateInto(output, inflateDPoints, inflateRadiusCells);
            combined.inflateInto(output, inflateDPoints, inflateRadiusCells);
        } else {
            output.addCosts(combined);
        }
        emit SendMsg(QVariantMap{{"costmap", output.bytes()}});
    }
};

void registerCostmapServer(Instance* inst) {
    inst->RegisterWorker<CostmapServer>("CostmapServer", {
        {"Reload", AsExtraMethod<&CostmapServer::Reload>},
    });
    inst->RegisterSchema<CostmapServerConfig>("CostmapServer");
}

} // namespace gaz_nav
