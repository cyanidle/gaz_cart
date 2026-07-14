// Radapter-native online 2D SLAM built on slam_toolbox's Karto scan matcher
// and Ceres pose-graph optimizer.  ROS is intentionally not part of this
// adapter: Lua supplies odometry and LaserScan-shaped maps, and this worker
// emits corrected poses plus the nav stack's compact GAMP occupancy grid.

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "karto_sdk/Mapper.h"
#include "ceres_solver.hpp"

#include <QDateTime>
#include <QTimer>

#include "radapter/radapter.hpp"
#include "radapter/worker.hpp"
#include "nav/nav_common.hpp"

using namespace radapter;

namespace solver_plugins {

RAD_DESCRIBE(LinearSolver) {
    RAD_ENUM(SPARSE_NORMAL_CHOLESKY);
    RAD_ENUM(SPARSE_SCHUR);
    RAD_ENUM(ITERATIVE_SCHUR);
    RAD_ENUM(CGNR);
}

RAD_DESCRIBE(Preconditioner) {
    RAD_ENUM(JACOBI);
    RAD_ENUM(IDENTITY);
    RAD_ENUM(SCHUR_JACOBI);
}

RAD_DESCRIBE(TrustStrategy) {
    RAD_ENUM(LEVENBERG_MARQUARDT);
    RAD_ENUM(DOGLEG);
}

RAD_DESCRIBE(DoglegType) {
    RAD_ENUM(TRADITIONAL_DOGLEG);
    RAD_ENUM(SUBSPACE_DOGLEG);
}

RAD_DESCRIBE(LossFunction) {
    RAD_ENUM(None);
    RAD_ENUM(HuberLoss);
    RAD_ENUM(CauchyLoss);
}

} // namespace solver_plugins

namespace gaz_slam {

struct MapConfig {
    WithDefault<int> width = 101;
    WithDefault<int> height = 151;
    WithDefault<double> resolution = 0.02;
    WithDefault<int> update_interval_ms = 1000;
    WithDefault<int> min_pass_through = 2;
    WithDefault<double> occupancy_threshold = 0.1;
};
RAD_DESCRIBE(MapConfig) {
    RAD_MEMBER(width);
    RAD_MEMBER(height);
    RAD_MEMBER(resolution);
    RAD_MEMBER(update_interval_ms);
    RAD_MEMBER(min_pass_through);
    RAD_MEMBER(occupancy_threshold);
}

struct LaserConfig {
    WithDefault<double> x = 0.0;
    WithDefault<double> y = 0.0;
    WithDefault<double> theta = 0.0;
    WithDefault<double> min_range = 0.0;
    WithDefault<double> max_range = 20.0;
};
RAD_DESCRIBE(LaserConfig) {
    RAD_MEMBER(x);
    RAD_MEMBER(y);
    RAD_MEMBER(theta);
    RAD_MEMBER(min_range);
    RAD_MEMBER(max_range);
}

struct MapperConfig {
    WithDefault<bool> use_scan_matching = true;
    WithDefault<bool> use_scan_barycenter = true;
    WithDefault<double> minimum_time_interval = 3600.0;
    WithDefault<double> minimum_travel_distance = 0.05;
    WithDefault<double> minimum_travel_heading = 0.05;
    WithDefault<int> scan_buffer_size = 40;
    WithDefault<double> scan_buffer_maximum_scan_distance = 10.0;
    WithDefault<double> link_match_minimum_response_fine = 0.1;
    WithDefault<double> link_scan_maximum_distance = 1.5;
    WithDefault<double> loop_search_maximum_distance = 3.0;
    WithDefault<bool> do_loop_closing = true;
    WithDefault<int> loop_match_minimum_chain_size = 10;
    WithDefault<double> loop_match_maximum_variance_coarse = 3.0;
    WithDefault<double> loop_match_minimum_response_coarse = 0.35;
    WithDefault<double> loop_match_minimum_response_fine = 0.45;
    WithDefault<double> correlation_search_space_dimension = 0.5;
    WithDefault<double> correlation_search_space_resolution = 0.01;
    WithDefault<double> correlation_search_space_smear_deviation = 0.1;
    WithDefault<double> loop_search_space_dimension = 8.0;
    WithDefault<double> loop_search_space_resolution = 0.05;
    WithDefault<double> loop_search_space_smear_deviation = 0.03;
    WithDefault<double> distance_variance_penalty = 0.5;
    WithDefault<double> angle_variance_penalty = 1.0;
    WithDefault<double> fine_search_angle_offset = 0.00349;
    WithDefault<double> coarse_search_angle_offset = 0.349;
    WithDefault<double> coarse_angle_resolution = 0.0349;
    WithDefault<double> minimum_angle_penalty = 0.9;
    WithDefault<double> minimum_distance_penalty = 0.5;
    WithDefault<bool> use_response_expansion = true;
};
RAD_DESCRIBE(MapperConfig) {
    RAD_MEMBER(use_scan_matching);
    RAD_MEMBER(use_scan_barycenter);
    RAD_MEMBER(minimum_time_interval);
    RAD_MEMBER(minimum_travel_distance);
    RAD_MEMBER(minimum_travel_heading);
    RAD_MEMBER(scan_buffer_size);
    RAD_MEMBER(scan_buffer_maximum_scan_distance);
    RAD_MEMBER(link_match_minimum_response_fine);
    RAD_MEMBER(link_scan_maximum_distance);
    RAD_MEMBER(loop_search_maximum_distance);
    RAD_MEMBER(do_loop_closing);
    RAD_MEMBER(loop_match_minimum_chain_size);
    RAD_MEMBER(loop_match_maximum_variance_coarse);
    RAD_MEMBER(loop_match_minimum_response_coarse);
    RAD_MEMBER(loop_match_minimum_response_fine);
    RAD_MEMBER(correlation_search_space_dimension);
    RAD_MEMBER(correlation_search_space_resolution);
    RAD_MEMBER(correlation_search_space_smear_deviation);
    RAD_MEMBER(loop_search_space_dimension);
    RAD_MEMBER(loop_search_space_resolution);
    RAD_MEMBER(loop_search_space_smear_deviation);
    RAD_MEMBER(distance_variance_penalty);
    RAD_MEMBER(angle_variance_penalty);
    RAD_MEMBER(fine_search_angle_offset);
    RAD_MEMBER(coarse_search_angle_offset);
    RAD_MEMBER(coarse_angle_resolution);
    RAD_MEMBER(minimum_angle_penalty);
    RAD_MEMBER(minimum_distance_penalty);
    RAD_MEMBER(use_response_expansion);
}

struct SolverConfig {
    WithDefault<solver_plugins::LinearSolver> linear_solver =
        solver_plugins::LinearSolver::SPARSE_NORMAL_CHOLESKY;
    WithDefault<solver_plugins::Preconditioner> preconditioner =
        solver_plugins::Preconditioner::JACOBI;
    WithDefault<solver_plugins::TrustStrategy> trust_strategy =
        solver_plugins::TrustStrategy::LEVENBERG_MARQUARDT;
    WithDefault<solver_plugins::DoglegType> dogleg_type =
        solver_plugins::DoglegType::TRADITIONAL_DOGLEG;
    WithDefault<solver_plugins::LossFunction> loss_function =
        solver_plugins::LossFunction::None;
    WithDefault<int> threads = 1;
    WithDefault<bool> debug_logging = false;
};
RAD_DESCRIBE(SolverConfig) {
    RAD_MEMBER(linear_solver);
    RAD_MEMBER(preconditioner);
    RAD_MEMBER(trust_strategy);
    RAD_MEMBER(dogleg_type);
    RAD_MEMBER(loss_function);
    RAD_MEMBER(threads);
    RAD_MEMBER(debug_logging);
}

struct SlamConfig : WorkerConfig {
    WithDefault<MapConfig> map{};
    WithDefault<LaserConfig> laser{};
    WithDefault<MapperConfig> mapper{};
    WithDefault<SolverConfig> solver{};
    WithDefault<int> throttle_scans = 1;
    // Bound constant-twist projection when a scan and odometry sample do not
    // have exactly the same timestamp. This replaces the old latest-pose hold.
    WithDefault<double> max_odometry_extrapolation = 0.20;
};
RAD_DESCRIBE(SlamConfig) {
    PARENT(WorkerConfig);
    RAD_MEMBER(map);
    RAD_MEMBER(laser);
    RAD_MEMBER(mapper);
    RAD_MEMBER(solver);
    RAD_MEMBER(throttle_scans);
    RAD_MEMBER(max_odometry_extrapolation);
}

struct ScanData {
    std::vector<double> ranges;
    double angleMin = 0;
    double angleIncrement = 0;
    double rangeMin = 0;
    double rangeMax = 0;
    double timestamp = 0;
};

static karto::Pose2 compose(const karto::Pose2& a, const karto::Pose2& b) {
    const auto c = std::cos(a.GetHeading());
    const auto s = std::sin(a.GetHeading());
    return {a.GetX() + c * b.GetX() - s * b.GetY(),
            a.GetY() + s * b.GetX() + c * b.GetY(),
            nav::normalizedTheta(a.GetHeading() + b.GetHeading())};
}

static karto::Pose2 inverse(const karto::Pose2& p) {
    const auto c = std::cos(p.GetHeading());
    const auto s = std::sin(p.GetHeading());
    return {-c * p.GetX() - s * p.GetY(),
             s * p.GetX() - c * p.GetY(),
            -p.GetHeading()};
}

static karto::Pose2 toKarto(nav::Position const& p) {
    return {p.x, p.y, p.theta.value};
}

static QVariantMap toVariant(karto::Pose2 const& p) {
    return {{"x", p.GetX()}, {"y", p.GetY()}, {"theta", p.GetHeading()}};
}

struct PoseCovariance {
    double xx = 0, xy = 0, xt = 0, yy = 0, yt = 0, tt = 0;
};

struct TwistCovariance {
    double vv = 0, vw = 0, ww = 0;
};

struct OdometryState {
    nav::Position pose;
    double linear = 0;
    double angular = 0;
    double timestamp = 0;
    PoseCovariance poseCovariance;
    TwistCovariance twistCovariance;
    bool hasTwist = false;
};

static double scalar(QVariant const& value, QString const& component = {}) {
    if (value.metaType().id() == QMetaType::QVariantMap) {
        return value.toMap().value(component).toDouble();
    }
    return value.toDouble();
}

static PoseCovariance parsePoseCovariance(QVariant const& value) {
    const auto m = value.toMap();
    return {m.value("xx").toDouble(), m.value("xy").toDouble(),
            m.value("xtheta").toDouble(), m.value("yy").toDouble(),
            m.value("ytheta").toDouble(), m.value("thetatheta").toDouble()};
}

static TwistCovariance parseTwistCovariance(QVariant const& value) {
    const auto m = value.toMap();
    return {m.value("linear").toDouble(), m.value("linear_angular").toDouble(),
            m.value("angular").toDouble()};
}

static QVariantMap toVariant(PoseCovariance const& p) {
    return {{"xx", p.xx}, {"xy", p.xy}, {"xtheta", p.xt},
            {"yy", p.yy}, {"ytheta", p.yt}, {"thetatheta", p.tt}};
}

static QVariantMap toVariant(TwistCovariance const& p) {
    return {{"linear", p.vv}, {"linear_angular", p.vw}, {"angular", p.ww}};
}

class Slam final : public Worker {
    SlamConfig config;

    // Dataset owns the laser and every accepted scan. Destruction/reset order
    // matters: Mapper and solver must disappear before their pointed-to data.
    std::unique_ptr<karto::Dataset> dataset;
    std::unique_ptr<solver_plugins::CeresSolver> solver;
    std::unique_ptr<karto::Mapper> mapper;
    karto::LaserRangeFinder* laser = nullptr;

    nav::Grid outputMap;
    OdometryState odometry;
    karto::Pose2 mapToOdom;
    bool hasOdometry = false;
    bool hasCorrection = false;
    bool mapDirty = true;
    bool paused = false;
    int receivedScans = 0;
    int processedScans = 0;
    double lastTimestamp = 0;
    int laserReadingCount = 0;
    double laserAngleMin = 0;
    double laserAngleIncrement = 0;

    QTimer* mapTimer;

public:
    Slam(SlamConfig conf, Instance* inst) :
        Worker(inst, conf, "slam"),
        mapTimer(new QTimer(this))
    {
        mapTimer->callOnTimeout(this, [this] { publishMap(false); });
        apply(std::move(conf));
    }

    QVariant Reload(SlamConfig conf) {
        apply(std::move(conf));
        return true;
    }

    QVariant Reset() {
        initialize();
        publishMap(true);
        return true;
    }

    QVariant Save(QString basePath) {
        if (basePath.isEmpty()) Raise("Slam.Save: path must not be empty");
        try {
            mapper->SaveToFile((basePath + ".posegraph").toStdString());
            dataset->SaveToFile((basePath + ".data").toStdString());
        } catch (std::exception const& e) {
            Raise("Slam.Save: {}", e.what());
        }
        return true;
    }

    void OnMsg(QVariant const& msg) override {
        const auto map = msg.toMap();
        if (map.contains("pause")) paused = map.value("pause").toBool();
        if (!map.value("reset").isNull()) Reset();

        auto odo = map.value("odometry");
        if (odo.isNull()) odo = map.value("position");
        if (!odo.isNull()) {
            odometry = parseOdometry(odo);
            hasOdometry = true;
            publishPoseOnly();
        }

        if (!paused) {
            if (auto scan = map.value("scan"); !scan.isNull()) {
                processScan(scan.toMap());
            }
        }
    }

private:
    MapConfig const& mapConf() const { return config.map.value; }
    LaserConfig const& laserConf() const { return config.laser.value; }
    MapperConfig const& mapperConf() const { return config.mapper.value; }
    SolverConfig const& solverConf() const { return config.solver.value; }

    OdometryState parseOdometry(QVariant const& value) const {
        const auto m = value.toMap();
        auto pose = m.value("pose");
        if (pose.isNull()) pose = m.value("position");
        // Backward compatibility: odometry={x,y,theta} and position={...}.
        if (pose.isNull()) pose = value;

        OdometryState state;
        state.pose = ParseAs<nav::Position>(pose);
        state.timestamp = m.value("timestamp").toDouble();
        if (!std::isfinite(state.timestamp) || state.timestamp <= 0) {
            state.timestamp = QDateTime::currentMSecsSinceEpoch() / 1000.0;
        }

        const auto twist = m.value("twist").toMap();
        auto linear = twist.value("linear");
        if (linear.isNull()) linear = twist.value("v");
        auto angular = twist.value("angular");
        if (angular.isNull()) angular = twist.value("omega");
        if (!linear.isNull() || !angular.isNull()) {
            state.linear = scalar(linear, "x");
            state.angular = scalar(angular, "z");
            state.hasTwist = std::isfinite(state.linear) && std::isfinite(state.angular);
            if (!state.hasTwist) Raise("Slam: odometry twist must be finite");
        }
        state.poseCovariance = parsePoseCovariance(m.value("pose_covariance"));
        state.twistCovariance = parseTwistCovariance(m.value("twist_covariance"));
        return state;
    }

    static void propagate(OdometryState& state, double dt) {
        const double dTheta = state.angular * dt;
        const double dFwd = state.linear * dt;
        const double mid = state.pose.theta.value + dTheta * 0.5;
        const double c = std::cos(mid), s = std::sin(mid);
        const double fxTheta = -dFwd * s, fyTheta = dFwd * c;
        const double gxV = dt * c, gyV = dt * s;
        const double gxW = -dFwd * s * dt * 0.5;
        const double gyW = dFwd * c * dt * 0.5;
        const auto p = state.poseCovariance;
        const auto q = state.twistCovariance;

        PoseCovariance next;
        next.xx = p.xx + 2 * fxTheta * p.xt + fxTheta * fxTheta * p.tt
            + gxV * gxV * q.vv + 2 * gxV * gxW * q.vw + gxW * gxW * q.ww;
        next.xy = p.xy + fxTheta * p.yt + fyTheta * p.xt + fxTheta * fyTheta * p.tt
            + gxV * gyV * q.vv + (gxV * gyW + gxW * gyV) * q.vw + gxW * gyW * q.ww;
        next.xt = p.xt + fxTheta * p.tt + gxV * dt * q.vw + gxW * dt * q.ww;
        next.yy = p.yy + 2 * fyTheta * p.yt + fyTheta * fyTheta * p.tt
            + gyV * gyV * q.vv + 2 * gyV * gyW * q.vw + gyW * gyW * q.ww;
        next.yt = p.yt + fyTheta * p.tt + gyV * dt * q.vw + gyW * dt * q.ww;
        next.tt = p.tt + dt * dt * q.ww;

        state.pose.x += dFwd * c;
        state.pose.y += dFwd * s;
        state.pose.theta = nav::normalizedTheta(state.pose.theta.value + dTheta);
        state.poseCovariance = next;
        state.timestamp += dt;
    }

    OdometryState odometryAt(double timestamp) const {
        auto state = odometry;
        if (!state.hasTwist || !std::isfinite(timestamp) || timestamp <= 0) return state;
        auto dt = timestamp - state.timestamp;
        const auto limit = config.max_odometry_extrapolation.value;
        dt = std::clamp(dt, -limit, limit);
        propagate(state, dt);
        return state;
    }

    PoseCovariance correctedCovariance(PoseCovariance const& p) const {
        if (!hasCorrection) return p;
        const double c = std::cos(mapToOdom.GetHeading());
        const double s = std::sin(mapToOdom.GetHeading());
        return {
            c*c*p.xx - 2*c*s*p.xy + s*s*p.yy,
            c*s*p.xx + (c*c-s*s)*p.xy - c*s*p.yy,
            c*p.xt - s*p.yt,
            s*s*p.xx + 2*c*s*p.xy + c*c*p.yy,
            s*p.xt + c*p.yt,
            p.tt,
        };
    }

    QVariantMap correctedOdometry(OdometryState const& state,
                                   karto::Pose2 const& corrected) const {
        return {
            {"timestamp", state.timestamp},
            {"pose", toVariant(corrected)},
            {"twist", QVariantMap{{"linear", state.linear}, {"angular", state.angular}}},
            {"pose_covariance", toVariant(correctedCovariance(state.poseCovariance))},
            {"twist_covariance", toVariant(state.twistCovariance)},
        };
    }

    void apply(SlamConfig conf) {
        config = std::move(conf);
        if (mapConf().resolution.value <= 0 || mapConf().update_interval_ms.value <= 0) {
            Raise("Slam: map resolution and update_interval_ms must be > 0");
        }
        if (config.throttle_scans.value <= 0) Raise("Slam: throttle_scans must be > 0");
        if (config.max_odometry_extrapolation.value < 0) {
            Raise("Slam: max_odometry_extrapolation must be >= 0");
        }
        initialize();
        mapTimer->start(mapConf().update_interval_ms);
        Info("slam: Karto/Ceres dynamic map @ {} m, loop closing {}",
             mapConf().resolution.value,
             mapperConf().do_loop_closing.value ? "enabled" : "disabled");
    }

    void initialize() {
        mapper.reset();
        solver.reset();
        dataset.reset();
        laser = nullptr;

        dataset = std::make_unique<karto::Dataset>();
        solver = std::make_unique<solver_plugins::CeresSolver>();
        solver_plugins::CeresSolverConfig sc;
        sc.linear_solver = solverConf().linear_solver.value;
        sc.preconditioner = solverConf().preconditioner.value;
        sc.trust_strategy = solverConf().trust_strategy.value;
        sc.dogleg_type = solverConf().dogleg_type.value;
        sc.loss_function = solverConf().loss_function.value;
        sc.num_threads = solverConf().threads.value;
        sc.debug_logging = solverConf().debug_logging.value;
        solver->Configure(sc);

        mapper = std::make_unique<karto::Mapper>();
        configureMapper();
        mapper->SetScanSolver(solver.get());

        outputMap.reset(1, 1, mapConf().resolution, 0, 0, nav::Grid::UnknownCost);
        odometry = {};
        hasOdometry = false;
        mapToOdom = karto::Pose2{};
        hasCorrection = false;
        mapDirty = true;
        receivedScans = 0;
        processedScans = 0;
        lastTimestamp = 0;
        laserReadingCount = 0;
    }

    void configureMapper() {
        auto& c = mapperConf();
        mapper->setParamUseScanMatching(c.use_scan_matching);
        mapper->setParamUseScanBarycenter(c.use_scan_barycenter);
        mapper->setParamMinimumTimeInterval(c.minimum_time_interval);
        mapper->setParamMinimumTravelDistance(c.minimum_travel_distance);
        mapper->setParamMinimumTravelHeading(c.minimum_travel_heading);
        mapper->setParamScanBufferSize(c.scan_buffer_size);
        mapper->setParamScanBufferMaximumScanDistance(c.scan_buffer_maximum_scan_distance);
        mapper->setParamLinkMatchMinimumResponseFine(c.link_match_minimum_response_fine);
        mapper->setParamLinkScanMaximumDistance(c.link_scan_maximum_distance);
        mapper->setParamLoopSearchMaximumDistance(c.loop_search_maximum_distance);
        mapper->setParamDoLoopClosing(c.do_loop_closing);
        mapper->setParamLoopMatchMinimumChainSize(c.loop_match_minimum_chain_size);
        mapper->setParamLoopMatchMaximumVarianceCoarse(c.loop_match_maximum_variance_coarse);
        mapper->setParamLoopMatchMinimumResponseCoarse(c.loop_match_minimum_response_coarse);
        mapper->setParamLoopMatchMinimumResponseFine(c.loop_match_minimum_response_fine);
        mapper->setParamCorrelationSearchSpaceDimension(c.correlation_search_space_dimension);
        mapper->setParamCorrelationSearchSpaceResolution(c.correlation_search_space_resolution);
        mapper->setParamCorrelationSearchSpaceSmearDeviation(c.correlation_search_space_smear_deviation);
        mapper->setParamLoopSearchSpaceDimension(c.loop_search_space_dimension);
        mapper->setParamLoopSearchSpaceResolution(c.loop_search_space_resolution);
        mapper->setParamLoopSearchSpaceSmearDeviation(c.loop_search_space_smear_deviation);
        mapper->setParamDistanceVariancePenalty(c.distance_variance_penalty);
        mapper->setParamAngleVariancePenalty(c.angle_variance_penalty);
        mapper->setParamFineSearchAngleOffset(c.fine_search_angle_offset);
        mapper->setParamCoarseSearchAngleOffset(c.coarse_search_angle_offset);
        mapper->setParamCoarseAngleResolution(c.coarse_angle_resolution);
        mapper->setParamMinimumAnglePenalty(c.minimum_angle_penalty);
        mapper->setParamMinimumDistancePenalty(c.minimum_distance_penalty);
        mapper->setParamUseResponseExpansion(c.use_response_expansion);
        mapper->setParamMinPassThrough(mapConf().min_pass_through);
        mapper->setParamOccupancyThreshold(mapConf().occupancy_threshold);
    }

    ScanData parseScan(QVariantMap const& msg) {
        ScanData scan;
        const auto ranges = msg.value("ranges").toList();
        scan.ranges.reserve(size_t(ranges.size()));
        for (auto const& value : ranges) scan.ranges.push_back(value.toDouble());
        scan.angleMin = msg.value("angle_min", 0.0).toDouble();
        scan.angleIncrement = msg.value("angle_increment").toDouble();
        scan.rangeMin = msg.value("range_min", laserConf().min_range.value).toDouble();
        scan.rangeMax = msg.value("range_max", laserConf().max_range.value).toDouble();
        scan.timestamp = msg.value("timestamp").toDouble();
        if (scan.timestamp <= 0) scan.timestamp = QDateTime::currentMSecsSinceEpoch() / 1000.0;
        if (scan.timestamp <= lastTimestamp) scan.timestamp = lastTimestamp + 1e-6;

        if (scan.ranges.size() < 2 || scan.angleIncrement <= 0 ||
            scan.rangeMax <= scan.rangeMin) {
            Raise("Slam: scan needs >=2 ranges, positive angle_increment, and range_max > range_min");
        }
        return scan;
    }

    void createLaser(ScanData const& scan) {
        laser = karto::LaserRangeFinder::CreateLaserRangeFinder(
            karto::LaserRangeFinder_Custom, karto::Name("gaz_cart_lidar"));
        laser->SetOffsetPose({laserConf().x, laserConf().y, laserConf().theta});
        const auto minimumRange = std::max(scan.rangeMin, laserConf().min_range.value);
        laser->SetMinimumRange(minimumRange);
        laser->SetMaximumRange(scan.rangeMax);
        laser->SetRangeThreshold(std::min(scan.rangeMax, laserConf().max_range.value));
        const auto span = scan.angleIncrement * scan.ranges.size();
        const bool is360 = std::abs(span - 2 * M_PI) <= scan.angleIncrement * 1.1;
        laser->SetMinimumAngle(scan.angleMin);
        laser->SetMaximumAngle(scan.angleMin + scan.angleIncrement *
            (is360 ? scan.ranges.size() : scan.ranges.size() - 1));
        laser->SetAngularResolution(scan.angleIncrement);
        laser->SetIs360Laser(is360);
        dataset->Add(laser, true);
        laserReadingCount = int(scan.ranges.size());
        laserAngleMin = scan.angleMin;
        laserAngleIncrement = scan.angleIncrement;
    }

    void validateLaser(ScanData const& scan) const {
        if (int(scan.ranges.size()) != laserReadingCount ||
            std::abs(scan.angleMin - laserAngleMin) > 1e-6 ||
            std::abs(scan.angleIncrement - laserAngleIncrement) > 1e-6) {
            Raise("Slam: lidar geometry changed; call Reset or Reload before feeding this scan");
        }
    }

    void processScan(QVariantMap const& msg) {
        auto scan = parseScan(msg);
        if (!hasOdometry) {
            auto fallback = msg.value("odometry");
            if (fallback.isNull()) fallback = msg.value("pose");
            if (!fallback.isNull()) {
                odometry = parseOdometry(fallback);
                if (!msg.contains("odometry")) odometry.timestamp = scan.timestamp;
                hasOdometry = true;
            } else {
                Warn("slam: scan ignored until odometry is available");
                return;
            }
        }

        ++receivedScans;
        if (receivedScans % config.throttle_scans.value != 0) return;
        if (!laser) createLaser(scan); else validateLaser(scan);

        auto* rangeScan = new karto::LocalizedRangeScan(laser->GetName(), scan.ranges);
        // Align the odometric prior with this scan instead of holding the last
        // asynchronous pose sample. Constant-twist projection is bounded by
        // max_odometry_extrapolation.
        const auto scanOdometry = odometryAt(scan.timestamp);
        const auto odomPose = toKarto(scanOdometry.pose);
        rangeScan->SetOdometricPose(odomPose);
        rangeScan->SetCorrectedPose(hasCorrection ? compose(mapToOdom, odomPose) : odomPose);
        rangeScan->SetTime(scan.timestamp);

        karto::Matrix3 covariance;
        covariance.SetToIdentity();
        bool processed = false;
        try {
            processed = mapper->Process(rangeScan, &covariance);
        } catch (std::exception const& e) {
            delete rangeScan;
            Raise("Slam: Karto failed: {}", e.what());
        }
        lastTimestamp = scan.timestamp;
        if (!processed) {
            delete rangeScan;
            publishPoseOnly();
            return;
        }

        dataset->Add(rangeScan);
        ++processedScans;
        mapDirty = true;
        const auto corrected = rangeScan->GetCorrectedPose();
        mapToOdom = compose(corrected, inverse(odomPose));
        hasCorrection = true;

        QVariantMap covarianceMsg;
        covarianceMsg["xx"] = covariance(0, 0);
        covarianceMsg["xy"] = covariance(0, 1);
        covarianceMsg["yy"] = covariance(1, 1);
        covarianceMsg["theta"] = covariance(2, 2);
        emit SendMsg(QVariantMap{
            {"position", toVariant(corrected)},
            {"odometry", correctedOdometry(scanOdometry, corrected)},
            {"covariance", covarianceMsg},
            {"scan", correctedScan(rangeScan)},
            {"slam", stats()},
        });
    }

    karto::Pose2 correctedPose(OdometryState const& state) const {
        const auto odom = toKarto(state.pose);
        return hasCorrection ? compose(mapToOdom, odom) : odom;
    }

    void publishPoseOnly() {
        if (!hasOdometry) return;
        const auto corrected = correctedPose(odometry);
        emit SendMsg(QVariantMap{
            {"position", toVariant(corrected)},
            {"odometry", correctedOdometry(odometry, corrected)},
            {"slam", stats()},
        });
    }

    QVariantMap correctedScan(karto::LocalizedRangeScan* scan) const {
        QVariantList points;
        auto const& readings = scan->GetPointReadings();
        points.reserve(int(readings.size()));
        for (auto const& p : readings) {
            points.append(QVariantMap{{"x", p.GetX()}, {"y", p.GetY()}});
        }
        return {{"pose", toVariant(scan->GetCorrectedPose())}, {"points", points}};
    }

    QVariantMap stats() const {
        return {{"received_scans", receivedScans},
                {"processed_scans", processedScans},
                {"localized", hasCorrection},
                {"paused", paused}};
    }

    void publishMap(bool force) {
        if (!force && !mapDirty) return;
        if (processedScans > 0) {
            std::unique_ptr<karto::OccupancyGrid> occupancy(
                karto::OccupancyGrid::CreateFromScans(
                    mapper->GetAllProcessedScans(), mapConf().resolution,
                    kt_int32u(mapConf().min_pass_through.value),
                    mapConf().occupancy_threshold.value));
            if (occupancy && occupancy->GetWidth() > 0 && occupancy->GetHeight() > 0) {
                const auto& origin = occupancy->GetCoordinateConverter()->GetOffset();
                outputMap.reset(occupancy->GetWidth(), occupancy->GetHeight(),
                                occupancy->GetResolution(), origin.GetX(), origin.GetY(),
                                nav::Grid::UnknownCost);
                for (int y = 0; y < outputMap.height(); ++y) {
                    for (int x = 0; x < outputMap.width(); ++x) {
                        const auto state = occupancy->GetValue({x, y});
                        if (state == karto::GridStates_Free) {
                            outputMap.set(x, y, 0);
                        } else if (state == karto::GridStates_Occupied) {
                            outputMap.set(x, y, nav::Grid::MaxCost);
                        } else {
                            outputMap.setUnknown(x, y);
                        }
                    }
                }
            }
        }
        mapDirty = false;
        QVariantMap msg{{"map", outputMap.bytes()}, {"slam", stats()}};
        if (hasOdometry) {
            const auto corrected = correctedPose(odometry);
            msg["position"] = toVariant(corrected);
            msg["odometry"] = correctedOdometry(odometry, corrected);
        }
        emit SendMsg(msg);
    }
};

void registerSlam(Instance* inst) {
    inst->RegisterWorker<Slam>("Slam", {
        {"Reload", AsExtraMethod<&Slam::Reload>},
        {"Reset", AsExtraMethod<&Slam::Reset>},
        {"Save", AsExtraMethod<&Slam::Save>},
    });
    inst->RegisterSchema<SlamConfig>("Slam");
}

} // namespace gaz_slam
