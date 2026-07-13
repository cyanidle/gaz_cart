// LocalPlanner — radapter port of bigbang's local_planer.py (path follower:
// pick the furthest cheaply-reachable point of the global path, drive toward
// it holonomically, rotate into the goal heading, report status).
//
// The blocking ROS services (direct_move / direct_drift / direct_unstable)
// are NOT ported — they were synchronous busy-loops; script such maneuvers in
// Lua by feeding `target`/`path` directly instead.
//
// Input message fields (former ROS topics):
//   path     — list of { x, y, theta } in meters (was `global_plan`);
//              an empty list stops the robot
//   costmap  — CostmapServer's bytes buffer (was `costmap`, OccupancyGrid)
//   position — { x, y, theta } (was `filtered_pos`, Measure2d)
//   pause    — bool (was `monte_carlo_state.is_bad`): freeze in place while true
//
// Output (data channel), every tick (tick_rate Hz):
//   cmd_vel      — { x, y, theta }: body-frame speed fractions (-1..1)
//                  (was `cmd_vel`, Twist)
//   status       — { reached, rotated, is_stuck, driving_for, idle_for }
//                  (was `planer_status`, PlanerStatus — feed back to GlobalPlanner)
//   local_target — { x, y, theta }: the path point currently being driven to
//                  (findBestTarget's pick); only present while a path is active.
//                  Named local_target, NOT target: this stream is piped back
//                  into GlobalPlanner, whose `target` field is a command.
//
// Former ROS params are the Lua constructor config; planner:Reload{...}
// re-applies a full config table.

#include <QTimer>
#include <algorithm>
#include <optional>
#include <vector>

#include "radapter/radapter.hpp"
#include "radapter/worker.hpp"
#include "nav_common.hpp"

using namespace radapter;

namespace gaz_nav {

struct MarginsConfig {
    WithDefault<double> position = 0.03; // m
    WithDefault<double> theta = 0.04;    // rad
};
RAD_DESCRIBE(MarginsConfig) {
    RAD_MEMBER(position);
    RAD_MEMBER(theta);
}

struct PathConfig {
    WithDefault<double> half_slow_per_cost_of = 20.0;
    WithDefault<int> approximation_step_points = 2;
    WithDefault<double> approximation_max_cost = 30.0;
    WithDefault<int> fallback_min_points_count = 3;
};
RAD_DESCRIBE(PathConfig) {
    RAD_MEMBER(half_slow_per_cost_of);
    RAD_MEMBER(approximation_step_points);
    RAD_MEMBER(approximation_max_cost);
    RAD_MEMBER(fallback_min_points_count);
}

// Holonomic (omni-wheel) mode: strafes straight at the target, optionally
// slewing toward the path point's heading as it goes.
struct OmniDriveConfig {
    WithDefault<bool> enable_mid_path_rotation = true;
    WithDefault<double> mid_path_rotation_gain = 1.0; // speed fraction per rad·m of mid-path rotation
};
RAD_DESCRIBE(OmniDriveConfig) {
    RAD_MEMBER(enable_mid_path_rotation);
    RAD_MEMBER(mid_path_rotation_gain);
}

// Differential (tank) mode: turns the body to face the target, then drives
// forward; never strafes (cmd.y stays 0).
struct DiffDriveConfig {
    WithDefault<double> heading_kp = 1.5;          // speed fraction per radian of heading error
    WithDefault<double> turn_in_place_angle = 0.8; // rad; above this error, stop and rotate in place
    WithDefault<bool> allow_reverse = false;       // drive backward toward targets behind the robot
};
RAD_DESCRIBE(DiffDriveConfig) {
    RAD_MEMBER(heading_kp);
    RAD_MEMBER(turn_in_place_angle);
    RAD_MEMBER(allow_reverse);
}

// Motion params common to both modes, plus the two optional mode blocks.
// Exactly one of diff_drive / omni_drive selects the kinematics; if neither is
// given, diff_drive is the default. Setting both is an error (see apply()).
struct DriveConfig {
    WithDefault<double> min_speed = 0.4;              // forward-speed floor, fraction of full
    WithDefault<double> min_rotation_speed = 0.3;     // in-place rotation floor, fraction of full
    WithDefault<double> rotation_gain = 2.0;          // in-place rotation speed fraction per radian
    WithDefault<double> full_speed_distance = 0.5;    // distance at which forward speed saturates at 1.0, m
    std::optional<DiffDriveConfig> diff_drive;        // present => differential (tank) mode
    std::optional<OmniDriveConfig> omni_drive;        // present => holonomic (omni) mode
};
RAD_DESCRIBE(DriveConfig) {
    RAD_MEMBER(min_speed);
    RAD_MEMBER(min_rotation_speed);
    RAD_MEMBER(rotation_gain);
    RAD_MEMBER(full_speed_distance);
    RAD_MEMBER(diff_drive);
    RAD_MEMBER(omni_drive);
}

struct LocalPlannerConfig : WorkerConfig {
    WithDefault<double> tick_rate = 12.0; // Hz
    WithDefault<MarginsConfig> margins{};
    WithDefault<PathConfig> path{};
    WithDefault<DriveConfig> drive{};
};
RAD_DESCRIBE(LocalPlannerConfig) {
    PARENT(WorkerConfig);
    RAD_MEMBER(tick_rate);
    RAD_MEMBER(margins);
    RAD_MEMBER(path);
    RAD_MEMBER(drive);
}

class LocalPlanner final : public Worker {
    LocalPlannerConfig config;
    double tickInterval = 1.0 / 12;

    nav::Grid costmap;
    nav::Position position;
    nav::Position target;
    std::optional<nav::Position> goal;
    std::vector<nav::Position> path;
    bool paused = false;

    struct {
        bool reached = false;
        bool rotated = false;
        bool is_stuck = false;
        double driving_for = 0;
        double idle_for = 0;
    } status;

    QTimer* tickTimer;

public:
    LocalPlanner(LocalPlannerConfig conf, Instance* inst) :
        Worker(inst, conf, "local_planner"),
        tickTimer(new QTimer(this))
    {
        tickTimer->callOnTimeout(this, &LocalPlanner::tick);
        apply(std::move(conf));
    }

    QVariant Reload(LocalPlannerConfig conf) {
        apply(std::move(conf));
        return true;
    }

    void OnMsg(QVariant const& msg) override {
        auto map = msg.toMap();
        if (auto cm = map.value("costmap"); !cm.isNull()) {
            costmap = nav::Grid::fromBytes(cm.toByteArray());
        }
        if (auto pos = map.value("position"); !pos.isNull()) {
            position = ParseAs<nav::Position>(pos);
        }
        if (map.contains("pause")) {
            paused = map.value("pause").toBool();
        }
        if (map.contains("path")) {
            onPath(map.value("path").toList());
        }
        if (!map.value("cancel").isNull()) {
            goal = position;
            target = position;
        }
    }

private:
    DriveConfig const& drive() const { return config.drive.value; }

    void apply(LocalPlannerConfig conf) {
        config = std::move(conf);
        if (config.tick_rate.value <= 0) {
            Raise("LocalPlanner: tick_rate must be > 0");
        }
        auto& d = config.drive.value;
        if (d.diff_drive && d.omni_drive) {
            Raise("LocalPlanner: set only one of drive.diff_drive / drive.omni_drive");
        }
        if (!d.diff_drive && !d.omni_drive) {
            d.diff_drive = DiffDriveConfig{}; // differential is the default mode
        }
        Info("drive mode: {}", d.diff_drive ? "differential" : "holonomic");
        tickInterval = 1.0 / config.tick_rate;
        status = {};
        tickTimer->start(int(tickInterval * 1000));
    }

    void onPath(QVariantList const& msg) {
        std::vector<nav::Position> newPath;
        Parse(newPath, msg);
        if (newPath.empty()) {
            path.clear();
            goal.reset();
        } else {
            path = std::move(newPath);
            goal = path.back();
            target = findBestTarget();
        }
    }

    nav::Position findBestTarget() const {
        size_t pointsFromEnd = 1;
        auto result = path[path.size() - pointsFromEnd];
        auto approxCost = lineCost(position, result);
        while (approxCost > config.path.value.approximation_max_cost) {
            pointsFromEnd += size_t(config.path.value.approximation_step_points.value);
            if (pointsFromEnd >= path.size()) {
                auto fallback = std::min(size_t(config.path.value.fallback_min_points_count.value),
                                         path.size() - 1);
                return path[fallback];
            }
            result = path[path.size() - pointsFromEnd];
            approxCost = lineCost(position, result);
        }
        return result;
    }

    // Cost of the straight cell-line between two points (Bresenham-ish walk).
    double lineCost(nav::Position const& start, nav::Position const& end) const {
        if (costmap.isEmpty()) return 0;
        auto d = costmap.metersDeltaToCells(end.x - start.x, end.y - start.y);
        auto startCell = costmap.metersToCells(start.x, start.y);
        auto endCell = costmap.metersToCells(end.x, end.y);
        int bigger;
        double xcoeff, ycoeff;
        if (std::abs(d.x) > std::abs(d.y)) {
            bigger = std::abs(d.x);
            xcoeff = d.x < 0 ? -1 : 1;
            ycoeff = double(d.y) / bigger;
        } else {
            bigger = std::abs(d.y);
            if (!bigger) return 0;
            xcoeff = double(d.x) / bigger;
            ycoeff = d.y < 0 ? -1 : 1;
        }
        double result = atSafe(startCell);
        for (int step = 0; step < bigger; ++step) {
            result += atSafe({startCell.x + int(std::lround(step * xcoeff)),
                              startCell.y + int(std::lround(step * ycoeff))});
        }
        result += atSafe(endCell);
        return result;
    }

    int atSafe(nav::Coord c) const {
        return costmap.valid(c) ? costmap.at(c) : nav::Grid::MaxCost;
    }

    bool reachedTarget() const {
        auto dx = target.x - position.x;
        auto dy = target.y - position.y;
        return std::sqrt(dx * dx + dy * dy) <= config.margins.value.position;
    }

    bool rotated() const {
        return std::abs(nav::normalizedTheta(target.theta) - nav::normalizedTheta(position.theta))
               <= config.margins.value.theta;
    }

    struct Cmd {
        double x = 0;
        double y = 0;
        double theta = 0;
    };

    // Distance-based forward-speed scale, shared by both modes: full speed
    // beyond full_speed_distance, easing to the min_speed floor as the
    // target gets close.
    double distCoeff() const {
        auto dx = target.x - position.x;
        auto dy = target.y - position.y;
        auto dist = std::sqrt(dx * dx + dy * dy);
        auto coeff = std::min(dist / drive().full_speed_distance, 1.0);
        return std::max(coeff, drive().min_speed.value);
    }

    // apply() guarantees exactly one mode block is set.
    Cmd driveCmd() const {
        return drive().diff_drive ? diffDriveCmd() : holonomicDriveCmd();
    }

    Cmd holonomicDriveCmd() const {
        Cmd cmd;
        auto const& omni = *drive().omni_drive;
        auto dx = target.x - position.x;
        auto dy = target.y - position.y;
        auto dist = std::sqrt(dx * dx + dy * dy);
        // rotate the world-frame direction into the body frame, normalize, scale
        auto cosT = std::cos(-position.theta);
        auto sinT = std::sin(-position.theta);
        auto bx = dx * cosT - dy * sinT;
        auto by = dx * sinT + dy * cosT;
        auto norm = std::sqrt(bx * bx + by * by);
        if (norm > 0) {
            cmd.x = bx / norm * distCoeff();
            cmd.y = by / norm * distCoeff();
        }
        if (omni.enable_mid_path_rotation) {
            cmd.theta = nav::normalizedTheta(nav::normalizedTheta(target.theta) -
                                             nav::normalizedTheta(position.theta))
                        * dist / omni.mid_path_rotation_gain;
            cmd.theta = std::clamp(cmd.theta, -1.0, 1.0);
        }
        return cmd;
    }

    // Tank control: turn the body toward the target, drive forward only once
    // roughly aligned (fading out to a pure in-place turn past turn_in_place_angle),
    // never strafe.
    Cmd diffDriveCmd() const {
        Cmd cmd;
        auto const& diff = *drive().diff_drive;
        auto dx = target.x - position.x;
        auto dy = target.y - position.y;
        auto bearing = std::atan2(dy, dx);
        auto headingErr = nav::normalizedTheta(bearing - position.theta);
        double forwardSign = 1.0;
        if (diff.allow_reverse && std::abs(headingErr) > M_PI / 2) {
            // target is behind: back into it instead of turning ~180 degrees
            headingErr = nav::normalizedTheta(headingErr - M_PI);
            forwardSign = -1.0;
        }
        cmd.theta = std::clamp(diff.heading_kp.value * headingErr, -1.0, 1.0);
        auto absErr = std::abs(headingErr);
        if (absErr < diff.turn_in_place_angle.value) {
            auto align = 1.0 - absErr / diff.turn_in_place_angle.value;
            cmd.x = forwardSign * distCoeff() * align;
        }
        return cmd; // cmd.y stays 0 — a tank cannot strafe
    }

    Cmd rotateCmd() const {
        Cmd cmd;
        auto diff = nav::normalizedTheta(nav::normalizedTheta(target.theta) -
                                         nav::normalizedTheta(position.theta));
        cmd.theta = diff / drive().rotation_gain;
        cmd.theta = std::clamp(cmd.theta, -1.0, 1.0);
        auto sign = diff > 0 ? 1.0 : -1.0;
        if (std::abs(cmd.theta) < drive().min_rotation_speed)
            cmd.theta = sign * drive().min_rotation_speed;
        return cmd;
    }

    // PlanerStatusWrapper transitions
    void stMoving()   { status.driving_for += tickInterval; status.idle_for = 0; status.is_stuck = false; status.reached = false; status.rotated = false; }
    void stStuck()    { status.driving_for = 0; status.idle_for += tickInterval; status.is_stuck = true; status.reached = false; status.rotated = false; }
    void stRotating() { status.driving_for += tickInterval; status.idle_for = 0; status.reached = true; status.rotated = false; status.is_stuck = false; }
    void stDone()     { status.driving_for = 0; status.idle_for += tickInterval; status.reached = true; status.rotated = true; status.is_stuck = false; }
    void stPaused()   { if (status.idle_for) status.idle_for += tickInterval; else status.driving_for += tickInterval; }

    bool reachedGoal() const {
        if (!goal) return true;
        auto dx = goal->x - position.x;
        auto dy = goal->y - position.y;
        return std::sqrt(dx * dx + dy * dy) <= config.margins.value.position;
    }

    void tick() {
        Cmd cmd; // zero: stop
        if (paused) {
            stPaused();
            path.clear();
        } else if (path.empty()) {
            if (reachedGoal()) stDone();
            else stStuck();
        } else if (!reachedTarget()) {
            stMoving();
            cmd = driveCmd();
        } else if (!rotated()) {
            stRotating();
            cmd = rotateCmd();
        } else if (reachedGoal()) {
            stDone();
            path.clear();
            goal.reset();
        } else {
            target = findBestTarget();
            stMoving();
            cmd = driveCmd();
        }
        QVariantMap out{
            {"cmd_vel", QVariantMap{
                {"x", cmd.x},
                {"y", cmd.y},
                {"theta", cmd.theta}}},
            {"status", QVariantMap{
                {"reached", status.reached},
                {"rotated", status.rotated},
                {"is_stuck", status.is_stuck},
                {"driving_for", status.driving_for},
                {"idle_for", status.idle_for}}},
        };
        if (path.size()) {
            out["local_target"] = QVariantMap{
                {"x", target.x},
                {"y", target.y},
                {"theta", target.theta.value},
            };
        } else {
            out["local_target"] = QVariantMap{
                {"x", position.x},
                {"y", position.y},
                {"theta", position.theta.value},
            };
        }
        emit SendMsg(out);
    }
};

void registerLocalPlanner(Instance* inst) {
    inst->RegisterWorker<LocalPlanner>("LocalPlanner", {
        {"Reload", AsExtraMethod<&LocalPlanner::Reload>},
    });
    inst->RegisterSchema<LocalPlannerConfig>("LocalPlanner");
}

} // namespace gaz_nav
