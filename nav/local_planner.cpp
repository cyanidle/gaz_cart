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
//   cmd_vel      — { x, y, theta }: body-frame linear m/s + angular rad/s
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

struct DriveConfig {
    WithDefault<double> min_speed_coeff = 0.4;
    WithDefault<double> min_rotation_spd = 0.3;
    WithDefault<double> full_rot_spd_per_radians = 2.0;
    WithDefault<bool> enable_mid_path_rotation = true;
    WithDefault<double> max_radians_per_meter = 1.0;
    WithDefault<double> max_speed_for_meters = 0.5;
};
RAD_DESCRIBE(DriveConfig) {
    RAD_MEMBER(min_speed_coeff);
    RAD_MEMBER(min_rotation_spd);
    RAD_MEMBER(full_rot_spd_per_radians);
    RAD_MEMBER(enable_mid_path_rotation);
    RAD_MEMBER(max_radians_per_meter);
    RAD_MEMBER(max_speed_for_meters);
}

struct LocalPlannerConfig {
    WithDefault<double> tick_rate = 12.0; // Hz
    WithDefault<MarginsConfig> margins{};
    WithDefault<PathConfig> path{};
    WithDefault<DriveConfig> drive{};
};
RAD_DESCRIBE(LocalPlannerConfig) {
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
    nav::Position target; // lookahead point currently driven to
    nav::Position goal;   // final path point — "done" is judged against this
    bool goalValid = false;
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
        Worker(inst, "local_planner"),
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
        tickInterval = 1.0 / config.tick_rate;
        status = {};
        tickTimer->start(int(tickInterval * 1000));
    }

    void onPath(QVariantList const& msg) {
        std::vector<nav::Position> newPath;
        Parse(newPath, msg);
        if (newPath.empty()) {
            path.clear();
        } else {
            path = std::move(newPath);
            goal = path.back();
            goalValid = true;
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
        auto d = costmap.metersToCells(end.x - start.x, end.y - start.y);
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

    Cmd driveCmd() const {
        Cmd cmd;
        auto dx = target.x - position.x;
        auto dy = target.y - position.y;
        auto dist = std::sqrt(dx * dx + dy * dy);
        auto distCoeff = dist / drive().max_speed_for_meters;
        distCoeff = std::min(distCoeff, 1.0);
        distCoeff = std::max(distCoeff, drive().min_speed_coeff.value);
        // rotate the world-frame direction into the body frame, normalize, scale
        auto cosT = std::cos(-position.theta);
        auto sinT = std::sin(-position.theta);
        auto bx = dx * cosT - dy * sinT;
        auto by = dx * sinT + dy * cosT;
        auto norm = std::sqrt(bx * bx + by * by);
        if (norm > 0) {
            cmd.x = bx / norm * distCoeff;
            cmd.y = by / norm * distCoeff;
        }
        if (drive().enable_mid_path_rotation) {
            cmd.theta = nav::normalizedTheta(nav::normalizedTheta(target.theta) -
                                             nav::normalizedTheta(position.theta))
                        * dist / drive().max_radians_per_meter;
        }
        return cmd;
    }

    Cmd rotateCmd() const {
        Cmd cmd;
        auto diff = nav::normalizedTheta(nav::normalizedTheta(target.theta) -
                                         nav::normalizedTheta(position.theta));
        cmd.theta = diff / drive().full_rot_spd_per_radians;
        auto sign = diff > 0 ? 1.0 : -1.0;
        if (std::abs(cmd.theta) > 1) cmd.theta = sign;
        if (std::abs(cmd.theta) < drive().min_rotation_spd) cmd.theta = sign * drive().min_rotation_spd;
        return cmd;
    }

    // PlanerStatusWrapper transitions
    void stMoving()   { status.driving_for += tickInterval; status.idle_for = 0; status.is_stuck = false; status.reached = false; status.rotated = false; }
    void stStuck()    { status.driving_for = 0; status.idle_for += tickInterval; status.is_stuck = true; status.reached = false; status.rotated = false; }
    void stRotating() { status.driving_for += tickInterval; status.idle_for = 0; status.reached = true; status.rotated = false; status.is_stuck = false; }
    void stDone()     { status.driving_for = 0; status.idle_for += tickInterval; status.reached = true; status.rotated = true; status.is_stuck = false; }
    void stPaused()   { if (status.idle_for) status.idle_for += tickInterval; else status.driving_for += tickInterval; }

    bool reachedGoal() const {
        auto dx = goal.x - position.x;
        auto dy = goal.y - position.y;
        return goalValid && std::sqrt(dx * dx + dy * dy) <= config.margins.value.position;
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
