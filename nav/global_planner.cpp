// GlobalPlanner — radapter port of bigbang's global_planer node (A* on the
// costmap, replanning on a timer while a target is active).
//
// Input message fields (former ROS topics):
//   costmap  — output of CostmapServer (was `costmap`, OccupancyGrid)
//   position — { x, y, theta } in meters (was `filtered_pos`, Measure2d)
//   target   — { x, y, theta? } in meters (was `global_planer/target` +
//              `move_base_simple/goal`, merged into one field)
//   cancel   — any non-nil value cancels the current target (was
//              `global_planer/cancel`)
//   status   — LocalPlanner status { reached, rotated, idle_for, ... } (was
//              `planer_status`); the run finishes once the local planner is
//              done (reached && rotated) and has idled long enough — a stuck
//              robot idles too, but never counts as done
//
// Output (data channel):
//   path — list of { x, y, theta } in meters; empty when planning
//          failed / target cancelled (was `global_plan`, nav_msgs/Path)
// Events:
//   planning — "success" | "failed"
//
// Former ROS params are the Lua constructor config; planner:Reload{...}
// re-applies a full config table.

#include <QTimer>
#include <unordered_set>
#include <vector>

#include "radapter/radapter.hpp"
#include "radapter/worker.hpp"
#include "nav_common.hpp"

using namespace radapter;

namespace gaz_nav {

struct AStarConfig {
    WithDefault<double> costmap_to_node_cost_coeff = 5.0;
    WithDefault<double> cell_cost = 10.0;
    WithDefault<double> diagonal_coeff = 1.25;
    WithDefault<int> max_cost = 35;
};
RAD_DESCRIBE(AStarConfig) {
    RAD_MEMBER(costmap_to_node_cost_coeff);
    RAD_MEMBER(cell_cost);
    RAD_MEMBER(diagonal_coeff);
    RAD_MEMBER(max_cost);
}

struct GlobalPlannerConfig {
    WithDefault<AStarConfig> a_star{};
    WithDefault<int> nodes_batch_size = 10000;
    WithDefault<int> reserve_in_path_size = 60;
    WithDefault<int> update_rate_ms = 50;
    WithDefault<int> max_points = 2000;
    WithDefault<double> consider_reached_after = 1.5; // s of local-planner idle
    WithDefault<double> min_time_for_target = 0.5;    // s before idle can finish a target
};
RAD_DESCRIBE(GlobalPlannerConfig) {
    RAD_MEMBER(a_star);
    RAD_MEMBER(nodes_batch_size);
    RAD_MEMBER(reserve_in_path_size);
    RAD_MEMBER(update_rate_ms);
    RAD_MEMBER(max_points);
    RAD_MEMBER(consider_reached_after);
    RAD_MEMBER(min_time_for_target);
}

struct LocalStatus {
    WithDefault<double> idle_for = 0.0;
    WithDefault<bool> reached = false;
    WithDefault<bool> rotated = false;
};
RAD_DESCRIBE(LocalStatus) {
    RAD_MEMBER(idle_for);
    RAD_MEMBER(reached);
    RAD_MEMBER(rotated);
}

struct PlannerNode {
    nav::Coord coord{};
    float theta = 0;
    quint32 parent = 0;
    float totalCost = 0;
};

class GlobalPlanner final : public Worker {
    GlobalPlannerConfig config;

    nav::Grid costmap;
    nav::Position position;
    nav::Position targetPos;
    PlannerNode target;
    bool canceled = true;
    bool reached = false;

    std::vector<PlannerNode> graph;
    std::unordered_set<quint32> open;
    std::unordered_set<nav::Coord, nav::CoordHash> covered;
    int currentCount = 0;
    double timeSinceNewTarget = 0;

    QTimer* updateTimer;

public:
    GlobalPlanner(GlobalPlannerConfig conf, Instance* inst) :
        Worker(inst, "global_planner"),
        updateTimer(new QTimer(this))
    {
        updateTimer->callOnTimeout(this, &GlobalPlanner::update);
        apply(std::move(conf));
    }

    QVariant Reload(GlobalPlannerConfig conf) {
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
        if (auto tgt = map.value("target"); !tgt.isNull()) {
            newTarget(ParseAs<nav::Position>(tgt));
        }
        if (!map.value("cancel").isNull()) {
            cancelTarget();
        }
        if (auto st = map.value("status"); !st.isNull()) {
            onLocalStatus(st);
        }
    }

private:
    void apply(GlobalPlannerConfig conf) {
        config = std::move(conf);
        graph.reserve(size_t(config.nodes_batch_size.value));
        open.reserve(size_t(config.nodes_batch_size.value));
        covered.reserve(size_t(config.nodes_batch_size.value));
        updateTimer->start(config.update_rate_ms);
    }

    AStarConfig const& aStar() const { return config.a_star.value; }

    void newTarget(nav::Position const& t) {
        timeSinceNewTarget = 0;
        targetPos = t;
        canceled = false;
        reached = false;
        Info("new target: {},{} ({} rad)", t.x, t.y, t.theta);
    }

    void cancelTarget() {
        if (!reached) {
            canceled = true;
            reached = true;
            publishPath({});
        }
    }

    void onLocalStatus(QVariant const& msg) {
        auto status = ParseAs<LocalStatus>(msg);
        // idle_for also grows while stuck — only reached && rotated is "done"
        if (!canceled &&
            status.reached && status.rotated &&
            timeSinceNewTarget > config.min_time_for_target &&
            status.idle_for >= config.consider_reached_after.value)
        {
            Info("target finished (local planner done, idle for {} s)", status.idle_for);
            cancelTarget();
        }
    }

    quint32 append(PlannerNode node) {
        graph.push_back(node);
        return quint32(graph.size() - 1);
    }

    quint32 bestOpenIndex() const {
        quint32 best = 0;
        float minCost = std::numeric_limits<float>::max();
        for (auto idx : open) {
            auto& node = graph[idx];
            if (node.totalCost < minCost) {
                minCost = node.totalCost;
                best = idx;
            }
        }
        return best;
    }

    float cost(nav::Coord coord, bool diagonalStep) const {
        auto dx = double(target.coord.x - coord.x);
        auto dy = double(target.coord.y - coord.y);
        auto dist = std::sqrt(dx * dx + dy * dy);
        auto stepCost = diagonalStep
            ? aStar().cell_cost * aStar().diagonal_coeff
            : aStar().cell_cost.value;
        auto mapCost = costmap.at(coord) * aStar().costmap_to_node_cost_coeff;
        return float(dist * aStar().cell_cost + stepCost + mapCost);
    }

    void spawnChildren(quint32 parentIndex) {
        auto x = graph[parentIndex].coord.x;
        auto y = graph[parentIndex].coord.y;
        open.erase(parentIndex);
        for (int dx = -1; dx < 2; ++dx) {
            for (int dy = -1; dy < 2; ++dy) {
                if (!dx && !dy) continue;
                nav::Coord current{x + dx, y + dy};
                if (current == target.coord) {
                    reached = true;
                    open.insert(append(PlannerNode{target.coord, target.theta, parentIndex, 0}));
                    graph[0].theta = float(position.theta);
                    return;
                }
                if (covered.find(current) != covered.end()) continue;
                if (!costmap.valid(current)) continue;
                if (costmap.at(current) > aStar().max_cost) continue;
                covered.emplace(current);
                open.insert(append(PlannerNode{current, 0, parentIndex, cost(current, dx && dy)}));
            }
        }
    }

    void update() {
        timeSinceNewTarget += config.update_rate_ms.value / 1000.0;
        if (canceled) return;
        if (costmap.isEmpty()) return; // no costmap received yet

        // resolve the target against the current costmap on every replan
        target.coord = costmap.metersToCells(targetPos.x, targetPos.y);
        target.theta = float(targetPos.theta);
        graph.clear();
        open.clear();
        covered.clear();
        currentCount = 0;
        reached = false;
        append(PlannerNode{costmap.metersToCells(position.x, position.y), float(position.theta), 0, 0});
        spawnChildren(0);
        while (!reached && !canceled) {
            auto best = bestOpenIndex();
            if (!best || currentCount++ >= config.max_points) {
                canceled = true;
                break;
            }
            spawnChildren(best);
        }
        if (!canceled) {
            emit SendEventField("planning", "success");
            publishPath(walkGraphBackwards());
        } else {
            emit SendEventField("planning", "failed");
            Warn("planning failed");
            publishPath({});
        }
        // like the original: keep replanning every tick (even after a failure)
        // until the target is cancelled or finished via local-planner idle
        canceled = false;
        reached = false;
    }

    QVariantList walkGraphBackwards() {
        QVariantList poses;
        poses.reserve(config.reserve_in_path_size);
        auto* node = &graph.back();
        auto endTheta = node->theta;
        auto startTheta = graph[0].theta;
        poses.push_back(pose(*node));
        int count = 0;
        while (node->parent) {
            node = &graph[node->parent];
            count++;
        }
        auto diff = std::remainder(endTheta - startTheta, 2 * M_PI);
        if (diff > M_PI) diff -= 2 * M_PI;
        else if (diff < -M_PI) diff += 2 * M_PI;
        auto thetaStep = count ? diff / count : 0;
        count = 0;
        node = &graph.back();
        while (node->parent) {
            node = &graph[node->parent];
            node->theta = float(endTheta - thetaStep * ++count);
            poses.push_back(pose(*node));
        }
        std::reverse(poses.begin(), poses.end());
        return poses;
    }

    QVariantMap pose(PlannerNode const& node) const {
        auto meters = costmap.cellsToMeters(node.coord);
        return {{"x", meters.x}, {"y", meters.y}, {"theta", double(node.theta)}};
    }

    void publishPath(QVariantList const& poses) {
        emit SendMsg(QVariantMap{{"path", poses}});
    }
};

void registerGlobalPlanner(Instance* inst) {
    inst->RegisterWorker<GlobalPlanner>("GlobalPlanner", {
        {"Reload", AsExtraMethod<&GlobalPlanner::Reload>},
    });
    inst->RegisterSchema<GlobalPlannerConfig>("GlobalPlanner");
}

} // namespace gaz_nav
