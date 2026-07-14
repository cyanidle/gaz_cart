// Native 2-D reference-frame tree for radapter Lua pipelines.
//
// set(parent, child, pose) stores parent <- child. The usual cart chain is
// map -> odom -> base_link. Wheel odometry owns odom -> base_link; localization
// or operator repositioning owns map -> odom.

#include "radapter/config.hpp"
#include "radapter/radapter.hpp"
#include "radapter/worker.hpp"

#include <QHash>
#include <QSet>
#include <cmath>
#include <optional>

using namespace radapter;

namespace gaz_frames {

struct FramesConfig : WorkerConfig {};
RAD_DESCRIBE(FramesConfig) { PARENT(WorkerConfig); }

struct Pose {
    double x = 0;
    double y = 0;
    double theta = 0;
};

struct Edge {
    QString parent;
    Pose transform; // parent <- child
};

static double normalizedTheta(double theta) {
    return std::remainder(theta, 2 * M_PI);
}

static Pose compose(Pose a, Pose b) {
    const auto c = std::cos(a.theta);
    const auto s = std::sin(a.theta);
    return {
        a.x + c * b.x - s * b.y,
        a.y + s * b.x + c * b.y,
        normalizedTheta(a.theta + b.theta),
    };
}

static Pose inverse(Pose transform) {
    const auto c = std::cos(transform.theta);
    const auto s = std::sin(transform.theta);
    return {
        -c * transform.x - s * transform.y,
         s * transform.x - c * transform.y,
        normalizedTheta(-transform.theta),
    };
}

static Pose parsePose(QVariantMap const& value, const char* context) {
    for (const auto& key : {"x", "y"}) {
        if (!value.contains(QLatin1String(key)))
            Raise("Frames.{}: missing '{}'", context, key);
    }
    Pose result{
        value.value("x").toDouble(),
        value.value("y").toDouble(),
        value.value("theta").toDouble(),
    };
    if (!std::isfinite(result.x) || !std::isfinite(result.y) || !std::isfinite(result.theta))
        Raise("Frames.{}: pose values must be finite", context);
    result.theta = normalizedTheta(result.theta);
    return result;
}

static QVariantMap dumpPose(Pose pose) {
    return {{"x", pose.x}, {"y", pose.y}, {"theta", pose.theta}};
}

class Frames final : public Worker {
    QHash<QString, Edge> edges;

    std::pair<QString, Pose> toRoot(QString frame) const {
        Pose transform;
        QSet<QString> visited;
        auto it = edges.constFind(frame);
        while (it != edges.cend()) {
            if (visited.contains(frame))
                Raise("Frames: reference-frame cycle at '{}'", frame.toStdString());
            visited.insert(frame);
            transform = compose(it->transform, transform);
            frame = it->parent;
            it = edges.constFind(frame);
        }
        return {frame, transform};
    }

    std::optional<Pose> lookupPose(QString const& target, QString const& source) const {
        if (target == source) return Pose{};
        const auto [sourceRoot, rootFromSource] = toRoot(source);
        const auto [targetRoot, rootFromTarget] = toRoot(target);
        if (sourceRoot != targetRoot) return std::nullopt;
        return compose(inverse(rootFromTarget), rootFromSource);
    }

    static QVariantMap rotateCovariance(QVariantMap covariance, double theta) {
        if (covariance.isEmpty()) return {};
        const auto c = std::cos(theta);
        const auto s = std::sin(theta);
        const auto xx = covariance.value("xx").toDouble();
        const auto xy = covariance.value("xy").toDouble();
        const auto xt = covariance.value("xtheta").toDouble();
        const auto yy = covariance.value("yy").toDouble();
        const auto yt = covariance.value("ytheta").toDouble();
        const auto tt = covariance.value("thetatheta").toDouble();
        return {
            {"xx", c*c*xx - 2*c*s*xy + s*s*yy},
            {"xy", c*s*xx + (c*c-s*s)*xy - c*s*yy},
            {"xtheta", c*xt - s*yt},
            {"yy", s*s*xx + 2*c*s*xy + c*c*yy},
            {"ytheta", s*xt + c*yt},
            {"thetatheta", tt},
        };
    }

public:
    Frames(FramesConfig config, Instance* inst) : Worker(inst, config, "frames") {}

    void set(QString parent, QString child, QVariantMap transform) {
        if (parent.isEmpty() || child.isEmpty()) Raise("Frames.set: frame names must not be empty");
        if (parent == child) Raise("Frames.set: a frame cannot be its own parent");
        edges.insert(std::move(child), {std::move(parent), parsePose(transform, "set")});
    }

    QVariant lookup(QString target, QString source) {
        auto transform = lookupPose(target, source);
        return transform ? QVariant(dumpPose(*transform)) : QVariant{};
    }

    QVariant transform_pose(QString target, QString source, QVariantMap pose) {
        auto transform = lookupPose(target, source);
        if (!transform) Raise("Frames.transform_pose: no transform from '{}' to '{}'", source.toStdString(), target.toStdString());
        return dumpPose(compose(*transform, parsePose(pose, "transform_pose")));
    }

    void reanchor(QString parent, QString source, QVariantMap sourcePose, QVariantMap desiredPose) {
        set(std::move(parent), std::move(source),
            dumpPose(compose(parsePose(desiredPose, "reanchor"), inverse(parsePose(sourcePose, "reanchor")))));
    }

    QVariant transform_odometry(QVariantMap odometry, QString targetFrame) {
        const auto source = odometry.value("frame_id").toString();
        if (source.isEmpty()) Raise("Frames.transform_odometry: odometry.frame_id is required");
        auto transform = lookupPose(targetFrame, source);
        if (!transform) Raise("Frames.transform_odometry: no transform from '{}' to '{}'", source.toStdString(), targetFrame.toStdString());
        const auto pose = odometry.value("pose").toMap();
        if (pose.isEmpty()) Raise("Frames.transform_odometry: odometry.pose is required");
        odometry.insert("frame_id", targetFrame);
        odometry.insert("pose", dumpPose(compose(*transform, parsePose(pose, "transform_odometry"))));
        const auto covariance = odometry.value("pose_covariance").toMap();
        if (!covariance.isEmpty()) odometry.insert("pose_covariance", rotateCovariance(covariance, transform->theta));
        return odometry;
    }

    void OnMsg(QVariant const& message) override {
        const auto command = message.toMap();
        const auto request = command.value("set").toMap();
        if (request.isEmpty()) {
            Warn("Frames accepts only {set = {parent, child, transform}} pipeline messages");
            return;
        }
        set(request.value("parent").toString(), request.value("child").toString(),
            request.value("transform").toMap());
    }
};

void registerFrames(Instance* instance) {
    instance->RegisterSchema<FramesConfig>("Frames");
    instance->RegisterWorker<Frames>("Frames", ExtraMethods{
        {"set", AsExtraMethod<&Frames::set>},
        {"lookup", AsExtraMethod<&Frames::lookup>},
        {"transform_pose", AsExtraMethod<&Frames::transform_pose>},
        {"reanchor", AsExtraMethod<&Frames::reanchor>},
        {"transform_odometry", AsExtraMethod<&Frames::transform_odometry>},
    });
}

} // namespace gaz_frames
