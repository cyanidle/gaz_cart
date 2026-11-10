// Native HID joystick worker: turns Universal RC Joystick reports into
// normalised axis/button messages.
//
// Report layout comes from the URCJ firmware
// (VBCores/8-Axis-Universal-RC-Joystick): N little-endian 16-bit axes in
// 0..32767, then one button byte (bit 0 = button 1), so the 8-axis build reads
// as 17 bytes. The firmware clamps each RC channel to 1000..2000 and scales it
// to 0..0x7fff, which is why -1..1 is recovered as 2 * raw / 0x7fff - 1.

#include "radapter/config.hpp"
#include "radapter/radapter.hpp"
#include "radapter/worker.hpp"

#include <QSocketNotifier>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

using namespace radapter;

namespace gaz_joystick {

namespace {

constexpr int kAxisMax = 0x7fff;
constexpr int kButtonBits = 8;

QString axisName(int index) {
    static const QStringList names{"x", "y", "z", "rx", "ry", "rz", "slider", "dial"};
    if (index < names.size()) return names.at(index);
    return QStringLiteral("axis%1").arg(index + 1);
}

QVariantMap decodeReport(char const* data, qsizetype size) {
    const auto axisCount = (size - 1) / 2;
    QVariantMap axes;
    QVariantList rawAxes;
    rawAxes.reserve(axisCount);
    for (auto i = 0; i < axisCount; ++i) {
        const auto raw = quint16(quint8(data[2 * i]) | (quint8(data[2 * i + 1]) << 8));
        rawAxes.push_back(raw);
        axes.insert(axisName(i), 2.0 * raw / kAxisMax - 1.0);
    }

    const auto bits = quint8(data[size - 1]);
    QVariantList buttons;
    for (auto bit = 0; bit < kButtonBits; ++bit) {
        if (bits & (1 << bit)) buttons.push_back(bit + 1);
    }

    return {
        {"axes", axes},
        {"buttons", buttons},
        {"raw", QVariantMap{{"axes", rawAxes}, {"buttons", bits}}},
    };
}

} // namespace

struct JoystickConfig : WorkerConfig {
    WithDefault<QString> path = QStringLiteral("/dev/hidraw5");
};

RAD_DESCRIBE(JoystickConfig) {
    PARENT(WorkerConfig);
    RAD_MEMBER(path);
}

class Joystick final : public Worker {
    Q_OBJECT

    int fd = -1;
    QSocketNotifier* notifier = nullptr;

    void drain() {
        while (true) {
            char buffer[64];
            const auto n = ::read(fd, buffer, sizeof buffer);
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) return;
                Error("hidraw read failed: {}", strerror(errno));
                notifier->setEnabled(false);
                return;
            }
            if (n == 0) return;
            if (n < 3 || (n - 1) % 2 != 0) {
                Warn("hidraw returned a {} byte report, expected 16-bit axes plus one button byte", n);
                notifier->setEnabled(false);
                return;
            }
            emit SendMsg(decodeReport(buffer, n));
        }
    }

public:
    Joystick(JoystickConfig config, Instance* inst) :
        Worker(inst, config, "joystick")
    {
        fd = ::open(config.path.value.toLocal8Bit().constData(),
            O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) {
            Raise("Joystick: cannot open '{}': {}", config.path.value.toStdString(), strerror(errno));
        }
        notifier = new QSocketNotifier(fd, QSocketNotifier::Read, this);
        connect(notifier, &QSocketNotifier::activated, this, [this] {
            drain();
        });
        Info("reading HID reports from {}", config.path.value.toStdString());
    }

    ~Joystick() override {
        if (fd >= 0) ::close(fd);
    }

    void OnMsg(QVariant const&) override {
        Warn("Joystick is a source worker: it accepts no input messages");
    }
};

void registerJoystick(Instance* instance) {
    instance->RegisterSchema<JoystickConfig>("Joystick");
    instance->RegisterWorker<Joystick>("Joystick");
}

} // namespace gaz_joystick

#include "joystick.moc"
