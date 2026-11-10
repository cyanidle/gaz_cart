#include "radapter/radapter.hpp"

namespace gaz_joystick {
void registerJoystick(radapter::Instance*);
}

RADAPTER_PLUGIN(GazJoystick, "radapter.plugins.GazJoystick") {
    gaz_joystick::registerJoystick(radapter);
}

#include "plugin.moc"
