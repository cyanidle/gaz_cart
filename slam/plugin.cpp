#include "radapter/radapter.hpp"

namespace gaz_slam {
void registerSlam(radapter::Instance*);
}

RADAPTER_PLUGIN(GazSlam, "radapter.plugins.GazSlam") {
    gaz_slam::registerSlam(radapter);
}

#include "plugin.moc"
