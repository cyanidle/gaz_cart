#include "radapter/radapter.hpp"

namespace gaz_frames {
void registerFrames(radapter::Instance*);
}

RADAPTER_PLUGIN(GazFrames, "radapter.plugins.GazFrames") {
    gaz_frames::registerFrames(radapter);
}

#include "plugin.moc"
