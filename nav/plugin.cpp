#include "radapter/radapter.hpp"
#include "radapter/worker.hpp"

namespace gaz_nav {
void registerCostmapServer(radapter::Instance*);
void registerGlobalPlanner(radapter::Instance*);
void registerLocalPlanner(radapter::Instance*);
void registerLidar(radapter::Instance*);
}

RADAPTER_PLUGIN(GazNav, "radapter.plugins.GazNav") {
    gaz_nav::registerCostmapServer(radapter);
    gaz_nav::registerGlobalPlanner(radapter);
    gaz_nav::registerLocalPlanner(radapter);
    gaz_nav::registerLidar(radapter);
}

#include "plugin.moc"
