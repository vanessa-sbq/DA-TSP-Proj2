#ifndef PROJ_DA_1_MENU_H
#define PROJ_DA_1_MENU_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <unordered_map>

#include "Graph.h"
#include "GeoPoint.h"

class TSP {
public:
    // Data parsing
    // TODO

    // Free allocated space
    // TODO

    // T2.1
    // TODO

    // T2.2
    // TODO

    // T2.3
    // TODO

    // T2.4
    // TODO

private:
    Graph<GeoPoint*> tspNetwork; // Graph  // FIXME: Use pointer?
    std::unordered_map<int, GeoPoint> geoMap; // Contains all geo points
};

#endif //PROJ_DA_1_MENU_H
