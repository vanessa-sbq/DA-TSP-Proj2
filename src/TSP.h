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
    void parseData(std::string nodesFilePath, std::string edgesFilePath, bool bothFilesProvided);

    // Free allocated space
    void dataGoBoom();

    // T2.1
    // TODO

    // T2.2
    // TODO

    // T2.3
    // TODO

    // T2.4
    // TODO

private:
    Graph<GeoPoint*> tspNetwork; // Graph

    std::unordered_map<int, GeoPoint*> geoMap; // Contains all geo points
    std::unordered_map<int, Vertex<GeoPoint*>*> vertexGeoMap; // Contains all vertexes that represent geo points

    void parseEdgesFromMemory(char* data, size_t size);
    void loadFileUsingMMap(const std::string& filename);

    void parsingGeoPointsAndEdges(std::ifstream &in);
    void parsingGeoPoints(std::ifstream &in);
    void parsingEdges(std::ifstream &in);
};

#endif //PROJ_DA_1_MENU_H
