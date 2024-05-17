#ifndef PROJ_DA_1_MENU_H
#define PROJ_DA_1_MENU_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <set>

#include "Graph.h"
#include "GeoPoint.h"
#include "MutablePriorityQueue.h"
#include <stack>

class TSP {
public:
    // Data parsing
    void parseData(std::string nodesFilePath, std::string edgesFilePath, bool bothFilesProvided);

    // Free allocated space
    void dataGoBoom();

    // T2.1
    // TODO

    // T2.2
    double triangularApproximation(std::stringstream &sd);

    // T2.3
    // TODO

    // T2.4
    double nearestNeighbour(int start);
    bool nnRecursion(int here, int id, std::vector<GeoPoint*> &path, double &count,std::vector<GeoPoint*> &bestPath, double &bestCount);
    // TODO

private:
    Graph<GeoPoint*> tspNetwork; // Graph
    int recursionTimes = 100000;

    std::unordered_map<int, GeoPoint*> geoMap; // Contains all geo points
    std::unordered_map<int, Vertex<GeoPoint*>*> vertexGeoMap; // Contains all vertexes that represent geo points
    std::unordered_map<int, std::set<GeoPoint*>> edgesGeoPoint; //contais the edges of vertex id = int
    void parseEdgesFromMemory(char* data, size_t size);
    void loadFileUsingMMap(const std::string& filename);

    void parsingGeoPointsAndEdges(std::ifstream &in);
    void parsingGeoPoints(std::ifstream &in);
    void parsingEdges(std::ifstream &in);

    //TSP 2
    std::vector<Vertex<GeoPoint*>*> prim(Graph<GeoPoint*> * g, std::vector<Vertex<GeoPoint*>*> &visitOrder);
    bool isAdjacent(Vertex<GeoPoint *> *&v1, Vertex<GeoPoint *> *&v2);
};

#endif //PROJ_DA_1_MENU_H
