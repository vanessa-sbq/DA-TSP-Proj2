#ifndef PROJ_DA_1_MENU_H
#define PROJ_DA_1_MENU_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <set>

#include "Graph.h"
#include "GeoPoint.h"
#include "MutablePriorityQueue.h"

class TSP {
public:
    // Data parsing
    void parseData(std::string nodesFilePath, std::string edgesFilePath, bool bothFilesProvided);

    // Free allocated space
    void dataGoBoom();

    // T2.1
    double tspBTSetup();

    // T2.2
    double triangularApproximation();

    // T2.3
    double otherHeuristic();

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

    // TSP 1
    void tspRec(unsigned int numVertexes, unsigned int currentVertex, double curBestMin, std::vector<Vertex<GeoPoint*>*>& curPath, double& min, std::vector<Vertex<GeoPoint*>*>& bestPath);

        //TSP 2
    template <class T>
    std::vector<Vertex<T> *> prim(Graph<T> * g);
    template <class T>
    double spanningTreeCost(const std::vector<Vertex<T> *> &res);
    template <class T>
    void preOrderWalk(Vertex<T>* root, std::vector<Vertex<T>*> &visitOrder);

    // T2.3
    void createClusters(std::vector<std::set<int>>& clusters, int k);
    std::set<Vertex<GeoPoint*> *> clusterPrim(Graph<GeoPoint*> * g);
    void preorderTraversal();
    void primClusterCentroids();

    bool isAdjacent(Vertex<GeoPoint *> *&v1, Vertex<GeoPoint *> *&v2);
};

#endif //PROJ_DA_1_MENU_H
