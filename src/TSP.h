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
#include <stack>

class TSP {
public:
    void setIsToyGraph(bool isToyGraph){
        this->isToyGraph = isToyGraph;
    }

    // Data parsing
    void parseData(std::string nodesFilePath, std::string edgesFilePath, bool bothFilesProvided);

    // Free allocated space
    void dataGoBoom();

    // T2.1
    bool makeGraphConnected();
    std::pair<double, std::vector<Vertex<GeoPoint*>*>> tspBTSetup(bool ignoreBound);
    void cleanUpGraph();

    // T2.2
    double triangularApproximation(std::stringstream &sd);

    // T2.3
    double otherHeuristic();

    // T2.4
    bool nnRecursion(int here, int id, std::vector<GeoPoint*> &path, double &count,std::vector<GeoPoint*> &bestPath, double &bestCount);

private:
    int recursionTimes = 100000;
    bool isToyGraph = false;
    bool isExtraGraph = false;
    int numNodesFromExtra = 25; // Number of nodes used if extra graphs selected

    Graph<GeoPoint*> tspNetwork; // Graph
    std::unordered_map<int, GeoPoint*> geoMap; // Contains all geo points
    std::unordered_map<int, Vertex<GeoPoint*>*> vertexGeoMap; // Contains all vertexes that represent geo points
    std::unordered_map<int, std::set<GeoPoint*>> edgesGeoPoint; //contais the edges of vertex id = int

    void parsingGeoPointsAndEdges(std::ifstream &in);
    void parsingGeoPoints(std::ifstream &in);
    void parsingEdges(std::ifstream &in);

    //TSP 1
    void tspRec(unsigned int numVertexes, unsigned int currentVertex, double curBestMin, std::vector<Vertex<GeoPoint*>*>& curPath, double& min, std::vector<Vertex<GeoPoint*>*>& bestPath, bool ignoreBound);
    std::vector<Edge<GeoPoint*>> edgesToRemove;

    //TSP 2
    std::vector<Vertex<GeoPoint*>*> prim(Graph<GeoPoint*> * g);

    //TSP 3
    void createClusters(std::vector<std::set<int>>& clusters, std::vector<int>& centroids, int k, Vertex<GeoPoint*>* rootVertex);
    std::vector<Vertex<GeoPoint*> *> clusterPrim(Graph<GeoPoint*> * g);
    void preOrderCluster(Vertex<GeoPoint*>* root, std::vector<Vertex<GeoPoint*>*>& preorder);
    double getWeightBetween(Vertex<GeoPoint*>* v1, Vertex<GeoPoint*>* v2);

    bool isAdjacent(Vertex<GeoPoint *> *&v1, Vertex<GeoPoint *> *&v2);

};

#endif //PROJ_DA_1_MENU_H
