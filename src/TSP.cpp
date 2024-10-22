#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <unordered_map>
#include "TSP.h"
#include "Application.h"
#include <set>


/**
 * @brief Frees the data on program exit
 */
void TSP::dataGoBoom() {
    for (Vertex<GeoPoint*> *tspVertex: tspNetwork.getVertexSet()) {
        tspNetwork.removeVertex(tspVertex->getInfo());
    }
}

/**
 * @brief Helper function for data parsing
 * @details Helps check if a given csv contains headers.
 * @return -1 if there are no matching headers
 * @return n > 0 where n is the number of headers
 * */
int countHeaders(const std::string& header) {
    std::stringstream ss(header);
    std::string word;
    int count = 0;
    while (std::getline(ss, word, ',')) {
        for (char& c : word) if (isdigit(c)) return -1;
        if (count == -1) break;
        count++;
    }

    if (std::getline(ss, word, '\r')) {
        for (char& c : word) if (isdigit(c)) return -1;

        count++;
    }
    return count;
}

/**
 * @brief Calculates the distance between two locations based on the Haversine formula
 * @param p1 First location
 * @param p2 Second location
 * @return Returns a double representing the distance between p1 and p2
 */
double calculateHaversineDistance(const std::pair<double, double> p1, const std::pair<double, double> p2) {
    double distanceLatitudes = (p2.first - p1.first) * M_PI / 180.0;
    double distanceLongitudes = (p2.second - p1.second) * M_PI / 180.0;

    double lat1 = p1.first * M_PI / 180.0;
    double lat2 = p2.first * M_PI / 180.0;

    double a = pow(sin(distanceLatitudes / 2),2) +
               pow(sin(distanceLongitudes / 2),2) *
               cos(lat1) * cos(lat2);

    double rad = 6371;
    double c = 2 * asin(sqrt(a));

    return rad * c;
}

/* Data parsing begin */

/**
 * @brief Function that helps parse data
 * @param bothFilesProvided If false the first argument will contain the file that incorporates both nodes / edges.
 * If true then first argument will contain the file path for the nodes csv and the second argument will contain the
 * file path for the edges csv.
 */
void TSP::parseData(std::string nodesFilePath, std::string edgesFilePath, bool bothFilesProvided) {
    std::ifstream in;

    if (edgesFilePath == DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_25_EDGES) numNodesFromExtra = 25;
    else if (edgesFilePath == DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_50_EDGES) numNodesFromExtra = 50;
    else if (edgesFilePath == DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_75_EDGES) numNodesFromExtra = 75;
    else if (edgesFilePath == DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_100_EDGES) numNodesFromExtra = 100;
    else if (edgesFilePath == DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_200_EDGES) numNodesFromExtra = 200;
    else if (edgesFilePath == DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_300_EDGES) numNodesFromExtra = 300;
    else if (edgesFilePath == DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_400_EDGES) numNodesFromExtra = 400;
    else if (edgesFilePath == DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_500_EDGES) numNodesFromExtra = 500;
    else if (edgesFilePath == DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_600_EDGES) numNodesFromExtra = 600;
    else if (edgesFilePath == DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_700_EDGES) numNodesFromExtra = 700;
    else if (edgesFilePath == DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_800_EDGES) numNodesFromExtra = 800;
    else if (edgesFilePath == DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_900_EDGES) numNodesFromExtra = 900;

    if (nodesFilePath == DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_NODES) isExtraGraph = true;

    if (bothFilesProvided) {
        in.open(nodesFilePath);
        if (!in.is_open()) {
            std::cout << "Unable to open nodes csv.\n";
            return;
        }
        parsingGeoPoints(in);
        in.close();

        in.open(edgesFilePath);
        if (!in.is_open()) {
            std::cout << "Unable to open edges csv.\n";
            return;
        }
        parsingEdges(in);
        in.close();
    } else {
        in.open(nodesFilePath);
        if (!in.is_open()) {
            std::cout << "Unable to open nodes|edges csv.\n";
            return;
        }
        parsingGeoPointsAndEdges(in);
        in.close();
    }
}

/**
 * @brief Function that helps parsing the nodes/edges that are inside the csv.
 * @details Time Complexity: O(V + E), where V is the number of nodes and E the number of edges to be parsed
 * @details This functions expects the following order:
 *
 *              "origin, destination, distance" or "origin, destination, distance, label origin, label destination"
 */
void TSP::parsingGeoPointsAndEdges(std::ifstream &in) {
    auto stringToLower = [&](std::string& label){for (char& c : label) c = (char)tolower(c);};
    std::string line;
    getline(in, line); // Headers

    bool expectLabelForGeoPoint = false;
    int numberOfHeadings = countHeaders(line);
    expectLabelForGeoPoint = (numberOfHeadings == 5); // If the number of headings is 5 then we are expecting labels.

    // Process nodes
    while (getline(in, line)) {
        std::istringstream s(line);
        std::string origin, destination, distance, label_origin = "", label_destination = "";

        if (expectLabelForGeoPoint) {
            if (!(std::getline(s, origin, ',') && std::getline(s, destination, ',') && std::getline(s, distance, ',')
                  && std::getline(s, label_origin, ',') && std::getline(s, label_destination, '\r'))) {
                std::cerr << "Error in parsingGeoPointsAndEdges, invalid line in the CSV file" << line << std::endl;
            }
        } else {
            if (!(std::getline(s, origin, ',') && std::getline(s, destination, ',') && std::getline(s, distance, '\r'))) {
                std::cerr << "Error in parsingGeoPointsAndEdges, invalid line in the CSV file" << line << std::endl;
            }
        }

        stringToLower(label_origin);
        stringToLower(label_destination);

        GeoPoint *geoPointSource = new GeoPoint(stoi(origin), label_origin, std::stod(distance), 0);
        GeoPoint *geoPointDestination = new GeoPoint(stoi(destination), label_destination, std::stod(distance), 0);


        this->geoMap[stoi(origin)] = geoPointSource;
        this->geoMap[stoi(destination)] = geoPointDestination;

        if(this->vertexGeoMap.find(stoi(origin))==this->vertexGeoMap.end())
            this->vertexGeoMap[stoi(origin)] = tspNetwork.addVertex(geoPointSource);
        if(this->vertexGeoMap.find(stoi(destination))==this->vertexGeoMap.end())
            this->vertexGeoMap[stoi(destination)] = tspNetwork.addVertex(geoPointDestination);
    }

    in.clear();
    in.seekg(0, std::ios::beg); // Return to the beginning of file.
    getline(in, line); // Headers

    // Process edges
    while (getline(in, line)) {
        std::istringstream s(line);
        std::string origin, destination, distance, label_origin = "", label_destination = "";

        if (expectLabelForGeoPoint) {
            if (!(std::getline(s, origin, ',') && std::getline(s, destination, ',') && std::getline(s, distance, ',')
                  && std::getline(s, label_origin, ',') && std::getline(s, label_destination, '\r'))) {
                std::cerr << "Error in parsingGeoPointsAndEdges, invalid line in the CSV file" << line << std::endl;
            }
        } else {
            if (!(std::getline(s, origin, ',') && std::getline(s, destination, ',') && std::getline(s, distance, '\r'))) {
                std::cerr << "Error in parsingGeoPointsAndEdges, invalid line in the CSV file" << line << std::endl;
            }
        }

        Vertex<GeoPoint*>* geoPointSource = vertexGeoMap[stoi(origin)] ;
        Vertex<GeoPoint*>* geoPointTarget = vertexGeoMap[stoi(destination)] ;

        if (geoPointSource == nullptr || geoPointTarget == nullptr) {
            std::cerr << "Error in parsingGeoPointsAndEdges, problem with finding vertex in geoMap.\n";
            continue;
        }

        auto e1 = geoPointSource->addEdge(geoPointTarget, std::stod(distance));

        this->edgesGeoPoint[geoPointSource->getInfo()->getId()].insert(geoPointTarget->getInfo());
        if (e1 == nullptr) {
            std::cerr << "Problem while adding an edge to the graph\n";
            continue;
        }

        auto e2 = geoPointTarget->addEdge(geoPointSource, std::stod(distance));

        this->edgesGeoPoint[geoPointTarget->getInfo()->getId()].insert(geoPointSource->getInfo());

        if (e2 == nullptr) {
            std::cerr << "Problem while adding an edge to the graph\n";
            continue;
        }

        e1->setReverse(e2);
        e2->setReverse(e1);
    }
}

/**
 * @brief Function that helps parsing the nodes that are inside the csv.
 * @details Time Complexity: O(V), where V is the number of nodes to be parsed
 * @details This functions expects the following order: id, longitude, latitude.
 * */
void TSP::parsingGeoPoints(std::ifstream &in) {
    std::string line;
    getline(in, line); // Header
    int nodeCounter = 0; // For Extra Fully Connected Graphs
    while (getline(in, line)) {
        std::istringstream s(line);
        std::string id, longitude, latitude;

        if (!(std::getline(s, id, ',') && std::getline(s, longitude, ',') && std::getline(s, latitude, '\r'))) {
            std::cerr << "Error in parsingGeoPoints, invalid line in the CSV file" << line << std::endl;
        }

        GeoPoint *geoPoint = new GeoPoint(stoi(id), "", std::stod(longitude), std::stod(latitude));

        this->geoMap[stoi(id)] = geoPoint;

        Vertex<GeoPoint*>* vertex = tspNetwork.addVertex(geoPoint);

        if (vertex == nullptr) {
            std::cout << "Error in parsingGeoPoints, error while adding vertex to graph.\n";
        }

        this->vertexGeoMap[stoi(id)] = vertex;

        nodeCounter++;
        if ((nodeCounter == numNodesFromExtra) && isExtraGraph){
            return;
        }
    }
}

/**
 * @brief Function that helps parsing the edges that are inside the csv.
 * @details Time Complexity: O(E), where E is the number of edges to be parsed
 * @details This functions expects the following order: origin, destination, haversine_distance.
 * */
void TSP::parsingEdges(std::ifstream &in) {
    std::string line;
    getline(in, line); // Header
    int numberOfHeadings = countHeaders(line);

    Vertex<GeoPoint*>* geoPointSource = nullptr;
    Vertex<GeoPoint*>* geoPointDestination = nullptr;

    if (numberOfHeadings == -1) { // The header itself is already data...
        std::istringstream s(line);
        std::string origin, destination, distance;
        if (!(std::getline(s, origin, ',') && std::getline(s, destination, ',') && std::getline(s, distance, '\r'))) {
            std::cerr << "Error in parsingEdges, invalid line in the CSV file" << line << std::endl;
        }

        geoPointSource = vertexGeoMap.at(std::stoi(origin));
        geoPointDestination = vertexGeoMap.at(std::stoi(destination));

        if (geoPointSource == nullptr || geoPointDestination == nullptr) {
            std::cerr << "Error in parsingEdges, problem with finding vertex in maps.\n";
        }

        if (!geoPointSource->addEdge(geoPointDestination, std::stod(distance))) {
            std::cerr << "Error in parsingEdges, problem while adding an edge to the graph\n";
        }

        this->edgesGeoPoint[geoPointSource->getInfo()->getId()].insert(geoPointDestination->getInfo());

        if (!geoPointDestination->addEdge(geoPointSource, std::stod(distance))) {
            std::cerr << "Error in parsingEdges, problem while adding an edge to the graph\n";
        }

        this->edgesGeoPoint[geoPointDestination->getInfo()->getId()].insert(geoPointSource->getInfo());
    } // else ignore the header and process the rest of the data.

    while (std::getline(in, line)) {
        std::istringstream s(line);
        std::string origin, destination, distance;

        if (!(std::getline(s, origin, ',') && std::getline(s, destination, ',') && std::getline(s, distance, '\r'))) {
            std::cerr << "Error in parsingEdges, invalid line in the CSV file" << line << std::endl;
        }

        geoPointSource = vertexGeoMap.at(std::stoi(origin));
        geoPointDestination = vertexGeoMap.at(std::stoi(destination));

        if (geoPointSource == nullptr || geoPointDestination == nullptr) {
            std::cerr << "Error in parsingEdges, problem with finding vertex in maps.\n";
            continue;
        }

        if (!geoPointSource->addEdge(geoPointDestination, std::stod(distance))) {
            std::cerr << "Error in parsingEdges, problem while adding an edge to the graph\n";
            continue;
        }

        if (!geoPointDestination->addEdge(geoPointSource, std::stod(distance))) {
            std::cerr << "Error in parsingEdges, problem while adding an edge to the graph\n";
            continue;
        }
    }
}

/* Data parsing end */

/**
 * @brief Helper function.
 * @details Time Complexity: O(E), where E is the number of adjacent edges of the current vertex
 * @details Given two vertexes, the one we are now, and the one we want to go to the function returns nullptr if it fails
 * to find the desired edge and returns the edge that connects the two vertexes in case it finds it.
 **/
Edge<GeoPoint*>* edgeFromCurVertexToNextVertex(Vertex<GeoPoint*>* currentVertex, Vertex<GeoPoint*>* nextVertex) {
    if (currentVertex == nullptr || nextVertex == nullptr) {
        return nullptr;
    }

    for (auto& edge : currentVertex->getAdj()) {
        if (edge->getDest() == nextVertex) {
            return edge;
        }
    }

    return nullptr;
}

/**
 * @brief Helper function.
 * @details Time Complexity: O(E), where E is the number of adjacent edges of the current vertex
 * @details Given two vertexes, the one we are now, and the one we want to go to the function returns the distance calculated with the haversine formula
 * if it fails to find the desired edge and returns the edge that connects the two vertexes in case it finds it.
 **/
Edge<GeoPoint*>* edgeFromCurVertexToNextVertexWithHarversine(Vertex<GeoPoint*>* currentVertex, Vertex<GeoPoint*>* nextVertex) {

    if (currentVertex == nullptr || nextVertex == nullptr) {
        return nullptr;
    }

    for (auto& edge : currentVertex->getAdj()) {
        if (edge->getDest() == nextVertex) {
            return edge;
        }
    }

    auto edge = new Edge<GeoPoint*>(currentVertex,nextVertex, calculateHaversineDistance({currentVertex->getInfo()->getLatitude(),currentVertex->getInfo()->getLongitude()},{nextVertex->getInfo()->getLatitude(),nextVertex->getInfo()->getLongitude()}));

    return edge;
}

/**
 * @brief Makes the graph connected, and sets non-existing edges to infinite distance (Should only be used on Toy Graphs)
 * @details Time Complexity: O(V * V * E), where V is the number of vertices in the graph and E is the number of adjacent edges of each vertex
 * @return Returns a bool telling if the graph was already connected or not
 */
bool TSP::makeGraphConnected() {
    bool isFullyConnected = true;
    for (Vertex<GeoPoint*>*& vertexA : tspNetwork.getVertexSet()) {
        for (Vertex<GeoPoint*>* vertexB : tspNetwork.getVertexSet()) {
            Edge<GeoPoint*>* edgeResult = edgeFromCurVertexToNextVertex(vertexA, vertexB);

            if (edgeResult == nullptr && vertexA != vertexB) {
                isFullyConnected = false;
                edgeResult = vertexA->addEdge(vertexB, INFINITY);
                edgesToRemove.push_back(*edgeResult);
            }
        }
    }
    return isFullyConnected;
}

/**
 * @brief Cleans up the graph after making it connected
 * @details Time Complexity: O(K * E), where K is the number of edges to remove and E is the number of adjacent edges of the origin vertex
 */
void TSP::cleanUpGraph() {
    for (Edge<GeoPoint*> edge : edgesToRemove) {
        tspNetwork.removeEdge(edge.getOrig()->getInfo(), edge.getDest()->getInfo());
    }
    edgesToRemove.clear();

    for (Vertex<GeoPoint*>* geoPointVertex : tspNetwork.getVertexSet()) {
        geoPointVertex->setVisited(false);
        geoPointVertex->setProcesssing(false);
        for (Edge<GeoPoint*>* edge : geoPointVertex->getAdj()) {
            edge->setSelected(false);
        }
    }
}

// T2.1
/**
 * @brief Backtracking and Bounding algorithm to solve the TSP for small graphs
 * @details Time Complexity: O(N!), where N is the number of nodes in the graph
 * @param n number of nodes in the graph
 * @param curI current index
 * @param curDist current distance
 * @param curPath current path
 * @param minDist minimum distance
 * @param path vector that contains the paths
 */
void TSP::tspRec(unsigned int n, unsigned int curI, double curDist, std::vector<Vertex<GeoPoint*>*>& curPath, double& minDist, std::vector<Vertex<GeoPoint*>*>& path, bool ignoreBound) {
    if (curI == n) {
        curDist += edgeFromCurVertexToNextVertex(curPath[n - 1], curPath[0])->getWeight();
        if (curDist < minDist) {
            minDist = curDist;
            path.clear();
            for (int i = 0; i < n; i++) {
                path.push_back(curPath.at(i));
            }
        }
        return;
    }

    for (int i = 1; i < n; i++) {
        if ((curDist + edgeFromCurVertexToNextVertex(curPath.at(curI - 1), curPath.at(curI))->getWeight() < minDist) || ignoreBound) {
            bool newNode = true;
            for (int j = 1; j < curI; j++) {
                if (curPath.at(j)->getInfo()->getId() == i) {
                    newNode = false;
                    break;
                }
            }

            if (newNode) {
                curPath.at(curI) = vertexGeoMap[i];
                double dist = edgeFromCurVertexToNextVertex(curPath.at(curI - 1), curPath.at(curI))->getWeight();
                tspRec(n, curI + 1 , curDist + dist, curPath ,minDist, path, ignoreBound);

            }
        }
    }
}

/**
 * @brief Set up function for the backtracking and bounding algorithm
 * @details Time Complexity: O(N!), because it calls the backtracking and bounding recursive function
 * @return Returns the minimum distance of the circuit and the corresponding circuit
 */
std::pair<double, std::vector<Vertex<GeoPoint*>*>> TSP::tspBTSetup(bool ignoreBound) {
    std::vector<Vertex<GeoPoint*>*> path;
    std::vector<Vertex<GeoPoint*>*> curPath;

    double minDistance = INFINITY;

    for (Vertex<GeoPoint*>*& geoPointVertex : tspNetwork.getVertexSet()) {
        geoPointVertex->setPath(nullptr);
        Edge<GeoPoint*> addedEdge = *geoPointVertex->addEdge(geoPointVertex, 0.0);
        edgesToRemove.push_back(addedEdge);
        curPath.push_back(vertexGeoMap[0]);
    }

    curPath[0] = vertexGeoMap[0];

    tspRec(vertexGeoMap.size(), 1, 0, curPath, minDistance, path, ignoreBound);

    return {minDistance, path};
}

/**
 * @brief Computes the Minimum Spanning Tree (MST) of a graph using Prim's algorithm.
 *
 * This function computes the MST of the given graph `g` using Prim's algorithm.
 * It returns the vertices of the MST in the order they are visited.
 * It has time complexity of O(E*log(V))
 *
 * @param g A pointer to the graph on which Prim's algorithm is to be executed.
 * @param visitOrder A reference to a vector that will store the order in which vertices are visited.
 * @return A vector of vertices that are part of the MST.
 */
std::vector<Vertex<GeoPoint*>*> TSP::prim(Graph<GeoPoint*> * g) {
    MutablePriorityQueue<Vertex<GeoPoint*>> q;
    std::vector<Vertex<GeoPoint*>*> res;
    for(Vertex<GeoPoint*>* v : g->getVertexSet()){
        v->setDist(std::numeric_limits<double>::infinity());
        v->setPath(nullptr);
        v->setVisited(false);
    }
    Vertex<GeoPoint*>* r = g->getVertexSet().front();
    r->setDist(0);
    q.insert(r);

    while(!q.empty()){
        Vertex<GeoPoint*>* u =  q.extractMin();
        u->setVisited(true);
        res.push_back(u);
        //std::cout << u->getInfo()->getId() << std:: endl;

        for(Edge<GeoPoint*> *e : u->getAdj()){
            Vertex<GeoPoint*>* v = e->getDest();

            if(!v->isVisited()){
                auto oldDist = v->getDist();

                if(e->getWeight() < oldDist){
                    v->setDist(e->getWeight());
                    v->setPath(e);

                    if(oldDist == std::numeric_limits<double>::infinity()){
                        q.insert(v);
                    } else {
                        q.decreaseKey(v);
                    }
                }
            }
        }
    }

    return res;
}

/**
 * @brief Checks if two vertices are adjacent in the graph.
 *
 * This function checks if there is an edge between the vertices `v1` and `v2`.
 * It has time complexity of O(E), being E the edges of vertex v1
 *
 * @param v1 A reference to a pointer to the first vertex.
 * @param v2 A reference to a pointer to the second vertex.
 * @return true if `v1` and `v2` are adjacent, false otherwise.
 */
bool TSP::isAdjacent(Vertex<GeoPoint *> *&v1, Vertex<GeoPoint *> *&v2) {
    for(auto v : v1->getAdj()){
        if(v->getDest()->getInfo()->getId() == v2->getInfo()->getId()){
            return true;
        }
    }
    return false;
}



// T2.2
/**
 * @brief Computes an approximate solution to the TSP using a triangular approximation.
 *
 * This function computes an approximate solution to the Traveling Salesman Problem (TSP) using
 * a minimum spanning tree (MST) and a pre-order traversal of the MST. The solution might not be
 * optimal but provides a feasible route even in non-fully connected graphs.
 *
 * It has time complexity O(V+E*log(V)), bounded by Prim's Algorithm Complexity
 *
 * @param sd A stringstream to store the sequence of visited vertices.
 * @return The total cost of the tour.
 */
double TSP::triangularApproximation(std::stringstream &sd){
    std::vector<Vertex<GeoPoint*>*> visitOrder;
    std::vector<Vertex<GeoPoint*>*> MST = prim(&tspNetwork);

    for (auto a : MST){
        if (a->getPath() != nullptr){
            a->getPath()->setSelected(true);
        }
    }

    for(Vertex<GeoPoint*>* v : MST){
        v->setVisited(false);
    }

    // Step 2: Perform a pre-order walk of the MST to create the visit order
    Vertex<GeoPoint*>* root = MST.front();
    preOrderCluster(root, visitOrder);

    // Step 3: Create the tour H using the visit order
    std::vector<Vertex<GeoPoint*>*> tour;
    std::unordered_set<Vertex<GeoPoint*>*> visited;
    Vertex<GeoPoint*>* auxV;
    visitOrder[0]->setPath(edgeFromCurVertexToNextVertexWithHarversine(visitOrder.back(), visitOrder[0]));
    for(size_t i = 0; i <= visitOrder.size() - 1; ++i){
        auto v = visitOrder.at(i);

        if(i == visitOrder.size() - 1){
            auxV = v;
            tour.push_back(v);
            break;
        }

        if(v->getPath()->getDest()->getInfo()->getId() == v->getInfo()->getId()){
            double newDist = 0.0;
            if(isAdjacent(v,visitOrder[i+1])){
                for(auto e : v->getAdj()){
                    if(e->getDest()->getInfo()->getId() == visitOrder[i+1]->getInfo()->getId()){
                        newDist = e->getWeight();
                        break;
                    }
                }
            }else {
                newDist = 1000 * calculateHaversineDistance({v->getInfo()->getLatitude(),v->getInfo()->getLongitude()},{visitOrder[i+1]->getInfo()->getLatitude(),visitOrder[i+1]->getInfo()->getLongitude()});
            }
            v->setDist(newDist);
            tour.push_back(v);

        }else{
            tour.push_back(v);

        }

    }

    // Add the root vertex again to complete the tour
    tour.push_back(root);


    for(const auto v : tour) {
        sd <<"[" << v->getInfo()->getId() <<"]" << "->";

    }

    // Step 4: Calculate the total distance of the tour
    double totalCost = 0.0;
    for(size_t i = 0; i < tour.size() - 1; i++){

        totalCost += tour[i]->getDist();

    }

    return totalCost;
}



// T2.3
/**
 * @brief Faster version of the Triangular Approximation Heuristic, analyzing only one cluster at a time instead of the whole network
 * @details Time Complexity: O(k * ((V + E) * log(V))), where k is the number of clusters and ((V + E) * log) is the complexity of Prim's algorithm for a cluster
 * @return Total distance
 */
double TSP::otherHeuristic(){
    Vertex<GeoPoint*>* rootVertex = nullptr;
    double res = 0; // TSP approximate solution tour length

    // 1. Create clusters, using K-means Clustering
    int k = 20; // For a big graph
    if (isToyGraph || isExtraGraph){
        k = 1; // No need for clustering
    }

    std::vector<std::set<int>> clusters;
    std::vector<int> centroids(k);
    std::vector<std::vector<Vertex<GeoPoint*>*>> clusterPreorders;
    createClusters(clusters, centroids, k, rootVertex);

    // 2. Compute the TSP for each cluster, using Prim's algorithm
    //    - Compute the MST
    //    - Choose the preorder traversal as the tour
    for (int i = 0; i < k; i++){  // MST for each cluster
        for (Vertex<GeoPoint*>* v : tspNetwork.getVertexSet()){
            v->setVisited(true);
        }
        for (int clusterV : clusters[i]){
            Vertex<GeoPoint*>* v = vertexGeoMap.at(clusterV);
            v->setVisited(false);
        }
        std::vector<Vertex<GeoPoint*> *> clusterMST = clusterPrim(&tspNetwork); // Apply prim ONLY on not visited vertices (vertices that are in the cluster)

        // Select only the cluster
        for(Vertex<GeoPoint*>* v : clusterMST){
            v->setVisited(false);
        }

        // Perform a pre-order traversal of each MST to create the visit order
        int centroidId = centroids[i];

        for (auto a : clusterMST){
            if (a->getPath() != nullptr){
                a->getPath()->setSelected(true);
            }
        }

        Vertex<GeoPoint*>* root = nullptr; // Select a node from the MST
        for (auto a : clusterMST){
            if (a->getPath() == nullptr){
                root = a;
                break;
            }
        }
        std::vector<Vertex<GeoPoint*>*> preOrder;
        preOrder.clear();
        preOrderCluster(root, preOrder);
        clusterPreorders.push_back(preOrder);
    }

    // 3. Connect the clusters
    std::vector<Vertex<GeoPoint*>*> finalTour;
    for (int i = 0; i < clusterPreorders.size(); i++){
        for (auto b : clusterPreorders[i]){
            finalTour.push_back(b);
        }
    }

    if (isToyGraph) { // Only print final tour for small graphs
        std::cout << "\nFinal tour: ";
        for (auto a: finalTour) {
            std::cout << a->getInfo()->getId() << " ";
        }
        std::cout << finalTour[0]->getInfo()->getId(); // To close the tour
    }

    double totalCost = 0.0;
    for (size_t i = 1; i < finalTour.size(); i++){
        if (i == finalTour.size() - 1){
            totalCost += getWeightBetween(finalTour[i], finalTour[0]); // Close the tour
        }
        totalCost += getWeightBetween(finalTour[i-1], finalTour[i]);
    }

    std::cout << "\n=> Total cost: " << totalCost;
    res = totalCost;

    return res;
}

/**
 * @brief Creates clusters of GeoPoints based on k and relative distance (K-means Clustering)
 * @details Time Complexity: O(V * k), where V is the number of vertices in the graph and k is the number of clusters
 * @param clusters vector of clusters to be filled
 * @param k number of clusters to be created
 */
void TSP::createClusters(std::vector<std::set<int>>& clusters, std::vector<int>& centroids, int k, Vertex<GeoPoint*>* rootVertex){
    std::unordered_set<int> chosenIds; // To ensure that the same point isn't chosen twice

    // Initialize centroids randomly (without choosing the same one twice)
    if (!isToyGraph) std::cout << "---------------- Chosen Centroids ----------------\n";
    srand((unsigned)time(0)); // "Randomize" seed
    for (int i = 0; i < k; i++) {
        GeoPoint* randomGP;
        do {
            randomGP = geoMap.at(rand() % geoMap.size());
        } while (chosenIds.count(randomGP->getId()) > 0); // Check if GeoPoint has been chosen before
        centroids[i] = randomGP->getId();
        chosenIds.insert(randomGP->getId());
        if (!isToyGraph) std::cout << "Centroid " << i << ": " << centroids[i] << "\n";
    }

    clusters.resize(k);
    for (int i = 0; i < k; i++) clusters[i].insert(centroids[i]); // Add centroids to clusters

    // Assign GeoPoints to clusters based on proximity to centroids
    for (auto& pair : geoMap) {
        GeoPoint* point = pair.second;

        // Skip if the current GeoPoint is a centroid
        if (std::find(centroids.begin(), centroids.end(), point->getId()) != centroids.end()) continue;

        double minDistance = INFINITY;
        int closestCentroidIdx = -1;

        // Find the closest centroid
        for (int i = 0; i < k; ++i) {
            GeoPoint* centroid = geoMap.at(centroids[i]);
            double distance = 1000 * calculateHaversineDistance(std::make_pair(point->getLatitude(), point->getLongitude()), std::make_pair(centroid->getLatitude(), centroid->getLongitude()));
            if (distance < minDistance) {
                minDistance = distance;
                closestCentroidIdx = i;
            }
            else if (distance == minDistance) { // Select random centroid in case of a tie
                if (rand() % 2 == 0) {
                    closestCentroidIdx = i;
                }
            }
        }

        // Assign the GeoPoint to the closest cluster
        clusters[closestCentroidIdx].insert(point->getId());
    }

    // Associate each vertex to a cluster
    for (int i = 0; i < k; i++){
        std::set<int> cluster = clusters[i];
        for (int id : cluster){
            Vertex<GeoPoint*>* v = vertexGeoMap.at(id);
            v->setIndegree(i);
        }
    }

    if (!isToyGraph) std::cout << "--------------------------------------------------";
}

/**
 * @brief Executes Prim's algorithm on a part of the graph (cluster), depending on which vertices are visited
 * @details Time Complexity: O((V + E) * log(V)), where V is the number of vertices in the cluster and E is the number of edges in the cluster
 * @return A set containing the MST of the cluster
 */
std::vector<Vertex<GeoPoint*> *> TSP::clusterPrim(Graph<GeoPoint*> * g) {
    std::vector<Vertex<GeoPoint*> *> resMST;
    std::unordered_map<int, Vertex<GeoPoint*>*> hashmap;
    resMST.clear();
    MutablePriorityQueue<Vertex<GeoPoint*>> q;
    for(Vertex<GeoPoint*>* v : g->getVertexSet()){
        v->setDist(std::numeric_limits<double>::infinity());
    }

    // Find first vertex of MST
    Vertex<GeoPoint*>* r = nullptr;
    for(Vertex<GeoPoint*>* v : g->getVertexSet()){
        if (!v->isVisited()) {
            r = v; // Choose vertex from cluster
            break;
        }
    }
    if (r == nullptr) return resMST; // Empty cluster

    r->setDist(0);
    r->setPath(nullptr);
    q.insert(r);
    resMST.push_back(r);
    hashmap[r->getInfo()->getId()];
    while(!q.empty()){
        Vertex<GeoPoint*>* u = q.extractMin();
        u->setVisited(true);
        for(Edge<GeoPoint*> *e : u->getAdj()){
            Vertex<GeoPoint*>* v = e->getDest();
            auto oldDist = v->getDist();

            if(!v->isVisited() && e->getWeight() < v->getDist()){
                v->setDist(e->getWeight());
                v->setPath(e);
                try{
                    hashmap.at(v->getInfo()->getId());
                } catch(std::out_of_range& oor) {
                    resMST.push_back(v);
                }
                if(oldDist == std::numeric_limits<double>::infinity()){
                    q.insert(v);
                } else {
                    q.decreaseKey(v);
                }
            }
        }
    }

    return resMST;
}

/**
 * @brief Does a pre-order traversal on an MST
 * @details Time Complexity: O(VE), where V is the number of vertices in the MST and E is the number of edges of each vertex
 * @param root root node of the pre-order traversal
 * @param preorder vector that contains the solution (vertices in preorder)
 */
void TSP::preOrderCluster(Vertex<GeoPoint*>* root, std::vector<Vertex<GeoPoint*>*>& preorder){
    if (root == nullptr) return;

    preorder.push_back(root);

    // Recursively traverse the left subtree
    for (auto e : root->getAdj()){
        if (e->isSelected()) {
            e->setSelected(false);
            e->getDest()->setPath(e);
            preOrderCluster(e->getDest(), preorder);
        }
    }
}

/**
 * @brief Computes the weight of an edge between two vertices
 * @details Time Complexity: O(E), where E is the number of outgoing edges of v1
 * @param v1 Origin vertex
 * @param v2 Destination vertex
 * @return weight between v1 and v2
 */
double TSP::getWeightBetween(Vertex<GeoPoint*>* v1, Vertex<GeoPoint*>* v2){
    for (auto e : v1->getAdj()){
        if (e->getDest()->getInfo()->getId() == v2->getInfo()->getId()){
            return e->getWeight();
        }
    }
    return 1000 * calculateHaversineDistance(std::make_pair(v1->getInfo()->getLatitude(), v1->getInfo()->getLongitude()), std::make_pair(v2->getInfo()->getLatitude(), v2->getInfo()->getLongitude()));
}



// T2.4
/**
 * @brief Performs a nearest neighbor recursion for the Travelling Salesman Problem (TSP).
 *
 * This function implements the nearest neighbor heuristic to find an approximate solution
 * to the Travelling Salesman Problem. It recursively explores paths from the current vertex
 * to its adjacent vertices, selecting the edge with the minimum weight that hasn't been selected yet.
 * It updates the best path and count of the path length found so far.
 * The function is bounded by the global variable recursionTimes, that guarantees that if a recursion
 * level has reached 100 thousand levels deep, the recursion must end, even if there's no answer to that
 * point.
 *
 * The function has average time complexity of O(10^5 * E²), however, in the worst case where E = V²
 * it has time complexity of O(10^5 * V^4)
 *
 * @param here The ID of the current vertex.
 * @param id The ID of the previous vertex, used to avoid going back to the same vertex immediately.
 * @param path A reference to the vector storing the current path of GeoPoints.
 * @param count A reference to the current path length.
 * @param bestPath A reference to the vector storing the best path of GeoPoints found so far.
 * @param bestCount A reference to the length of the best path found so far.
 * @return True if a complete tour is found; otherwise, false.
 */
bool TSP::nnRecursion(int here, int id, std::vector<GeoPoint *> &path, double& count, std::vector<GeoPoint*> &bestPath, double &bestCount)
{
    Vertex<GeoPoint*>* startVertex = this->vertexGeoMap[here];
    startVertex->setVisited(true);

    path.push_back(startVertex->getInfo());

    if(path.size() > bestPath.size()){
        bestPath = path;
        bestCount = count;
    }

    if(this->recursionTimes == 0){
        return true;
    }

    if(path.size() == this->vertexGeoMap.size() + 1){
        return true;
    }

    for(auto e : startVertex->getAdj()){
        if(e->getDest()->isVisited()){
            if(path.size()==this->vertexGeoMap.size() && e->getDest()->getInfo()->getId() == 0) continue;
            e->setSelected(true);
        }
    }


    for(auto e : startVertex->getAdj()){
        double min = INF;
        Edge<GeoPoint*>* minEdge = nullptr;

        for(auto e2: startVertex->getAdj()){
            if(!e2->isSelected()){
                if(e2->getWeight() < min){
                    min = e2->getWeight();
                    minEdge = e2;
                    startVertex->setDist(min);
                }
            }
        }

        if(!minEdge) break;

        count += min;
        this->recursionTimes--;
        if(nnRecursion(minEdge->getDest()->getInfo()->getId(),here,path,count,bestPath,bestCount)) return true;

        this->vertexGeoMap[minEdge->getDest()->getInfo()->getId()]->setVisited(false);
        count -= min;
        minEdge->setSelected(true);
    }

    for(auto e : startVertex->getAdj()){
        e->setSelected(false);
    }

    if(id != -1){
        for(auto e : this->vertexGeoMap[id]->getAdj()){
            if(e->getDest()->getInfo()->getId() == here){
                e->setSelected(true);
            }
        }
    }

    path.pop_back();
    startVertex->setVisited(false);
    return false;
}
