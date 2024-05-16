#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <unordered_map>
//#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include "TSP.h"

/* Data parsing begin */

/**
 * @brief Helper function for data parsing
 * @details Helps check if a given csv contains headers.
 *
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


/**
 * @brief Function that helps parse data
 * @param bothFilesProvided If false the first argument will contain the file that incorporates both nodes / edges.
 * If true then first argument will contain the file path for the nodes csv and the second argument will contain the
 * file path for the edges csv.
 **/
void TSP::parseData(std::string nodesFilePath, std::string edgesFilePath, bool bothFilesProvided) {
    std::ifstream in;

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
 * @brief Function that helps parsing the nodes|edges that are inside the csv.
 * @details This functions expects the following order:
 *
 *              "origin, destination, distance" or "origin, destination, distance, label origin, label destination"
 * */
void TSP::parsingGeoPointsAndEdges(std::ifstream &in) {
    auto stringToLower = [&](std::string& label){for (char& c : label) c = (char)tolower(c);};
    std::string line;
    getline(in, line); // Headers

    bool expectLabelForGeoPoint = false;
    int numberOfHeadings = countHeaders(line);
    // TODO REMOVE DEBUG std::cout << "NUMBER OF HEADINGS: " << numberOfHeadings << std::endl;
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
 * @details This functions expects the following order: id, longitude, latitude.
 * */
void TSP::parsingGeoPoints(std::ifstream &in) {
    std::string line;
    getline(in, line); // Header
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
    }
}

void TSP::parseEdgesFromMemory(char* data, size_t size) {
    /*std::istringstream in(std::string(data, size));
    std::string line;

    // Skip the header
    std::getline(in, line);

    // Process the rest of the data
    while (std::getline(in, line)) {
        std::cout << "line: " << line << std::endl;
        std::istringstream s(line);
        std::string origin, destination, distance;

        if (!(std::getline(s, origin, ',') && std::getline(s, destination, ',') && std::getline(s, distance, '\r'))) {
            std::cerr << "Error in parsingEdges, invalid line in the CSV file: " << line << std::endl;
            continue;
        }

        Vertex<GeoPoint*>* geoPointSource = vertexGeoMap.at(std::stoi(origin));
        Vertex<GeoPoint*>* geoPointDestination = vertexGeoMap.at(std::stoi(destination));

        if (geoPointSource == nullptr || geoPointDestination == nullptr) {
            std::cerr << "Error in parsingEdges, problem with finding vertex in maps.\n";
            continue;
        }

        if (!geoPointSource->addEdge(geoPointDestination, std::stod(distance))) {
            std::cerr << "Error in parsingEdges, problem while adding an edge to the graph\n";
            continue;
        }
    }*/
}

void TSP::loadFileUsingMMap(const std::string& filename) {
    /*int fd = open(filename.c_str(), O_RDONLY);
    if (fd == -1) {
        std::cerr << "Error opening file." << std::endl;
        return;
    }

    struct stat st;
    if (fstat(fd, &st) == -1) {
        std::cerr << "Error getting file size." << std::endl;
        close(fd);
        return;
    }

    char* data = static_cast<char*>(mmap(NULL, st.st_size, PROT_READ, MAP_PRIVATE, fd, 0));
    if (data == MAP_FAILED) {
        std::cerr << "Error mapping file to memory." << std::endl;
        close(fd);
        return;
    }

    // Parse the data from memory
    parseEdgesFromMemory(data, st.st_size);

    munmap(data, st.st_size);
    close(fd);*/
}

/**
 * @brief Function that helps parsing the edges that are inside the csv.
 * @details This functions expects the following order: origin, destination, haversine_distance.
 * */
void TSP::parsingEdges(std::ifstream &in) {

    // FALL BACK

    // DO NOT REMOVE
    //loadFileUsingMMap(in);

    std::string line;
    getline(in, line); // Header
    int numberOfHeadings = countHeaders(line);

    std::cout << "NUMBER OF HEADINGS: " << numberOfHeadings << std::endl;


    Vertex<GeoPoint*>* geoPointSource = nullptr;
    Vertex<GeoPoint*>* geoPointDestination = nullptr;

    if (numberOfHeadings == -1) { // The header itself is already data...
        std::istringstream s(line);
        std::string origin, destination, distance;
        if (!(std::getline(s, origin, ',') && std::getline(s, destination, ',') && std::getline(s, distance, '\r'))) {
            std::cerr << "Error in parsingEdges, invalid line in the CSV file" << line << std::endl;
        }

        Vertex<GeoPoint*>* geoPointSource = vertexGeoMap.at(std::stoi(origin));
        Vertex<GeoPoint*>* geoPointDestination = vertexGeoMap.at(std::stoi(destination));

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


// T2.1
// TODO

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

std::vector<Vertex<GeoPoint*>*> TSP::prim(Graph<GeoPoint*> * g,std::vector<Vertex<GeoPoint*>*> &visitOrder) {
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
        visitOrder.push_back(u);

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
 * It has time complexity O(E*log(V)), bounded by Prim's Algorithm Complexity
 *
 * @param sd A stringstream to store the sequence of visited vertices.
 * @return The total cost of the tour.
 */
double TSP::triangularApproximation(std::stringstream &sd){
    std::vector<Vertex<GeoPoint*>*> visitOrder;
    std::vector<Vertex<GeoPoint*>*> MST = prim(&tspNetwork, visitOrder);
/*
    std::stringstream ss;
    for(const auto v : MST) {
        ss << v->getInfo()->getId() << "->";
        if ( v->getPath() != nullptr ) {
            ss << v->getPath()->getDest()->getInfo()->getId();
        }


        ss << "|";
    }
    std::cout << "MST  " <<ss.str() << std::endl;
*/
    for(Vertex<GeoPoint*>* v : MST){
        v->setVisited(false);
    }

    // Step 2: Perform a pre-order walk of the MST to create the visit order
    Vertex<GeoPoint*>* root = MST.front();
    //std::vector<Vertex<GeoPoint*>*> visitOrder;
    //preOrderWalk(root, visitOrder);
/*
    std::stringstream sr;

    for(const auto v : visitOrder) {
        sr << v->getInfo()->getId() << "->";
    }

    std::cout << "Visit Order"<< sr.str() << std::endl;
    */
    // Step 3: Create the tour H using the visit order
    std::vector<Vertex<GeoPoint*>*> tour;
    std::unordered_set<Vertex<GeoPoint*>*> visited;
    Vertex<GeoPoint*>* auxV;
    tour.push_back(visitOrder[0]);
    for(size_t i = 1; i <= visitOrder.size() - 1; ++i){
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
    for(size_t i = 0; i < tour.size() - 1; ++i){
        /*
        if (!isAdjacent(tour[i],tour[i+1])) {
            // Calculate distance using Haversine function if nodes are not directly connected
            double distance = calculateHaversineDistance({tour[i]->getInfo()->getLatitude(),tour[i]->getInfo()->getLongitude()},{tour[i+1]->getInfo()->getLatitude(),tour[i+1]->getInfo()->getLongitude()});
            totalCost += distance;
        }
*/
   //else{
        totalCost += tour[i]->getDist();
        //}
    }

    for(auto v : root->getAdj()){
        if(v->getDest()->getInfo()->getId() == auxV->getInfo()->getId()) {
            totalCost += v->getWeight();
            break;
        }
    }
    return totalCost;
}



// T2.3
// TODO

// T2.4
/**
 * @brief Computes a tour using the nearest neighbor heuristic with backtracking, ensuring all vertices are visited.
 *
 * This function attempts to find a tour starting from the given vertex using the nearest neighbor
 * heuristic. If no direct unvisited neighbor is found, it backtracks and tries alternative paths.
 *
 * @param start The starting vertex ID.
 * @return The total cost of the tour (currently a placeholder value).
 */
double TSP::nearestNeighbour(int start) {
    Vertex<GeoPoint*>* startVertex = this->vertexGeoMap[start];
    int n = this->tspNetwork.getNumVertex();

    for(auto v : this->tspNetwork.getVertexSet()){
        v->setVisited(false);
        v->setProcesssing(false);
        for(auto e : v->getAdj()){
            e->setSelected(false);
        }
    }

    std::vector<Vertex<GeoPoint*>*> tour;
    tour.push_back(startVertex);
    startVertex->setVisited(true);

    std::stack<Vertex<GeoPoint*>*> backtrackStack;
    backtrackStack.push(startVertex);

    while (tour.size() < n) {
        auto currentVertex = backtrackStack.top();
        double minDist = std::numeric_limits<double>::infinity();
        Vertex<GeoPoint*>* nextVertex = nullptr;
        Edge<GeoPoint*>* auxedge;

        // Find the nearest unvisited neighbor
        std::cout << "Vertex " << currentVertex->getInfo()->getId() << " to ";
        for (auto edge : currentVertex->getAdj()) {
            if (!edge->getDest()->isVisited() && !edge->isSelected()) {
                std::cout << edge->getDest()->getInfo()->getId() << ",";
                double dist = edge->getWeight();
                if (dist < minDist) {
                    minDist = dist;
                    nextVertex = edge->getDest();
                    auxedge = edge;
                }
            }
        }
        std::cout << std::endl;

        if (nextVertex) {
            tour.push_back(nextVertex);
            nextVertex->setVisited(true);
            auxedge->setSelected(false);
            backtrackStack.push(nextVertex);
        } else {
            // No unvisited neighbors found, backtrack to the previous vertex
            backtrackStack.pop();
            tour.pop_back();
            currentVertex->setVisited(true);
            if (backtrackStack.empty()) {
                // No path found, all backtracking options exhausted
                std::cerr << "Error: No complete tour found. Some vertices may be unreachable." << std::endl;
                return -1.0; // Return an error value or handle the incomplete tour case
            }
        }
    }

    // Complete the tour by returning to the start vertex
    tour.push_back(startVertex);

    std::cout << tour.size() << std::endl;

    // Output the tour for debugging purposes
    for (auto v : tour) {
        std::cout << v->getInfo()->getId() << "->";
    }
    std::cout << "Start" << std::endl;

    // Calculate and return the total distance of the tour
    double totalCost = 0.0;
    for (size_t i = 0; i < tour.size() - 1; ++i) {
        for (auto edge : tour[i]->getAdj()) {
            if (edge->getDest() == tour[i + 1]) {
                totalCost += edge->getWeight();
                break;
            }
        }
    }

    return totalCost;
}


// TODO

