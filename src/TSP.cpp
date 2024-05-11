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
#include <set>

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

        if (!geoPointSource->addEdge(geoPointTarget, std::stod(distance))) {
            std::cerr << "Problem while adding an edge to the graph\n";
            continue;
        }

        if (!geoPointTarget->addEdge(geoPointSource, std::stod(distance))) {
            std::cerr << "Problem while adding an edge to the graph\n";
            continue;
        }

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

bool allNodesVisited(std::unordered_map<int, Vertex<GeoPoint*>*>& geoPoints) {
    for (const auto& geoPoint : geoPoints) {
       if (!geoPoint.second->isVisited()) {
           return false;
       }
    }
    return true;
}

// T2.1

/**
 * @brief Helper function.
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


void TSP::tspRec(unsigned int n, unsigned int curI, double curDist, std::vector<Vertex<GeoPoint*>*>& curPath, double& minDist, std::vector<Vertex<GeoPoint*>*>& path) {

    if (curI == n) {
        curDist += edgeFromCurVertexToNextVertex(curPath.at(n - 1), curPath.at(0))->getWeight();
        if (curDist < minDist) { // Since we reached the last vertex we wll now check if this newly found tsp path has a distance that is smaller than the one before.
            minDist = curDist; // Update the minimum distance found.
            path.clear();
            for (int i = 0; i < n; i++) { // Update the optimal path that leads to the desired outcome. (tsp)
                path.push_back(curPath.at(i));
            }
        }
        return;
    }



    for (int i = 1; i < n; i++) { // Iterate through possible nodes
        auto nextEdge = edgeFromCurVertexToNextVertex(curPath.at(curI), vertexGeoMap[i]);
        double distance = 0;
        if (nextEdge != nullptr) {
            distance = nextEdge->getWeight();
        }

        if (curDist + distance < minDist) { // Bound -> If the distance of the current node to i is bigger than minDist then we should skip.
            bool newNode = true; // Set the current node as a possible new node to visit.
            for (int j = 1; j < curI; j++) { // The number of nodes that we need to check for possibly being in the tsp path already are the nodes from the starting node until the current node
                if (curPath.at(j) == nullptr) {
                    continue;
                }
                if (curPath.at(j) == vertexGeoMap[i]) { // Check's if the next node is already in the tsp Path. // Fixme: curPath[j]
                    newNode = false;
                    break;
                }
            }

            if (newNode) { // If we haven't checked this new node yet then let's do it now.
                curPath.at(curI) = vertexGeoMap[i];
                auto dist = edgeFromCurVertexToNextVertex(curPath.at(curI - 1), curPath.at(curI));
                if (dist != nullptr) {
                    curDist += dist->getWeight();
                }
                tspRec(n, curI + 1 , curDist , curPath ,minDist, path);
                if (dist != nullptr) {
                    curDist -= dist->getWeight();
                }
            }
        }
    }


}


double TSP::tspBTSetup() {
    std::vector<Vertex<GeoPoint*>*> path;
    std::vector<Vertex<GeoPoint*>*> curPath;

    double minDistance = INFINITY;

    for (Vertex<GeoPoint*>*& geoPointVertex : tspNetwork.getVertexSet()) {
        geoPointVertex->setVisited(false);
        geoPointVertex->setPath(nullptr);
        geoPointVertex->addEdge(geoPointVertex, 0.0);
        curPath.push_back(nullptr);
    }

    curPath[0] = vertexGeoMap[0];

    tspRec(vertexGeoMap.size(), 1, 0, curPath, minDistance, path);

    std::cout << "\n";
    for (Vertex<GeoPoint*>* vertex : path) {
        std::cout <<  vertex->getInfo()->getId() << " -> ";
    }
    std::cout << "\n";


    return minDistance;
}


template <class T>
std::vector<Vertex<T> *> TSP::prim(Graph<T> * g) {
    MutablePriorityQueue<Vertex<T>> q;
    for(Vertex<T>* v : g->getVertexSet()){
        v->setDist(std::numeric_limits<double>::infinity());
    }
    Vertex<T>* r = g->getVertexSet().front();
    r->setDist(0);
    q.insert(r);

    while(!q.empty()){
        Vertex<T>* u =  q.extractMin();
        u->setVisited(true);
        for(Edge<T> *e : u->getAdj()){
            Vertex<T>* v = e->getDest();
            if(!v->isVisited() && e->getWeight() < v->getDist()){
                v->setDist(e->getWeight());
                v->setPath(e);
                q.insert(v);
                q.decreaseKey(v);
            }
        }
    }

    return g->getVertexSet();
}

template <class T>
double TSP::spanningTreeCost(const std::vector<Vertex<T> *> &res){
    double ret = 0;
    for(const Vertex<T> *v: res){
        if(v->getPath() == nullptr) continue;
        const Vertex<T> *u = v->getPath()->getOrig();
        for(const auto e: u->getAdj()){
            if(e->getDest()->getInfo() == v->getInfo()){
                ret += e->getWeight();
                break;
            }
        }
    }
    return ret;
}

template <class T>
void TSP::preOrderWalk(Vertex<T>* root, std::vector<Vertex<T>*> &visitOrder){
    if(root == nullptr) return;
    visitOrder.push_back(root);
    root->setVisited(true);
    for(Edge<T>* edge : root->getAdj()){
        if(!edge->getDest()->isVisited()){
            preOrderWalk(edge->getDest(), visitOrder);
        }
    }


}

bool TSP::isAdjacent(Vertex<GeoPoint *> *&v1, Vertex<GeoPoint *> *&v2) {
/*    // Get the source vertex
    auto s = v1;
    if (s == nullptr) {
        return false;
    }

    // Perform the actual BFS using a queue
    std::queue<Vertex<GeoPoint*> *> q;
    q.push(s);
    s->setVisited(true);
    while (!q.empty()) {
        auto v = q.front();
        q.pop();
        if(v->getInfo()->getId() == v2->getInfo()->getId()) return true;
            for (auto & e : v->getAdj()) {
                auto w = e->getDest();
                if ( ! w->isVisited()) {
                    q.push(w);
                    w->setVisited(true);
                }
            }
        }
        return false;
        */
    for(auto v : v1->getAdj()){
        if(v->getDest()->getInfo()->getId() == v2->getInfo()->getId()) return true;
    }
    return false;

    //FIND ANOTHER WAY TO DO THIS
}

// T2.2
double TSP::triangularApproximation(){
    for(Vertex<GeoPoint*>* v : tspNetwork.getVertexSet()){
        //std::cout << v->getInfo()->getLabel() << std::endl;
        v->setVisited(false);
    }

    std::vector<Vertex<GeoPoint*>*> MST = prim(&tspNetwork);
/*
    std::stringstream ss;
    for(const auto v : MST) {
        ss << v->getInfo()->getId() << "->";
        if ( v->getPath() != nullptr ) {
            ss << v->getPath()->getOrig()->getInfo()->getId();
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
    std::vector<Vertex<GeoPoint*>*> visitOrder;
    preOrderWalk(root, visitOrder);
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
    for(Vertex<GeoPoint*>* v : visitOrder){
        if(visited.find(v) == visited.end()){
            v->setVisited(false);
            tour.push_back(v);
            auxV = v;
            visited.insert(v);
        }
    }

    // Add the root vertex again to complete the tour
    tour.push_back(root);
/*
    std::stringstream sd;
    for(const auto v : tour) {
        sd << v->getInfo()->getId() << "<-";
        if ( v->getPath() != nullptr ) {
            sd << v->getPath()->getDest()->getInfo()->getId();
        }
        sd << "|";
    }
    std::cout << "TOUR >> " <<sd.str() << std::endl;
*/
    // Step 4: Calculate the total distance of the tour
    double totalCost = 0.0;
    for(size_t i = 0; i < tour.size() - 1; ++i){

        if (!isAdjacent(tour[i],tour[i+1])) {
            // Calculate distance using Haversine function if nodes are not directly connected
            double distance = calculateHaversineDistance({tour[i]->getInfo()->getLatitude(),tour[i]->getInfo()->getLongitude()},{tour[i+1]->getInfo()->getLatitude(),tour[i+1]->getInfo()->getLongitude()});
            totalCost += distance;
        }

    else{
        totalCost += tour[i]->getDist();
        }
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
double TSP::otherHeuristic(){
    double res = 0; // TSP approximate solution tour length

    // 1. Create clusters, using K-means Clustering
    int k = 9; // FIXME: Change value of k
    std::vector<std::set<int>> clusters;
    createClusters(clusters, k);

    // 2. Compute the TSP for each cluster, using Prim's algorithm
    //    - Compute the MST
    //    - Choose the preorder traversal as the tour
    for (int i = 0; i < k; i++){  // MST for each cluster
        for (Vertex<GeoPoint*>* v : tspNetwork.getVertexSet()){
            v->setVisited(true);
        }
        for (int clusterV : clusters[i]){
            GeoPoint* gp = geoMap.at(clusterV);
            for (auto v : tspNetwork.getVertexSet()){ // FIXME: findVertex() not working for some reason
                if (v->getInfo()->getId() == gp->getId()) {
                    v->setVisited(false);
                    break;
                }
            }
        }
        std::set<Vertex<GeoPoint*> *> clusterMST = clusterPrim(&tspNetwork); // Apply prim ONLY on not visited vertices (vertices that are in the cluster)
        /*std::cout << "\nCluster " << i << " MST: ";
        for (auto mst : clusterMST){
            std::cout << mst->getInfo()->getId() << ", ";
        }*/

        //preorderTraversal(); // Choose tour inside each cluster, using preoder traversal
    }

    // 3. Connect the clusters:
    //    - Compute the MST of the cluster centroids
    primClusterCentroids();

    // 4. Locally optimize the initial TSP solution:
    //    - Apply 2-opt optimizations inside each cluster
    //    - Repeat until no further improvements can be made
    // TODO

    return res;
}

/**
 * @brief Creates clusters of GeoPoints based on k and relative distance (K-means Clustering)
 * @param clusters vector of clusters to be filled
 * @param k number of clusters to be created
 */
void TSP::createClusters(std::vector<std::set<int>>& clusters, int k){
    std::vector<int> centroids(k);
    std::unordered_set<int> chosenIds; // To ensure that the same point isn't chosen twice

    // Initialize centroids randomly (without choosing the same one twice)
    srand((unsigned)time(0)); // "Randomize" seed
    for (int i = 0; i < k; i++) {
        GeoPoint* randomGP;
        do {
            randomGP = geoMap.at(rand() % geoMap.size());
        } while (chosenIds.count(randomGP->getId()) > 0); // Check if GeoPoint has been chosen before
        centroids[i] = randomGP->getId();
        chosenIds.insert(randomGP->getId());
        std::cout << "centroid: " << centroids[i] << "\n"; // TODO: Remove (DEBUG)
    }

    clusters.resize(k);

    for (int i = 0; i < k; i++){
        clusters[i].insert(centroids[i]); // Add centroids to clusters
    }

    // Assign GeoPoints to clusters based on proximity to centroids
    for (auto& pair : geoMap) {
        GeoPoint* point = pair.second;
        double minDistance = INFINITY;
        int closestCentroidIdx = -1;

        // Find the closest centroid
        for (int i = 0; i < k; ++i) {
            GeoPoint* centroid = geoMap.at(centroids[i]);
            double distance = calculateHaversineDistance(std::make_pair(point->getLatitude(), point->getLongitude()), std::make_pair(centroid->getLatitude(), centroid->getLongitude())); // TODO: Implement computeDistance
            if (distance < minDistance) {
                minDistance = distance;
                closestCentroidIdx = i;
            }
        }

        // FIXME: DUPLICATE VERTICES IN CLUSTERS (-> Problem is size of k)
        // TODO: PRINT SEED FOR DEBUGGING

        // Assign the GeoPoint to the closest cluster
        clusters[closestCentroidIdx].insert(point->getId());
    }

    // Associate each vertex to a cluster
    for (int i = 0; i < k; i++){
        std::set<int> cluster = clusters[i];
        for (int id : cluster){
            GeoPoint* gp = geoMap.at(id);

            //Vertex<GeoPoint*>* v = tspNetwork.findVertex(gp); // FIXME: findVertex not working for some reason (used for loop instead)
            for (auto v : tspNetwork.getVertexSet()){
                if (v->getInfo()->getId() == gp->getId()) {
                    v->setIndegree(i);
                    break;
                }
            }
        }
    }

    // TODO: Remove (DEBUG)
    for (int i = 0; i < k; i++){
        std::cout << "\nCluster " << i << ": ";
        std::set<int> cluster = clusters[i];
        for (int id : cluster){
            std::cout << id << ", ";
        }
    }
}

std::set<Vertex<GeoPoint*> *> TSP::clusterPrim(Graph<GeoPoint*> * g) {
    std::set<Vertex<GeoPoint*> *> resMST;
    resMST.clear();
    MutablePriorityQueue<Vertex<GeoPoint*>> q;
    for(Vertex<GeoPoint*>* v : g->getVertexSet()){
        v->setDist(std::numeric_limits<double>::infinity());
    }
    std::cout << "\nprim\n";

    // Find first vertex of MST
    Vertex<GeoPoint*>* r = nullptr;
    for(Vertex<GeoPoint*>* v : g->getVertexSet()){
        if (!v->isVisited()) {
            //std::cout << "first: " << v->getInfo()->getId() << "\n";
            r = v; // Choose vertex from cluster
            break;
        }
    }
    if (r == nullptr) return resMST; // Empty cluster

    r->setDist(0);
    q.insert(r);
    while(!q.empty()){
        Vertex<GeoPoint*>* u = q.extractMin();
        std::cout << u->getInfo()->getId() << ", ";
        u->setVisited(true);
        resMST.insert(u);
        for(Edge<GeoPoint*> *e : u->getAdj()){
            Vertex<GeoPoint*>* v = e->getDest();
            if(!v->isVisited() && e->getWeight() < v->getDist()){
                v->setDist(e->getWeight());
                v->setPath(e);
                q.insert(v);
                q.decreaseKey(v);
            }
        }
    }
    return resMST;
}

/**
 * @brief Does a preorder traversal on an MST
 */
void TSP::preorderTraversal(){
    // TODO
}

/**
 * @brief Finds a minimal spanning tree using the cluster centroids, in order to find the final tour in the full graph
 */
void TSP::primClusterCentroids(){
    // TODO
}


// T2.4
// TODO

