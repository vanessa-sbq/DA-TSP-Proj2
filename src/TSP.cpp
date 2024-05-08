#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <sys/mman.h>
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

        this->vertexGeoMap[stoi(origin)] = tspNetwork.addVertex(geoPointSource);
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

    }
}

/* Data parsing end */


// T2.1
// TODO

// T2.2
// TODO

// T2.3
// TODO

// T2.4
// TODO

