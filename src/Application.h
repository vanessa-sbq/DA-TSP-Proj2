#ifndef APLICATION_H
#define APLICATION_H

#include <fstream>
#include <sstream>
#include <thread>
#include <iostream>
#include <utility>
#include <vector>
#include <cstdlib>
#include <chrono>
#include <regex>
#include <iomanip>

#include "TSP.h"
#include "Graph.h"
#include "GeoPoint.h"

#define DATASET_PATHS "../dataset/"

#define EXTRA_FULLY_CONNECTED_GRAPHS_25_EDGES "Extra_Fully_Connected_Graphs/edges_25.csv"
#define EXTRA_FULLY_CONNECTED_GRAPHS_50_EDGES "Extra_Fully_Connected_Graphs/edges_50.csv"
#define EXTRA_FULLY_CONNECTED_GRAPHS_75_EDGES "Extra_Fully_Connected_Graphs/edges_75.csv"
#define EXTRA_FULLY_CONNECTED_GRAPHS_100_EDGES "Extra_Fully_Connected_Graphs/edges_100.csv"
#define EXTRA_FULLY_CONNECTED_GRAPHS_200_EDGES "Extra_Fully_Connected_Graphs/edges_200.csv"
#define EXTRA_FULLY_CONNECTED_GRAPHS_300_EDGES "Extra_Fully_Connected_Graphs/edges_300.csv"
#define EXTRA_FULLY_CONNECTED_GRAPHS_400_EDGES "Extra_Fully_Connected_Graphs/edges_400.csv"
#define EXTRA_FULLY_CONNECTED_GRAPHS_500_EDGES "Extra_Fully_Connected_Graphs/edges_500.csv"
#define EXTRA_FULLY_CONNECTED_GRAPHS_600_EDGES "Extra_Fully_Connected_Graphs/edges_600.csv"
#define EXTRA_FULLY_CONNECTED_GRAPHS_700_EDGES "Extra_Fully_Connected_Graphs/edges_700.csv"
#define EXTRA_FULLY_CONNECTED_GRAPHS_800_EDGES "Extra_Fully_Connected_Graphs/edges_800.csv"
#define EXTRA_FULLY_CONNECTED_GRAPHS_900_EDGES "Extra_Fully_Connected_Graphs/edges_900.csv"

#define EXTRA_FULLY_CONNECTED_GRAPHS_NODES "Extra_Fully_Connected_Graphs/nodes.csv"

#define TOY_GRAPH_SHIPPING "Toy-Graphs/shipping.csv"
#define TOY_GRAPH_STADIUMS "Toy-Graphs/stadiums.csv"
#define TOY_GRAPH_TOURISM "Toy-Graphs/tourism.csv"

#define REAL_WORLD_GRAPH_1_EDGES "Real-world Graphs/graph1/edges.csv"
#define REAL_WORLD_GRAPH_1_NODES "Real-world Graphs/graph1/nodes.csv"

#define REAL_WORLD_GRAPH_2_EDGES "Real-world Graphs/graph2/edges.csv"
#define REAL_WORLD_GRAPH_2_NODES "Real-world Graphs/graph2/nodes.csv"

#define REAL_WORLD_GRAPH_3_EDGES "Real-world Graphs/graph3/edges.csv"
#define REAL_WORLD_GRAPH_3_NODES "Real-world Graphs/graph3/nodes.csv"

/**@brief Class that manages the menu.*/
class Application {
public:
    Application(std::string env) {
        this->env = std::move(env);
        restartConstructor:
        clearScreen();

        std::string choiceString;
        int choice;
        std::cout << "Select a graph:\n";
        std::cout << "1. Extra Fully Connected Graphs\n";
        std::cout << "2. Toy Graphs\n";
        std::cout << "3. Real World Graphs\n";
        std::cout << "Enter your choice: ";
        std::cin >> choiceString;

        try {
             choice = std::stoi(choiceString);
        } catch (std::invalid_argument& argument) {
            std::cout << "\n* Error while parsing option, please input a valid numeric option. *\n";
            delay(2000);
            goto restartConstructor;
        }

        std::string stringGraphChoice;
        int graphChoice;

        switch(choice) {
            case 1:
                clearScreen();
                this->nodePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_NODES;
                this->edgeFileSeparatedFromNodeFile = true;
                std::cout << "Extra Fully Connected Graphs:\n";
                std::cout << "1. 25 Nodes\n";
                std::cout << "2. 50 Nodes\n";
                std::cout << "3. 75 Nodes\n";
                std::cout << "4. 100 Nodes\n";
                std::cout << "5. 200 Nodes\n";
                std::cout << "6. 300 Nodes\n";
                std::cout << "7. 400 Nodes\n";
                std::cout << "8. 500 Nodes\n";
                std::cout << "9. 600 Nodes\n";
                std::cout << "10. 700 Nodes\n";
                std::cout << "11. 800 Nodes\n";
                std::cout << "12. 900 Nodes\n\nInput: ";
                std::cin >> stringGraphChoice;

                try {
                    graphChoice = std::stoi(stringGraphChoice);
                } catch (std::invalid_argument& argument) {
                    std::cout << "\n* Error while parsing option, please input a valid numeric option. *\n";
                    delay(2000);
                    goto restartConstructor;
                }

                switch (graphChoice) {
                    case 1:
                        this->edgePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_25_EDGES;
                        this->graphChoosen = 1;
                        break;
                    case 2:
                        this->edgePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_50_EDGES;
                        this->graphChoosen = 2;
                        break;
                    case 3:
                        this->edgePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_75_EDGES;
                        this->graphChoosen = 3;
                        break;
                    case 4:
                        this->edgePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_100_EDGES;
                        this->graphChoosen = 4;
                        break;
                    case 5:
                        this->edgePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_200_EDGES;
                        this->graphChoosen = 5;
                        break;
                    case 6:
                        this->edgePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_300_EDGES;
                        this->graphChoosen = 6;
                        break;
                    case 7:
                        this->edgePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_400_EDGES;
                        this->graphChoosen = 7;
                        break;
                    case 8:
                        this->edgePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_500_EDGES;
                        this->graphChoosen = 8;
                        break;
                    case 9:
                        this->edgePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_600_EDGES;
                        this->graphChoosen = 9;
                        break;
                    case 10:
                        this->edgePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_700_EDGES;
                        this->graphChoosen = 10;
                        break;
                    case 11:
                        this->edgePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_800_EDGES;
                        this->graphChoosen = 11;
                        break;
                    case 12:
                        this->edgePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_900_EDGES;
                        this->graphChoosen = 12;
                        break;
                    default:
                        delay(3000);
                        std::cout << "Unrecognized option, defaulting to graph with 25 edges.\n";
                        this->edgePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_25_EDGES;
                        this->graphChoosen = 1;
                }

                break;
            case 2:
                isToyGraph = true;
                clearScreen();
                std::cout << "Toy Graphs:\n";
                std::cout << "1. Shipping\n";
                std::cout << "2. Stadiums\n";
                std::cout << "3. Tourism\n\nInput: ";
                std::cin >> stringGraphChoice;

                try {
                    graphChoice = std::stoi(stringGraphChoice);
                } catch (std::invalid_argument& argument) {
                    std::cout << "\n* Error while parsing option, please input a valid numeric option. *\n";
                    delay(2000);
                    goto restartConstructor;
                }

                switch (graphChoice) {
                    case 1:
                        this->nodePath = DATASET_PATHS TOY_GRAPH_SHIPPING;
                        this->graphChoosen = 13;
                        break;
                    case 2:
                        this->nodePath = DATASET_PATHS TOY_GRAPH_STADIUMS;
                        this->graphChoosen = 14;
                        break;
                    case 3:
                        this->nodePath = DATASET_PATHS TOY_GRAPH_TOURISM;
                        this->graphChoosen = 15;
                        break;
                    default:
                        delay(3000);
                        std::cout << "Unrecognized option, defaulting to shipping graph.\n";
                        this->nodePath = DATASET_PATHS TOY_GRAPH_SHIPPING;
                        this->graphChoosen = 13;
                }
                break;
            case 3:
                clearScreen();
                this->edgeFileSeparatedFromNodeFile = true;
                this->isBigGraph = true;
                std::cout << "Real World Graphs:\n";
                std::cout << "1. Graph 1\n";
                std::cout << "2. Graph 2\n";
                std::cout << "3. Graph 3\n\nInput: ";
                std::cin >> stringGraphChoice;

                try {
                    graphChoice = std::stoi(stringGraphChoice);
                } catch (std::invalid_argument& argument) {
                    std::cout << "\n* Error while parsing option, please input a valid numeric option. *\n";
                    delay(2000);
                    goto restartConstructor;
                }

                switch (graphChoice) {
                    case 1:
                        this->edgePath = DATASET_PATHS REAL_WORLD_GRAPH_1_EDGES;
                        this->nodePath = DATASET_PATHS REAL_WORLD_GRAPH_1_NODES;
                        this->graphChoosen = 16;
                        break;
                    case 2:
                        this->edgePath = DATASET_PATHS REAL_WORLD_GRAPH_2_EDGES;
                        this->nodePath = DATASET_PATHS REAL_WORLD_GRAPH_2_NODES;
                        this->graphChoosen = 17;
                        break;
                    case 3:
                        this->edgePath = DATASET_PATHS REAL_WORLD_GRAPH_3_EDGES;
                        this->nodePath = DATASET_PATHS REAL_WORLD_GRAPH_3_NODES;
                        this->graphChoosen = 18;
                        break;
                    default:
                        delay(3000);
                        std::cout << "Unrecognized option, defaulting to first real world graph.\n";
                        this->edgePath = DATASET_PATHS REAL_WORLD_GRAPH_1_EDGES;
                        this->nodePath = DATASET_PATHS REAL_WORLD_GRAPH_1_NODES;
                        this->graphChoosen = 16;
                }
                break;
            default:
                delay(3000);
                std::cout << "Unrecognized option, defaulting to extra fully connected graph with 25 edges.\n";
                this->edgeFileSeparatedFromNodeFile = true;
                this->edgePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_25_EDGES;
                this->nodePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_NODES;
                this->graphChoosen = 1;
        }

        // DEBUG
        // TODO remove clearScreen();
        // TODO remove std::cout << "The current path for nodes is " << nodePath << " and edge's is " << edgePath << " cur env " << env << "\n";

        tsp.parseData(nodePath, edgePath, edgeFileSeparatedFromNodeFile);
        clearScreen();
    }

    void clearScreen();

    void run(int processedKey);
    void recordRuntime(const std::string& functionName, int duration);

    static void delay(long sleepTime);

    static int processKey(const std::string &option);

private:

    TSP tsp;
    bool isBigGraph = false;
    int graphChoosen;

    /* Private functions */
    std::string showMainMenu();
    void showGoBackMenu(int option, const std::string& functionName);
    void showRuntime();
    /* TSP functions */

    void backtrackingAlgorithmTSP(); // T2.1
    void triangularApproximationTSP(); // T2.2
    void optimizedTSP(); // T2.3
    void realWorldTSP(); // T2.4

    /* Attributes */
    std::string env; // System environment variable
    std::string edgePath = ""; // Path for edges.csv
    std::string nodePath = ""; // Path for nodes.csv

    bool isToyGraph = false; // Used for adding missing edges
    bool edgeFileSeparatedFromNodeFile = false;
};

#endif //APLICATION_H