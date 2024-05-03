#ifndef APLICATION_H
#define APLICATION_H

#include <fstream>
#include <sstream>
#include <thread>
#include <iostream>
#include <utility>
#include <vector>
#include <cstdlib>

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

#define REAL_WORLD_GRAPH_1_EDGES "Real-Word/graph1/edges.csv"
#define REAL_WORLD_GRAPH_1_NODES "Real-Word/graph1/nodes.csv"

#define REAL_WORLD_GRAPH_2_EDGES "Real-Word/graph2/edges.csv"
#define REAL_WORLD_GRAPH_2_NODES "Real-Word/graph2/nodes.csv"

#define REAL_WORLD_GRAPH_3_EDGES "Real-Word/graph3/edges.csv"
#define REAL_WORLD_GRAPH_3_NODES "Real-Word/graph3/nodes.csv"

/**@brief Class that manages the menu.*/
class Application {
public:
    Application(std::string env) {
        this->env = std::move(env);
        clearScreen();

        int choice;

        std::cout << "Select a graph:\n";
        std::cout << "1. Extra Fully Connected Graphs\n";
        std::cout << "2. Toy Graphs\n";
        std::cout << "3. Real World Graphs\n";
        std::cout << "Enter your choice: ";
        std::cin >> choice;

        int graphChoice;

        switch(choice) {
            case 1:
                clearScreen();
                this->nodePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_NODES;
                this->edgeFileSeparatedFromNodeFile = true;
                std::cout << "Extra Fully Connected Graphs:\n";
                std::cout << "1. 25 Edges\n";
                std::cout << "2. 50 Edges\n";
                std::cout << "3. 75 Edges\n";
                std::cout << "4. 100 Edges\n";
                std::cout << "5. 200 Edges\n";
                std::cout << "6. 300 Edges\n";
                std::cout << "7. 400 Edges\n";
                std::cout << "8. 500 Edges\n";
                std::cout << "9. 600 Edges\n";
                std::cout << "10. 700 Edges\n";
                std::cout << "11. 800 Edges\n";
                std::cout << "12. 900 Edges\n\nInput: ";
                std::cin >> graphChoice;

                switch (graphChoice) {
                    case 1:
                        this->edgePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_25_EDGES;
                        break;
                    case 2:
                        this->edgePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_50_EDGES;
                        break;
                    case 3:
                        this->edgePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_75_EDGES;
                        break;
                    case 4:
                        this->edgePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_100_EDGES;
                        break;
                    case 5:
                        this->edgePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_200_EDGES;
                        break;
                    case 6:
                        this->edgePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_300_EDGES;
                        break;
                    case 7:
                        this->edgePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_400_EDGES;
                        break;
                    case 8:
                        this->edgePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_500_EDGES;
                        break;
                    case 9:
                        this->edgePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_600_EDGES;
                        break;
                    case 10:
                        this->edgePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_700_EDGES;
                        break;
                    case 11:
                        this->edgePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_800_EDGES;
                        break;
                    case 12:
                        this->edgePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_900_EDGES;
                        break;
                    default:
                        std::cout << "Unrecognized option, defaulting to graph with 25 edges.\n";
                        this->edgePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_25_EDGES;
                }

                break;
            case 2:
                clearScreen();
                std::cout << "Toy Graphs:\n";
                std::cout << "1. Shipping\n";
                std::cout << "2. Stadiums\n";
                std::cout << "3. Tourism\n\nInput: ";
                std::cin >> graphChoice;

                switch (graphChoice) {
                    case 1:
                        this->edgePath = DATASET_PATHS TOY_GRAPH_SHIPPING;
                        break;
                    case 2:
                        this->edgePath = DATASET_PATHS TOY_GRAPH_STADIUMS;
                        break;
                    case 3:
                        this->edgePath = DATASET_PATHS TOY_GRAPH_TOURISM;
                        break;
                    default:
                        std::cout << "Unrecognized option, defaulting to shipping graph.\n";
                        this->edgePath = DATASET_PATHS TOY_GRAPH_SHIPPING;
                }
                break;
            case 3:
                clearScreen();
                this->edgeFileSeparatedFromNodeFile = true;
                std::cout << "Real World Graphs:\n";
                std::cout << "1. Graph 1\n";
                std::cout << "2. Graph 2\n";
                std::cout << "3. Graph 3\n\nInput: ";
                std::cin >> graphChoice;

                switch (graphChoice) {
                    case 1:
                        this->edgePath = DATASET_PATHS REAL_WORLD_GRAPH_1_EDGES;
                        this->nodePath = DATASET_PATHS REAL_WORLD_GRAPH_1_NODES;
                        break;
                    case 2:
                        this->edgePath = DATASET_PATHS REAL_WORLD_GRAPH_2_EDGES;
                        this->nodePath = DATASET_PATHS REAL_WORLD_GRAPH_2_NODES;
                        break;
                    case 3:
                        this->edgePath = DATASET_PATHS REAL_WORLD_GRAPH_3_EDGES;
                        this->nodePath = DATASET_PATHS REAL_WORLD_GRAPH_3_NODES;
                        break;
                    default:
                        std::cout << "Unrecognized option, defaulting to first real world graph.\n";
                        this->edgePath = DATASET_PATHS REAL_WORLD_GRAPH_1_EDGES;
                        this->nodePath = DATASET_PATHS REAL_WORLD_GRAPH_1_NODES;
                }
                break;
            default:
                std::cout << "Unrecognized option, defaulting to extra fully connected graph with 25 edges.\n";
                this->edgeFileSeparatedFromNodeFile = true;
                this->edgePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_25_EDGES;
                this->nodePath = DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_NODES;
        }

        // DEBUG
        // TODO remove clearScreen();
        // TODO remove std::cout << "The current path for nodes is " << nodePath << " and edge's is " << edgePath << " cur env " << env << "\n";
    }

    void clearScreen();

    void run(int processedKey);

    static void delay(long sleepTime);

    static int processKey(const std::string &option);

private:

    /* Private functions */
    std::string showMainMenu();
    void showGoBackMenu(int option, const std::string& functionName);

    /* TSP functions */

    void backtrackingAlgorithmTSP(); // T2.1
    void triangularApproximationTSP(); // T2.2
    void optimizedTSP(); // T2.3
    void realWorldTSP(); // T2.4

    /* Attributes */
    std::string env; // System environment variable
    std::string edgePath; // Path for edges.csv
    std::string nodePath; // Path for nodes.csv
    bool edgeFileSeparatedFromNodeFile = false;
};

#endif //APLICATION_H