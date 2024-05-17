#include "Application.h"

void Application::run(int processedKey) {
    L1:
    clearScreen();
    while (processedKey == -1){
        clearScreen();
        processedKey = processKey(showMainMenu());
    }

    switch (processedKey) {
        case 0:
            showRuntime();
            std::cout << 0;
            break;
        case 1:
            backtrackingAlgorithmTSP(); // T2.1
            std::cout << 1;
            break;
        case 2:
            triangularApproximationTSP(); // T2.2
            std::cout << 2;
            break;
        case 3:
            optimizedTSP(); // T2.3
            std::cout << 3;
            break;
        case 4:
            realWorldTSP(); // T2.4
            break;
        case 5:
            throw std::invalid_argument(std::to_string(-2));
            //something that crashes program
            break;
        case 6:
            //dataGoBoom();
            std::cout << "Thank you very much and Bye-Bye.\n";
            break;
        default:
            goto L1;
    }
}

/**
 * @brief clears (or simulates clearing) the screen, depending on the OS used
 */
void Application::clearScreen() {
    if (env == "win")
        L1:
        std::cout << "\n\n\n\n\n\n\n\n\n"
                     "\n\n\n\n\n\n\n\n\n"
                     "\n\n\n\n\n\n\n\n\n"
                     "\n\n\n\n\n\n\n\n\n";
    else if (env == "unix")
        if ( system("clear") == -1 ) goto L1;
}

/**
 * @brief Small delay for design purposes
 */
void Application::delay(long sleepMS) {
    std::this_thread::sleep_for(std::chrono::milliseconds(sleepMS));
}


/** @brief Handles the exception thrown by std::stoi. */
int Application::processKey(const std::string& option) {
    try {
        int intOPT = std::stoi(option);
        if (intOPT < 0 || option.size() > 1 || intOPT > 6) throw std::invalid_argument("NegativeNumber");
        return intOPT;
    } catch (std::invalid_argument& argument) {
        std::cout << "\n* Error while parsing option, please input a valid numeric option. *\n";
        delay(2000);
        return -1;
    }
}

/**
 * @brief Shows the main menu
 */
std::string Application::showMainMenu() {
    std::string opti;
    std::cout << "\nSelect an operation you would like to do:\n\n"
              << "0 - Calculate Runtime for a function\n"
              << "1 - Execute backtracking algorithm for TSP.\n"
              << "2 - Execute triangle approximation heuristic for TSP.\n"
              << "3 - Execute optimized TSP.\n"
              << "4 - Execute TSP in the Real World.\n"
              << "5 - Choose a different graph.\n"
              << "6 - Exit.\n";

    std::cout << "Input: ";
    std::cin >> opti;
    std::cout << "\n";
    return opti;
}

/**
 * @brief Shows the "Go back Menu" after each function
 */
void Application::showGoBackMenu(int functionNumber, const std::string& functionName) {
    L1:
    std::cout << "\nPress enter to continue";
    std::string a;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::getline(std::cin, a);
    clearScreen();
    std::cout << "\n\nWhat would you like to do next:\n"
              << "1 - Return to main menu.\n"
              << "2 - (again) " << functionName << "\n";

    std::string opt;
    std::cout << "Input: ";
    std::cin >> opt;
    std::cout << "\n";

    int processedKey = processKey(opt);

    switch (processedKey) {
        case -1:
            goto L1;
        case 1:
            throw std::invalid_argument("-1");
        case 2:
            // the no-no function -> run(functionNumber)
            throw std::invalid_argument(std::to_string(functionNumber));
        default:
            std::cout << "\n* Error while parsing option, please input a valid numeric option. *\n";
            goto L1;
    }
}

// Data structure to store runtime information
using RuntimeData = std::unordered_map<int,std::unordered_map<std::string, int>>;

// Function to read data from CSV file and organize it
RuntimeData readRuntimeData(const std::string& filename) {
    RuntimeData runtimeData;

    std::cout << "-------------DESCRIPTION--------------------" << std::endl;
    std::cout << "|Extra Fully Connected - 25 nodes --> ID 1 |" << std::endl;
    std::cout << "|Extra Fully Connected - 50 nodes --> ID 2 |" << std::endl;
    std::cout << "|Extra Fully Connected - 75 nodes --> ID 3 |" << std::endl;
    std::cout << "|Extra Fully Connected - 100 nodes -> ID 4 |" << std::endl;
    std::cout << "|Extra Fully Connected - 200 nodes -> ID 5 |" << std::endl;
    std::cout << "|Extra Fully Connected - 300 nodes -> ID 6 |" << std::endl;
    std::cout << "|Extra Fully Connected - 400 nodes -> ID 7 |" << std::endl;
    std::cout << "|Extra Fully Connected - 500 nodes -> ID 8 |" << std::endl;
    std::cout << "|Extra Fully Connected - 600 nodes -> ID 9 |" << std::endl;
    std::cout << "|Extra Fully Connected - 700 nodes -> ID 10|" << std::endl;
    std::cout << "|Extra Fully Connected - 800 nodes -> ID 11|" << std::endl;
    std::cout << "|Extra Fully Connected - 900 nodes -> ID 12|" << std::endl;
    std::cout << "|Toy Graphs - Shipping Graph -------> ID 13|" << std::endl;
    std::cout << "|Toy Graphs - Stadiums Graph -------> ID 14|" << std::endl;
    std::cout << "|Toy Graphs - Tourism Graph --------> ID 15|" << std::endl;
    std::cout << "|Real World Graphs - Graph 1 -------> ID 16|" << std::endl;
    std::cout << "|Real World Graphs - Graph 2 -------> ID 17|" << std::endl;
    std::cout << "|Real World Graphs - Graph 3 -------> ID 18|" << std::endl;
    std::cout << "--------------------------------------------" << std::endl;
    std::cout << "The X axis represent the functions, the Y axis\n"
                 "represents the graphs and the (X,Y) value the \n"
                 "time in miliseconds. Enjoy <3" << std::endl;
    std::cout << "--------------------------------------------" << std::endl;
    // Open the CSV file
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open " << filename << " for reading.\n";
        return runtimeData;
    }

    // Read each line from the CSV file
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string function, graphId, runtime;
        if (std::getline(ss, function, ',') && std::getline(ss, graphId, ',') && std::getline(ss, runtime, ',')) {
            // Convert runtime to integer
            try {
                int runtimeValue = std::stoi(runtime);
                // Add runtime to the data structure
                std::unordered_map<std::string, int> functionRun;
                functionRun[function] = runtimeValue;
                runtimeData[std::stoi(graphId)] = functionRun;
            } catch (const std::invalid_argument& e) {
                std::cerr << "Error: Invalid runtime value \"" << runtime << "\" encountered. Skipping line.\n";
            }
        } else {
            std::cerr << "Error: Malformed line in CSV file. Skipping line.\n";
        }
    }

    // Close the CSV file
    file.close();

    return runtimeData;
}

// Function to display runtime data
void displayRuntimeData(const RuntimeData& runtimeData) {
    // Calculate column widths
    std::vector<size_t> columnWidths;
    for (const auto& entry : runtimeData.begin()->second) {
        size_t width = entry.first.length();
        for (const auto& outerEntry : runtimeData) {
            size_t dataWidth = std::to_string(outerEntry.second.at(entry.first)).length();
            if (dataWidth > width) {
                width = dataWidth;
            }
        }
        columnWidths.push_back(width);
    }

// Print header
    std::cout << "| Graph ID ";
    size_t index = 0;
    for (const auto& entry : runtimeData.begin()->second) {
        std::cout << "| " << std::setw(columnWidths[index]) << entry.first << " ";
        index++;
    }
    std::cout << "|" << std::endl;

// Print separator
    std::cout << "|----------";
    for (size_t width : columnWidths) {
        std::cout << "|";
        for (size_t i = 0; i < width + 2; ++i) {
            std::cout << "-";
        }
    }
    std::cout << "|" << std::endl;

// Print data
    for (const auto& outerEntry : runtimeData) {
        std::cout << "| " << std::setw(8) << outerEntry.first << " ";
        index = 0;
        for (const auto& innerEntry : outerEntry.second) {
            std::cout << "| " << std::setw(columnWidths[index]) << innerEntry.second << " ";
            index++;
        }
        std::cout << "|" << std::endl;
    }
}

void Application::recordRuntime(const std::string& functionName, int duration) {
    // Open the CSV file in append mode
    std::ofstream outputFile("../dataset/runtime.csv", std::ios::app);
    if (!outputFile.is_open()) {
        std::cerr << "Error: Unable to open runtimes.csv for writing.\n";
        return;
    }

    // Write the function name and runtime to the CSV file
    outputFile << functionName << "," << this->graphChoosen << "," << duration << "\n";

    // Close the CSV file
    outputFile.close();
}

void Application::showRuntime() {

    std::string opti;
    std::cout << "\nSelect the function runtime you would like to calculate:\n\n"
              << "1 - Execute backtracking algorithm for TSP.\n"
              << "2 - Execute triangle approximation heuristic for TSP.\n"
              << "3 - Execute optimized TSP.\n"
              << "4 - Execute TSP in the Real World.\n"
              << "5 - Exit.\n";

    std::cout << "Input: ";
    std::cin >> opti;
    std::cout << "\n";

    clearScreen();
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    std::stringstream s;

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    switch (std::stoi(opti)) {
        case 1:
            backtrackingAlgorithmTSP(); // T2.1
            std::cout << 1;
            break;
        case 2:
            start = std::chrono::high_resolution_clock::now();
            tsp.triangularApproximation(s);
            end = std::chrono::high_resolution_clock::now();

            duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);


            // T2.2
            std::cout <<"It takes " <<duration.count() << " miliseconds" <<std::endl;
            recordRuntime("4.2",duration.count());
            break;
        case 3:
            optimizedTSP(); // T2.3
            std::cout << 3;
            break;
        case 4:
            realWorldTSP(); // T2.4
            break;
        case 5:
            //dataGoBoom();
            std::cout << "Thank you very much and Bye-Bye.\n";
            break;
        default:
            break;
    }

    RuntimeData runtimeData = readRuntimeData("../dataset/runtime.csv");
    displayRuntimeData(runtimeData);

    showGoBackMenu(0,"Execute runtime."); // At the end make a call to goBackMenu()


}



// T2.1
void Application::backtrackingAlgorithmTSP(){
    clearScreen();

    std::cout << "Checking if the graph is fully connected.\n" << "** Note: The time complexity of this function will not be considered for metrics **\n";

    std::string output = tsp.makeGraphConnected() ? "\nGraph was already fully connected.\n" : "\nGraph was not fully connected.\n";

    std::cout << output;

    clearScreen();

    auto start = std::chrono::high_resolution_clock::now();
    std::pair<double, std::vector<Vertex<GeoPoint*>*>> result = tsp.tspBTSetup();
    auto stop = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    std::cout << "\nNow executing cleanup.\n**Note: The time complexity of this function will not be considered for metrics.**\n";
    tsp.cleanUpGraph();
    std::cout << "\nBacktracking took " << duration.count() << " ms.\n";

    if (result.second.empty()) {
        std::cout << "No path was found\n";
    } else {
        std::cout << "The optimal path is: ";

        for (const Vertex<GeoPoint*>* geoPointVertex : result.second) {
            std::cout << geoPointVertex->getInfo()->getId() << " -> ";
        }
        std::cout << result.second.at(0)->getInfo()->getId() << "\n";
        std::cout << "And the final result is: " << result.first << "\n";
    }
    showGoBackMenu(1,"Execute backtracking algorithm for TSP."); // At the end make a call to goBackMenu()
}

// T2.2
void Application::triangularApproximationTSP(){


    clearScreen();
    std::stringstream tourDescription;
    double totalDistance = tsp.triangularApproximation(tourDescription);
    std::cout << std::endl<<"Triangular Approximation TSP:" << std::endl;
    if(!this->isBigGraph) {
        // Print the tour description
        std::cout << "Tour:" << std::endl;
        std::cout << tourDescription.str();
        if(this->graphChoosen == 14) {
            double ratio = totalDistance / 341.5;
            std::cout << std::endl << "Ratio (TA distance/Optimal): " << ratio << std::endl;
        }else if(this->graphChoosen == 15){
            double ratio = totalDistance / 2600.0;
            std::cout << std::endl << "Ratio (TA distance/Optimal): " << ratio << std::endl;
        }
    }
    // Print the total distance

    std::cout << std::endl << "Total distance: " << totalDistance << " meters" << std::endl;



    showGoBackMenu(2, "Execute triangle approximation heuristic for TSP."); // At the end make a call to goBackMenu()
}

// T2.3
void Application::optimizedTSP(){
    clearScreen();

    std::string response;
    bool connectingNeeded = false;

    if (isToyGraph){
        tsp.setIsToyGraph(true);
    }

    if (nodePath == DATASET_PATHS EXTRA_FULLY_CONNECTED_GRAPHS_NODES || nodePath == DATASET_PATHS TOY_GRAPH_STADIUMS || nodePath == DATASET_PATHS TOY_GRAPH_TOURISM){
        std::cout << "\nThe graph is connected.\nExecuting optimized algorithm...";
    }
    else {
        connectingNeeded = true;
        std::cout << "This function needs the graph to be fully connected.\n";
        std::cout << "Connecting the graph...";
        if (isToyGraph){
            std::string output = tsp.makeGraphConnected() ? "\nGraph was already fully connected.\n" : "\nGraph was not fully connected.\n";
            std::cout << output;
        }
        else{
            //std::string output = tsp.makeGraphConnectedWithHaversine() ? "\nGraph was already fully connected.\n" : "\nGraph was not fully connected.\n";
            //std::cout << output;
        }
    }

    clearScreen();

    auto start = std::chrono::high_resolution_clock::now();
    tsp.otherHeuristic(false, -1);
    auto stop = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "\n=> Optimized TSP algorithm took " << duration.count() << " ms.\n";
    if (connectingNeeded){
        std::cout << "** Note: The time complexity of connecting the graph is not considered for metrics **\n";
        std::cout << "\nNow executing cleanup.\n**Note: The time complexity of this function will not be considered for metrics.**\n";
        tsp.cleanUpGraph();
    }

    showGoBackMenu(3, "Execute optimized TSP."); // At the end make a call to goBackMenu()
}

// T2.4
void Application::realWorldTSP(){
    clearScreen();


    std::string opti;
    std::cout << "\nType the vertex id that you want to start the tour: ";
    std::cin >> opti;
    std::cout << "\n";

    clearScreen();


    std::cout << "Loading......";


    std::vector<GeoPoint*> res;
    double count = 0.0;
    std::vector<GeoPoint*> bestres;
    double best = 0.0;
    clearScreen();
    if(tsp.nnRecursion(stoi(opti),-1,res, count,bestres,best)) std::cout <<"RIGHT";



    std::cout << std::endl << "Count "<< count;
    std::cout << " BEST size -> " << bestres.size() << std::endl;
    std::cout << "BEST COUNT -> " << best << std::endl;

    tsp.cleanUpGraph();

    showGoBackMenu(4, "Execute TSP in the Real World."); // At the end make a call to goBackMenu()
}
