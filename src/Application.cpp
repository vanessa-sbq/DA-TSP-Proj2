#include "Application.h"

void Application::run(int processedKey) {
    L1:
    clearScreen();
    while (processedKey == -1){
        clearScreen();
        processedKey = processKey(showMainMenu());
    }

    switch (processedKey) {
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
        if (intOPT <= 0 || option.size() > 1 || intOPT > 5) throw std::invalid_argument("NegativeNumber");
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
              << "1 - Execute backtracking algorithm for TSP.\n"
              << "2 - Execute triangle approximation heuristic for TSP.\n"
              << "3 - Execute optimized TSP.\n"
              << "4 - Execute TSP in the Real World.\n"
              << "5 - Exit.\n";

    std::cout << "Input: ";
    std::cin >> opti;
    std::cout << "\n";
    return opti;
}

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

// T2.1
void Application::backtrackingAlgorithmTSP(){
    clearScreen();

    std::string response;
    std::cout << "This function needs the graph to be fully connected.\n"
              << "Is this graph fully connected ?\n"
              << "Yes - y/Y | " << "No - n/N | " << "Don't know - ?\n";
    std::cout << "Input: ";
    std::cin >> response;

    if (std::regex_match(response, std::regex("[y]",std::regex_constants::icase))) {
        std::cout << "\nAssuming graph is fully connected\n";
    } else {
        std::cout << "\nAssuming graph is not fully connected\n" << "Checking and adding edges if needed.\n"
                  << "** Note: The time complexity of this function will not be considered for metrics **\n";
        std::string output = tsp.makeGraphConnected() ? "\nGraph was already fully connected.\n" : "\nGraph was not fully connected.\n";
        std::cout << output;
    }

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

    double res = tsp.triangularApproximation();
    std::cout << res;
    showGoBackMenu(2, "Execute triangle approximation heuristic for TSP."); // At the end make a call to goBackMenu()
}

// T2.3
void Application::optimizedTSP(){
    clearScreen();

    std::string response;
    std::cout << "This function needs the graph to be fully connected.\n"
              << "Is this graph fully connected ?\n"
              << "Yes - y/Y | " << "No - n/N | " << "Don't know - ?\n";
    std::cout << "Input: ";
    std::cin >> response;
    bool connectingNeeded = false;

    if (!std::regex_match(response, std::regex("[y]",std::regex_constants::icase))) {
        connectingNeeded = true;
        if (isToyGraph){
            std::cout << "TOY GRAPH"; // TODO: REMOVE
            tsp.setIsToyGraph(true);
            std::string output = tsp.makeGraphConnected() ? "\nGraph was already fully connected.\n" : "\nGraph was not fully connected.\n";
            std::cout << output;
        } else{
            std::string output = tsp.makeGraphConnectedWithHaversine() ? "\nGraph was already fully connected.\n" : "\nGraph was not fully connected.\n";
            std::cout << output;
        }
    }

    clearScreen();

    auto start = std::chrono::high_resolution_clock::now();
    tsp.otherHeuristic();
    auto stop = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "\n=> Optimized TSP algorithm took " << duration.count() << " ms.\n";
    if (connectingNeeded){
        std::cout << "** Note: The time complexity of connecting the graph is not considered for metrics **\n";
    }

    std::cout << "\nNow executing cleanup.\n**Note: The time complexity of this function will not be considered for metrics.**\n";
    tsp.cleanUpGraph();
    std::cout << "\nBacktracking took " << duration.count() << " ms.\n";

    showGoBackMenu(3, "Execute optimized TSP."); // At the end make a call to goBackMenu()
}

// T2.4
void Application::realWorldTSP(){
    clearScreen();
    //Code here

    showGoBackMenu(4, "Execute TSP in the Real World."); // At the end make a call to goBackMenu()
}
