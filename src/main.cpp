#include "Application.h"

/**@brief Main function that gets the app started.*/
int main() {

    /** @brief Check the environment we are on.
     *  @details This is done to use system specific commands like system("clear").
     *  @details Time Complexity: O(1)
    * */
    const char *os = std::getenv("OS");
    std::string env;
    if (os == nullptr) env = "null";
    else env = std::string(os);
    if (env == "Windows_NT") {
        env = "win";
    } else {
        env = "unix";
    }

    chooseDifferentGraph:
    Application app = Application(env);
    int nextRun = -1;

    goBack:
    if (nextRun == -2 ){
        goto chooseDifferentGraph;
    }
    try {
        app.run(nextRun);
    } catch (std::invalid_argument &invalid_argument) {
        nextRun = std::stoi(invalid_argument.what());
        goto goBack;
    }

    return 0;
}