#include "simulationsystem.hpp"
#include "visualizer.hpp"

int main(int argc, char* argv[]) {
    std::cout << "Starting simulator" << std::endl;

    SimulationSystem system;
    system.init();
    system.runSimulation();

    std::cout << "Starting visualizer" << std::endl;

    Visualizer vis;
    vis.show();

    std::cout << "Visualizer finished" << std::endl;

    system.waitForSimulation();
    return 0;
}
