#ifndef SIMULATIONSYSTEM_HPP
#define SIMULATIONSYSTEM_HPP

#include "simulator.hpp"

#include <atomic>
#include <mutex>
#include <stack>
#include <thread>

class SimulationSystem {
    std::mutex _actorsAccessMutex;
    std::atomic_bool _hasWork;
    std::stack<std::shared_ptr<Actor>> _actors;
    std::vector<std::thread> _threads;

  public:
    SimulationSystem();
    ~SimulationSystem();
    void init();

    void runSimulation();
    void simulationWorker();
    void waitForSimulation();
};

#endif // SIMULATIONSYSTEM_HPP
