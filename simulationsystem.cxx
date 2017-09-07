#include "simulationsystem.hpp"

SimulationSystem::SimulationSystem() {}

SimulationSystem::~SimulationSystem() { waitForSimulation(); }

void SimulationSystem::init() {}

void SimulationSystem::runSimulation() {
    std::cout << "Simulation start..." << std::endl;
    _hasWork.store(true);
    _actorsAccessMutex.unlock();

    _actors.push(std::make_shared<Actor>());
    _actors.push(std::make_shared<Actor>());
    _actors.push(std::make_shared<Actor>());
    _actors.push(std::make_shared<Actor>());
    _actors.push(std::make_shared<Actor>());
    _actors.push(std::make_shared<Actor>());
    _actors.push(std::make_shared<Actor>());

    for (int i = 0; i < 666; ++i) {
        std::shared_ptr<Actor> actor(new Actor());
        _actors.push(actor);
    }

    _threads.emplace_back([this]() { this->simulationWorker(); });
    _threads.emplace_back([this]() { this->simulationWorker(); });
    _threads.emplace_back([this]() { this->simulationWorker(); });
    _threads.emplace_back([this]() { this->simulationWorker(); });
    _threads.emplace_back([this]() { this->simulationWorker(); });
    _threads.emplace_back([this]() { this->simulationWorker(); });
    _threads.emplace_back([this]() { this->simulationWorker(); });
    //_threads.emplace_back([this]() { simulationWorker(); });
    std::cout << "Threads created..." << std::endl;
}

void SimulationSystem::simulationWorker() {
    Simulator sim;
    std::cout << "Thread..." << std::endl;
    while (_hasWork.load()) {
        _actorsAccessMutex.lock();
        std::cout << "Size: " << _actors.size() << std::endl;
        if (_actors.empty()) {
            _hasWork.store(false);
            _actorsAccessMutex.unlock();
            break;
        }
        auto actor = _actors.top();
        _actors.pop();

        _actorsAccessMutex.unlock();

        sim.testActor(actor);
    }
    std::cout << "Thread ready" << std::endl;
}

void SimulationSystem::waitForSimulation() {
    std::cout << "Waiting for simulation to finish..." << std::endl;
    for (auto& thread : _threads) {
        thread.join();
    }
    _threads.clear();
    std::cout << "Simulation finished" << std::endl;
}
