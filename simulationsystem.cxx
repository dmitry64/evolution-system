#include "simulationsystem.hpp"

SimulationSystem::SimulationSystem() {}

SimulationSystem::~SimulationSystem() { waitForSimulation(); }

void SimulationSystem::init() {}

void SimulationSystem::runSimulation() {
    _hasWork.store(true);

    _actors.push(std::make_shared<Actor>());
    _actors.push(std::make_shared<Actor>());
    _actors.push(std::make_shared<Actor>());
    _actors.push(std::make_shared<Actor>());
    _actors.push(std::make_shared<Actor>());
    _actors.push(std::make_shared<Actor>());
    _actors.push(std::make_shared<Actor>());

    for (int i = 0; i < 100; ++i) {
        std::shared_ptr<Actor> actor(new Actor());
        _actors.push(actor);
    }

    _threads.emplace_back([this]() { simulationWorker(); });
    _threads.emplace_back([this]() { simulationWorker(); });
    _threads.emplace_back([this]() { simulationWorker(); });
    _threads.emplace_back([this]() { simulationWorker(); });
    _threads.emplace_back([this]() { simulationWorker(); });
    _threads.emplace_back([this]() { simulationWorker(); });
    _threads.emplace_back([this]() { simulationWorker(); });
    _threads.emplace_back([this]() { simulationWorker(); });
}

void SimulationSystem::simulationWorker() {
    Simulator sim;
    while (_hasWork.load()) {
        _actorsAccessMutex.lock();
        if (_actors.empty()) {
            _hasWork.store(false);
            _actorsAccessMutex.unlock();
            break;
        }
        auto actor = _actors.top();
        _actors.pop();
        _actorsAccessMutex.unlock();

        sim.testActor(actor);
        sim.cleanup();
    }
}

void SimulationSystem::waitForSimulation() {
    for (auto& thread : _threads) {
        thread.join();
    }
    _threads.clear();
}
