#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"

#include "actor.hpp"

class Simulator {
    std::unique_ptr<chrono::ChSystemNSC> _physicalSystem;

  public:
    Simulator();
    void init();
    void cleanup();
    void testActor(std::shared_ptr<Actor>& actor);
    chrono::ChSystemNSC* physicalSystem() const;
};

#endif // SIMULATOR_HPP
