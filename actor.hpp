#ifndef ACTOR_HPP
#define ACTOR_HPP

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"

class Actor {
    std::unique_ptr<std::vector<std::shared_ptr<chrono::ChBody>>> _bodies;

  public:
    Actor();
    void init();
    void simulateTime(double time);
    std::vector<std::shared_ptr<chrono::ChBody>>* getBodies();
};

#endif // ACTOR_HPP
