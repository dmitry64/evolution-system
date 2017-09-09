#ifndef ACTOR_HPP
#define ACTOR_HPP

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"

class Actor {
    std::unique_ptr<std::vector<std::shared_ptr<chrono::ChBody>>> _bodies;
    std::unique_ptr<std::vector<std::shared_ptr<chrono::ChLink>>> _links;

  public:
    Actor();
    void simulateTime(double time);

    std::vector<double> getNNInput();
    void useNNOutput(const std::vector<double>& values);
    std::vector<std::vector<chrono::ChVector<>>> getLines();
    std::vector<std::shared_ptr<chrono::ChLink>>* getLinks() const;
    std::vector<std::shared_ptr<chrono::ChBody>>* getBodies() const;
};

#endif // ACTOR_HPP
