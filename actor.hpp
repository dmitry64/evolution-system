#ifndef ACTOR_HPP
#define ACTOR_HPP

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"

class Actor {
    std::unique_ptr<std::vector<std::shared_ptr<chrono::ChBody>>> _bodies;
    std::unique_ptr<std::vector<std::shared_ptr<chrono::ChLink>>> _links;
    std::shared_ptr<chrono::ChFunction_Const> _leftLegMuscle;
    std::shared_ptr<chrono::ChFunction_Const> _leftFootMuscle;
    std::shared_ptr<chrono::ChFunction_Const> _rightLegMuscle;
    std::shared_ptr<chrono::ChFunction_Const> _rightFootMuscle;
    std::shared_ptr<chrono::ChFunction_Const> _bodyRightLegMuscle;
    std::shared_ptr<chrono::ChFunction_Const> _bodyLeftLegMuscle;

    std::shared_ptr<chrono::ChBodyEasyBox> _leftUpperLeg;
    std::shared_ptr<chrono::ChBodyEasyBox> _rightUpperLeg;

  public:
    Actor();
    void simulateTime(double time);

    std::vector<double> getNNInput();
    void useNNOutput(const std::vector<double>& values);
    std::vector<std::vector<chrono::ChVector<>>> getLines();
    std::vector<std::shared_ptr<chrono::ChLink>>* getLinks() const;
    std::vector<std::shared_ptr<chrono::ChBody>>* getBodies() const;

  private:
    void createLeftLeg();
    void createRightLeg();
};

#endif // ACTOR_HPP
