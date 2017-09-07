#include "simulator.hpp"

chrono::ChSystemNSC* Simulator::physicalSystem() const {
    return _physicalSystem.get();
}

Simulator::Simulator() {}

void Simulator::init() {
    _physicalSystem.reset(new chrono::ChSystemNSC());
    auto mmaterial = std::make_shared<chrono::ChMaterialSurfaceNSC>();
    mmaterial->SetFriction(0.4f);
    mmaterial->SetCompliance(0.0000005f);
    mmaterial->SetComplianceT(0.0000005f);
    mmaterial->SetDampingF(0.2f);

    // FLOOR
    // auto floor_texture =
    // std::make_shared<chrono::ChTexture>(chrono::GetChronoDataFile("concrete.jpg"));
    auto mrigidFloor =
        std::make_shared<chrono::ChBodyEasyBox>(250, 4,
                                                250,   // x,y,z size
                                                1000,  // density
                                                true,  // collide enable?
                                                true); // visualization?

    mrigidFloor->SetPos(chrono::ChVector<>(0, -2, 0));
    mrigidFloor->SetMaterialSurface(mmaterial);
    mrigidFloor->SetBodyFixed(true);

    _physicalSystem->Add(mrigidFloor);
}

void Simulator::cleanup() {}

void Simulator::testActor(const std::shared_ptr<Actor>& actor) {
    init();
    std::vector<std::shared_ptr<chrono::ChBody>>* bodies = actor->getBodies();

    for (const auto& bodyPtr : (*bodies)) {
        _physicalSystem->Add(bodyPtr);
    }

    double chronoTime = 0;
    while (chronoTime < 10.0) {
        chronoTime += 0.001;
        _physicalSystem->DoFrameDynamics(chronoTime);
        actor->simulateTime(chronoTime);
    }
}
