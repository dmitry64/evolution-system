#include "simulator.hpp"

chrono::ChSystemNSC* Simulator::physicalSystem() const {
    return _physicalSystem.get();
}

Simulator::Simulator() {}

void Simulator::init() {
    _physicalSystem.reset(new chrono::ChSystemNSC());
    _physicalSystem->SetSolverType(chrono::ChSolver::Type::SOR);

    // mphysicalSystem.SetUseSleeping(true);

    _physicalSystem->SetMaxPenetrationRecoverySpeed(
        1.6); // used by Anitescu stepper only
    _physicalSystem->SetMaxItersSolverSpeed(40);
    _physicalSystem->SetMaxItersSolverStab(
        20); // unuseful for Anitescu, only Tasora uses this
    _physicalSystem->SetSolverWarmStarting(true);

    auto mmaterial = std::make_shared<chrono::ChMaterialSurfaceNSC>();
    mmaterial->SetFriction(0.8f);
    // mmaterial->SetCompliance(0.0000005f);
    // mmaterial->SetComplianceT(0.0000005f);
    mmaterial->SetDampingF(0.2f);

    // FLOOR
    // auto floor_texture =
    // std::make_shared<chrono::ChTexture>(chrono::GetChronoDataFile("concrete.jpg"));
    auto mrigidFloor =
        std::make_shared<chrono::ChBodyEasyBox>(250, 4,
                                                250,   // x,y,z size
                                                80000, // density
                                                true,  // collide enable?
                                                true); // visualization?
    // mrigidFloor->SetMass(50.0);
    mrigidFloor->SetPos(chrono::ChVector<>(0, -3, 0));
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

    std::vector<std::shared_ptr<chrono::ChLink>>* links = actor->getLinks();

    for (const auto& linkPtr : (*links)) {
        _physicalSystem->AddLink(linkPtr);
    }

    double chronoTime = 0;
    while (chronoTime < 3.0) {
        chronoTime += 0.03;
        _physicalSystem->DoFrameDynamics(chronoTime);
        actor->simulateTime(chronoTime);
    }
}
