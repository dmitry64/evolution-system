#include "actor.hpp"

Actor::Actor() : _bodies(new std::vector<std::shared_ptr<chrono::ChBody>>()) {
    init();
}

void Actor::init() {
    auto mmaterial = std::make_shared<chrono::ChMaterialSurfaceNSC>();
    mmaterial->SetFriction(0.4f);
    mmaterial->SetCompliance(0.0000005f);
    mmaterial->SetComplianceT(0.0000005f);
    mmaterial->SetDampingF(0.2f);

    // BALL
    // auto ball_texture =
    // std::make_shared<chrono::ChTexture>(GetChronoDataFile("pinkwhite.png"));
    auto mrigidBall =
        std::make_shared<chrono::ChBodyEasySphere>(4,     // radius
                                                   8000,  // density
                                                   true,  // collide enable?
                                                   true); // visualization?
    mrigidBall->SetPos(chrono::ChVector<>(0, -2, 0));
    mrigidBall->SetMaterialSurface(mmaterial);
    mrigidBall->SetPos(chrono::ChVector<>(0, 3, -8));
    mrigidBall->SetPos_dt(chrono::ChVector<>(0, 0, 2.5));
    mrigidBall->GetMaterialSurfaceNSC()->SetFriction(0.4f);
    mrigidBall->GetMaterialSurfaceNSC()->SetCompliance(0.0);
    mrigidBall->GetMaterialSurfaceNSC()->SetComplianceT(0.0);
    mrigidBall->GetMaterialSurfaceNSC()->SetDampingF(0.2f);
    // mrigidBall->AddAsset(ball_texture);
    _bodies->push_back(mrigidBall);
}

void Actor::simulateTime(double time) {
    // TODO: fix
    _bodies->at(0)->SetPos_dt(chrono::ChVector<>(0, time, 2.5));
}

std::vector<std::shared_ptr<chrono::ChBody>>* Actor::getBodies() {
    return _bodies.get();
}
