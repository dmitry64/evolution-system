#include "actor.hpp"

std::vector<std::shared_ptr<chrono::ChLink>>* Actor::getLinks() const {
    return _links.get();
}

std::vector<std::shared_ptr<chrono::ChBody>>* Actor::getBodies() const {
    return _bodies.get();
}

Actor::Actor()
    : _bodies(new std::vector<std::shared_ptr<chrono::ChBody>>()),
      _links(new std::vector<std::shared_ptr<chrono::ChLink>>()) {
    auto mmaterial = std::make_shared<chrono::ChMaterialSurfaceNSC>();
    mmaterial->SetFriction(0.5f);
    mmaterial->SetCompliance(0.0000005f);
    mmaterial->SetComplianceT(0.0000005f);
    mmaterial->SetDampingF(0.2f);

    // BALL
    auto mrigidBall1 =
        std::make_shared<chrono::ChBodyEasyBox>(8, 2, 3, // radius
                                                1000,    // density
                                                true,    // collide enable?
                                                true);   // visualization?
    // mrigidBall->SetMass(1.0);

    mrigidBall1->SetMaterialSurface(mmaterial);
    mrigidBall1->SetPos(chrono::ChVector<>(-6, 18, -8));
    mrigidBall1->SetPos_dt(chrono::ChVector<>(0, 0, 0));
    mrigidBall1->GetMaterialSurfaceNSC()->SetFriction(0.9f);
    mrigidBall1->GetMaterialSurfaceNSC()->SetCompliance(0.0);
    mrigidBall1->GetMaterialSurfaceNSC()->SetComplianceT(0.0);
    mrigidBall1->GetMaterialSurfaceNSC()->SetDampingF(0.2f);
    // mrigidBall1->SetBodyFixed(true);
    mrigidBall1->SetName("Ball 1");

    _bodies->push_back(mrigidBall1);

    auto mrigidBall2 =
        std::make_shared<chrono::ChBodyEasySphere>(3,     // radius
                                                   1000,  // density
                                                   true,  // collide enable?
                                                   true); // visualization?
    // mrigidBall->SetMass(1.0);

    mrigidBall2->SetMaterialSurface(mmaterial);
    mrigidBall2->SetPos(chrono::ChVector<>(6, 18, -8));
    mrigidBall2->SetPos_dt(chrono::ChVector<>(0, 0, 0));
    mrigidBall2->GetMaterialSurfaceNSC()->SetFriction(0.4f);
    mrigidBall2->GetMaterialSurfaceNSC()->SetCompliance(0.0);
    mrigidBall2->GetMaterialSurfaceNSC()->SetComplianceT(0.0);
    mrigidBall2->GetMaterialSurfaceNSC()->SetDampingF(0.2f);

    _bodies->push_back(mrigidBall2);

    auto mrigidBall3 =
        std::make_shared<chrono::ChBodyEasySphere>(6,     // radius
                                                   1000,  // density
                                                   true,  // collide enable?
                                                   true); // visualization?
    mrigidBall3->SetMaterialSurface(mmaterial);
    mrigidBall3->SetPos(chrono::ChVector<>(9, 18, -12));
    mrigidBall3->SetPos_dt(chrono::ChVector<>(0, 0, 0));
    mrigidBall3->GetMaterialSurfaceNSC()->SetFriction(0.4f);
    mrigidBall3->GetMaterialSurfaceNSC()->SetCompliance(0.0);
    mrigidBall3->GetMaterialSurfaceNSC()->SetComplianceT(0.0);
    mrigidBall3->GetMaterialSurfaceNSC()->SetDampingF(0.2f);

    _bodies->push_back(mrigidBall3);

    auto mrigidBall4 =
        std::make_shared<chrono::ChBodyEasySphere>(3,     // radius
                                                   1000,  // density
                                                   true,  // collide enable?
                                                   true); // visualization?
    mrigidBall4->SetMaterialSurface(mmaterial);
    mrigidBall4->SetPos(chrono::ChVector<>(0, 29, -5));
    mrigidBall4->SetPos_dt(chrono::ChVector<>(0, 0, 0));
    mrigidBall4->GetMaterialSurfaceNSC()->SetFriction(0.4f);
    mrigidBall4->GetMaterialSurfaceNSC()->SetCompliance(0.0);
    mrigidBall4->GetMaterialSurfaceNSC()->SetComplianceT(0.0);
    mrigidBall4->GetMaterialSurfaceNSC()->SetDampingF(0.2f);

    _bodies->push_back(mrigidBall4);

    /*auto my_link_01 = std::make_shared<chrono::ChLinkLinActuator>();
    my_link_01->Initialize(mrigidBall1, mrigidBall2,
                           chrono::ChCoordsys<>(chrono::ChVector<>(0, 0, 0)));
    my_link_01->GetLimit_X()->Set_active(true);
    my_link_01->GetLimit_X()->Set_max(0.5);
    my_link_01->GetLimit_X()->Set_min(-0.0);
    _links->push_back(my_link_01);*/

    auto actuator_fun = std::make_shared<chrono::ChFunction_Const>(16.0);
    // actuator_fun->Set_yconst(6.5);

    auto actuator1 = std::make_shared<chrono::ChLinkLinActuator>();
    actuator1->Initialize(
        mrigidBall1, mrigidBall2, false,
        chrono::ChCoordsys<>(mrigidBall1->GetPos(), chrono::QUNIT),
        chrono::ChCoordsys<>(mrigidBall2->GetPos(), chrono::QUNIT));
    actuator1->SetName("actuator 1");
    actuator1->Set_lin_offset(0.00001);
    actuator1->Set_dist_funct(actuator_fun);
    actuator1->GetForce_Rx()->Set_R(100);
    _links->push_back(actuator1);

    auto actuator2 = std::make_shared<chrono::ChLinkLinActuator>();
    actuator2->Initialize(
        mrigidBall2, mrigidBall3, false,
        chrono::ChCoordsys<>(mrigidBall2->GetPos(), chrono::QUNIT),
        chrono::ChCoordsys<>(mrigidBall3->GetPos(), chrono::QUNIT));
    actuator2->SetName("actuator 2");
    actuator2->Set_lin_offset(0.00001);
    actuator2->Set_dist_funct(actuator_fun);
    _links->push_back(actuator2);

    auto actuator3 = std::make_shared<chrono::ChLinkLinActuator>();
    actuator3->Initialize(
        mrigidBall3, mrigidBall1, false,
        chrono::ChCoordsys<>(mrigidBall3->GetPos(), chrono::QUNIT),
        chrono::ChCoordsys<>(mrigidBall1->GetPos(), chrono::QUNIT));
    actuator3->SetName("actuator 32");
    actuator3->Set_lin_offset(0.00001);
    actuator3->Set_dist_funct(actuator_fun);
    actuator3->GetLimit_X()->Set_active(true);
    actuator3->GetLimit_X()->Set_min(3);
    actuator3->GetLimit_X()->Set_max(19);
    _links->push_back(actuator3);

    auto actuator12 = std::make_shared<chrono::ChLinkLinActuator>();
    actuator12->Initialize(
        mrigidBall1, mrigidBall4, false,
        chrono::ChCoordsys<>(mrigidBall1->GetPos(), chrono::QUNIT),
        chrono::ChCoordsys<>(mrigidBall4->GetPos(), chrono::QUNIT));
    actuator12->SetName("actuator 12");
    actuator12->Set_lin_offset(0.00001);
    actuator12->Set_dist_funct(actuator_fun);
    _links->push_back(actuator12);

    auto actuator22 = std::make_shared<chrono::ChLinkLinActuator>();
    actuator22->Initialize(
        mrigidBall2, mrigidBall4, false,
        chrono::ChCoordsys<>(mrigidBall2->GetPos(), chrono::QUNIT),
        chrono::ChCoordsys<>(mrigidBall4->GetPos(), chrono::QUNIT));
    actuator22->SetName("actuator 22");
    actuator22->Set_lin_offset(0.00001);
    actuator22->Set_dist_funct(actuator_fun);
    _links->push_back(actuator22);

    auto actuator32 = std::make_shared<chrono::ChLinkLinActuator>();
    actuator32->Initialize(
        mrigidBall3, mrigidBall4, false,
        chrono::ChCoordsys<>(mrigidBall3->GetPos(), chrono::QUNIT),
        chrono::ChCoordsys<>(mrigidBall4->GetPos(), chrono::QUNIT));
    actuator32->SetName("actuator 32");
    actuator32->Set_lin_offset(0.00001);
    actuator32->Set_dist_funct(actuator_fun);
    _links->push_back(actuator32);

    //   my_system.AddLink(my_link_01);

    /*auto mrigid2 =
        std::make_shared<chrono::ChBodyEasyCylinder>(1, // radius
                                                     6,
                                                     3,     // density
                                                     false, // collide enable?
                                                     true); // visualization?
    mrigid2->SetMaterialSurface(mmaterial);s
    mrigid2->SetPos(chrono::ChVector<>(5, 7, -8));
    mrigid2->SetPos_dt(chrono::ChVector<>(0, 0, 6.5));
    mrigid2->GetMaterialSurfaceNSC()->SetFriction(0.4f);
    mrigid2->GetMaterialSurfaceNSC()->SetCompliance(0.0);
    mrigid2->GetMaterialSurfaceNSC()->SetComplianceT(0.0);
    mrigid2->GetMaterialSurfaceNSC()->SetDampingF(0.2f);
    _bodies->push_back(mrigid2);*/
}

void Actor::simulateTime(double time) {
    chrono::ChLinkLinActuator* act =
        static_cast<chrono::ChLinkLinActuator*>(_links->at(0).get());
    chrono::ChFunction_Const* fun = reinterpret_cast<chrono::ChFunction_Const*>(
        act->Get_dist_funct().get());
    fun->Set_yconst(16.0 + ((std::sin(time) + 1.0) * 10.0));
    // act->Get_dist_funct()->Set_yconst(6.5);
    // TODO: fix
    //_bodies->at(0)->SetPos_dt(chrono::ChVector<>(0, 0, 2.5));
}

std::vector<double> Actor::getNNInput() {
    std::vector<double> result;

    for (const auto& body : *_bodies) {
        const auto& speed = body->GetPos_dt();
        result.push_back(speed.x());
        result.push_back(speed.y());
        result.push_back(speed.z());
    }

    return result;
}

void Actor::useNNOutput(const std::vector<double>& values) {}

std::vector<std::vector<chrono::ChVector<>>> Actor::getLines() {
    std::vector<std::vector<chrono::ChVector<>>> result;

    for (const auto& link : *_links) {
        std::vector<chrono::ChVector<>> pair;
        chrono::ChVector<> vect1 = link->GetBody1()->GetPos();
        chrono::ChVector<> vect2 = link->GetBody2()->GetPos();
        pair.push_back(vect1);
        pair.push_back(vect2);
        result.push_back(pair);
    }

    return result;
}
