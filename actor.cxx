#include "actor.hpp"

std::vector<std::shared_ptr<chrono::ChLink>>* Actor::getLinks() const {
    return _links.get();
}

std::vector<std::shared_ptr<chrono::ChBody>>* Actor::getBodies() const {
    return _bodies.get();
}

void Actor::createLeftLeg() {
    auto mmaterial = std::make_shared<chrono::ChMaterialSurfaceNSC>();
    mmaterial->SetFriction(0.5f);
    mmaterial->SetCompliance(0.0000005f);
    mmaterial->SetComplianceT(0.0000005f);
    mmaterial->SetDampingF(0.2f);

    _leftUpperLeg =
        std::make_shared<chrono::ChBodyEasyBox>(1, 20, 1, // radius
                                                800,      // density
                                                true,     // collide enable?
                                                true);    // visualization?
    // mrigidBall->SetMass(1.0);

    _leftUpperLeg->SetMaterialSurface(mmaterial);
    _leftUpperLeg->SetPos(chrono::ChVector<>(0, 37, -10));
    // upperLeg->SetRot(chrono::Q_from_AngX(-3.14159265359 / 6.0));
    _leftUpperLeg->SetPos_dt(chrono::ChVector<>(0, 0, 0));
    _leftUpperLeg->GetMaterialSurfaceNSC()->SetFriction(0.9f);
    _leftUpperLeg->GetMaterialSurfaceNSC()->SetCompliance(0.0);
    _leftUpperLeg->GetMaterialSurfaceNSC()->SetComplianceT(0.0);
    _leftUpperLeg->GetMaterialSurfaceNSC()->SetDampingF(0.2f);
    // mrigidBall1->SetBodyFixed(true);
    _leftUpperLeg->SetName("Upper leg");
    // upperLeg->SetBodyFixed(true);

    _bodies->push_back(_leftUpperLeg);

    auto bottomLeg =
        std::make_shared<chrono::ChBodyEasyBox>(1, 20, 1, // radius
                                                800,      // density
                                                true,     // collide enable?
                                                true);    // visualization?
    // mrigidBall->SetMass(1.0);

    bottomLeg->SetMaterialSurface(mmaterial);
    bottomLeg->SetPos(chrono::ChVector<>(0, 18, -10));
    // bottomLeg->SetRot(chrono::Q_from_AngX(3.14159265359 / 6.0));
    bottomLeg->SetPos_dt(chrono::ChVector<>(0, 0, 0));
    bottomLeg->GetMaterialSurfaceNSC()->SetFriction(0.9f);
    bottomLeg->GetMaterialSurfaceNSC()->SetCompliance(0.0);
    bottomLeg->GetMaterialSurfaceNSC()->SetComplianceT(0.0);
    bottomLeg->GetMaterialSurfaceNSC()->SetDampingF(0.2f);
    // mrigidBall1->SetBodyFixed(true);
    bottomLeg->SetName("Bottom leg");

    _bodies->push_back(bottomLeg);

    auto kneeJoint = std::make_shared<chrono::ChLinkLockSpherical>();
    kneeJoint->Initialize(
        _leftUpperLeg, bottomLeg, false,
        chrono::ChCoordsys<>(_leftUpperLeg->GetPos() +
                                 chrono::ChVector<>(0, -11, 0),
                             chrono::QUNIT),
        chrono::ChCoordsys<>(bottomLeg->GetPos() + chrono::ChVector<>(0, 11, 0),
                             chrono::QUNIT));
    kneeJoint->SetName("Knee joint");
    kneeJoint->GetLimit_Rx()->Set_active(true);
    kneeJoint->GetLimit_Rx()->Set_max(0.01);
    kneeJoint->GetLimit_Rx()->Set_min(0.01);

    kneeJoint->GetLimit_Ry()->Set_active(true);
    kneeJoint->GetLimit_Ry()->Set_max(0.01);
    kneeJoint->GetLimit_Ry()->Set_min(0.01);

    kneeJoint->GetLimit_Rp()->Set_active(true);
    kneeJoint->GetLimit_Rp()->Set_max(0.01);
    kneeJoint->GetLimit_Rp()->Set_min(0.01);

    _links->push_back(kneeJoint);

    _leftLegMuscle = std::make_shared<chrono::ChFunction_Const>(12.0);
    auto actuator1 = std::make_shared<chrono::ChLinkLinActuator>();
    actuator1->Initialize(
        _leftUpperLeg, bottomLeg, false,
        chrono::ChCoordsys<>(_leftUpperLeg->GetPos(), chrono::QUNIT),
        chrono::ChCoordsys<>(bottomLeg->GetPos(), chrono::QUNIT));
    actuator1->SetName("actuator 1");
    actuator1->Set_lin_offset(0.00001);
    actuator1->GetLimit_D()->Set_active(true);
    actuator1->GetLimit_D()->Set_max(30);
    actuator1->GetLimit_D()->Set_min(2);

    actuator1->Set_dist_funct(_leftLegMuscle);

    _links->push_back(actuator1);

    auto foot = std::make_shared<chrono::ChBodyEasyBox>(8, 2, 2, // radius
                                                        1000,    // density
                                                        true, // collide enable?
                                                        true); // visualization?

    foot->SetMaterialSurface(mmaterial);
    foot->SetPos(chrono::ChVector<>(2, 3, -10));
    // upperLeg->SetRot(chrono::Q_from_AngX(-3.14159265359 / 6.0));
    foot->SetPos_dt(chrono::ChVector<>(0, 0, 0));
    foot->GetMaterialSurfaceNSC()->SetFriction(0.9f);
    foot->GetMaterialSurfaceNSC()->SetCompliance(0.0);
    foot->GetMaterialSurfaceNSC()->SetComplianceT(0.0);
    foot->GetMaterialSurfaceNSC()->SetDampingF(0.2f);
    foot->SetName("Foot");
    // foot->SetBodyFixed(true);

    _bodies->push_back(foot);

    auto footJoint = std::make_shared<chrono::ChLinkLockSpherical>();
    footJoint->Initialize(
        bottomLeg, foot, false,
        chrono::ChCoordsys<>(
            bottomLeg->GetPos() + chrono::ChVector<>(0, -11, 0), chrono::QUNIT),
        chrono::ChCoordsys<>(foot->GetPos() + chrono::ChVector<>(-2, 2, 0),
                             chrono::QUNIT));
    footJoint->SetName("Foot joint");
    footJoint->GetLimit_Rx()->Set_active(true);
    footJoint->GetLimit_Rx()->Set_max(0.01);
    footJoint->GetLimit_Rx()->Set_min(0.01);

    footJoint->GetLimit_Ry()->Set_active(true);
    footJoint->GetLimit_Ry()->Set_max(0.01);
    footJoint->GetLimit_Ry()->Set_min(0.01);

    footJoint->GetLimit_Rp()->Set_active(true);
    footJoint->GetLimit_Rp()->Set_max(0.01);
    footJoint->GetLimit_Rp()->Set_min(0.01);

    _links->push_back(footJoint);

    _leftFootMuscle = std::make_shared<chrono::ChFunction_Const>(10.0);
    auto actuator2 = std::make_shared<chrono::ChLinkLinActuator>();
    actuator2->Initialize(
        bottomLeg, foot, false,
        chrono::ChCoordsys<>(bottomLeg->GetPos() + chrono::ChVector<>(0, 0, 0),
                             chrono::QUNIT),
        chrono::ChCoordsys<>(foot->GetPos() + chrono::ChVector<>(+2, 2, 0),
                             chrono::QUNIT));
    actuator2->SetName("actuator 2");
    actuator2->Set_lin_offset(0.00001);
    actuator2->GetLimit_D()->Set_active(true);
    actuator2->GetLimit_D()->Set_max(30);
    actuator2->GetLimit_D()->Set_min(2);

    actuator2->Set_dist_funct(_leftFootMuscle);

    _links->push_back(actuator2);
}

void Actor::createRightLeg() {
    auto mmaterial = std::make_shared<chrono::ChMaterialSurfaceNSC>();
    mmaterial->SetFriction(0.5f);
    mmaterial->SetCompliance(0.0000005f);
    mmaterial->SetComplianceT(0.0000005f);
    mmaterial->SetDampingF(0.2f);

    _rightUpperLeg =
        std::make_shared<chrono::ChBodyEasyBox>(1, 20, 1, // radius
                                                800,      // density
                                                true,     // collide enable?
                                                true);    // visualization?
    // mrigidBall->SetMass(1.0);

    _rightUpperLeg->SetMaterialSurface(mmaterial);
    _rightUpperLeg->SetPos(chrono::ChVector<>(0, 37, 10));
    // upperLeg->SetRot(chrono::Q_from_AngX(-3.14159265359 / 6.0));
    _rightUpperLeg->SetPos_dt(chrono::ChVector<>(0, 0, 0));
    _rightUpperLeg->GetMaterialSurfaceNSC()->SetFriction(0.9f);
    _rightUpperLeg->GetMaterialSurfaceNSC()->SetCompliance(0.0);
    _rightUpperLeg->GetMaterialSurfaceNSC()->SetComplianceT(0.0);
    _rightUpperLeg->GetMaterialSurfaceNSC()->SetDampingF(0.2f);
    // mrigidBall1->SetBodyFixed(true);
    _rightUpperLeg->SetName("Upper leg");
    // upperLeg->SetBodyFixed(true);

    _bodies->push_back(_rightUpperLeg);

    auto bottomLeg =
        std::make_shared<chrono::ChBodyEasyBox>(1, 20, 1, // radius
                                                800,      // density
                                                true,     // collide enable?
                                                true);    // visualization?
    // mrigidBall->SetMass(1.0);

    bottomLeg->SetMaterialSurface(mmaterial);
    bottomLeg->SetPos(chrono::ChVector<>(0, 18, 10));
    // bottomLeg->SetRot(chrono::Q_from_AngX(3.14159265359 / 6.0));
    bottomLeg->SetPos_dt(chrono::ChVector<>(0, 0, 0));
    bottomLeg->GetMaterialSurfaceNSC()->SetFriction(0.9f);
    bottomLeg->GetMaterialSurfaceNSC()->SetCompliance(0.0);
    bottomLeg->GetMaterialSurfaceNSC()->SetComplianceT(0.0);
    bottomLeg->GetMaterialSurfaceNSC()->SetDampingF(0.2f);
    // mrigidBall1->SetBodyFixed(true);
    bottomLeg->SetName("Bottom leg");

    _bodies->push_back(bottomLeg);

    auto kneeJoint = std::make_shared<chrono::ChLinkLockSpherical>();
    kneeJoint->Initialize(
        _rightUpperLeg, bottomLeg, false,
        chrono::ChCoordsys<>(_rightUpperLeg->GetPos() +
                                 chrono::ChVector<>(0, -11, 0),
                             chrono::QUNIT),
        chrono::ChCoordsys<>(bottomLeg->GetPos() + chrono::ChVector<>(0, 11, 0),
                             chrono::QUNIT));
    kneeJoint->SetName("Knee joint");
    kneeJoint->GetLimit_Rx()->Set_active(true);
    kneeJoint->GetLimit_Rx()->Set_max(0.01);
    kneeJoint->GetLimit_Rx()->Set_min(0.01);

    kneeJoint->GetLimit_Ry()->Set_active(true);
    kneeJoint->GetLimit_Ry()->Set_max(0.01);
    kneeJoint->GetLimit_Ry()->Set_min(0.01);

    kneeJoint->GetLimit_Rp()->Set_active(true);
    kneeJoint->GetLimit_Rp()->Set_max(0.01);
    kneeJoint->GetLimit_Rp()->Set_min(0.01);

    _links->push_back(kneeJoint);

    _rightLegMuscle = std::make_shared<chrono::ChFunction_Const>(12.0);
    auto actuator1 = std::make_shared<chrono::ChLinkLinActuator>();
    actuator1->Initialize(
        _rightUpperLeg, bottomLeg, false,
        chrono::ChCoordsys<>(_rightUpperLeg->GetPos(), chrono::QUNIT),
        chrono::ChCoordsys<>(bottomLeg->GetPos(), chrono::QUNIT));
    actuator1->SetName("actuator 1");
    actuator1->Set_lin_offset(0.00001);
    actuator1->GetLimit_D()->Set_active(true);
    actuator1->GetLimit_D()->Set_max(30);
    actuator1->GetLimit_D()->Set_min(2);

    actuator1->Set_dist_funct(_rightLegMuscle);

    _links->push_back(actuator1);

    auto foot = std::make_shared<chrono::ChBodyEasyBox>(8, 2, 2, // radius
                                                        1000,    // density
                                                        true, // collide enable?
                                                        true); // visualization?

    foot->SetMaterialSurface(mmaterial);
    foot->SetPos(chrono::ChVector<>(2, 3, 10));
    // upperLeg->SetRot(chrono::Q_from_AngX(-3.14159265359 / 6.0));
    foot->SetPos_dt(chrono::ChVector<>(0, 0, 0));
    foot->GetMaterialSurfaceNSC()->SetFriction(0.9f);
    foot->GetMaterialSurfaceNSC()->SetCompliance(0.0);
    foot->GetMaterialSurfaceNSC()->SetComplianceT(0.0);
    foot->GetMaterialSurfaceNSC()->SetDampingF(0.2f);
    foot->SetName("Foot");
    // foot->SetBodyFixed(true);

    _bodies->push_back(foot);

    auto footJoint = std::make_shared<chrono::ChLinkLockSpherical>();
    footJoint->Initialize(
        bottomLeg, foot, false,
        chrono::ChCoordsys<>(
            bottomLeg->GetPos() + chrono::ChVector<>(0, -11, 0), chrono::QUNIT),
        chrono::ChCoordsys<>(foot->GetPos() + chrono::ChVector<>(-2, 2, 0),
                             chrono::QUNIT));
    footJoint->SetName("Foot joint");
    footJoint->GetLimit_Rx()->Set_active(true);
    footJoint->GetLimit_Rx()->Set_max(0.01);
    footJoint->GetLimit_Rx()->Set_min(0.01);

    footJoint->GetLimit_Ry()->Set_active(true);
    footJoint->GetLimit_Ry()->Set_max(0.01);
    footJoint->GetLimit_Ry()->Set_min(0.01);

    footJoint->GetLimit_Rp()->Set_active(true);
    footJoint->GetLimit_Rp()->Set_max(0.01);
    footJoint->GetLimit_Rp()->Set_min(0.01);

    _links->push_back(footJoint);

    _rightFootMuscle = std::make_shared<chrono::ChFunction_Const>(10.0);
    auto actuator2 = std::make_shared<chrono::ChLinkLinActuator>();
    actuator2->Initialize(
        bottomLeg, foot, false,
        chrono::ChCoordsys<>(bottomLeg->GetPos() + chrono::ChVector<>(0, 0, 0),
                             chrono::QUNIT),
        chrono::ChCoordsys<>(foot->GetPos() + chrono::ChVector<>(+2, 2, 0),
                             chrono::QUNIT));
    actuator2->SetName("actuator 2");
    actuator2->Set_lin_offset(0.00001);
    actuator2->GetLimit_D()->Set_active(true);
    actuator2->GetLimit_D()->Set_max(30);
    actuator2->GetLimit_D()->Set_min(2);

    actuator2->Set_dist_funct(_rightFootMuscle);

    _links->push_back(actuator2);
}

Actor::Actor()
    : _bodies(new std::vector<std::shared_ptr<chrono::ChBody>>()),
      _links(new std::vector<std::shared_ptr<chrono::ChLink>>()) {

    createLeftLeg();
    createRightLeg();

    auto mmaterial = std::make_shared<chrono::ChMaterialSurfaceNSC>();
    mmaterial->SetFriction(0.5f);
    mmaterial->SetCompliance(0.0000005f);
    mmaterial->SetComplianceT(0.0000005f);
    mmaterial->SetDampingF(0.2f);

    auto mainbody =
        std::make_shared<chrono::ChBodyEasyBox>(3, 20, 12, // radius
                                                1000,      // density
                                                true,      // collide enable?
                                                true);     // visualization?
    // mrigidBall->SetMass(1.0);

    mainbody->SetMaterialSurface(mmaterial);
    mainbody->SetPos(chrono::ChVector<>(0, 55, 0));
    // upperLeg->SetRot(chrono::Q_from_AngX(-3.14159265359 / 6.0));
    mainbody->SetPos_dt(chrono::ChVector<>(0, 0, 0));
    mainbody->GetMaterialSurfaceNSC()->SetFriction(0.9f);
    mainbody->GetMaterialSurfaceNSC()->SetCompliance(0.0);
    mainbody->GetMaterialSurfaceNSC()->SetComplianceT(0.0);
    mainbody->GetMaterialSurfaceNSC()->SetDampingF(0.2f);
    // mrigidBall1->SetBodyFixed(true);
    mainbody->SetName("Upper leg");
    // mainbody->SetBodyFixed(true);

    _bodies->push_back(mainbody);

    auto leftLegJoint = std::make_shared<chrono::ChLinkLockSpherical>();
    leftLegJoint->Initialize(
        _leftUpperLeg, mainbody, false,
        chrono::ChCoordsys<>(_leftUpperLeg->GetPos() +
                                 chrono::ChVector<>(0, 9, 1),
                             chrono::QUNIT),
        chrono::ChCoordsys<>(mainbody->GetPos() + chrono::ChVector<>(0, -9, -8),
                             chrono::QUNIT));
    leftLegJoint->SetName("LEft leg joint");
    leftLegJoint->GetLimit_Rx()->Set_active(true);
    leftLegJoint->GetLimit_Rx()->Set_max(0.01);
    leftLegJoint->GetLimit_Rx()->Set_min(0.01);

    leftLegJoint->GetLimit_Ry()->Set_active(true);
    leftLegJoint->GetLimit_Ry()->Set_max(0.01);
    leftLegJoint->GetLimit_Ry()->Set_min(0.01);

    leftLegJoint->GetLimit_Rp()->Set_active(true);
    leftLegJoint->GetLimit_Rp()->Set_max(0.01);
    leftLegJoint->GetLimit_Rp()->Set_min(0.01);
    _links->push_back(leftLegJoint);

    auto fun1 = std::make_shared<chrono::ChFunction_Const>(10.79);
    auto leftLegActuator = std::make_shared<chrono::ChLinkLinActuator>();
    leftLegActuator->Initialize(
        _leftUpperLeg, mainbody, false,

        chrono::ChCoordsys<>(_leftUpperLeg->GetPos() +
                                 chrono::ChVector<>(1, 6, 1),
                             chrono::QUNIT),
        chrono::ChCoordsys<>(mainbody->GetPos() + chrono::ChVector<>(3, 0, -7),
                             chrono::QUNIT));
    leftLegActuator->SetName("leftLegActuator");
    leftLegActuator->Set_lin_offset(0.00001);
    leftLegActuator->GetLimit_D()->Set_active(true);
    leftLegActuator->GetLimit_D()->Set_max(30);
    leftLegActuator->GetLimit_D()->Set_min(2);

    leftLegActuator->Set_dist_funct(fun1);

    _links->push_back(leftLegActuator);

    auto rightLegJoint = std::make_shared<chrono::ChLinkLockSpherical>();
    rightLegJoint->Initialize(
        _rightUpperLeg, mainbody, false,
        chrono::ChCoordsys<>(_rightUpperLeg->GetPos() +
                                 chrono::ChVector<>(0, 9, -1),
                             chrono::QUNIT),
        chrono::ChCoordsys<>(mainbody->GetPos() + chrono::ChVector<>(0, -9, 8),
                             chrono::QUNIT));
    rightLegJoint->SetName("LEft leg joint");
    rightLegJoint->GetLimit_Rx()->Set_active(true);
    rightLegJoint->GetLimit_Rx()->Set_max(0.01);
    rightLegJoint->GetLimit_Rx()->Set_min(0.01);

    rightLegJoint->GetLimit_Ry()->Set_active(true);
    rightLegJoint->GetLimit_Ry()->Set_max(0.01);
    rightLegJoint->GetLimit_Ry()->Set_min(0.01);

    rightLegJoint->GetLimit_Rp()->Set_active(true);
    rightLegJoint->GetLimit_Rp()->Set_max(0.01);
    rightLegJoint->GetLimit_Rp()->Set_min(0.01);
    _links->push_back(rightLegJoint);

    auto fun2 = std::make_shared<chrono::ChFunction_Const>(10.79);
    auto rightLegActuator = std::make_shared<chrono::ChLinkLinActuator>();
    rightLegActuator->Initialize(
        _rightUpperLeg, mainbody, false,

        chrono::ChCoordsys<>(_rightUpperLeg->GetPos() +
                                 chrono::ChVector<>(1, 6, -1),
                             chrono::QUNIT),
        chrono::ChCoordsys<>(mainbody->GetPos() + chrono::ChVector<>(3, 0, 7),
                             chrono::QUNIT));
    rightLegActuator->SetName("leftLegActuator");
    rightLegActuator->Set_lin_offset(0.00001);
    rightLegActuator->GetLimit_D()->Set_active(true);
    rightLegActuator->GetLimit_D()->Set_max(30);
    rightLegActuator->GetLimit_D()->Set_min(2);

    rightLegActuator->Set_dist_funct(fun2);

    _links->push_back(rightLegActuator);

    /*    auto leftLegJoint = std::make_shared<chrono::ChLinkLockSpherical>();
        leftLegJoint->Initialize(
            bottomLeg, foot, false,
            chrono::ChCoordsys<>(
                bottomLeg->GetPos() + chrono::ChVector<>(0, -11, 0),
       chrono::QUNIT),
            chrono::ChCoordsys<>(foot->GetPos() + chrono::ChVector<>(-2, 2, 0),
                                 chrono::QUNIT));
        leftLegJoint->SetName("Foot joint");*/
    /*  leftLegJoint->GetLimit_Rx()->Set_active(true);
      leftLegJoint->GetLimit_Rx()->Set_max(0.0);
      leftLegJoint->GetLimit_Rx()->Set_min(0.0);

      leftLegJoint->GetLimit_Ry()->Set_active(true);
      leftLegJoint->GetLimit_Ry()->Set_max(0.0);
      leftLegJoint->GetLimit_Ry()->Set_min(0.0);

      leftLegJoint->GetLimit_Rp()->Set_active(true);
      leftLegJoint->GetLimit_Rp()->Set_max(0.0);
      leftLegJoint->GetLimit_Rp()->Set_min(0.0);*/

    //  _links->push_back(leftLegJoint);

    /*
        auto actuator_fun2 = std::make_shared<chrono::ChFunction_Const>(16.0);
        auto actuator2 = std::make_shared<chrono::ChLinkLinActuator>();
        actuator2->Initialize(
            upperLeg, bottomLeg, false,
            chrono::ChCoordsys<>(upperLeg->GetPos() + chrono::ChVector<>(0, -11,
       0),
                                 chrono::QUNIT),
            chrono::ChCoordsys<>(bottomLeg->GetPos() + chrono::ChVector<>(0, -3,
       0),
                                 chrono::QUNIT));
        actuator2->SetName("actuator 2");
        actuator2->Set_lin_offset(0.00001);
        actuator2->Set_dist_funct(actuator_fun2);

        _links->push_back(actuator2);*/

    /*  auto mmaterial = std::make_shared<chrono::ChMaterialSurfaceNSC>();
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

      auto mrigid2 =
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
      _bodies->push_back(mrigid2);
      */
}

void Actor::simulateTime(double time) {

    _leftLegMuscle->Set_yconst(17.5 + ((std::sin(time) + 1.0) * 0.4));
    _leftFootMuscle->Set_yconst(8.9 + ((std::sin(time) + 1.0) * 0.2));

    _rightLegMuscle->Set_yconst(17.5 + ((std::sin(time) + 1.0) * 0.4));
    _rightFootMuscle->Set_yconst(8.9 + ((std::sin(time) + 1.0) * 0.2));
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
