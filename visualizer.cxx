#include "visualizer.hpp"

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_irrlicht/ChBodySceneNode.h"
#include "chrono_irrlicht/ChBodySceneNodeTools.h"
#include "chrono_irrlicht/ChIrrApp.h"

#include <irrlicht.h>

#include "actor.hpp"
#include "simulator.hpp"

using namespace chrono;
using namespace chrono::irrlicht;

using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

Visualizer::Visualizer() {}

void Visualizer::show() {
    Simulator sim;
    auto ptr = std::shared_ptr<Actor>(new Actor());
    std::cout << "V3" << std::endl;
    sim.testActor(ptr);
    std::cout << "V4" << std::endl;
    chrono::ChSystemNSC* chSystem = sim.physicalSystem();

    ChIrrApp application(chSystem, L"Simulation",
                         core::dimension2d<u32>(800, 600), false);

    chrono::irrlicht::ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(),
                                    core::vector3df(12, 15, -20));

    application.AssetBindAll();
    application.AssetUpdateAll();

    chSystem->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_PROJECTED);

    application.SetStepManage(true);
    application.SetTimestep(0.001);
    application.SetTryRealtime(true);

    double time = 0.0;
    while (application.GetDevice()->run()) {
        application.BeginScene(true, true, SColor(255, 140, 161, 192));
        application.DrawAll();

        for (int i = 1; i < 10; ++i) {
            ChIrrTools::drawCircle(application.GetVideoDriver(), 10.0 * i,
                                   ChCoordsys<>(ChVector<>(0, 0.08, 0),
                                                Q_from_AngX(3.14159265359 / 2)),
                                   video::SColor(255, 0, 255, 0), 36, true);
        }

        time += 0.001;
        application.DoStep();
        ptr->simulateTime(time);
        application.EndScene();
    }
}
