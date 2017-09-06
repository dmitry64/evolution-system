#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_irrlicht/ChBodySceneNode.h"
#include "chrono_irrlicht/ChBodySceneNodeTools.h"
#include "chrono_irrlicht/ChIrrApp.h"

#include <irrlicht.h>

#include "actor.hpp"
#include "simulationsystem.hpp"

using namespace chrono;
using namespace chrono::irrlicht;

using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

int main(int argc, char* argv[]) {
    SimulationSystem system;
    system.init();
    system.runSimulation();
    system.waitForSimulation();

    Simulator sim;
    sim.init();
    auto ptr = std::shared_ptr<Actor>(new Actor());
    sim.testActor(ptr);
    chrono::ChSystemNSC* chSystem = sim.physicalSystem();

    ChIrrApp application(chSystem, L"Simulation",
                         core::dimension2d<u32>(800, 600), false);

    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(),
                                    core::vector3df(12, 15, -20));

    application.AssetBindAll();
    application.AssetUpdateAll();

    chSystem->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_PROJECTED);

    application.SetStepManage(true);
    application.SetTimestep(0.01);
    application.SetTryRealtime(true);

    while (application.GetDevice()->run()) {
        application.BeginScene(true, true, SColor(255, 140, 161, 192));
        application.DrawAll();

        for (int i = 1; i < 10; ++i) {
            ChIrrTools::drawCircle(application.GetVideoDriver(), 10.0 * i,
                                   ChCoordsys<>(ChVector<>(0, 0.08, 0),
                                                Q_from_AngX(3.14159265359 / 2)),
                                   video::SColor(255, 0, 255, 0), 36, true);
        }

        application.DoStep();
        application.EndScene();
    }

    return 0;
}
