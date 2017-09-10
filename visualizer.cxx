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
    sim.init();
    auto ptr = std::shared_ptr<Actor>(new Actor());

    // sim.testActor(ptr);

    chrono::ChSystemNSC* chSystem = sim.physicalSystem();
    assert(chSystem);
    const auto bodies = ptr->getBodies();
    for (auto body : *bodies) {
        auto ball_texture = std::make_shared<chrono::ChTexture>(
            GetChronoDataFile("pinkwhite.png"));
        chSystem->Add(body);
        body->AddAsset(ball_texture);
    }

    const auto links = ptr->getLinks();

    for (auto linkPtr : (*links)) {
        chSystem->AddLink(linkPtr);
    }

    ChIrrApp application(chSystem, L"Simulation",
                         core::dimension2d<u32>(1024, 728), false);

    chrono::irrlicht::ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(),
                                    core::vector3df(-50, 80, -50));

    application.AssetBindAll();
    application.AssetUpdateAll();

    chSystem->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_PROJECTED);

    application.SetStepManage(true);
    application.SetTimestep(0.05);
    application.SetTryRealtime(true);

    double time = 0.0;
    while (application.GetDevice()->run()) {
        application.BeginScene(true, true, SColor(255, 140, 161, 192));
        application.DrawAll();

        for (int i = 1; i < 3; ++i) {
            ChIrrTools::drawCircle(
                application.GetVideoDriver(), 15.0 * i,
                ChCoordsys<>(ChVector<>(0, 0, 0),
                             Q_from_AngX(3.14159265359 / 2.0)),
                video::SColor(0, 0, 255, 124), 50, true);
            // ChIrrTools::drawPolyline()
        }

        /*ChIrrTools::drawAllLinks(
            *chSystem, application.GetVideoDriver(), 1.0,
            chrono::irrlicht::ChIrrTools::eCh_LinkDrawMode::LINK_REACT_FORCE);*/

        /*   ChIrrTools::drawAllBoundingBoxes(*chSystem,
                                            application.GetVideoDriver());*/

        ChIrrTools::drawAllLinkframes(*chSystem, application.GetVideoDriver(),
                                      10.0);

        const auto& lines = ptr->getLines();
        for (const auto& line : lines) {
            chrono::ChVector<> vect1 = line[0];
            chrono::ChVector<> vect2 = line[1];
            ChIrrTools::drawSegment(application.GetVideoDriver(), vect1, vect2,
                                    SColor(255, 255, 255, 192), false);
        }

        time += 0.05;
        application.DoStep();
        ptr->simulateTime(time);
        application.EndScene();
    }
}
