#include <Core/Core.h>

using namespace Upp;

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkTSDA.h"
#include <iostream>
#include <fstream>

using namespace chrono;

	
void SpringDamper() {
    ChSystemNSC system;

	system.SetGravitationalAcceleration(ChVector3<>(0, -9.81, 0));

    // Create the fixed body (anchor)
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    system.Add(ground);

    // Create the dynamic body (mass)
    auto mass = chrono_types::make_shared<ChBodyEasySphere>(0.1, 1000, false, false);
    mass->SetPos(ChVector3<>(0, -1, 0));
    system.Add(mass);

    // Create the spring-damper
    auto spring = chrono_types::make_shared<ChLinkTSDA>();
    spring->Initialize(ground, mass, false,
                       ChVector3<>(0, 0, 0),        // Ground attachment
                       ChVector3<>(0, 0.1, 0));     // Mass attachment (slightly above center)
    spring->SetSpringCoefficient(100);  // Spring stiffness [N/m]
    spring->SetDampingCoefficient(1);   // Damping [N⋅s/m]
    spring->SetRestLength(1.0);         // Rest length [m]
    system.AddLink(spring);

    // Simulation parameters
    double time_step = 0.001;
    double end_time = 5.0;

    std::ofstream log("spring_mass_output.csv");
    log << "time,pos_y,vel_y\n";

    for (double t = 0; t < end_time; t += time_step) {
        system.DoStepDynamics(time_step);

        double pos_y = mass->GetPos().y();
        double vel_y = mass->GetPosDt().y();
        log << t << "," << pos_y << "," << vel_y << "\n";
    }

    log.close();
    std::cout << "\nSimulation complete. Output saved to spring_mass_output.csv\n";
}

void demo_FEA_basic();
void demo_CH_powertrain();
void demo_CH_linalg();
void demo_CH_coords();

CONSOLE_APP_MAIN
{
	Cout() << "Project::Chrono demos\n";

	Cout() << "\nSpring-Damper demo";
	SpringDamper();

	Cout() << "\ndemo_FEA_basic";
	Cout() << "\nTutorial that teaches how to use the FEA module to create basic FEA elements and nodes, performing simple static analysis.";
	MemoryIgnoreLeaksBegin();	
	demo_FEA_basic();
	MemoryIgnoreLeaksEnd();	
	
	Cout() << "\ndemo_CH_powertrain";
	Cout() << "\nTutorial that teaches how to use 1D elements, clutches, engines, gears and their interface with 3D bodies";
	demo_CH_powertrain();

	Cout() << "\ndemo_CH_linalg";
	Cout() << "\nTutorial that teaches how to do operations on matrices and vectors";
	demo_CH_linalg();
	
	Cout() << "\ndemo_CH_coords";
	Cout() << "\nDemo on how to use Chrono coordinate transformations";
	demo_CH_coords();

	Cout() << "\nPress Enter to end";
	ReadStdIn();
}

