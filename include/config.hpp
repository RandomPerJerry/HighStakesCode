// config.hpp
#include "pros/apix.h"
#include "lemlib/api.hpp"

#ifndef CONFIG_HPP
#define CONFIG_HPP

namespace robot {
    // Controller declarations
    extern pros::Controller masterController;
    extern pros::Controller partnerController;

    namespace drivetrain {
        extern lemlib::Chassis chassis;
    }

    namespace mechanisms {
        extern pros::MotorGroup lbMotors;
        extern pros::Motor intakeMotor;
        
        // Pneumatics/Digital outputs
        extern pros::ADIDigitalOut hang;
        extern pros::ADIDigitalOut clamp;
        extern pros::ADIDigitalOut doinker;
        extern pros::ADIDigitalOut intake;
        // Sensors/Digital inputs
        extern pros::ADIDigitalIn lbLimitSwitch;
        extern pros::Optical opticalSensor;
    }

    // Constants 
    namespace constants {
        constexpr int INTAKE_SPEED = 600;
        constexpr int LOOP_DELAY = 25;
    }
}

#endif 

