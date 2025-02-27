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
        extern pros::Motor lbMotor;
        extern pros::Motor intakeMotor;
        
        // Pneumatics/Digital outputs
        extern pros::ADIDigitalOut hang;
        extern pros::ADIDigitalOut clamp;
        extern pros::ADIDigitalOut doinker;
        extern pros::ADIDigitalOut intake;

        // Sensors/Digital inputs
        extern pros::Rotation lbRotationSensor;
        extern pros::Optical opticalSensor;
        extern pros::Distance distanceSensor;
    }

    namespace pid {
        extern lemlib::PID lbPID;
    }

    // Constants 
    namespace constants {
        constexpr int INTAKE_SPEED = 600;
        constexpr int LOOP_DELAY = 25;
    }

    namespace lb {
        // LB States
        enum class LBToggleState {
            IDLE,
            INTAKE,
            CLEAR
        };

        // LB Constants
        constexpr double LB_POSITIONS[] = {
            0.0,    // IDLE
            4200.0, // INTAKE
            6000.0  // CLEAR
        };
    }
}

#endif 

