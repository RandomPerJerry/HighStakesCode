#include "config.hpp"

namespace robot {
    // Controllers
    pros::Controller masterController(pros::E_CONTROLLER_MASTER);
    pros::Controller partnerController(pros::E_CONTROLLER_PARTNER);

    namespace drivetrain {
        // Drive Train Motors
        pros::MotorGroup leftMotors({-18, -20, 19}, pros::MotorGearset::blue); // -18, -20, 19
        pros::MotorGroup rightMotors({12, -13, 14}, pros::MotorGearset::blue); // 12, -13, 14

        // Sensors
        pros::Rotation verticalRotation(-1);
        pros::Imu imu(17);

        // Tracking wheel setup
        lemlib::TrackingWheel verticalTrackingWheel(
            &verticalRotation,
            lemlib::Omniwheel::NEW_2, 
            0.0
        ); 

        // Drivetrain configuration
        lemlib::Drivetrain drivetrain( 
            &leftMotors,
            &rightMotors,
            11.4,
            lemlib::Omniwheel::NEW_325,
            450,
            2
        );

        // Odometry sensors
        lemlib::OdomSensors sensors(
            &verticalTrackingWheel,
            nullptr,
            nullptr,
            nullptr,
            &imu
        );

        // PID Controllers
        lemlib::ControllerSettings lateralController(
            10, // kP
            0,  // kI
            7,  // kD
            3, // anti windup
            1, // small error range, in inches
            100, // small error range timeout, in milliseconds
            3, // large error range, in inches
            500, // large error range timeout, in milliseconds
            20 // maximum acceleration (slew)
        );
        
        lemlib::ControllerSettings angularController(
            2,  // kP
            0,  // kI
            10, // kD
            3,  // anti-windup
            1,  // small error range
            100,// small error timeout
            3,  // large error range
            500,// large error timeout
            0   // slew rate    
        );  

        lemlib::ExpoDriveCurve throttleCurve(
            10, // Joystick deadband
            20, // Minimum output
            0.995 // Curve gain
        );

        lemlib::ExpoDriveCurve turnCurve(
            10, // Joystick deadband
            20, // Minimum output
            0.995 // Curve gain
        );

        // Chassis instance
        lemlib::Chassis chassis(
            drivetrain,
            lateralController,
            angularController,
            sensors,
            &throttleCurve,
            &turnCurve
        );
    }

    namespace mechanisms {
        pros::Motor lbMotor(10, pros::MotorGearset::red);
        pros::Motor intakeMotor(9, pros::MotorGearset::blue);
   
        // Digital I/O
        pros::Rotation lbRotationSensor (15);
        pros::Optical opticalSensor(8);

        // Digital Out
        pros::ADIDigitalOut hang('F');
        pros::ADIDigitalOut clamp('H');
        pros::ADIDigitalOut doinker('G');
        pros::ADIDigitalOut intake('B');
    } 
    namespace pid {
        // PID gains account for conversion from centidegrees to RPM
        // Kp units: RPM/centidegree
        // Ki units: RPM/(centidegree*second)
        // Kd units: RPM/(centidegree/second)

        lemlib::PID lbPID (
            0.01,    // Kp
            0.0,    // Ki
            0.0,    // Kd
            0,      // windup range
            false   // sign flip reset
        );
    }       
}
