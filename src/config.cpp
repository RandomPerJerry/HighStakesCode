#include "config.hpp"

namespace robot {
    // Controllers
    pros::Controller masterController(pros::E_CONTROLLER_MASTER);
    pros::Controller partnerController(pros::E_CONTROLLER_PARTNER);

    namespace drivetrain {
        // Drive Train Motors
        pros::MotorGroup leftMotors({-18, -19, -20}, pros::MotorGearset::blue); 
        pros::MotorGroup rightMotors({17, 14, 15}, pros::MotorGearset::blue); 

        // Sensors
        pros::adi::Encoder verticalEncoder('E', 'F');
        pros::Imu imu(16);

        // Tracking wheel setup
        lemlib::TrackingWheel verticalTrackingWheel(
            &verticalEncoder,
            lemlib::Omniwheel::NEW_275, 
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
            3,  // kD
            3,  // anti-windup
            1,  // small error range
            100,// small error timeout
            3,  // large error range
            500,// large error timeout
            20  // slew rate
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

        // Chassis instance
        lemlib::Chassis chassis(
            drivetrain,
            lateralController,
            angularController,
            sensors
        );
    }

    namespace mechanisms {
        pros::MotorGroup lbMotors({11, -20}, pros::MotorGearset::green);
        pros::Motor intakeMotor(1, pros::MotorGearset::blue);

        // Digital I/O
        pros::ADIDigitalIn lbLimitSwitch('H');

        // Digital Out
        pros::ADIDigitalOut hang('D');
        pros::ADIDigitalOut clamp('G');
        pros::ADIDigitalOut doinker('C', true);
    }
}