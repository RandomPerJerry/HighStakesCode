#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/pid.hpp"
#include "timer.hpp"
#include "config.hpp"
#include "auto.h"
#include <cstdint>
#include <limits>
#include <utility>
#include <typeinfo>

/*
    L2: LB down
    R2: LB up
    R1: INTAKE
    L1: OUTTAKE
    RIGHT: LB Toggle
    DOWN: Doinker
    LEFT: Hang
    UP: Intake Toggle
    B: Clamp
    Y: REVERSE DRIVE
    X: BRAKE MODE
*/

namespace controls {
    class Mechanisms {
    private:
        enum class LBToggleState {
            IDLE,
            INTAKE,
            CLEAR
        }; 

        static constexpr double LB_POSITIONS[] = {
            0.0,    // IDLE
            4800.0,  // INTAKE    
            18000.0   // CLEAR
        };

        struct intakeState {
            inline static bool targetColor = (current_auto == AutonomousMode::BLUE_RING ||
            current_auto == AutonomousMode::BLUE_STAKE); // false = red team (eject blue), true = blue team (eject red)
            inline static std::pair<double, Timer> pendingEjectTimers[2] = {{0, Timer(200)}, {0, Timer(200)}};
            inline static double lastSeenUnits = 0.0;
            inline static bool isEjecting = false;
        };

        static constexpr bool ENABLE_COLOR_SORT = true;
        static constexpr double EJECT_COOLDOWN_UNIT = 660.0;
        static constexpr double EJECT_EJECTING_INTERVAL_UNIT = 400.0;
        static constexpr double EJECT_DELAY_UNIT = 510.0;

        static constexpr int LB_POSITION_LOSS_BOUNDARY = 6000;
        static constexpr int LB_FINETUNE_BOUNDARY = 5200;
        static constexpr double MIN_VELOCITY = 50.0;  
        static constexpr double SLEW_RATE = 100.0;

        static inline Timer rollBackTimer = Timer(100);



        static double slewMove(double targetVelocity, double currentVelocity) {
            if (targetVelocity == currentVelocity) {
                return targetVelocity;
            }
            if (targetVelocity == 0.0) {
                // Slowing down
                if (std::abs(currentVelocity) < MIN_VELOCITY) {
                    return 0.0;
                }
                if (currentVelocity > 0.0) {
                    return currentVelocity - SLEW_RATE;
                }
                if (currentVelocity < 0.0) {
                    return currentVelocity + SLEW_RATE;
                }
            }
            // Speeding up or maintaining speed
            if (currentVelocity < targetVelocity) {
                return std::min(currentVelocity + SLEW_RATE, targetVelocity);
            }
            if (currentVelocity > targetVelocity) {
                return std::max(currentVelocity - SLEW_RATE, targetVelocity);
            }
                
            return targetVelocity;
        }
    public:     
        static void update_LB(){ 
            // Static Variables
            static LBToggleState lbState = LBToggleState::IDLE;
            static bool isAutoMoving = false;
            static bool isOutOfBounds = false;

            // Position Variables
            double currentPosition = robot::mechanisms::lbRotationSensor.get_position();

            // Manual Speed Variables
            int manualSpeed = (currentPosition < LB_FINETUNE_BOUNDARY) ? 20 : 100;

            if (currentPosition < 200) {
                robot::mechanisms::lbMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                if (currentPosition < 100) {
                    robot::mechanisms::lbRotationSensor.reset_position();
                }
            } else {
                robot::mechanisms::lbMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            }

            // Manual Movement
            if (robot::masterController.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                robot::mechanisms::lbMotor.move_velocity(-manualSpeed);
                isOutOfBounds = true;
                isAutoMoving = false;

            } else if (robot::masterController.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                robot::mechanisms::lbMotor.move_velocity(manualSpeed);
                isOutOfBounds = true;
                isAutoMoving = false;

            } else if (!isAutoMoving) {
                double currentVelocity = robot::mechanisms::lbMotor.get_actual_velocity();
                currentVelocity = slewMove(0, currentVelocity);
                robot::mechanisms::lbMotor.move_velocity(currentVelocity);
            }

            // Toggle Auto Movement
            if (robot::masterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
                isAutoMoving = true;
                robot::pid::lbPID.reset();

                // After user interrupts
                if (isOutOfBounds) {
                    isOutOfBounds = false;
                    if (currentPosition < LB_POSITION_LOSS_BOUNDARY) {
                        lbState = LBToggleState::INTAKE;
                    } else {
                        lbState = LBToggleState::IDLE;
                    }

                } else {
                    switch (lbState) {
                    case LBToggleState::IDLE:
                        lbState = LBToggleState::INTAKE;
                        break;
                    case LBToggleState::INTAKE:
                        rollBackTimer.start();
                        lbState = LBToggleState::CLEAR;
                        break;
                    case LBToggleState::CLEAR:
                        lbState = LBToggleState::IDLE;
                        break;
                    }
                }
            }

            // Auto Movement 
            if (isAutoMoving) {
                double targetPosition = LB_POSITIONS[static_cast<int>(lbState)];
                double error = targetPosition - currentPosition;

                if (rollBackTimer.isTimerRunning()) {
                    robot::mechanisms::intakeMotor.move_velocity(600);
                } else {
                    rollBackTimer.reset();
                    robot::mechanisms::intakeMotor.move_velocity(0);
                }

                // Get PID output
                double pidOutput = robot::pid::lbPID.update(error);
                double velocityCommand = std::clamp(pidOutput, -100.0, 100.0);
                
                // Apply the output
                robot::mechanisms::lbMotor.move_velocity(velocityCommand);
                
                if (std::abs(error) < 100) {  
                    isAutoMoving = false;
                    robot::mechanisms::lbMotor.move_velocity(0);
                    if (lbState == LBToggleState::IDLE) {
                        robot::mechanisms::lbRotationSensor.reset_position();
                    }
                }
            }
        }

        static void drive(){
            static bool reverseDrive = false;
            static bool brakeMode = false;


            int x = robot::masterController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            int y = robot::masterController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            if (robot::masterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
                reverseDrive = !reverseDrive;
            }
            if (reverseDrive) {
                x = -x;
            }
            if (robot::masterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
                brakeMode = !brakeMode;
                if (brakeMode) {
                    pros::delay(500);
                    robot::drivetrain::chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
                } else {
                    pros::delay(500);
                    robot::drivetrain::chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
                }
            }  
            robot::drivetrain::chassis.arcade(x, y);
        }
        
        static void update_hang() {
            
            static bool hangState = false;
            if (robot::masterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
                hangState = !hangState;
                robot::mechanisms::hang.set_value(hangState);
            }
        }

        static void update_intake() {
            static bool intakeToggle = false;
            if (robot::masterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
                intakeToggle = !intakeToggle;
            }

            const int intake_speed = robot::constants::INTAKE_SPEED;
            if (robot::masterController.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { 
                if (ENABLE_COLOR_SORT) {

                    // pros::lcd::print(1, "First: (%f, %f)", intakeState::pendingEjectTimers[0].first, intakeState::pendingEjectTimers[0].second.getTimeRemaining());
                    // pros::lcd::print(2, "Second: (%f, %f)", intakeState::pendingEjectTimers[1].first, intakeState::pendingEjectTimers[1].second.getTimeRemaining());
                    // pros::lcd::print(3, "Ejecting: %d", intakeState::isEjecting);
                    double current_revolution = std::abs(robot::mechanisms::intakeMotor.get_position());
                    if (intakeState::targetColor) { // Blue team rejecting red
                        if (robot::mechanisms::opticalSensor.get_hue() >= 0 && 
                            robot::mechanisms::opticalSensor.get_hue() <= 25 &&
                            current_revolution - intakeState::lastSeenUnits > EJECT_COOLDOWN_UNIT) {
                            for (int i = 0; i < 2; i++) {
                                if (intakeState::pendingEjectTimers[i].first == 0.0) {
                                    intakeState::pendingEjectTimers[i].first = current_revolution + EJECT_DELAY_UNIT;
                                    intakeState::lastSeenUnits = current_revolution;
                                    break;
                                }
                            }
                        }
                        
                    } else if (!intakeState::targetColor) {
                        if (robot::mechanisms::opticalSensor.get_hue() >= 100 && 
                            robot::mechanisms::opticalSensor.get_hue() <= 220 &&
                            current_revolution - intakeState::lastSeenUnits > EJECT_COOLDOWN_UNIT) {
                            for (int i = 0; i < 2; i++) {
                                if (intakeState::pendingEjectTimers[i].first == 0.0) {
                                    intakeState::pendingEjectTimers[i].first = current_revolution + EJECT_DELAY_UNIT;
                                    intakeState::lastSeenUnits = current_revolution;
                                    break;
                                }
                            }
                        }
                    }

                    for (int i = 0; i < 2; i++) {
                        if (intakeState::pendingEjectTimers[i].first != 0.0) {

                            if (intakeState::pendingEjectTimers[i].first <= current_revolution && 
                                intakeState::pendingEjectTimers[i].second.isTimerRunning()) {

                                intakeState::pendingEjectTimers[i].second.start();
                                intakeState::isEjecting = true;
                                break;
                            } else if (intakeState::pendingEjectTimers[i].second.isDone()) {
                                intakeState::pendingEjectTimers[i].first = 0.0;
                                intakeState::pendingEjectTimers[i].second.reset();
                                intakeState::isEjecting = false;
                            }
                        }
                    }

                    if (intakeState::isEjecting) {
                        robot::mechanisms::intakeMotor.move_velocity(0);
                    } else {
                        robot::mechanisms::intakeMotor.move_velocity(-intake_speed);
                    }
                } else {
                    robot::mechanisms::intakeMotor.move_velocity(-intake_speed);
                }
            } else if (robot::masterController.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                robot::mechanisms::intakeMotor.move_velocity(intake_speed);
            } else if (intakeToggle) {
                robot::mechanisms::intakeMotor.move_velocity(-intake_speed);
            } else {    
                robot::mechanisms::intakeMotor.move_velocity(0);
            }
        }

        static void update_clamp() {
            static bool clampState = false;
            if (robot::masterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
                clampState = !clampState;
                robot::mechanisms::clamp.set_value(clampState);
            }
        }

        static void update_doinker() {
            static bool doinkerState = false;
            if (robot::masterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
                doinkerState = !doinkerState;
                robot::mechanisms::doinker.set_value(doinkerState);
            }
        }


    }; 
} 

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    
    robot::drivetrain::chassis.calibrate(); // calibrate sensors
    robot::mechanisms::lbMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    robot::drivetrain::chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    robot::mechanisms::lbRotationSensor.reset_position();

    robot::mechanisms::opticalSensor.set_led_pwm(100);
    // print position to brain screen
    pros::Task screen_task([&]() { 
        while (true) {

            // // Debugging Printing Area
            // pros::lcd::print(0, "Chassis Position: x: %f", robot::drivetrain::chassis.getPose().x);
            // pros::lcd::print(1, "Chassis Position: y: %f", robot::drivetrain::chassis.getPose().y);
            // pros::lcd::print(2, "Chassis Position: heading : %f", robot::drivetrain::chassis.getPose().theta);
            // pros::lcd::print(3, "Intake Motor: %f", robot::mechanisms::intakeMotor.get_actual_velocity());
            pros::lcd::print(0, "Encoder Units: %f", std::abs(robot::mechanisms::intakeMotor.get_position()));
            pros::lcd::print(3, "Hue: %f", robot::mechanisms::opticalSensor.get_hue());

            pros::delay(robot::constants::LOOP_DELAY);
        }   
    });
}


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}  

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    robot::mechanisms::doinker.set_value(false);
    Timer allianceTimer = Timer(1000);
    allianceTimer.start();


    while (true) {

        controls::Mechanisms::drive();
        controls::Mechanisms::update_hang();
        controls::Mechanisms::update_intake();      
        controls::Mechanisms::update_clamp();
        controls::Mechanisms::update_LB(); // Needs to be after update_intake
        controls::Mechanisms::update_doinker();

        pros::delay(robot::constants::LOOP_DELAY);
    }
}