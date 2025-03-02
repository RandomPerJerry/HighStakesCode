#include "main.h"
#include "config.hpp"
#include "auto.h"
#include "lemlib/timer.hpp"
  
// Current autonomous selection
AutonomousMode current_auto = AutonomousMode::BLUE_STAKE;

namespace autosetting {
    struct IntakeState {
        // Ring Eject States
        static bool targetColor;
        static bool isEjecting;   
        static bool ringDetected;
        static uint32_t ejectStartTime;
        static uint32_t ringEjectCooldown;
        static uint32_t ringDetectedTime;

        // Run Intake States
        static bool shouldRun;
        static uint32_t startTime;
        static uint32_t duration;
        static uint32_t runSpeed;
    };

    struct LBState {
        static double targetPosition;
        static double runSpeed;
        static bool isRunning;
    };

    // Tunable constants
    constexpr bool ENABLE_COLOR_SORT = false;  // Set to true to enable color sorting/ejection
    constexpr int EJECT_TIME = 500;   // Time to stop intake for ejection (ms)
    constexpr int RING_TRAVEL_TIME = 200; // Time for ring to reach top after detection (ms)
    constexpr int RING_EJECT_COOLDOWN = 1000; // Time to wait before detecting another ring (ms)

    // Ring eject state variables
    bool IntakeState::targetColor = (current_auto == AutonomousMode::BLUE_RING ||
    current_auto == AutonomousMode::BLUE_STAKE); // false = red team (eject blue), true = blue team (eject red)
    bool IntakeState::isEjecting = false;
    uint32_t IntakeState::ejectStartTime = 0;
    uint32_t IntakeState::ringDetectedTime = 0;
    uint32_t IntakeState::ringEjectCooldown = 0;
    bool IntakeState::ringDetected = false;
    uint32_t IntakeState::runSpeed = robot::constants::INTAKE_SPEED;

    bool IntakeState::shouldRun = false;
    uint32_t IntakeState::startTime = 0;
    uint32_t IntakeState::duration = 0;

    // LB state variables
    double LBState::targetPosition = 0;
    double LBState::runSpeed = 100.0;
    bool LBState::isRunning = false;

    void intake_task_fn(void* param) {
        while (pros::competition::is_autonomous()) {
            uint32_t currentTime = pros::millis();
            
            if (IntakeState::shouldRun && 
                (currentTime - IntakeState::startTime < IntakeState::duration)) {
                
                if (ENABLE_COLOR_SORT) {
                    if (IntakeState::isEjecting) {
                        if (currentTime - IntakeState::ejectStartTime < EJECT_TIME) {
                            robot::mechanisms::intakeMotor.move_velocity(0);
                        } else {
                            IntakeState::isEjecting = false;
                            IntakeState::ringDetected = false;
                        }
                    } else if (IntakeState::ringDetected) {
                        if (currentTime - IntakeState::ringDetectedTime >= RING_TRAVEL_TIME) {
                            IntakeState::isEjecting = true;            
                            IntakeState::ejectStartTime = currentTime;
                            IntakeState::ringDetected = false;
                        }
                        robot::mechanisms::intakeMotor.move_velocity(-IntakeState::runSpeed);
                    } else {
                        robot::mechanisms::intakeMotor.move_velocity(-IntakeState::runSpeed);

                        if (IntakeState::targetColor) { 
                            if (robot::mechanisms::opticalSensor.get_hue() >= 0 && 
                                robot::mechanisms::opticalSensor.get_hue() <= 25 &&
                                currentTime >= IntakeState::ringEjectCooldown) {
                                IntakeState::ringDetected = true;
                                IntakeState::ringDetectedTime = currentTime;
                                IntakeState::ringEjectCooldown = currentTime + RING_EJECT_COOLDOWN;
                            }
                        } else { 
                            if (robot::mechanisms::opticalSensor.get_hue() >= 100 && 
                                robot::mechanisms::opticalSensor.get_hue() <= 220 &&
                                currentTime >= IntakeState::ringEjectCooldown) {
                                IntakeState::ringDetected = true;
                                IntakeState::ringDetectedTime = currentTime;
                                IntakeState::ringEjectCooldown = currentTime + RING_EJECT_COOLDOWN;
                            }
                        }
                    }
                } else {
                    robot::mechanisms::intakeMotor.move_velocity(-IntakeState::runSpeed);
                }
            } else {
                robot::mechanisms::intakeMotor.move_velocity(0);
                IntakeState::shouldRun = false;
                IntakeState::isEjecting = false;
                IntakeState::ringDetected = false;
                IntakeState::ringEjectCooldown = 0;
                IntakeState::runSpeed = robot::constants::INTAKE_SPEED;
            }
            pros::delay(10);
        } 
    }
  
    void run_intake(int runTime, uint32_t intakeSpeed = robot::constants::INTAKE_SPEED) {
        // Reset all intake state variables
        IntakeState::startTime = pros::millis();
        IntakeState::duration = runTime;
        IntakeState::shouldRun = true;
        IntakeState::isEjecting = false;
        IntakeState::ringDetected = false;
        IntakeState::ringEjectCooldown = 0;
        IntakeState::runSpeed = intakeSpeed;
    }

    void reset_intake() {
        IntakeState::startTime = 0;
        IntakeState::duration = 0;
        IntakeState::shouldRun = false;
        IntakeState::isEjecting = false;
        IntakeState::ringDetected = false;
        IntakeState::ringEjectCooldown = 0;
        IntakeState::runSpeed = robot::constants::INTAKE_SPEED;
    }

    void lb_task_fn(void* param) {
        while (pros::competition::is_autonomous()) {
            double currentPosition = robot::mechanisms::lbRotationSensor.get_position();
            double error = LBState::targetPosition - currentPosition;
            if (LBState::isRunning) {
                if (std::abs(error) < 200) {
                    LBState::isRunning = false;
                    robot::mechanisms::lbMotor.move_velocity(0);
                    if (LBState::targetPosition == 0) {
                        robot::mechanisms::lbRotationSensor.reset_position();
                    }
                } else {
                    double pidOutput = robot::pid::lbPID.update(error);
                    double velocityCommand = std::clamp(pidOutput, -LBState::runSpeed, LBState::runSpeed);
                    robot::mechanisms::lbMotor.move_velocity(velocityCommand);
                }
            }

            if (currentPosition <= 200) {
                robot::mechanisms::lbMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            } else {
                robot::mechanisms::lbMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            }
            pros::delay(10);
        }
    }

    void run_LB(double angle, double speed = 100.0) {
        LBState::targetPosition = angle;
        LBState::runSpeed = speed;
        LBState::isRunning = true;
        robot::pid::lbPID.reset();
    }

    bool is_LB_running() {
        return LBState::isRunning;
    }

    void wait_until_LB_done() {
        while (is_LB_running()) {
            pros::delay(10);
        }
    }

    void pickup_ring(float x, float y, float exitRange1, float exitRange2) {
        robot::drivetrain::chassis.moveToPoint(x, y, 1000, {.minSpeed = 127, .earlyExitRange = exitRange1});
        robot::drivetrain::chassis.moveToPoint(x, y, 1000, {.maxSpeed = 70, .earlyExitRange = exitRange2});
        robot::drivetrain::chassis.moveToPoint(x, y, 1000, {.maxSpeed = 120});
    }

    float distance_calculator(float x1, float y1, float x2 = robot::drivetrain::chassis.getPose().x, float y2 = robot::drivetrain::chassis.getPose().y) {
        return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    }

    void moveForward(float distance, float maxTime, float maxSpeed = 120) {
        float currentX = robot::drivetrain::chassis.getPose().x;
        float currentY = robot::drivetrain::chassis.getPose().y; 
        float currentHeading = robot::drivetrain::chassis.getPose().theta;
        robot::drivetrain::chassis.setPose(currentX, currentY, currentHeading);
        robot::drivetrain::chassis.moveToPoint(distance, 0, maxTime, {.maxSpeed = maxSpeed});
        robot::drivetrain::chassis.waitUntilDone();
        robot::drivetrain::chassis.setPose(currentX + distance, currentY, currentHeading);
    }
}

/*
          0
     270     90
         180
*/
ASSET(Skill1_txt)
ASSET(Skill2_txt)
void skills_auto() {

    float WS1x = 5.9;
    float WS2x = 3;
    try {
        robot::drivetrain::chassis.setPose(-60, 0, 90);
        autosetting::run_intake(1000);
        pros::delay(500);
        robot::drivetrain::chassis.moveToPoint(-54, 0, 1000);
        robot::drivetrain::chassis.turnToPoint(-19.039, -25.238, 500);
        robot::drivetrain::chassis.moveToPoint(-19.039, -25.238, 1500, {.minSpeed = 127, .earlyExitRange = 30});
        robot::drivetrain::chassis.moveToPoint(-19.039, -25.238, 800, {.maxSpeed = 70});
        robot::drivetrain::chassis.waitUntil(autosetting::distance_calculator(-21.37, -24.461) - 10); // wait until 10 inches away
        autosetting::run_intake(800);
        robot::drivetrain::chassis.turnToHeading(90, 700);
        robot::drivetrain::chassis.moveToPoint(-52.169, -24.073, 1500, {.forwards = false, .minSpeed = 127, .earlyExitRange = 30});
        robot::drivetrain::chassis.moveToPoint(-52.169, -24.073, 800, {.forwards = false, .maxSpeed = 70});
        robot::drivetrain::chassis.waitUntil(autosetting::distance_calculator(-51.169, -24.073) - 8); // wait until 10 inches away
        robot::mechanisms::clamp.set_value(true);
        robot::drivetrain::chassis.waitUntilDone();
        autosetting::run_intake(2000);
   
        robot::drivetrain::chassis.turnToPoint(33.588, -50.678, 500);
        robot::drivetrain::chassis.moveToPoint(33.588, -50.678, 1500, {.minSpeed = 127, .earlyExitRange = 40});
        robot::drivetrain::chassis.waitUntilDone();
        autosetting::run_LB(4800);
        robot::drivetrain::chassis.moveToPoint(33.588, -50.678, 1500, {.maxSpeed = 100});
        robot::drivetrain::chassis.waitUntil(autosetting::distance_calculator(33.588, -50.678) - 10); 
        autosetting::run_intake(1800);
        robot::drivetrain::chassis.turnToPoint(WS1x, -45.551, 500, {.forwards = false});
        robot::drivetrain::chassis.moveToPoint(WS1x, -45.551, 1500, {.forwards = false}); // Line up with wall stake
        robot::drivetrain::chassis.waitUntilDone();
        autosetting::run_intake(120, -600);
        autosetting::run_LB(8000);
        robot::drivetrain::chassis.turnToHeading(180, 1000);
        robot::drivetrain::chassis.waitUntilDone();
        robot::drivetrain::chassis.setPose(0, -48.542, 180);
        autosetting::wait_until_LB_done();
        autosetting::run_intake(2000);
        robot::drivetrain::chassis.moveToPoint(0, -69, 1800, {.maxSpeed = 60});
        robot::drivetrain::chassis.waitUntil(20);
        autosetting::run_LB(18000);
        pros::delay(800);
        robot::drivetrain::chassis.waitUntilDone();
        robot::drivetrain::chassis.setPose(0, -56, robot::drivetrain::chassis.getPose().theta);
        robot::drivetrain::chassis.moveToPoint(0, -45.162, 1000, {.forwards = false});
        robot::drivetrain::chassis.waitUntilDone();
        autosetting::run_LB(11000);

        robot::drivetrain::chassis.turnToHeading(260, 700);
        robot::drivetrain::chassis.waitUntilDone();
        robot::drivetrain::chassis.setPose(robot::drivetrain::chassis.getPose().x, robot::drivetrain::chassis.getPose().y, 270);
        autosetting::run_intake(6500);
        robot::drivetrain::chassis.moveToPoint(-48.751, -41.162, 3000, {.maxSpeed = 60}); // -48.751 3 ring
        robot::drivetrain::chassis.moveToPoint(-63.821, -41.162, 2000, {.maxSpeed = 30});

        robot::drivetrain::chassis.swingToHeading(180, lemlib::DriveSide::RIGHT, 1000);
        robot::drivetrain::chassis.moveToPoint(-50.888, -51.388, 1000); // 6th ring
        pros::delay(500);
        robot::drivetrain::chassis.turnToHeading(60, 600);
        robot::drivetrain::chassis.moveToPoint(-58.656, -53.475, 1000, {.forwards = false}); // wall hit
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::clamp.set_value(false);
        robot::drivetrain::chassis.moveToPoint(-54.635, -55.144, 1500);
        robot::drivetrain::chassis.turnToHeading(180, 800);
        robot::drivetrain::chassis.waitUntilDone();

        autosetting::reset_intake(); 
        robot::drivetrain::chassis.moveToPoint(-51.169, 34.943, 1500, {.forwards = false, .minSpeed = 100, .earlyExitRange = 50});    
        robot::drivetrain::chassis.moveToPoint(-51.169, 34.943, 1500, {.forwards = false, .maxSpeed = 50});
        robot::drivetrain::chassis.waitUntil(autosetting::distance_calculator(-51.169, 34.943) - 5); // second stake
        robot::mechanisms::clamp.set_value(true);
        robot::drivetrain::chassis.turnToPoint(-19.428, 23.312, 800);
        robot::drivetrain::chassis.waitUntilDone();
        robot::drivetrain::chassis.setPose(-47.392, 24.312, robot::drivetrain::chassis.getPose().theta);
        autosetting::run_intake(8000);
        robot::drivetrain::chassis.moveToPoint(-19.428, 23.312, 1500, {.maxSpeed = 60});
        pros::delay(200);
        robot::drivetrain::chassis.moveToPoint(-38.071, 23.312, 1500, {.forwards = false});
        robot::drivetrain::chassis.turnToPoint(27.597, 47.392, 500); // ring #
        robot::drivetrain::chassis.moveToPoint(27.597, 47.392, 1500, {.minSpeed = 127, .earlyExitRange = 40});
        robot::drivetrain::chassis.moveToPoint(27.597, 47.392, 1500, {.maxSpeed = 70});
        robot::drivetrain::chassis.waitUntil(autosetting::distance_calculator(27.597, 47.392) - 10); // 10 inches away
        autosetting::run_LB(4800);
        robot::drivetrain::chassis.moveToPoint(WS2x, 44.508, 1500, {.forwards = false});
        robot::drivetrain::chassis.waitUntilDone();
        autosetting::run_intake(140, -600);
        autosetting::run_LB(8000);
        robot::drivetrain::chassis.turnToHeading(5, 800);
        robot::drivetrain::chassis.waitUntilDone();
        robot::drivetrain::chassis.setPose(0, 48.542, 0);
        pros::delay(200);
        autosetting::run_intake(2000);
        robot::drivetrain::chassis.moveToPoint(0, 68, 1500, {.maxSpeed = 60});
        robot::drivetrain::chassis.waitUntil(20);
        autosetting::run_LB(18000);
        pros::delay(500);  
        robot::drivetrain::chassis.moveToPoint(0, 56, 1000, {.forwards = false});
        robot::drivetrain::chassis.waitUntilDone();
        autosetting::run_LB(0);
        robot::drivetrain::chassis.turnToHeading(270, 700);
        autosetting::run_intake(6500);
        robot::drivetrain::chassis.moveToPoint(-48.751, 56, 3000, {.maxSpeed = 60});
        robot::drivetrain::chassis.moveToPoint(-63.821, 56, 2000, {.maxSpeed = 30});
        robot::drivetrain::chassis.swingToHeading(0, lemlib::DriveSide::LEFT, 1000);
        robot::drivetrain::chassis.moveToPoint(-50.888, 62.481, 1000);
        pros::delay(700);
        robot::drivetrain::chassis.turnToHeading(90, 600);
        robot::drivetrain::chassis.moveToPoint(-63.044, 67.481, 1200, {.forwards = false}); //wall hit
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::clamp.set_value(false);
        robot::drivetrain::chassis.moveToPoint(-8, 53, 1000);
        robot::drivetrain::chassis.waitUntilDone();
        robot::drivetrain::chassis.setPose(0, 48, robot::drivetrain::chassis.getPose().theta);
        robot::drivetrain::chassis.turnToPoint(32.617, 18.845, 800);
        robot::drivetrain::chassis.moveToPoint(32.287, 18.845, 1500, {.minSpeed = 127, .earlyExitRange = 30});
        robot::drivetrain::chassis.moveToPoint(32.287, 18.845, 1500, {.maxSpeed = 70});
        robot::drivetrain::chassis.waitUntil(autosetting::distance_calculator(32.287, 16.32) - 10); // 10 inches away
        autosetting::run_intake(800);
        pros::delay(200);
        robot::drivetrain::chassis.turnToPoint(55.339, -4.594, 800, {.forwards = false});
        robot::drivetrain::chassis.moveToPoint(55.339, -4.594, 1500, {.forwards = false, .minSpeed = 127, .earlyExitRange = 20});
        robot::drivetrain::chassis.moveToPoint(55.339, -4.594, 1500, {.forwards = false, .maxSpeed = 70});
        robot::drivetrain::chassis.waitUntil(autosetting::distance_calculator(55.339, -3.294) - 10); // 10 inches away
        robot::mechanisms::clamp.set_value(true);
        robot::drivetrain::chassis.waitUntilDone();
        autosetting::run_intake(8000);
        robot::drivetrain::chassis.turnToHeading(340, 800);
        robot::drivetrain::chassis.follow(Skill1_txt, 12, 2500);
        robot::drivetrain::chassis.turnToPoint(61.33, 62.51, 800);
        robot::drivetrain::chassis.waitUntil(10);
        robot::mechanisms::doinker.set_value(true);
        robot::drivetrain::chassis.waitUntilDone();   
        robot::drivetrain::chassis.moveToPoint(62.33, 62.51, 1500);
        robot::drivetrain::chassis.turnToPoint(65, 62.51, 800, {.forwards = false});
        robot::drivetrain::chassis.moveToPoint(65, 62.51, 800, {.forwards = false});
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::clamp.set_value(false);
        robot::mechanisms::doinker.set_value(false);
        robot::drivetrain::chassis.turnToPoint(46.697, 42.828, 800); 
        robot::drivetrain::chassis.waitUntilDone();       
        
        pros::delay(200);
        robot::drivetrain::chassis.moveToPoint(46.697, 42.828, 1200);
        robot::drivetrain::chassis.turnToPoint(64.096, -12.712, 500);
        robot::drivetrain::chassis.moveToPoint(64.096, -12.712, 2500, {.minSpeed = 60});
        robot::drivetrain::chassis.moveToPoint(64.096, -65.534, 2500);
        robot::drivetrain::chassis.waitUntilDone();   
        robot::drivetrain::chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
        robot::mechanisms::hang.set_value(true);
        robot::drivetrain::chassis.moveToPoint(0, 0, 3500, {.forwards = false, .minSpeed = 100});
        



    } catch (const std::exception& e) {
        pros::lcd::print(0, "Skills Auto Error: %s", e.what()); 
    };
}

/*
          0
     270     90
         180
*/
ASSET(RedRing1_txt);
void red_ring_auto() {
    try {
        robot::mechanisms::lbRotationSensor.set_position(4800);
        robot::drivetrain::chassis.setPose(-54.383, 16.126, 180); 
        robot::drivetrain::chassis.moveToPoint(-54.383, 0, 500);
        robot::drivetrain::chassis.turnToHeading(270, 800);
        robot::drivetrain::chassis.waitUntil(50);
        autosetting::run_LB(25000);
        pros::delay(600);
        
        robot::drivetrain::chassis.turnToPoint(-19.01, 24.865, 300, {.forwards = false});

        robot::drivetrain::chassis.moveToPoint(-19.01, 24.865, 2000, {.forwards = false, .minSpeed = 127, .earlyExitRange = 35});
        pros::delay(200);
        autosetting::run_LB(0);

        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::lbMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        robot::mechanisms::lbMotor.move_velocity(0);

        robot::drivetrain::chassis.moveToPoint(-19.01, 24.865, 2000, {.forwards = false, .maxSpeed = 70});
        robot::drivetrain::chassis.waitUntil(30);
        robot::mechanisms::clamp.set_value(true);
        robot::mechanisms::lbRotationSensor.set_position(0);
        robot::drivetrain::chassis.turnToHeading(330, 600);
        autosetting::run_intake(7000);
        robot::drivetrain::chassis.follow(RedRing1_txt, 8, 2500);
        pros::delay(2000);

        robot::drivetrain::chassis.moveToPoint(-29.914, 48.946, 1000, {.forwards = false});
        robot::drivetrain::chassis.turnToPoint(-45.256, 14, 1000);
        robot::drivetrain::chassis.moveToPoint(-45.256, 14, 2000, {.minSpeed = 127, .earlyExitRange = 30});
        robot::drivetrain::chassis.moveToPoint(-45.256, 14, 2000, {.maxSpeed = 60});
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::doinker.set_value(true);
        pros::delay(300);
        robot::drivetrain::chassis.moveToPoint(-37.585, 31.468, 1000, {.forwards = false});
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::doinker.set_value(false);
        autosetting::run_intake(4000);
        robot::drivetrain::chassis.turnToPoint(-41.178, 12.825, 1000);
        robot::drivetrain::chassis.moveToPoint(-41.178, 12.825, 1000);
        pros::delay(200);
        robot::drivetrain::chassis.turnToPoint(-35.546, 6.222, 800);
        robot::drivetrain::chassis.moveToPoint(-35.546, 6.222, 1000);
        robot::drivetrain::chassis.waitUntil(5);
        robot::mechanisms::doinker.set_value(true);
    } catch (const std::exception& e) {
        pros::lcd::print(0, "Red Auto Error: %s", e.what());
    }
}
/*
          0 
     270     90
         180
*/
ASSET(RedStakeRush_txt)
ASSET(RedStakeReturn_txt)
void red_stake_auto() {
    try {
        robot::drivetrain::chassis.setPose(-52.053, -59.611, 90);
        robot::drivetrain::chassis.follow(RedStakeRush_txt, 10, 10000);
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::doinker.set_value(true);
        pros::delay(100);
        robot::drivetrain::chassis.follow(RedStakeReturn_txt, 10, 10000, false);
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::doinker.set_value(false);
        robot::drivetrain::chassis.moveToPoint(-49.528, -60.194, 1000, {.forwards = false});
        robot::drivetrain::chassis.turnToHeading(270, 1000);
        robot::drivetrain::chassis.moveToPoint(-19.74, -60.194, 1500, {.forwards = false, .maxSpeed = 70});
        robot::drivetrain::chassis.waitUntil(25);
        robot::mechanisms::clamp.set_value(true);
        robot::drivetrain::chassis.waitUntilDone();
        autosetting::run_intake(2000);
        robot::drivetrain::chassis.moveToPoint(-50.499, -59.028, 1500);
        robot::drivetrain::chassis.turnToPoint(-29.137, -50.095, 1000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::clamp.set_value(false);
        pros::delay(100);

        // Day 2 stuff (need tuning)
        robot::drivetrain::chassis.moveToPoint(-25.253, -47.182, 1500, {.minSpeed = 127, .earlyExitRange = 20});
        robot::drivetrain::chassis.waitUntilDone();
        autosetting::run_intake(900);
        robot::drivetrain::chassis.moveToPoint(-25.253, -47.182, 1500, {.maxSpeed = 70});
        robot::drivetrain::chassis.waitUntilDone();
        robot::drivetrain::chassis.turnToPoint(-28.361, -18.334, 1000, {.forwards = false});
        robot::drivetrain::chassis.moveToPoint(-28.361, -18.334, 1500, {.forwards = false, .maxSpeed = 80}); //-24.477
        robot::drivetrain::chassis.waitUntil(27);
        robot::mechanisms::clamp.set_value(true);
        robot::drivetrain::chassis.waitUntilDone();  
        autosetting::run_intake(3000);
        pros::delay(300);
        robot::drivetrain::chassis.turnToHeading(20, 1000);
        robot::drivetrain::chassis.moveToPoint(-21.564, -12.809, 1500);
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::doinker.set_value(true);
    } catch (const std::exception& e) {
        pros::lcd::print(0, "Two Stake Red Auto Error: %s", e.what());
    }
}
/* 
          0
     270     90
         180
*/

ASSET(BlueRing1_txt)
void blue_ring_auto() {
    try {
        
        robot::mechanisms::lbRotationSensor.set_position(4800);
        robot::drivetrain::chassis.setPose(54.383, 16.126, 180); //------------
        robot::drivetrain::chassis.moveToPoint(54.383, 8.747, 500);
        robot::drivetrain::chassis.turnToHeading(90, 800);
        robot::drivetrain::chassis.waitUntil(50);
        autosetting::run_LB(25000);
        pros::delay(600);
        autosetting::run_LB(0);
 
    
        robot::drivetrain::chassis.turnToPoint(19.01, 24.865, 1000, {.forwards = false});

        robot::drivetrain::chassis.moveToPoint(19.01, 24.865, 2000, {.forwards = false, .minSpeed = 127, .earlyExitRange = 35});
        pros::delay(200);
        autosetting::run_LB(0);

        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::lbMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        robot::mechanisms::lbMotor.move_velocity(0);

        robot::drivetrain::chassis.moveToPoint(19.01, 24.865, 2000, {.forwards = false, .maxSpeed = 70});
        robot::drivetrain::chassis.waitUntil(30);
        robot::mechanisms::clamp.set_value(true);
        robot::mechanisms::lbRotationSensor.set_position(0);
        robot::drivetrain::chassis.turnToHeading(30, 600);
        autosetting::run_intake(7000);
        robot::drivetrain::chassis.follow(BlueRing1_txt, 8, 2500);
        pros::delay(2000);

        robot::drivetrain::chassis.moveToPoint(29.914, 48.946, 1000, {.forwards = false});
        robot::drivetrain::chassis.turnToPoint(45.256, 14, 1000);
        robot::drivetrain::chassis.moveToPoint(45.256, 14, 2000, {.minSpeed = 127, .earlyExitRange = 30});
        robot::drivetrain::chassis.moveToPoint(45.256, 14, 2000, {.maxSpeed = 60});
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::doinker.set_value(true);
        pros::delay(300);
        robot::drivetrain::chassis.moveToPoint(37.585, 31.468, 1000, {.forwards = false});
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::doinker.set_value(false);
        autosetting::run_intake(4000);
        robot::drivetrain::chassis.turnToPoint(41.178, 12.825, 1000);
        robot::drivetrain::chassis.moveToPoint(41.178, 12.825, 1000);
        pros::delay(200);
        robot::drivetrain::chassis.turnToPoint(35.546, 6.222, 800);
        robot::drivetrain::chassis.moveToPoint(35.546, 6.222, 1000);
        robot::drivetrain::chassis.waitUntil(5);
        robot::mechanisms::doinker.set_value(true);

    } catch (const std::exception& e) {
        pros::lcd::print(0, "One Stake Blue Auto Error: %s", e.what());
                        }
                    }
/*
          0
     270     90
         180
*///55

ASSET(BlueStakeRush_txt);   
ASSET(BlueStakeReturn_txt);
void blue_stake_auto() {
    float ring1x = 12.421;
    float ring1y = -59.028;
    float stake2x = 19.8;
    float stake2y = -20.966;
    try {
        robot::drivetrain::chassis.setPose(-52.053, -59.611, 90);
        robot::drivetrain::chassis.follow(RedStakeRush_txt, 10, 10000);
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::doinker.set_value(true);
        pros::delay(100);
        robot::drivetrain::chassis.follow(RedStakeReturn_txt, 10, 10000, false);
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::doinker.set_value(false);
        robot::drivetrain::chassis.moveToPoint(-49.528, -60.194, 1000, {.forwards = false});
        robot::drivetrain::chassis.waitUntilDone();
        robot::drivetrain::chassis.setPose(49.528, -35.806, 270);

        robot::drivetrain::chassis.turnToPoint(20.189, -43.493, 1000, {.forwards = false});
        robot::drivetrain::chassis.moveToPoint(20.189, -43.493, 1000, {.forwards = false, .maxSpeed = 60});
        robot::drivetrain::chassis.waitUntil(32);
        robot::mechanisms::clamp.set_value(true);
        autosetting::run_intake(1700);
        pros::delay(200);
        robot::drivetrain::chassis.moveToPoint(54.95, -41.162, 1500);
        robot::drivetrain::chassis.turnToPoint(ring1x, ring1y, 1000);
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::clamp.set_value(false);


        // Day 2 stuff (need tuning)

        robot::drivetrain::chassis.turnToPoint(stake2x, stake2y, 1000, {.forwards = false});  
        robot::drivetrain::chassis.moveToPoint(stake2x, stake2y, 1500, {.forwards = false, .minSpeed = 127, .earlyExitRange = 20});
        robot::drivetrain::chassis.moveToPoint(stake2x, stake2y, 1500, {.forwards = false, .maxSpeed = 60});
        robot::drivetrain::chassis.waitUntil(autosetting::distance_calculator(stake2x, stake2y) - 6);
        robot::mechanisms::clamp.set_value(true);
        autosetting::run_intake(10000);
        robot::drivetrain::chassis.turnToPoint(11.644, -59.028, 1000);
        robot::drivetrain::chassis.moveToPoint(11.644, -59.028, 2000);
        pros::delay(500);
        robot::drivetrain::chassis.moveToPoint(33.006, -39.22, 5000, {.forwards = false, .maxSpeed = 100});

        
    } catch (const std::exception& e) {
        pros::lcd::print(0, "Two Stake Blue Auto Error: %s", e.what());
    }
}
/*
          0
     270     90
         180
*/
void test_auto() {
    try {
        autosetting::run_LB(4800);



    } catch (const std::exception& e) {
        pros::lcd::print(0, "Test Auto Error: %s", e.what());
    }

}

void a(){
    robot::drivetrain::chassis.setPose(0, 0, 90);
    robot::drivetrain::chassis.moveToPoint(14, 0, 1000);
}

void autonomous() {
    robot::mechanisms::intakeMotor.move_velocity(200);
    std::cout << "Running Auto" << std::endl;
    // Create task at start of autonomous
    pros::Task intake_task(autosetting::intake_task_fn, nullptr, "Intake Task");
    pros::Task lb_task(autosetting::lb_task_fn, nullptr, "LB Task");
    
    // Your existing autonomous code
    switch (current_auto) {
        case AutonomousMode::SKILLS:
            skills_auto();
            break;
        case AutonomousMode::RED_RING:
            red_ring_auto();
            break;
        case AutonomousMode::RED_STAKE:
            red_stake_auto();
            break;
        case AutonomousMode::BLUE_RING:
            blue_ring_auto();
            break;
        case AutonomousMode::BLUE_STAKE:
            blue_stake_auto();
            break;
        case AutonomousMode::TEST:
            test_auto();
            break;
        case AutonomousMode::SCREW:
            a();
            break;
    }
}

