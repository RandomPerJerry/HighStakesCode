#include "main.h"
#include "config.hpp"
#include "auto.h"
#include "lemlib/timer.hpp"

enum class AutonomousMode {
    SKILLS,
    RED_RING,
    RED_STAKE,
    BLUE_RING,
    BLUE_STAKE,      
    TEST,
<<<<<<< HEAD
    liam_skills
};
  
// Current autonomous selection
static AutonomousMode current_auto = AutonomousMode::liam_skills;
=======
    LIAM_SKILLS,
    SKILLS_AUTO
};
  
// Current autonomous selection
static AutonomousMode current_auto = AutonomousMode::LIAM_SKILLS;
>>>>>>> c6f96015ecaf8884782d51ad20585e033abc083a

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

    void lb_task_fn(void* param) {
        while (pros::competition::is_autonomous()) {
            double currentPosition = robot::mechanisms::lbRotationSensor.get_position();
            double error = LBState::targetPosition - currentPosition;
            
            if (std::abs(error) < 100) {
                LBState::isRunning = false;
                robot::mechanisms::lbMotor.move_velocity(0);
                if (LBState::targetPosition == 0) {
                    robot::mechanisms::lbRotationSensor.reset_position();
                }
            } else {
                LBState::isRunning = true;
                double pidOutput = robot::pid::lbPID.update(error);
                double velocityCommand = std::clamp(pidOutput, -LBState::runSpeed, LBState::runSpeed);
                robot::mechanisms::lbMotor.move_velocity(velocityCommand);
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

    bool isLBRunning() {
        return LBState::isRunning;
    }

    void pickup_ring(float x, float y, float exitRange1, float exitRange2) {
        robot::drivetrain::chassis.moveToPoint(x, y, 1000, {.minSpeed = 127, .earlyExitRange = exitRange1});
        robot::drivetrain::chassis.moveToPoint(x, y, 1000, {.maxSpeed = 70, .earlyExitRange = exitRange2});
        robot::drivetrain::chassis.moveToPoint(x, y, 1000, {.maxSpeed = 120});
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
    // Q1
    // Ring 1
    float point1x = -21;
    float point1y = 26.03;

    // Ring 2
    float point2x = 29.899;
    float point2y = 50.305;

    // Ring 3, 4, 5
    float point3x = -60.015;
    float point3y = 49.917;

    // Ring 6
    float point4x = -45.45;
    float point4y = 60.209;

    // Corner
    float point5x = -61.763;
    float point5y = 62.928;

    // Q2
    // Stake 2 (part 1)
    float point6x = -46.809;
    float point6y = 34.381;

    // Stake 2 (part 2)
    float point7x = -47.198;
    float point7y = -26.015;

    // Ring 1
    float point8x = -23.506;
    float point8y = -26.419;

    // Ring 2
    float point9x = 29.899;
    float point9y = -50.305;

    // Ring 3, 4, 5
    float point10x = -60.015;
    float point10y = -49.917;

    // Ring 6
    float point11x = -45.45;
    float point11y = -60.209;

    // Corner
    float point12x = -61.763;
    float point12y = -62.928;


    try {
        robot::drivetrain::chassis.setPose(-55.635, 0, 270);
        robot::mechanisms::lbRotationSensor.set_position(4800);
        
        autosetting::run_LB(25000);
        pros::delay(600);

        robot::drivetrain::chassis.moveToPoint(-47, 0, 1000, {.forwards = false});
        robot::drivetrain::chassis.waitUntilDone();
        autosetting::run_LB(0);


        robot::drivetrain::chassis.turnToHeading(180, 1000);
        robot::drivetrain::chassis.moveToPoint(-47, 26.03, 1000, {.forwards = false, .maxSpeed = 60});
        robot::drivetrain::chassis.waitUntil(21); // 11 
        robot::mechanisms::clamp.set_value(true);
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::lbMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        robot::mechanisms::lbMotor.move_velocity(0);

        // Q1 ---
        robot::drivetrain::chassis.turnToPoint(point1x, point1y, 1000); 
        robot::drivetrain::chassis.waitUntilDone();  
        autosetting::run_intake(20000);
        autosetting::pickup_ring(point1x, point1y, 9, 4); //1111111

        pros::delay(200);
        robot::drivetrain::chassis.turnToPoint(point2x, point2y, 1000);
        autosetting::pickup_ring(point2x, point2y, 9, 4); //2222222
        pros::delay(200);
        robot::drivetrain::chassis.turnToPoint(point3x, point3y, 1000);
        robot::drivetrain::chassis.moveToPoint(point3x, point3y, 4000, {.maxSpeed = 50});
        pros::delay(200);
        robot::drivetrain::chassis.turnToPoint(point4x, point4y, 1000);
        autosetting::pickup_ring(point4x, point4y, 9, 4); //3333333
        pros::delay(200);
        robot::drivetrain::chassis.turnToPoint(point5x, point5y, 1000, {.forwards = false});
        robot::drivetrain::chassis.moveToPoint(point5x, point5y, 1500, {.forwards = false, .maxSpeed = 70});
        pros::delay(200);
        robot::mechanisms::clamp.set_value(false);

        // Q2 ---
        /*
        robot::drivetrain::chassis.turnToPoint(point6x, point6y, 800, {.minSpeed = 127, .earlyExitRange = 20});
        robot::drivetrain::chassis.moveToPoint(point6x, point6y, 1500);
        
        robot::drivetrain::chassis.turnToPoint(point7x, point7y, 700, {.forwards = false});
        robot::drivetrain::chassis.moveToPoint(point7x, point7y, 2000, {.forwards = false, .minSpeed = 127, .earlyExitRange = 22});
        robot::drivetrain::chassis.moveToPose(point7x, point7y, 0, 1500, {.forwards = false, .maxSpeed = 60});
        robot::drivetrain::chassis.waitUntil(53); // travles total 60.396 in (12)
        robot::mechanisms::clamp.set_value(true);
        
        robot::drivetrain::chassis.turnToPoint(point8x, point8y, 1000); 
        robot::drivetrain::chassis.waitUntilDone();
        autosetting::run_intake(20000);
        robot::drivetrain::chassis.moveToPoint(point8x, point8y, 1000, {.minSpeed = 120, .earlyExitRange = 20});
        robot::drivetrain::chassis.moveToPoint(point8x, point8y, 1000, {.maxSpeed = 70});
        pros::delay(200);
        robot::drivetrain::chassis.turnToPoint(point9x, point9y, 1000);
        robot::drivetrain::chassis.moveToPoint(point9x, point9y, 1500, {.minSpeed = 127, .earlyExitRange = 20});  
        robot::drivetrain::chassis.moveToPoint(point9x, point9y, 1500, {.maxSpeed = 70});
        pros::delay(200);
        robot::drivetrain::chassis.turnToPoint(point10x, point10y, 1000);
        robot::drivetrain::chassis.moveToPoint(point10x, point10y, 4000, {.maxSpeed = 50});
        pros::delay(200);
        robot::drivetrain::chassis.turnToPoint(point11x, point11y, 1000);
        robot::drivetrain::chassis.moveToPoint(point11x, point11y, 1500, {.minSpeed = 127, .earlyExitRange = 20});
        robot::drivetrain::chassis.moveToPoint(point11x, point11y, 1500, {.maxSpeed = 70});
        pros::delay(200);
        robot::drivetrain::chassis.turnToPoint(point12x, point12y, 1000, {.forwards = false});
        robot::drivetrain::chassis.moveToPoint(point12x, point12y, 1500, {.forwards = false, .maxSpeed = 70});
        pros::delay(200);
        robot::mechanisms::clamp.set_value(false);
        */
        // Q3


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
        robot::drivetrain::chassis.swingToHeading(236, lemlib::DriveSide::RIGHT, 800);
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
        robot::drivetrain::chassis.turnToPoint(-22.923, -19.334, 1000, {.forwards = false});
        robot::drivetrain::chassis.moveToPoint(-22.923, -19.334, 1500, {.forwards = false, .maxSpeed = 80});
        robot::drivetrain::chassis.waitUntil(25);
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
        robot::drivetrain::chassis.swingToHeading(124, lemlib::DriveSide::LEFT, 800);
        robot::drivetrain::chassis.waitUntil(50);
        autosetting::run_LB(25000);
        pros::delay(600);
 
    
        robot::drivetrain::chassis.turnToPoint(19.01, 24.865, 300, {.forwards = false});

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
    try {
        robot::drivetrain::chassis.setPose(52.053, -59.611, 270);
        robot::drivetrain::chassis.follow(BlueStakeRush_txt, 10, 10000);
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::doinker.set_value(true);
        pros::delay(100);
        robot::drivetrain::chassis.follow(BlueStakeReturn_txt, 10, 10000, false);
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::doinker.set_value(false);
        /*
        robot::drivetrain::chassis.moveToPoint(49.528, -60.194, 1000, {.forwards = false});
        robot::drivetrain::chassis.turnToHeading(270, 1000);
        robot::drivetrain::chassis.moveToPoint(19.74, -60.194, 1500, {.forwards = false, .maxSpeed = 70});
        robot::drivetrain::chassis.waitUntil(25);
        robot::mechanisms::clamp.set_value(true);
        robot::drivetrain::chassis.waitUntilDone();
        autosetting::run_intake(2000);
        robot::drivetrain::chassis.moveToPoint(50.499, -59.028, 1500);
        robot::drivetrain::chassis.turnToPoint(29.137, -50.095, 1000, {.direction = AngularDirection::CW_CLOCKWISE});
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::clamp.set_value(false);
        pros::delay(100);

        // Day 2 stuff (need tuning)
        robot::drivetrain::chassis.moveToPoint(25.253, -47.182, 1500, {.minSpeed = 127, .earlyExitRange = 20});
        robot::drivetrain::chassis.waitUntilDone();
        autosetting::run_intake(900);
        robot::drivetrain::chassis.moveToPoint(25.253, -47.182, 1500, {.maxSpeed = 70});
        robot::drivetrain::chassis.waitUntilDone();
        robot::drivetrain::chassis.turnToPoint(22.923, -19.334, 1000, {.forwards = false});
        robot::drivetrain::chassis.moveToPoint(22.923, -19.334, 1500, {.forwards = false, .maxSpeed = 80});
        robot::drivetrain::chassis.waitUntil(25);
        robot::mechanisms::clamp.set_value(true);
        robot::drivetrain::chassis.waitUntilDone();  
        autosetting::run_intake(3000);
        pros::delay(300);
        robot::drivetrain::chassis.turnToHeading(340, 1000);
        robot::drivetrain::chassis.moveToPoint(21.564, -12.809, 1500);
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::doinker.set_value(true);
        */
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
        robot::drivetrain::chassis.setPose(0, 0, 90);
        autosetting::run_LB(1000);



    } catch (const std::exception& e) {
        pros::lcd::print(0, "Test Auto Error: %s", e.what());
    }

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
        case AutonomousMode::LIAM_SKILLS:
            liam_skills();
            break;
    }
}

void liam_skills() {
        // Q1
    // Ring 1
    float point1x = -21;
    float point1y = 26.03;

    // Ring 2
    float point2x = 29.899;
    float point2y = 50.305;

    // Ring 3, 4, 5
    float point3x = -60.015;
    float point3y = 49.917;

    // Ring 6
    float point4x = -45.45;
    float point4y = 60.209;

    // Corner
    float point5x = -61.763;
    float point5y = 62.928;

    // Q2
    // Stake 2 (part 1)
    float point6x = -47.285;
    float point6y = 34.881;

    // Stake 2 (part 2)
    float point7x = -44.198;    
    float point7y = -25.015;

    // Ring 1
    float point8x = -23.506;
    float point8y = -24.419;

    // Ring 2
    float point9x = 29.899;
    float point9y = -47.305;

    // Ring 3, 4, 5
    float point10x = -60.015;
    float point10y = -48.917;

    // Ring 6
    float point11x = -48.45;
    float point11y = -55.209;

    // Corner
    float point12x = -64.763;
    float point12y = -62.928;


    try {
        robot::drivetrain::chassis.setPose(-55.635, 0, 270);
        robot::mechanisms::lbRotationSensor.set_position(0);
        /* lb score on alliance stake (currently too big)
        autosetting::run_LB(25000);
        pros::delay(600);
        */
        robot::drivetrain::chassis.moveToPoint(-47, 0, 1000, {.forwards = false});
        robot::drivetrain::chassis.waitUntilDone();
       // autosetting::run_LB(0);


        robot::drivetrain::chassis.turnToHeading(180, 1000);
        robot::drivetrain::chassis.moveToPoint(-47, 26.03, 1000, {.forwards = false, .maxSpeed = 60});
        robot::drivetrain::chassis.waitUntil(21); // 11 
        robot::mechanisms::clamp.set_value(true);
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::lbMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        robot::mechanisms::lbMotor.move_velocity(0);

        // Q1 ---
        robot::drivetrain::chassis.turnToPoint(point1x, point1y, 1000); 
        robot::drivetrain::chassis.waitUntilDone();  
        autosetting::run_intake(20000);
        autosetting::pickup_ring(point1x, point1y, 9, 4); //1111111

        pros::delay(200);
        robot::drivetrain::chassis.turnToPoint(point2x, point2y, 1000);
        autosetting::pickup_ring(point2x, point2y, 9, 4); //2222222
        pros::delay(200);
        robot::drivetrain::chassis.turnToPoint(point3x, point3y, 1000);
        robot::drivetrain::chassis.moveToPoint(point3x, point3y, 4000, {.maxSpeed = 50});
        pros::delay(200);
        robot::drivetrain::chassis.turnToPoint(point4x, point4y, 1000);
        autosetting::pickup_ring(point4x, point4y, 9, 4); //3333333
        pros::delay(200);
        robot::drivetrain::chassis.turnToPoint(point5x, point5y, 1000, {.forwards = false});
        robot::drivetrain::chassis.moveToPoint(point5x, point5y, 1500, {.forwards = false, .maxSpeed = 70});
        pros::delay(200);
        robot::mechanisms::clamp.set_value(false); 
                //Liam Code---------------------------------------
       /*
        //go to wall stake
        pros::delay(500);
        robot::drivetrain::chassis.turnToPoint(0, 57.96, 1000, {.forwards = true});
        autosetting::run_LB(4800);
        robot::drivetrain::chassis.moveToPoint(-3, 55.69, 1500, {.forwards = true, .maxSpeed = 150});
        autosetting::run_intake(5000);
        robot::drivetrain::chassis.waitUntilDone(); 
        robot::drivetrain::chassis.turnToHeading(0, 1000);
        robot::drivetrain::chassis.waitUntilDone();
        robot::drivetrain::chassis.moveToPoint(0, 67.69, 1500, {.forwards = true, .maxSpeed = 150});
        robot::drivetrain::chassis.waitUntilDone();
        //wall stake
        autosetting::run_LB(18000);
        pros::delay(1500);
        autosetting::run_LB(0);
        robot::drivetrain::chassis.moveToPoint(0, 38.69, 1500, {.forwards = false, .maxSpeed = 150});
*/

        // Q2 ---
        
        robot::drivetrain::chassis.turnToPoint(point6x, point6y, 800, {.minSpeed = 127, .earlyExitRange = 20});
        robot::drivetrain::chassis.moveToPoint(point6x, point6y, 1500);
        
        robot::drivetrain::chassis.turnToPoint(point7x-12, point7y, 700, {.forwards = false});
        robot::drivetrain::chassis.moveToPoint(point7x-12, point7y, 2000, {.forwards = false, .minSpeed = 127, .earlyExitRange = 22});
        robot::drivetrain::chassis.moveToPose(point7x-12, point7y, 0, 1500, {.forwards = false, .maxSpeed = 60});

        robot::drivetrain::chassis.turnToPoint(point7x, point7y, 700, {.forwards = false});
        robot::drivetrain::chassis.moveToPoint(point7x, point7y, 2000, {.forwards = false, .minSpeed = 127, .earlyExitRange = 22});
        robot::drivetrain::chassis.moveToPose(point7x, point7y, 0, 1500, {.forwards = false, .maxSpeed = 60});
        robot::drivetrain::chassis.waitUntil(53); // travles total 60.396 in (12)
        robot::mechanisms::clamp.set_value(true);
        
        robot::drivetrain::chassis.turnToPoint(point8x, point8y, 1000); 
        robot::drivetrain::chassis.waitUntilDone();
        autosetting::run_intake(20000);
        robot::drivetrain::chassis.moveToPoint(point8x, point8y, 1000, {.minSpeed = 120, .earlyExitRange = 20});
        robot::drivetrain::chassis.moveToPoint(point8x, point8y, 1000, {.maxSpeed = 70});
        pros::delay(200);
        robot::drivetrain::chassis.turnToPoint(point9x, point9y, 1000);
        robot::drivetrain::chassis.moveToPoint(point9x, point9y, 1500, {.minSpeed = 127, .earlyExitRange = 20});  
        robot::drivetrain::chassis.moveToPoint(point9x, point9y, 1500, {.maxSpeed = 70});
        pros::delay(200);
        robot::drivetrain::chassis.turnToPoint(point10x, point10y, 1000);
        robot::drivetrain::chassis.moveToPoint(point10x, point10y, 4000, {.maxSpeed = 50});
        pros::delay(200);
        robot::drivetrain::chassis.turnToPoint(point11x, point11y, 1000);
        robot::drivetrain::chassis.moveToPoint(point11x, point11y, 1500, {.minSpeed = 127, .earlyExitRange = 20});
        robot::drivetrain::chassis.moveToPoint(point11x, point11y, 1500, {.maxSpeed = 70});
        pros::delay(200);
        robot::drivetrain::chassis.turnToPoint(point12x, point12y, 1000, {.forwards = false});
        robot::drivetrain::chassis.moveToPoint(point12x, point12y, 1500, {.forwards = false, .maxSpeed = 70});
        pros::delay(200);
        robot::mechanisms::clamp.set_value(false);
        
        // Q3
    } catch (const std::exception& e) {
        pros::lcd::print(0, "Liam skills auto error: %s", e.what());
    }
}
