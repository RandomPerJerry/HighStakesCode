#include "main.h"
#include "lemlib/api.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

//reverse later 1
pros::MotorGroup left_motors({-16, -18, -17}, pros::MotorGearset::blue); // left motors on ports 1, 2, 3
pros::MotorGroup right_motors({4, 3, 1}, pros::MotorGearset::blue); // right motors on ports 4, 5, 6

pros::Motor ramp_motor1(11, pros::MotorGearset::blue); // ramp motor on port D
pros::Motor ramp_motor2(12, pros::MotorGearset::blue); // ramp motor on port D

pros::ADIDigitalOut clampPiston('A'); // clamp on port E
pros::ADIDigitalOut intakePiston('B'); // intake on port F
pros::ADIDigitalOut dongerPiston('C'); // intake on port G

//Drive train
lemlib::Drivetrain drivetrain(
	&left_motors, // left motor group
	&right_motors, // right motor group
	11.4, // 11.4 inch track width
    lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
    450, // drivetrain rpm is 450
    2 // horizontal drift is 2 (for now)
);

pros::adi::Encoder vertical_encoder('D', 'E'); // left encoder on ports 1, 2 Relative to the tracking center: front is positive, back is negative
pros::Imu imu(5);

lemlib::TrackingWheel vertical_tracking_wheel(
	&vertical_encoder, // encoder
	lemlib::Omniwheel::NEW_275, 
	0.0 // 0 inch distance from tracking center
);

lemlib::OdomSensors sensors(&vertical_tracking_wheel,
                            nullptr,
                            nullptr,
                            nullptr, 
                            &imu
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen 
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "Intake: %d", ramp_motor1.get_target_velocity());
            // delay to save resources
            pros::delay(20);
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
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

ASSET(lap1_txt)
void autonomous() {
    chassis.setPose(47.482, -46.707, 0); // set robot position to (0, 0, 0)
    chassis.follow(lap1_txt, 10, 20000); // follow the path
}

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


void intake() {
    static bool r2_toggle = false;
    static bool clamp_state = false;

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
        r2_toggle = !r2_toggle;
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        ramp_motor1.move_velocity(600);
        ramp_motor2.move_velocity(-600);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        ramp_motor1.move_velocity(-600);
        ramp_motor2.move_velocity(600);
    } else if (r2_toggle) {
        ramp_motor1.move_velocity(600); 
        ramp_motor2.move_velocity(-600);
    } else {
        ramp_motor1.move_velocity(0);
        ramp_motor2.move_velocity(0);
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        clamp_state = !clamp_state;
    }
    intakePiston.set_value(clamp_state);
}

void doungler_control(){
    static bool donger_state = false;
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
        donger_state = !donger_state;
    }
    dongerPiston.set_value(donger_state);
}

void clamp_control() {
    static bool clamp_state = true;
    
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        clamp_state = !clamp_state;
    }
    clampPiston.set_value(clamp_state);
}

void drive_control(bool &isReversed){
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            isReversed = !isReversed;
        }
        if (isReversed) {
            leftY = -leftY;
        }
        chassis.arcade(leftY, rightX);
};

void opcontrol() {
    bool reverse_drive = false;
    // loop forever
    while (true) {
        
        // drive_control(reverse_drive);
        clamp_control();
        intake(); 
        doungler_control();

        // delay to save resources
        pros::delay(25);
    }
}