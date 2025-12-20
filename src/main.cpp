#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "pros/adi.hpp"


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// user-operated motors
// Port 1 = Scorer
pros::Motor scorer(3);
// Port 2 & 3 = Intake (use MotorGroup to control both together)
pros::MotorGroup intake({-1, 2});

pros::adi::DigitalOut double_park('H');
bool doubleParkState = true;
pros::adi::DigitalOut will('A');
bool willState = true;
pros::adi::DigitalOut middle_goal('B');

pros::adi::DigitalOut wing('C');
bool wing_state = true;

// motor groups
pros::MotorGroup leftMotors({-11, 12, -13}, pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({14, -15, 16}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 20
pros::Imu imu(17);

// tracking wheels

// vertical tracking wheel encoder. Rotation sensor, port 19, reversed
pros::Rotation verticalEnc(10);

// vertical tracking wheel. 2.75" diameter, 0" offset
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 0.25);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 2.75" omnis
                              450, // drivetrain rpm is 450
                              8 // horizontal drift is 8. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings lateral_controller(6.3, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              8, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


// angular PID controller
lemlib::ControllerSettings angular_controller(1.8, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              12, // derivative gain (kD)
                                              3, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

lemlib::Pose pose = chassis.getPose();

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
            
            // delay to save resources
            pros::delay(20);
        }
    });
}



/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
// ASSET(path2loader2_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    
    
    chassis.moveToPoint(0, -29, 1000, {.forwards = false, .maxSpeed = 80}); // forward to laoder 
    chassis.turnToHeading(-85, 750); //turn to align to loader
    intake.move(100);
    will.set_value(true);
    chassis.moveToPoint(30, -31, 1500, {.forwards = false, .maxSpeed = 70});
    //move forward to loader and get blocks
    chassis.moveToPoint(-25, -33.5, 2000); //from loader to goal
    chassis.waitUntil(20);.         
    scorer.move(127);
    pros::delay(4000);
    will.set_value(false);
    //wings time
    chassis.moveToPose(chassis.getPose().x + 30, -45, -90, 2000);

    chassis.moveToPoint(chassis.getPose().x - 30, chassis.getPose().y + 20, 2000, {.forwards = false});

    // //push
    // chassis.moveToPoint(chassis.getPose().x + 6, chassis.getPose().y, 500, {.forwards = false});
    // pros::delay(500);
    // will.set_value(true);
    // chassis.moveToPoint(chassisddddrrrrrtghuuh`.getPose().x - 6, chassis.getPose().y, 500, {.minSpeed = 127});
    // pros::delay(500);
// //BTW AT THIS POINT THE ROBOT IS AT X = -8 AND Y = -30 THETA = 90
//     chassis.moveToPoint(-5, chassis.getPose().y, 1500, {.forwards = false}); //back away from goal
//     will.set_value(false);
//     intake.move(100);
//     scorer.move(-20);
//     chassis.turnToHeading(115, 1000);
//     chassis.moveToPoint(-30, -2, 2000, {.forwards = false, .maxSpeed = 75});
//     chassis.moveToPose(-35, 20, 115, 1000);
//     chassis.moveToPoint(-40, 25, 1000);
//     // chassis.turnToHeading(-135, 4000);
//     // chassis.moveToPoint(-40, 15, 2000);
//     pros::delay(1000);
//     scorer.move(80);
//     middle_goal.set_value(true);

}

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
    // drivetrain
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    chassis.arcade(-leftY, rightX);

    // Scorer control (Port 1)
    // - Turn forward when L2 is pressed
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        scorer.move(127);
        intake.move(127);
    } 
    
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        scorer.move(-127);
    }

    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        intake.move(-127);
    } 
    
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        intake.move(127);
        scorer.move(-10); // stop scorer when intaking
    }

    else {
        intake.move(0);
        scorer.move(0);
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
    {
        doubleParkState = !doubleParkState;
        double_park.set_value(doubleParkState);
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
    {
        willState = !willState;
        will.set_value(willState);
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
    {
        scorer.move(64);
        intake.move(127);
        middle_goal.set_value(true);
    }
    else
    {
        middle_goal.set_value(false);
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
    {
        wing_state = !wing_state;
        wing.set_value(wing_state);
    }

    
    pros::delay(10);
}

}