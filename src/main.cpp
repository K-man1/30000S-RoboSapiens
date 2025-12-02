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
pros::MotorGroup leftMotors({11, -12, 13}, pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({-14, 15, -16}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

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
lemlib::ControllerSettings linearController(
    5.0,   // kP  (same)
    0.0,   // kI
    1.0,   // kD  <-- add small D to damp approach
    3,     // anti windup (keep)
    1,     // small error range, inches
    100,   // small error range timeout ms
    3,     // large error range, inches
    500,   // large error range timeout ms
    12     // max acceleration (slew) <-- lower from 20 to 12
);


// angular motion controller
lemlib::ControllerSettings angularController(
    2,  // kP  << drastically lower
    0.0,   // kI
    5,   // kD  << strong damping
    20,
    1,
    200,
    5,
    500,
    0
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
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    
    pros::lcd::initialize();
    imu.reset();
    pros::delay(1500);          // wait for IMU to stabilize
    chassis.setPose(0, 0, 0);   // reset robot pose
    
    pros::Task screenTask([&]() {
        while(true) {
            auto pose = chassis.getPose();
            pros::lcd::print(0, "X: %f", pose.x);
            pros::lcd::print(1, "Y: %f", pose.y);
            pros::lcd::print(2, "Theta: %f", pose.theta);
            pros::delay(50);
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
    pros::lcd::print(4, "Automous function is running");
        // Move forward 24 inches (2 feet).
        // Use a timeout (milliseconds) and set async=false so this call blocks
        // until the motion completes or the timeout elapses.
        chassis.setPose(0, 0, 0);
        chassis.turnToHeading(90, 10000);
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
    chassis.arcade(leftY, -rightX);

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