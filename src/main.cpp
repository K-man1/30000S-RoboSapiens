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

pros::adi::DigitalOut right_wing('C');
bool right_wing_state = true;
pros::adi::DigitalOut left_wing('D');
bool left_wing_state = true;

// motor groups
pros::MotorGroup leftMotors({11, -12, 13},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({-14, 15, -16}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 20
pros::Imu imu(17);

// tracking wheels

// vertical tracking wheel encoder. Rotation sensor, port 19, reversed
pros::Rotation verticalEnc(-10);

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
lemlib::ControllerSettings linearController(2, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            0, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(6, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             3, // derivative gain (kD)
                                             20, // anti windup
                                             1, // small error range, in degrees
                                             200, // small error range timeout, in milliseconds
                                             5, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
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
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    pros::delay(500);
    chassis.setPose(0, 0, 0); // reset the robot position to x:0, y:0, heading: 0
    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // print vertical encoder rotation (rotation sensor position)
            pros::lcd::print(3, "Vert: %f", (double)verticalEnc.get_position());
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::lcd::print(7, "Rotation Sensor: %i", verticalEnc.get_position());
            pros::delay(10); // delay to save resources. DO NOT REMOVE
            // delay to save resources
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
        chassis.moveToPoint(0, 24, 10000, {}, false);
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

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B))
    {
        doubleParkState = !doubleParkState;
        double_park.set_value(doubleParkState);
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
    {
        willState = !willState;
        will.set_value(willState);
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
    {
        middle_goal.set_value(true);
        scorer.move(64);
        intake.move(127);
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
    {
        right_wing_state = !right_wing_state;
        right_wing.set_value(right_wing_state);
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
    {
        left_wing_state = !left_wing_state;
        left_wing.set_value(left_wing_state);
    }
    pros::delay(10);
}

}