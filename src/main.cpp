#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "pros/adi.hpp"



// user-operated motors
// Port 1 = Scorer
pros::MotorGroup scorer({-16, 17});
// Port 2 & 3 = Intake (use MotorGroup to control both together)
pros::Motor intake(1);

pros::adi::DigitalOut will('H');
bool willState = true;
pros::adi::DigitalOut middle_goal('B');

// pros::adi::DigitalOut wing('C');
// bool wing_state = true;

// motor groups
pros::MotorGroup leftMotors({8, -9, -10}, pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({-7,6,5}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 20
pros::Imu imu(2);

// tracking wheels

// vertical tracking wheel encoder. Rotation sensor, port 19, reversed
pros::Rotation verticalEnc(4);

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
lemlib::ControllerSettings lateral_controller(6.25, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              5, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);


// angular PID controller
lemlib::ControllerSettings angular_controller(1.235, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              6, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              500, // small error range timeout, in milliseconds
                                              2, // large error range, in degrees
                                              1000, // large error range timeout, in milliseconds
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
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::lcd::print(3, "IMU: %f", (double)imu.get_rotation());

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

    chassis.setPose(0, 0, 0);

    //Route to loader
    
    //Move forward to loader
    chassis.moveToPoint(0, 30.5, 5000);
    //deploy little will
    will.set_value(true);
    willState = true;
    //start intake
    intake.move(127);
    scorer.move(-10);
    //Turn to face loader
    chassis.turnToHeading(-90, 3000);
    // Go into loader
    chassis.moveToPoint(-15, 30.5, 5000);

    //STAY HERE UNTIL OPTICAL SENSOR SEES BLUE. ADD CODE HERE.

    pros::delay(1000);
    //Loader to goal

    //Back from loader into goal
    chassis.moveToPoint(20, 32.1, 5000, {.forwards = false});

    //Score blocks
    scorer.move(127);
    pros::delay(2500);

    //Route to middle goal

    //leave goal
    will.set_value(false);
    willState = false;
    chassis.turnToHeading(180, 3000);

    //intake corner blocks
    chassis.moveToPoint(24, 10, 5000);

    
}

/**
 * Runs in driver control
 */
pros::Controller controller(pros::E_CONTROLLER_MASTER);

void opcontrol() {
    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);

        // score
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        scorer.move(127);
        intake.move(127);
        } 
        
        // middle goal
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
        scorer.move(64);
        intake.move(127);
        middle_goal.set_value(false);
        }


        // outtake everythin
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.move(-127);
            scorer.move(-10);
        } 
        
        // just intake
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move(127);
            scorer.move(-127); // stop scorer when intaking
        }

        else {
            intake.move(0);
            scorer.move(0);
            middle_goal.set_value(true);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
    {
        willState = !willState;
        will.set_value(willState);
    }

        // delay to save resources
        pros::delay(25);
    }
    

}