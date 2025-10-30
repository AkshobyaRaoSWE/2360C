#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/motors.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_motors({1, 2, 3}); 
pros::MotorGroup right_motors({4, 5, 6}); 

pros::Motor intake_motor_1(7);
pros::Motor intake_motor_2(8);
pros::Motor intake_motor_3(9);

lemlib::Drivetrain drivetrain(&left_motors, 
	&right_motors, 
	10, // change
	lemlib::Omniwheel::NEW_275, 
	420,
	2
);

pros::Imu imu(10);

// odometry settings
lemlib::OdomSensors sensors(nullptr, 
	nullptr, 
	nullptr, 
	nullptr, 
	&imu
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(
	10, // proportional gain (kP)
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
lemlib::ControllerSettings angular_controller(
	2, // proportional gain (kP)
    0, // integral gain (kI)
    10, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in degrees
    100, // small error range timeout, in milliseconds
    3, // large error range, in degrees
    500, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(
	3, 
	10, 
	1.019 
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(
	3, 
	10, 
	1.019 
);

// create the chassis
lemlib::Chassis chassis(
	drivetrain,
	lateral_controller,
	angular_controller,
	sensors,
	&throttle_curve, 
	&steer_curve
);	

void on_center_button() {}


void initialize() {
	pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
	pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	while (true) {

		int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

		left_motors.move(leftY);
		right_motors.move(rightX);

		pros::delay(20);
	}
}