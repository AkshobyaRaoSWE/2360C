#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"

// check reversed motors, motor types


pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_motors({1, 2, 3}, pros::v5::MotorGears::blue); 
pros::MotorGroup right_motors({4, 5, 6},pros::v5::MotorGears::blue); 

pros::Motor intake_motor_1(7);
pros::Motor intake_motor_2(8);

pros::Motor outake_motor_1(9);

pros::adi::Pneumatics blocker1('C', false);
pros::adi::Pneumatics blocker2('D', false);
pros::adi::Pneumatics intake1('E', false);
pros::adi::Pneumatics lift('F', false);
pros::adi::Pneumatics tounge('G', false);



lemlib::Drivetrain drivetrain(
	&left_motors, 
	&right_motors, 
	10, // change
	lemlib::Omniwheel::NEW_275, 
	420,
	2
);

pros::Imu imu(10);

// odometry settings
lemlib::OdomSensors sensors(
	nullptr, 
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
	bool pistonToogle = false;
	while (true) {

		int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

		left_motors.move(leftY);
		right_motors.move(rightX);

		pros::delay(20);

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			intake_motor_1.move(600);
		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			intake_motor_1.move(-600);
		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			intake_motor_1.move(600);
		} else {
			intake_motor_1.move(0);
		}

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			intake_motor_2.move(600);
		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			intake_motor_2.move(-600);
		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			intake_motor_2.move(600);
		} else {
			intake_motor_2.move(0);
		}

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			outake_motor_1.move(300);
		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			outake_motor_1.move(-300);
		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			outake_motor_1.move(300);
		} else {
			outake_motor_1.move(0);
		}


		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
			if(pistonToogle == false){
				blocker1.extend();
				blocker2.extend();
				pros::delay(200);
				pistonToogle = true;
			}
			else{
				blocker1.retract();
				blocker2.retract();
				pros::delay(200);
				pistonToogle = false;
			}
		} 

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
			if(pistonToogle == false){
				intake1.extend();
				pros::delay(200);
				pistonToogle = true;
			}
			else{
				intake1.retract();
				pros::delay(200);
				pistonToogle = false;
			}
		}
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
			if(pistonToogle == false){
				tounge.extend();
				pros::delay(200);
				pistonToogle = true;
			}
			else{
				tounge.retract();
				pros::delay(200);
				pistonToogle = false;
			}
		}
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
			if(pistonToogle == false){
				lift.extend();
				pros::delay(200);
				pistonToogle = true;
			}
			else{
				lift.retract();
				pros::delay(200);
				pistonToogle = false;
			}
		}

		pros::delay(10);
		
	}
}