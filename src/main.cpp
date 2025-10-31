#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"

// check reversed motors, motor types
// make sure to specificy the motor type(green, red, etc.)
// if you need anything to be faster change the delays to be less

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_motors({1, 2, 3}, pros::v5::MotorGears::blue); // to change the direction the motor spins, you can just put a negative in front of the port number
// pros::MotorGroup left_motors({1, -2, -3}, pros::v5::MotorGears::red); example
pros::MotorGroup right_motors({4, 5, 6},pros::v5::MotorGears::blue); 

pros::Motor intake_motor_1(7); // need to specify the motor type
pros::Motor intake_motor_2(8); // need to specify the motor type

pros::Motor outake_motor_1(9); // need to specify the motor type

pros::adi::Pneumatics blocker1('C', false); // if the pneumatics logic doesn't work, perhaps the piston is starting at an extended position, so try changing false to true
pros::adi::Pneumatics blocker2('D', false);
pros::adi::Pneumatics intake1('E', false);
pros::adi::Pneumatics lift('F', false);
pros::adi::Pneumatics tounge('G', false);



lemlib::Drivetrain drivetrain(
	&left_motors, 
	&right_motors, 
	10, // measure the distance between the left and right wheels
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
	
	bool intakeToogle = false;
	bool toungeToogle = false;
	bool liftToogle = false;
	bool blockerToogle = false;
	while (true) {

		int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

		left_motors.move(leftY);
		right_motors.move(rightY);

		pros::delay(20);

		// pressing r1 will move the intake motor forward, r2 will move it backward, l1 will move it forward
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			intake_motor_1.move(600);
		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			intake_motor_1.move(-600);
		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			intake_motor_1.move(600);
		} else {
			intake_motor_1.move(0);
		}

		// to change the button, you just say change pros::E_CONTROLLER_DIGITAL_R2 to pros::E_CONTROLLER_DIGITAL_R1, or pros::E_CONTROLLER_DIGITAL_UP, etc.
		// to make the motor move at a speed, just say the motorName.move -> e.g. intake_motor_1.move(600); where 600 is the speed, 127 is actually the max, so anything above that will be set to the max speed of 127 e,g. 300 = 127

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

		// if the toggle isn't working, try changing master.get_digital_new_press to just master.get_digital
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
			if(blockerToogle == false){
				blocker1.extend();
				blocker2.extend();
				pros::delay(200);
				blockerToogle = true;
			}
			else{
				blocker1.retract();
				blocker2.retract();
				pros::delay(200);
				blockerToogle = false;
			}
		} 

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
			if(intakeToogle == false){
				intake1.extend();
				pros::delay(200);
				intakeToogle = true;
			}
			else{
				intake1.retract();
				pros::delay(200);
				intakeToogle = false;
			}
		}

		// every time the button is pressed, the code checks if the piston is extended or not, if it is not, it extends it, if it is, it retracts it
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
			if(toungeToogle == false){
				tounge.extend();
				pros::delay(200);
				toungeToogle = true;
			}
			else{
				tounge.retract();
				pros::delay(200);
				toungeToogle = false;
			}
		}

		// if you want the lift to extend from a different button, you can just change the pros::E_CONTROLLER_DIGITAL_DOWN to the button you want. e.g. pros::E_CONTROLLER_DIGITAL_UP, pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT, etc.

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
			if(liftToogle == false){
				lift.extend();
				pros::delay(200); // note: all delays are in milliseconds
				liftToogle = true;
			}
			else{
				lift.retract();
				pros::delay(200);
				liftToogle = false;
			}
		}

		// if the toggles aren't working, try this instead: 
		// // Separate toggle variables for each mechanism
		

		// if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
		// 	blockerToogle = !blockerToogle;
			
		// 	if(blockerToogle){
		// 		blocker1.extend();
		// 		blocker2.extend();
		// 	}
		// 	else{
		// 		blocker1.retract();
		// 		blocker2.retract();
		// 	}
		// }

		// if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
		// 	intakeToogle = !intakeToogle;
			
		// 	if(intakeToogle){
		// 		intake1.extend();
		// 	}
		// 	else{
		// 		intake1.retract();
		// 	}
		// }

		// if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){  // Changed button
		// 	toungeToogle = !toungeToogle;
			
		// 	if(toungeToogle){
		// 		tounge.extend();
		// 	}
		// 	else{
		// 		tounge.retract();
		// 	}
		// }

		// if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){  // Changed button
		// 	liftToogle = !liftToogle;
			
		// 	if(liftToogle){
		// 		lift.extend();
		// 	}
		// 	else{
		// 		lift.retract();
		// 	}
		// }

		// or if you want everything in one toggle do this:
		// bool pistonToogle = false;
		// if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
		// 	pistonToogle = !pistonToogle;
			
		// 	if(pistonToogle){
		// 		// All extend together
		// 		blocker1.extend();
		// 		blocker2.extend();
		// 		intake1.extend();
		// 		tounge.extend();
		// 		lift.extend();
		// 	}
		// 	else{
		// 		// All retract together
		// 		blocker1.retract();
		// 		blocker2.retract();
		// 		intake1.retract();
		// 		tounge.retract();
		// 		lift.retract();
		// 	}
		// }
		pros::delay(10);
		
	}
}

// below is a commented copy of the code, it is not runnable, but it is a good way to see how the code is structured
// #include "main.h"
// #include "lemlib/api.hpp" // IWYU pragma: keep
// #include "lemlib/chassis/trackingWheel.hpp"
// #include "pros/abstract_motor.hpp"
// #include "pros/adi.hpp"
// #include "pros/motors.hpp"

// // check reversed motors, motor types


// pros::Controller master(pros::E_CONTROLLER_MASTER);

// pros::MotorGroup left_motors({1, 2, 3}, pros::v5::MotorGears::blue); 
// pros::MotorGroup right_motors({4, 5, 6},pros::v5::MotorGears::blue); 

// pros::Motor intake_motor_1(7);
// pros::Motor intake_motor_2(8);

// pros::Motor outake_motor_1(9);

// pros::adi::Pneumatics blocker1('C', false);
// pros::adi::Pneumatics blocker2('D', false);
// pros::adi::Pneumatics intake1('E', false);
// pros::adi::Pneumatics lift('F', false);
// pros::adi::Pneumatics tounge('G', false);



// lemlib::Drivetrain drivetrain(
// 	&left_motors, 
// 	&right_motors, 
// 	10, // change
// 	lemlib::Omniwheel::NEW_275, 
// 	420,
// 	2
// );

// pros::Imu imu(10);

// // odometry settings
// lemlib::OdomSensors sensors(
// 	nullptr, 
// 	nullptr, 
// 	nullptr, 
// 	nullptr, 
// 	&imu
// );

// // lateral PID controller
// lemlib::ControllerSettings lateral_controller(
// 	10, // proportional gain (kP)
//     0, // integral gain (kI)
//     3, // derivative gain (kD)
//     3, // anti windup
//     1, // small error range, in inches
//     100, // small error range timeout, in milliseconds
//     3, // large error range, in inches
//     500, // large error range timeout, in milliseconds
//     20 // maximum acceleration (slew)
// );

// // angular PID controller
// lemlib::ControllerSettings angular_controller(
// 	2, // proportional gain (kP)
//     0, // integral gain (kI)
//     10, // derivative gain (kD)
//     3, // anti windup
//     1, // small error range, in degrees
//     100, // small error range timeout, in milliseconds
//     3, // large error range, in degrees
//     500, // large error range timeout, in milliseconds
//     0 // maximum acceleration (slew)
// );

// // input curve for throttle input during driver control
// lemlib::ExpoDriveCurve throttle_curve(
// 	3, 
// 	10, 
// 	1.019 
// );

// // input curve for steer input during driver control
// lemlib::ExpoDriveCurve steer_curve(
// 	3, 
// 	10, 
// 	1.019 
// );

// // create the chassis
// lemlib::Chassis chassis(
// 	drivetrain,
// 	lateral_controller,
// 	angular_controller,
// 	sensors,
// 	&throttle_curve, 
// 	&steer_curve
// );	

// void on_center_button() {}

// void initialize() {
// 	pros::lcd::initialize(); // initialize brain screen
//     chassis.calibrate(); // calibrate sensors
// 	pros::Task screen_task([&]() {
//         while (true) {
//             // print robot location to the brain screen
//             pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
//             pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
//             pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
//             // delay to save resources
//             pros::delay(20);
//         }
//     });
// }

// void disabled() {}

// void competition_initialize() {}

// void autonomous() {}

// void opcontrol() {
// 	bool pistonToogle = false;
// 	while (true) {

// 		int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
// 		int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

// 		left_motors.move(leftY);
// 		right_motors.move(rightX);

// 		pros::delay(20);

// 		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
// 			intake_motor_1.move(600);
// 		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
// 			intake_motor_1.move(-600);
// 		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
// 			intake_motor_1.move(600);
// 		} else {
// 			intake_motor_1.move(0);
// 		}

// 		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
// 			intake_motor_2.move(600);
// 		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
// 			intake_motor_2.move(-600);
// 		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
// 			intake_motor_2.move(600);
// 		} else {
// 			intake_motor_2.move(0);
// 		}

// 		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
// 			outake_motor_1.move(300);
// 		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
// 			outake_motor_1.move(-300);
// 		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
// 			outake_motor_1.move(300);
// 		} else {
// 			outake_motor_1.move(0);
// 		}


// 		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
// 			if(pistonToogle == false){
// 				blocker1.extend();
// 				blocker2.extend();
// 				pros::delay(200);
// 				pistonToogle = true;
// 			}
// 			else{
// 				blocker1.retract();
// 				blocker2.retract();
// 				pros::delay(200);
// 				pistonToogle = false;
// 			}
// 		} 

// 		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
// 			if(pistonToogle == false){
// 				intake1.extend();
// 				pros::delay(200);
// 				pistonToogle = true;
// 			}
// 			else{
// 				intake1.retract();
// 				pros::delay(200);
// 				pistonToogle = false;
// 			}
// 		}
// 		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
// 			if(pistonToogle == false){
// 				tounge.extend();
// 				pros::delay(200);
// 				pistonToogle = true;
// 			}
// 			else{
// 				tounge.retract();
// 				pros::delay(200);
// 				pistonToogle = false;
// 			}
// 		}
// 		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
// 			if(pistonToogle == false){
// 				lift.extend();
// 				pros::delay(200);
// 				pistonToogle = true;
// 			}
// 			else{
// 				lift.retract();
// 				pros::delay(200);
// 				pistonToogle = false;
// 			}
// 		}

// 		pros::delay(10);
		
// 	}
// }