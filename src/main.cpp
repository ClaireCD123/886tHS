#include "main.h" //test 
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/llemu.hpp"
#include "pros/optical.hpp"
#include "pros/rtos.h"
Motor Lf(20);
Motor Lb(18);
Motor Rf(-16);
Motor Rb(-17);
Motor Rt(15);
Motor Lt(-19);
Controller master(E_CONTROLLER_MASTER);
MotorGroup left_side_motors({-20, -18,19}); // Creates a motor group with forwards ports 1 & 3 and reversed port 2
MotorGroup right_side_motors({16,17, -15});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
MotorGroup intake_mg({13, -9});
Motor intakeBottom(-9);
Motor intakeTop(13);
Optical optical_sensor(1);
Rotation odomVerticalPod (3);
Imu imu (4);
adi::Pneumatics piston('h', false);//clamp
adi::Pneumatics arm('g', false);//arm
adi::Pneumatics doinker('f', false);//doinker

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
// Lemlib drivetrain  struct

lemlib::Drivetrain drivetrain{
    &left_side_motors,  // left drrivetrain motors
    &right_side_motors, // right train motors
    10,                 // track width in INCHES
    3.25,               // wheel diameter
    360,                // wheel rpm
    8                   // tune this value later : )

};

// define odom pod
lemlib::TrackingWheel vertical_tracking_wheel(&odomVerticalPod, 2, 1);
// odom struct
lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, nullptr, nullptr, &imu);

// forward/backward PID
lemlib::ControllerSettings lateralController(14.35, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              40, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// turning PID
 /*lemlib::ControllerSettings angularController{

     1.5, // kP (1.6)
     0,//KI
     1, // kD (1)
     0,//windup
     2,   // smallErrorRange
     100, // smallErrorTimeout
     3,   // largeErrorRange
     250, // largeErrorTimeout
     5    // slew rate
};*/

lemlib::ControllerSettings angularController(3, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              5, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController,
                        sensors);


void redSideAWP(){
chassis.setPose(0,0,90);
   chassis.moveToPose(5,0,90, 1000);//alliance goal
   delay(1000);
   doinker.extend();
   delay(500);
   chassis.moveToPose(0,0,90, 1000, {.forwards=false});//back up from alliance
   doinker.retract();
   piston.extend();
   chassis.moveToPose(-16.5,39,135, 2500,{.forwards=false});//mobile goal
  chassis.waitUntilDone();
   piston.retract();//get mobile goal
    intake_mg.move(115);
    delay(500);
chassis.waitUntilDone();
chassis.moveToPose(-27,33,270,1900,{.maxSpeed=90});//first set of rings
chassis.moveToPose(-3,13,90,3000,{.maxSpeed=90});//second set of rings
delay(500);
doinker.extend();//hit the wanted ring off
delay(500);
 chassis.turnToHeading(50,1000,{.maxSpeed=45});//hit the ring
 delay(1000);
 intake_mg.move(115);
 chassis.moveToPose(11,23,62,500);//go pick up the ring
 doinker.retract();
 chassis.moveToPose(9,30,20,500,{.maxSpeed=50});//go to wall
chassis.moveToPose(9,40,20,500,{.maxSpeed=50});//go to wall x2
}

void blueSideAwp(){
 chassis.setPose(0,0,90);
   chassis.moveToPose(5,0,90, 1000);//alliance goal
   delay(1000);
   doinker.extend();
   delay(500);
   chassis.moveToPose(0,0,90, 1000, {.forwards=false});//back up from alliance
   doinker.retract();
   piston.extend();
   chassis.moveToPose(-15,39,135, 2500,{.forwards=false});//mobile goal
  chassis.waitUntilDone();
   piston.retract();//get mobile goal
    intake_mg.move(115);
    delay(500);
chassis.waitUntilDone();
chassis.moveToPose(-27,33,270,1900,{.maxSpeed=90});//first set of rings
chassis.moveToPose(-3,13,90,3000,{.maxSpeed=90});//second set of rings
delay(500);
doinker.extend();//hit the wanted ring off
delay(500);
 chassis.turnToHeading(50,1000,{.maxSpeed=45});//hit the ring
 delay(1000);
 intake_mg.move(115);
 chassis.moveToPose(11,23,62,500);//go pick up the ring
 doinker.retract();
 chassis.moveToPose(15,17,70,500,{.forwards=false,.maxSpeed=60});//go to wall
chassis.moveToPose(15,30,70,500);

}

void skills(){
    chassis.setPose(-12,-60,90);
  /*  chassis.moveToPose(-7,-60,90,1000);//go to alliance stake
   chassis.waitUntilDone();
   doinker.extend();
   delay(500);
   chassis.moveToPoint(-12,-60, 1000, {.forwards=false});//back up from alliance
   chassis.waitUntilDone();
    chassis.moveToPose(-12,-60,150,1000,{.forwards=false});//turn to mobile goal
   doinker.retract();
   piston.extend();
   chassis.moveToPose(-25,-46,150,1000,{.forwards=false,.lead=0.3,.maxSpeed=40});//go to mobile goal
   chassis.waitUntilDone();
   delay(1000);
   piston.retract();
   intake_mg.move(115);
   delay(500);
   chassis.turnToHeading(0,1000);
   chassis.moveToPose(-24,-30,0,1000);//go to ring
   chassis.turnToHeading(-45,1000);
   chassis.moveToPoint(-43,-2,1000,{.maxSpeed=80});//move to coordinate 
   chassis.moveToPose(-46,13,-15,1300);//move to second ring
   delay(2000);
   intake_mg.move(80);
   chassis.waitUntilDone();
    chassis.moveToPose(-52,0,-161,2000);//wallstake ring
    
    delay(3000);
   chassis.waitUntilDone();
   piston.extend();*/
}
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
            pros::lcd::print(3, "Hue value: %f", optical_sensor.get_hue());
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

void autonomous() {
    skills();
    



// intake_mg.move(115);
// chassis.waitUntilDone();
// chassis.moveToPose(18,12,90,5000,{.maxSpeed=90});//second set of rings
// chassis.waitUntil(12);
// intake_mg.move(80);
// chassis.waitUntil(28);
// intake_mg.move(115);
// chassis.moveToPose(12,36,0,2000);//touch ladder
   

	 // set position to x:0, y:0, heading:0
 /*  chassis.setPose(0, 0, 0);
intake_mg.move(-100);
   delay(1000);
   intake_mg.move(0);
   chassis.moveToPose(-2.38, 7.18, 1.88,1500);
   chassis.waitUntilDone();
    piston.extend();
   chassis.turnToHeading(110,1000);
    chassis.moveToPose(-17, 13, 108,1000,{.forwards=false, .maxSpeed=180});
	chassis.waitUntilDone();
	piston.retract();
	chassis.turnToHeading(0,1000);
	chassis.moveToPose(-20, 27.1, 0,1000);
	intake_mg.move(-100);
	delay(5000);
	intake_mg.move(0); Unfinished auto skills*/


 // set position to x:0, y:0, heading:0
 /*  chassis.setPose(0, 0, 0);
intake_mg.move(-100);
   delay(1000);
   intake_mg.move(0);
   chassis.moveToPose(-2.38, 7.18, 1.88,1500);
   chassis.waitUntilDone();
    piston.extend();
   chassis.turnToHeading(110,1000);
    chassis.moveToPose(-17, 13, 108,1000,{.forwards=false, .maxSpeed=180});
	chassis.waitUntilDone();
	piston.retract();
	chassis.turnToHeading(0,1000);
	chassis.moveToPose(-20, 27.1, 0,1000);
	intake_mg.move(-100);
	delay(5000);
	intake_mg.move(0); Unfinished auto skills*/

	// chassis.setPose(0,0,0);
	// chassis.moveToPose(-1,-15,0,1000,{.forwards=false});

//awp red

// chassis.setPose(0,0,0);
// piston.extend();
// chassis.moveToPose(0,-32,0,1300,{.forwards=false, .maxSpeed = 100});//go to first goal
// chassis.waitUntilDone();
// piston.retract();
// intake_mg.move(-127);
// delay(350);
// chassis.moveToPose(-1,-32,-90 ,500,{.forwards=true,.maxSpeed=180});//turn towards first set of rings
// intake_mg.move(-100);
// delay(200);//intake preload into goal
// chassis.moveToPose(-23,-31,-90 ,950,{.forwards=true,.maxSpeed=100});//goes to first set of rings
// delay(2100);
// piston.extend();
// intake_mg.move(-127);
// piston.extend();
// chassis.moveToPose(17 , -12.5,  90, 3000, {.forwards=true,});//goes to second set
// chassis.moveToPose(30 , -12.5,  90, 1300, {.forwards=true, .maxSpeed=95});
// chassis.waitUntil(11);
// intakeTop.move(0);
// chassis.waitUntilDone();
// chassis.turnToHeading(145,1000);
// piston.retract();
// chassis.moveToPose(2 , 0,  90, 1500, {.forwards=false, .minSpeed = 127});//adjustments go to push ring
// chassis.moveToPose(24, -2,  90, 1500, {.forwards=true, .maxSpeed=100});// go towards goal
// chassis.turnToHeading(-180,500);//turn towards goal
// chassis.moveToPose(25.7, 6, 180, 1900, {.forwards=false, .maxSpeed = 60});//go TO goal
// delay(500);
// intake_mg.move(-127);
// delay(1000);
// chassis.moveToPose(21, -25, -180, 1500, { .forwards = true , .minSpeed = 65});//go to field ladder thingy
// intake_mg.move(0);

//AWP left BLUE

// chassis.setPose(0,0,0);
// piston.extend();
// chassis.moveToPose(0,-32,0,1300,{.forwards=false, .maxSpeed = 100});//go to first goal
// chassis.waitUntilDone();
// piston.retract();
// intake_mg.move(-127);
// delay(350);
// chassis.moveToPose(1,-32,90 ,500,{.forwards=true,.maxSpeed=180});//turn towards first set of rings
// intake_mg.move(-110);
// delay(200);//intake preload into goal
// chassis.moveToPose(23,-31,90 ,950,{.forwards=true,.maxSpeed=100});//goes to first set of rings
// delay(2100);
// piston.extend();
// intake_mg.move(-127);
// piston.extend();
// chassis.moveToPose(-17 , -12,  -90, 3000, {.forwards=true,});//goes to second set
// chassis.moveToPose(-30 , -12,  -90, 1300, {.forwards=true, .maxSpeed=95});
// chassis.waitUntil(11);
// intakeTop.move(0);
// chassis.waitUntilDone();
// chassis.turnToHeading(-145,1000);
// chassis.moveToPose(-2 , 0,  -90, 1500, {.forwards=false, .minSpeed = 127});//adjustments go to push ring
// chassis.moveToPose(-22.5, -2,  -90, 1500, {.forwards=true, .maxSpeed=100});// go towards goal
// chassis.turnToHeading(180,500);//turn towards goal
// chassis.moveToPose(-22.5 , 6,  180, 1500, {.forwards=false});//go TO goal
// intake_mg.move(-127);
// delay(1500);
// chassis.moveToPose(-21, -25, 180, 1500, { .forwards = true , .minSpeed = 127});//go to field ladder thingy
// intake_mg.move(0);

// }

//Auto BLUE RIGHT SIDE
/*
chassis.setPose(0,0,0);
piston.extend();
chassis.moveToPose(0,-32,0,1300,{.forwards=false, .maxSpeed = 100});//go to first goal
chassis.waitUntilDone();
piston.retract();
intake_mg.move(-127);
delay(350);
chassis.moveToPose(-1,-32,-90 ,500,{.forwards=true,.maxSpeed=180});//turn towards first set of rings
intake_mg.move(-100);
delay(200);//intake preload into goal
chassis.moveToPose(-23,-31,-90 ,950,{.forwards=true,.maxSpeed=100});//goes to first set of rings
delay(2100);
piston.extend();
intake_mg.move(0);
*/
//auto red left side

// chassis.setPose(0,0,0);
// piston.extend();
// chassis.moveToPose(0,-32,0,1300,{.forwards=false, .maxSpeed = 100});//go to first goal
// chassis.waitUntilDone();
// piston.retract();
// intake_mg.move(-127);
// delay(350);
// chassis.moveToPose(1,-32,90 ,500,{.forwards=true,.maxSpeed=180});//turn towards first set of rings
// intake_mg.move(-110);
// delay(200);//intake preload into goal
// chassis.moveToPose(23,-31,90 ,950,{.forwards=true,.maxSpeed=100});//goes to first set of rings
// delay(2100);
// piston.extend();
// intake_mg.move(0);


 /* Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.*/
 

/*chassis.setPose(0, 0, 0);
piston.set_value(false);
chassis.moveToPose(0, -30, 0, 3000, {false, 8,0, 45, 40});
chassis.waitUntil(26);
piston.set_value(true);
intake_mg.move (-100);
delay(1000);
chassis.turnToHeading(-100, 700, {.maxSpeed = 50, .minSpeed = 40});
delay (700);
chassis.moveToPoint(-10, -25, 2000);*/
}


void opcontrol() {

	bool pstate = true;
    bool astate = true;
    bool dstate = true;
    int starttime=0;
	while (true)
	{
     
		    // loop forever
        // get left y and right x positions
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);

        // delay to save resources
        pros::delay(25);
	
		// Arcade control scheme
		// int dir = master.get_analog(ANALOG_LEFT_Y);	  // Gets amount forward/backward from left joystick
		// int turn = .90 * master.get_analog(ANALOG_RIGHT_X); // Gets the turn left/right from right joystick
		// left_side_motors.move(dir + turn);					  // Sets left motor voltage
		// right_side_motors.move(dir - turn);					  // Sets right motor voltage


		if (master.get_digital(DIGITAL_R1))
		{
          
			intake_mg.move(115);
            
		}
		else if (master.get_digital(DIGITAL_R2))
		{
			intake_mg.move(-115);
		}
		else
		{
			intake_mg.move(0);
		}

		if (master.get_digital_new_press(DIGITAL_A)){//clamp
			pstate = !pstate;
		}

		if (pstate == false){
			piston.extend();
		}
		else {
			piston.retract();
		}
        if (master.get_digital_new_press(DIGITAL_B)){//ARM
			astate = !astate;
		}

		if (astate == false){
			arm.extend();
		}
		else {
			arm.retract();
		}
         if (master.get_digital_new_press(DIGITAL_Y)){//DOINKER
			dstate = !dstate;
		}

		if (dstate == false){
			doinker.extend();
		}
		else {
			doinker.retract();
		}

		// if (master.get_digital(DIGITAL_A))
		// {
		// 	piston.extend();
		// }
		// if (master.get_digital(DIGITAL_B))
		// {
		// 	piston.retract();
		// }
		delay(20); // Run for 20 ms then update
	}
}