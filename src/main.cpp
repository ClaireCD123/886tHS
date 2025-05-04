#include "main.h" //test 
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/llemu.hpp"
#include "pros/optical.hpp"
#include "pros/rtos.h"
Motor L1(-2);
Motor L2(3);
Motor R1(10);
Motor R2(-7);
Motor R3(19);
Motor L3(-4);
Controller master(E_CONTROLLER_MASTER);
MotorGroup left_side_motors({2, 4,-3}); // Creates a motor group with forwards ports 1 & 3 and reversed port 2
MotorGroup right_side_motors({-10,-19, 7});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
MotorGroup intake_mg({-5,9,21});
Motor intakeBottom(21);
MotorGroup intake_top({-5,9});
Motor l_arm(5);
Motor r_arm(-9);
Optical colorsorter(12);
Rotation odomVerticalPod(18);
Rotation odomHorizontalPod(13);
Rotation rotation_sensor(1);
Imu imu (6);
adi::Pneumatics piston('a', false);//clamp+ring rush doinker
adi::Pneumatics goaldoinker('b', false);//goal rush doinkerdoinker
adi::Pneumatics ringdoinker('c',false);

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
lemlib::TrackingWheel vertical_tracking_wheel(&odomVerticalPod, lemlib::Omniwheel::NEW_2,2.5);
lemlib::TrackingWheel horizontal_tracking_wheel(&odomHorizontalPod,lemlib::Omniwheel::NEW_2, -1.5);
// odom struct
lemlib::OdomSensors sensors(&vertical_tracking_wheel,  nullptr, &horizontal_tracking_wheel,nullptr, &imu);

// forward/backward PID
lemlib::ControllerSettings lateralController(14.35, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              40, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);
/*lemlib::ControllerSettings lateralController(14.35, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              40, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);*/
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

lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              11, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController,
                        sensors);
 bool color_state = true;


void color_sort(){
bool side = true; //blue = false; red = true
bool check = true;
bool on = true;
   while(check = true){
       double hue = colorsorter.get_hue();
       colorsorter.set_led_pwm(100);
       if(on == true){
       if (hue >= 150 && hue <= 230 ){ //BLUE
            //if (intake_mg.get_current_draw() >= 2400){
                check = true;
                //intake_mg.move(0);
            //}
       }
       else if (hue <= 30 && on == true){ // RED

                //pros:: delay (700);
                intake_mg.move(70);
                on == false;
            
        }
        }
        pros::delay(10);
        }
     
   }

   bool isBlueDetected() {
    double hue = colorsorter.get_hue();
    return (hue > 100 && hue < 240);  // Adjust based on testing
}
void colorSortTask() {
    bool handlingblue = false;
    bool bluePreviouslyDetected = false;
    uint32_t blueDetectedTime = 0;

    enum class State {
        RUNNING,
        DELAY_BEFORE_STOP,
        STOPPING,
        RESUMED
    };

    State currentState = State::RUNNING;

    while (true) {
        bool blueNow = isBlueDetected();
        uint32_t now = pros::millis();

        switch (currentState) {
            case State::RUNNING:

                if (blueNow && !bluePreviouslyDetected) {
                    blueDetectedTime = now;
                    currentState = State::DELAY_BEFORE_STOP;
                    pros::lcd::print(6,"blue dectected");
                }
                break;

            case State::DELAY_BEFORE_STOP:
                intake_top.move(127);  // Keep running
                if (now - blueDetectedTime >=15) {  // 1 sec passed
                    blueDetectedTime = now;
                    currentState = State::STOPPING;
                
                }
                break;

            case State::STOPPING:
                intake_top.move(-10);  // Stop motor
                if (now - blueDetectedTime >= 500) {  // 0.5 sec passed
                    currentState = State::RESUMED;
                }
                break;

            case State::RESUMED:
                intake_top.move(127);  // Resume motor
                currentState = State::RUNNING;
                break;
        }

        bluePreviouslyDetected = blueNow;
        pros::delay(10);
    }
}

//    bool isRedDetected() {
//     double hue = colorsorter.get_hue();
//     return (hue > 0 && hue < 30);  // Adjust based on testing
// }
// void colorSortTask() {
//     bool handlingRed = false;
//     bool redPreviouslyDetected = false;
//     uint32_t redDetectedTime = 0;

//     enum class State {
//         RUNNING,
//         DELAY_BEFORE_STOP,
//         STOPPING,
//         RESUMED
//     };

//     State currentState = State::RUNNING;

//     while (true) {
//         bool redNow = isRedDetected();
//         uint32_t now = pros::millis();

//         switch (currentState) {
//             case State::RUNNING:

//                 if (redNow && !redPreviouslyDetected) {
//                     redDetectedTime = now;
//                     currentState = State::DELAY_BEFORE_STOP;
//                     pros::lcd::print(6,"red dectected");
//                 }
//                 break;

//             case State::DELAY_BEFORE_STOP:
//                 intake_top.move(127);  // Keep running
//                 if (now - redDetectedTime >=102) {  // 1 sec passed
//                     redDetectedTime = now;
//                     currentState = State::STOPPING;
                
//                 }
//                 break;

//             case State::STOPPING:
//                 intake_top.move(-10);  // Stop motor
//                 if (now - redDetectedTime >= 500) {  // 0.5 sec passed
//                     currentState = State::RESUMED;
//                 }
//                 break;

//             case State::RESUMED:
//                 intake_top.move(127);  // Resume motor
//                 currentState = State::RUNNING;
//                 break;
//         }

//         redPreviouslyDetected = redNow;
//         pros::delay(10);
//     }
// }


// void redSideAWP(){
// chassis.setPose(0,0,90);
//    chassis.moveToPose(5,0,90, 1000);//alliance goal
//    chassis.waitUntilDone();
//    doinker.extend();
//    delay(500);
//    chassis.moveToPose(0,0,90, 1000, {.forwards=false});//back up from alliance
//    delay(500);
//    doinker.retract();
//    piston.extend();
//    chassis.moveToPose(-16.5,39,135, 2500,{.forwards=false});//mobile goal
//   chassis.waitUntilDone();
//   delay(500);
//    piston.retract();//get mobile goal
//    chassis.waitUntilDone();
//    delay(500);
//     intake_mg.move(105);
//     delay(500);
// chassis.waitUntilDone();
// chassis.moveToPose(-28,33,270,1900,{.maxSpeed=80});//first set of rings
// chassis.waitUntilDone();
// chassis.moveToPose(-3,13,90,3000,{.maxSpeed=60});//second set of rings
// delay(500);
// doinker.extend();//hit the wanted ring off
// delay(500);
//  chassis.turnToHeading(50,1000,{.maxSpeed=45});//hit the ring
//  delay(1000);
//  intake_mg.move(105);
//  chassis.moveToPose(15,23,66,700);//go pick up the ring
//  doinker.retract();
//  chassis.moveToPose(9,30,20,500,{.maxSpeed=50});//go to wall
//  delay(1500);
// intake_mg.move(0);
// chassis.moveToPoint(9,55,500,{.maxSpeed=40});//go to wall x2

// }

// void blueSideAwp(){
//  chassis.setPose(0,0,90);
//  delay(2000);
//  chassis.waitUntilDone();
//    chassis.moveToPose(5,0,90, 1000, {.maxSpeed=40});//alliance goal
//    chassis.waitUntilDone();
//    doinker.extend();
//    delay(500);
//    chassis.moveToPose(0,0,90, 700, {.forwards=false});//back up from alliance
//    doinker.retract();
//    piston.extend();
//    chassis.moveToPose(-18,41,135, 1500,{.forwards=false,.maxSpeed=110});//mobile goal
//   chassis.waitUntilDone();
//   delay(400);
//    piston.retract();//get mobile goal
//    chassis.waitUntilDone();
//    delay(400);
//     intake_mg.move(95);
// chassis.waitUntilDone();
// chassis.moveToPose(-28,34,270,1400,{.maxSpeed=80});//first set of rings
// chassis.waitUntilDone();
// delay(500);
// chassis.moveToPose(-3,13,90,2600,{.maxSpeed=60});//second set of rings
// chassis.waitUntilDone();
// doinker.extend();//hit the wanted ring off
// delay(500);
//  chassis.turnToHeading(50,800,{.maxSpeed=30});//hit the ring
//  delay(1000);
//  intake_mg.move(95);
//  chassis.moveToPose(15,23,75,800);//go pick up the ring
//  doinker.retract();
//  chassis.waitUntilDone();
//  delay(800);
//  chassis.moveToPose(10,7,70,1000,{.forwards=false,.maxSpeed=40});//go to wall


// }
void sixred(){
  chassis.setPose(0,0,0);
   piston.extend();
   chassis.moveToPose(7,-35,-16,2000, {.forwards=false, .lead=0.8, .maxSpeed=60});//mobile goal
  chassis.waitUntil(34.5);
   piston.retract();//get mobile goal
    delay(1000);
   intake_mg.move(127);
    chassis.turnToHeading(93,500);
    chassis.moveToPose(33,-48, 95,2000, {.maxSpeed=60});//go to first set
    chassis.moveToPoint(40,-47,1000);
     chassis.waitUntilDone();
    chassis.moveToPose(39,-41,4,500);
    delay(1000);
   // chassis.moveToPose(45,-4,3,1000, {.maxSpeed = 90});//go to second set
   chassis.waitUntilDone();
   chassis.moveToPose(36,-20,4,1000,{.maxSpeed=80});//go to second set
    delay(1000);
    chassis.waitUntilDone();
    chassis.turnToHeading(45,500);
    chassis.moveToPose(60,5,45,2500,{.minSpeed=200});//third set
    chassis.waitUntilDone();
    delay(500);
    chassis.moveToPose(63,11,45,1000,{.minSpeed=127});
    delay(2000);
    chassis.waitUntilDone();
    chassis.moveToPoint(48,-10,1000,{.forwards=false});
    chassis.turnToHeading(-90,500);
    chassis.moveToPose(5,-13,-90,2000);//go to last set
    chassis.waitUntilDone();
    chassis.moveToPose(8,-11,-90,1000,{.forwards=false});
    chassis.waitUntilDone();
    chassis.moveToPose(-8,-11,-90,2500);

   

    // chassis.moveToPose(14,-9.5,-90,2000);//go to last set
    // chassis.waitUntilDone();
    // goaldoinker.extend();
    // chassis.waitUntilDone();
    // delay(500);
    // chassis.turnToHeading(-140,500);//off top
    // chassis.waitUntilDone();
    // chassis.moveToPose(-7,-22,-117,1000);//collect top ring


    
    // chassis.waitUntil(45);
    // goaldoinker.retract();
    // chassis.waitUntilDone();
    // chassis.moveToPose(60,-20,40,1000);//third set
    // chassis.moveToPose(54,9,45,900,{.minSpeed=200});//chese
    // chassis.waitUntilDone();
    //  delay(1200);
    // chassis.moveToPose(30,-13,-85,800);
    // chassis.waitUntilDone();
    // goaldoinker.extend();
    // intake_mg.move(127);
    
    // chassis.moveToPose(1,-14,-90,2000);//go to last set
    // chassis.waitUntilDone();
    // delay(500);
    // chassis.turnToHeading(-150,500);//off top
    // chassis.waitUntilDone();
    // chassis.moveToPose(-9,-25,-110,1000);//collect top ring
    // chassis.moveToPose(-9,-35,250,500,{.maxSpeed=50});//go to wall x2
    // chassis.turnToHeading(-130,500);



 }

void fiveblue(){
chassis.setPose(0,0,0);
 piston.extend();
 chassis.moveToPose(-8,-36,15,2000, {.forwards=false, .lead=0.5, .maxSpeed=50});//mobile goal
 chassis.waitUntil(33);
 piston.retract();//get mobile goal
 delay(1000);
   intake_mg.move(127);
   chassis.turnToHeading(-95,500);

//        delay(500);
//     intake_mg.move(120);
chassis.moveToPose(-30,-53,-95,2000, {.maxSpeed=60});//go to first set

 chassis.moveToPoint(-45,-50,1800, {.maxSpeed = 85});
    chassis.waitUntilDone();
    chassis.turnToHeading(-3,500);
    delay(300);
    goaldoinker.extend();
    chassis.waitUntilDone();
    delay(1000);
    chassis.turnToHeading(45,800,{.maxSpeed=90});//hit ring off
    chassis.waitUntilDone();
    delay(500);
    chassis.turnToHeading(20,800);
    chassis.moveToPose(-37,-25,20,1000,{.maxSpeed=80});//go to second set
    chassis.waitUntilDone();
    delay(1000);
//     chassis.moveToPose(-40,-34,-5,800, {.maxSpeed = 70});//go to second set
//     chassis.turnToHeading(20,500,{.maxSpeed=100});
//     chassis.waitUntil(45);
//     doinker.retract();
//     chassis.waitUntilDone();
//     chassis.moveToPose(-36,-20,-35,800, {.minSpeed = 50});// go to 3rd set
//     chassis.moveToPose(-52,2,-38,2300,{.minSpeed=90});//confirmation movement
//     chassis.moveToPoint(-54,4,500,{.minSpeed=100});
//     delay(500);
//     chassis.turnToHeading(115,500);
//     intakeTop.move(115);
    chassis.moveToPose(-1,-14,90,2000);//go to last set
    chassis.waitUntilDone();
    delay(500);
    chassis.turnToHeading(150,500);//off top
    chassis.waitUntilDone();
    chassis.moveToPose(9,-25,110,1000);//collect top ring
    chassis.moveToPose(9,-35,-250,500,{.maxSpeed=50});//go to wall x2
    chassis.turnToHeading(130,500);


}

// void otherblue(){
//     chassis.setPose(0,0,0);
//     piston.extend();
// chassis.moveToPose(0,-36,0,1300,{.forwards=false, .maxSpeed = 100});//go to first goal
// chassis.waitUntilDone();
// piston.retract();
// delay(500);
// intake_mg.move(105);
// delay(350);
// chassis.moveToPose(-1,-36,-90 ,500,{.forwards=true,.maxSpeed=180});//turn towards first set of rings
// intake_mg.move(105);
// delay(200);//intake preload into goal
// chassis.moveToPose(-23,-36,-90 ,950,{.forwards=true,.maxSpeed=100});//goes to first set of rings
// delay(2100);
// chassis.moveToPose(-21,-50,0,1100);//go to second set

// }
// void otherotherred(){
//     chassis.setPose(0,0,90);
//    chassis.moveToPose(4,0,90, 1000);//alliance goal
//    chassis.waitUntilDone();
//    doinker.extend();
//    delay(500);
//    chassis.moveToPose(0,0,90, 1000, {.forwards=false});//back up from alliance
//    delay(500);
//    doinker.retract();
//    intake_mg.move(127);
//    piston.extend();
//    chassis.moveToPose(18,2,86,700,{.maxSpeed=50});//drive through blue ring
//    chassis.moveToPose(13,2,86,800,{.forwards=false});
//    delay(900);
//    chassis.moveToPose(23,2,85,800,{.maxSpeed=50});
//    chassis.moveToPose(39,10,85,1000,{.maxSpeed=70});
//    chassis.waitUntilDone();
// chassis.turnToHeading(180,500);
// chassis.moveToPose(36,37,180,1000,{.forwards=false,.maxSpeed=55});//go to mobile goal
// intake_mg.move(0);
// chassis.waitUntilDone();
//  piston.retract();
//  chassis.waitUntilDone();
//  delay(500);
//   chassis.turnToHeading(90,500);
//   intake_mg.move(95);
//   chassis.waitUntilDone();
// chassis.moveToPose(55,37,90,1000,{.minSpeed=100});//first set of rings
//    chassis.waitUntilDone();
//    chassis.moveToPose(50,35,90,500,{.forwards=false});
//    delay(1700);
//    chassis.waitUntilDone();
//    intake_mg.move(0);
//    piston.extend();
//    chassis.moveToPose(58.5,35,90,500);
//    chassis.turnToHeading(180,500);
//    chassis.moveToPose(59,54.3,180,1000,{.forwards=false,.maxSpeed=55});//second mobile goal
//    chassis.waitUntilDone();
//    piston.retract();
//    chassis.waitUntilDone();
//    delay(500);
//    intake_mg.move(100);
// chassis.moveToPose(80,-7,120,900,{.minSpeed=200});//corner rings
// chassis.waitUntilDone();
// chassis.moveToPose(120,-40,120,1000,{.minSpeed=200});
// chassis.waitUntilDone();
// delay(700);
// chassis.moveToPose(60,14,120,500,{.forwards=false});//back up
// chassis.turnToHeading(-90,500);
//    doinker.extend();
//    chassis.waitUntilDone();
//    chassis.moveToPose(20,12,-90,1200, {.minSpeed=90});//set of rings
//    chassis.turnToHeading(-110,500);
//    chassis.moveToPose(6,0,-100,1000);
//    delay(500);
//    chassis.waitUntilDone();
//     intakeBottom.move(0);
//    chassis.moveToPoint(14,23,700,{.minSpeed=120});
    
// //    chassis.moveToPose(10,6,235,500);
// // chassis.moveToPose(5,4,200,1000);
// //  chassis.waitUntilDone();
// //     delay(500);
// //     doinker.retract();
    

// }
// void otherred(){
//     chassis.setPose(0,0,0);
//     piston.extend();
// chassis.moveToPose(0,-36,0,1500,{.forwards=false, .maxSpeed = 65});//go to first goal
// chassis.waitUntilDone();
// piston.retract();
// intake_mg.move(105);
// chassis.waitUntilDone();
// delay(500);
// chassis.moveToPose(-1,-36,-70,500,{.forwards=true,.maxSpeed=100});//turns to first set of rings
// intake_mg.move(105);
// delay(200);
// chassis.waitUntilDone();
// chassis.moveToPose(-16,-36,-70,1000,{.forwards=true,.maxSpeed=100}); //goes to first set of rings
// chassis.waitUntilDone();
// delay(500);
// chassis.moveToPose(-10,-36,-70,500,{.forwards=false});//back up
// chassis.waitUntilDone();
// piston.extend();
// chassis.turnToHeading(0,500);
// chassis.waitUntilDone();
// chassis.moveToPose(-21,-56,0,1100,{.forwards=false,.maxSpeed=60});//go to second goal
// chassis.waitUntilDone();
// delay(800);
// piston.retract();
// chassis.waitUntilDone();
// doinker.extend();
//  chassis.moveToPose(10,-10,125,3000,{.lead=0.4,.maxSpeed=70});//go to second set
// delay(500);
// chassis.moveToPose(10,-10,125,500,{.maxSpeed=30});//knock it off
//  delay(500);
// chassis.moveToPose(26,-17,130,1000,{.maxSpeed=100});//pick up ring
// chassis.turnToHeading(123,500,{.maxSpeed=30}); //touch tower


// }

// void skills(){
//     //unfinished wallstake
//     chassis.setPose(0,-60,0);
//    intake_mg.move(127);//alliance stake
//    delay(900);
//  chassis.moveToPose(0,-45,0,500);
//  chassis.waitUntilDone();
//  intake_mg.move(0);
//  chassis.turnToHeading(90,500);
//  piston.extend();
//  chassis.waitUntilDone();
//     chassis.moveToPose(-26,-44,90,1000,{.forwards=false,.maxSpeed=50});//go to mobile goal
//    chassis.waitUntilDone();
//    delay(500);
//    piston.retract();
//    chassis.waitUntilDone();
//    delay(500);
//    intake_mg.move(127);
//    chassis.turnToHeading(0,800);
//    chassis.moveToPose(-22,-30,0,800, {.maxSpeed=80 });//go to ring
//    chassis.waitUntilDone();
//    delay(500);
//    chassis.turnToHeading(-45,500);
//    chassis.moveToPoint(-39,-2,900,{.maxSpeed=80});//move to coordinate 
//     intakeTop.move(0);
//    chassis.moveToPose(-43,19,-10,1000);//move to second ring
//    chassis.waitUntil(23);
//     intakeTop.move(60); //turn your second intake on #2
//     delay(988);
//     intakeTop.move(-80); //ring gets loaded and unload #2
//     delay(800);
//     intakeTop.move(0);
//     arm.extend();
//     chassis.waitUntilDone();
//     chassis.moveToPoint(-37,-2,1000,{.forwards=false,.maxSpeed=50});//back up to wallstake ring
//     chassis.waitUntilDone();
//     chassis.turnToHeading(-90,800);
//     chassis.waitUntilDone();
//     chassis.moveToPose(-100,9,-83,1500,{.minSpeed=90});// go to wallstake
//     chassis.waitUntilDone();
//     delay(500);
//     intakeBottom.move(105);
//     delay(800);
//     chassis.moveToPose(-38,-2,0,500,{.forwards=false});//back up
//     chassis.waitUntilDone();
//     delay(500);
//     arm.retract();
//      intake_mg.move(105);
//      chassis.waitUntilDone();
//     chassis.turnToHeading(180,500);
// chassis.moveToPose(-43,-57,180,3200, {.maxSpeed=50});//move down to set of rings
// chassis.waitUntilDone();
// chassis.moveToPoint(-34,-37,1000,{.forwards=false});
// chassis.moveToPose(-56,-42,-90,1200, {.maxSpeed=70});//other ring
// chassis.waitUntilDone();
// chassis.moveToPose(-54,-56,45,2000, {.forwards=false, .maxSpeed=70});//corner
// chassis.waitUntilDone();
//  piston.extend();//drop goal
// intake_mg.move(0);
//  delay(500);
//  chassis.moveToPoint(-48,-48,500);
//  chassis.turnToHeading(90,500);
//  chassis.moveToPose(-10,-47,-90,1000,{.forwards=false});
//    chassis.moveToPose(31,-48,-90,2300,{.forwards=false,.maxSpeed=70});//go to second mobile goal #2
//     chassis.waitUntilDone();
//    piston.retract();
//   chassis.waitUntilDone();
//     delay(500);
//     intake_mg.move(110);
//    chassis.moveToPose(26,-23,0,1000);//go to ring #2
//    chassis.waitUntilDone();
//    delay(500);
//    chassis.turnToHeading(45,500);
//    chassis.moveToPoint(38,-6,900);//move to coordinate #2
//    chassis.moveToPose(46,24,0,2000);//move to second ring #2
//     chassis.waitUntil(23);
//     intake_mg.move(0);
//     intake_mg.move(60); //turn your second intake on #2
//     delay(998);
//     intakeTop.move(-80); //ring gets loaded and unload #2
//     delay(800);
//     intakeTop.move(0);
// arm.extend();
//     chassis.waitUntilDone();
//     delay(1000);
//     intakeBottom.move(110);
//     chassis.moveToPose(45,-6,-3,1000,{.forwards=false,.maxSpeed=50});//back up to wallstake ring #2
//     chassis.waitUntilDone();
//     chassis.turnToHeading(90,700);
//     chassis.waitUntilDone();
//     chassis.moveToPose(89,-2,87,1000,{.maxSpeed=100});// go to wallstake #2
//     chassis.waitUntilDone();
//     delay(500);
//     chassis.moveToPose(40,-2,0,500,{.forwards=false});//back up #2
//     chassis.waitUntilDone();
//     arm.retract();
//     intake_mg.move(110);
//     chassis.turnToHeading(180,500);
//    chassis.moveToPose(50,-60,180,3200, {.maxSpeed=60});//move down to set of rings #2
//    chassis.waitUntilDone();
//    chassis.moveToPoint(43,-44,700,{.forwards=false});
//    chassis.waitUntilDone();
//    chassis.moveToPose(58,-47,90,800, {.maxSpeed=70});//other ring #2
//    chassis.waitUntilDone();
//    delay(500);
//    chassis.moveToPose(70,-80,-45,1000, {.forwards=false, .maxSpeed=70});// corner #2
//     chassis.waitUntilDone();
//      intake_mg.move(0);
//      chassis.waitUntilDone();
//    piston.extend();//drop goal
//    chassis.moveToPoint(50,-48,500);//back up
//    chassis.turnToHeading(0,500);
//    chassis.moveToPoint(45,10,1500,{.forwards=false}); //move out of way
//    chassis.waitUntilDone();
//    chassis.moveToPose(25,30,-80,1500, {.forwards=true,.minSpeed=60});
//    chassis.waitUntilDone();
//    chassis.moveToPose(-85,66,-90, 2600, {.forwards = true, .minSpeed = 150});//push other side mobile goal
//    chassis.waitUntilDone(); 
//     chassis.moveToPose(0,50,-90, 1300, {.forwards = false, .minSpeed = 127});// go to center-ish
//     chassis.moveToPose(100,65,-90, 2000, {.forwards = false, .minSpeed = 127});//push the other mobile goal
//     chassis.waitUntilDone();  
//     delay(500);
//     chassis.moveToPose(50,65,-90,500,{.forwards=false});//back up
//     chassis.turnToHeading(-135,500);
//     arm.extend();
//     chassis.waitUntilDone();
//     chassis.moveToPose(5,10,-135,2000,{.maxSpeed=80});//go to hang
    
    
    //current one
//     chassis.setPose(0,-60,0);
//    intake_mg.move(127);//alliance stake
//    delay(1000);
//  chassis.moveToPose(0,-48,0,1000);
//  chassis.turnToHeading(90,500);
//  piston.extend();
//  chassis.waitUntilDone();
//     chassis.moveToPose(-24,-45,90,1000,{.forwards=false,.maxSpeed=50});//go to mobile goal
//    chassis.waitUntilDone();
//    delay(800);
//    piston.retract();
//    chassis.waitUntilDone();
//    delay(600);
//    intake_mg.move(95);
//    delay(500);
//    chassis.turnToHeading(0,1000);
//    chassis.moveToPose(-24,-30,0,1000, {.maxSpeed=80 });//go to ring
//    chassis.waitUntilDone();
//    delay(800);
//    chassis.turnToHeading(-45,1000);
//    chassis.moveToPoint(-43,-2,1000,{.maxSpeed=80});//move to coordinate 
//    chassis.moveToPose(-44,15,-15,1300);//move to second ring
//    chassis.waitUntilDone();
//    delay(800);
//    chassis.turnToHeading(180,700, {.maxSpeed=85});
//    chassis.moveToPose(-43,-60,180,4000, {.maxSpeed=50});//move down to set of rings
//    chassis.waitUntilDone();
//    chassis.moveToPoint(-46,-37,1000,{.forwards=false});
//    chassis.moveToPose(-55,-43,-90,2000, {.maxSpeed=70});//other ring
//    chassis.waitUntilDone();
//    chassis.moveToPose(-56,-60,45,2000, {.forwards=false, .maxSpeed=70});//corner
//    chassis.waitUntilDone();
//    piston.extend();//drop goal
//    intake_mg.move(0);
//    delay(1000);
//    chassis.moveToPoint(-48,-48,500);
//    chassis.turnToHeading(90,1000);
//    chassis.moveToPose(24,-50,-90,4000,{.forwards=false,.maxSpeed=70});//go to second mobile goal #2
//     chassis.waitUntilDone();
//    piston.retract();
//   delay(1000);
//     intake_mg.move(95);
//    chassis.moveToPose(26,-26,0,2000);//go to ring #2
//    chassis.waitUntilDone();
//    chassis.moveToPose(26,-30,0,500,{.forwards=false});
//    delay(800);
//    chassis.moveToPoint(46,-2,1000,{.maxSpeed=80});//move to coordinate #2
//    chassis.moveToPose(46,13,15,1300);//move to second ring #2
//    chassis.waitUntilDone();
//    delay(800);
//    chassis.turnToHeading(180,700);
//    chassis.moveToPose(46,-70,180,4000, {.maxSpeed=60});//move down to set of rings #2
//    chassis.waitUntilDone();
//    chassis.moveToPoint(46,-50,700,{.forwards=false});
//    chassis.waitUntilDone();
//    chassis.moveToPose(56,-50,90,1500, {.maxSpeed=70});//other ring #2
//    chassis.waitUntilDone();
//    delay(800);
//    chassis.moveToPose(65,-70,-45,2000, {.forwards=false, .maxSpeed=70});// corner #2
//     chassis.waitUntilDone();
//      intake_mg.move(0);
//      chassis.waitUntilDone();
//    piston.extend();//drop goal
//    chassis.moveToPoint(48,-48,500);//back up
//    chassis.turnToHeading(0,500);
//    chassis.moveToPoint(35,10,2000,{.forwards=false}); //move out of way
//    chassis.waitUntilDone();
//    chassis.moveToPose(25,20,-80,1500, {.forwards=true,.minSpeed=60});
//    chassis.waitUntilDone();
//    chassis.moveToPose(-85,62,-90, 3000, {.forwards = true, .minSpeed = 127});//push other side mobile goal
//    chassis.waitUntilDone(); 
//     chassis.moveToPose(0,35,-90, 1700, {.forwards = false, .minSpeed = 127});// go to center-ish
//     chassis.moveToPose(100,65,-90, 3000, {.forwards = false, .minSpeed = 127});//push the other mobile goal
//     chassis.moveToPose(80,70,-90,1000,{.forwards=false});
    

//     chassis.setPose(-12,-60,90);
//    chassis.moveToPose(-7,-60,90,1000);//go to alliance stake
//    chassis.waitUntilDone();
//    doinker.extend();
//    delay(1000);
//    doinker.retract();
//       chassis.moveToPoint(-13,-57, 1000, {.forwards=false});//back up from alliance
//    delay(500);
//    chassis.waitUntilDone();
//     chassis.moveToPose(-13,-56,150,1000,{.forwards=false});//turn to mobile goal
//  piston.extend();
//    //chassis.moveToPoint(-24,-48,1000,{.forwards=false, .maxSpeed=50});//go to mobile goal
//    chassis.waitUntilDone();
//    delay(800);
//    piston.retract();
//    chassis.waitUntilDone();
//    delay(1000);
//    intake_mg.move(100);
//    delay(500);
//    chassis.turnToHeading(0,1000);
//    chassis.moveToPose(-24,-30,0,1000, {.maxSpeed=80 });//go to ring
//    chassis.waitUntilDone();
//    delay(800);
//    chassis.turnToHeading(-45,1000);
//    chassis.moveToPoint(-43,-2,1000,{.maxSpeed=80});//move to coordinate 
//    chassis.moveToPose(-46,13,-15,1300);//move to second ring
//    chassis.waitUntilDone();
//    delay(800);
//    chassis.turnToHeading(180,700, {.maxSpeed=85});
//    chassis.moveToPose(-43,-60,180,4000, {.maxSpeed=50});//move down to set of rings
//    chassis.waitUntilDone();
//    chassis.moveToPoint(-46,-50,500,{.forwards=false});
//    chassis.moveToPose(-55,-51,-90,2000, {.maxSpeed=70});//other ring
//    chassis.waitUntilDone();
//    chassis.moveToPose(-56,-60,45,2000, {.forwards=false, .maxSpeed=70});//corner
//    chassis.waitUntilDone();
//    piston.extend();//drop goal
//    intake_mg.move(0);
//    delay(1000);
//    chassis.moveToPoint(-48,-48,500);
//    chassis.turnToHeading(90,1000);
//    chassis.moveToPose(24,-55,-90,4000,{.forwards=false,.maxSpeed=70});//go to second mobile goal #2
//     chassis.waitUntilDone();
//    piston.retract();
//   delay(1000);
//     intake_mg.move(100);
//    chassis.moveToPose(26,-26,0,2000);//go to ring #2
//    chassis.waitUntilDone();
//    chassis.moveToPose(26,-30,0,500,{.forwards=false});
//    delay(800);
//    chassis.moveToPoint(46,-2,1000,{.maxSpeed=80});//move to coordinate #2
//    chassis.moveToPose(46,13,15,1300);//move to second ring #2
//    chassis.waitUntilDone();
//    delay(800);
//    chassis.turnToHeading(180,700);
//    chassis.moveToPose(47,-67,180,4000, {.maxSpeed=60});//move down to set of rings #2
//    chassis.waitUntilDone();
//    chassis.moveToPoint(46,-50,700,{.forwards=false});
//    chassis.moveToPose(56,-55,90,1500, {.maxSpeed=70});//other ring #2
//    chassis.waitUntilDone();
//    chassis.moveToPose(65,-70,-45,2000, {.forwards=false, .maxSpeed=70});// corner #2
//     chassis.waitUntilDone();
//      intake_mg.move(0);
//      chassis.waitUntilDone();
//    piston.extend();//drop goal
//    chassis.moveToPoint(48,-48,500);//back up
//    chassis.turnToHeading(0,500);
//    chassis.moveToPoint(33,10,2000,{.forwards=false}); //move out of way
//    chassis.waitUntilDone();
//    chassis.moveToPose(15,20,-80,1500, {.forwards=true});
//    chassis.waitUntilDone();
//    chassis.moveToPose(-80,62,-90, 2500, {.forwards = true, .minSpeed = 127});//push other side mobile goal
//    chassis.waitUntilDone(); 
//     chassis.moveToPose(0,30,-90, 1700, {.forwards = false, .minSpeed = 127});// go to center-ish
//     chassis.moveToPose(100,65,-90, 2000, {.forwards = false, .minSpeed = 127});//push the other mobile goal
//     chassis.moveToPose(70,70,-90,1000,{.forwards=false});

//    delay(2000);
//    intakeTop.move(0); 
//    chassis.waitUntilDone();
//     chassis.moveToPose(-52,-4,-161,2000);//wallstake ring
//     chassis.waitUntilDone(); //wait til youre there 
//     intakeTop.move(80); //turn your second intake on 
//     delay(800); 
//     intakeTop.move(-30); //ring gets loaded and unload 
//    chassis.moveToPose(-50,-3,-90,1500);
//    delay(1000);
//     piston.extend();
//    chassis.waitUntilDone();
//    intake_mg.move(0);//stop all intake 
//    arm.extend();
// chassis.moveToPose(-47,-3,-90,1500);
// chassis.waitUntilDone();
// arm.retract();


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
 int armState = 0;
void arm(){
    int angle;
int error = 0;

while(true){
    if (armState == 0){
        angle= 11000; //arm down height
    
     } else if (armState==1){
        angle=14845; //scoring height 
    }
    }
    error = angle - rotation_sensor.get_position()/3;
    intake_mg.move(error*-0.02);
pros::delay(20);
}
;

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    colorsorter.set_led_pwm(100);
    // print position to brain screen
    pros::Task sortTask(colorSortTask);
 
    pros::Task screen_task([&]() {
    
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", (chassis.getPose().x )); // x
            pros::lcd::print(1, "Y: %f", (chassis.getPose().y )); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "Hue value: %f", colorsorter.get_hue());
            pros::lcd::print(4,"Rotation:%d",rotation_sensor.get_position ());
            // delay to save resources
            pros::delay(20);
        }
    });
   // pros:: Task armTask(arm);
   
   
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

void autonomous() 
{
 sixred();

}


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


		if (master.get_digital(DIGITAL_L2))
		{
          
			intake_mg.move(-127);
            if (rotation_sensor.get_position()>41200){
                intake_top.move(0);
            }
    
            
		}
		else if (master.get_digital(DIGITAL_L1))
		{
			intake_mg.move(127);
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
        if (master.get_digital_new_press(DIGITAL_B)){//goalDOINKER
			astate = !astate;
		}

		if (astate == false){
			goaldoinker.extend();
		}
		else {
			goaldoinker.retract();
		}
        if (master.get_digital_new_press(DIGITAL_X)){//ringDOINKER
			dstate = !dstate;
		}

		if (dstate == false){
			ringdoinker.extend();
		}
		else {
			ringdoinker.retract();
		}
        
        

		// if (master.get_digital(DIGITAL_A))
		// {
		// 	piston.extend();
		// }
		// if (master.get_digital(DIGITAL_B))
		// {
		// 	piston.retract();
		// }
		 // Run for 20 ms then update
	}
}