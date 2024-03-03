#include "main.h"
#include "display/lv_objx/lv_list.h"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "pros/vision.hpp"
#include "pros/vision.h"
#include <string>
#include <cmath>
#include "MarinRoboticsCustom/math_utils.h"
using namespace std;

// Custom Variables
bool one_cata = false;
bool Cata_toggle = false;
bool wing_out = false;
bool end_out = false;
int cata_speed = 80;
std::string routes[3] = {"AWP", "Match"  "Score"};
int selected_route = 0;
bool pto_activated = false;
//----------------------------------------//


bool skills = false; // SKILLS SELECTOR
 

//----------------------------------------//

  // Drive Controls
bool drive_plus_turning = true; 
bool drive_plus_forward = true; 
bool standard_drive = true;
  // Auton
bool auton_mode = false;
float Wheel_Diameter = 4;
float Turning_Diameter = 12.6;
float Turn_Tuning_Factor = .8;
float Move_Tuning_Factor = .667; // inches
  // Slew Rate Limiting
float up_step = 50;
float down_step = -50;
float turn_constant = 0.7;

// Static Variables
  // Main Drive
int left_x, left_y, right_x, right_y;
  // Auton Drive Vars
float Wheel_Circumference = Wheel_Diameter * 3.1416;
float Turning_Circumference = Turning_Diameter * 3.1416;
float Turning_Distance, Wheel_Revolutions, Turn_Wheel_Rotation, Forward_Wheel_Rotation;
bool wait = false;
  // Slew Rate Limiting
float y_current, x_current, y_direction, x_direction;
float y_true_step;
float x_true_step;


// Defining Motors

pros::ADIDigitalOut PTO(1); //e
pros::ADIDigitalIn  auton_selector(2); //f
pros::ADIDigitalOut WingR(3);
pros::ADIDigitalOut WingL(4); //g
pros::ADIDigitalOut endgame(5); //h
pros::ADIDigitalOut Hook(6);

pros::Motor fourbar(8,pros::E_MOTOR_GEAR_GREEN, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Cata_Motor(19,pros::E_MOTOR_GEAR_RED, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Snarfer(5,pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor left_front_motor(12,pros::E_MOTOR_GEAR_GREEN, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_front_motor(2,pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor left_back_motor(14,pros::E_MOTOR_GEAR_GREEN, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_back_motor(4,pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor left_mid_motor(16,pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_mid_motor(6,pros::E_MOTOR_GEAR_GREEN, true, pros::E_MOTOR_ENCODER_DEGREES);
// Set motor groups
pros::Motor_Group left_motors ({left_front_motor, left_back_motor, left_mid_motor});
pros::Motor_Group right_motors ({right_front_motor, right_back_motor, right_mid_motor});

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);
void on_center_button() {
  static bool pressed = false;
  
	pressed = !pressed;
  auton_mode = pressed;
	if (pressed) {
		pros::lcd::set_text(2, "Auton Mode: Offensive");
	} else {
		pros::lcd::set_text(2, "Auton Mode: Defensive");
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::register_btn1_cb(on_center_button);  
  left_front_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
  left_back_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
  left_mid_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
  right_front_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
  right_back_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
  right_mid_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
  fourbar.set_brake_mode(MOTOR_BRAKE_HOLD);
  Cata_Motor.set_brake_mode(MOTOR_BRAKE_COAST);
  Snarfer.set_brake_mode(MOTOR_BRAKE_HOLD);

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  while (true){
    if (auton_selector.get_new_press()){
      if (selected_route > sizeof(routes)/sizeof(routes[0])-2){
        selected = 0;
      } else {
        selected_route ++;
      }
    }
    if (selected == 0) {
      fill_screen(0xFF0000); //red
    } if else (selected == 1) {
      fill_screen(0x00FF00); //green
    } if else (selected == 2) {
      fill_screen(0x0000FF); //blue
    }
    pros::screen::print(TEXT_SMALL, 1, "Selected: %s", routes[selected_route]);
    pros::delay(20);
  }
}

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

// Autonomous Functions

void fill_screen(int color=0xFF0000) {
    // Set pen color
    pros::screen::set_pen(color);
    pros::screen::fill_rect(0, 0, pros::screen::get_width(), pros::screen::get_height());
}

// On button press
// stop motors
// delay
// extend pnuematic
// delay
// switch to coast for motors
// switch driving function to just full power with joystick


float autonselect=1 ;
pros::Mutex action;
void turn (float angle, float velocity) {
	//action.take(1000);
	Turning_Distance = angle/360 * Turning_Circumference;
  Wheel_Revolutions = Turning_Distance/Wheel_Circumference;
  Turn_Wheel_Rotation = Turn_Tuning_Factor*Wheel_Revolutions*360; // Degrees that the wheel should turn
  left_motors.move_relative(Turn_Wheel_Rotation*autonselect, velocity);
  right_motors.move_relative(-Turn_Wheel_Rotation*autonselect, velocity);
  pros::delay(750);
}

void move( float inches, float velocity) { 	//action.take(1000);
	Forward_Wheel_Rotation = Move_Tuning_Factor*(inches/Wheel_Circumference)*360;

  left_motors.tare_position();
  right_motors.tare_position();

  left_motors.move_relative(Forward_Wheel_Rotation,velocity);
  right_motors.move_relative(Forward_Wheel_Rotation,velocity);
  Snarfer.brake();
	}

void autonawp() {
  move(6, 50);
  pros::delay(400);
  turn(-50, 50);
  move(30, 100);
  pros::delay(1000);
  turn(50, 100);
  Snarfer.move_velocity(-200);
  pros::delay(700);
  move(20, 200);
  pros::delay(1200);
  move(-10, 100);
  pros::delay(600);
  move( 12, 100);
  pros::delay(800);
  move(-7, 100);
  pros::delay(700);
  turn(180, 70);
  pros::delay(800);
  move(7, 50);
  pros::delay(600);
  turn(-40, 50);

  Wing.set_value(HIGH);
  move(21, 100);
  pros::delay(900);
  Wing.set_value(LOW);
  pros::delay(200);
  turn(-10, 50);
  move(11, 100);
pros::delay(700);
  turn(-40, 50);
  pros::delay(200);
  Snarfer.move_velocity(-200);
  move(32, 70);
  Snarfer.move_velocity(-200);
  pros::delay(1800);
  Snarfer.move_velocity(-200);
  pros::delay(1800);
  Snarfer.move_velocity(0);
  
}

void autonscore(){
  Wing.set_value(HIGH);
  move(15, 100);
  pros::delay(800);
  Wing.set_value(LOW);
  move(-3, 70);
  pros::delay(300);
  move(18, 70);
  pros::delay(800);
  turn(-40, 50);
  
  Snarfer.move_velocity(-200);
  Snarfer.move_velocity(-200);
  Snarfer.move_velocity(-200);
  pros::delay(150);
  Snarfer.move_velocity(-200);
  move(42, 200);
  Snarfer.move_velocity(-200);
  Snarfer.move_velocity(-200);
  pros::delay(900);
  move(-10, 100);
  pros::delay(500);
  move( 12, 200);
  pros::delay(700);
  move(-14, 100);
  Snarfer.brake();
  pros::delay(700);
  turn(-65, 50);
  pros::delay(150);

  move(57, 200);
  pros::delay(1100);
  Snarfer.move_velocity(200);
  pros::delay(700);
  Snarfer.brake();
  turn(145, 70);
  pros::delay(350);
  Snarfer.move_velocity(-200);
  Snarfer.move_velocity(-200);
  Snarfer.move_velocity(-200);
  
  Snarfer.move_velocity(-200);
  move(10, 100);
  Snarfer.move_velocity(-200);
  pros::delay(800);
  move(-8, 100);
  Snarfer.move_velocity(-200);
  pros::delay(600);
  Snarfer.brake();
  turn(-92, 50);
  pros::delay(100);
  Snarfer.move_velocity(200);
  Snarfer.move_velocity(200);
  Snarfer.move_velocity(200);
  pros::delay(50);
  Snarfer.move_velocity(200);
  Snarfer.move_velocity(200);
  move(22.5, 100);
  Snarfer.move_velocity(200);
  Snarfer.move_velocity(200);
  pros::delay(1050);
  turn(115, 70);
  pros::delay(200);
  Snarfer.brake();
  pros::delay(100);
  Snarfer.move_velocity(-200);
  Wing.set_value(HIGH);
  move(50, 200);
  pros::delay(800);
  move(-20, 200);
  Wing.set_value(LOW);
  pros::delay(900);
  
 


  
  
  

}

void autonmatch() {
  
  move(55.5, 150);
  pros::delay(1300);
  turn(-95, 100);
  
  Snarfer.move_velocity(-200);
  pros::delay(200);
  move(20, 200);
  pros::delay(800);
  move(-10, 100);
  pros::delay(500);
  move( 12, 100);
  Snarfer.move_velocity(-200);
  pros::delay(700);

  move(-10, 100);
  pros::delay(400);
  Wing.set_value(HIGH);
  pros::delay(200);
  move(-24.5, 150);
  pros::delay(900);
  turn(320, 100);
  Wing.set_value(LOW);
  pros::delay(1300);
  move(62, 150);
  pros::delay(1400);
  Wing.set_value(HIGH);
  pros::delay(300);
  turn(-90, 100);
  pros::delay(200);
  
  move(6, 100);
pros::delay(500);
  Wing.set_value(LOW);
  pros::delay(500);
  move(12, 100);
pros::delay(800);
  turn(-40, 100);
  Snarfer.move_velocity(-200);
  move(32, 150);
  Snarfer.move_velocity(-200);
  pros::delay(1700);
  Snarfer.move_velocity(0);
  
  






  
}

void autonskills() {
  move(6, 50);
  pros::delay(500);
  turn(-50, 50);
  move(35, 150);
  pros::delay(700);
  turn(51, 70);
  Snarfer.move_velocity(-200);
  pros::delay(700);
  move(20, 200);
  pros::delay(900);
  move(-15, 100);
  fourbar.move_relative(-720, 200);
  pros::delay(1200);
  turn(70, 50);
  move(-10, 20);
  Cata_Motor.move_velocity(-100);
  pros::delay(30000);
  Cata_Motor.move_velocity(0);
  fourbar.move_relative(720, 200);
  move(50, 150);
  
  pros::delay(1600);
  turn(31, 50);
  move(60, 200);
  pros::delay(1400);
  turn(-40, 50);
  Wing.set_value(HIGH);
  pros::delay(200);
  move(40, 150);
  pros::delay(1900);
  Wing.set_value(LOW);
  turn(20, 50);
  move(-30, 150);
  pros::delay(1100);
  turn(-60, 100);
  move(70, 200);
  pros::delay(1600);
  turn(120, 100);
  Wing.set_value(HIGH);
  pros::delay(200);
  move(45, 200);
  
  pros::delay(1600);
  Wing.set_value(LOW);
  move(-25, 150);
  pros::delay(1200);
  turn(-45, 50);
  move(50, 150);
  pros::delay(1100);
  turn(110, 100);
  move(30, 200);
  Snarfer.move_velocity(-200);
  pros::delay(800);
  move(-10, 100);
  pros::delay(500);
  move( 15, 200);
  pros::delay(900);
  move(-10, 200);
  pros::delay(700);
 
  





}

void autonomous() {
  if (skills) {
    auton_skills(); 
  } else if (selected_route == 0){ // AWP
    autonawp();
  } else if (selected_route == 1){ // Match
    autonmatch();
  } else if (selected_route == 2){ // Score
    autonscore();
  }
    
}

void opcontrol() {
  int cata_speed = 100;
  while (true) {

    pros::lcd::clear();
    // Get joystick values
    float left_y = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    float left_x = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
    float right_y = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
    float right_x = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

    // Deadzone for left-stick turning while moving forward in a straight line
    if ((abs(left_x) < 25)) {
      left_x = 0;
    }
    
    // Change catapult speed with Up & Down Buttons
    if ((controller.get_digital_new_press(DIGITAL_RIGHT)) && (cata_speed < 100)){
      cata_speed = cata_speed + 5; 
    } else if  ((controller.get_digital_new_press(DIGITAL_LEFT)) && (cata_speed > 0)) {
    cata_speed = cata_speed - 5; 
    }

    // Toggle wings
    if (controller.get_digital_new_press(DIGITAL_A)){
      wing_out = !wing_out;
    }
    if (wing_out){
      Wing.set_value(HIGH);
    } else {
      Wing.set_value(LOW);
    }

    // Toggle endgame
    if (controller.get_digital_new_press(DIGITAL_UP)){
      end_out = !end_out;
    }
    if (end_out){
      endgame.set_value(HIGH);
    } else {
     endgame.set_value(LOW); 
    }

    // Snarfer Controls
    if(controller.get_digital(DIGITAL_R1)) { // Outtake with R1
      Snarfer.move_velocity(-200);
    } else if (controller.get_digital(DIGITAL_R2)){ // Intake with R2
      Snarfer.move_velocity(200);
    } else {
      Snarfer.move_velocity(0);
    }

    // Move fourbar
    if(controller.get_digital(DIGITAL_L1)) { 
      fourbar.move_velocity(-200);
    } else if (controller.get_digital(DIGITAL_L2)){
      fourbar.move_velocity(200);
    } else {
      fourbar.move_velocity(0);
    }
    
    // Catapult Toggle
    if(controller.get_digital_new_press(DIGITAL_R2)) {
      Cata_toggle = !Cata_toggle;
    }
    if(Cata_toggle){
      Cata_Motor.move_velocity(-cata_speed);
    } else {
     Cata_Motor.move_velocity(0);
    }

    // Activate Engame PTO
    if (controller.get_digital_new_press(DIGITAL_DOWN)) {
      pto_activated = true;
      pros::delay(300);
      PTO.set_value(HIGH);
      pros::delay(300);
    }

    // Drive Code:
    if (drive_plus_turning) { // Option to use the right stick for turning
      if (abs(right_x) < abs(left_x)) {
        right_x = left_x;
      }
    }
    if (drive_plus_forward) { // Option to use the right stick for forward
      if (abs(right_y) < abs(left_y)) {
        right_y = left_y;
      }
    }

    // Drive Control Loop (LEFT)
    y_direction = sgn(left_y);
    x_direction = sgn(right_x);

    float y_goal = powf((abs(left_y) / 127), 2) * 127;
    float x_goal = powf((abs(right_x) / 127), 2) * 127;
    
    float y_error = y_goal - y_current;
    float x_error = x_goal - x_current;

    if (((up_step > y_error) && (y_error > 0)) || ((down_step < y_error) && (y_error <= 0))) {
      y_true_step = y_error;
    } else if (y_goal > y_current) {
      y_true_step = up_step;
    } else if (y_goal < y_current) {
      y_true_step = down_step;
    } else {
      y_true_step = 0;
    }
    if (((up_step > x_error) && (x_error > 0)) || ((down_step < x_error) && (x_error <= 0))) {
      x_true_step = x_error;
    } else if (x_goal > x_current) {
      x_true_step = up_step;
    } else if (x_goal < x_current) {
      x_true_step = down_step;
    } else {
      x_true_step = 0;
    }
    y_current += y_true_step;
    x_current += x_true_step;

    if (pto_activated) {
      left_motors.move(y_current * y_direction);
      right_motors.move(y_current * y_direction);
    } else {
      if (standard_drive) { // Drive forwards
        left_motors.move((y_current * y_direction) + (turn_constant * x_current * x_direction));
        right_motors.move(((y_current * y_direction) - (turn_constant * x_current * x_direction))/1.005);
      } else { // Drive backwards
        left_motors.move((y_current * y_direction) - (turn_constant * x_current * x_direction));
        right_motors.move(((y_current * y_direction) + (turn_constant * x_current * x_direction))/1.005);
      }
    }
    pros::delay(20);
  }
}
