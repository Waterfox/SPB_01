

#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
  #error "Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu."
#endif

#ifndef BOARD_NAME
  #define BOARD_NAME "RAMPS 1.4"
#endif

//Endstop Pins
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19
//Endstop Positions
#define Z_MIN_POS          365
#define Z_MAX_POS          187


//Stepper Pins
#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_CS_PIN           40

#ifndef RAMPS_D9_PIN
  #define RAMPS_D9_PIN   9
#endif  


//Stepper Driver
#include "DRV8825.h"
#define MODE0 10
#define MODE1 11
#define MODE2 12

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 240
#define MAX_STEPS 300 //max steps in one loop
#define DEADBAND 130 // deadband steps

// Since microstepping is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
#define MICROSTEPS 1

//Other pins
#define SOLENOID 9  //flow valve
#define US_PWR 10   //turn on ultrasound power
#define US_PIN A9   //Ultrasound analog
#define TOP_IR_PIN A4 //top infrared
#define SIDE_IR_PIN A3 // side infrared


float topIR2dist(int);
float US2dist(int);
//endstop callbacks - can't be a part of the endstop class to attach interrupt. 
void max_callback();
void min_callback();
void manual_scroll();
int spb_move(int);
void update_tray_pos(void);
float measure_topIR();
float measure_US();
