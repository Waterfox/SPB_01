

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
#define MOTOR_STEPS 200 // steps per rotation
#define RPM 240 // 240 max
#define MAX_STEPS 80 //max steps in one loop
#define DEADBAND 15 // deadband steps
#define STEPSPERMM 50 //steps per mm of travel
#define PITCH 4 //mm per rotation

// Since microstepping is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
#define MICROSTEPS 16

//Other pins
#define SOLENOID 9  //flow valve
#define US_PWR 10   //turn on ultrasound power
#define US_PIN A9   //Ultrasound analog
#define TOP_IR_PIN A4 //top infrared
#define SIDE_IR_PIN A3 // side infrared
#define ESTOP 3 // estop pin //THESE ARE NOT INTERRUPT PINS CHANGE TO 2
#define STARTBTN 5 // start button //THESE ARE NOT INTERRUPT PINS CHANGE TO 3
#define STARTLED 4 // start button LED
#define LED_PIN 6 // Neopixel pin
#define NUMPIXELS 26

#define SIDEIRTHRESH 1.23 // ADC fraction increase when glass detected
#define SIDEIRPOS 125  //mm from the top
#define TUBEPOS 180 //mm tube length
#define STOPDISTANCE 190.0 // How far to stop the bottom of the glass  (STOPDISTANCE - TUBEPOS = Xmm from bottom of glass)
#define SETPOINT 15  // the tube will be X mm from surface
#define SURFOFFSET 40 // fill the glass this far from the glass top
#define TPUB1 300 //
#define TPUB2 500 //

float topIR2dist(int);
float US2dist(int);
void manual_scroll();
int spb_move(int);
void update_tray_pos(void);
float measure_topIR();
int measure_sideIR();
float measure_US();
float measure_CV();
void home_tray();
void beer_time();
float filterloop(float);
void set_lights(int);
void publish_all(void);
void publish_tray(void);


