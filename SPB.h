

//#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
//  #error "Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu."
//#endif

#define BAUDRATE 500000

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

#define ROS 0

//Stepper Driver
//#include "DRV8825.h"
//#define MODE0 10
//#define MODE1 11
//#define MODE2 12

//VL51L1X, VL6180X 
#define topCE 17  //VL51L1X
#define sideCE 16 //VL6180X 

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
//#define MOTOR_STEPS 400//400 // steps per rotation // likely incorrect - maybe 100
//#define RPM 9 // 240 max
//#define RPM_POUR 4
//#define MAX_STEPS 400 //max steps in one loop 400
#define DEADBAND 15 // deadband steps
#define STEPSPERMM 400.0 //steps per mm of travel // MEASURED
#define TRAVEL 178.0 //total distance of travel (may be 174)
//#define PITCH 4 //mm per rotation // not sure
//#define V_TRAVEL 40000000

#define ES_SAFETY_DIST 3

#define GLASSHEIGHT_DEFAULT 160  //CHECK if glassheight is the top of the glass to the bottom (and not total stem height)
#define GLASSBOTTOM_DETAULT 10
#define MAXGLASSHEIGHT 220
#define MINGLASSHEIGHT 50
// Since microstepping is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
#define MICROSTEPS 8  // TIC500: 8

//Other pins
#define SOLENOID 9  //flow valve
#define TICPWR 8 // T500 stepper controller
#define US_PWR 10   //turn on ultrasound power
#define US_PIN A9   //Ultrasound analog
#define TOP_IR_PIN A4 //top infrared
#define SIDE_IR_PIN A3 // side infrared
#define ESTOP 3 // estop pin //THESE ARE NOT INTERRUPT PINS CHANGE TO 2
#define STARTBTN 5 // start button //THESE ARE NOT INTERRUPT PINS CHANGE TO 3
#define STARTLED 4 // start button LED
#define LED_PIN 6 // Neopixel pin
#define NUMPIXELS 26 // Number of LEDs
#define LED_START_VAL 180 // Starting brightness

//#define SIDEIRTHRESH 1.22 // ADC fraction increase when glass detected
#define SIDEIRTHRESH 125 // mm VL6180X
#define SIDEIRPOS 125  //mm from the top
#define TUBEPOS 180 //mm tube length
#define STOPDISTANCE 190 //190 // How far to stop the bottom of the glass from zero (STOPDISTANCE - TUBEPOS = Xmm from bottom of glass)
#define SETPOINT 20  // the tube will be X mm from surface
#define SURFOFFSET  40// fill the glass this far from the glass top
#define GLASSCVFUDGE 20 //fudge factor when measuring the glass height with CV - accounts for lens angle


float topIR2dist(int);
float US2dist(int);
void manual_scroll();
bool update_tray_pos(void);
int measure_topIR();
int measure_sideIR();
float measure_US();
float measure_CV();
void home_tray();
void beer_time();
float filterloop(float);
void set_lights(int);
void estop_LED(void);
void side_ghd(void);
void process_glass_height(void);
bool trayUpLoop(void);
bool trayDownLoop(void);
void spb_v(long);
void home_tray_zero(void);
void check_clean(void);
