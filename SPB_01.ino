#include <Arduino.h>
#include "SPB.h"
#include "endstops.h"
#include <Adafruit_NeoPixel.h>
#include <stdlib.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <Tic.h>


/*
 * BRANCH FOR USE WITH TIC500 * 
   rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=500000
   NOTE: direction changed for tmc 2208

   STATE:
   0: E-Stopped
   1: Ready
   2: Pouring
   3: ROS not connected
*/


TicSerial tic(Serial3);

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);


VL53L1X top_sensor;
VL53L1X side_sensor;
bool tVLinitfail = false;
bool sVLinitfail = false;

float topIRAvg = 0;
float USAvg = 0;
float sideIR = 0;
endstops es;
int trayPosStp = 0; //tray position from stepper count - measured from the top down
int surfPosCV = 0;  //surface position from computer vision output
int linePosCV = 0;  //line measurement from CV output
int surfPos = 0;  //surface position used in control calculation

int lastDirn = 0;
int steps = 0;
float delta = 0; //error in P control loop
int state = 0; //0: Estopped, 1:Waiting, 2:Pouring

int useGlassHeightDetection = true;
//int useCVGlassHeightDetection = false;
int defaultGlassHeight = GLASSHEIGHT_DEFAULT;
int defaultGlassBottom = GLASSBOTTOM_DETAULT; //used in the future to elminate ultrasound
bool side_detected = false;
int glassTop = 0;
int glassBot = 0;
int glassHeight = GLASSHEIGHT_DEFAULT;  //top of glass to the bottom (not stem height)
float usVal = 0;
float topIRVal = 0;
volatile int curLightVal = 175;
long v_travel = 100000000;


float USnow; // last ultrasound value
int tir;  // top ToF last value
int sir; // side ToF last value


unsigned wait_time_micros;


long LED_timer = 0;
bool LED_state = 0;
long RPM_timer = 0;
long loop_timer3 = 100;
long loop_timer4 = 0;
long loop_timer5 = 100;
long loop_timer6 = 0;
bool CV_EN = false;
bool CV_LINES_EN = false;



//**********************************************
void setup() {
  state = 1; // Oh shazbot what was this? Estop condition?

  //configure the VLX ToF sensors
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  //change the top ToF i2c address
  pinMode(topCE, INPUT); // will float high
  pinMode(sideCE, OUTPUT);
  digitalWrite(sideCE, LOW);
  delay(50);

  top_sensor.setTimeout(500);

  if (!top_sensor.init())
  {
    tVLinitfail = true;
  }
  top_sensor.setDistanceMode(VL53L1X::Short);
  top_sensor.setMeasurementTimingBudget(20000);
  top_sensor.startContinuous(20);
  top_sensor.setAddress(0x31);
  delay(50);

  //  delay(2000);
  //  Serial.println(top_sensor.getAddress());
  pinMode(sideCE, INPUT); // will float high, turn side back on
  delay(50);
  if (!side_sensor.init())
  {
    sVLinitfail = true;
  }
  side_sensor.setDistanceMode(VL53L1X::Short);
  side_sensor.setMeasurementTimingBudget(20000);
  side_sensor.startContinuous(20);



  pinMode(SOLENOID, OUTPUT);
  pinMode(US_PWR, OUTPUT);
  pinMode(TICPWR, OUTPUT);

  //Turn off the Valve
  digitalWrite(SOLENOID, LOW);

  //Turn on the TIC T500 Stepper Driver
  digitalWrite(TICPWR, HIGH);

  // Init the endstops and buttons
  es.check_endstops();
  buttons_init();

  //turn on the ultrasound
  digitalWrite(US_PWR, true);

  //Turn the Lights On
  pixels.begin();
  set_lights(curLightVal);



  Serial.begin(115200);
  if (tVLinitfail){
    Serial.println("Top VL sensor fail");
  }
  if (sVLinitfail){
    Serial.println("side VL sensor fail");
  }
  Serial.println("Home the Tray");
  

  //TIC 500 is connected to Serial3
  Serial3.begin(57600);
  // Give the Tic some time to start up.
  delay(20);
  tic.haltAndSetPosition(0);
  tic.setProduct(TicProduct::T500);
  tic.setStepMode(TicStepMode::Microstep8);
  tic.setMaxSpeed(120000000);  //12V MAX Speed
  
  
  tic.exitSafeStart();
  home_tray_zero();
  
}



//----------------------------------------------
void loop() {

  check_estop();
  if (state > 0) {
    check_start();
    check_tof_start();
    // ADD CHECK TOF START
  }
  else if (state==0){
    check_clean();
  }
  
  update_tray_pos();
  estop_LED();

  delay(1);

}
//------------------------------------------------





//conver ultrasound measurement to distance in mm (distance from sensor)
float US2dist(int usVal) {
  return (0.1466275 * usVal + 100.0); //150.0mm/1023.0 * usVal + 100mm
}


void spb_v(long v) {
  v= -v; // reversing v: velocity so that "+ve" is up, "-ve" v is down at function input
  if ((v<0 && es.enUp == true)||(v>0 && es.enDown == true) ){tic.setTargetVelocity(v);}
  else if (v==0) {tic.setTargetVelocity(0);}
  else {
    Serial.println("Move not valid!");
    Serial.print("enUp: ");Serial.print(es.enUp);Serial.print("   enDown: ");Serial.println(es.enDown);
    }
}

bool update_tray_pos(void) 
//updates the tray position in the global trayPosStp variable
//stops the motor if the tray is too close to an endstop and returns true.
{
  trayPosStp = tic.getCurrentPosition()/(STEPSPERMM);
  /*
  if (trayPosStp > (Z_MIN_POS - ES_SAFETY_DIST))
  {
    tic.setTargetVelocity(0);
    Serial.println("Z_MIN_POS ERR");
    return 1;
  }
  
  if(trayPosStp < (Z_MAX_POS + ES_SAFETY_DIST))
  {
    tic.setTargetVelocity(0);
    Serial.println("Z_MAX_POS ERR");
    return 1;
  }
  */
  
  return 0;
}

int measure_topIR() {
  //VL53L1X
  top_sensor.read();
  tir = top_sensor.ranging_data.range_mm;
  return tir;
}

int measure_sideIR() {;
  //VL53L1X
  side_sensor.read();
  sir = side_sensor.ranging_data.range_mm;
  return sir;

}

float measure_US() {
  USnow = US2dist(analogRead(US_PIN)); //skip the average - this only works continuously
  return USnow;
}

void home_tray_zero()
//lowers the tray until the lower end-stop is touched and the system-zeros
{
  es.check_endstops();
  
  while (es.enDown) {
    if (!state) {
      break;
    }
    spb_v(-v_travel/4);
  
  }
  //stepper contacts lower end-stop - triggers min callback trayPosStp = Z_MIN_POS;
  delay(500);
  tic.exitSafeStart(); // just in case something happens
  delay(500);
  Serial.print("Home Tray Zero complete, tray at ");Serial.print(trayPosStp);Serial.println("mm");

//  long targetPosition = (Z_MIN_POS-50)*STEPSPERMM;
//  tic.setTargetPosition(targetPosition);
//  while (tic.getCurrentPosition()/(STEPSPERMM) > targetPosition)
//  {
//    tic.setTargetPosition(targetPosition);
//    delay(100);
//  }
//  Serial.println("ye has done");
  return;
}

void home_tray()
//lowers the tray to 5mm above lower end-stop
{
  es.check_endstops();
  long targetPosition = (Z_MIN_POS-5)*STEPSPERMM; // move 5mm away from lower endstop
  if (update_tray_pos()){return;}
  tic.setTargetPosition(targetPosition);
  while (es.enDown && (trayPosStp < (Z_MIN_POS-10))) 
  //run this loop while the stepper is beyond 10mm from Estop
  {
    if (!state) {
      break;
    }
    if (update_tray_pos()){break;}
    Serial.print(trayPosStp);Serial.print("target ");Serial.println(Z_MIN_POS-5);
    delay(1);
  }
  Serial.print("Home Tray complete, tray at ");Serial.print(trayPosStp);Serial.println("mm");
  return;
}

void set_lights(int lightVal) {
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, lightVal, lightVal, lightVal);
  }
  pixels.show();
}


void estop_LED() {
  //ESTOP CONDITION
  if (!state) {
    //flash LED
    long t1 = millis();
    if (t1 - LED_timer > 1250) {
      if (!LED_state) {
        digitalWrite(STARTLED, HIGH);
        LED_state = true;
      }
      else {
        digitalWrite(STARTLED, LOW);
        LED_state = false;
      }
      LED_timer = t1;
    }
  }
  //NH not connectected
  else if (state == 3) {
    //pulse LED
    long t1 = millis();
    if ((t1 - LED_timer > 1000) && (LED_state == false)) {
      digitalWrite(STARTLED, HIGH);
      LED_state = true;
      LED_timer = t1;
    }
    else if ((t1 - LED_timer > 500) && (LED_state == true)) {
      digitalWrite(STARTLED, LOW);
      LED_state = false;
      LED_timer = t1;
    }

  }
}
