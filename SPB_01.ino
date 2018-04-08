
#include <Arduino.h>
#include "SPB.h"
#include "endstops.h"
#include "BasicStepperDriver.h"




DRV8825 stepper(MOTOR_STEPS, Z_DIR_PIN, Z_STEP_PIN, Z_ENABLE_PIN, MODE0, MODE1, MODE2);

float topIRAvg = 0;
endstops es;
char inByte = '0';
float trayPosStp = 0; //tray position from stepper count - measured from the top down
float trayPosIR = 0; //tray position from IR measurement - measured from the sensor down
float trayPosUS = 0; //tray position from Ultrasound measurement - measured from the sensor down
int lastDirn = 0;
int MODE = 0; // 0: tray level, 1: manual override


//**********************************************
void setup() {
  pinMode(SOLENOID,OUTPUT);
  pinMode(US_PWR,OUTPUT);
  Serial.begin(115200);
  stepper.begin(RPM, MICROSTEPS);
  stepper.enable();
  es.check_endstops();
  digitalWrite(SOLENOID,true);
  digitalWrite(US_PWR,true);
//  home the stepper motor
  Serial.println("");
  Serial.println("Begin homing the tray");
  while (es.enDown){
    unsigned wait_time_micros_1 = stepper.nextAction();
    if (wait_time_micros_1 <= 0) {
      stepper.startMove(-MOTOR_STEPS*MICROSTEPS); //"+ve" is up
    }
  }
  Serial.println("Tray Initialized"); 
}

//----------------------------------------------
void loop() {
  unsigned wait_time_micros = stepper.nextAction();

  //stepper stopped
  if (wait_time_micros <= 0) {
    stepper.disable();
    update_tray_pos();
    Serial.print("stp pos: ");Serial.print(trayPosStp);Serial.print(" ultrasound: ");Serial.print(measure_US()); Serial.print(" top_ir: ");Serial.println(measure_topIR());
    manual_scroll();

    if (MODE == 0){ // if in auto position mode
      int setPoint = 198;
      trayPosIR = measure_topIR();
      trayPosUS = measure_US();
      int steps = ((setPoint - trayPosIR)*-50.0);   // 50 steps per mm travel
      lastDirn = spb_move(steps);
    }
  }


  // //stepper in motion execute other code if we have enough time
  if (wait_time_micros > 100){
    delay(1);
  }
}
//------------------------------------------------


//interrupt callbacks can't be class members :S 
void max_callback(){
  stepper.stop();
  stepper.disable();
  if (digitalRead(Z_MAX_PIN) == true){
    es.enUp = false;  //disable updward motion
    trayPosStp = Z_MAX_POS;
  }
  else es.enUp=true;
}

void min_callback(){
  stepper.stop();
  stepper.disable();
  if (digitalRead(Z_MIN_PIN) == true){
    es.enDown = false;  //disable downward motion
    trayPosStp = Z_MIN_POS;
  }
  else es.enDown=true;
  
}

//convert IR measurement to distance in mm (distance from sensor)
float topIR2dist(float topVal){
//  return(26734.0*pow(topVal,-0.883));
    return 0.0038*topVal*topVal - 2.544*topVal+588.85;
}

//conver ultrasound measurement to distance in mm (distance from sensor)
float US2dist(int usVal){
  return(0.1466275*usVal + 100.0);   //150.0mm/1023.0 * usVal + 100mm
}

//Read the serial port and scroll up or down
void manual_scroll(){
  if (Serial.available() > 0) {
    inByte = Serial.read();
    if (inByte == '5'){
      MODE = !MODE;
    }
    if (MODE == 1){ // if in manual mode
      if (inByte == '8'){
        lastDirn = spb_move(200*2);
      }
      else if (inByte == '2'){
        lastDirn = spb_move(-200*2);
      }
    }
  }
}

//Check endstop conditions and move the stepper motor
//return direction of steps moved
int spb_move(int steps){ 
  if (abs(steps) > DEADBAND){
    if (steps > MAX_STEPS && es.enUp == true){
        stepper.enable();
        stepper.startMove(MAX_STEPS);
        return 1;
    }
    else if(steps < -MAX_STEPS && es.enDown == true){
        stepper.enable();
        stepper.startMove(-MAX_STEPS);
        return -1;
    }
    else if ((steps>0 && es.enUp ==true) || (steps<0 && es.enDown == true)){
      stepper.enable();
      stepper.startMove(steps);
      return ((steps > 0) - (steps < 0)); 
    }
    else return 0;  
  }
  return 0;
}

void update_tray_pos(void){
    trayPosStp = trayPosStp - (stepper.step_count*lastDirn / 50.0); // distance travelled in mm
    stepper.step_count = 0;
}

float measure_topIR() {
  float topIRVal = analogRead(TOP_IR_PIN)*0.05+topIRAvg*0.95;
  topIRAvg = topIRVal;
  return topIR2dist(topIRAvg);
//  return topIRAvg;
}


float measure_US() {
  int usVal = analogRead(US_PIN);
  return US2dist(usVal);
}
  


