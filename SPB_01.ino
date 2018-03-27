
#include <Arduino.h>
#include "SPB.h"
#include "endstops.h"
#include "BasicStepperDriver.h"




DRV8825 stepper(MOTOR_STEPS, Z_DIR_PIN, Z_STEP_PIN, Z_ENABLE_PIN, MODE0, MODE1, MODE2);

float topIRAvg = 0;
endstops es;
char inByte = '0';

void setup() {
  pinMode(SOLENOID,OUTPUT);
  Serial.begin(115200);
  stepper.begin(RPM, MICROSTEPS);
  stepper.enable();
  es.check_endstops();
  //stepper.startMove(-MOTOR_STEPS*MICROSTEPS*5); //"+ve" is up"
}

//----------------------------------------------
void loop() {
  unsigned wait_time_micros = stepper.nextAction();
  if (wait_time_micros <= 0) {
    stepper.disable();
    manual_scroll();

    int setPoint = 240;
    float trayPos = measure_topIR();
    int steps = ((setPoint - trayPos)*-50.0);   // 50 steps per mm travel
    int outsteps = spb_move(steps);
    Serial.println(trayPos);
  }


  // (optional) execute other code if we have enough time
  if (wait_time_micros > 100){
    Serial.print("top_ir: ");Serial.println(measure_topIR());
  }
}
//------------------------------------------------


//interrupt callbacks can't be class members :S 
void max_callback(){
  stepper.stop();
  stepper.disable();
  if (digitalRead(Z_MAX_PIN) == true){
    es.enUp = false;  //disable updward motion
  }
  else es.enUp=true;
}

void min_callback(){
  stepper.stop();
  stepper.disable();
  if (digitalRead(Z_MIN_PIN) == true){
    es.enDown = false;  //disable downward motion
  }
  else es.enDown=true;
  
}

//convert IR measurement to distance in mm
float topIR2dist(float topVal){
  return(26734.0*pow(topVal,-0.883));
}

//Read the serial port and scroll up or down
void manual_scroll(){
  if (Serial.available() > 0) {
    inByte = Serial.read();
    if (inByte == '8'){
      stepper.enable();
      stepper.startMove(200);
    }
    else if (inByte == '2'){
      stepper.enable();
      stepper.startMove(-200);
    }
  }
}

//Check endstop conditions and move the stepper motor
//return # of steps moved
int spb_move(int steps){ 
  if (abs(steps) > DEADBAND){
    if (steps > MAX_STEPS && es.enUp == true){
        stepper.enable();
        stepper.startMove(MAX_STEPS);
        return MAX_STEPS;
    }
    else if(steps < -MAX_STEPS && es.enDown == true){
        stepper.enable();
        stepper.startMove(-MAX_STEPS);
        return MAX_STEPS;
    }
    else if ((steps>0 && es.enUp ==true) || (steps<0 && es.enDown == true)){
      stepper.enable();
      stepper.startMove(steps);
      return steps;
    }  
  }
}

float measure_topIR() {
  float topIRVal = analogRead(TOP_IR_PIN)*0.1+topIRAvg*0.9;
  topIRAvg = topIRVal;
  return topIR2dist(topIRAvg);
}

  


