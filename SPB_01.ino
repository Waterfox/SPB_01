
#include <Arduino.h>
#include "SPB.h"
#include "endstops.h"
#include "BasicStepperDriver.h"




DRV8825 stepper(MOTOR_STEPS, Z_DIR_PIN, Z_STEP_PIN, Z_ENABLE_PIN, MODE0, MODE1, MODE2);

float topIRAvg = 0;
float sideIR = 0;
endstops es;
char inByte = '0';
float trayPosStp = 0; //tray position from stepper count - measured from the top down
float trayPosIR = 0; //tray position from IR measurement - measured from the sensor down
float trayPosUS = 0; //tray position from Ultrasound measurement - measured from the sensor down
int lastDirn = 0;
int MODE = 0; // 0: tray level, 1: manual override
int state = 0; //0: Estopped, 1:Waiting, 2:Pouring

bool side_detected = false;
float glassTop = 0; 
float glassBot = 0;


//**********************************************
void setup() {
  pinMode(SOLENOID,OUTPUT);
  pinMode(US_PWR,OUTPUT);
  Serial.begin(115200);
  stepper.begin(RPM, MICROSTEPS);
  stepper.enable();
  es.check_endstops();
  buttons_init();
//  digitalWrite(SOLENOID,true);
  digitalWrite(US_PWR,true);  //turn on the ultrasound
//  home the stepper motor

  home_tray();
}

//----------------------------------------------
void loop() {

  check_estop();
  check_start();
  
  unsigned wait_time_micros = stepper.nextAction();
  if (wait_time_micros <= 0) {
    stepper.disable();
    update_tray_pos();
    manual_scroll();
    Serial.print("stp pos: ");Serial.print(trayPosStp);Serial.print(" ultrasound: ");Serial.print(measure_US()); Serial.print(" top_ir: ");Serial.println(measure_topIR());
      
  }
  else delay(1);

}
//------------------------------------------------




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

int measure_sideIR() {
  return analogRead(SIDE_IR_PIN);
}

float measure_US() {
  int usVal = analogRead(US_PIN);
  return US2dist(usVal);
}
  

void home_tray(){
  Serial.println("");
  Serial.println("Begin homing the tray");
  while (es.enDown){
    unsigned wait_time_micros_1 = stepper.nextAction();
    if (wait_time_micros_1 <= 0) {
//      stepper.startMove(-MOTOR_STEPS*MICROSTEPS); //"+ve" is up
        spb_move(-600);
    }
    else {
      delay(1);
    }
  }
  stepper.disable();
  Serial.println("Tray Initialized"); 
}


