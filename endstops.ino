#include "endstops.h"

//<constructor>
endstops::endstops(){
  endstops::endstops_init();
}

//<destructor>
endstops::~endstops(){}


void endstops::endstops_init() {
  pinMode(Z_MIN_PIN, INPUT_PULLUP);
  pinMode(Z_MAX_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Z_MAX_PIN), max_callback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Z_MIN_PIN), min_callback, CHANGE);
  enUp = false;
  enDown = false;
}

void endstops::check_endstops() {
  if (digitalRead(Z_MAX_PIN) == true){
    es.enUp = false;  //disable updward motion
    trayPosStp = Z_MAX_POS;
  }
  else es.enUp=true;
  
  if (digitalRead(Z_MIN_PIN) == true){
    es.enDown = false;  //disable downward motion
    trayPosStp = Z_MIN_POS;
  }
  else es.enDown=true;
}

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


//The other buttons

void buttons_init(){
  pinMode(ESTOP, INPUT_PULLUP);
  pinMode(STARTBTN, INPUT_PULLUP);
  pinMode(STARTLED, OUTPUT);
  digitalWrite(STARTLED, LOW);
  attachInterrupt(digitalPinToInterrupt(ESTOP), estop_callback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(STARTBTN), start_callback, CHANGE);
}

void check_estop(){
  if (digitalRead(ESTOP)==true){
    state = 0; //e-stopped
    stepper.stop();
    stepper.disable();
    digitalWrite(SOLENOID,LOW);
  }
  else {
    state = 1; //waiting
    digitalWrite(STARTLED, LOW);
    home_tray();
  }
}

void check_start(){
  if (digitalRead(STARTBTN)==false){
    state = 2;
    beer_time();
  }
}

void estop_callback(){
  if(ROS){nh.loginfo("Estop Callback");}
  if(!ROS){Serial.println("Estop Callback");}
  if (digitalRead(ESTOP)== true){
    //turn off stepper and solenoid
    state = 0; //e-stopped
    stepper.stop();
    stepper.disable();
    digitalWrite(SOLENOID,LOW);
    
  }
  else {
//    Serial.println("ESTOP Released");
    state = 1; //waiting
    digitalWrite(STARTLED, LOW);
//    home_tray();
  }
}


void start_callback(){
// NOT USED
  if (state == 1){
//      beer_time();  //intiate pouring process
      return;
  }
}
