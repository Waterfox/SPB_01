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
    tic.haltAndSetPosition(Z_MAX_POS*STEPSPERMM);
  }
  else es.enUp=true;
  
  if (digitalRead(Z_MIN_PIN) == true){
    es.enDown = false;  //disable downward motion
    trayPosStp = Z_MIN_POS;
    tic.haltAndSetPosition(Z_MIN_POS*STEPSPERMM); 
  }
  else es.enDown=true;
}

//interrupt callbacks can't be class members :S 

void max_callback(){
//  top switch
//  tic.haltAndSetPosition(Z_MAX_POS*STEPSPERMM);
  if (digitalRead(Z_MAX_PIN) == true){
    tic.haltAndSetPosition(Z_MAX_POS*STEPSPERMM);
    es.enUp = false;  //disable updward motion
    trayPosStp = Z_MAX_POS; // 187 (measured from top down)
  }
  else es.enUp=true;
}

void min_callback(){
//  bottom switch 
//  tic.haltAndSetPosition(Z_MIN_POS*STEPSPERMM); 
//  STEPSPERMM
  if (digitalRead(Z_MIN_PIN) == true){
    tic.haltAndSetPosition(Z_MIN_POS*STEPSPERMM); 
    es.enDown = false;  //disable downward motion
    trayPosStp = Z_MIN_POS;   // 365 (measured from top down)
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
//    tic.haltAndHold();
    spb_v(0);
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

void check_tof_start(){
  measure_sideIR();
  if (sir < SIDEIRTHRESH){
    //start
    long TofStartTimer = millis();
    int fakeHits = 0;
    int maxFakeHits = 20;
    while ((sir < SIDEIRTHRESH)||(fakeHits<maxFakeHits)){
      measure_sideIR();
      if (sir>SIDEIRTHRESH){fakeHits++;};
      delay(30);
    }
    long t1 = millis();
    if ((t1 - TofStartTimer > 2000) && (t1 - TofStartTimer < 5000)){
      state = 2;
      beer_time();
    }
    TofStartTimer = 0;
  }
}

void check_clean(){
  if (digitalRead(STARTBTN)==false){
    long cleanStartTimer = millis();
    while (digitalRead(STARTBTN)==false){
      if (millis() - cleanStartTimer > 2000){
        digitalWrite(SOLENOID, true); //Open the solenoid valve
      }
    }
  }
  digitalWrite(SOLENOID, false); //Open the solenoid valve
}

void estop_callback(){

  Serial.println("Estop Callback");
  if (digitalRead(ESTOP)== true){
    //turn off stepper and solenoid
    state = 0; //e-stoppe);
//    tic.haltAndHold();
    spb_v(0);
    digitalWrite(SOLENOID,LOW);
    
  }
  else {
    Serial.println("ESTOP Released");
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
