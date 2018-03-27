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
  }
  else es.enUp=true;
  if (digitalRead(Z_MIN_PIN) == true){
    es.enDown = false;  //disable downward motion
  }
  else es.enDown=true;
}

