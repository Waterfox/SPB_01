

void beer_time(){
  digitalWrite(STARTLED, HIGH);
  
  // 1. Raise the tray ----------------------------
  Serial.println("Begin raising the tray");
  int STOPDISTANCE = 190;
  sideIR=measure_sideIR();
  
  while (measure_US() > STOPDISTANCE){
//    Serial.println(trayPosStp);
    unsigned wait_time_micros_1 = stepper.nextAction();
    if (wait_time_micros_1 <= 0) {
      update_tray_pos();

      //detect the top of the glass ***
      if ((measure_sideIR()/sideIR) > SIDEIRTHRESH && side_detected == false){
        side_detected = true;
        glassTop = trayPosStp - SIDEIRPOS;
      }
      
      lastDirn = spb_move(200);
    }
    else delay(1);
  }
  
  stepper.disable();
  glassBot = trayPosStp - STOPDISTANCE;
//  int glassHeight = glassTop-glassBot;
  int glassHeight = 165;
  Serial.print("glass top position is: "); Serial.println(glassTop);
  Serial.print("glass bottom position is: "); Serial.println(glassBot);
  Serial.print("glass height is: "); Serial.println(glassTop-glassBot);
  

  //2. Begin filling! --------------------------------
  digitalWrite(SOLENOID,true);  //Open the solenoid valve
  
  int setPoint = 15; // the tube will be X mm from surface
  int SURFOFFSET = 70; // fill the glass this far from the glass top
  
  while (trayPosStp - STOPDISTANCE < glassHeight - SURFOFFSET){
    unsigned wait_time_micros = stepper.nextAction();

    //stepper stopped
    if (wait_time_micros <= 0) {
      stepper.disable();
      update_tray_pos();
  //    Serial.print("stp pos: ");Serial.print(trayPosStp);Serial.print(" ultrasound: ");Serial.print(measure_US()); Serial.print(" top_ir: ");Serial.println(measure_topIR());
      
//    float  trayPos = measure_topIR() -10.0;
      float trayPos = measure_US();
      int steps = ((setPoint + TUBEPOS - trayPos)*-STEPSPERMM);   // 50 steps per mm travel
      lastDirn = spb_move(steps);
    }
  
  
    // //stepper in motion execute other code if we have enough time
//    if (wait_time_micros > 100){
//      delay(1);
//    } 
    else delay(1);
  }
  digitalWrite(SOLENOID,false);  //Open the solenoid valve

//  3. Lower the Tray -------------------------------------
  home_tray();
  digitalWrite(STARTLED, LOW);
  
}

