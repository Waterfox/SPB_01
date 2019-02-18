

void beer_time(){

  curRPM = 12; //lower RPM working: 12 max with no delay
  stepper.setRPM(curRPM);
  // 0. The tray is already lowered

  // Turn on red button LED
  digitalWrite(STARTLED, HIGH);
  
  // 1. Raise the tray ----------------------------
  nh.loginfo("Raising the Tray");
  //baseline side IR measurement
  sideIR=measure_sideIR();

  //Use the ultrasound to stop raising the tray -- should also use the tray position
  while (measure_US() > STOPDISTANCE){  // USE ULTRASOUND
//  while (trayPosStp - 15> STOPDISTANCE){  // USE Tray Position + offset

    if ((!state)) {return;}  //E-STOP
    if (nh.connected()==false) {return;}

    update_tray_pos();
    nh.spinOnce();
    
    wait_time_micros = stepper.nextAction();
    if (wait_time_micros <= 0) {
      stepper.step_count = 0;
      publish_sensors();
      
      //detect the top of the glass ***
      if ((measure_sideIR()/sideIR) > SIDEIRTHRESH && side_detected == false){
        side_detected = true;

        //the of glass location based on the tray position when the glass is detected
        glassTop = trayPosStp - SIDEIRPOS;
      }
      
      lastDirn = spb_move(MAX_STEPS);
    }
//    else delayMicroseconds(500);
//    else delay(1);
  }
  
  stepper.disable();

  //the bottom of the glass location based on
  glassBot = trayPosStp - STOPDISTANCE;

  if (side_detected) {
    glassHeight = glassTop-glassBot;
    if (glassHeight > 180) {
      glassHeight = 180;
    }
  }
  else {glassHeight = GLASSHEIGHT_DEFAULT;}
  glassHeight = GLASSHEIGHT_DEFAULT;


  //2. Begin filling! --------------------------------
  nh.loginfo("Begin filling!");
  curRPM = 5; //lower RPM  FOR SOME REASON THIS VALUE CANNOT BE SET TO 4 WTF
  stepper.setRPM(curRPM);
  digitalWrite(SOLENOID,true);  //Open the solenoid valve
  
//  while (trayPosStp - STOPDISTANCE < glassHeight - SURFOFFSET - (STOPDISTANCE - TUBEPOS)){
  while (trayPosStp - STOPDISTANCE < glassHeight - SURFOFFSET){
    wait_time_micros = stepper.nextAction();
      
    if (wait_time_micros <= 0) {
      update_tray_pos();
      publish_sensors();
      nh.spinOnce();
      if ((!state)) {return;}  //E-STOP
      if (nh.connected()==false) {
        digitalWrite(SOLENOID,LOW);
        return;
        }

      //Stop if there is too much foam!
//      if (ir_msg.data < trayPosStp - glassHeight + 10){
      if ((measure_topIR() < trayPosStp - glassHeight + 14)||(measure_US() < trayPosStp - glassHeight + 14))
      { 
        //--If foam is within 10mm from top of glass, break 
        nh.loginfo("Foam Alert!");
        break;
      }


      
//    Use the CV reading
      surfPos = surfPosCV;

//    CONTROL LOOP
      steps = 0;
      steps = int((SETPOINT + TUBEPOS - surfPos)*-STEPSPERMM); 
      // adjust the tray - only downwards
      if ((steps < 0) && (steps < -DEADBAND)) {
        /*
         *nh.loginfo("ctrl");
         *char output[8];
         *itoa(steps,output,10);
         *nh.loginfo(output);
         */
        lastDirn = spb_move(steps);
      } 
    }
    else delayMicroseconds(500);
  }
  
  digitalWrite(SOLENOID,false);  //Close the solenoid valve

//  3. Lower the Tray -------------------------------------
  nh.loginfo("Lowering the tray");
  home_tray();
  digitalWrite(STARTLED, LOW);
  nh.loginfo("Process complete");
  state=1;
  
}

