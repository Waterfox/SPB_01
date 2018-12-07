

void beer_time(){

  // 0. The tray is already lowered

  // Turn on red button LED
  digitalWrite(STARTLED, HIGH);
  
  // 1. Raise the tray ----------------------------
  nh.loginfo("Raising the Tray");
  //baseline side IR measurement
  sideIR=measure_sideIR();

  //Use the ultrasound to stop raising the tray -- should also use the tray position
//  while (measure_US() > STOPDISTANCE){  // USE ULTRASOUND
  while (trayPosStp - 15> STOPDISTANCE){  // USE Tray Position + offset
    if (!state) {return;}  //E-STOP

    nh.spinOnce();
    
//    Serial.println(trayPosStp);
    unsigned wait_time_micros_1 = stepper.nextAction();
    if (wait_time_micros_1 <= 0) {
      update_tray_pos();
      publish_sensors();
//      publish_tray();
      
      //detect the top of the glass ***
      if ((measure_sideIR()/sideIR) > SIDEIRTHRESH && side_detected == false){
        side_detected = true;

        //the of glass location based on the tray position when the glass is detected
        glassTop = trayPosStp - SIDEIRPOS;
      }
      
      lastDirn = spb_move(400);
    }
    else delay(1);

  }
  
  stepper.disable();

  //the bottom of the glass location based on
  glassBot = trayPosStp - STOPDISTANCE;

  if (side_detected) {glassHeight = glassTop-glassBot;}
  else {glassHeight = 165;}
//  glassHeight = 160;


  //2. Begin filling! --------------------------------
  nh.loginfo("Begin filling!");
//  curRPM = 10; //lower RPM
//  stepper.setRPM(curRPM);
  digitalWrite(SOLENOID,true);  //Open the solenoid valve
  
  
//  while (trayPosStp - STOPDISTANCE < glassHeight - SURFOFFSET - (STOPDISTANCE - TUBEPOS)){
  while (trayPosStp - STOPDISTANCE < glassHeight - SURFOFFSET){
    
    
    unsigned wait_time_micros = stepper.nextAction();
    
    
    //stepper stopped
    if (wait_time_micros <= 0) {
      if (!state) {break;}  //E-STOP
      nh.spinOnce();
      update_tray_pos();
      publish_sensors(); 
//      publish_tray();

      //Stop if there is too much foam!
//      if (measure_topIR() < trayPosStp - glassHeight + 10){ 
//        //--If foam is within 10mm from top of glass, break 
//        nh.loginfo("Foam Alert!");
//        break;
//      }
      
//    Use the CV reading
      surfPos = surfPosCV;

//    CONTROL LOOP
      int steps = ((SETPOINT + TUBEPOS - surfPos)*-STEPSPERMM); 
      // adjust the tray - only downwards  -NOT WORKING
      if (steps < 0) {
        lastDirn = spb_move(steps);
      }
              
//    increase speed if level is high
//      long t2 = millis();   
//      if ((steps > 1000) && (t2 - RPM_timer > 500) && (curRPM <=12))  {
//        curRPM = curRPM +1;
//        nh.loginfo(curRPM);
//        RPM_timer = t2;
//      }
//    decrease speed if level is low
//      if ((steps < 1000) && (t2 - RPM_timer > 500) && (curRPM >=2)) {
//        curRPM = curRPM -1;
//        nh.loginfo(curRPM);
//        RPM_timer = t2;
//      }

      
    }
  
    else delay(1);
  }
  digitalWrite(SOLENOID,false);  //Close the solenoid valve

//  3. Lower the Tray -------------------------------------
  nh.loginfo("Lowering the tray");
  home_tray();
  digitalWrite(STARTLED, LOW);
  nh.loginfo("Process complete");
  state=1;
  
}

