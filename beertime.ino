

void beer_time() {
  side_detected = false; //not used
  zero_glass_arr(glassArr, L);
  zero_glass_arr(glassDev, L);
  glassAv = 0;
  glassStdDev = 0;
  glassN = 0; //number of glass detections
  glassHeight = 0; //default low glass height val
  curRPM = 9; //lower RPM working: 12 max with no delay
  stepper.setRPM(curRPM);
  useGlassHeightDetection = true;
  useCVGlassHeightDetection = false;
  defaultGlassHeight = GLASSHEIGHT_DEFAULT;
  //Use parameter server to set glass height params
  if (nh.getParam("useGHD", &useGlassHeightDetection))
  {
    nh.loginfo("Retrieved param");
    if (useGlassHeightDetection) {
      nh.loginfo("using glass height detection");
    }
    //    else {nh.loginfo("NOT using glass height detection");}
  }
  if (!useGlassHeightDetection)
  {
    nh.loginfo("No GH detection");
    //If there us a glass height parameter use it
    if (nh.getParam("GH", &defaultGlassHeight))
    {
      nh.loginfo("GH from params is:");
      char output2[4];
      itoa(defaultGlassHeight, output2, 10);
      nh.loginfo(output2);
    }
    //Otherwise take the firmware default of 160
    else
    {
      nh.loginfo("Default GH is:");
      char output2[4];
      itoa(defaultGlassHeight, output2, 10);
      nh.loginfo(output2);
    }
  }

  // 0. The tray is already lowered

  // Turn on red button LED
  digitalWrite(STARTLED, HIGH);

  // 1. Raise the tray --------------------------------------------
  nh.loginfo("Raising the Tray");

  for (short i; i < 20; i++) { // fill the US average
    USnow = measure_US();
  }
  measure_topIR();

  //Use the ultrasound to stop raising the tray
  //  while ((measure_US() > STOPDISTANCE) && (es.enUp == true))
  //  while (trayPosStp - 15> STOPDISTANCE){  // USE Tray Position + offset
  //  while(es.enUp)
  while ((USnow > STOPDISTANCE && tir > STOPDISTANCE ) && (es.enUp == true)) /// UNTESTED
  {
    if ((!state)) {
      return; //E-STOP
    }

    wait_time_micros = stepper.nextAction();
    long t5 = millis();
    long t6 = millis();
    long t3 = millis();
    long t4 = millis();
    
    if (wait_time_micros <= 0) {

      if (nh.connected() == false) {
        return;
      }
      if (t5 - loop_timer5 > 200) { // timer for ultrasound read
//        USnow = measure_US();
        loop_timer5 = t5;
        measure_topIR();
        side_ghd (); 
//        update_tray_pos();
        nh.spinOnce();
        //      publish_sensors();
      }
      if (t6 - loop_timer6 > 200) { // timer for ultrasound read
        USnow = measure_US();
        loop_timer6 = t6;
//        measure_topIR();
//        side_ghd (); 
        update_tray_pos();
//        nh.spinOnce();
        //      publish_sensors();
      }

      lastDirn = spb_move(MAX_STEPS);
      nh.loginfo(".");
    }
//    else if (wait_time_micros > 400) {
//      
//
//      if (t4 - loop_timer4 > 200) { // timer for ultrasound read
////        USnow = measure_US();
//        loop_timer4 = t4;
////        measure_topIR();
////        side_ghd (); 
////        update_tray_pos();
//        nh.spinOnce();
//        //      publish_sensors();
//      }
//      if (t3 - loop_timer3 > 200) { // timer for ultrasound read
//        USnow = measure_US();
//        loop_timer3 = t3;
////        measure_topIR();
////        side_ghd (); 
//        update_tray_pos();
////        nh.spinOnce();
//        //      publish_sensors();
//      }
////       lastDirn = spb_move(MAX_STEPS);
//    nh.loginfo("+");
//    }
  }



  stepper.stop();
  stepper.disable();
  delay(300);

  process_glass_height();
  
  spb_move(-500); // start moving down?
  delay(500);
  stepper.stop();
  stepper.disable();


  //2. Begin filling! --------------------------------
  nh.loginfo("Begin filling!");
  curRPM = 8; //lower RPM  FOR SOME REASON THIS VALUE CANNOT BE SET TO 4 WTF
  stepper.setRPM(curRPM);
  //  stepper.enable();
  //  spb_move(-100);
  digitalWrite(SOLENOID, true); //Open the solenoid valve
  delay(800);
  loop_timer3 = 0;
  //  while (trayPosStp - STOPDISTANCE < glassHeight - SURFOFFSET - (STOPDISTANCE - TUBEPOS)){
  //  while (trayPosStp - STOPDISTANCE < glassHeight - SURFOFFSET){
  while (trayPosStp - TUBEPOS < glassHeight - SURFOFFSET && es.enDown) {
    wait_time_micros = stepper.nextAction();

    if (wait_time_micros <= 0) {
      update_tray_pos();
      publish_sensors();
      nh.spinOnce();
      if ((!state)) {
        return; //E-STOP
      }
      if (nh.connected() == false) {
        digitalWrite(SOLENOID, LOW);
        return;
      }

      //FOAM ALERT stop if there is too much foam
      //      if (ir_msg.data < trayPosStp - glassHeight + 10){
      tir = measure_topIR();
      if ((tir < trayPosStp - glassHeight + SURFOFFSET) || (measure_US() < trayPosStp - glassHeight + SURFOFFSET))
      {
        //--If foam is within 10mm from top of glass, break
        nh.loginfo("Foam Alert!");
        break;
      }



      //    Use the CV reading
//      surfPos = surfPosCV;

      //    Use top UR reading
      surfPos = tir;

      //    CONTROL LOOP
      steps = 0;
      steps = int((SETPOINT + TUBEPOS - surfPos) * -STEPSPERMM);
      // adjust the tray - only downwards
      if ((steps < 0) && (steps < -DEADBAND)) {

        nh.loginfo("ctrl");
        char output[8];
        itoa(steps, output, 10);
        nh.loginfo(output);

        lastDirn = spb_move(steps);
      }
    }
    else if (wait_time_micros > 300) {
      long t3 = millis();
      if (t3 - loop_timer3 > 200) { //timer for SPB move
        update_tray_pos();
        publish_sensors();
        nh.spinOnce();
        if ((!state)) {
          return; //E-STOP
        }
        if (nh.connected() == false) {
          digitalWrite(SOLENOID, LOW);
          return;
        }
        //FOAM ALERT stop if there is too much foam
        measure_topIR();
        if ((tir < trayPosStp - glassHeight + SURFOFFSET) || (measure_US() < trayPosStp - glassHeight + SURFOFFSET))
        {
          //--If foam is within 10mm from top of glass, break
          nh.loginfo("Foam Alert!");
          break;
        }
        //    Use the CV reading
//        surfPos = surfPosCV;
      //    Use top UR reading
        surfPos = tir;  

        //    CONTROL LOOP
        steps = 0;
        steps = int((SETPOINT + TUBEPOS - surfPos) * -STEPSPERMM);
        // adjust the tray - only downwards
        if ((steps < 0) && (steps < -DEADBAND)) {
          lastDirn = spb_move(steps);
        }
        loop_timer3 = t3;
      }
    }
  }

  digitalWrite(SOLENOID, false); //Close the solenoid valve

  //  3. Lower the Tray -------------------------------------
  nh.loginfo("Lowering the tray");
  home_tray();
  digitalWrite(STARTLED, LOW);
  nh.loginfo("Process complete");
  state = 1;

}
