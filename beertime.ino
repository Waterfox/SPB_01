

void beer_time() {
  side_detected = false; //not used
  zero_glass_arr(glassArr, L);
  zero_glass_arr(glassDev, L);
  glassAv = 0;
  glassStdDev = 0;
  glassN = 0; //number of glass detections
  glassHeight = 0; //default low glass height val
  curRPM = 12; //lower RPM working: 12 max with no delay
  stepper.setRPM(curRPM);
  useGlassHeightDetection = true;
  useCVGlassHeightDetection = false;
  defaultGlassHeight = GLASSHEIGHT_DEFAULT;
  //Use parameter server to set glass height params
  if (ROS)
  {
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
  }
  // 0. The tray is already lowered

  // Turn on red button LED
  digitalWrite(STARTLED, HIGH);

  // 1. Raise the tray --------------------------------------------
  if(ROS){nh.loginfo("Raising the Tray");}
  if(!ROS){Serial.println("Raising the Tray");}

  for (short i; i < 20; i++) { // fill the US average
    USnow = measure_US();
  }
  measure_topIR();

  //Use the ultrasound and top infraredto stop raising the tray
  //  while ((measure_US() > STOPDISTANCE) && (es.enUp == true))
  //  while (trayPosStp - 15> STOPDISTANCE){  // USE Tray Position + offset
  //  while(es.enUp)
  while ((USnow > STOPDISTANCE && tir > STOPDISTANCE ) && (es.enUp == true)) /// UNTESTED
  {
    if ((!state)) {
      return; //E-STOP
    }

    wait_time_micros = stepper.nextAction();
    long t4 = millis();
    
    if (wait_time_micros <= 0) {

      if (nh.connected() == false) {
        return;
      }
      if (t4 - loop_timer4 > 50) { // timer for ultrasound read
        USnow = measure_US();
        loop_timer4 = t4;
//        measure_topIR();
      }
//      stepper.stop();
      side_ghd (); 
//      stepper.enable();
      update_tray_pos();

      if(ROS){nh.spinOnce();}
//      publish_sensors();


      lastDirn = spb_move(MAX_STEPS);
    }
    else if (wait_time_micros > 300) {
      
      long t3 = millis();
      if (t3 - loop_timer3 > 200) { //timer for SPB move
        update_tray_pos();
        if(ROS){nh.spinOnce();}

        if (t4 - loop_timer4 > 300) { //timer for ultrasound read
          USnow = measure_US();
          measure_topIR();
          loop_timer4 = t4;
          side_ghd (); 
        }
        lastDirn = spb_move(MAX_STEPS);
        //       USnow = measure_US();
        loop_timer3 = t3;
      }
    }
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
  if(ROS){nh.loginfo("Begin filling!");}
  if(!ROS){Serial.println("Begin filling");}
  curRPM = 8; //lower RPM  FOR SOME REASON THIS VALUE CANNOT BE SET TO 4 WTF
  stepper.setRPM(curRPM);
  //  stepper.enable();
  //  spb_move(-100);
  digitalWrite(SOLENOID, true); //Open the solenoid valve
  delay(800);
  loop_timer3 = 0;

  while (trayPosStp - TUBEPOS < glassHeight - SURFOFFSET && es.enDown) {
    wait_time_micros = stepper.nextAction();

    if (wait_time_micros <= 0) {
      update_tray_pos();
      if (ROS){
        publish_sensors();
        nh.spinOnce();
      }
      if ((!state)) {
        return; //E-STOP
      }

      if(ROS){
        if (nh.connected() == false) {
          digitalWrite(SOLENOID, LOW);
          return;
        }
      }

      //FOAM ALERT stop if there is too much foam
      //      if (ir_msg.data < trayPosStp - glassHeight + 10){
//      tir = measure_topIR();
      if ((tir < trayPosStp - glassHeight + SURFOFFSET) || (measure_US() < trayPosStp - glassHeight + SURFOFFSET))
      {
        //--If foam is within 10mm from top of glass, break
        if(ROS){nh.loginfo("Foam Alert!");}
        if(!ROS){Serial.println("Foam Overflow");}
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
        if(ROS){
          if (nh.connected() == false) {
            digitalWrite(SOLENOID, LOW);
            return;
          }
        }
        //FOAM ALERT stop if there is too much foam
//        measure_topIR();
        if ((tir < trayPosStp - glassHeight + SURFOFFSET) || (measure_US() < trayPosStp - glassHeight + SURFOFFSET))
        {
          //--If foam is within 10mm from top of glass, break
          if(ROS){nh.loginfo("Foam Alert!");}
          if(!ROS){Serial.println("Foam Overflow");}
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
  if(ROS){nh.loginfo("Lowering the tray");}
  if(!ROS){Serial.println("Lowering the tray");}
  home_tray();
  digitalWrite(STARTLED, LOW);
  if(ROS){nh.loginfo("Process complete");}
  if(!ROS){Serial.println("Process complete");}
  state = 1;

}
