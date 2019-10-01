void trayUpLoop()
{
  long t4 = millis();
  if (t4 - loop_timer4 > 200) { // timer for ultrasound read
    measure_US();
    measure_topIR();
    side_ghd (); 
    update_tray_pos();
    if(ROS){nh.spinOnce();}
//      publish_sensors();
    loop_timer4 = t4;
  }
}

bool trayDownLoop()
{
  update_tray_pos();
  if (ROS){
    publish_sensors();
    nh.spinOnce();
  }
  measure_topIR();
  measure_US();
  //FOAM ALERT stop if there is too much foam
//      tir = measure_topIR();
  if ((tir < trayPosStp - glassHeight + SURFOFFSET) || (USnow < trayPosStp - glassHeight + SURFOFFSET))
  {
    //--If foam is within 10mm from top of glass, break
    if(ROS){nh.loginfo("Foam Alert!");}
    if(!ROS){Serial.println("Foam Overflow");}
    return 0;
  }

  //    Use the CV reading
//      surfPos = surfPosCV;

  //    Use top UR reading
  surfPos = min(tir,USnow);

  //    CONTROL LOOP
  steps = 0;
  steps = int((SETPOINT + TUBEPOS - surfPos) * -STEPSPERMM);
  // adjust the tray - only downwards
  if ((steps < 0) && (steps < -DEADBAND)) {
//        nh.loginfo("ctrl");
//        char output[8];
//        itoa(steps, output, 10);
//        nh.loginfo(output);
    lastDirn = spb_move(steps);
  }
  return 1;
}

void beer_time() {
  side_detected = false; //not used
  zero_glass_arr(glassArr, L);
  zero_glass_arr(glassDev, L);
  glassAv = 0;
  glassStdDev = 0;
  glassN = 0; //number of glass detections
  glassHeight = 0; //default low glass height val

  curRPM = 10; //lower RPM working: 12 max with no delay

  stepper.setRPM(curRPM);
  useGlassHeightDetection = true;
  useCVGlassHeightDetection = false;
  defaultGlassHeight = GLASSHEIGHT_DEFAULT;
  get_ros_params();
  // 0. The tray is already lowered

  // Turn on red button LED
  digitalWrite(STARTLED, HIGH);

  // 1. Raise the tray --------------------------------------------
  if(ROS){nh.loginfo("Raising the Tray");}
  if(!ROS){Serial.println("Raising the Tray");}

  Serial.println(measure_US());
  Serial.println(measure_topIR());
  Serial.println(state);

  //Use the ultrasound and top infraredto stop raising the tray
  while ((USnow > STOPDISTANCE && tir > STOPDISTANCE ) && (es.enUp == true)) /// UNTESTED
  {
    if ((!state)) {
      return; //E-STOP
    }

    wait_time_micros = stepper.nextAction();

    
    if (wait_time_micros <= 0) {
      if (ROS){
        if (nh.connected() == false) {
          return;
        }
      }
//      Serial.print("-");
      trayUpLoop();
      Serial.println(tir);

      lastDirn = spb_move(MAX_STEPS);
      nh.loginfo(".");
    }

    else if (wait_time_micros > 300) {
//      Serial.print("+");
      trayUpLoop();
//      lastDirn = spb_move(MAX_STEPS); 
    }

  }



  stepper.stop();
  stepper.disable();
  delay(300);

  process_glass_height();
  if (glassHeight < MINGLASSHEIGHT){
    if(!ROS){Serial.print("GH below min height of ");Serial.println(MINGLASSHEIGHT);}
    return;}
  spb_move(-500); // start moving down?
  delay(500);
  stepper.stop();
  stepper.disable();


  //2. Begin filling! --------------------------------
  if(ROS){nh.loginfo("Begin filling!");}
  if(!ROS){Serial.println("Begin filling");}
  
  curRPM = 8; //lower RPM  FOR SOME REASON THIS VALUE CANNOT BE SET TO 4 WTF
  stepper.setRPM(curRPM);


  digitalWrite(SOLENOID, true); //Open the solenoid valve
  delay(800);
  loop_timer3 = 0;

  while (trayPosStp - TUBEPOS < glassHeight - SURFOFFSET && es.enDown) {
    wait_time_micros = stepper.nextAction();
    if (ROS){
      if (nh.connected() == false) {

      digitalWrite(SOLENOID, LOW);
      return; }
    } 
    if ((!state)) {
      return; //E-STOP
    }


    if (wait_time_micros <= 0) {
      if(!trayDownLoop()){break;}
//      Serial.print("-");
        Serial.print(tir);Serial.print(" - ");Serial.print(USnow);Serial.print(" - ");Serial.println(trayPosStp); // debug
    }
    else if (wait_time_micros > 300) {
      long t3 = millis();
      if (t3 - loop_timer3 > 200) { //timer for SPB move

//        if(!trayDownLoop()){break;} // Broken must move step fn out of this
//        Serial.print("+");

        loop_timer3 = t3;
      }
    }
  }

  digitalWrite(SOLENOID, false); //Close the solenoid valve

  //  3. Lower the Tray -------------------------------------
  if(ROS){nh.loginfo("Lowering the tray");}
  if(!ROS){Serial.println("\nLowering the tray");}
  home_tray();
  digitalWrite(STARTLED, LOW);
  if(ROS){nh.loginfo("Process complete");}
  if(!ROS){Serial.println("Process complete");}
  state = 1;

}
