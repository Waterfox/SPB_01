void zero_glass_arr(int b[], int arrSize){
  for (short i=0;i<arrSize;i++){
    b[i]=0;
  }
}

void beer_time(){
  side_detected = false;
  zero_glass_arr(glassArr,L);
  glassN = 0; //number of glass detections
  glassHeight = 0; //default low glass height val
  
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
  while (measure_US() > STOPDISTANCE){  // USE ULTRASOUND  || measure_topIR() <STOPDISTANCE)
//  while (trayPosStp - 15> STOPDISTANCE){  // USE Tray Position + offset

    if ((!state)) {return;}  //E-STOP
    if (nh.connected()==false) {return;}

    update_tray_pos();
    nh.spinOnce();
    
    wait_time_micros = stepper.nextAction();
    if (wait_time_micros <= 0) {
      stepper.step_count = 0;
      publish_sensors();
      
      //DETECT GLASS HEIGHT USING IR ***
      /*
      if ((measure_sideIR()/sideIR) > SIDEIRTHRESH && side_detected == false){
        side_detected = true;
        //the of glass location based on the tray position when the glass is detected
        glassTop = trayPosStp - SIDEIRPOS;
      }
      */

      //DETECT GLASS HEIGHT USING CV ***
      /* If the tray is high enough
       * Measure glass top with CV
       * Subtract the tray position
       * Tray edge will be in frameat trayPosStp < 235
       * (not used)Take the largest measurement - assume no false detections above glass rim)
       * Save the measurement into an array
       */

       if (trayPosStp > 260 && side_detected == false && linePosCV > 150 && glassN<L){
         /*log glass height measurement if conditions are met
          1. tray is low enough
          2. side hasn't been detected (not used)
          3. the linePosCV is below the top of the camera
         */
         
         glassTop = linePosCV; 
         //ADD FILTERING
         glassBot = trayPosStp;
         int gh = glassBot - glassTop + GLASSCVFUDGE; //15mm fudge to account for lens angle!
         glassArr[glassN] = gh; //populate array with measurement
         glassN++;
      }

      lastDirn = spb_move(MAX_STEPS);
      if (es.enUp == false) {break;} //tray is at the top
    }
  }
  
//  stepper.disable();

  //the bottom of the glass location based on
  //DETECT GLASS HEIGHT USING IR ***
  //glassBot = trayPosStp - STOPDISTANCE;


  //PROCESS GLASS HEIGHT FROM CV ***
  nh.loginfo("processing glassHeight");
  if (glassN > 5){ //if we have enough measurements
    nh.loginfo("processing av glassHeight");
    for (short i=0;i<glassN;i++){
      char out[4];
      itoa(glassArr[i],out,10);
      nh.loginfo(out);
      glassAv = glassAv + glassArr[i];
    }
    glassAv = glassAv / glassN;
    glassHeight = int(glassAv); //temp
  }
  else {glassHeight = 70;} // take the shortest case - don't overpour

  
  //if (glassHeight < 85){
  //  glassHeight = 95; //default GH
  //}
  
        
   nh.loginfo("glassHeight measured: ");
   char output[8];
   itoa(glassAv,output,10);
   nh.loginfo(output);
   itoa(glassN,output,10);
   nh.loginfo("Count: ");
   nh.loginfo(output);

  //2. Begin filling! --------------------------------
  nh.loginfo("Begin filling!");
  curRPM = 5; //lower RPM  FOR SOME REASON THIS VALUE CANNOT BE SET TO 4 WTF
  stepper.setRPM(curRPM);
  stepper.enable();
  spb_move(-100);
  digitalWrite(SOLENOID,true);  //Open the solenoid valve
  
//  while (trayPosStp - STOPDISTANCE < glassHeight - SURFOFFSET - (STOPDISTANCE - TUBEPOS)){
//  while (trayPosStp - STOPDISTANCE < glassHeight - SURFOFFSET){
while (trayPosStp - TUBEPOS < glassHeight - SURFOFFSET){
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

      //FOAM ALERT stop if there is too much foam
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

