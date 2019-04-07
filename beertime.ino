void zero_glass_arr(int b[], int arrSize){
  for (short i=0;i<arrSize;i++){
    b[i]=0;
  }
}

void beer_time(){
  side_detected = false; //not used
  zero_glass_arr(glassArr,L);
  zero_glass_arr(glassDev,L);
  glassAv=0;
  glassStdDev = 0;
  glassN = 0; //number of glass detections
  glassHeight = 0; //default low glass height val
  curRPM = 12; //lower RPM working: 12 max with no delay
  stepper.setRPM(curRPM);
  useGlassHeightDetection = false;
  defaultGlassHeight = GLASSHEIGHT_DEFAULT;
//Use parameter server to set glass height params
  if (nh.getParam("useGHD",&useGlassHeightDetection))
  {
    nh.loginfo("Retrieved param");
    if (useGlassHeightDetection) {nh.loginfo("using glass height detection");}
//    else {nh.loginfo("NOT using glass height detection");}
  }
  if (!useGlassHeightDetection)
  {
    nh.loginfo("No CV GH detection");
    //If there us a glass height parameter use it
    if (nh.getParam("GH",&defaultGlassHeight))
    {
      nh.loginfo("GH from params is:");
      char output2[4];
      itoa(defaultGlassHeight,output2,10);
      nh.loginfo(output2);
    }
    //Otherwise take the firmware default of 160
    else 
    {
      nh.loginfo("Default GH is:");
      char output2[4];
      itoa(defaultGlassHeight,output2,10);
      nh.loginfo(output2);
    }
  }
  
  // 0. The tray is already lowered

  // Turn on red button LED
  digitalWrite(STARTLED, HIGH);
  
  // 1. Raise the tray --------------------------------------------
  nh.loginfo("Raising the Tray");
  //baseline side IR measurement
  sideIR=measure_sideIR();

  for (short i;i<20;i++){ // fill the US average
  USnow = measure_US();
  }

  //Use the ultrasound to stop raising the tray 
//  while ((measure_US() > STOPDISTANCE) && (es.enUp == true)) 
//  while (trayPosStp - 15> STOPDISTANCE){  // USE Tray Position + offset
//  while(es.enUp)
  while ((USnow > STOPDISTANCE) && (es.enUp == true)) 
  {

    if ((!state)) {return;}  //E-STOP
    
    wait_time_micros = stepper.nextAction();
    long t4 = millis();
    if (wait_time_micros <= 0) {
      
       if (nh.connected()==false) {return;}

      
      if (t4 - loop_timer4 > 10){ // timer for ultrasound read
       USnow = measure_US();
       loop_timer4 = t4;
      }
      
       update_tray_pos();

       nh.spinOnce();
//      publish_sensors();

      
      //DETECT GLASS HEIGHT USING IR ***
      
//      if ((measure_sideIR()/sideIR) > SIDEIRTHRESH && side_detected == false){
//        side_detected = true;
//        //the of glass location based on the tray position when the glass is detected
//        glassTop = trayPosStp - SIDEIRPOS;
//      }
      
      if (useGlassHeightDetection)
      {
        //DETECT GLASS HEIGHT USING CV ***
  //       Measure glass top with CV
  //       * Subtract the tray position
  //       * Tray edge will be in frameat trayPosStp < 235
  //       * (not used)Take the largest measurement - assume no false detections above glass rim)
  //       * Save the measurement into an array
              
         if (trayPosStp > 290  && linePosCV > 150 && glassN<L){
  //          log glass height measurement if conditions are met
  //          1. tray is low enough
  //          2. the linePosCV is below the top of the camera
  //          3. logged less than L measurements
           
           glassTop = linePosCV; 
           glassBot = trayPosStp;
           int gh = glassBot - glassTop + GLASSCVFUDGE; //15mm fudge to account for lens angle!
           //record measurements if N < 5 or if dX/X < 0.2
           if (glassN<10) {
            glassAv = glassAv*(glassN-1)/glassN + gh / glassN;
            glassArr[glassN] = gh; //populate array with measurement
            glassN++;
           }
           else if (abs(glassAv -gh)/glassAv < 0.2) {
            glassAv = glassAv*(glassN-1)/glassN + gh / glassN;
            glassArr[glassN] = gh; //populate array with measurement
            glassN++;
           }         
        }
      }

      lastDirn = spb_move(MAX_STEPS);
    }
    else if (wait_time_micros > 300){
      long t3 = millis();
      if (t3 - loop_timer3 > 200){ //timer for SPB move
       update_tray_pos();
       nh.spinOnce();
   
       if (t4 - loop_timer4 > 10){ //timer for ultrasound read
        USnow = measure_US();
        loop_timer4 = t4;
      }
       lastDirn = spb_move(MAX_STEPS);
//       USnow = measure_US();
       loop_timer3 = t3;
      }
    }
  }
  


/*
  //----------Experimental Code for raising without pinging -----------
  long pub_timer3 = 0;
   
  volatile bool tryRPM = false;
  while (es.enUp) {

    if (!state) {
      break;
    }
    wait_time_micros = stepper.nextAction();
    
    if (wait_time_micros <= 0) {
      update_tray_pos();
      nh.spinOnce();
//      publish_sensors(); // currently overhead is too high, no point in this
      lastDirn = spb_move(MAX_STEPS);
    }

    
    else if (wait_time_micros > 200){ // EXPERIMENT put a timer in here to spin the node
      long t3 = millis();
      if (t3 - pub_timer3 > 500){ //only 
  //      if (tryRPM==false){
  //        tryRPM=true;
  //        stepper.stop();
  //        stepper.setRPM(curRPM);
  //      }
        update_tray_pos();
        nh.spinOnce();
  //      publish_sensors();  // currently overhead is too high, no point in this
        lastDirn = spb_move(MAX_STEPS);

        pub_timer3 = t3;
      }
    }
  }
*/


  stepper.stop();
  stepper.disable();
  delay(300);
  //the bottom of the glass location based on
  //DETECT GLASS HEIGHT USING IR ***
  //glassBot = trayPosStp - STOPDISTANCE;
  glassAv=0;


  if (useGlassHeightDetection)
  {
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
      //Use Average only - comment below
      
      //reject outliers
      for (short i=0;i<glassN;i++){
        glassDev[i] = abs(glassArr[i] - int(glassAv)); // deviation
        glassStdDev = glassStdDev + glassDev[i]*glassDev[i]; //add to top half of STD
      }
        glassStdDev = sqrt(glassStdDev / (glassN-1)); // standard deviation
        glassAv = 0;
        int AvCnt = 0;
      for (short i=0;i<glassN;i++){
        if (glassDev[i]/glassStdDev < 1.0){ // if measurement is within 1 std, use in average
          glassAv = glassAv + glassArr[i];
          AvCnt++;
        }
      }
      glassAv = glassAv/AvCnt; // new average with only 1std measurementsc
      glassHeight = int(glassAv); //temp
    }
    else {glassHeight = 70;} // if there are not enough measurements take the shortest case - don't overpour
    
    nh.loginfo("glassHeight measured: ");
    char output[8];
    itoa(glassAv,output,10);
    nh.loginfo(output);
    itoa(glassN,output,10);
    nh.loginfo("Count: ");
    nh.loginfo(output);    
  }

  //not using glass height detection
  else
  {
    glassHeight = defaultGlassHeight;
  }


  spb_move(-500); // start moving down?
  delay(500);
  stepper.stop();
  stepper.disable();

    
  //2. Begin filling! --------------------------------
  nh.loginfo("Begin filling!");
  curRPM = 6; //lower RPM  FOR SOME REASON THIS VALUE CANNOT BE SET TO 4 WTF
  stepper.setRPM(curRPM);
//  stepper.enable();
//  spb_move(-100);
  digitalWrite(SOLENOID,true);  //Open the solenoid valve
  delay(800);
  loop_timer3 = 0;
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
        
         nh.loginfo("ctrl");
         char output[8];
         itoa(steps,output,10);
         nh.loginfo(output);
         
        lastDirn = spb_move(steps);
      } 
    }
    else if (wait_time_micros > 300) {
      long t3 = millis();
      if (t3 - loop_timer3 > 200){ //timer for SPB move
        update_tray_pos();
        publish_sensors();
        nh.spinOnce();
        if ((!state)) {return;}  //E-STOP
        if (nh.connected()==false) {
          digitalWrite(SOLENOID,LOW);
          return;
        }
      //FOAM ALERT stop if there is too much foam
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
          lastDirn = spb_move(steps);
        } 
      loop_timer3 = t3; 
      }
    }
  }
  
  digitalWrite(SOLENOID,false);  //Close the solenoid valve

//  3. Lower the Tray -------------------------------------
  nh.loginfo("Lowering the tray");
  home_tray();
  digitalWrite(STARTLED, LOW);
  nh.loginfo("Process complete");
  state=1;
  
}
