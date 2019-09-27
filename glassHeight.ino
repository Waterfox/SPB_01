void zero_glass_arr(int b[], int arrSize) {
  for (short i = 0; i < arrSize; i++) {
    b[i] = 0;
  }
}

void get_ros_params()
{
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
}

void side_ghd () {
  //DETECT GLASS HEIGHT USING IR ***
  //      nh.loginfo("GHD");
  if (useGlassHeightDetection && side_detected == false)
  {
    int sir = measure_sideIR();
    //        int sir = 160;
    if (sir < SIDEIRTHRESH )
    { 
      //the of glass location based on the tray position when the glass is detected
      glassHeight = trayPosStp - SIDEIRPOS;
      if (glassHeight< MAXGLASSHEIGHT){//sometimes an initial measurement of 240 happens - hackyhack
        side_detected = true;
        if (ROS){
        nh.loginfo("GH measured: ");
        char output[8];
        itoa(glassHeight, output, 10);
        nh.loginfo(output);
        }
        if(!ROS){
          Serial.print("GH measured: ");Serial.println(glassHeight);
        }
      }
    }
  }

  else if (useCVGlassHeightDetection)
  {
    //DETECT GLASS HEIGHT USING CV ***
    //       Measure glass top with CV
    //       * Subtract the tray position
    //       * Tray edge will be in frameat trayPosStp < 235
    //       * (not used)Take the largest measurement - assume no false detections above glass rim)
    //       * Save the measurement into an array

    if (trayPosStp > 290  && linePosCV > 150 && glassN < L) {
      //          log glass height measurement if conditions are met
      //          1. tray is low enough
      //          2. the linePosCV is below the top of the camera
      //          3. logged less than L measurements

      glassTop = linePosCV;
      glassBot = trayPosStp;
      int gh = glassBot - glassTop + GLASSCVFUDGE; //15mm fudge to account for lens angle!
      //record measurements if N < 5 or if dX/X < 0.2
      if (glassN < 10) {
        glassAv = glassAv * (glassN - 1) / glassN + gh / glassN;
        glassArr[glassN] = gh; //populate array with measurement
        glassN++;
      }
      else if (abs(glassAv - gh) / glassAv < 0.2) {
        glassAv = glassAv * (glassN - 1) / glassN + gh / glassN;
        glassArr[glassN] = gh; //populate array with measurement
        glassN++;
      }
    }
  }
}

void process_glass_height() {

  if (useGlassHeightDetection)
  {
    if(ROS){
      nh.loginfo("glassHeight measured with ToF: ");
      char output[8];
      itoa(glassHeight, output, 10);
      nh.loginfo(output);
    }
    if(!ROS){
      Serial.print("\nglassHeight measured with ToF: ");Serial.println(glassHeight);
    }
  }


  if (useCVGlassHeightDetection)
  {
    glassAv = 0;
    //PROCESS GLASS HEIGHT FROM CV ***
    nh.loginfo("processing glassHeight");
    if (glassN > 5) { //if we have enough measurements
      nh.loginfo("processing av glassHeight");
      for (short i = 0; i < glassN; i++) {
        char out[4];
        itoa(glassArr[i], out, 10);
        nh.loginfo(out);
        glassAv = glassAv + glassArr[i];
      }
      glassAv = glassAv / glassN;
      //Use Average only - comment below

      //reject outliers
      for (short i = 0; i < glassN; i++) {
        glassDev[i] = abs(glassArr[i] - int(glassAv)); // deviation
        glassStdDev = glassStdDev + glassDev[i] * glassDev[i]; //add to top half of STD
      }
      glassStdDev = sqrt(glassStdDev / (glassN - 1)); // standard deviation
      glassAv = 0;
      int AvCnt = 0;
      for (short i = 0; i < glassN; i++) {
        if (glassDev[i] / glassStdDev < 1.0) { // if measurement is within 1 std, use in average
          glassAv = glassAv + glassArr[i];
          AvCnt++;
        }
      }
      glassAv = glassAv / AvCnt; // new average with only 1std measurementsc
      glassHeight = int(glassAv); //temp
    }
    else {
      glassHeight = 70; // if there are not enough measurements take the shortest case - don't overpour
    }

    nh.loginfo("glassHeight measured: ");
    char output[8];
    itoa(glassAv, output, 10);
    nh.loginfo(output);
    itoa(glassN, output, 10);
    nh.loginfo("Count: ");
    nh.loginfo(output);
  }

  //not using glass height detection
  if (!(useCVGlassHeightDetection || useGlassHeightDetection))
  {
    glassHeight = defaultGlassHeight;
  }
}
