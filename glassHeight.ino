
void side_ghd () {
  //DETECT GLASS HEIGHT USING IR ***
  //      nh.loginfo("GHD");
  if (useGlassHeightDetection && side_detected == false)
  {
    measure_sideIR();
    
    //        int sir = 160;
    if (sir < SIDEIRTHRESH )
    { 
      //the of glass location based on the tray position when the glass is detected
      glassHeight = trayPosStp - SIDEIRPOS;
      if (glassHeight < MAXGLASSHEIGHT){//sometimes an initial measurement of 240 happens - hackyhack
        side_detected = true;
        Serial.print("GH measured: ");Serial.println(glassHeight);
      }
      else
        {glassHeight = 0;}
    }
  }
}

void process_glass_height() {

  if (useGlassHeightDetection)
  {
    Serial.print("\nglassHeight measured with ToF: ");Serial.println(glassHeight);
  }

  //not using glass height detection
  else if (!useGlassHeightDetection)
  {
    glassHeight = defaultGlassHeight;
  }
}
