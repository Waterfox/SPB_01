bool trayUpLoop()
{
  long t4 = millis();
  if (t4 - loop_timer4 > 200) { // timer for ultrasound read
    measure_US();
    measure_topIR();
    side_ghd (); 
    if (update_tray_pos()){return 1;}
    loop_timer4 = t4;
    Serial.print("TIR: ");Serial.print(tir);Serial.print("   TRAY: ");Serial.println(trayPosStp);
  }
  return 0;
}

bool trayDownLoop()
{
  update_tray_pos();
  measure_topIR();
  measure_US();
  
  //FOAM ALERT stop if there is too much foam
  if ((tir < trayPosStp - glassHeight + SURFOFFSET) || (USnow < trayPosStp - glassHeight + SURFOFFSET))
  {
    //--If foam is within 10mm from top of glass, break
    Serial.println("Foam Overflow");
    return 0;
  }

  //    Use the CV reading
//      surfPos = surfPosCV;

  //    Use top UR reading
  surfPos = min(tir,USnow);  // **FILTER USNOW?**  **FILTER SURFPOS**

  //    CONTROL LOOP
  //**UPDATE FOR PROPER SCALING
  delta = 0;
  long v_out = 0;
  /* *WORKING
  delta = -(SETPOINT + TUBEPOS - surfPos);  // error in mm
  /* *WORKING
  // Adjust the Speed Proportional to delta position
  if (delta < 0)  {
    float P = 1000000.0;
    v_out = long(P*delta);
    long v_max = 60000000;
    if (v_out>v_max){v_out=v_max;}
    else if (-v_out < 5000000){v_out=0;}
    else if (v_out<-v_max){v_out=-v_max;}
    spb_v(v_out); 
  }
  else spb_v(0);
  Serial.print("delta: "); Serial.print(delta);Serial.print("  v: "); Serial.print(v_out);Serial.print("  TIR: ");Serial.print(tir);Serial.print(" US ");Serial.print(USnow);Serial.print(" TRAY ");Serial.println(trayPosStp); // debug
  return 1;
  */
  //Quadratic fit
  delta = surfPos - TUBEPOS; //
  long v_max = v_travel;
  if (delta > 40){v_out=0;}
  else if (delta <= 0) {v_out=-v_travel;}
  else {v_out = -80000000 + 3100000*delta - 30000*delta*delta;} // quadtratic function from 0..50mm, trimmed at 40. 
  spb_v(v_out);
  Serial.print("delta: "); Serial.print(delta);Serial.print("  v: "); Serial.print(v_out);Serial.print("  TIR: ");Serial.print(tir);Serial.print(" US ");Serial.print(USnow);Serial.print(" TRAY ");Serial.println(trayPosStp); // debug
  return 1;
}

void beer_time() {
  side_detected = false; //not used
  glassHeight = 0; //default low glass height val


  useGlassHeightDetection = true;
  defaultGlassHeight = GLASSHEIGHT_DEFAULT;
  // 0. The tray is already lowered

  // Turn on red button LED
  digitalWrite(STARTLED, HIGH);

  // 1. Raise the tray --------------------------------------------
  Serial.println("Raising the Tray");

  Serial.println(measure_US());
  Serial.println(measure_topIR());
  Serial.println(state);

  spb_v(v_travel);
  //Use the ultrasound and top infraredto stop raising the tray
  while ((USnow > STOPDISTANCE && tir > STOPDISTANCE ) && (es.enUp == true)) /// UNTESTED
  {
    if ((!state)) {
      return; //E-STOP
    }
      spb_v(v_travel);
      if (trayUpLoop()){return;}
      if (trayPosStp < Z_MAX_POS+5){break;} //don't hit top endstop
      delay(1);
      

  }
  spb_v(0);
  delay(500);
  tic.exitSafeStart(); // just in case something happens
  delay(200);

  process_glass_height();
  if (glassHeight < MINGLASSHEIGHT){
    Serial.print("GH below min height of ");Serial.println(MINGLASSHEIGHT);
    return;}



  //2. Begin filling! --------------------------------
  Serial.println("Begin filling");

  digitalWrite(SOLENOID, true); //Open the solenoid valve
  delay(800);
  loop_timer3 = 0;

  while (trayPosStp - TUBEPOS < glassHeight - SURFOFFSET && es.enDown) {

    if ((!state)) {
      return; //E-STOP
    }
    if(!trayDownLoop()){break;}
    delay(1);

  }

  digitalWrite(SOLENOID, false); //Close the solenoid valve
  spb_v(0);

  //  3. Lower the Tray -------------------------------------
  Serial.println("\nLowering the tray");
  home_tray();
  digitalWrite(STARTLED, LOW);
  Serial.println("Process complete");
  state = 1;

}
