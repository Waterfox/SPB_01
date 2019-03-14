/*
 * rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=500000
 * 
 * rostopic pub /spb/cmd_pos std_msgs/UInt16 "data: 188"
 * rostopic pub /spb/cmd_pos std_msgs/UInt16 "data: 364"
 * 
 * rostopic pub /spb/cmd_led std_msgs/UInt16 "data: 254"
 */


/*
    increase speed if level is high
      long t2 = millis();   
      if ((steps > 1000) && (t2 - RPM_timer > 500) && (curRPM <=12))  {
        curRPM = curRPM +1;
        nh.loginfo(curRPM);
        RPM_timer = t2;
      }
    decrease speed if level is low
      if ((steps < 1000) && (t2 - RPM_timer > 500) && (curRPM >=2)) {
        curRPM = curRPM -1;
        nh.loginfo(curRPM);
        RPM_timer = t2;
      }
*/
/*
      //Stop if there is too much foam!

//      if (ir_msg.data < trayPosStp - glassHeight + 10){ 
//        //--If foam is within 10mm from top of glass, break 
//        nh.loginfo("Foam Alert!");
//        break;
//      }
*/
