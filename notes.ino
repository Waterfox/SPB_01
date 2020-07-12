/*
 * Sept 9:  ToDo: Switch side sensor to VL53L1X - code for 6180 is wrecking side detection
 * ToDo: Move GHD processing to another function, another page
 * 
 * Sept 10: Add more comments, move paramters code in beertime to glassHeight
 * ToDo: implement better velocity control vs. on/off
 * Non-blocking stepper code.
 * 
 * 
 * Sept 26
 * Add ultrasound to tray height
 *  -compare US to ToF
 *    -No Foam - IR reads ~20mm high / US reads correct
 *    -Foam - IR reads correct / US reads high or noisy
 *    -soln: take min 
 * See if startMove can be called in nonblocked section
 * 
 * July 09 2020
 * Completed Integrating TIC T500 stepper controller
 * Removed ROS
 */

 
