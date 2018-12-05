#include <Arduino.h>
#include "SPB.h"
#include "endstops.h"
#include "BasicStepperDriver.h"
#include <Adafruit_NeoPixel.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>

/*
 * rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=500000
 * NOTE: direction changed for tmc 2208
 * 
 * STATE:
 * 0: E-Stopped
 * 1: Ready
 * 2: Pouring
 */




DRV8825 stepper(MOTOR_STEPS, Z_DIR_PIN, Z_STEP_PIN, Z_ENABLE_PIN, MODE0, MODE1, MODE2);


Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);


float topIRAvg = 0;
float USAvg = 0;
float sideIR = 0;
endstops es;
char inByte = '0';
float trayPosStp = 0; //tray position from stepper count - measured from the top down
float cmdPosStp = 0;
//float surfPosIR = 0; //surface position from IR measurement - measured from the sensor down
//float surfPosUS = 0; //surface position from Ultrasound measurement - measured from the sensor down
int surfPosCV = 0;  //surface position from computer vision output
int surfPos = 0;  //surface position used in control calculation

int lastDirn = 0;
int MODE = 0; // 0: tray level, 1: manual override -- NOT USED
int state = 0; //0: Estopped, 1:Waiting, 2:Pouring -- NOT USED

bool side_detected = false;
short glassTop = 0; 
short glassBot = 0;
short glassHeight = 165;
int curLightVal = 100;
int curRPM = RPM;

long pub_timer1 = 0;
long pub_timer2 = 0;
long LED_timer = 0;
bool LED_state = 0;
long RPM_timer = 0;

ros::NodeHandle  nh;


void cv_cb(const std_msgs::UInt16& lvl_msg){
  surfPosCV = lvl_msg.data;
}

void cmd_cb(const std_msgs::UInt16& cmd_msg){
  //controls the tray from a rostopic
  cmdPosStp = cmd_msg.data;
  // if the cmd value is not zero and in range
  if (cmdPosStp != 0 && cmdPosStp > Z_MAX_POS && cmdPosStp < Z_MIN_POS ) {
    while (abs(cmdPosStp - trayPosStp) > 1){
      unsigned wait_time_micros = stepper.nextAction();
      if (wait_time_micros <= 0) {
        update_tray_pos();
        int steps = ((cmdPosStp - trayPosStp)*-STEPSPERMM);
        lastDirn = spb_move(steps);
      }
      else {delay(1);}
    }
  }
  stepper.disable();
}

void cmd_valve_cb(const std_msgs::Bool& cmd_valve_msg){
  //open the valve if publishing true
  if (cmd_valve_msg.data){digitalWrite(SOLENOID,true);}
  else {digitalWrite(SOLENOID,false);}
}

void cmd_led_cb(const std_msgs::UInt16& cmd_led_msg){
  curLightVal = cmd_led_msg.data;
  set_lights(curLightVal);
}


ros::Subscriber<std_msgs::UInt16> sub_cv("spb/lvl", cv_cb);
ros::Subscriber<std_msgs::UInt16> sub_cmd("spb/cmd_pos", cmd_cb);
ros::Subscriber<std_msgs::Bool> sub_valve("spb/cmd_valve", cmd_valve_cb); 
ros::Subscriber<std_msgs::UInt16> sub_led("spb/cmd_led", cmd_led_cb);
std_msgs::UInt16 us_msg;
std_msgs::UInt16 ir_msg;
std_msgs::UInt16 gh_msg;
std_msgs::UInt16 tp_msg;
ros::Publisher pubUS("spb/us", &us_msg);
ros::Publisher pubIR("spb/ir", &ir_msg);
ros::Publisher pubGH("spb/glass_height", &gh_msg);
ros::Publisher pubTP("spb/tray_pos", &tp_msg);


void publish_sensors(void) {
  long t1 = millis();
  if (t1 > pub_timer1 + TPUB1){
    us_msg.data = int(measure_US());
    ir_msg.data = int(measure_topIR());
    gh_msg.data = int(glassHeight);
    pubUS.publish(&us_msg);
    pubIR.publish(&ir_msg);
    pubGH.publish(&gh_msg);
    pub_timer1 = t1;
  }
}

void publish_tray(void) {
  long t2 = millis();
  if (t2 > pub_timer2 + TPUB2){
    tp_msg.data = int(trayPosStp);
    pubTP.publish(&tp_msg);
    pub_timer2 = t2;
  }
}
//**********************************************
void setup() {
  pinMode(SOLENOID,OUTPUT);
  pinMode(US_PWR,OUTPUT);

//Turn off the Valve
  digitalWrite(SOLENOID,LOW);

//Init the stepper
  stepper.begin(curRPM, MICROSTEPS);
  stepper.enable();

// Init the endstops and buttons
  es.check_endstops();
  buttons_init();

  //turn on the ultrasound  
  digitalWrite(US_PWR,true);  

  //Turn the Lights On
  pixels.begin();
  set_lights(curLightVal);


//Init ROS
  nh.getHardware()->setBaud(BAUDRATE);
  nh.initNode();
  nh.subscribe(sub_cv);
  nh.subscribe(sub_cmd);
  nh.subscribe(sub_valve);
  nh.subscribe(sub_led);
  nh.advertise(pubUS);
  nh.advertise(pubIR);
  nh.advertise(pubGH);
  nh.advertise(pubTP);


 // Home the Tray
  nh.loginfo("Home the tray");
  home_tray();


}

//----------------------------------------------
void loop() {

  check_estop();
  if (state) {check_start();}
  publish_sensors();
  nh.spinOnce();
  
  unsigned wait_time_micros = stepper.nextAction();
  if (wait_time_micros <= 0) {
    stepper.disable();
    update_tray_pos();
      
  }

  //ESTOP CONDITION
  if (!state){
    //flash LED
    long t1 = millis();
    if (t1 - LED_timer > 1000){
      if (LED_state == false) {
        digitalWrite(STARTLED, HIGH);
        LED_state = true;
      }
      else {
        digitalWrite(STARTLED, LOW);
        LED_state = false;
      }
      LED_timer = t1;
    }
  }
  
  else delay(1);

}
//------------------------------------------------




//convert IR measurement to distance in mm (distance from sensor)
float topIR2dist(float topVal){
//  return(26734.0*pow(topVal,-0.883));
    return 0.0038*topVal*topVal - 2.544*topVal+588.85;
}

//conver ultrasound measurement to distance in mm (distance from sensor)
float US2dist(int usVal){
  return(0.1466275*usVal + 100.0);   //150.0mm/1023.0 * usVal + 100mm
}


//Check endstop conditions and move the stepper motor
//return direction of steps moved
int spb_move(int steps){ 
  if (abs(steps) > DEADBAND){
    if (steps > MAX_STEPS && es.enUp == true){
        stepper.enable();
        stepper.startMove(-MAX_STEPS);
        return 1;
    }
    else if(steps < -MAX_STEPS && es.enDown == true){
        stepper.enable();
        stepper.startMove(+MAX_STEPS);
        return -1;
    }
    else if ((steps>0 && es.enUp ==true) || (steps<0 && es.enDown == true)){
      stepper.enable();
      stepper.startMove(-steps);
      return ((steps > 0) - (steps < 0)); 
    }
    else return 0;  
  }
  return 0;
}

void update_tray_pos(void){
    trayPosStp = trayPosStp - (stepper.step_count*lastDirn / 100.0); // distance travelled in mm
    stepper.step_count = 0;
    publish_tray();
}

float measure_topIR() {
  float topIRVal = analogRead(TOP_IR_PIN)*0.05+topIRAvg*0.95;
  topIRAvg = topIRVal;
  return topIR2dist(topIRAvg);
//  return topIRAvg;
}

int measure_sideIR() {
  return analogRead(SIDE_IR_PIN);
}

float measure_US() {
  float usVal = analogRead(US_PIN)*0.05 + USAvg*0.95;
  USAvg = usVal;
  return US2dist(USAvg);
}


void home_tray(){

  while (es.enDown){
    if (!state) return;
    update_tray_pos();
    unsigned wait_time_micros_1 = stepper.nextAction();
    if (wait_time_micros_1 <= 0) {
        spb_move(-MAX_STEPS);
    }
    else {
      delay(1);
    }
    nh.spinOnce();
  }
  stepper.disable();
//  Serial.println("Tray Initialized"); 
}


void set_lights(int lightVal) {
   for(int i=0;i<NUMPIXELS;i++){ 
    pixels.setPixelColor(i,lightVal,lightVal,lightVal); 
  }
  pixels.show(); 
}
