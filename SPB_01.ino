#include <Arduino.h>
#include "SPB.h"
#include "endstops.h"
#include "BasicStepperDriver.h"
#include <Adafruit_NeoPixel.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include <stdlib.h>
#include <std_srvs/Empty.h>
#include <Wire.h>
#include <VL53L1X.h>
//#include <VL6180X.h>


/*
   rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=500000
   NOTE: direction changed for tmc 2208

   STATE:
   0: E-Stopped
   1: Ready
   2: Pouring
   3: ROS not connected
*/

using std_srvs::Empty;


DRV8825 stepper(MOTOR_STEPS, Z_DIR_PIN, Z_STEP_PIN, Z_ENABLE_PIN, MODE0, MODE1, MODE2);


Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

VL53L1X top_sensor;
VL53L1X side_sensor;
bool tVLinitfail = false;
bool sVLinitfail = false;

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
int linePosCV = 0;  //line measurement from CV output
int surfPos = 0;  //surface position used in control calculation

int lastDirn = 0;
int steps = 0;
int state = 0; //0: Estopped, 1:Waiting, 2:Pouring

int useGlassHeightDetection = true;
int useCVGlassHeightDetection = false;
int defaultGlassHeight = GLASSHEIGHT_DEFAULT;
int defaultGlassBottom = GLASSBOTTOM_DETAULT; //used in the future to elminate ultrasound
bool side_detected = false;
int glassTop = 0;
int glassBot = 0;
int glassHeight = GLASSHEIGHT_DEFAULT;  //top of glass to the bottom (not stem height)
int glassN = 0; //counts how many times the edge us detected
const int L = 50; // max number of gh measurements
int glassArr[L]; //array of gh measurements
int glassDev[L]; //deviation from mean
float glassAv = 0; //average gh
float glassStdDev = 0; //mean deviation
float usVal = 0;
float topIRVal = 0;
volatile int curLightVal = 175;
int curRPM = RPM;
float USnow; // last ultrasound value
int tir;  // top ToF last value
int sir; // side ToF last value

unsigned wait_time_micros;

long pub_timer1 = 0;
long pub_timer2 = 0;
long sub_timer3 = 0;
long sub_timer4 = 0;
long LED_timer = 0;
bool LED_state = 0;
long RPM_timer = 0;
long loop_timer3 = 0;
long loop_timer4 = 0;
bool CV_EN = false;
bool CV_LINES_EN = false;



  ros::NodeHandle  nh;


  void cv_cb(const std_msgs::UInt16 & lvl_msg) {
    surfPosCV = lvl_msg.data;
    long t3 = millis();
    if (t3 - sub_timer3 < 1000) { //if we've received a topic in the last 1s, unlock
      CV_EN = true;
    }
    else {
      CV_EN = false;
    }
    sub_timer3 = t3;
  }

  void cv_lines_cb(const std_msgs::UInt16 & lvl_line_msg) {
    linePosCV = lvl_line_msg.data;
    long t4 = millis();
    if (t4 - sub_timer4 < 300) { //if we've received a topic in the last 300ms, unlock
      CV_LINES_EN = true;
    }
    else {
      CV_LINES_EN = false;
    }
    sub_timer4 = t4;
  }

  void cmd_cb(const std_msgs::UInt16 & cmd_msg) {
    //controls the tray from a rostopic
    cmdPosStp = cmd_msg.data;
    // if the cmd value is not zero and in range
    if (cmdPosStp != 0 && cmdPosStp > Z_MAX_POS && cmdPosStp < Z_MIN_POS ) {
      while (abs(cmdPosStp - trayPosStp) > 1) {
        unsigned wait_time_micros = stepper.nextAction();
        if (wait_time_micros <= 0) {
          update_tray_pos();
          steps = ((cmdPosStp - trayPosStp) * -STEPSPERMM);
          lastDirn = spb_move(steps);
        }
        else {
          delay(1);
        }
      }
    }
    stepper.disable();
  }

  void cmd_valve_cb(const std_msgs::Bool & cmd_valve_msg) {
    //open the valve if publishing true
    if (cmd_valve_msg.data) {
      digitalWrite(SOLENOID, true);
    }
    else {
      digitalWrite(SOLENOID, false);
    }
  }

  void cmd_led_cb(const std_msgs::UInt16 & cmd_led_msg) {
    curLightVal = cmd_led_msg.data;
    set_lights(curLightVal);
  }

  void beer_callback(const Empty::Request & req, Empty::Response & res)
  {
    // Simulate function running for a non-deterministic amount of time
    beer_time();
  }


  ros::Subscriber<std_msgs::UInt16> sub_cv("spb/lvl", cv_cb);
  ros::Subscriber<std_msgs::UInt16> sub_cv_lines("spb/level_lines", cv_lines_cb);
  ros::Subscriber<std_msgs::UInt16> sub_cmd("spb/cmd_pos", cmd_cb);
  ros::Subscriber<std_msgs::Bool> sub_valve("spb/cmd_valve", cmd_valve_cb);
  ros::Subscriber<std_msgs::UInt16> sub_led("spb/cmd_led", cmd_led_cb);
  std_msgs::UInt16 us_msg;
  std_msgs::UInt16 ir_msg;
  std_msgs::UInt16 sir_msg;
  std_msgs::UInt16 gh_msg;
  std_msgs::UInt16 tp_msg;
  std_msgs::Bool esUp_msg;
  std_msgs::Bool esDown_msg;
  ros::Publisher pubUS("spb/us", &us_msg);
  ros::Publisher pubIR("spb/ir", &ir_msg);
  ros::Publisher pubSIR("spb/sir", &sir_msg);
  //ros::Publisher pubGH("spb/glass_height", &gh_msg);
  ros::Publisher pubTP("spb/tray_pos", &tp_msg);
  //ros::Publisher pubEU("spb/esUp", &esUp_msg);
  //ros::Publisher pubED("spb/esDown", &esDown_msg);
  ros::ServiceServer<Empty::Request, Empty::Response> beertime_server("beertime", &beer_callback);


  void publish_sensors(void) {
    long t1 = millis();
    if (t1 - pub_timer1 > TPUB1) {
      us_msg.data = (int)USnow;
      ir_msg.data = (int)tir;
      sir_msg.data = (int)sir;
      //    gh_msg.data = (int)glassHeight;
      pubUS.publish(&us_msg);
      pubIR.publish(&ir_msg);
      pubSIR.publish(&sir_msg);
      //    pubGH.publish(&gh_msg);
      //    esUp_msg.data = es.enUp;
      //    esDown_msg.data = es.enDown;
      //    pubEU.publish(&esUp_msg);
      //    pubED.publish(&esDown_msg);
      pub_timer1 = t1;
    }
  }

  void publish_tray(void) {
    long t2 = millis();
    if (t2 - pub_timer2  > TPUB2) {
      tp_msg.data = (int)trayPosStp;
      pubTP.publish(&tp_msg);
      pub_timer2 = t2;
    }
  }

//**********************************************
void setup() {
  state = 1; // Oh shazbot what was this? Estop condition?

  //configure the VLX ToF sensors
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  //change the top ToF i2c address
  pinMode(topCE, INPUT); // will float high
  pinMode(sideCE, OUTPUT);
  digitalWrite(sideCE, LOW);
  delay(50);

  top_sensor.setTimeout(500);

  if (!top_sensor.init())
  {
    tVLinitfail = true;
  }
  top_sensor.setDistanceMode(VL53L1X::Short);
  top_sensor.setMeasurementTimingBudget(20000);
  top_sensor.startContinuous(20);
  top_sensor.setAddress(0x31);
  delay(50);

  //  delay(2000);
  //  Serial.println(top_sensor.getAddress());
  pinMode(sideCE, INPUT); // will float high, turn side back on
  delay(50);
  if (!side_sensor.init())
  {
    sVLinitfail = true;
  }
  side_sensor.setDistanceMode(VL53L1X::Short);
  side_sensor.setMeasurementTimingBudget(20000);
  side_sensor.startContinuous(20);



  pinMode(SOLENOID, OUTPUT);
  pinMode(US_PWR, OUTPUT);

  //Turn off the Valve
  digitalWrite(SOLENOID, LOW);

  //Init the stepper
  stepper.begin(curRPM, MICROSTEPS);
  stepper.enable();

  // Init the endstops and buttons
  es.check_endstops();
  buttons_init();

  //turn on the ultrasound
  digitalWrite(US_PWR, true);

  //Turn the Lights On
  pixels.begin();
  set_lights(curLightVal);

  if (ROS)
  {
    //Init ROS
    nh.getHardware()->setBaud(BAUDRATE);
    nh.initNode();
    nh.subscribe(sub_cv);
    nh.subscribe(sub_cv_lines);
    nh.subscribe(sub_cmd);
    nh.subscribe(sub_valve);
    nh.subscribe(sub_led);
    nh.advertise(pubUS);
    nh.advertise(pubIR);
    nh.advertise(pubSIR);
    //  nh.advertise(pubGH);
    nh.advertise(pubTP);
    //  nh.advertise(pubEU);
    //  nh.advertise(pubED);
    nh.advertiseService(beertime_server);
    check_estop();
  
    // Home the Tray
    if (tVLinitfail) { nh.loginfo("topVL sensor init fail");}
    if (sVLinitfail) { nh.loginfo("sideVL sensor init fail");}
    nh.loginfo("Home the tray");
  }
  else
  {
    Serial.begin(115200);
    if (tVLinitfail){
      Serial.println("Top VL sensor fail");
    }
    if (sVLinitfail){
      Serial.println("side VL sensor fail");
    }
    Serial.println("Home the Tray");
  }


  home_tray();
  

}

//----------------------------------------------
void loop() {

  check_estop();
  if (state > 0) {
    check_start();
    if (ROS) {publish_sensors();}
  }
  nh.spinOnce();
  update_tray_pos();
  if (ROS){check_nh();}
  estop_LED();

  //  wait_time_micros = stepper.nextAction();
  //  if (wait_time_micros <= 0) {
  //    stepper.disable();
  //  }
  // else {delay(1);}
  //  if (VLinitfail) { nh.loginfo("VL sensor init fail"); delay(2000);}
  delay(1);

}
//------------------------------------------------




//convert IR measurement to distance in mm (distance from sensor)
float topIR2dist(float topVal) {
  //  return(26734.0*pow(topVal,-0.883));
  return 0.0038 * topVal * topVal - 2.544 * topVal + 588.85;
}

//conver ultrasound measurement to distance in mm (distance from sensor)
float US2dist(int usVal) {
  return (0.1466275 * usVal + 100.0); //150.0mm/1023.0 * usVal + 100mm
}


//Check endstop conditions and move the stepper motor
//return direction of steps moved
int spb_move(int move_steps) {
  if (abs(move_steps) > DEADBAND) {
    if (move_steps > MAX_STEPS && es.enUp == true) {
      stepper.enable();
      stepper.startMove(-MAX_STEPS);
      return 1;
    }
    else if (move_steps < -MAX_STEPS && es.enDown == true) {
      stepper.enable();
      stepper.startMove(+MAX_STEPS);
      return -1;
    }
    else if ((move_steps > 0 && es.enUp == true) || (move_steps < 0 && es.enDown == true)) {
      stepper.enable();
      stepper.startMove(-move_steps);
      return ((move_steps > 0) - (move_steps < 0));
    }
    else {
      return 0;
    }
  }
  return 0;
}

void update_tray_pos(void) {
  trayPosStp = trayPosStp - ((stepper.step_count * lastDirn) / 100.0); // distance travelled in mm
  stepper.step_count = 0;
  if (ROS){publish_tray();}
}

int measure_topIR() {
  //  topIRVal = analogRead(TOP_IR_PIN) * 0.05 + topIRAvg * 0.95;
  //  topIRAvg = topIRVal;
  //  return topIR2dist(topIRAvg);
  //    return topIR2dist(float(analogRead(TOP_IR_PIN))); ORIGINAL

  //VL53L1X
  top_sensor.read();
  tir = top_sensor.ranging_data.range_mm;
  return tir;
}

int measure_sideIR() {
  //  return analogRead(SIDE_IR_PIN); ORGINAL

  //VL6180X
  //    return side_sensor.readRangeSingleMillimeters();
  //    return side_sensor.readRangeContinuousMillimeters();
  //VL53L1X
  side_sensor.read();
  sir = side_sensor.ranging_data.range_mm;
  return sir;
}

float measure_US() {
  //  usVal = analogRead(US_PIN) * 0.05 + USAvg * 0.95;
  //  USAvg = usVal;
  //  return US2dist(USAvg);
  USnow = US2dist(analogRead(US_PIN)); //skip the average - this only works continuously
  return USnow;
}


void home_tray()
{
  volatile bool tryRPM = false;
  curRPM = 12; //raise RPM 12 working
  stepper.setRPM(curRPM);

  while (es.enDown) {

    if (!state) {
      break;
    }
    wait_time_micros = stepper.nextAction();
    if (wait_time_micros <= 0) {
      update_tray_pos();
      if (ROS){nh.spinOnce();}
      lastDirn = spb_move(-MAX_STEPS);
    }

    else if (wait_time_micros > 100) { // EXPERIMENT put a timer in here to spin the node
      if (tryRPM == false) {
        tryRPM = true;
        stepper.stop();
        stepper.setRPM(curRPM);
      }
      update_tray_pos();
      if (ROS){nh.spinOnce();}
      lastDirn = spb_move(-MAX_STEPS);
      //      nh.loginfo(stepper.getCurrentRPM());s
    }


  }
  //  nh.loginfo("home_tray complete");
  stepper.disable();
}


void set_lights(int lightVal) {
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, lightVal, lightVal, lightVal);
  }
  pixels.show();
}

void check_nh() {
  if ((!(nh.connected())) || (!state)) {
    state = 3;
  }
  else if (!state) {
    state = 1;
  }
}

void estop_LED() {
  //ESTOP CONDITION
  if (!state) {
    //flash LED
    long t1 = millis();
    if (t1 - LED_timer > 1250) {
      if (!LED_state) {
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
  //NH not connectected
  else if (state == 3) {
    //pulse LED
    long t1 = millis();
    if ((t1 - LED_timer > 1000) && (LED_state == false)) {
      digitalWrite(STARTLED, HIGH);
      LED_state = true;
      LED_timer = t1;
    }
    else if ((t1 - LED_timer > 500) && (LED_state == true)) {
      digitalWrite(STARTLED, LOW);
      LED_state = false;
      LED_timer = t1;
    }

  }
}
