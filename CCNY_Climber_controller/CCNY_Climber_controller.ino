/* CCNY-City-Climber Robot Controller code (CCNY Robotics Lab)
 * Author:   Bing Li, Jonathan Liu, Zihui Cui
 * Data:     Aug. 2014
 * Platform: Arduino Yun Board (as Arduino Leonardo + Embedded Linux)
 * Password: arduino548 for uploading/port/web (http://arduino.local) of Arduino Yun Board
 * Romoter:  use Android Tablet to communicate with Arduino Yun via WIFI
 * Hardware:
 *    Left and right two wheels (PWM 50HZ)
 *    Two vacuum motorLs (I2C)
 *    Pressure sensor (Analog pid_input)
 *    Serial Bridge between Leonardo and Linux onboard
 * Attention:
 * you have own module, focus on your own setup_xxx(), test_xxx and other related functions.
 * The Arduino Yun, we use wifi for IDE, take longer time than serial port
 * if can not see Arduino in port, reset 32u4, wait for around 20s, if still doesn't work, power off & on
 * everytime you uploading, may need to wait for 10s then you open the port monitor
 * for print debugging:
 *    use my_print instead of Serial.print, including my_print(3.14) float variable
 *    use my_println instead of Serial.println
 *    Also: my_println_val to print like my_println_val("i=%d", i), ok for c%, d%, not for %.f
 */

#define ARDUINO_YUN
// #define TEST

#define ENABLE_WHEEL_ODOMETRY
#define TEST_WHEEL_ENCODER

#define ENABLE_SERVO_CTL
#define TEST_SERVO_CTL

#define ENABLE_VACUUM_CTL
#define TEST_VACUUM_CTL

// #define ENABLE_TIMERPWM_CTL
// #define TEST_TIMERPWM_CTL

#define ENABLE_DEBUG_RECORD

#ifdef ADAFRUIT_MOTOR_SHIELD
#include <Adafruit_motorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#endif

#ifdef ARDUINO_YUN
#include <Bridge.h>
#include <Console.h>
#include <YunServer.h>
#include <YunClient.h>
#endif

#ifdef ENABLE_TIMERPWM_CTL
#include <TimerOne.h>
#include <TimerThree.h>
#endif

#ifdef ENABLE_SERVO_CTL
#include <Servo.h>
#endif

#ifdef ENABLE_VACUUM_CTL
#include <Wire.h>
#include <PID_v1.h>
#endif

// define a selective print
char serialPrintString[32] = {0};
#ifdef  ARDUINO_YUN
  #define my_print(val) \
    if (Console) Console.print(val)
  #define my_println(string) \
    if (Console) Console.println(string)
#else
  #define my_print(val) \
    Serial.print(val)
  #define my_println(string) \
    Serial.println(string)
#endif

#define my_println_val(string, value) \
  { sprintf(serialPrintString, string, value); \
    my_println(serialPrintString); } \
  delay(1)

// global variables for testing & debugging
#define LEFT 		  1
#define RIGT 		  2
int     loop_cnt          = 0;
int     ledPin            = 13;
int     speed_stage       = 0;
int     speed_turn        = 0;

#ifdef ENABLE_DEBUG_RECORD
double  record_val[4]     = {0};
#endif

#ifdef ENABLE_WHEEL_ODOMETRY
// Odometry for wheels
#define odo_LEncA      2
#define odo_LEncB      7
#define odo_REncA      3
#define odo_REncB      8
#define pi             3.14159265359
#define twopi          pi*2

/* Not needed for interrupts
int odo_LEncAState     = digitalRead(odo_LEncA);
int odo_preLEncAState  = odo_LEncAState;
int odo_REncAState     = digitalRead(odo_REncA);
int odo_preREncAState  = odo_REncAState; */

volatile int  odo_Lcount = 0; // counter for left wheel
volatile int  odo_Rcount = 0;
#ifdef TEST_WHEEL_ENCODER
volatile bool odo_bool_dCheckL = 0;
#endif

float 	odo_Dl  = 0; // total displacement for left wheel
float 	odo_Dr  = 0;
float 	odo_D   = 0;
float 	odo_Vl  = 0; // current velocity for left wheel
float 	odo_Vr  = 0;
float 	odo_V   = 0;
float 	odo_x   = 0;
float 	odo_w   = 0;
float 	odo_y   = 0;
double 	odo_phi = 0;

long  	odo_Lcurrent_time = 0;
long  	odo_Lpre_time     = 0;

long  	odo_Rcurrent_time = 0;
long  	odo_Rpre_time     = 0;

float 	odo_Lperiod       = 0;
float   odo_Rperiod       = 0;
int     odo_dCheckL_cnt   = 0;
int     odo_dCheckR_cnt   = 0;
#endif

#ifdef ENABLE_TIMERPWM_CTL
// PWM control wheel motorLs
// percent: 0~100 since range is 0~1024
#define PWM_DUTY(percent)       ((int)(percent * 10.24))
#define PWM_FREQUENCY           50    // HZ
#define PWM_PERIOD              (int)(1000000/PWM_FREQUENCY)
#define PWM_TEST_DELAY          1200  // ms
#define LEFT_WHEEL_CTL_PIN      9 // pwm pin for left wheel
#define RIGT_WHEEL_CTL_PIN      5 // pwm pin for right wheel
#define PWM_DUTY_MIN            3
#define PWM_DUTY_MID            7 // 7 seems to be neutral, 6 is extremely slow
#define PWM_DUTY_MAX            10
#endif

#ifdef  ENABLE_SERVO_CTL
#define LEFT_WHEEL_CTL_PIN      9
#define RIGT_WHEEL_CTL_PIN      5
#define PWM_DUTY_MAX     	2400
#define PWM_DUTY_MIN     	500
#define PWM_DUTY_MID     	1360 // ((PWM_DUTY_MAX + PWM_DUTY_MIN) / 2) == 1440
#define SERVO_NEG        	1240 // servo control negative-start duty
#define SERVO_POS        	(1480 - 3) // servo control positive-start duty. If right is faster, I should increase here
#define MID_OFFSET     		4
#define STAGE_NUM      		(2 * MID_OFFSET + 1)
int servoValL[STAGE_NUM] = { \
    SERVO_NEG - 300, SERVO_NEG - 200, SERVO_NEG - 100, SERVO_NEG, \
    PWM_DUTY_MID, \
    SERVO_POS, SERVO_POS + 100, SERVO_POS + 200, SERVO_POS + 300 };
int servoValR[STAGE_NUM]  =  {0};
Servo motorL;
Servo motorR;
#endif

#ifdef ENABLE_VACUUM_CTL
// I2C control dyson motorLs
// I2C Devices address
// Use analog A0 to detect the pressure on left vacuum
// Use analog A1 to detect the pressure on right vacuum
#define VACUUM_LEFT_I2C_ADDR    0x62
#define VACUUM_RIGT_I2C_ADDR    0x63

#define VACUUM_START  0
#define VACUUM_STOP   1
#define VACUUM_ACCE   2
#define VACUUM_DECE   3
#define VACUUM_READ   4
#define VACUUM_10K    5
#define VACUUM_20K    6
#define VACUUM_30K    7
#define VACUUM_40K    8
#define VACUUM_50K    9
#define VACUUM_CMD_LENGTH  7

int vacuum_on_off = 0; // Determine if Vacuum is On or Off
unsigned char vacuum_cmd[][VACUUM_CMD_LENGTH] = { // vacuum control commands
  {0xe0, 0x00, 0x01, 0x7e, 0x00, 0x00, 0x7f}, // VACUUM_START
  {0xe0, 0x00, 0x01, 0x7f, 0x00, 0x00, 0x7e}, // VACUUM_STOP
  {0xe0, 0x00, 0x01, 0x02, 0x00, 0x00, 0x03}, // VACUUM_ACCE
  {0xe0, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00}, // VACUUM_DECE
  {0xe0, 0x00, 0x01, 0x10, 0x00, 0x00, 0x11}, // VACUUM_READ
  {0xe0, 0x00, 0x01, 0x03, 0x00, 0x78, 0x7a}, // VACUUM_10K
  {0xe0, 0x00, 0x01, 0x03, 0x00, 0x3c, 0x3e}, // VACUUM_20K
  {0xe0, 0x00, 0x01, 0x03, 0x00, 0x28, 0x2a}, // VACUUM_30K
  {0xe0, 0x00, 0x01, 0x03, 0x00, 0x1e, 0x1c}, // VACUUM_40K
  {0xe0, 0x00, 0x01, 0x03, 0x00, 0x1a, 0x18}  // VACUUM_50K
};

// pid set parameters
#define PID_PRESSURE_SET  0 //  Desire pressure set to (unit: kpa)
#define PID_P   1.0
#define PID_I   0.2
#define PID_D   0.3

// Dyson motor PID control setting
double pid_set, pid_input, pid_output_acc, pid_output_dec;
PID PID_acc(&pid_input, &pid_output_acc, &pid_set, PID_P, PID_I, PID_D, DIRECT);
PID PID_dec(&pid_input, &pid_output_dec, &pid_set, PID_P, PID_I, PID_D, REVERSE);
#endif

#ifdef ARDUINO_YUN
// arduino-linux bridge services
void doStop(YunClient client);
void process(YunClient client);
#endif

#ifdef ADAFRUIT_MOTOR_SHIELD;
Adafruit_motorShield AFMS = Adafruit_motorShield();
// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCmotorL *frontWheels = AFMS.getmotorL(3);
// You can also make another motorL on port M2
Adafruit_DCmotorL *rearWheels = AFMS.getmotorL(4);
#endif

#ifdef ARDUINO_YUN
// Socket port 5555 to communicate.
YunServer server(5555); 

// Internal settings - Can be changed dynamically from an external application using the "settings" action. 
const char _settingHonkPin = 8;// Cannot be changed by external application.
int settingHonkNote = 440;
int settingHonkDuration = 250;// Won't be changed from external application actually. 

void setup_wifi() {
  // Bridge startup for Arduino Yun
  Bridge.begin();
  Console.begin();
  // while (!Console){ ; } // wait for Console to connect.
  // Listen the entire network (socket communication)
  server.noListenOnLocalhost();
  server.begin();
}
#endif

#ifdef ENABLE_WHEEL_ODOMETRY
void setup_wheel_odometry() {
  pinMode(odo_LEncA, pid_input);
  pinMode(odo_LEncB, pid_input);	
  attachInterrupt(0, odo_dCheckL_isr, CHANGE);
  attachInterrupt(1, odo_dCheckR_isr, CHANGE);
  odo_Lcurrent_time = micros();	
  odo_Rcurrent_time = micros();	
}
#endif

#ifdef ENABLE_TIMERPWM_CTL
// https://www.pjrc.com/teensy/td_libs_TimerOne.html
void setup_timerpwm_ctl() {
  // setup timer for pwm control for left and right Wheel motorLs
  Timer1.initialize(PWM_PERIOD);
  // Timer1.setPeriod(PWM_PERIOD);
  Timer1.pwm(LEFT_WHEEL_CTL_PIN, PWM_DUTY(PWM_DUTY_MID));
  // Timer1.disablePwm(LEFT_WHEEL_CTL_PIN);
  Timer3.initialize(PWM_PERIOD);
  // Timer3.setPeriod(PWM_PERIOD);
  Timer3.pwm(RIGT_WHEEL_CTL_PIN, PWM_DUTY(PWM_DUTY_MID));
  // Timer3.disablePwm(RIGT_WHEEL_CTL_PIN);
  // delay(PWM_TEST_DELAY); // bing add this delay in the final
}
#endif

#ifdef ENABLE_SERVO_CTL
void setup_servo_ctl() {
  motorL.attach(LEFT_WHEEL_CTL_PIN);
  motorR.attach(RIGT_WHEEL_CTL_PIN);
  motorL.writeMicroseconds(PWM_DUTY_MID);
  motorR.writeMicroseconds(PWM_DUTY_MID);
  for (int i = 0; i < STAGE_NUM; i++) {
    servoValR[i] = servoValL[STAGE_NUM -1 - i];
  }
}
#endif

#ifdef ENABLE_VACUUM_CTL
void setup_vacuum_ctl() {
  // I2C for two vacuum motorLs
  Wire.begin();

  vacuum_on_off = 1; // Set vacuum to off

  pid_set = PID_PRESSURE_SET;
  PID_acc.SetOutputLimits(0.0, 99.0);
  PID_acc.SetMode(AUTOMATIC);
  PID_dec.SetOutputLimits(0.0, 99.0);
  PID_dec.SetMode(AUTOMATIC);
  
  vacuum_ctl(VACUUM_LEFT_I2C_ADDR, VACUUM_STOP);
  vacuum_ctl(VACUUM_RIGT_I2C_ADDR, VACUUM_STOP);
}
#endif

/**
* Entry point of the program.
* Initialize everything. Called when the Arduino is powered.
*/
void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  #ifdef ARDUINO_YUN
  setup_wifi();
  #endif
  #ifdef ENABLE_WHEEL_ODOMETRY
  setup_wheel_odometry();
  #endif
  #ifdef ENABLE_TIMERPWM_CTL
  setup_timerpwm_ctl();
  #endif
  #ifdef ENABLE_SERVO_CTL
  setup_servo_ctl();
  #endif
  #ifdef ENABLE_VACUUM_CTL
  setup_vacuum_ctl();
  #endif

  #ifdef ADAFRUIT_MOTOR_SHIELD
  AFMS.begin();  // create with the default frequency 1.6KHz
  // Set the speed to start, from 70 to 255 (max speed). 0 (off)
  frontWheels->setSpeed(0);
  frontWheels->run(FORWARD);
  frontWheels->run(RELEASE);
  //'turning' motorL
  rearWheels->setSpeed(0);
  rearWheels->run(FORWARD);
  rearWheels->run(RELEASE);
  #endif
}

/**
* Has to act as an -infinite- loop.
* Contains the program logic.
* Wait for a client then process it when he's found.
*/
void loop() {

  #ifdef ENABLE_VACUUM_CTL
  if (1 == vacuum_on_off) {
    float input = 0;
    input = vacuum_pressure(VACUUM_LEFT_I2C_ADDR);
    vacuum_pid(VACUUM_LEFT_I2C_ADDR, pid_set, input);
    // input = vacuum_pressure(VACUUM_RIGT_I2C_ADDR); // bing
    // vacuum_pid(VACUUM_RIGT_I2C_ADDR, pid_set, input);
  }
  #endif

  #ifdef TEST_WHEEL_ENCODER
  if (1 == odo_bool_dCheckL) {
    odo_bool_dCheckL = 0;
    my_print(odo_x);
    my_println("");
    my_print(odo_y);
    my_println("");
    my_print(odo_phi);
    my_println("");
    my_println("");
  }
  #endif

  int incomingByte;
  #ifdef ARDUINO_YUN
  // Get clients coming from server
  YunClient client = server.accept();
  // see if there's incoming serial data:
  if (Console.available() > 0) {
    // read the oldest byte in the serial buffer:
    incomingByte = Console.read();
    test(incomingByte);
  }
  #else
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    test(incomingByte);
  }
  #endif

  #ifdef ARDUINO_YUN
  // There is a new client
  if (client) {
    // Change the predifined timeout from 2000 to 5. Avoid impressive timeout.
    client.setTimeout(5);
    Serial.println("Client connected!");

    // When we get a client, go in the loop and exit only when the client disconnect. This will happens when the android application is killed (the socket must be closed by the app). This will automatically happens from the website for each http request.
    while(client.connected()){
      // Process request
      process(client);
    }
    // Stop the car for security reasons.
    doStop(client);

    // Close connection and free resources.
    client.stop();
  } else {
    Serial.println("no client connected, retrying");
  }
  #endif

  // Delay for the battery, for the debug too. Doesn't affect the response time of the Arduino. (Check if there is another client each second)
  delay(1000);
}

#ifdef ARDUINO_YUN
/**
* Will get the command request from the socket/http/etc. entry point.
* Will parse the request and execute it.
*/
void process(YunClient client) {
  // Format: COMMAND/SPEED
  String command = client.readStringUntil('/');// Get the first element of the command.
  
  // Avoid interferences when there is no request. (Because we are in an infinite loop!)
    if(command.length() > 0){
  
    //Serial.println("Query:"+client.readString()); 
    //return;// DEBUG
  
    // Parse the speed.
    int speed = client.parseInt();// Get the second element of the command.
    Serial.println((String) speed);
    if (command == "forward") {
      client.print(F("forward"));
      Serial.println("forward");  
      #ifdef ADAFRUIT_MOTOR_SHIELD
      frontWheels->setSpeed(speed);
      frontWheels->run(FORWARD);
      #else
        // #ifdef ENABLE_TIMERPWM_CTL
        #ifdef ENABLE_SERVO_CTL
        run_wheel(speed_stage);
        my_println_val("forward, speed = %d", (int)speed);
        #endif
      #endif
    }
    else if (command == "backward") {
      client.print(F("backward"));
      Serial.println("backward"); 
      #ifdef ADAFRUIT_MOTOR_SHIELD
      frontWheels->setSpeed(speed);
      frontWheels->run(BACKWARD);
      #else
        #ifdef ENABLE_SERVO_CTL
        run_wheel(speed_stage * (-1));
        my_println_val("backward, speed = %d", (int)speed);
        #endif
      #endif
    }
    else if (command == "left") {
      client.print(F("left"));
      Serial.println("left"); 
      #ifdef ADAFRUIT_MOTOR_SHIELD
      rearWheels->setSpeed(speed);// If use speed, doesn't works. (Bad parsing)
      rearWheels->run(BACKWARD);
      #else
        #ifdef ENABLE_SERVO_CTL
        turn_wheel(LEFT, speed_turn);
        my_println_val("turn left, speed = %d", (int)speed);
        #endif
      #endif
    }
    else if(command == "right"){
      client.print(F("right"));
      Serial.println("right");
      #ifdef ADAFRUIT_MOTOR_SHIELD
      rearWheels->setSpeed(speed);// If use speed, doesn't works. (Bad parsing)
      rearWheels->run(FORWARD);
      #else
        #ifdef ENABLE_SERVO_CTL
        turn_wheel(RIGT, speed_turn);
        my_println_val("turn right, speed = %d", (int)speed);
        #endif
      #endif
    }
    else if(command == "stop"){
      doStop(client);
      #ifdef ENABLE_SERVO_CTL
      run_wheel(0);
      #endif
    }
    else if(command == "stopTurn"){
      client.print(F("stopTurn"));
      Serial.println("stopTurn"); 
      #ifdef ADAFRUIT_MOTOR_SHIELD
      rearWheels->run(RELEASE);// Stop turn but don't do anything more.
      #else
        #ifdef ENABLE_SERVO_CTL
        run_wheel(0);
        #endif
      #endif
    }
    else if(command == "honk0") {
      speed_turn  = 0;
      speed_stage = 0;
    }
    else if(command == "honk1") {
      speed_turn  = 1;
      speed_stage = 1;
    }
    else if(command == "honk2") {
      speed_turn  = 1;
      speed_stage = 2;
    }
    else if(command == "honk3") {
      speed_turn  = 1;
      speed_stage = 3;
    }
    else if(command == "honk4") {
      speed_turn  = 1;
      speed_stage = 4;
      client.print(F("honk"));
      Serial.println("honk"); 
      tone(_settingHonkPin, settingHonkNote, settingHonkDuration);  //(PinNumber, Note, duration)
    }
    else if(command == "photo"){
      client.print(F("photo"));
      Serial.println("photo"); 
      // TODO Take a photo
    }
    else if(command == "settings"){
      client.print(F("settings"));
      Serial.println("settings");

    // Load the custom tone.
      settingHonkNote = client.parseInt();// Get the third element of the command.
    }
  }
}

void doStop(YunClient client){
  client.print(F("stop"));
  Serial.println("stop");
  #ifdef ADAFRUIT_MOTOR_SHIELD
    rearWheels->run(RELEASE);
    frontWheels->run(RELEASE);
  #else
    #ifdef ENABLE_SERVO_CTL
    run_wheel(0);
    #endif
  #endif
}
#endif

#ifdef ENABLE_WHEEL_ODOMETRY
// odometry
void odo_dCheckL_isr() {
  #ifdef TEST_WHEEL_ENCODER
  odo_bool_dCheckL = 1;
  #endif

  odo_Lcurrent_time = micros();

  if (odo_dCheckL_cnt == 0) { // First rotation does not have odo_pre_time
    odo_Lpre_time = odo_Lcurrent_time;
    odo_dCheckL_cnt = 1;
    return;
  }

  odo_Lperiod = (float)(odo_Lcurrent_time - odo_Lpre_time)/1000000;

  if (digitalRead(odo_LEncA) == digitalRead(odo_LEncB)) {
    odo_Lcount++;
    odo_Vl = 0.247369529/2/odo_Lperiod;
  } else {
    odo_Lcount--;
    odo_Vl = -0.247369529/2/odo_Lperiod;
  }
  odo_Dl = odo_Lcount*0.24736952914243591789542161710666;

  odo_D = (odo_Dl+odo_Dr)/2;
  odo_V = (odo_Vl+odo_Vr)/2;
  odo_w = (odo_Vl-odo_Vr)/12; // 12 inches is the distance between the left and right wheel
  odo_phi += odo_w*odo_Lperiod;
  // odo_phi -= (double)((int)(odo_phi/twopi))*twopi;
  odo_x += odo_V*sin(odo_phi)*odo_Lperiod;
  odo_y += odo_V*cos(odo_phi)*odo_Lperiod;

  odo_Lpre_time = odo_Lcurrent_time;

  //my_print(odo_Dl);
  //my_print("\t");
  //my_println(odo_Vl);
  //my_println(odo_phi);
  //my_print(odo_x);
  //my_print("\t");
  //my_println(odo_y);
  //my_println(odo_period,10);
}

void odo_dCheckR_isr() { 
  if (odo_dCheckR_cnt == 0) { // First rotation does not have odo_pre_time
    odo_Rpre_time = odo_Rcurrent_time;
    odo_dCheckR_cnt = 1;
    return;
  }

  odo_Rperiod = (float)(odo_Rcurrent_time - odo_Rpre_time)/1000000;

  if (digitalRead(odo_LEncA) == digitalRead(odo_LEncB)) {
    odo_Rcount++;
    odo_Vr = 0.247369529/2/odo_Rperiod;
  } else {
    odo_Rcount--;
    odo_Vr = -0.247369529/2/odo_Rperiod;
  }
  odo_Dr = odo_Rcount*0.24736952914243591789542161710666;

  odo_Rpre_time = odo_Rcurrent_time;
  }
#endif

#ifdef ENABLE_VACUUM_CTL
void vacuum_ctl(int addr, int cmd)
{
  Wire.beginTransmission(addr);
  Wire.write(&vacuum_cmd[cmd][0], VACUUM_CMD_LENGTH);
  Wire.endTransmission();
  delay(500);
}

float vacuum_pressure(int which)
{
  int sensorValue = 0;
  float voltage   = 0;
  float pressure  = 0;
  #ifdef TEST_VACUUM_CTL
  my_println("\n");
  #endif

  // read the pid_input on analog pin 0:
  if (VACUUM_LEFT_I2C_ADDR == which)
    sensorValue = analogRead(A0);
  else if (VACUUM_RIGT_I2C_ADDR == which)
    sensorValue = analogRead(A1);
  #ifdef TEST_VACUUM_CTL
  my_println_val("sensorValue = %d", sensorValue);
  #endif

  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  voltage = sensorValue * (5.0 / 1023.0);
  // print out the value you read:
  pressure = (voltage / 5.0 - 0.04) /0.009;
  #ifdef TEST_VACUUM_CTL
  my_print(pressure);
  my_println("");
  #endif

  return pressure;
}

void vacuum_pid(int addr, float set, float input)
{
  pid_input = input;
  pid_set = set;

  PID_dec.Compute();
  PID_acc.Compute();
  #ifdef TEST_VACUUM_CTL
  my_print(pid_output_acc);
  my_println("");
  my_print(pid_output_dec);
  my_println("");
  #endif

  if(pid_output_acc - pid_output_dec > 4)
    vacuum_ctl(addr, VACUUM_DECE);
  else if(pid_output_acc - pid_output_dec < -4)
  	vacuum_ctl(addr, VACUUM_ACCE);
  else {
  }
}
#endif

#ifdef TEST_VACUUM_CTL
void test_vacuum_ctl(int addr, char c)
{
  switch (c)
  {
  case 'S':
    vacuum_ctl(addr, VACUUM_START);
    delay(500);
    vacuum_ctl(addr, VACUUM_50K);
    vacuum_on_off = 1;
    break;
  case 'B':
    vacuum_ctl(addr, VACUUM_STOP);
    // vacuum_on_off = 0; // bing
    break;
  case 'A': vacuum_ctl(addr, VACUUM_ACCE);
    break;
  case 'D': vacuum_ctl(addr, VACUUM_DECE);
    break;
  case 'R': vacuum_ctl(addr, VACUUM_READ);
    break;
  case '5': vacuum_ctl(addr, VACUUM_50K);
    break;
  case '4': vacuum_ctl(addr, VACUUM_40K);
    break;
  case '3': vacuum_ctl(addr, VACUUM_30K);
    break;
  case '2': vacuum_ctl(addr, VACUUM_20K);
    break;
  case '1': vacuum_ctl(addr, VACUUM_10K);
    break;
  default: return; // other command just return;
  }
}
#endif

#ifdef ENABLE_TIMERPWM_CTL
// if right side is faster, I should increase this value
double adjuct = 0.772;
// 5.67: 58.0608  / 5.68: 58.1632 / 5.7:  58.368
// 5.772:  59.10528, still right side is faster
// 5.87:   60.1088

void turn_wheel(int dir)
{
  // duty default is 1st stage speed
  double positive_start_duty = 8;
  double negative_start_duty = 5 + adjuct;
  if (LEFT == dir) {
    Timer1.pwm(LEFT_WHEEL_CTL_PIN, PWM_DUTY(negative_start_duty));
    Timer3.pwm(RIGT_WHEEL_CTL_PIN, PWM_DUTY(negative_start_duty));
  }
  else if (RIGT == dir) {
    Timer1.pwm(LEFT_WHEEL_CTL_PIN, PWM_DUTY(positive_start_duty));
    Timer3.pwm(RIGT_WHEEL_CTL_PIN, PWM_DUTY(positive_start_duty));
  } else {
    run_wheel(0);
  }
}

void run_wheel(int dir_duty) // -3 -2 -1 1 2 3
{
  double value_l = 0;
  double value_r = 0;
  switch(dir_duty) {
    case -3:
      value_l = 3 + adjuct;
      value_r = 10;
      break;
    case -2:
      value_l = 4 + adjuct;
      value_r = 9;
      break;
    case -1:
      value_l = 5 + adjuct;
      value_r = 8;
      break;
    case  0:
      value_l = PWM_DUTY_MID;
      value_r = PWM_DUTY_MID;
      break;
    case  1:
      value_l = 8;
      value_r = 5 + adjuct;
      break;
    case  2:
      value_l = 9;
      value_r = 4 + adjuct;
      break;
    case  3:
      value_l = 10;
      value_r = 3 + adjuct;
     break;
  }
  Timer1.pwm(LEFT_WHEEL_CTL_PIN, PWM_DUTY(value_l));
  Timer3.pwm(RIGT_WHEEL_CTL_PIN, PWM_DUTY(value_r));
}
#endif

#ifdef TEST_TIMERPWM_CTL
void test_timerpwm_ctl(int value)
{
  double speed = PWM_DUTY_MID;
  double step = value * 0.1;

  while(1) {
    if (Serial.available()) {
      int input = Serial.read();
      if (input == 'a'){
        speed += step;
        Timer1.pwm(LEFT_WHEEL_CTL_PIN, PWM_DUTY(speed));
        Timer3.pwm(RIGT_WHEEL_CTL_PIN, PWM_DUTY(speed));
        Serial.println(speed);
        input = 0;
      } else if (input == 'b') {
        speed -= step;
        Timer1.pwm(LEFT_WHEEL_CTL_PIN, PWM_DUTY(speed));
        Timer3.pwm(RIGT_WHEEL_CTL_PIN, PWM_DUTY(speed));
        Serial.println(speed);
        input = 0;
      } else if (input == 'q') {
        Timer1.pwm(LEFT_WHEEL_CTL_PIN, PWM_DUTY(PWM_DUTY_MID));
        Timer3.pwm(RIGT_WHEEL_CTL_PIN, PWM_DUTY(PWM_DUTY_MID));
        return;
      }
    }
  }
}
#endif

#ifdef ENABLE_SERVO_CTL
void turn_wheel(int dir, int speedTurn)
{
  if (LEFT == dir) { // duty default is 1st stage speed
    motorL.writeMicroseconds(servoValL[MID_OFFSET - speedTurn]);
    motorR.writeMicroseconds(servoValR[MID_OFFSET + speedTurn]);
  }
  else if (RIGT == dir) {
    motorL.writeMicroseconds(servoValL[MID_OFFSET + speedTurn]);
    motorR.writeMicroseconds(servoValR[MID_OFFSET - speedTurn]);
  } else {
    run_wheel(0);
  }
}

void run_wheel(int dir_duty)
{
  dir_duty += MID_OFFSET;
  motorL.writeMicroseconds(servoValL[dir_duty]);
  motorR.writeMicroseconds(servoValR[dir_duty]);
}
#endif

#ifdef TEST_SERVO_CTL
void test_servo_ctl(int value)
{
  int speed = PWM_DUTY_MID;
  int step = value * 10;

  while(1) {
    if (Serial.available()) {
      int input = Serial.read();
      if (input == 'a'){
        speed += step;
        motorL.writeMicroseconds(speed);
        Serial.println(speed);
        input = 0;
      } else if (input == 'b') {
        speed -= step;
        motorL.writeMicroseconds(speed);
        Serial.println(speed);
        input = 0;
      } else if (input == 'q') {
        motorL.writeMicroseconds(PWM_DUTY_MID);
        return;
      }
    }
  }
}
#endif

#ifdef ENABLE_DEBUG_RECORD
void test_record_val()
{
  my_print(0);
  my_println(": print & println");
  my_println_val("sizeof(char)     = %d", sizeof(char));
  my_println_val("sizeof(int)      = %d", sizeof(int));
  my_println_val("sizeof(double)   = %d", sizeof(double));
  my_println_val("sizeof(long)     = %d", sizeof(long));
  my_println_val("sizeof(float)    = %d", sizeof(float));
  my_println_val("sizeof(long int) = %d", sizeof(long int));
  for (int i = 0; i < 4; i++) {
    my_println_val("record_val[] = %d", (int) (100 * record_val[i]));
  }
}
#endif

void test(int incomingByte)
{
  my_println_val("incomingByte: %c", incomingByte);
  if (incomingByte == '\n')  { }
  else if (incomingByte == 'o') { // o, turn on the LED:
    digitalWrite(ledPin, HIGH);
    my_println("turn on LED");
  }
  else if (incomingByte == 'f') { // f, turn off the LED:
    digitalWrite(ledPin, LOW);
    my_println("turn off LED");
  }
#ifdef TEST
  else if (incomingByte == 't') {
    #ifdef ENABLE_DEBUG_RECORD
    my_println_val("test_record_val %c", incomingByte);
    test_record_val();
    #endif
  }
  else {
    int value = incomingByte - '0';
    #ifdef TEST_VACUUM_CTL
    test_vacuum_ctl(VACUUM_LEFT_I2C_ADDR, incomingByte);
    #endif
    #ifdef TEST_TIMERPWM_CTL
    test_timerpwm_ctl(value);
    #endif
    #ifdef TEST_SERVO_CTL
    test_servo_ctl(value);
    #endif
  }
#endif
}


