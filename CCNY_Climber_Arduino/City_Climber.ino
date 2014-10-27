/* CCNY-City-Climber Robot Controller code (CCNY Robotics Lab)
 * Author:   Bing Li, Jonathan Liu, Zihui Cui
 * Date:     Aug. 2014
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

char enable_print = 0;

//#define DEBUG
#define ARDUINO_YUN
#define WIFI_PRINT

#define ENABLE_WHEEL_ODOMETRY

//#define ENABLE_VACUUM_CTL
//#define TEST_VACUUM_CTL
//#define ONLY_ONE_VACUUM_CTL

//#define ENABLE_SERVO_CTL
//#define TEST_SERVO_CTL

//#define ENABLE_TIMERPWM_CTL
//#define TEST_TIMERPWM_CTL

//#define ENABLE_DEBUG_RECORD

double pid_input;

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

#ifdef ENABLE_VACUUM_CTL
#include <Wire.h>
#include <PID_v1.h>
#include "Vacuum.h"
#endif

#ifdef ENABLE_WHEEL_ODOMETRY
#include "Odometry.h"
#endif

#ifdef ENABLE_TIMERPWM_CTL
#include <TimerOne.h>
#include <TimerThree.h>
#include "PWM.h"
#endif

#ifdef  ENABLE_SERVO_CTL
#include <Servo.h>
#include "ServoMotor.h"
#endif

// define a selective print
char serialPrintString[32] = {0};
#ifdef  WIFI_PRINT
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
  loop_cnt++;

  #ifdef ENABLE_VACUUM_CTL
  vacuum_process();
  #endif
  
  char incomingByte;
  #ifdef ARDUINO_YUN
    // Get clients coming from server
  YunClient client = server.accept();
    // see if there's incoming serial data:
  if (Console.available() > 0) {
    // read the oldest byte in the serial buffer:
    incomingByte = Console.read();
  }
  #else
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
  }
  #endif
    
  #ifdef DEBUG
  test(incomingByte);
  #endif
  
  #ifdef ARDUINO_YUN
  // There is a new client
  if (client.connected()) {
    // Change the predifined timeout from 2000 to 5. Avoid impressive timeout.
    client.setTimeout(5);
    Serial.println("Client connected!");
    // When we get a client, go in the loop and exit only when the client disconnect. This will happens when the android application is killed (the socket must be closed by the app). This will automatically happens from the website for each http request.
    int client_connect_count = 0;
    while(client.connected()){
      // Process request
      process(client);
      Bridge.put("values", String(x) + "," + String(y) + "," + String(phi));
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


