#ifdef ARDUINO_YUN
void setup_wifi() {
  // Bridge startup for Arduino Yun
  Bridge.begin();
  Console.begin();
  // while (!Console){ ; } // wait for Console to connect.
  // Listen the entire network (socket communication)
  server.noListenOnLocalhost();
  server.begin();
}

/**
* Will get the command request from the socket/http/etc. entry point.
* Will parse the request and execute it.
*/
void process(YunClient client) {
  // Format: COMMAND/SPEED
  String command = client.readStringUntil('/');// Get the first element of the command.
  
  // Avoid interferences when there is no request. (Because we are in an infinite loop!)
    if(command.length() > 0) {
  
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
    } else if (command == "backward") {
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
    } else if (command == "left") {
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
    } else if(command == "right") {
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
    } else if(command == "stop") {
      doStop(client);
      #ifdef ENABLE_SERVO_CTL
      run_wheel(0);
      #endif
    } else if(command == "stopTurn") {
      enable_vacuum = 0; // bing for vacuum on/off
      client.print(F("stopTurn"));
      Serial.println("stopTurn"); 
      #ifdef ADAFRUIT_MOTOR_SHIELD
      rearWheels->run(RELEASE);// Stop turn but don't do anything more.
      #else
        #ifdef ENABLE_SERVO_CTL
        run_wheel(0);
        #endif
      #endif
    } else if(command == "honk0") {
      speed_turn  = 0;
      speed_stage = 0;
    } else if(command == "honk1") {
      speed_turn  = 1;
      speed_stage = 1;
    } else if(command == "honk2") {
      speed_turn  = 1;
      speed_stage = 2;
    } else if(command == "honk3") {
      speed_turn  = 1;
      speed_stage = 3;
    } else if(command == "honk4") {
      enable_vacuum = 1; // bing for vacuum on/off
      speed_turn  = 1;
      speed_stage = 4;
      client.print(F("honk"));
      Serial.println("honk"); 
      tone(_settingHonkPin, settingHonkNote, settingHonkDuration);  //(PinNumber, Note, duration)
    } else if(command == "photo"){
      client.print(F("photo"));
      Serial.println("photo"); 
      // TODO Take a photo
    } else if(command == "settings"){
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
