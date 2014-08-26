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

void turn_wheel(int dir, int speedTurn) {
  if (LEFT == dir) { // duty default is 1st stage speed
    motorL.writeMicroseconds(servoValL[MID_OFFSET - speedTurn]);
    motorR.writeMicroseconds(servoValR[MID_OFFSET + speedTurn]);
  } else if (RIGT == dir) {
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
void test_servo_ctl(int value) {
  int speed = PWM_DUTY_MID;
  int step = value * 10;

  while(1) {
    if (Serial.available()) {
      int input = Serial.read();
      if (input == 'a'){
        speed += 10;
        motorL.writeMicroseconds(speed);
        Serial.println(speed);
        input = 0;
      } else if (input == 'b') {
        speed -= 10;
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
