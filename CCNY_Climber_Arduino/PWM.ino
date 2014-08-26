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

// if right side is faster, I should increase this value
double adjuct = 0.772;
// 5.67: 58.0608  / 5.68: 58.1632 / 5.7:  58.368
// 5.772:  59.10528, still right side is faster
// 5.87:   60.1088

void turn_wheel(int dir) {
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

void run_wheel(int dir_duty) { // -3 -2 -1 1 2 3
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
void test_timerpwm_ctl(int value) {
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
