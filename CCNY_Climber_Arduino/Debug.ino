#ifdef ENABLE_DEBUG_RECORD
void test_record_val() {
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

#ifdef DEBUG
void test(char incomingByte) {
  if (incomingByte == '\n') {
  }
  else if (incomingByte == 'p') { // enable/disable print
    my_println_val("incomingByte: %c", incomingByte);
    if (0 == enable_print) {
      enable_print = 1;
      my_println("enable print");
    } else if (1 == enable_print) {
      enable_print = 0;
      my_println("disable print");
    }
  }
  else if (incomingByte == 'o') { // o, turn on the LED:
    digitalWrite(ledPin, HIGH);
    my_println("turn on LED");
  }
  else if (incomingByte == 'f') { // f, turn off the LED:
    digitalWrite(ledPin, LOW);
    my_println("turn off LED");
  }
  else if (incomingByte == 'r') { // autuomatically run after 1.5s
    my_println_val("incomingByte: %c", incomingByte);
    #ifdef TEST_VACUUM_CTL
    test_vacuum_run(VACUUM_LEFT_I2C_ADDR);
    #endif
  }
  else {
    // my_println_val("incomingByte: %c", incomingByte);
    int value = incomingByte - '0';
    #ifdef ENABLE_DEBUG_RECORD
    test_record_val();
    #endif
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
}
#endif


