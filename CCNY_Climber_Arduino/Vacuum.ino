#ifdef ENABLE_VACUUM_CTL
void setup_vacuum_ctl() {
  // I2C for two vacuum motorLs
  Wire.begin();
  enable_vacuum = 0; // Set vacuum to off
  vacuum_ctl(VACUUM_LEFT_I2C_ADDR, VACUUM_STOP);
  vacuum_ctl(VACUUM_RIGT_I2C_ADDR, VACUUM_STOP);

  pid_set = PID_PRESSURE_SET;
  PID_acc.SetOutputLimits(0.0, 99.0);
  PID_acc.SetMode(AUTOMATIC);
  PID_dec.SetOutputLimits(0.0, 99.0);
  PID_dec.SetMode(AUTOMATIC);
}

void vacuum_process() {
  float input = 0;
  if ((1 == enable_vacuum) && (1 == vacuum_status)) {
    input = vacuum_pressure(VACUUM_LEFT_I2C_ADDR);
    vacuum_pid(VACUUM_LEFT_I2C_ADDR, pid_set, input);
    input = vacuum_pressure(VACUUM_RIGT_I2C_ADDR); // bing
    vacuum_pid(VACUUM_RIGT_I2C_ADDR, pid_set, input);
    return;
  }
  if ((1 == enable_vacuum) && (0 == vacuum_status)) {
    // first time, start vacuum motor
    vacuum_ctl(VACUUM_LEFT_I2C_ADDR, VACUUM_START);
    vacuum_ctl(VACUUM_RIGT_I2C_ADDR, VACUUM_START);
    delay(2000);
    return;
  }
}

void vacuum_ctl(char addr, int cmd)
{
#ifdef ONLY_ONE_VACUUM_CTL
  if (VACUUM_RIGT_I2C_ADDR == addr) // support right I2C or not, bing
    return;
#endif

  if (VACUUM_START == cmd) {
    delay(4000); // bing
    vacuum_status = 1;
    enable_vacuum = 1;
  } else if (VACUUM_STOP == cmd) {
    vacuum_status = 0;
    enable_vacuum = 0;
  }

  if (1 == enable_print) {
    my_println_val("vacuum_ctl: %s", &vacuum_cmd_str[cmd][0]);
  }

  Wire.beginTransmission(addr);
  Wire.write(&vacuum_cmd[cmd][0], VACUUM_CMD_LENGTH);
  Wire.endTransmission();
  delay(500); // bing
}

float vacuum_pressure(char addr)
{
  int sensorValue = 0;
  float voltage   = 0;
  float pressure  = 0;

#ifdef ONLY_ONE_VACUUM_CTL
  if (VACUUM_RIGT_I2C_ADDR == addr) // support right I2C or not, bing
    return 0;
#endif

  // read the pid_input on analog pin 0:
  if (VACUUM_LEFT_I2C_ADDR == addr)
    sensorValue = analogRead(A0);
  else if (VACUUM_RIGT_I2C_ADDR == addr)
    sensorValue = analogRead(A1);

  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  voltage = sensorValue * (5.0 / 1023.0);
  // print out the value you read:
  pressure = (voltage / 5.0 - 0.04) /0.009;

  if (1 == enable_print) {
    my_println("\n");
    my_println_val("sensorValue = %d", sensorValue);
    my_print(voltage);
    my_println(" V");  
    my_print(pressure);
    my_println(" Kpa");
  }

  return pressure;
}

void vacuum_pid(char addr, float set, float input)
{
  pid_input = input;
  pid_set = set;

#ifdef ONLY_ONE_VACUUM_CTL
  if (VACUUM_RIGT_I2C_ADDR == addr) // support right I2C or not, bing
    return;
#endif

  PID_dec.Compute();
  PID_acc.Compute();
  if(pid_output_acc - pid_output_dec > 4)
    vacuum_ctl(addr, VACUUM_ACCE);
  else if(pid_output_acc - pid_output_dec < -4)
    vacuum_ctl(addr, VACUUM_DECE);

  if (1 == enable_print) {
    my_print(pid_output_acc);
    my_println(" acc");
    my_print(pid_output_dec);
    my_println(" dec");
  }
}
#endif

#ifdef TEST_VACUUM_CTL
void test_vacuum_run(char addr)
{
  digitalWrite(ledPin, HIGH);
  vacuum_ctl(addr, VACUUM_START);
  delay(3000);
  vacuum_ctl(addr, VACUUM_20K);
  delay(3000);
  vacuum_ctl(addr, VACUUM_40K);
  delay(3000);
  vacuum_ctl(addr, VACUUM_20K);
  delay(3000);
  vacuum_ctl(addr, VACUUM_STOP);
  delay(3000);
  digitalWrite(ledPin, LOW);
}

void test_vacuum_ctl(char addr, char c)
{
  switch (c)
  {
    case 'V':
      if (0 == enable_vacuum) { // enable/disable vacuum
        enable_vacuum = 1;
        my_println("enable vacuum");
      } else if (1 == enable_vacuum) {
        enable_vacuum = 0;
        my_println("disable vacuum");
      }
      break;
    case 'S':
      vacuum_ctl(addr, VACUUM_START);
      break;
    case 'B':
      vacuum_ctl(addr, VACUUM_STOP);
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

