// Odometry

#ifdef ENABLE_WHEEL_ODOMETRY

// setup for odometry
void setup_wheel_odometry() {
  pinMode(LEncA, pid_input);
  pinMode(LEncB, pid_input);	
  attachInterrupt(0, dCheckL_isr, CHANGE);
  attachInterrupt(1, dCheckR_isr, CHANGE);
  Lcurrent_time = micros();	
  Rcurrent_time = micros();	
}

void dCheckL_isr() {
  bool_dCheckL = 1;

  Lcurrent_time = micros();

  if (dCheckL_cnt == 0) { // First rotation does not have pre_time
    Lpre_time = Lcurrent_time;
    dCheckL_cnt = 1;
    return;
  }

  Lperiod = (float)(Lcurrent_time - Lpre_time)/1000000;

  if (digitalRead(LEncA) == digitalRead(LEncB)) {
    Lcount++;
    Vl = 0.247369529/2/Lperiod;
  } else {
    Lcount--;
    Vl = -0.247369529/2/Lperiod;
  }
  Dl = Lcount*0.247369529;

  D = (Dl+Dr)/2;
  V = (Vl+Vr)/2;
  w = (Vl-Vr)/12; // 12 inches is the distance between the left and right wheel
  phi += w*Lperiod;
  // phi -= (double)((int)(phi/twopi))*twopi;
  x += V*sin(phi)*Lperiod;
  y += V*cos(phi)*Lperiod;

  Lpre_time = Lcurrent_time;
}

void dCheckR_isr() { 
  if (dCheckR_cnt == 0) { // First rotation does not have pre_time
    Rpre_time = Rcurrent_time;
    dCheckR_cnt = 1;
    return;
  }

  Rperiod = (float)(Rcurrent_time - Rpre_time)/1000000;

  if (digitalRead(LEncA) == digitalRead(LEncB)) {
    Rcount++;
    Vr = 0.247369529/2/Rperiod;
  } else {
    Rcount--;
    Vr = -0.247369529/2/Rperiod;
  }
  Dr = Rcount*0.247369529;

  Rpre_time = Rcurrent_time;
  }
  
void display_odo() {
  if (1 == bool_dCheckL) {
    bool_dCheckL = 0;
    my_println(x);
    my_println(y);
    my_println(phi);
    my_println("");
  }
}
#endif
