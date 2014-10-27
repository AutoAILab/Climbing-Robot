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
  Lcurrent_time = micros();

  if (Lcheck) { // First rotation does not have pre_time
    Lcheck = false;
    Lpre_time = Lcurrent_time;
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
  Rcurrent_time = micros();
  
  if (Rcheck) { // First rotation does not have pre_time
    Rcheck = false;
    Rpre_time = Rcurrent_time;
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
#endif
