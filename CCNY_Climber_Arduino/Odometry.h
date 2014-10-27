// Odometry header

#define LEncA      3 // Interrupt 0
#define LEncB      8
#define REncA      2 // Interrupt 1
#define REncB      10
//#define pi         3.14159265359
//#define twopi      pi*2

volatile int  Lcount = 0; // counter for left wheel
volatile int  Rcount = 0;

boolean Lcheck = true; // check if first rotation
boolean Rcheck = true;

float    Dl  = 0; // total displacement for left wheel
float    Dr  = 0;
float    D   = 0;
float    Vl  = 0; // current velocity for left wheel
float    Vr  = 0;
float    V   = 0;
float    x   = 0; // x position
float    w   = 0;
float    y   = 0; // y position
double   phi = 0; // angle

long    Lcurrent_time = 0;
long    Lpre_time     = 0;
long    Rcurrent_time = 0;
long    Rpre_time     = 0;
float   Lperiod       = 0;
float   Rperiod       = 0;

void resetOdometry() {
  Lcount = 0;
  Rcount = 0;
  
  Lcheck = true;
  Rcheck = true;
  
  Dl  = 0;
  Dr  = 0;
  D   = 0;
  Vl  = 0;
  Vr  = 0;
  V   = 0;
  x   = 0;
  w   = 0;
  y   = 0;
  phi = 0;
  
  Lcurrent_time = 0;
  Lpre_time     = 0;
  Rcurrent_time = 0;
  Rpre_time     = 0;
  Lperiod       = 0;
  Rperiod       = 0;
}
