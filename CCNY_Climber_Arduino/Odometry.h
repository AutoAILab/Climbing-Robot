// Odometry header

#define LEncA      2
#define LEncB      7
#define REncA      3
#define REncB      8
//#define pi         3.14159265359
//#define twopi      pi*2

volatile int  Lcount = 0; // counter for left wheel
volatile int  Rcount = 0;

volatile bool bool_dCheckL = 0;

float 	Dl  = 0; // total displacement for left wheel
float 	Dr  = 0;
float 	D   = 0;
float 	Vl  = 0; // current velocity for left wheel
float 	Vr  = 0;
float 	V   = 0;
float 	x   = 0; // x position
float 	w   = 0;
float 	y   = 0; // y position
double 	phi = 0; // angle

long  	Lcurrent_time = 0;
long  	Lpre_time     = 0;

long  	Rcurrent_time = 0;
long  	Rpre_time     = 0;

float 	Lperiod       = 0;
float   Rperiod       = 0;
int     dCheckL_cnt   = 0;
int     dCheckR_cnt   = 0;
