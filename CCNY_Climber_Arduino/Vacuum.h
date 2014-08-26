// Vacuum header

// I2C control dyson motorLs
// I2C Devices address
// Use analog A0 to detect the pressure on left vacuum
// Use analog A1 to detect the pressure on right vacuum
#define VACUUM_LEFT_I2C_ADDR    0x62
#define VACUUM_RIGT_I2C_ADDR    0x63

#define VACUUM_START  0
#define VACUUM_STOP   1
#define VACUUM_ACCE   2
#define VACUUM_DECE   3
#define VACUUM_READ   4
#define VACUUM_10K    5
#define VACUUM_20K    6
#define VACUUM_30K    7
#define VACUUM_40K    8
#define VACUUM_50K    9
#define VACUUM_CMD_LENGTH  7

int enable_vacuum = 0; // enable/disable vacuum, control by User interface
int vacuum_status = 0; // status of vacuum motor, start/stop
unsigned char vacuum_cmd[][VACUUM_CMD_LENGTH] = { // vacuum control commands
  {0xe0, 0x00, 0x01, 0x7e, 0x00, 0x00, 0x7f}, // VACUUM_START
  {0xe0, 0x00, 0x01, 0x7f, 0x00, 0x00, 0x7e}, // VACUUM_STOP
  {0xe0, 0x00, 0x01, 0x02, 0x00, 0x00, 0x03}, // VACUUM_ACCE
  {0xe0, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00}, // VACUUM_DECE
  {0xe0, 0x00, 0x01, 0x10, 0x00, 0x00, 0x11}, // VACUUM_READ
  {0xe0, 0x00, 0x01, 0x03, 0x00, 0x78, 0x7a}, // VACUUM_10K
  {0xe0, 0x00, 0x01, 0x03, 0x00, 0x3c, 0x3e}, // VACUUM_20K
  {0xe0, 0x00, 0x01, 0x03, 0x00, 0x28, 0x2a}, // VACUUM_30K
  {0xe0, 0x00, 0x01, 0x03, 0x00, 0x1e, 0x1c}, // VACUUM_40K
  {0xe0, 0x00, 0x01, 0x03, 0x00, 0x1a, 0x18}  // VACUUM_50K
};

char vacuum_cmd_str[][10] = {
  "start",  "stop",  "acce",  "dece",  "read",
  "10k",  "20k",  "30k",  "40k",  "50k"
};

// pid set parameters
#define PID_PRESSURE_SET  10 //  Desire pressure set to (unit: kpa)
#define PID_P   1.0
#define PID_I   0.2
#define PID_D   0.3

// Dyson motor PID control setting
double pid_set, pid_output_acc, pid_output_dec, pid_input;
PID PID_acc(&pid_input, &pid_output_acc, &pid_set, PID_P, PID_I, PID_D, DIRECT);
PID PID_dec(&pid_input, &pid_output_dec, &pid_set, PID_P, PID_I, PID_D, REVERSE);
