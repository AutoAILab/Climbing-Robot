// PWM header (motor control)

// PWM control wheel motorLs
// percent: 0~100 since range is 0~1024
#define PWM_DUTY(percent)       ((int)(percent * 10.24))
#define PWM_FREQUENCY           50    // HZ
#define PWM_PERIOD              (int)(1000000/PWM_FREQUENCY)
#define PWM_TEST_DELAY          1200  // ms
#define LEFT_WHEEL_CTL_PIN      9 // pwm pin for left wheel
#define RIGT_WHEEL_CTL_PIN      5 // pwm pin for right wheel
#define PWM_DUTY_MIN            3
#define PWM_DUTY_MID            7 // 7 seems to be neutral, 6 is extremely slow
#define PWM_DUTY_MAX            10
