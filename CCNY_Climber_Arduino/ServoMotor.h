// Servo header (motor control)

#define LEFT_WHEEL_CTL_PIN      9
#define RIGT_WHEEL_CTL_PIN      5
#define PWM_DUTY_MAX     	2400
#define PWM_DUTY_MIN     	500
#define PWM_DUTY_MID     	1360 // ((PWM_DUTY_MAX + PWM_DUTY_MIN) / 2) == 1440
#define SERVO_NEG        	(1240 - 0) // servo control negative-start duty
#define SERVO_POS        	(1480 - 20) // servo control positive-start duty. If right is faster, I should increase here
#define MID_OFFSET     		4
#define STAGE_NUM      		(2 * MID_OFFSET + 1)
int servoValL[STAGE_NUM] = { \
    SERVO_NEG - 300, SERVO_NEG - 200, SERVO_NEG - 100, SERVO_NEG, \
    PWM_DUTY_MID, \
    SERVO_POS, SERVO_POS + 100, SERVO_POS + 200, SERVO_POS + 300 };
int servoValR[STAGE_NUM]  =  {0};
Servo motorL;
Servo motorR;
