#ifndef __CPLD_FAN_H__

#define DRV_NAME "cpld_fan"
#define FAN_NUM_MAX 6
#define FANR_NUM_MAX 6
#define TEMP_NUM_MAX 6
#define PRODUCT_NAME_PATH "/sys/class/swmon/ctrl/product_id"

/* XX: AS4630/AS4610
 * controlled by DUTY_BIT macro
 * for example:
 * bits is 4(0b0000 to 0b10000),
 * then 255 = 8 + 16*15 + 8 (devide into 17 parts)
 * bits is 3
 * then 255 = 16 + 32*7 + 16 (devide into 9 parts)
*/
#define AS_PWM_TO_DUTY(X,Y)		((((X)>>(8-(Y)-1)) + 1)>>1)
#define AS_DUTY_TO_PWM(X,Y)		((X) == 0 ?(1<<((8-(Y)-2))) :\
	((X) == ((1<<(Y)) /*-1*/) ? (255 - (1<<((8-(Y)-2)))) : ((X)<<(8-(Y)))))

/* XX AS5XXX
 * 255 = 4 + 19*13 + 4
 */
//#define DUTY_TO_PWM(X) (((X) * 255) / 20)
//#define PWM_TO_DUTY(X) (((X) * 20) / 255)
#define AS5_PWM_TO_DUTY(X)          (((X)<4)?0:(((X)-4)/13+1))
#define AS5_DUTY_TO_PWM(X)          ((X)==0?2:((X)==20?(255-2):(4+((X)-1)*(13)+(13/2))))

/* XX: AS7XXX
 * 255 = 8 + 14*17 + 9
 */
//#define DUTY_TO_PWM(X) (((X+1) << 4 ) - 1)
//#define PWM_TO_DUTY(Y) (((Y+1) >> 4) - 1)
#define AS7_PWM_TO_DUTY(X)          (((X)<8)?0:(((X)-8)/17 + 1))
#define AS7_DUTY_TO_PWM(X)          ((X)==0?4:((X)==15?(255-4):(8+((X)-1)*(17) + (17/2))))

struct fan_info {
	struct mutex update_lock;
	unsigned long last_updated;	/* In jiffies */
	struct platform_device *pdev;
	int valid;

	// fan
	int fan_num;
	u32 fan_input[FAN_NUM_MAX];
	bool fan_fault[FAN_NUM_MAX];

	// fanr
	int fanr_num;
	u32 fanr_input[FANR_NUM_MAX];
	bool fanr_fault[FANR_NUM_MAX];

	// pwm: same to fan, not fanr
	u8 pwm[FAN_NUM_MAX];

	// temperature
	int temp_num;
	u32 temp_input[TEMP_NUM_MAX];

	// func
	int (*cpld_get_fan_input)(int index);
	int (*cpld_get_fanr_input)(int index);
	int (*cpld_get_fan_fault)(int index);
	int (*cpld_get_fanr_fault)(int index);
	int (*cpld_get_pwm)(int index);
	int (*cpld_set_pwm)(int index, u8 pwm);
	int (*cpld_get_temp_input)(int index);

	//
};

extern void fan_prepare_probe(int fan_num, int fanr_num, int temp_num,
	     struct fan_info *f);
extern int fan_common_probe(struct platform_device *pdev);
#endif
