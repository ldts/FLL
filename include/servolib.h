#ifndef __SERVOLIB_H_
#define __SERVOLIB_H_

#define MAX_DUTY	95
#define MIN_DUTY	5

struct servo_params {
	int channel;
	int position;
	int home_position;
	int min_position;
	int max_position;
	int poserr;
	int accel_limit;
	int speed_limit;
};

#ifdef __cplusplus
extern "C" {
#endif

enum servo_channel {pan_channel = 1, tilt_channel = 0};
enum servo_type	{pan = 1, tilt = 0};

int servoio_set_pulse(int id, int value);
int servoio_get_position(int id);
int servoio_init(void);
#ifdef __cplusplus
}
#endif

#endif
