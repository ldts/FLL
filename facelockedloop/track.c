#include <termios.h>
#include <fcntl.h>
#include <netdb.h>
#include <errno.h>
#include <stdio.h>
#include <malloc.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "kernel_utils.h"
#include "time_utils.h"
#include "servolib.h"
#include "track.h"

#define FRAME_WIDTH	680
#define FRAME_HEIGHT	480

static sem_t lock;

static
void track_stage_up(struct stage *stg, struct stage_params *p,
			     struct stage_ops *o,struct pipeline *pipe)
{
	stage_up(stg, p, o, pipe);
	pipeline_register(pipe, stg);
}

static
void track_stage_down(struct stage *stg)
{
 	stage_down(stg);
	pipeline_deregister(stg->pipeline, stg);
}

static
int track_stage_input(struct stage *stg, void **it)
{
	struct tracker *tracer  = container_of(stg, struct tracker, step);
	void *itin;

	stage_input(stg, &itin);
	tracer->params.bbox  = (struct store_box*) itin;

	return 0;
}

/**
 * the _extremely_ simple servo decision algorithms:
 */
static
int process_pan(int cpos, int delta, int bbox_center, int middle)
{
	static int last_pan_delta = 0;
	int duty;

	if ( (last_pan_delta == delta) || delta < 50)  {
		/* motor still moving or distance not significant */
		printf("\tkeep current\n");
		return cpos;
	}

	last_pan_delta = delta;

	switch (delta) {
	case 51 ... 100:
		duty = 5; break;
	case 101 ... 200:
		duty = 10; break;
	case 201 ... FRAME_WIDTH/2:
		duty = 15; break;
	}

	if (bbox_center > middle) {
		duty = duty * (-1);
		printf("\tback\t[%3d, %3d]\n", cpos + duty, servoio_get_position(tilt_channel));

	} else
		printf("\tforward\t[%3d, %3d]\n", cpos + duty, servoio_get_position(tilt_channel));

	return cpos + duty;
}

static
int process_tilt(int cpos, int delta, int bbox_center, int middle)
{
	static int last_tilt_delta = 0;
	int duty;

	if ((last_tilt_delta == delta) || delta < 30)  {
		/* motor still moving or distance not significant */
		printf("\tkeep current\n");
		return cpos;
	}

	last_tilt_delta = delta;

	switch (delta) {
	case 31 ...  140:
		duty = 5;
		break;
	case 141 ... 180:
		duty = 10;
		break;
	case 181 ... FRAME_HEIGHT/2:
		duty = 15;
		 break;
	}

	if (bbox_center < middle) {
		duty = duty * (-1);
		printf("\tback\t[%3d, %3d]\n", servoio_get_position(pan_channel),  cpos + duty);
	} else
		printf("\tforward\t[%3d, %3d]\n", servoio_get_position(pan_channel),  cpos + duty);

	return cpos + duty;
}

static
int next_servo_position(enum servo_type servo, int channel, int bbox_center)
{
	int middle = (servo == pan) ? FRAME_WIDTH/2 : FRAME_HEIGHT/2;
	int delta = abs(bbox_center - middle);
	int cpos;

	printf("move\t%s", (servo == pan) ? "pan" : "tilt");
	cpos = servoio_get_position(channel);
	if (cpos < 0)
		return -EIO;

	if (servo == pan)
		return process_pan(cpos, delta, bbox_center, middle);

	return process_tilt(cpos, delta, bbox_center, middle);
}

static
int bbox_center(int b, int a)
{
	return ((b - a) >> 1) + a;
}

static int
scan_for_targets(struct tracker_params *p)
{
	static struct {
		enum {fwd, bck} direction;
		int skip;
		int step;
		int x;

	} value = {
		.direction = fwd,
		.step = 1,
		.skip = 0,
		};

	/* skip every other frame */
	value.skip = ~value.skip;
	if (value.skip)
		return 0;

	value.x = servoio_get_position(pan_channel);

	if (value.direction == bck) {
		if (value.x - (value.step - 1) > MIN_DUTY)
				value.x -= value.step;
		else {
			value.direction = fwd;
			value.x = MIN_DUTY;
			goto done;
		}
	}

	if (value.direction == fwd) {
		if (value.x + (value.step - 1) < MAX_DUTY)
			value.x += value.step;
		else {
			value.direction = bck;
			value.x = MAX_DUTY;
		}
	}
done:
	printf("search\t[%d, %d]\n", value.x, servoio_get_position(tilt_channel));

	return servoio_set_pulse(pan_channel, value.x);
}

static
int track_run(struct tracker *t)
{
	struct tracker_params *p = &t->params;
	int x, y, npos;
	int ret = 0;

	ret = sem_trywait(&lock);
	if (ret < 0)
		return 0;

	if (p->bbox->scan) {
		/* if a face was not detected after SCAN_WAIT_PERIOD, we initiate a
		 *  camera scan sequence looking for targets: the camera will
		 *  autonomously move to the left and right
		 */
		ret = scan_for_targets(p);
		goto done;
	}

	/* a face was detected, now track it so it remains at the center of the screen */
	x = bbox_center(p->bbox->ptB_x, p->bbox->ptA_x);
	npos = next_servo_position(pan, p->pan_params.channel, x);
	ret = servoio_set_pulse(p->pan_params.channel, npos);
	if (ret < 0)
		goto done;

	y = bbox_center(p->bbox->ptB_y, p->bbox->ptA_y);
	npos = next_servo_position(tilt, p->tilt_params.channel, y);
	ret = servoio_set_pulse(p->tilt_params.channel, npos);
	if (ret < 0)
		goto done;
done:
	free(p->bbox);
	sem_post(&lock);

	return ret;
}

static
int track_stage_run(struct stage *stg)
{
	struct tracker *tracer = container_of(stg, struct tracker, step);
	static long last = 0;
	struct timespec spec;
	long current;

	if (!tracer)
		return -EINVAL;

	/* 650 msecs between motor moves:
	 *
	 * IMPORTANT
	 * ---------
	 *
	 *  This value is a physical limit:
	 *       - latency from motor request to pwm signal reaching the motor
	 *       - latency for motor to reach its final position
	 * This is required because we dont have a mechanism to read the motor
	 * position with precision.
	 *
	 * This makes the video display lag but unfortunately cant do any better
	 * with GPIO based PWM and slow motors
	 *
	 */
	clock_gettime(CLOCK_REALTIME, &spec);
	current = timespec_msecs(&spec);
	if (current < last)
		return 0;

	last = timespec_msecs(&spec) + 650;

	return track_run(tracer);
}

static
void clear_screen(void)
{
	const char* cmd = "\e[1;1H\e[2J";

	write(2, cmd, strlen(cmd));
}

static
void print_config(void)
{
	printf("Use the UP/DOWN cursor keys to calibrate the servos\n");
	printf("any other key to exit\n");
	printf("\tpan  :\t\t%3d\n", servoio_get_position(pan_channel));
	printf("\ttilt :\t\t%3d\n", servoio_get_position(tilt_channel));
}

static
void *servo_ctrl(void *cookie)
{
	int id, duty, locked = 0;
	char c;

	for (;;) {
		clear_screen();
		print_config();

		c = kbhit_irq();
		if (!locked && (c == 'A' || c == 'B' || c == 'C' || c == 'D')) {
			locked = 1;
			sem_wait(&lock);
		}
		else if (locked && (c == 'X')) {
			sem_post(&lock);
			locked = 0;
			continue;
		}

		if (c == 'C' || c == 'D') {
			id = pan_channel;
		} else {
			id = tilt_channel;
		}

		duty = servoio_get_position(id);

		switch(c) {
		case 'A':
		case 'C':
			if (duty < MAX_DUTY)
				servoio_set_pulse(id, duty + 1);
			break;
		case 'B':
		case 'D':
			if (duty > MIN_DUTY)
				servoio_set_pulse(id, duty - 1);
			break;
		}
	}

	return NULL;
}

static
int setup_sched_parameters(pthread_attr_t *attr, int prio)
{
	struct sched_param p;
	int ret;

	ret = pthread_attr_init(attr);
	if (ret)
		return -1;

	ret = pthread_attr_setinheritsched(attr, PTHREAD_EXPLICIT_SCHED);
	if (ret)
		return -1;

	ret = pthread_attr_setschedpolicy(attr, prio ? SCHED_FIFO : SCHED_OTHER);
	if (ret)
		return -1;

	p.sched_priority = prio;
	ret = pthread_attr_setschedparam(attr, &p);
	if (ret)
		return -1;

	return 0;
}

static
struct stage_ops track_ops = {
	.input = track_stage_input,
	.down = track_stage_down,
	.run = track_stage_run,
	.up = track_stage_up,
	.wait = stage_wait,
	.go = stage_go,

};

int track_initialize(struct tracker *t, struct tracker_params *p, struct pipeline *pipe)
{
  	struct stage_params stgparams = {
		.nth_stage = TRACKING_STAGE,
		.data_in = NULL,
		.data_out = NULL,
	};
	pthread_attr_t tattr;
	pthread_t ctrl;
	int ret;

	ret = servoio_init();
	if (ret) {
		printf("failed to initialize the servo io\n");
		return -EIO;
	}

	ret = sem_init(&lock, 0, 1);
	if (ret < 0) {
		printf("track: failed to create lock\n");
		return -EIO;
	}

	ret = setup_sched_parameters(&tattr, 0);
	if (ret) {
		printf("track: failed to set control task attr\n");
		return -EIO;
	}

	/* control thread to manually drive the camera stopping the
	 *  pipeline while doing it
	 */
	ret = pthread_create(&ctrl, &tattr, servo_ctrl, NULL);
	if (ret) {
		printf("track: failed to create control task\n");
		return -EIO;
	}

	t->params = *p;
	track_stage_up(&t->step, &stgparams, &track_ops, pipe);

	return ret;
}

