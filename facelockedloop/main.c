/**
 * @file facelockedloop/main.c
 * @brief Application that allows face tracking within a given video stream.
 * 
 * @author Raquel Medina <raquel.medina.rodriguez@gmail.com>
 *
 */
#include <sys/types.h>
#include <time.h>
#include <signal.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <getopt.h>
#include <errno.h>
#include "time_utils.h"
#include "pipeline.h"
#include "capture.h"
#include "detect.h"
#include "track.h"

static struct pipeline fllpipe;

static const struct option options[] = {
	{
#define help_opt	0
		.name = "help",
		.has_arg = 0,
		.flag = NULL,
	},
	{
#define camera_opt      1
		.name = "video",
		.has_arg = 1,
		.flag = NULL,
	},
	{
#define trackdev_opt	2
		.name = "servodev",
		.has_arg = 1,
		.flag = NULL,
	},
	{
#define dmins_opt	3
		.name = "min_s",
		.has_arg = 1,
		.flag = NULL,
	},
	{
#define dmaxs_opt	4
		.name = "max_s",
		.has_arg = 1,
		.flag = NULL,
	},
};

static
void usage(void)
{
	fprintf(stderr, "usage: fll  <options>, with:                   \n");
	fprintf(stderr, "            --video[=<camera-index>] 	     "
		":specifies which camera to use (default: any camera)    \n");
	fprintf(stderr, "            --servodevnode=<dev-node-index> "
		":specifies the servos device control node (default: 0)  \n");
	fprintf(stderr, "            --min_s=<n>]                    "
		":specifies min size for the detector (default: 80)     \n");
	fprintf(stderr, "            --max_s=<n>]                    "
		":specifies max size for the detector (default: 180)    \n");
	fprintf(stderr, "            --help                          "
		"this help\n");
}

static
void *signal_catch(void *arg)
{
	sigset_t *monitorset = arg;
	int sig;
	
	for (;;) {
		sigwait(monitorset, &sig);
		
		//printf("caught signal %d. Terminate!\n", sig);
		pipeline_terminate(&fllpipe, -EINTR);
	}
	return NULL;
}
	
static
void setup_term_signals(void)
{
	static sigset_t set;
	pthread_attr_t attr;
	pthread_t id;

	sigemptyset(&set);
	sigaddset(&set, SIGTERM);
	sigaddset(&set, SIGHUP);
	sigaddset(&set, SIGINT);
	sigaddset(&set, SIGQUIT);
	pthread_sigmask(SIG_BLOCK, &set, NULL);
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	pthread_create(&id, &attr, signal_catch, &set);
	pthread_attr_destroy(&attr);
}

int main(int argc, char *const argv[])
{
	struct timespec start_time, stop_time, duration;
	struct detector_params algorithm_params;
	struct tracker_params servo_params;
	struct imager_params camera_params;
	struct detector algorithm;
	struct tracker servo;
	struct imager camera;
	int lindex, c, ret, servodevnode;
	int dmins, dmaxs;
	int video = -1;

	/* default config options */
	servodevnode = 0;
	dmins = 100;
	dmaxs = 180;

	for (;;) {
		lindex = -1;
		c = getopt_long_only(argc, argv, "", options, &lindex);
		if (c == EOF)
			break;
		switch (lindex) {
		case help_opt:
			usage();
			exit(0);
		case camera_opt:
			video = atoi(optarg);
			break;
		case trackdev_opt:
			servodevnode = atoi(optarg);
			break;
		case dmins_opt:
			dmins = atoi(optarg);
			break;
		case dmaxs_opt:
			dmaxs = atoi(optarg);
			break;
		default:
			usage();
			exit(1);
		}
	}

	if (video < 0)
		video = 0;

	setup_term_signals();

	/* setup the vide pipeline */
	pipeline_init(&fllpipe);

	/* first stage */
	ret = asprintf(&camera_params.name, "FLL cam%d", video);
	if (ret < 0)
		goto terminate;

	camera_params.videocam = NULL;
	camera_params.vididx = video;
	camera_params.frame = NULL;
	ret = capture_initialize(&camera, &camera_params, &fllpipe);
	if (ret) {
		printf("capture init ret:%d.\n", ret);
		goto terminate;
	}

	/* second stage */
	algorithm_params.cascade_xml = "haarcascade_frontalface_default.xml";
	algorithm_params.scratchbuf = NULL;
	algorithm_params.algorithm = NULL;
	algorithm_params.srcframe = NULL;
	algorithm_params.dstframe = NULL;
	algorithm_params.odt = CDT_HAAR;

	algorithm_params.min_size = dmins;
	algorithm_params.max_size = dmaxs;
	ret = detect_initialize(&algorithm, &algorithm_params, &fllpipe);
	if (ret) {
		printf("detection init ret:%d.\n", ret);
		goto terminate;
	}

	/* third stage */
	servo_params.tilt_params.channel = tilt_channel;
	servo_params.pan_params.channel = pan_channel;
	servo_params.dev = servodevnode;
	servo_params.tilt_tgt = 0;
	servo_params.pan_tgt = 0;
	ret = track_initialize(&servo , &servo_params, &fllpipe);
	if (ret) {
		printf("tracking init ret:%d.\n", ret);
		goto terminate;
	}

	ret = pipeline_getcount(&fllpipe);
	if (ret != PIPELINE_MAX_STAGE) {
		printf("missing stages for fll, only %d present.\n", ret);
		goto terminate;
	}

	/**
	 * execute the video pipeline
	 */
	clock_gettime(CLOCK_MONOTONIC, &start_time);
	for (;;)  {
		ret = pipeline_run(&fllpipe);
		if (ret) {
			printf("cannot run FLL, ret:%d.\n", ret);
			break;
		}

		if (fllpipe.status == STAGE_ABRT)
			break;
	};

	clock_gettime(CLOCK_MONOTONIC, &stop_time);
	timespec_substract(&duration, &stop_time, &start_time);
	printf("duration->  %lds %ldns .\n", duration.tv_sec , duration.tv_nsec);

terminate:
	free(camera_params.name);
	pipeline_teardown(&fllpipe);

	return 0;
}
