#ifndef __TRACK_H_
#define __TRACK_H_

#include "pipeline.h"
#include "servolib.h"
#include "store.h"

#ifdef __cplusplus
extern "C" {
#endif

struct tracker_params {
	int dev;
	int pan_tgt;
	int tilt_tgt;
	struct servo_params pan_params;
	struct servo_params tilt_params;
	struct store_box *bbox;
};

struct tracker {
	struct stage step;
	struct tracker_params params;
	int status;
};

int track_initialize(struct tracker *t, struct tracker_params *p, struct pipeline *pipe);

#ifdef __cplusplus
}
#endif

#endif /* __TRACK_H_ */
