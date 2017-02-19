/**
 * @file facelockedloop/capture.c
 * @brief Frame capture stage, based on OpenCV libraries.
 * 
 * @author Raquel Medina <raquel.medina.rodriguez@gmail.com>
 *
 */
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include "kernel_utils.h"
#include "capture.h"

static
void capture_stage_up(struct stage *stg, struct stage_params *p,
			     struct stage_ops *o,struct pipeline *pipe)
{
	stage_up(stg, p, o, pipe);
	pipeline_register(pipe, stg);
}

static
void capture_teardown(struct imager *i)
{
	cvDestroyWindow(i->params.name);
	cvReleaseCapture(&i->params.videocam);
}

static
void capture_stage_down(struct stage *stg)
{
	struct imager *imgr = container_of(stg, struct imager, step);

	capture_teardown(imgr);
	stage_down(stg);
	pipeline_deregister(stg->pipeline, stg);
}

static
int capture_run(struct imager *i)
{
	IplImage *srcframe;

	if (i->params.vididx < 0)
		return -EINVAL;

	if (!(i->params.videocam))
		return -ENODEV;

	if (cvGrabFrame(i->params.videocam)) {
		srcframe = cvRetrieveFrame(i->params.videocam, 0);
		if (!srcframe)
			return -EIO;

		i->params.frame = srcframe;
		cvWaitKey(10);
	}

	return 0;
}

static
int capture_stage_run(struct stage *stg)
{
	struct imager *imgr;
	int ret;
	imgr = container_of(stg, struct imager, step);
	if (!imgr)
		return -EINVAL;

	ret = capture_run(imgr);
	stg->params.data_out = imgr->params.frame;

	return ret;
}

static
int capture_stage_output(struct stage *stg, void* it)
{
	return stage_output(stg, stg->params.data_out);
}


static
struct stage_ops capture_ops = {
	.output = capture_stage_output,
	.down = capture_stage_down,
	.run = capture_stage_run,
	.up = capture_stage_up,
	.wait = stage_wait,
	.go = stage_go,
};


int capture_initialize(struct imager *i, struct imager_params *p,
		       struct pipeline *pipe)
{
	struct stage_params stgparams;

	stgparams.nth_stage = CAPTURE_STAGE;
	stgparams.data_out = NULL;
	stgparams.data_in = NULL;

	i->params.vididx = p->vididx;
	i->params.frame = p->frame;
	i->params.name = p->name;
	i->params.frameidx = 0;

	i->params.videocam = cvCreateCameraCapture(CV_CAP_ANY + i->params.vididx);
	if (!(i->params.videocam))
		return -ENODEV;

	p->videocam = i->params.videocam;

#if 0
	/* this requires modifications to the servo algorithms */
	cvSetCaptureProperty(i->params.videocam, CV_CAP_PROP_FRAME_WIDTH, 1280.0);
	cvSetCaptureProperty(i->params.videocam, CV_CAP_PROP_FRAME_HEIGHT, 720.0);
#endif
	cvSetCaptureProperty(i->params.videocam, CV_CAP_PROP_FPS, 30);
	capture_stage_up(&i->step, &stgparams, &capture_ops, pipe);

	return 0;
}

