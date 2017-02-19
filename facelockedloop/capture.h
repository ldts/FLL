#ifndef __CAPTURE_H_
#define __CAPTURE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "pipeline.h"

#if defined(HAVE_OPENCV2)
#include "highgui/highgui_c.h"

struct imager_params {
	char* name;
	int vididx;
	int frameidx;
	IplImage* frame;
	CvCapture* videocam;
};

#else
struct imager_params {
	char *name];
	int vididx;
	int frameidx;
	void* frame;
	void* videocam;
};

#endif

struct imager {
	struct stage step;
	struct imager_params params;
	int status;
};

struct pipeline;
  
int capture_initialize(struct imager *i, struct imager_params *p, struct pipeline *pipe);
#ifdef __cplusplus
}
#endif

#endif
