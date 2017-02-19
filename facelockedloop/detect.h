#ifndef __DETECT_H_
#define __DETECT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "pipeline.h"

#if defined(HAVE_OPENCV2)
#include "highgui/highgui_c.h"
#endif
  
enum object_detector_t {
	CDT_HAAR = 0,
	CDT_UNKNOWN = 1,
};

#if defined(HAVE_OPENCV2)

struct detector_params {
	CvMemStorage* scratchbuf;
	IplImage* srcframe;
	IplImage* dstframe;
	enum object_detector_t odt;
	struct store_box *faceboxs;
	char *cascade_xml;
	void *algorithm;
	int min_size;
	int max_size;
};

#else
struct detector_params {
	enum object_detector_t odt;
	struct store_box *faceboxs;
	char *cascade_xml;
	void *scratchbuf;
	void *algorithm;
	void* srcframe;
	void* dstframe;
	int min_size;
	int max_size;
};

#endif

struct detector {
	struct stage step;
	struct detector_params params;
	int status;
};
  
int detect_initialize(struct detector *d, struct detector_params *p, struct pipeline *pipe);

#ifdef __cplusplus
}
#endif

#endif  /* __DETECT_H_ */
