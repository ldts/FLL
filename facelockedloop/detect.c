/**
 * @file facelockedloop/detect.c
 * @brief Face detection stage, based on OpenCV libraries.
 *
 * @author Raquel Medina <raquel.medina.rodriguez@gmail.com>
 *
 */
#include <errno.h>
#include <stdio.h>
#include <malloc.h>
#include <unistd.h>

#include "kernel_utils.h"
#include "time_utils.h"
#include "detect.h"
#include "store.h"

#include "objdetect/objdetect.hpp"
#include "highgui/highgui_c.h"
#include "imgproc/imgproc_c.h"

static
void detect_stage_up(struct stage *stg, struct stage_params *p,
		     struct stage_ops *o,struct pipeline *pipe)
{
	stage_up(stg, p, o, pipe);
	pipeline_register(pipe, stg);
}

static
int detect_stage_output(struct stage *stg, void* it)
{
	return stage_output(stg, stg->params.data_out);
}

static
int detect_stage_input(struct stage *stg, void **it)
{
	void *itin = NULL;
	struct detector *algo;

	algo = container_of(stg, struct detector, step);
	stage_input(stg, &itin);

	algo->params.srcframe = itin;
	algo->params.faceboxs = NULL;

	if (!algo->params.scratchbuf)
		return -EINVAL;

	return 0;
}

static
void detect_teardown(struct detector *d)
{
	cvDestroyWindow("FLL detection");

	if (d->params.dstframe)
		cvReleaseImage(&(d->params.dstframe));

	if (d->params.scratchbuf)
		cvReleaseMemStorage(&(d->params.scratchbuf));
}

static
void detect_stage_down(struct stage *stg)
{
	struct detector *algo;

	algo = container_of(stg, struct detector, step);
	detect_teardown(algo);
	stage_down(stg);
	pipeline_deregister(stg->pipeline, stg);
}

static
struct store_box* detect_store(CvSeq* faces, IplImage* img, int scale)
{
	struct store_box *bbpos;
	CvPoint ptA, ptB;
	int nbbox, i;
	CvFont font;
	char *text;

	nbbox = faces->total ? faces->total : 1;
	bbpos = calloc(nbbox, sizeof(*bbpos));
	if (!bbpos)
		return NULL;

	if (!faces || !faces->total) {
		bbpos->scan = 1;
		goto done;
	}

	cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1.0, 1.0, 0, 1, 8);

	for (i = 0; i < faces->total; i++) {
		CvRect* rAB = (CvRect*)cvGetSeqElem(faces, i);
		ptA.x = rAB->x * scale;
		ptB.x = (rAB->x + rAB->width)*scale;
		ptA.y = rAB->y*scale;
		ptB.y = (rAB->y+rAB->height)*scale;
		cvRectangle(img, ptA, ptB, CV_RGB(0,255,0), 2, 5, 0 );

		bbpos[i].ptA_x = ptA.x;
		bbpos[i].ptA_y = ptA.y;
		bbpos[i].ptB_x = ptB.x;
		bbpos[i].ptB_y = ptB.y;

		asprintf(&text, "detected: %dx%d", rAB->width, rAB->height);
		ptB.y += 15;
		ptB.x = ptA.x;
		cvPutText(img, text, ptB, &font, CV_RGB(0,255,0));

		free(text);
	}
done:
	return bbpos;
}

static
int detect_run(struct detector *d)
{
	CvSeq* faces;

	if (!d->params.dstframe) {
		printf("allocate gray image only once\n");
		d->params.dstframe = cvCreateImage(cvSize(d->params.srcframe->width, d->params.srcframe->height),
							  d->params.srcframe->depth, 1);
		if (!d->params.dstframe)
			return -ENOMEM;
	}

	cvCvtColor(d->params.srcframe, d->params.dstframe, CV_BGR2GRAY);
	cvClearMemStorage(d->params.scratchbuf);
	faces = cvHaarDetectObjects(d->params.dstframe,
		(CvHaarClassifierCascade*)(d->params.algorithm),
		d->params.scratchbuf,
		1.2, /* default scale factor: 1.1 */
		2,   /* default min neighbours: 3 */
		CV_HAAR_DO_CANNY_PRUNING | CV_HAAR_FIND_BIGGEST_OBJECT,
		cvSize(d->params.min_size,d->params.min_size),
		cvSize(d->params.max_size,d->params.max_size) );

	d->params.faceboxs = detect_store(faces, d->params.srcframe, 1);
	if (!d->params.faceboxs)
		return -ENOMEM;

	cvShowImage("FLL detection", (CvArr*)(d->params.srcframe));
	cvWaitKey(5);

	return 0;
}

static
int detect_stage_run(struct stage *stg)
{
	struct detector *algo;
	int ret;

	algo = container_of(stg, struct detector, step);
	if (!algo)
		return -EINVAL;

	ret = detect_run(algo);

	/* pass only first face detected to next stage */
	stg->params.data_out = algo->params.faceboxs;

	return ret;
}

static
struct stage_ops detect_ops = {
	.output = detect_stage_output,
	.input = detect_stage_input,
	.down = detect_stage_down,
	.run = detect_stage_run,
	.up = detect_stage_up,
	.wait = stage_wait,
	.go = stage_go,
};

int detect_initialize(struct detector *d, struct detector_params *p,
		      struct pipeline *pipe)
{
	struct stage_params stgparams;
	int ret = 0;

	if (access(p->cascade_xml, F_OK) || access(p->cascade_xml, R_OK)) {
		printf("error: can't open %s\n", p->cascade_xml);
		return -ENOENT;
	}

	stgparams.nth_stage = DETECTION_STAGE;
	stgparams.data_out = NULL;
	stgparams.data_in = NULL;
	d->params = *p;

	cvNamedWindow("FLL detection", CV_WINDOW_AUTOSIZE);

	d->params.scratchbuf = cvCreateMemStorage(0);
	if (d->params.scratchbuf == NULL)
		return -ENOMEM;

	d->params.algorithm = (void*) cvLoad(d->params.cascade_xml, 0, 0, 0 );
	if (!d->params.algorithm)
		return -ENOENT;


	detect_stage_up(&d->step, &stgparams, &detect_ops, pipe);

	return ret;
}

