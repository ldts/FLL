#ifndef __STORE_H_
#define __STORE_H_

#ifdef __cplusplus
extern "C" {
#endif
  
#include <time.h>

struct store_box {
	/* if there is no coordinates, request a scan*/
	int scan;
	/* boundix box coordinates */
	int ptA_x;
	int ptA_y;
	int ptB_x;
	int ptB_y;
};

struct facepos {
	struct timeval timestamp;
	struct store_box box;
};

#ifdef __cplusplus
}
#endif

#endif
