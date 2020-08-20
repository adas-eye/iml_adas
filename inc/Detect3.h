#ifndef DETECT3_H
#define DETECT3_H

//#include "cctv.h"
#include "ctdft.hpp"
#include <stdint.h>
#include <vector>

typedef struct CtTracker_t {
	int is_tracking;
	int frames;
	CtRectF last_rect;
} CtTracker_t;


#define NUM_SEG 9

//const int img_cols = 752;
//const int img_rows = 480;
#define IMG_COLS 752
#define IMG_ROWS 480

#define MAT_NUM	 20
#define F1_NUM	 1

#define F1_LEN	IMG_ROWS * IMG_COLS << 3
//const int f1_len = img_rows * img_cols << 8;

#define F2_NUM	2
#define F2_LEN	IMG_ROWS * IMG_COLS << 2

#define F3_NUM	2
#define F3_LEN	IMG_ROWS * IMG_COLS << 1

#define F4_NUM	4
#define F4_LEN	IMG_ROWS * IMG_COLS

#define F5_NUM	8
#define F5_LEN	IMG_ROWS * IMG_COLS >> 1

#define F6_NUM	8
#define F6_LEN	IMG_ROWS * IMG_COLS >> 2

int tracker_create();
int tracker_release();
int tracker_init(const CtRectF &roi, CtMat *image);
int tracker_update(CtMat *image, CtRectF *rect);
int tracker_check(CtMat *image, CtRect *rect, int index);
int tracker_check(std::vector<float> &val_vec);
void cv_LoadCvMat(
                      CtMat       *pFilter,
//                      int32_t     nType,
//                      uint32_t    nRows,
//                      uint32_t    nCols,
                      void        *pMask
                      );
#endif
