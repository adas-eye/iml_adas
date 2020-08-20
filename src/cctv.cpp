#ifndef CCTV_CPP
#define CCTV_CPP

#include "cctv.hpp"
#include <math.h>
/*
type: [1]CV_8U [2]CV_32F [3]CV_32FC2
*/

CtMat *ctGenMat( int rows, int cols, int type, float* data)
{
    CtMat *m = (CtMat*)malloc(sizeof(CtMat));
    m->cols = cols;
    m->rows = rows;
	int type_size = 0;
	if (type == 3) {
		type_size = 4;
		m->type = 1111638021;
	}
	m->step = m->cols * type_size;
    m->data.ptr = (uchar*)data;
    m->refcount = NULL;
    m->hdr_refcount = 0;
    return m;
}

CtMat *ctCreateMat(int rows, int cols, int type)
{
	CtMat *img = (CtMat*)malloc(sizeof(CtMat));
	img->rows = rows;
	img->cols = cols;
	int type_size = 0;
	if (type == 1) {
		type_size = 1;
		img->type = 1111638016;
	} else if (type == 2) {
		type_size = 4;
		img->type = 1111638021;
	} else if (type == 3) {
		type_size = 8;
		img->type = 1111638029;
	}
	img->step = cols * type_size;
	img->data.ptr = (uchar*)malloc(rows * img->step);
    img->refcount = NULL;
    img->hdr_refcount = 0;
	return img;
}

CtMat *ctCloneMat(CtMat *img)
{
	CtMat *dst = (CtMat*)malloc(sizeof(CtMat));
	dst->rows = img->rows;
	dst->cols = img->cols;
	dst->type = img->type;
	dst->step = img->step;
	dst->refcount = img->refcount;
	dst->hdr_refcount = img->hdr_refcount;
	dst->data.ptr = (uchar*)malloc(img->rows * img->step);
	memcpy(dst->data.ptr, img->data.ptr, img->rows * img->step);
	return dst;
}

int ctMul_ch1(CtMat *mat, float val)
{
	//float *p = (float*)(mat->data.fl);
	for (int ii = 0; ii < mat->rows; ii++) {
		float* p = (float*)(mat->data.ptr + ii * mat->step);
		for (int jj = 0; jj < mat->cols; jj++) {
			p[jj] *= val;
		}
		//p += mat->step / 4;
	}
	return 0;
}

int ctMul_ch2(CtMat *mat, float val)
{
	//float *p = mat->data.fl;
	for (int ii = 0; ii < mat->rows; ii++) {
		float* p = (float*)(mat->data.ptr + ii * mat->step);
		for (int jj = 0; jj < mat->cols; jj++) {
			p[2 * jj] *= val;
			p[2 * jj + 1] *= val;
		}
		//p += mat->step / 4;
	}
	return 0;
}

int ctPlus_ch1(CtMat *mat1, CtMat *mat2)
{
	//float *p1 = (float*)(mat1->data.fl);
	//float *p2 = (float*)(mat2->data.fl);
	for (int ii = 0; ii < mat1->rows; ii++) {
		float* p1 = (float*)(mat1->data.ptr + ii * mat1->step);
		float* p2 = (float*)(mat2->data.ptr + ii * mat2->step);
		for (int jj = 0; jj < mat1->cols; jj++) {
			p1[jj] += p2[jj];
		}
		//p1 += mat1->step / 4;
		//p2 += mat2->step / 4;
	}
	return 0;
}

int ctPlus_ch2(CtMat *mat1, CtMat *mat2)
{
	//float *p1 = (float*)(mat1->data.fl);
	//float *p2 = (float*)(mat2->data.fl);
	for (int ii = 0; ii < mat1->rows; ii++) {
		float* p1 = (float*)(mat1->data.ptr + ii * mat1->step);
		float* p2 = (float*)(mat2->data.ptr + ii * mat2->step);
		for (int jj = 0; jj < mat1->cols; jj++) {
			p1[2 * jj] += p2[2 * jj];
			p1[2 * jj + 1] += p2[2 * jj + 1];
		}
		//p1 += mat1->step / 4;
		//p2 += mat2->step / 4;
	}
	return 0;
}

int ctPlus_const_ch1(CtMat *mat1, float val)
{
	for (int ii = 0; ii < mat1->rows; ii++) {
		float* p1 = (float*)(mat1->data.ptr + ii * mat1->step);
		for (int jj = 0; jj < mat1->cols; jj++) {
			p1[jj] += val;
		}
	}
	return 0;
}

int ctPlus_const_ch2(CtMat *mat1, float val)
{
	for (int ii = 0; ii < mat1->rows; ii++) {
		float* p1 = (float*)(mat1->data.ptr + ii * mat1->step);
		for (int jj = 0; jj < mat1->cols; jj++) {
			p1[2 * jj] += val;
			p1[2 * jj + 1] += val;
		}
	}
	return 0;
}

int ctMulMat_ch1(CtMat *mat1, CtMat *mat2)
{
	for (int ii = 0; ii < mat1->rows; ii++) {
		float* p1 = (float*)(mat1->data.ptr + ii * mat1->step);
		float* p2 = (float*)(mat2->data.ptr + ii * mat2->step);
		for (int jj = 0; jj < mat1->cols; jj++) {
			p1[jj] *= p2[jj];
		}
	}
	return 0;
}

int ctMulMat_ch2_conj(CtMat *mat1, CtMat *mat2)
{
	for (int ii = 0; ii < mat1->rows; ii++) {
		float* p1 = (float*)(mat1->data.ptr + ii * mat1->step);
		float* p2 = (float*)(mat2->data.ptr + ii * mat2->step);
		for (int jj = 0; jj < mat1->cols; jj++) {
			float real = p1[2 * jj] * p2[2 * jj] + p1[2 * jj + 1] * p2[2 * jj + 1];
			p1[2 * jj + 1] = p1[2 * jj + 1] * p2[2 * jj] - p1[2 * jj] * p2[2 * jj + 1];
			p1[2 * jj] = real; 
		}
	}
	return 0;
}

float ctSum_mul_ch1(CtMat *mat1)
{
	float sum = 0;
	for (int ii = 0; ii < mat1->rows; ii++) {
		float* p1 = (float*)(mat1->data.ptr + ii * mat1->step);
		for (int jj = 0; jj < mat1->cols; jj++) {
			sum += p1[jj] * p1[jj];
		}
	}
	return sum;
}

int ctMax_ch1(CtMat *mat1, float val)
{
	for (int ii = 0; ii < mat1->rows; ii++) {
		float* p1 = (float*)(mat1->data.ptr + ii * mat1->step);
		for (int jj = 0; jj < mat1->cols; jj++) {
			if (p1[jj] < val) {
				p1[jj] = val;
			}
		}
	}
	return 0;
}

int ctExp_ch1(CtMat *mat1)
{
	for (int ii = 0; ii < mat1->rows; ii++) {
		float* p1 = (float*)(mat1->data.ptr + ii * mat1->step);
		for (int jj = 0; jj < mat1->cols; jj++) {
			p1[jj] = exp(p1[jj]);
		}
	}
	return 0;
}
int ctdft(CtMat *src, CtMat *dst, int flags);

#endif