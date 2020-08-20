#ifndef CCTV_HPP
#define CCTV_HPP

#include "stdio.h"
#include "stdlib.h"
#include "vector"
#include "string"
#include "string.h"
//#include "OpenCV.h"
#define CT_SWAP(a,b,t) ((t) = (a), (a) = (b), (b) = (t))
#define CT_PI 3.1415926535897932384626433832795

#define BitRev(i,shift) \
   ((int)((((unsigned)bitrevTab[(i)&255] << 24)+ \
           ((unsigned)bitrevTab[((i)>> 8)&255] << 16)+ \
           ((unsigned)bitrevTab[((i)>>16)&255] <<  8)+ \
           ((unsigned)bitrevTab[((i)>>24)])) >> (shift)))
typedef unsigned char uchar;

typedef struct CtRect
{
    int x, y, width, height;
} CtRect;

typedef struct CtSize
{
    int width, height;
} CtSize;

typedef struct CtPoint
{
	int x, y;
} CtPoint;

typedef struct CtPointF
{
	float x, y;
} CtPointF;

typedef struct CtRectF
{
	float x, y, width, height;
} CtRectF;

typedef struct CtMat
{
    int type;
    int step;

    int* refcount;
    int hdr_refcount;

    union
    {
        uchar* ptr;
        short* s;
        int* i;
        float* fl;
        double* db;
    } data;

#ifdef __cplusplus
    union
    {
        int rows;
        int height;
    };

    union
    {
        int cols;
        int width;
    };
#else
    int rows;
    int cols;
#endif

} CtMat;

typedef struct CtLSVMFeatureMapCaskade{
    int sizeX;
    int sizeY;
    int numFeatures;
    float *map;
} CtLSVMFeatureMapCaskade;


int ctMulMat_ch1(CtMat *mat1, CtMat *mat2);
int ctMulMat_ch2_conj(CtMat *mat1, CtMat *mat2);
int ctPlus_ch1(CtMat *mat1, CtMat *mat2);
int ctPlus_ch2(CtMat *mat1, CtMat *mat2);
float ctSum_mul_ch1(CtMat *mat1);
int ctPlus_const_ch1(CtMat *mat1, float val);
int ctMul_ch2(CtMat *mat, float val);
int ctMul_ch1(CtMat *mat, float val);
int ctPlus_const_ch2(CtMat *mat1, float val);
int ctMax_ch1(CtMat *mat1, float val);
int ctExp_ch1(CtMat *mat1);

//int ctReadMat_char(CtMat *mat, int x1, int y1, int width, int height);
//int ctReadMat_float(CtMat *mat, int x1, int y1, int width, int height);

#endif
