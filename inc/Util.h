#ifndef _UTIL_H_
#define _UTIL_H_

#include<stdlib.h>
#include <string.h>
#include <sys/types.h>
#include "Haar.h"
#include "common.h"
#define LONG long

//#define MAX(a,b)    a > b ? a : b
#define calc_sum(rect,offset) \
((rect).p0[offset] - (rect).p1[offset] - (rect).p2[offset] + (rect).p3[offset])

typedef struct tagBITMAPFILEHEADER_U { 
	WORD    bfType; 
	DWORD   bfSize; 
	WORD    bfReserved1; 
	WORD    bfReserved2; 
	DWORD   bfOffBits; 
} BITMAPFILEHEADER_U;

typedef struct tagBITMAPINFOHEADER_U{ 
	DWORD      biSize; 
	LONG        biWidth; 
	LONG        biHeight; 
	WORD       biPlanes; 
	WORD       biBitCount; 
	DWORD      biCompression; 
	DWORD      biSizeImage; 
	LONG        biXPelsPerMeter; 
	LONG        biYPelsPerMeter; 
	DWORD      biClrUsed; 
	DWORD      biClrImportant; 
} BITMAPINFOHEADER_U;

typedef struct tagRGBQUAD_U { 
	BYTE    rgbBlue; 
	BYTE    rgbGreen; 
	BYTE    rgbRed; 
	BYTE    rgbReserved; 
} RGBQUAD_U;

extern void OutputResult(Image &img, int index);

extern int LoadTrueBitmapFile(const char * FileName,unsigned char *buf, int bufsize,int& X,int&Y);


extern int WriteBmp24bit(const char *fn,unsigned char *pImg,int w,int h);

//extern int WriteRawData(unsigned char *data,char *out_path);

extern void CvtRGB2GRAY(unsigned char* src ,unsigned char* dst , int X,int Y);

extern int WriteBmp24bit_Gray(const char *fn,unsigned char *pImg,int w,int h);

extern void WriteSumAndSqSumToFile(const Mat32 sum, const Mat64 sqsum);

extern void WriteCascadeToFile(HidHaarClassifierCascade* hidCascade);

extern void WriteMaskRowToFile(Image tmp);
//extern void Delay(int nDelay)
/*
extern HidHaarStageClassifier HidStages[MAXSTAGES];
extern HidHaarClassifier HidClassifiers[MAXSTAGES][MAXCLASSIFER];
extern HidHaarClassifierCascade HidCascade;
extern HidHaarTreeNode HidTreeNode[MAXSTAGES][MAXCLASSIFER][MAXTREENODE];
extern int	Alpha[MAXSTAGES][MAXCLASSIFER][MAXALPHA];*/
#endif
