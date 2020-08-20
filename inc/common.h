#ifndef _COMMON_H_
#define _COMMON_H_

#include <sys/time.h>

#if (_MSC_VER == 1700)
#include "OpenCV.h"
#elif (_MSC_VER == 1500)
#include "OpenCV-2008.h"
#else
#include "OpenCV.h"
#endif

typedef unsigned long DWORD;
typedef int BOOL;
typedef unsigned char BYTE;
typedef unsigned short WORD;
typedef float FLOAT;
typedef FLOAT *PFLOAT;
//typedef BOOL near *PBOOL;
//typedef BOOL far *LPBOOL;
//typedef BYTE near *PBYTE;
//typedef BYTE far *LPBYTE;
//typedef int near *PINT;
//typedef int far *LPINT;
//typedef WORD near *PWORD;
//typedef WORD far *LPWORD;
//typedef long far *LPLONG;
//typedef DWORD near *PDWORD;
//typedef DWORD far *LPDWORD;
//typedef void far *LPVOID;
//typedef CONST void far *LPCVOID;

typedef int INT;
typedef unsigned int UINT;
typedef unsigned int *PUINT;

extern struct timeval time_a, time_b;
extern double time_c;
extern double time_sum;
extern int time_cnt;

#define TIME_ST \
{ \
	gettimeofday(&time_a, NULL); \
}

#define TIME_ED \
{ \
	gettimeofday(&time_b, NULL); \
	time_c = 1000000*(time_b.tv_sec-time_a.tv_sec)+time_b.tv_usec-time_a.tv_usec; \
	if (time_c > 1000) { \
		time_sum += time_c / 1000; \
		time_cnt++; \
		printf("test time is %.6f ms;\t", time_c / 1000 ); \
		printf("avge time is %.6f[%.6f][%d] ms\n", time_sum / time_cnt, time_sum, time_cnt); \
		fflush(stdout); \
	} \
}

#endif
