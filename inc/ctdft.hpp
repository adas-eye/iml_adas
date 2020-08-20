#ifndef CT_DFT_HPP_
#define CT_DFT_HPP_

#include "cctv.hpp"

#define CT_SWAP(a,b,t) ((t) = (a), (a) = (b), (b) = (t))
#define CT_PI 3.1415926535897932384626433832795

class ctComplexf
{
public:
	ctComplexf() {};
	ctComplexf(float _re, float _im = 0) {re = _re; im = _im;};
	float re, im;
};

class ctComplexd
{
public:
	ctComplexd() {};
	ctComplexd(double _re, double _im = 0) {re = _re; im = _im;};
	double re, im;
};

//static int DFTFactorize(int n, int* factors);
//static void DFTInit(int n0, int nf, int* factors, int* itab, int elem_size, void* _wave, int inv_itab);
//static void DFT_32f(const ctComplexf* src, ctComplexf* dst, int n, int nf, const int* factors, const int* itab, const ctComplexf* wave, int tab_size, const void* spec, ctComplexf* buf, int flags, double _scale);
//static void CopyColumn(const uchar* _src, size_t src_step, uchar* _dst, size_t dst_step, int len, size_t elem_size);
//static void CopyFrom2Columns(const uchar* _src, size_t src_step, uchar* _dst0, uchar* _dst1, int len, size_t elem_size);
//static void CopyTo2Columns(const uchar* _src0, const uchar* _src1, uchar* _dst, size_t dst_step, int len, size_t elem_size);

 int DFTFactorize(int n, int* factors);
 void DFTInit(int n0, int nf, int* factors, int* itab, int elem_size, void* _wave, int inv_itab);
 void DFT_32f(const ctComplexf* src, ctComplexf* dst, int n, int nf, const int* factors, const int* itab, const ctComplexf* wave, int tab_size, const void* spec, ctComplexf* buf, int flags, double _scale);
 void CopyColumn(const uchar* _src, size_t src_step, uchar* _dst, size_t dst_step, int len, size_t elem_size);
 void CopyFrom2Columns(const uchar* _src, size_t src_step, uchar* _dst0, uchar* _dst1, int len, size_t elem_size);
 void CopyTo2Columns(const uchar* _src0, const uchar* _src1, uchar* _dst, size_t dst_step, int len, size_t elem_size);

void swap(int & __restrict a, int & __restrict b);

#define BitRev(i,shift) \
   ((int)((((unsigned)bitrevTab[(i)&255] << 24)+ \
           ((unsigned)bitrevTab[((i)>> 8)&255] << 16)+ \
           ((unsigned)bitrevTab[((i)>>16)&255] <<  8)+ \
           ((unsigned)bitrevTab[((i)>>24)])) >> (shift)))

enum { DFT_NO_PERMUTE = 256, DFT_COMPLEX_INPUT_OR_OUTPUT = 512 };

int ctdft(CtMat *src, CtMat *dst, int flags);

#endif
