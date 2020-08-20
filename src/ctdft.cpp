#ifndef CTDFT_CPP
#define CTDFT_CPP

#include "ctdft.hpp"
#include <math.h>

double DFTTab[][2] =
{
	{ 1.00000000000000000, 0.00000000000000000 },
	{ -1.00000000000000000, 0.00000000000000000 },
	{ 0.00000000000000000, 1.00000000000000000 },
	{ 0.70710678118654757, 0.70710678118654746 },
	{ 0.92387953251128674, 0.38268343236508978 },
	{ 0.98078528040323043, 0.19509032201612825 },
	{ 0.99518472667219693, 0.09801714032956060 },
	{ 0.99879545620517241, 0.04906767432741802 },
	{ 0.99969881869620425, 0.02454122852291229 },
	{ 0.99992470183914450, 0.01227153828571993 },
	{ 0.99998117528260111, 0.00613588464915448 },
	{ 0.99999529380957619, 0.00306795676296598 },
	{ 0.99999882345170188, 0.00153398018628477 },
	{ 0.99999970586288223, 0.00076699031874270 },
	{ 0.99999992646571789, 0.00038349518757140 },
	{ 0.99999998161642933, 0.00019174759731070 },
	{ 0.99999999540410733, 0.00009587379909598 },
	{ 0.99999999885102686, 0.00004793689960307 },
	{ 0.99999999971275666, 0.00002396844980842 },
	{ 0.99999999992818922, 0.00001198422490507 },
	{ 0.99999999998204725, 0.00000599211245264 },
	{ 0.99999999999551181, 0.00000299605622633 },
	{ 0.99999999999887801, 0.00000149802811317 },
	{ 0.99999999999971945, 0.00000074901405658 },
	{ 0.99999999999992983, 0.00000037450702829 },
	{ 0.99999999999998246, 0.00000018725351415 },
	{ 0.99999999999999567, 0.00000009362675707 },
	{ 0.99999999999999889, 0.00000004681337854 },
	{ 0.99999999999999978, 0.00000002340668927 },
	{ 0.99999999999999989, 0.00000001170334463 },
	{ 1.00000000000000000, 0.00000000585167232 },
	{ 1.00000000000000000, 0.00000000292583616 }
};
unsigned char bitrevTab[] =
{
	0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0, 0x10, 0x90, 0x50, 0xd0, 0x30, 0xb0, 0x70, 0xf0,
	0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8, 0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8,
	0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0xe4, 0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4,
	0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec, 0x1c, 0x9c, 0x5c, 0xdc, 0x3c, 0xbc, 0x7c, 0xfc,
	0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2, 0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2,
	0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0xea, 0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa,
	0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6, 0x16, 0x96, 0x56, 0xd6, 0x36, 0xb6, 0x76, 0xf6,
	0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee, 0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe,
	0x01, 0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0xe1, 0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1,
	0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9, 0x19, 0x99, 0x59, 0xd9, 0x39, 0xb9, 0x79, 0xf9,
	0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5, 0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5,
	0x0d, 0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0xed, 0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd,
	0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3, 0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3,
	0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb, 0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb,
	0x07, 0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0xe7, 0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7,
	0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef, 0x1f, 0x9f, 0x5f, 0xdf, 0x3f, 0xbf, 0x7f, 0xff
};
int ctdft(CtMat *src, CtMat *dst, int flags)
{
	int nonzero_rows = 0;
	uchar *buf;
	int prev_len = 0;
	int stage = 0;
	bool inv = (flags & 1) != 0;
	int nf = 0;
	int elem_size = sizeof(float);
	int complex_elem_size = elem_size * 2;
	int factors[34];
	bool inplace_transform = false;
	elem_size = complex_elem_size;
	for (;;) {
		double scale = 1;
		uchar* wave = 0;
		int* itab = 0;
		uchar* ptr;
		int i, len, count, sz = 0;
		int use_buf = 0;

		if (stage == 0) {
			len = !inv ? src->cols : dst->cols;
			count = src->rows;
			if (len == 1) {
				len = !inv ? src->rows : dst->rows;
				count = 1;
			}
		} else {
			len = dst->rows;
			count = !inv ? src->cols : dst->cols;
			sz = 2 * len*complex_elem_size;
		}

		void *spec = 0;

		{
			if (len != prev_len)
				nf = DFTFactorize(len, factors);

			inplace_transform = factors[0] == factors[nf - 1];
			sz += len*(complex_elem_size + sizeof(int));
			i = nf > 1 && (factors[0] & 1) == 0;
			if ((factors[i] & 1) != 0 && factors[i] > 5)
				sz += (factors[i] + 1)*complex_elem_size;

			if ((stage == 0 && !inplace_transform) ||
				(stage == 1 && !inplace_transform)) {
				use_buf = 1;
				sz += len*complex_elem_size;
			}
		}

		buf = (uchar*)malloc(sz + 32);
		ptr = (uchar*)buf;
		if (ptr != (uchar*)buf)
			prev_len = 0;
		ptr = (uchar*)buf;
		if (!spec) {
			wave = ptr;
			ptr += len*complex_elem_size;
			itab = (int*)ptr;
			ptr = (uchar*)(((size_t)(ptr + len*sizeof(int)) + 15) & (-16));

			if (len != prev_len)
				DFTInit(len, nf, factors, itab, complex_elem_size, wave, 0);
		}

		if (stage == 0) {
			uchar* tmp_buf = 0;
			int dptr_offset = 0;
			int dst_full_len = len*elem_size;
			int _flags = (int)inv;

			if (use_buf) {
				tmp_buf = ptr;
				ptr += len*complex_elem_size;
			}

			if (count > 1)
				stage = 1;
			else if (flags)
				scale = 1. / (len * count);

			nonzero_rows = count;
			for (i = 0; i < nonzero_rows; i++) {
				const uchar* sptr = src->data.ptr + i * src->step;
				uchar* dptr0 = dst->data.ptr + i * dst->step;
				uchar* dptr = dptr0;

				if (tmp_buf)
					dptr = tmp_buf;

				DFT_32f(static_cast<const ctComplexf*>((void*)sptr), static_cast<ctComplexf*>((void*)dptr), len, nf, factors, itab, static_cast<const ctComplexf*>((void*)wave), len, spec, static_cast<ctComplexf*>((void*)ptr), _flags, scale);

				if (dptr != dptr0)
					memcpy(dptr0, dptr + dptr_offset, dst_full_len);
			}

			for (; i < count; i++) {
				uchar* dptr0 = dst->data.ptr + i * dst->step;
				memset(dptr0, 0, dst_full_len);
			}

			if (stage != 1) {
				break;
			}

			src = dst;
		} else {
			int a = 0, b = count;
			uchar *buf0, *buf1, *dbuf0, *dbuf1;
			const uchar* sptr0 = src->data.ptr;
			uchar* dptr0 = dst->data.ptr;
			buf0 = ptr;
			ptr += len*complex_elem_size;
			buf1 = ptr;
			ptr += len*complex_elem_size;
			dbuf0 = buf0, dbuf1 = buf1;

			if (use_buf) {
				dbuf1 = ptr;
				dbuf0 = buf1;
				ptr += len*complex_elem_size;
			}

			if (flags)
				scale = 1. / (len * count);

			for (i = a; i < b; i += 2) {
				if (i + 1 < b) {
					CopyFrom2Columns(sptr0, src->step, buf0, buf1, len, complex_elem_size);
					DFT_32f(static_cast<const ctComplexf*>((void*)buf1), static_cast<ctComplexf*>((void*)dbuf1), len, nf, factors, itab, static_cast<const ctComplexf*>((void*)wave), len, spec, static_cast<ctComplexf*>((void*)ptr), inv, scale);
				} else
					CopyColumn(sptr0, src->step, buf0, complex_elem_size, len, complex_elem_size);

				DFT_32f(static_cast<const ctComplexf*>((void*)buf0), static_cast<ctComplexf*>((void*)dbuf0), len, nf, factors, itab, static_cast<const ctComplexf*>((void*)wave), len, spec, static_cast<ctComplexf*>((void*)ptr), inv, scale);

				if (i + 1 < b)
					CopyTo2Columns(dbuf0, dbuf1, dptr0, dst->step, len, complex_elem_size);
				else
					CopyColumn(dbuf0, complex_elem_size, dptr0, dst->step, len, complex_elem_size);
				sptr0 += 2 * complex_elem_size;
				dptr0 += 2 * complex_elem_size;
			}

			if (stage != 0) {
				free(buf);
				break;
			}
			src = dst;
		}
		free(buf);
	}

	return 0;
}

struct DFT_VecR4
{
	int operator()(ctComplexf*, int, int, int&, const ctComplexf*) const { return 1; }
};

 void DFT_32f(const ctComplexf* src, ctComplexf* dst, int n, int nf, const int* factors, const int* itab, const ctComplexf* wave, int tab_size, const void* spec, ctComplexf* buf, int flags, double _scale)
{
	static const float sin_120 = (float)0.86602540378443864676372317075294;
	static const float fft5_2 = (float)0.559016994374947424102293417182819;
	static const float fft5_3 = (float)-0.951056516295153572116439333379382;
	static const float fft5_4 = (float)-1.538841768587626701285145288018455;
	static const float fft5_5 = (float)0.363271264002680442947733378740309;

	int n0 = n, f_idx, nx;
	int inv = flags & 1;
	int dw0 = tab_size, dw;
	int i, j, k;
	ctComplexf t;
	float scale = (float)_scale;
	int tab_step;

	tab_step = tab_size == n ? 1 : tab_size == n * 2 ? 2 : tab_size / n;

	if (dst != src) {
		if (!inv) {
			for (i = 0; i <= n - 2; i += 2, itab += 2 * tab_step) {
				int k0 = itab[0], k1 = itab[tab_step];
				dst[i] = src[k0]; dst[i + 1] = src[k1];
			}

			if (i < n)
				dst[n - 1] = src[n - 1];
		} else {
			for (i = 0; i <= n - 2; i += 2, itab += 2 * tab_step) {
				int k0 = itab[0], k1 = itab[tab_step];
				t.re = src[k0].re; t.im = -src[k0].im;
				dst[i] = t;
				t.re = src[k1].re; t.im = -src[k1].im;
				dst[i + 1] = t;
			}

			if (i < n) {
				t.re = src[n - 1].re; t.im = -src[n - 1].im;
				dst[i] = t;
			}
		}
	} else {
		{
			if (nf == 1) {
				if ((n & 3) == 0) {
					int n2 = n / 2;
					ctComplexf* dsth = dst + n2;

					for (i = 0; i < n2; i += 2, itab += tab_step * 2) {
						j = itab[0];

						CT_SWAP(dst[i + 1], dsth[j], t);
						if (j > i) {
							CT_SWAP(dst[i], dst[j], t);
							CT_SWAP(dsth[i + 1], dsth[j + 1], t);
						}
					}
				}
			} else {
				for (i = 0; i < n; i++, itab += tab_step) {
					j = itab[0];
					if (j > i)
						CT_SWAP(dst[i], dst[j], t);
				}
			}
		}

		if (inv) {
			for (i = 0; i <= n - 2; i += 2) {
				float t0 = -dst[i].im;
				float t1 = -dst[i + 1].im;
				dst[i].im = t0; dst[i + 1].im = t1;
			}

			if (i < n)
				dst[n - 1].im = -dst[n - 1].im;
		}
	}

	n = 1;
	if ((factors[0] & 1) == 0) {
		if (factors[0] >= 4) {
			DFT_VecR4 vr4;
			n = vr4(dst, factors[0], n0, dw0, wave);
		}

		for (; n * 4 <= factors[0];) {
			nx = n;
			n *= 4;
			dw0 /= 4;

			for (i = 0; i < n0; i += n) {
				ctComplexf *v0, *v1;
				float r0, i0, r1, i1, r2, i2, r3, i3, r4, i4;

				v0 = dst + i;
				v1 = v0 + nx * 2;

				r0 = v1[0].re; i0 = v1[0].im;
				r4 = v1[nx].re; i4 = v1[nx].im;

				r1 = r0 + r4; i1 = i0 + i4;
				r3 = i0 - i4; i3 = r4 - r0;

				r2 = v0[0].re; i2 = v0[0].im;
				r4 = v0[nx].re; i4 = v0[nx].im;

				r0 = r2 + r4; i0 = i2 + i4;
				r2 -= r4; i2 -= i4;

				v0[0].re = r0 + r1; v0[0].im = i0 + i1;
				v1[0].re = r0 - r1; v1[0].im = i0 - i1;
				v0[nx].re = r2 + r3; v0[nx].im = i2 + i3;
				v1[nx].re = r2 - r3; v1[nx].im = i2 - i3;

				for (j = 1, dw = dw0; j < nx; j++, dw += dw0) {
					v0 = dst + i + j;
					v1 = v0 + nx * 2;

					r2 = v0[nx].re*wave[dw * 2].re - v0[nx].im*wave[dw * 2].im;
					i2 = v0[nx].re*wave[dw * 2].im + v0[nx].im*wave[dw * 2].re;
					r0 = v1[0].re*wave[dw].im + v1[0].im*wave[dw].re;
					i0 = v1[0].re*wave[dw].re - v1[0].im*wave[dw].im;
					r3 = v1[nx].re*wave[dw * 3].im + v1[nx].im*wave[dw * 3].re;
					i3 = v1[nx].re*wave[dw * 3].re - v1[nx].im*wave[dw * 3].im;

					r1 = i0 + i3; i1 = r0 + r3;
					r3 = r0 - r3; i3 = i3 - i0;
					r4 = v0[0].re; i4 = v0[0].im;

					r0 = r4 + r2; i0 = i4 + i2;
					r2 = r4 - r2; i2 = i4 - i2;

					v0[0].re = r0 + r1; v0[0].im = i0 + i1;
					v1[0].re = r0 - r1; v1[0].im = i0 - i1;
					v0[nx].re = r2 + r3; v0[nx].im = i2 + i3;
					v1[nx].re = r2 - r3; v1[nx].im = i2 - i3;
				}
			}
		}

		for (; n < factors[0];) {
			nx = n;
			n *= 2;
			dw0 /= 2;

			for (i = 0; i < n0; i += n) {
				ctComplexf* v = dst + i;
				float r0 = v[0].re + v[nx].re;
				float i0 = v[0].im + v[nx].im;
				float r1 = v[0].re - v[nx].re;
				float i1 = v[0].im - v[nx].im;
				v[0].re = r0; v[0].im = i0;
				v[nx].re = r1; v[nx].im = i1;

				for (j = 1, dw = dw0; j < nx; j++, dw += dw0) {
					v = dst + i + j;
					r1 = v[nx].re*wave[dw].re - v[nx].im*wave[dw].im;
					i1 = v[nx].im*wave[dw].re + v[nx].re*wave[dw].im;
					r0 = v[0].re; i0 = v[0].im;

					v[0].re = r0 + r1; v[0].im = i0 + i1;
					v[nx].re = r0 - r1; v[nx].im = i0 - i1;
				}
			}
		}
	}

	for (f_idx = (factors[0] & 1) ? 0 : 1; f_idx < nf; f_idx++) {
		int factor = factors[f_idx];
		nx = n;
		n *= factor;
		dw0 /= factor;

		if (factor == 3) {
			for (i = 0; i < n0; i += n) {
				ctComplexf* v = dst + i;

				float r1 = v[nx].re + v[nx * 2].re;
				float i1 = v[nx].im + v[nx * 2].im;
				float r0 = v[0].re;
				float i0 = v[0].im;
				float r2 = sin_120*(v[nx].im - v[nx * 2].im);
				float i2 = sin_120*(v[nx * 2].re - v[nx].re);
				v[0].re = r0 + r1; v[0].im = i0 + i1;
				r0 -= (float)0.5*r1; i0 -= (float)0.5*i1;
				v[nx].re = r0 + r2; v[nx].im = i0 + i2;
				v[nx * 2].re = r0 - r2; v[nx * 2].im = i0 - i2;

				for (j = 1, dw = dw0; j < nx; j++, dw += dw0) {
					v = dst + i + j;
					r0 = v[nx].re*wave[dw].re - v[nx].im*wave[dw].im;
					i0 = v[nx].re*wave[dw].im + v[nx].im*wave[dw].re;
					i2 = v[nx * 2].re*wave[dw * 2].re - v[nx * 2].im*wave[dw * 2].im;
					r2 = v[nx * 2].re*wave[dw * 2].im + v[nx * 2].im*wave[dw * 2].re;
					r1 = r0 + i2; i1 = i0 + r2;

					r2 = sin_120*(i0 - r2); i2 = sin_120*(i2 - r0);
					r0 = v[0].re; i0 = v[0].im;
					v[0].re = r0 + r1; v[0].im = i0 + i1;
					r0 -= (float)0.5*r1; i0 -= (float)0.5*i1;
					v[nx].re = r0 + r2; v[nx].im = i0 + i2;
					v[nx * 2].re = r0 - r2; v[nx * 2].im = i0 - i2;
				}
			}
		} else if (factor == 5) {
			for (i = 0; i < n0; i += n) {
				for (j = 0, dw = 0; j < nx; j++, dw += dw0) {
					ctComplexf* v0 = dst + i + j;
					ctComplexf* v1 = v0 + nx * 2;
					ctComplexf* v2 = v1 + nx * 2;

					float r0, i0, r1, i1, r2, i2, r3, i3, r4, i4, r5, i5;

					r3 = v0[nx].re*wave[dw].re - v0[nx].im*wave[dw].im;
					i3 = v0[nx].re*wave[dw].im + v0[nx].im*wave[dw].re;
					r2 = v2[0].re*wave[dw * 4].re - v2[0].im*wave[dw * 4].im;
					i2 = v2[0].re*wave[dw * 4].im + v2[0].im*wave[dw * 4].re;

					r1 = r3 + r2; i1 = i3 + i2;
					r3 -= r2; i3 -= i2;

					r4 = v1[nx].re*wave[dw * 3].re - v1[nx].im*wave[dw * 3].im;
					i4 = v1[nx].re*wave[dw * 3].im + v1[nx].im*wave[dw * 3].re;
					r0 = v1[0].re*wave[dw * 2].re - v1[0].im*wave[dw * 2].im;
					i0 = v1[0].re*wave[dw * 2].im + v1[0].im*wave[dw * 2].re;

					r2 = r4 + r0; i2 = i4 + i0;
					r4 -= r0; i4 -= i0;

					r0 = v0[0].re; i0 = v0[0].im;
					r5 = r1 + r2; i5 = i1 + i2;

					v0[0].re = r0 + r5; v0[0].im = i0 + i5;

					r0 -= (float)0.25*r5; i0 -= (float)0.25*i5;
					r1 = fft5_2*(r1 - r2); i1 = fft5_2*(i1 - i2);
					r2 = -fft5_3*(i3 + i4); i2 = fft5_3*(r3 + r4);

					i3 *= -fft5_5; r3 *= fft5_5;
					i4 *= -fft5_4; r4 *= fft5_4;

					r5 = r2 + i3; i5 = i2 + r3;
					r2 -= i4; i2 -= r4;

					r3 = r0 + r1; i3 = i0 + i1;
					r0 -= r1; i0 -= i1;

					v0[nx].re = r3 + r2; v0[nx].im = i3 + i2;
					v2[0].re = r3 - r2; v2[0].im = i3 - i2;

					v1[0].re = r0 + r5; v1[0].im = i0 + i5;
					v1[nx].re = r0 - r5; v1[nx].im = i0 - i5;
				}
			}
		} else {
			int p, q, factor2 = (factor - 1) / 2;
			int d, dd, dw_f = tab_size / factor;
			ctComplexf* a = buf;
			ctComplexf* b = buf + factor2;

			for (i = 0; i < n0; i += n) {
				for (j = 0, dw = 0; j < nx; j++, dw += dw0) {
					ctComplexf* v = dst + i + j;
					ctComplexf v_0 = v[0];
					ctComplexf vn_0 = v_0;

					if (j == 0) {
						for (p = 1, k = nx; p <= factor2; p++, k += nx) {
							float r0 = v[k].re + v[n - k].re;
							float i0 = v[k].im - v[n - k].im;
							float r1 = v[k].re - v[n - k].re;
							float i1 = v[k].im + v[n - k].im;

							vn_0.re += r0; vn_0.im += i1;
							a[p - 1].re = r0; a[p - 1].im = i0;
							b[p - 1].re = r1; b[p - 1].im = i1;
						}
					} else {
						const ctComplexf* wave_ = wave + dw*factor;
						d = dw;

						for (p = 1, k = nx; p <= factor2; p++, k += nx, d += dw) {
							float r2 = v[k].re*wave[d].re - v[k].im*wave[d].im;
							float i2 = v[k].re*wave[d].im + v[k].im*wave[d].re;

							float r1 = v[n - k].re*wave_[-d].re - v[n - k].im*wave_[-d].im;
							float i1 = v[n - k].re*wave_[-d].im + v[n - k].im*wave_[-d].re;

							float r0 = r2 + r1;
							float i0 = i2 - i1;
							r1 = r2 - r1;
							i1 = i2 + i1;

							vn_0.re += r0; vn_0.im += i1;
							a[p - 1].re = r0; a[p - 1].im = i0;
							b[p - 1].re = r1; b[p - 1].im = i1;
						}
					}

					v[0] = vn_0;

					for (p = 1, k = nx; p <= factor2; p++, k += nx) {
						ctComplexf s0 = v_0, s1 = v_0;
						d = dd = dw_f*p;

						for (q = 0; q < factor2; q++) {
							float r0 = wave[d].re * a[q].re;
							float i0 = wave[d].im * a[q].im;
							float r1 = wave[d].re * b[q].im;
							float i1 = wave[d].im * b[q].re;

							s1.re += r0 + i0; s0.re += r0 - i0;
							s1.im += r1 - i1; s0.im += r1 + i1;

							d += dd;
							d -= -(d >= tab_size) & tab_size;
						}

						v[k] = s0;
						v[n - k] = s1;
					}
				}
			}
		}
	}

	if (scale != 1) {
		float re_scale = scale, im_scale = scale;
		if (inv)
			im_scale = -im_scale;

		for (i = 0; i < n0; i++) {
			float t0 = dst[i].re*re_scale;
			float t1 = dst[i].im*im_scale;
			dst[i].re = t0;
			dst[i].im = t1;
		}
	} else if (inv) {
		for (i = 0; i <= n0 - 2; i += 2) {
			float t0 = -dst[i].im;
			float t1 = -dst[i + 1].im;
			dst[i].im = t0;
			dst[i + 1].im = t1;
		}

		if (i < n0)
			dst[n0 - 1].im = -dst[n0 - 1].im;
	}
}

 void CopyColumn(const uchar* _src, size_t src_step, uchar* _dst, size_t dst_step, int len, size_t elem_size)
{
	int i, t0, t1;
	const int* src = (const int*)_src;
	int* dst = (int*)_dst;
	src_step /= sizeof(src[0]);
	dst_step /= sizeof(dst[0]);

	if (elem_size == sizeof(int)) {
		for (i = 0; i < len; i++, src += src_step, dst += dst_step)
			dst[0] = src[0];
	} else if (elem_size == sizeof(int) * 2) {
		for (i = 0; i < len; i++, src += src_step, dst += dst_step) {
			t0 = src[0]; t1 = src[1];
			dst[0] = t0; dst[1] = t1;
		}
	} else if (elem_size == sizeof(int) * 4) {
		for (i = 0; i < len; i++, src += src_step, dst += dst_step) {
			t0 = src[0]; t1 = src[1];
			dst[0] = t0; dst[1] = t1;
			t0 = src[2]; t1 = src[3];
			dst[2] = t0; dst[3] = t1;
		}
	}
}

 void CopyFrom2Columns(const uchar* _src, size_t src_step, uchar* _dst0, uchar* _dst1, int len, size_t elem_size)
{
	int i, t0, t1;
	const int* src = (const int*)_src;
	int* dst0 = (int*)_dst0;
	int* dst1 = (int*)_dst1;
	src_step /= sizeof(src[0]);

	if (elem_size == sizeof(int)) {
		for (i = 0; i < len; i++, src += src_step) {
			t0 = src[0]; t1 = src[1];
			dst0[i] = t0; dst1[i] = t1;
		}
	} else if (elem_size == sizeof(int) * 2) {
		for (i = 0; i < len * 2; i += 2, src += src_step) {
			t0 = src[0]; t1 = src[1];
			dst0[i] = t0; dst0[i + 1] = t1;
			t0 = src[2]; t1 = src[3];
			dst1[i] = t0; dst1[i + 1] = t1;
		}
	} else if (elem_size == sizeof(int) * 4) {
		for (i = 0; i < len * 4; i += 4, src += src_step) {
			t0 = src[0]; t1 = src[1];
			dst0[i] = t0; dst0[i + 1] = t1;
			t0 = src[2]; t1 = src[3];
			dst0[i + 2] = t0; dst0[i + 3] = t1;
			t0 = src[4]; t1 = src[5];
			dst1[i] = t0; dst1[i + 1] = t1;
			t0 = src[6]; t1 = src[7];
			dst1[i + 2] = t0; dst1[i + 3] = t1;
		}
	}
}

 void CopyTo2Columns(const uchar* _src0, const uchar* _src1, uchar* _dst, size_t dst_step, int len, size_t elem_size)
{
	int i, t0, t1;
	const int* src0 = (const int*)_src0;
	const int* src1 = (const int*)_src1;
	int* dst = (int*)_dst;
	dst_step /= sizeof(dst[0]);

	if (elem_size == sizeof(int)) {
		for (i = 0; i < len; i++, dst += dst_step) {
			t0 = src0[i]; t1 = src1[i];
			dst[0] = t0; dst[1] = t1;
		}
	} else if (elem_size == sizeof(int) * 2) {
		for (i = 0; i < len * 2; i += 2, dst += dst_step) {
			t0 = src0[i]; t1 = src0[i + 1];
			dst[0] = t0; dst[1] = t1;
			t0 = src1[i]; t1 = src1[i + 1];
			dst[2] = t0; dst[3] = t1;
		}
	} else if (elem_size == sizeof(int) * 4) {
		for (i = 0; i < len * 4; i += 4, dst += dst_step) {
			t0 = src0[i]; t1 = src0[i + 1];
			dst[0] = t0; dst[1] = t1;
			t0 = src0[i + 2]; t1 = src0[i + 3];
			dst[2] = t0; dst[3] = t1;
			t0 = src1[i]; t1 = src1[i + 1];
			dst[4] = t0; dst[5] = t1;
			t0 = src1[i + 2]; t1 = src1[i + 3];
			dst[6] = t0; dst[7] = t1;
		}
	}
}

 int DFTFactorize(int n, int* factors)
{
	int nf = 0, f, i;

	if (n <= 5) {
		factors[0] = n;
		return 1;
	}

	f = (((n - 1) ^ n) + 1) >> 1;
	if (f > 1) {
		factors[nf++] = f;
		n = f == n ? 1 : n / f;
	}

	for (f = 3; n > 1;) {
		int d = n / f;
		if (d*f == n) {
			factors[nf++] = f;
			n = d;
		} else {
			f += 2;
			if (f*f > n)
				break;
		}
	}

	if (n > 1)
		factors[nf++] = n;

	f = (factors[0] & 1) == 0;
	for (i = f; i < (nf + f) / 2; i++)
		swap(factors[i], factors[nf - i - 1 + f]);

//		std::swap(factors[i], factors[nf - i - 1 + f]);

	return nf;
}


void swap(int & __restrict a, int & __restrict b)
{
	a ^= b;
	b ^= a;
	a ^= b;
}
 void DFTInit(int n0, int nf, int* factors, int* itab, int elem_size, void* _wave, int inv_itab)
{
	int digits[34], radix[34];
	int n = factors[0], m = 0;
	int* itab0 = itab;
	int i, j, k;
	ctComplexd w, w1;
	double t;

	if (n0 <= 5) {
		itab[0] = 0;
		itab[n0 - 1] = n0 - 1;

		if (n0 != 4) {
			for (i = 1; i < n0 - 1; i++)
				itab[i] = i;
		} else {
			itab[1] = 2;
			itab[2] = 1;
		}
		if (n0 == 5) {
			if (elem_size == sizeof(ctComplexd))
				((ctComplexd*)_wave)[0] = ctComplexd(1., 0.);
			else
				((ctComplexf*)_wave)[0] = ctComplexf(1.f, 0.f);
		}
		if (n0 != 4)
			return;
		m = 2;
	} else {
		radix[nf] = 1;
		digits[nf] = 0;
		for (i = 0; i < nf; i++) {
			digits[i] = 0;
			radix[nf - i - 1] = radix[nf - i] * factors[nf - i - 1];
		}

		if (inv_itab && factors[0] != factors[nf - 1])
			itab = (int*)_wave;

		if ((n & 1) == 0) {
			int a = radix[1], na2 = n*a >> 1, na4 = na2 >> 1;
			for (m = 0; (unsigned)(1 << m) < (unsigned)n; m++)
				;
			if (n <= 2) {
				itab[0] = 0;
				itab[1] = na2;
			} else if (n <= 256) {
				int shift = 10 - m;
				for (i = 0; i <= n - 4; i += 4) {
					j = (bitrevTab[i >> 2] >> shift)*a;
					itab[i] = j;
					itab[i + 1] = j + na2;
					itab[i + 2] = j + na4;
					itab[i + 3] = j + na2 + na4;
				}
			} else {
				int shift = 34 - m;
				for (i = 0; i < n; i += 4) {
					int i4 = i >> 2;
					j = BitRev(i4, shift)*a;
					itab[i] = j;
					itab[i + 1] = j + na2;
					itab[i + 2] = j + na4;
					itab[i + 3] = j + na2 + na4;
				}
			}

			digits[1]++;

			if (nf >= 2) {
				for (i = n, j = radix[2]; i < n0;) {
					for (k = 0; k < n; k++)
						itab[i + k] = itab[k] + j;
					if ((i += n) >= n0)
						break;
					j += radix[2];
					for (k = 1; ++digits[k] >= factors[k]; k++) {
						digits[k] = 0;
						j += radix[k + 2] - radix[k];
					}
				}
			}
		} else {
			for (i = 0, j = 0;;) {
				itab[i] = j;
				if (++i >= n0)
					break;
				j += radix[1];
				for (k = 0; ++digits[k] >= factors[k]; k++) {
					digits[k] = 0;
					j += radix[k + 2] - radix[k];
				}
			}
		}

		if (itab != itab0) {
			itab0[0] = 0;
			for (i = n0 & 1; i < n0; i += 2) {
				int k0 = itab[i];
				int k1 = itab[i + 1];
				itab0[k0] = i;
				itab0[k1] = i + 1;
			}
		}
	}

	if ((n0 & (n0 - 1)) == 0) {
		w.re = w1.re = DFTTab[m][0];
		w.im = w1.im = -DFTTab[m][1];
	} else {
		t = -CT_PI * 2 / n0;
		w.im = w1.im = sin(t);
//		w.re = w1.re = std::sqrt(1. - w1.im*w1.im);
		w.re = w1.re = sqrt(1. - w1.im*w1.im);
	}
	n = (n0 + 1) / 2;

	if (elem_size == sizeof(ctComplexd)) {
		ctComplexd* wave = (ctComplexd*)_wave;

		wave[0].re = 1.;
		wave[0].im = 0.;

		if ((n0 & 1) == 0) {
			wave[n].re = -1.;
			wave[n].im = 0;
		}

		for (i = 1; i < n; i++) {
			wave[i] = w;
			wave[n0 - i].re = w.re;
			wave[n0 - i].im = -w.im;

			t = w.re*w1.re - w.im*w1.im;
			w.im = w.re*w1.im + w.im*w1.re;
			w.re = t;
		}
	} else {
		ctComplexf* wave = (ctComplexf*)_wave;

		wave[0].re = 1.f;
		wave[0].im = 0.f;

		if ((n0 & 1) == 0) {
			wave[n].re = -1.f;
			wave[n].im = 0.f;
		}

		for (i = 1; i < n; i++) {
			wave[i].re = (float)w.re;
			wave[i].im = (float)w.im;
			wave[n0 - i].re = (float)w.re;
			wave[n0 - i].im = (float)-w.im;

			t = w.re*w1.re - w.im*w1.im;
			w.im = w.re*w1.im + w.im*w1.re;
			w.re = t;
		}
	}
}

#endif
