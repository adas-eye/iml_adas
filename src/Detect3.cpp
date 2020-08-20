#ifndef DETECT3_C
#define DETECT3_C

#include "Detect3.h"
#include <float.h>
#include <math.h>
#include <time.h>
#include <vector>
#include "model_core.h"
#include "common.h"

using namespace std;

//long time_a = 0;
//long time_b = 0;
//double time_c;

//malloc的数据一操作就失败

//float map_f1[18252];
float map_f1[18252*2];

float newData_f1[24*24*108*2];

float newData_f2[24*24*31*2];

CtTracker_t _ct_tracker;
int _width_min = 15;
int _template_size = 96;
const int _cell_size = 4;
float _lambda = 0.0001f;
float _peak_max = 0.4f;
float _interp_factor = 0.012f;
float _sigma = 0.6f;
float _padding = 2.5f;
float _output_sigma_factor = 0.125f;
float _scale_step = 1.05f;
float _scale_weight = 0.95f;

CtMat *_alphaf; //32c2
CtMat *_prob;	//32c2
CtMat *_tmpl;	//32c1

int _dim = 1728;
void *netinfo;
struct CtRectF _roi;
int size_patch[3];

CtSize _tmpl_sz;
float _scale;
int _gaussian_size;

vector<float> mean_vec;
vector<float> var_vec;
vector<float> data_vec;
CtMat mat_array[20];
int mat_idx[20];
const int mat_num = 20;
const int f1_num = 1;

const int f1_len = IMG_ROWS * IMG_COLS << 3;
//uchar mat_f1[f1_num][f1_len];

uchar mat_f1[F1_NUM][F1_LEN];

int f1_idx[F1_NUM];
//int f1_idx[f1_num];

const int f2_num = 2;
const int f2_len = IMG_ROWS * IMG_COLS << 2;

uchar mat_f2[F2_NUM][F2_LEN];

int f2_idx[F2_NUM];
//uchar mat_f2[f2_num][f2_len];
//int f2_idx[f2_num];

const int f3_num = 2;
const int f3_len = IMG_ROWS * IMG_COLS << 1;
//uchar mat_f3[f3_num][f3_len];
//int f3_idx[f3_num];
uchar mat_f3[F3_NUM][F3_LEN];
int f3_idx[F3_NUM];

const int f4_num = 4;
const int f4_len = IMG_ROWS * IMG_COLS;

uchar mat_f4[F4_NUM][F4_LEN];
int f4_idx[F4_NUM];

const int f5_num = 8;
const int f5_len = IMG_ROWS * IMG_COLS >> 1;

uchar mat_f5[F5_NUM][F5_LEN];
int f5_idx[F5_NUM];

const int f6_num = 8;
const int f6_len = IMG_ROWS * IMG_COLS >> 2;

uchar mat_f6[F6_NUM][F6_LEN];
int f6_idx[F6_NUM];

CtMat *_hann;	//32c1

int ctApplyMat_f1(uchar **pData, int mat_len)
{
	if (f1_len < mat_len)
	{
		printf("error to apply memory!\n");
		exit(-1);
	}
	for (int kk = 0; kk < f1_num; kk++) {
		if (f1_idx[kk] != 8) {
			*pData = &mat_f1[kk][0];
			f1_idx[kk] = 8;
			return (1 << 5) + kk;
		}
		if (kk == f1_num - 1) {
			printf("error to apply memory!\n");
			exit(-1);
		}
	}
	return -1;
}

int ctApplyMat_f2(uchar **pData, int mat_len)
{
	if (f2_len < mat_len)
		return ctApplyMat_f1(pData, mat_len);
	for (int kk = 0; kk < f2_num; kk++) {
		if (f2_idx[kk] != 8) {
			*pData = &mat_f2[kk][0];
			f2_idx[kk] = 8;
			return (2 << 5) + kk;
		}
		if (kk == f2_num - 1)
			return ctApplyMat_f1(pData, mat_len);
	}
	return -1;
}

int ctApplyMat_f3(uchar **pData, int mat_len)
{
	if (f3_len < mat_len)
		return ctApplyMat_f2(pData, mat_len);
	for (int kk = 0; kk < f3_num; kk++) {
		if (f3_idx[kk] != 8) {
			*pData = &mat_f3[kk][0];
			f3_idx[kk] = 8;
			return (3 << 5) + kk;
		}
		if (kk == f3_num - 1)
			return ctApplyMat_f2(pData, mat_len);
	}
	return -1;
}

int ctApplyMat_f4(uchar **pData, int mat_len)
{
	if (f4_len < mat_len)
		return ctApplyMat_f3(pData, mat_len);
	for (int kk = 0; kk < f4_num; kk++) {
		if (f4_idx[kk] != 8) {
			*pData = &mat_f4[kk][0];
			f4_idx[kk] = 8;
			return (4 << 5) + kk;
		}
		if (kk == f4_num - 1)
			return ctApplyMat_f3(pData, mat_len);
	}
	return -1;
}

int ctApplyMat_f5(uchar **pData, int mat_len)
{
	if (f5_len < mat_len)
		return ctApplyMat_f4(pData, mat_len);
	for (int kk = 0; kk < f5_num; kk++) {
		if (f5_idx[kk] != 8) {
			*pData = &mat_f5[kk][0];
			f5_idx[kk] = 8;
			return (5 << 5) + kk;
		}
		if (kk == f5_num - 1)
			return ctApplyMat_f4(pData, mat_len);
	}
	return -1;
}

int ctApplyMat_f6(uchar **pData, int mat_len)
{
	if (f6_len < mat_len)
		return ctApplyMat_f5(pData, mat_len);
	for (int kk = 0; kk < f6_num; kk++) {
		if (f6_idx[kk] != 8) {
			*pData = &mat_f6[kk][0];
			f6_idx[kk] = 8;
			return (6 << 5) + kk;
		}
		if (kk == f6_num - 1)
			return ctApplyMat_f5(pData, mat_len);
	}
	return -1;
}

CtMat *ctApplyMat(int rows, int cols, int type, int flag_init)
{
	CtMat *img;
	int kk = 0;
	for ( ; kk < mat_num; kk++) {
		if (mat_idx[kk] != 8) {
			img = &mat_array[kk];
			mat_idx[kk] = 8;
			break;
		}
		if (kk == mat_num - 1) {
			printf("error to apply mat!\n");
			exit(-1);
		}
	}
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
    img->refcount = NULL;
    img->hdr_refcount = 0;
	int mat_len = rows * img->step;
	int rlt = ctApplyMat_f6(&img->data.ptr, mat_len);
	rlt += kk << 10;
	img->hdr_refcount = rlt;
	if (flag_init) {
		memset(img->data.ptr, 0, mat_len);
	}
	//printf("apply memory idx: [%d]\n", rlt);
	return img;
}

int ctResetMat(CtMat *img)
{
	int mat_stp = img->hdr_refcount >> 10;
	int mem_num_mark = 31 << 5;
	int mem_num = (img->hdr_refcount & mem_num_mark) >> 5;
	int mem_idx_mark = 31;
	int mem_idx = img->hdr_refcount & mem_idx_mark;
	mat_idx[mat_stp] = 0;
	switch (mem_num) {
	case 1:
		f1_idx[mem_idx] = 0;
		break;
	case 2:
		f2_idx[mem_idx] = 0;
		break;
	case 3:
		f3_idx[mem_idx] = 0;
		break;
	case 4:
		f4_idx[mem_idx] = 0;
		break;
	case 5:
		f5_idx[mem_idx] = 0;
		break;
	case 6:
		f6_idx[mem_idx] = 0;
		break;
	}
	return 0;
}

inline void limit(CtRect &rect, int width, int height, int x = 0, int y = 0)
{
	CtRect limit;
	limit.x = x;
	limit.y = y;
	limit.width = width;
	limit.height = height;
	if (rect.x + rect.width > limit.x + limit.width)rect.width = (limit.x + limit.width - rect.x);
    if (rect.y + rect.height > limit.y + limit.height)rect.height = (limit.y + limit.height - rect.y);
    if (rect.x < limit.x)
    {
        rect.width -= (limit.x - rect.x);
        rect.x = limit.x;
    }
    if (rect.y < limit.y)
    {
        rect.height -= (limit.y - rect.y);
        rect.y = limit.y;
    }
    if(rect.width<0)rect.width=0;
    if(rect.height<0)rect.height=0;
}

inline int x2(const CtRect &rect)
{
    return rect.x + rect.width;
}

inline int y2(const CtRect &rect)
{
    return rect.y + rect.height;
}

inline CtRect getBorder(const CtRect &original, CtRect & limited)
{
    CtRect res;
    res.x = limited.x - original.x;
    res.y = limited.y - original.y;
    res.width = x2(original) - x2(limited);
    res.height = y2(original) - y2(limited);
    return res;
}

inline CtMat *subwindow(const CtMat *in, const CtRect window)
{
	CtRect cutWindow = window;
	limit(cutWindow, (*in).cols, (*in).rows);
	CtRect border = getBorder(window, cutWindow);
	CtMat *res = ctApplyMat(window.height, window.width, 1, 0);
	for (int ii = 0; ii < cutWindow.height; ii++) {
		uchar *p1 = (uchar*)(in->data.ptr + (ii + cutWindow.y) * in->step);
		uchar *p2 = (uchar*)(res->data.ptr + (ii + border.y) * res->step);
		for (int jj = 0; jj < cutWindow.width; jj++) {
			p2[jj + border.x] = p1[jj + cutWindow.x];
		}
	}
	if (border.y > 0) {
		int tmp_width = border.x + cutWindow.width;
		for (int ii = 0; ii < border.y; ii++) {
			uchar *p1 = (uchar*)(res->data.ptr + border.y * res->step);
			uchar *p2 = (uchar*)(res->data.ptr + ii * res->step);
			for (int jj = border.x; jj < tmp_width; jj++) {
				p2[jj] = p1[jj];
			}
		}
	}
	if (border.height > 0) { 
		int tmp_width = border.x + cutWindow.width;
		int tmp_height = window.height - border.height;
		for (int ii = tmp_height; ii < window.height; ii++) {
			uchar *p1 = (uchar*)(res->data.ptr + (tmp_height - 1) * res->step);
			uchar *p2 = (uchar*)(res->data.ptr + ii * res->step);
			for (int jj = border.x; jj < tmp_width; jj++) {
				p2[jj] = p1[jj];
			}
		}
	}
	if (border.x > 0) {
		for (int ii = 0; ii < window.height; ii++) {
			uchar *p1 = (uchar*)(res->data.ptr + ii * res->step);
			for (int jj = 0; jj < border.x; jj++) {
				p1[jj] = p1[border.x];
			}
		}
	}
	if (border.width > 0) {
		int tmp_width = window.width - border.width;
		for (int ii = 0; ii < window.height; ii++) {
			uchar *p1 = (uchar*)(res->data.ptr + ii * res->step);
			for (int jj = tmp_width; jj < window.width; jj++) {
				p1[jj] = p1[tmp_width - 1];
			}
		}
	}
	return res;
}

int allocFeatureMapObject(CtLSVMFeatureMapCaskade **obj, const int sizeX, 
                          const int sizeY, const int numFeatures)
{
    int i;
    (*obj) = (CtLSVMFeatureMapCaskade *)malloc(sizeof(CtLSVMFeatureMapCaskade));
    (*obj)->sizeX       = sizeX;
    (*obj)->sizeY       = sizeY;
    (*obj)->numFeatures = numFeatures;
//    (*obj)->map = (float *) malloc(sizeof (float) *
//                                  (sizeX * sizeY  * numFeatures));
    (*obj)->map = map_f1;
    for(i = 0; i < sizeX * sizeY * numFeatures; i++)
    {
        (*obj)->map[i] = 0.0f;
//        printf("i is %d\n", i);
    }
    return 0;
}

int getFeatureMaps(const CtMat* image, const int k, CtLSVMFeatureMapCaskade **map, const int seg_num)
{
    int sizeX, sizeY;
    int p, px, stringSize;
    int height, width;
    int i, j, kk, c, ii, jj, d;
    float  * datadx, * datady;
    
    float x, y;
    CtMat *dx, *dy;
    int nearest[_cell_size];
	float w[_cell_size * 2];
    float a_x, b_x;

    float * r;
    int   * alfa;
    
    float boundary_x[seg_num + 1];
    float boundary_y[seg_num + 1];
    float max, dotProd;
    int   maxi;

    height = image->height;
    width  = image->width ;

//    printf("The height is %d\n", height);
//    printf("The width is %d\n", width);
	dx = ctApplyMat(image->rows, image->cols, 2, 0);
	dy = ctApplyMat(image->rows, image->cols, 2, 0);

    sizeX = width  / k;
    sizeY = height / k;
    px    = 3 * seg_num; 
    p     = px;
    stringSize = sizeX * p;
    allocFeatureMapObject(map, sizeX, sizeY, p);

	for (int ii = 0; ii < image->height; ii++) {
		uchar* p1_pre = (uchar*)(image->data.ptr + (ii - 1) * image->step);
		uchar* p1_cur = (uchar*)(image->data.ptr + ii * image->step);
		uchar* p1_next = (uchar*)(image->data.ptr + (ii + 1) * image->step);
		if (ii == 0) {
			p1_pre = p1_cur;
		} else if (ii == image->height - 1) {
			p1_next = p1_cur;
		}
		float* p2 = (float*)(dx->data.ptr + ii * dx->step);
		float* p3 = (float*)(dy->data.ptr + ii * dy->step);
		for (int jj = 0; jj < image->width; jj++) {
			int jj_pre = jj == 0 ? jj : jj - 1;
			int jj_next = jj == image->width - 1 ? jj : jj+ 1;
			p2[jj] = float(p1_cur[jj_next] - p1_cur[jj_pre]);
			p3[jj] = float(p1_next[jj] - p1_pre[jj]);
		}
	}

    float arg_vector;
    for(i = 0; i <= seg_num; i++)
    {
        arg_vector    = ( (float) i ) * ( (float)(CT_PI) / (float)(seg_num) );
        boundary_x[i] = cosf(arg_vector);
        boundary_y[i] = sinf(arg_vector);
    }

	CtMat *r_mat = ctApplyMat(height, width, 2, 0);
	r = (float*)r_mat->data.ptr;
	CtMat *alfa_mat = ctApplyMat(height, width, 3, 0);
	alfa = (int*)alfa_mat->data.ptr;

    for(j = 1; j < height - 1; j++)
    {
		datadx = (float*)(dx->data.ptr + dx->step * j);
		datady = (float*)(dy->data.ptr + dy->step * j);
        for(i = 1; i < width - 1; i++)
        {
            c = 0;
            x = (datadx[i + c]);
            y = (datady[i + c]);

            r[j * width + i] =sqrtf(x * x + y * y);
            
            max  = boundary_x[0] * x + boundary_y[0] * y;
            maxi = 0;
            for (kk = 0; kk < seg_num; kk++) 
            {
                dotProd = boundary_x[kk] * x + boundary_y[kk] * y;
                if (dotProd > max) 
                {
                    max  = dotProd;
                    maxi = kk;
                }
                else 
                {
                    if (-dotProd > max) 
                    {
                        max  = -dotProd;
                        maxi = kk + seg_num;
                    }
                }
            }
            alfa[j * width * 2 + i * 2    ] = maxi % seg_num;
            alfa[j * width * 2 + i * 2 + 1] = maxi;  
        }
    }

    for(i = 0; i < k / 2; i++)
    {
        nearest[i] = -1;
    }
    for(i = k / 2; i < k; i++)
    {
        nearest[i] = 1;
    }

    for(j = 0; j < k / 2; j++)
    {
        b_x = k / 2 + j + 0.5f;
        a_x = k / 2 - j - 0.5f;
        w[j * 2    ] = 1.0f/a_x * ((a_x * b_x) / ( a_x + b_x)); 
        w[j * 2 + 1] = 1.0f/b_x * ((a_x * b_x) / ( a_x + b_x));  
    }
    for(j = k / 2; j < k; j++)
    {
        a_x = j - k / 2 + 0.5f;
        b_x =-j + k / 2 - 0.5f + k;
        w[j * 2    ] = 1.0f/a_x * ((a_x * b_x) / ( a_x + b_x)); 
        w[j * 2 + 1] = 1.0f/b_x * ((a_x * b_x) / ( a_x + b_x));  
    }

    for(i = 0; i < sizeY; i++)
    {
      for(j = 0; j < sizeX; j++)
      {
        for(ii = 0; ii < k; ii++)
        {
          for(jj = 0; jj < k; jj++)
          {
            if ((i * k + ii > 0) && 
                (i * k + ii < height - 1) && 
                (j * k + jj > 0) && 
                (j * k + jj < width  - 1))
            {
              d = (k * i + ii) * width + (j * k + jj);
              (*map)->map[ i * stringSize + j * (*map)->numFeatures + alfa[d * 2    ]] += 
                  r[d] * w[ii * 2] * w[jj * 2];
              (*map)->map[ i * stringSize + j * (*map)->numFeatures + alfa[d * 2 + 1] + seg_num] += 
                  r[d] * w[ii * 2] * w[jj * 2];
              if ((i + nearest[ii] >= 0) && 
                  (i + nearest[ii] <= sizeY - 1))
              {
                (*map)->map[(i + nearest[ii]) * stringSize + j * (*map)->numFeatures + alfa[d * 2    ]             ] += 
                  r[d] * w[ii * 2 + 1] * w[jj * 2 ];
                (*map)->map[(i + nearest[ii]) * stringSize + j * (*map)->numFeatures + alfa[d * 2 + 1] + seg_num] += 
                  r[d] * w[ii * 2 + 1] * w[jj * 2 ];
              }
              if ((j + nearest[jj] >= 0) && 
                  (j + nearest[jj] <= sizeX - 1))
              {
                (*map)->map[i * stringSize + (j + nearest[jj]) * (*map)->numFeatures + alfa[d * 2    ]             ] += 
                  r[d] * w[ii * 2] * w[jj * 2 + 1];
                (*map)->map[i * stringSize + (j + nearest[jj]) * (*map)->numFeatures + alfa[d * 2 + 1] + seg_num] += 
                  r[d] * w[ii * 2] * w[jj * 2 + 1];
              }
              if ((i + nearest[ii] >= 0) && 
                  (i + nearest[ii] <= sizeY - 1) && 
                  (j + nearest[jj] >= 0) && 
                  (j + nearest[jj] <= sizeX - 1))
              {
                (*map)->map[(i + nearest[ii]) * stringSize + (j + nearest[jj]) * (*map)->numFeatures + alfa[d * 2    ]             ] += 
                  r[d] * w[ii * 2 + 1] * w[jj * 2 + 1];
                (*map)->map[(i + nearest[ii]) * stringSize + (j + nearest[jj]) * (*map)->numFeatures + alfa[d * 2 + 1] + seg_num] += 
                  r[d] * w[ii * 2 + 1] * w[jj * 2 + 1];
              }
            }
          }
        }
	  }
	}
    
	ctResetMat(dx);
	ctResetMat(dy);
	ctResetMat(r_mat);
	ctResetMat(alfa_mat);

    return 0;
}

int normalizeAndTruncate(CtLSVMFeatureMapCaskade *map, const float alfa)
{
    int i,j, ii;
    int sizeX, sizeY, p, pos, pp, xp, pos1, pos2;
    float * partOfNorm;
    float * newData;
    float   valOfNorm;
    sizeX     = map->sizeX;
    sizeY     = map->sizeY;
    partOfNorm = (float *)malloc (sizeof(float) * (sizeX * sizeY));
    p  = NUM_SEG;
    xp = NUM_SEG * 3;
    pp = NUM_SEG * 12;
    for(i = 0; i < sizeX * sizeY; i++)
    {
        valOfNorm = 0.0f;
        pos = i * map->numFeatures;
        for(j = 0; j < p; j++)
        {
            valOfNorm += map->map[pos + j] * map->map[pos + j];
        }
        partOfNorm[i] = valOfNorm;
    }
    sizeX -= 2;
    sizeY -= 2;
//    newData = (float *)malloc (sizeof(float) * (sizeX * sizeY * pp));
    newData = newData_f1;
    for(i = 1; i <= sizeY; i++)
    {
        for(j = 1; j <= sizeX; j++)
        {
            valOfNorm = sqrtf(
                partOfNorm[(i    )*(sizeX + 2) + (j    )] +
                partOfNorm[(i    )*(sizeX + 2) + (j + 1)] +
                partOfNorm[(i + 1)*(sizeX + 2) + (j    )] +
                partOfNorm[(i + 1)*(sizeX + 2) + (j + 1)]) + FLT_EPSILON;
            pos1 = (i  ) * (sizeX + 2) * xp + (j  ) * xp;
            pos2 = (i-1) * (sizeX    ) * pp + (j-1) * pp;
            for(ii = 0; ii < p; ii++)
            {
                newData[pos2 + ii        ] = map->map[pos1 + ii    ] / valOfNorm;
            }
            for(ii = 0; ii < 2 * p; ii++)
            {
                newData[pos2 + ii + p * 4] = map->map[pos1 + ii + p] / valOfNorm;
            }
            valOfNorm = sqrtf(
                partOfNorm[(i    )*(sizeX + 2) + (j    )] +
                partOfNorm[(i    )*(sizeX + 2) + (j + 1)] +
                partOfNorm[(i - 1)*(sizeX + 2) + (j    )] +
                partOfNorm[(i - 1)*(sizeX + 2) + (j + 1)]) + FLT_EPSILON;
            for(ii = 0; ii < p; ii++)
            {
                newData[pos2 + ii + p    ] = map->map[pos1 + ii    ] / valOfNorm;
            }
            for(ii = 0; ii < 2 * p; ii++)
            {
                newData[pos2 + ii + p * 6] = map->map[pos1 + ii + p] / valOfNorm;
            }
            valOfNorm = sqrtf(
                partOfNorm[(i    )*(sizeX + 2) + (j    )] +
                partOfNorm[(i    )*(sizeX + 2) + (j - 1)] +
                partOfNorm[(i + 1)*(sizeX + 2) + (j    )] +
                partOfNorm[(i + 1)*(sizeX + 2) + (j - 1)]) + FLT_EPSILON;
            for(ii = 0; ii < p; ii++)
            {
                newData[pos2 + ii + p * 2] = map->map[pos1 + ii    ] / valOfNorm;
            }
            for(ii = 0; ii < 2 * p; ii++)
            {
                newData[pos2 + ii + p * 8] = map->map[pos1 + ii + p] / valOfNorm;
            }
            valOfNorm = sqrtf(
                partOfNorm[(i    )*(sizeX + 2) + (j    )] +
                partOfNorm[(i    )*(sizeX + 2) + (j - 1)] +
                partOfNorm[(i - 1)*(sizeX + 2) + (j    )] +
                partOfNorm[(i - 1)*(sizeX + 2) + (j - 1)]) + FLT_EPSILON;
            for(ii = 0; ii < p; ii++)
            {
                newData[pos2 + ii + p * 3 ] = map->map[pos1 + ii    ] / valOfNorm;
            }
            for(ii = 0; ii < 2 * p; ii++)
            {
                newData[pos2 + ii + p * 10] = map->map[pos1 + ii + p] / valOfNorm;
            }
        }
    }

    for(i = 0; i < sizeX * sizeY * pp; i++)
    {
        if(newData [i] > alfa) newData [i] = alfa;
    }

    map->numFeatures  = pp;
    map->sizeX = sizeX;
    map->sizeY = sizeY;

//    free (map->map);
    map->map = NULL;
    free (partOfNorm);

    map->map = newData;

    return 0;
}


int PCAFeatureMaps(CtLSVMFeatureMapCaskade *map)
{ 
    int i,j, ii, jj, k;
    int sizeX, sizeY, p,  pp, xp, yp, pos1, pos2;
    float * newData;
    float val;
    float nx, ny;
    
    sizeX = map->sizeX;
    sizeY = map->sizeY;
    p     = map->numFeatures;
    pp    = NUM_SEG * 3 + 4;
    yp    = 4;
    xp    = NUM_SEG;

    nx    = 1.0f / sqrtf((float)(xp * 2));
    ny    = 1.0f / sqrtf((float)(yp    ));

//    newData = (float *)malloc (sizeof(float) * (sizeX * sizeY * pp));
    newData = newData_f2;

    for(i = 0; i < sizeY; i++)
    {
        for(j = 0; j < sizeX; j++)
        {
            pos1 = ((i)*sizeX + j)*p;
            pos2 = ((i)*sizeX + j)*pp;
            k = 0;
            for(jj = 0; jj < xp * 2; jj++)
            {
                val = 0;
                for(ii = 0; ii < yp; ii++)
                {
                    val += map->map[pos1 + yp * xp + ii * xp * 2 + jj];
                }
                newData[pos2 + k] = val * ny;
                k++;
            }
            for(jj = 0; jj < xp; jj++)
            {
                val = 0;
                for(ii = 0; ii < yp; ii++)
                {
                    val += map->map[pos1 + ii * xp + jj];
                }
                newData[pos2 + k] = val * ny;
                k++;
			}
            for(ii = 0; ii < yp; ii++)
            {
                val = 0;
                for(jj = 0; jj < 2 * xp; jj++)
                {
                    val += map->map[pos1 + yp * xp + ii * xp * 2 + jj];
                }
                newData[pos2 + k] = val * nx;
                k++;
            }
        }
    }
    map->numFeatures = pp;
//    free (map->map);
    map->map = NULL;
    map->map = newData;
    return 0;
}

int freeFeatureMapObject (CtLSVMFeatureMapCaskade **obj)
{
    if(*obj == NULL)
		return 2;
//    free((*obj)->map);
    (*obj)->map = NULL;
    free(*obj);
    (*obj) = NULL;
    return 0;
}

int createHanningMats()
{
	int hann1t_num = size_patch[1];
	int hann2t_num = size_patch[0];
	float *hann1t = (float*)malloc(size_patch[1] * sizeof(float*));
	float *hann2t = (float*)malloc(size_patch[0] * sizeof(float*));
	float *hann3d = (float*)malloc(size_patch[0] * size_patch[1] * sizeof(float*));
    for (int i = 0; i < hann1t_num; i++) {
//        hann1t[i] = 0.5f * (1 - std::cos(2 * 3.14159265358979323846f * i / (hann1t_num - 1)));
        hann1t[i] = 0.5f * (1 - cos(2 * 3.14159265358979323846f * i / (hann1t_num - 1)));
	}
    for (int i = 0; i < hann2t_num; i++) {
//        hann2t[i] = 0.5f * (1 - std::cos(2 * 3.14159265358979323846f * i / (hann2t_num - 1)));
        hann2t[i] = 0.5f * (1 - cos(2 * 3.14159265358979323846f * i / (hann2t_num - 1)));
	}
	for (int ii = 0; ii < hann2t_num; ii++) {
		int idx = ii * hann1t_num;
		for (int jj = 0; jj < hann1t_num; jj++) {
			hann3d[idx + jj] = hann2t[ii] * hann1t[jj];
		}
	}
	_hann = ctApplyMat(size_patch[2], size_patch[0]*size_patch[1], 2, 0);
	for (int i = 0; i < size_patch[2]; i++) {
		float* p = (float*)(_hann->data.ptr + i * _hann->step);
		for (int j = 0; j<size_patch[0]*size_patch[1]; j++)
		{
			p[j] = hann3d[j];
		}
	}
	free(hann1t);
	free(hann2t);
	free(hann3d);
	return 0;
}

CtMat *getFeatures(const CtMat *image, bool inithann, float scale_adjust)
{
    CtRect extracted_roi;

    float cx = _roi.x + _roi.width / 2;
    float cy = _roi.y + _roi.height / 2;

    if (inithann) {
        int padded_w = int(_roi.width * _padding);
        int padded_h = int(_roi.height * _padding);
        
        if (_template_size > 1) {
            if (padded_w >= padded_h)
                _scale = padded_w / (float) _template_size;
            else
                _scale = padded_h / (float) _template_size;

            _tmpl_sz.width = int(padded_w / _scale);
            _tmpl_sz.height = int(padded_h / _scale);
        }
        else {
            _tmpl_sz.width = padded_w;
            _tmpl_sz.height = padded_h;
            _scale = 1;
		}
		_tmpl_sz.width = ( ( (int)(_tmpl_sz.width / (2 * _cell_size)) ) * 2 * _cell_size ) + _cell_size*2;
		_tmpl_sz.height = ( ( (int)(_tmpl_sz.height / (2 * _cell_size)) ) * 2 * _cell_size ) + _cell_size*2;
	}

    extracted_roi.width = int(scale_adjust * _scale * _tmpl_sz.width);
    extracted_roi.height = int(scale_adjust * _scale * _tmpl_sz.height);

    extracted_roi.x = int(cx - (extracted_roi.width >> 1));
    extracted_roi.y = int(cy - (extracted_roi.height >> 1));


    //0.013
	CtMat *z = subwindow(image, extracted_roi);

	 //0.032
	CtMat *z_resize;
	if (z->cols != _tmpl_sz.width || z->rows != _tmpl_sz.height) {
		z_resize = ctApplyMat(_tmpl_sz.height, _tmpl_sz.width, 1, 0);

		float x_scale = z->cols / (float)z_resize->cols;
		float y_scale = z->rows / (float)z_resize->rows;
		for (int ii = 0; ii < z_resize->rows; ii++) {
			float y_cur = (float)((ii + 0.5) * y_scale - 0.5);
			int y_pre = (int)y_cur;
			int y_next = y_pre + 1;
			float y_factor = y_cur - y_pre;
			float y_factor_inv = 1 - y_factor;
			uchar *p1_pre = (uchar *)(z->data.ptr + y_pre * z->step);
			uchar *p1_next = (uchar *)(z->data.ptr + y_next * z->step);
			uchar *p2 = (uchar *)(z_resize->data.ptr + ii * z_resize->step);
			for (int jj = 0; jj < z_resize->cols; jj++) {
				float x_cur = (float)((jj + 0.5) * x_scale - 0.5);
				int x_pre = (int)x_cur;
				int x_next = x_pre + 1;
				float x_factor = x_cur - x_pre;
				float x_factor_inv = 1 - x_factor;
				p2[jj] = (uchar)((x_factor_inv * y_factor_inv * p1_pre[x_pre]
						+ x_factor * y_factor_inv * p1_pre[x_next]
						+ x_factor_inv * y_factor * p1_next[x_pre]
						+ x_factor * y_factor * p1_next[x_next]) + 0.5);
			}
		}
	}
	else{
		z_resize = z;
	}
	ctResetMat(z);

	CtLSVMFeatureMapCaskade *map;

	//0.083
	getFeatureMaps(z_resize, _cell_size, &map, NUM_SEG);
	ctResetMat(z);
	ctResetMat(z_resize);

	//0.048
	normalizeAndTruncate(map,0.2f);
	PCAFeatureMaps(map);
	//

	size_patch[0] = map->sizeY;
	size_patch[1] = map->sizeX;
	size_patch[2] = map->numFeatures;

	CtMat *FeaturesMap = ctApplyMat(map->sizeX*map->sizeY, map->numFeatures, 2, 0);
	CtMat *FeaturesMap_t = ctApplyMat(map->numFeatures, map->sizeX*map->sizeY, 2, 0);
	memcpy(FeaturesMap->data.fl, map->map, map->sizeX * map->sizeY * map->numFeatures * sizeof(float));
	for (int ii = 0; ii < FeaturesMap->rows; ii++) {
		for (int jj = 0; jj < FeaturesMap->cols; jj++) {
			float* p1 = (float*)(FeaturesMap->data.ptr + ii * FeaturesMap->step);
			float* p2 = (float*)(FeaturesMap_t->data.ptr + jj * FeaturesMap_t->step);
			p2[ii] = p1[jj];
		}
	}
	ctResetMat(FeaturesMap);
	freeFeatureMapObject(&map);
    if (inithann) {
        createHanningMats();
    }
	ctMulMat_ch1(FeaturesMap_t, _hann);
	return FeaturesMap_t;
}

CtMat *genFeat(const CtMat *image)
{
	const CtMat *z = image;
	CtMat *z_resize;
	CtSize tmp_size;
	tmp_size.width = 32;
	tmp_size.height = 32;

	z_resize = ctApplyMat(tmp_size.height, tmp_size.width, 1, 0);
	float x_scale = z->cols / (float)z_resize->cols;
	float y_scale = z->rows / (float)z_resize->rows;
	for (int ii = 0; ii < z_resize->rows; ii++) {
		float y_cur = (float)((ii + 0.5) * y_scale - 0.5);
		int y_pre = (int)y_cur;
		int y_next = y_pre + 1;
		float y_factor = y_cur - y_pre;
		float y_factor_inv = 1 - y_factor;
		uchar *p1_pre = (uchar *)(z->data.ptr + y_pre * z->step);
		uchar *p1_next = (uchar *)(z->data.ptr + y_next * z->step);
		uchar *p2 = (uchar *)(z_resize->data.ptr + ii * z_resize->step);
		for (int jj = 0; jj < z_resize->cols; jj++) {
			float x_cur = (float)((jj + 0.5) * x_scale - 0.5);
			int x_pre = (int)x_cur;
			int x_next = x_pre + 1;
			float x_factor = x_cur - x_pre;
			float x_factor_inv = 1 - x_factor;
			p2[jj] = (uchar)((x_factor_inv * y_factor_inv * p1_pre[x_pre]
					+ x_factor * y_factor_inv * p1_pre[x_next]
					+ x_factor_inv * y_factor * p1_next[x_pre]
					+ x_factor * y_factor * p1_next[x_next]) + 0.5);
		}
	}

	CtLSVMFeatureMapCaskade *map;
	getFeatureMaps(z_resize, _cell_size, &map, NUM_SEG);
	ctResetMat(z_resize); 
	size_patch[0] = map->sizeY;
	size_patch[1] = map->sizeX;
	size_patch[2] = map->numFeatures;

	CtMat *FeaturesMap = ctApplyMat(map->sizeX*map->sizeY, map->numFeatures, 2, 0);
	CtMat *FeaturesMap_t = ctApplyMat(map->numFeatures, map->sizeX*map->sizeY, 2, 0);
	memcpy(FeaturesMap->data.fl, map->map, map->sizeX * map->sizeY * map->numFeatures * sizeof(float));
	for (int ii = 0; ii < FeaturesMap->rows; ii++) {
		for (int jj = 0; jj < FeaturesMap->cols; jj++) {
			float* p1 = (float*)(FeaturesMap->data.ptr + ii * FeaturesMap->step);
			float* p2 = (float*)(FeaturesMap_t->data.ptr + jj * FeaturesMap_t->step);
			p2[ii] = p1[jj];
		}
	}
	ctResetMat(FeaturesMap);
	freeFeatureMapObject(&map);
	return FeaturesMap_t;
}

int tracker_create()
{
	char *mean_path = (char*)"lib/libvar.a";
	const char *file_path = (char*)"lib/libnet.a";
	FILE *fp_mean = fopen(mean_path, "rt");
	char content0[50000];
	while (!feof(fp_mean)) {
		memset(content0, '\0', 50000);
		char *tt = fgets(content0, 50000, fp_mean);
		char *temp = strtok(content0, " \r\n");
		if (temp == NULL || temp[0] == '#' || temp[0] == '\0') {
			continue;
		}
		mean_vec.push_back(atof(temp));
		temp = strtok(NULL, " \r\n");
		var_vec.push_back(sqrt(atof(temp)));
	}
	netinfo = score_namespace::model_init((void*)file_path);
}

int tracker_check(vector<float> &val_vec)
{
	score_namespace::model_score(netinfo, data_vec, val_vec);
	return 0;
}

int tracker_check(CtMat *image, CtRect *rect, int index)
{
	data_vec.resize((index + 1) * _dim);
	CtMat *sub_mat = subwindow(image, *rect);
	CtMat * feat_mat = genFeat(sub_mat);
	int cnt = 0;
	int offset = index * _dim;
	for (int ii = 0; ii < feat_mat->rows; ii++) {
		float* p1 = (float*)(feat_mat->data.ptr + ii * feat_mat->step);
		for (int jj = 0; jj < feat_mat->cols; jj++) {
			//data_vec[offset + cnt++] = p1[jj];
			float val = p1[jj];
			if (var_vec[cnt] >= 0.000001f && fabs(val) >= 0.000001f) {
				val = (val - mean_vec[cnt]) / var_vec[cnt];;
			}
			data_vec[offset + cnt] = val;
			cnt++;
		}
	}
	for (int kk = cnt; kk < _dim; kk++) {
		data_vec[offset + kk] = 0.0f;
	}

	ctResetMat(sub_mat);
	ctResetMat(feat_mat);
	return 0;
}

void fftd(CtMat *img_src, CtMat *img_dst, bool backwards)
{
	//int channels = img_src->step / img_src->cols / sizeof(float);
	int channels = img_src->step / img_src->cols / sizeof(float);
	if (channels == 1) {
		for (int ii = 0; ii < img_dst->rows; ii++) {
			float* p1 = (float*)(img_dst->data.ptr + ii * img_dst->step);
			float* p2 = (float*)(img_src->data.ptr + ii * img_src->step);
			for (int jj = 0; jj < img_dst->cols; jj++) {
				p1[2 * jj] = p2[jj];
				p1[2 * jj + 1] = 0;
			}
		}
		ctdft(img_dst, img_dst, backwards ? 3 : 0 );
		return;
	} else {
		ctdft(img_src, img_src, backwards ? 3 : 0 );
		return;
	}
}

int createGaussianPeak(int sizey, int sizex, CtMat **img_dst)
{
	CtMat *res = ctApplyMat(sizey, sizex, 2, 0);

//    int syh = (sizey) / 2;
//    int sxh = (sizex) / 2;
    int syh = (sizey) >> 1;
    int sxh = (sizex) >> 1;

//    float output_sigma = std::sqrt((float) sizex * sizey) / _padding * _output_sigma_factor;
    float output_sigma = sqrt((float) sizex * sizey) / _padding * _output_sigma_factor;
    float mult = -0.5f / (output_sigma * output_sigma);

    for (int i = 0; i < sizey; i++) {
		float* pData=(float*)(res->data.ptr + i * res->step);
        for (int j = 0; j < sizex; j++)
        {
            int ih = i - syh;
            int jh = j - sxh;
//            pData[j] = std::exp(mult * (float) (ih * ih + jh * jh));
            pData[j] = exp(mult * (float) (ih * ih + jh * jh));
        }
	}
	*img_dst = ctApplyMat(res->rows, res->cols, 3, 0);
	fftd(res, *img_dst, false);
	ctResetMat(res);
	return 0;
}

void rearrange(CtMat *img)
{
//    int cx = img->cols / 2;
//    int cy = img->rows / 2;
    int cx = img->cols >> 1;
    int cy = img->rows >> 1;

	for (int ii = 0; ii < cy; ii++) {
		float* p1 = (float*)(img->data.ptr + ii * img->step);
		float* p2 = (float*)(img->data.ptr + (cy + ii) * img->step);
		for (int jj = 0; jj < cx; jj++) {
			float tmp1 = p1[2 * jj];
			float tmp2 = p1[2 * jj + 1];
			p1[2 * jj] = p2[2 * (cx + jj)];
			p1[2 * jj + 1] = p2[2 * (cx + jj) + 1];
			p2[2 * (cx + jj)] = tmp1;
			p2[2 * (cx + jj) + 1] = tmp2;
		}
	}
	int cy_2 = 2 * cy;
	for (int ii = cy; ii < cy_2; ii++) {
		float* p1 = (float*)(img->data.ptr + ii * img->step);
		float* p2 = (float*)(img->data.ptr + (ii - cy) * img->step);
		for (int jj = 0; jj < cx; jj++) {
			float tmp1 = p1[2 * jj];
			float tmp2 = p1[2 * jj + 1];
			p1[2 * jj] = p2[2 * (cx + jj)];
			p1[2 * jj + 1] = p2[2 * (cx + jj) + 1];
			p2[2 * (cx + jj)] = tmp1;
			p2[2 * (cx + jj) + 1] = tmp2;
		}
	}
}

int real(CtMat *img, CtMat *res)
{
	for (int ii = 0; ii < img->rows; ii++) {
		float* p1 = (float*)(img->data.ptr + ii * img->step);
		float* p2 = (float*)(res->data.ptr + ii * res->step);
		for (int jj = 0; jj < img->cols; jj++) {
			p2[jj] = p1[2 * jj];
		}
	}
	return 0;
}

CtMat *gaussianCorrelation(CtMat *x1, CtMat *x2)
{
	CtMat *c;
	CtMat *x1aux = ctApplyMat(size_patch[0], size_patch[1], 2, 0);
	CtMat *x2aux = ctApplyMat(size_patch[0], size_patch[1], 2, 0);

	CtMat *x1_rlt = ctApplyMat(x1aux->rows, x1aux->cols, 3, 0);
	CtMat *x2_rlt = ctApplyMat(x2aux->rows, x2aux->cols, 3, 0);

	CtMat *c_part = ctApplyMat(x1_rlt->rows, x1_rlt->cols, 2, 0);


//	printf("size_patch[2] is %d\n", size_patch[2]);
	for (int kk = 0; kk < size_patch[2]; kk++)
	{

		//0.003
		float* p2 = (float*)(x1->data.ptr + kk * x1->step);
		for (int ii = 0; ii < x1aux->rows; ii++)
		{
			float* p1 = (float*)(x1aux->data.ptr + ii * x1aux->step);
			int idx = ii * size_patch[1];
			for (int jj = 0; jj < x1aux->cols; jj++)
			{
				p1[jj] = p2[idx + jj];
			}
		}

		p2 = (float*)(x2->data.ptr + kk * x2->step);
		for (int ii = 0; ii < x2aux->rows; ii++) {
			float* p1 = (float*)(x2aux->data.ptr + ii * x2aux->step);
			int idx = ii * size_patch[1];
			for (int jj = 0; jj < x2aux->cols; jj++) {
				p1[jj] = p2[idx + jj];
			}
		}
		//
		//0.0015
		fftd(x1aux, x1_rlt, false);
		//0.0015
		fftd(x2aux, x2_rlt, false);

		//0.0015
		ctMulMat_ch2_conj(x1_rlt, x2_rlt);
		//0.0015
		fftd(x1_rlt, NULL, true);
		rearrange(x1_rlt);

		if (kk == 0) {
			c = ctApplyMat(x1_rlt->rows, x1_rlt->cols, 2, 0);
			real(x1_rlt, c);
		} else {
			real(x1_rlt, c_part);
			ctPlus_ch1(c, c_part);
		}

	}

	ctResetMat(x1_rlt);
	ctResetMat(x2_rlt);
	ctResetMat(x1aux);
	ctResetMat(x2aux);
	ctResetMat(c_part);
	float ss1 = ctSum_mul_ch1(x1);
	float ss2 = ctSum_mul_ch1(x2);
	ctMul_ch1(c, -2.0f);
	ctPlus_const_ch1(c, ss1 + ss2);
	ctMul_ch1(c, 1.0f / (size_patch[0]*size_patch[1]*size_patch[2]));
	ctMax_ch1(c, 0);
	ctMul_ch1(c, -1.0f / (_sigma * _sigma));
	ctExp_ch1(c);
	return c;
}

int complexDivision(CtMat *a, CtMat *b, CtMat *res)
{
	for (int ii = 0; ii < a->rows; ii++) {
		float* p1 = (float*)(a->data.ptr + ii * a->step);
		float* p2 = (float*)(b->data.ptr + ii * b->step);
		float* p3 = (float*)(res->data.ptr + ii * res->step);
		for (int jj = 0; jj < a->cols; jj++) {
			float a0 = p1[2 * jj];
			float a1 = p1[2 * jj + 1];
			float b0 = p2[2 * jj];
			float b1 = p2[2 * jj + 1];
			float divisor = 1.0f / (b0 * b0 + b1 * b1);
			p3[2 * jj] = (a0 * b0 + a1 * b1) * divisor;
			p3[2 * jj + 1] = (a1 * b0 + a0 * b1) * divisor;
		}
	}
	return 0;
}

void tracker_train(CtMat *x, float train_interp_factor)
{

    CtMat *k = gaussianCorrelation(x, x);

	CtMat *fft_res = ctApplyMat(k->rows, k->cols, 3, 0);


	fftd(k, fft_res, false);

	ctResetMat(k);

	ctPlus_const_ch2(fft_res, _lambda);
	CtMat *alphaf = ctApplyMat(_prob->rows, _prob->cols, 3, 0);
	complexDivision(_prob, fft_res, alphaf);
	ctResetMat(fft_res);
    
	CtMat *x_tmp2 = ctApplyMat(x->rows, x->cols, 2, 0);
	memcpy(x_tmp2->data.ptr, x->data.ptr, x->rows * x->cols * 4);

	ctMul_ch1(_tmpl, 1 - train_interp_factor);
	ctMul_ch1(x_tmp2, train_interp_factor);
	ctPlus_ch1(_tmpl, x_tmp2);
	ctResetMat(x_tmp2);

	ctMul_ch2(_alphaf, 1 - train_interp_factor);
	ctMul_ch2(alphaf, train_interp_factor);
	ctPlus_ch2(_alphaf, alphaf);
	ctResetMat(alphaf);

}

int tracker_init(const CtRectF &roi, CtMat *image)
{
    _roi = roi;
    //0.182317
	_tmpl = getFeatures(image, 1, 1.0f);
	createGaussianPeak(size_patch[0], size_patch[1], &_prob);
	_alphaf = ctApplyMat(size_patch[0], size_patch[1], 3, 1);
	//0.302868
	tracker_train(_tmpl, 1.0); // train with initial frame

	_ct_tracker.is_tracking = 1;
	_ct_tracker.frames = 1;
	_ct_tracker.last_rect = roi;
	return 0;
}

int tracker_release()
{
	ctResetMat(_hann);
	ctResetMat(_tmpl);
	ctResetMat(_prob);
	ctResetMat(_alphaf);
	_ct_tracker.is_tracking = 0;
	_ct_tracker.frames = 0;
	_ct_tracker.last_rect.x = 0;
	_ct_tracker.last_rect.y = 0;
	_ct_tracker.last_rect.width = 0;
	_ct_tracker.last_rect.height = 0;
	return 0;
}

int complexMultiplication(CtMat *a, CtMat *b, CtMat *res)
{
	for (int ii = 0; ii < a->rows; ii++) {
		float* p1 = (float*)(a->data.ptr + ii * a->step);
		float* p2 = (float*)(b->data.ptr + ii * b->step);
		float* p3 = (float*)(res->data.ptr + ii * res->step);
		for (int jj = 0; jj < a->cols; jj++) {
			float a0 = p1[2 * jj];
			float a1 = p1[2 * jj + 1];
			float b0 = p2[2 * jj];
			float b1 = p2[2 * jj + 1];
			p3[2 * jj] = (a0 * b0 - a1 * b1);
			p3[2 * jj + 1] = (a0 * b1 + a1 * b0);
		}
	}
	return 0;
}

float subPixelPeak(float left, float center, float right)
{   
    float divisor = 2 * center - right - left;

    if (divisor == 0)
        return 0;
    
    return 0.5f * (right - left) / divisor;
}

int cal_max_elem(CtMat *img, double *pv, CtPoint *pi)
{
	double pv_max = 0.0;
//	float test = 0.222;
	CtPoint pi_max;
//	printf("img->step is %d\n", img->step);
//	printf("img->rows is %d\n", img->rows);
//	printf("img->cols is %d\n", img->cols);
//	float* p_test = (float*)(img->data.ptr);
//	printf("p_test is %f\n", p_test[0]);

	for (int ii = 0; ii < img->rows; ii++)
	{
		float* p1 = (float*)(img->data.ptr + ii * img->step);
//		printf("p1 is %f\n", p1[0]);

		for (int jj = 0; jj < img->cols; jj++)
		{
//			printf("P1[jj] is %f\n", p1[jj]);
			if (p1[jj] > pv_max)
			{
				pv_max = p1[jj];
				pi_max.x = jj;
				pi_max.y = ii;
			}
		}
	}
	*pv = pv_max;
	*pi = pi_max;
	return 0;
}

CtPointF tracker_detect(CtMat *z, CtMat *x, float &peak_value)
{

    CtMat *k = gaussianCorrelation(x, z);
	CtMat *b_mat = ctApplyMat(k->rows, k->cols, 3, 0);
	fftd(k, b_mat, false);
	ctResetMat(k);

	CtMat *c_mat = ctApplyMat(_alphaf->rows, _alphaf->cols, 3, 0);
	complexMultiplication(_alphaf, b_mat, c_mat);
	fftd(c_mat, NULL, true);

	CtMat *res = ctApplyMat(c_mat->rows, c_mat->cols, 2, 0);
	real(c_mat, res);
	ctResetMat(b_mat);
	ctResetMat(c_mat);

    CtPoint pi;
    double pv;
	cal_max_elem(res, &pv, &pi);
    peak_value = (float) pv;

    CtPointF p;
	p.x = (float)pi.x;
	p.y = (float)pi.y;

    if (pi.x > 0 && pi.x < res->cols-1) {
		float* p1 = (float*)(res->data.ptr + pi.y * res->step);
		p.x += subPixelPeak(p1[pi.x-1], peak_value, p1[pi.x+1]);
    }

    if (pi.y > 0 && pi.y < res->rows-1) {
		float* p1 = (float*)(res->data.ptr + (pi.y-1) * res->step);
		float* p2 = (float*)(res->data.ptr + (pi.y+1) * res->step);
		p.y += subPixelPeak(p1[pi.x], peak_value, p2[pi.x]);
    }
//
//    p.x -= (res->cols) / 2;
//    p.y -= (res->rows) / 2;

    p.x -= (res->cols) >> 1;
    p.y -= (res->rows) >> 1;

	ctResetMat(res);

    return p;
}

int tracker_update(CtMat *image, CtRectF *rect)
{
    if (_roi.x + _roi.width <= 0) _roi.x = -_roi.width + 1;
    if (_roi.y + _roi.height <= 0) _roi.y = -_roi.height + 1;
    if (_roi.x >= (*image).cols - 1) _roi.x = float((*image).cols - 2);
    if (_roi.y >= (*image).rows - 1) _roi.y = float((*image).rows - 2);

    float cx = _roi.x + _roi.width / 2.0f;
    float cy = _roi.y + _roi.height / 2.0f;

    float peak_value;

	CtMat *feat_mat = getFeatures(image, 0, 1.0f);//0.182317

	CtPointF res = tracker_detect(_tmpl, feat_mat, peak_value);//0.30

	ctResetMat(feat_mat);

	if (_scale_step != 1) {
		float new_peak_value;

		feat_mat = getFeatures(image, 0, 1.0f / _scale_step);//0.18

		//time_a = clock();
		CtPointF new_res = tracker_detect(_tmpl, feat_mat, new_peak_value);//0.30
		//time_b = clock();
		//time_c = ((double)(time_b - time_a)/CLOCKS_PER_SEC);
		ctResetMat(feat_mat);

        if (_scale_weight * new_peak_value > peak_value) {
            res = new_res;
            peak_value = new_peak_value;
            _scale /= _scale_step;
            _roi.width /= _scale_step;
            _roi.height /= _scale_step;
        }

		feat_mat = getFeatures(image, 0, _scale_step);//0.18
		new_res = tracker_detect(_tmpl, feat_mat, new_peak_value);//0.30
		ctResetMat(feat_mat);

        if (_scale_weight * new_peak_value > peak_value) {
            res = new_res;
            peak_value = new_peak_value;
            _scale *= _scale_step;
            _roi.width *= _scale_step;
            _roi.height *= _scale_step;
        }
    }
	if (peak_value < _peak_max || _roi.width < _width_min) {
		return -1;
	}
    _roi.x = cx - _roi.width / 2.0f + ((float) res.x * _cell_size * _scale);
    _roi.y = cy - _roi.height / 2.0f + ((float) res.y * _cell_size * _scale);
    if (_roi.x >= (*image).cols - 1) _roi.x = float((*image).cols - 1);
    if (_roi.y >= (*image).rows - 1) _roi.y = float((*image).rows - 1);
    if (_roi.x + _roi.width <= 0) _roi.x = -_roi.width + 2;
	if (_roi.y + _roi.height <= 0) _roi.y = -_roi.height + 2;
	feat_mat = getFeatures(image, 0, 1.0f);//0.18
	tracker_train(feat_mat, _interp_factor);//0.30
	ctResetMat(feat_mat);
	_ct_tracker.frames++;
	_ct_tracker.last_rect = _roi;
	*rect = _roi;
    return 0;
}

void cv_LoadCvMat(
                      CtMat       *pFilter,
//                      int32_t     nType,
//                      uint32_t    nRows,
//                      uint32_t    nCols,
                      void        *pMask
                      )
{
	    pFilter->type = 0;
	    pFilter->rows = IMG_ROWS;
	    pFilter->step = IMG_COLS;
//	    pFilter->height = IMG_ROWS;
	    pFilter->cols = IMG_COLS;
	    //pFilter->refcount = 1;
//	    pFilter->width = IMG_COLS;
	    pFilter->data.ptr = (uchar *)pMask;
}
#endif
