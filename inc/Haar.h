#ifndef _HAAR_H_
#define _HAAR_H_
#include "Tables.h"
//#include "OpenCV.h"
//#include "test_score.h"
#include "common.h"
#include "loadCascade.h"

#define NODE_THRESHOLD_SHIFT 22

#define MAXHIDCASCADE 200000  //隐式级联分类器所占空间(字节)
#define MAXROWS   480   
#define MAXCOLS   752	
#define MAXSTAGES  22   
#define MAXCLASSIFER  213
#define MAXTREENODE 2
#define MAXALPHA   2
#define MAXSEQS    2000
#define MaxMatNum  3
#define RGBCHANNEL 1
#define BITS8         0x00000001
#define BITS32        0x00000010
#define BITS64        0x00000100


#define CV_8TO16U_SQR(x)  my8x16uSqrTab[(x)+128]
#define CLR_RESULT_QUEUE() 	result_seq.tail = 0;\
							result_seq.total = 0; 





typedef unsigned char	(*ImgPtr);
typedef unsigned char	(*Mat8Ptr);
typedef int				(*Mat32Ptr);
typedef long long			(*Mat64Ptr);

/*****************并查集数据结构*******************************/
#define MAXPTREENODES 100
typedef struct PTreeNode
{
    struct PTreeNode* parent;
    char* element;
    int rank;
}PTreeNode;

/************************积分图变量***************************/
typedef int sumtype;
typedef long long sqsumtype;

/************************************************************/
typedef struct Rect
{
	   int x;
	   int y;
	   int width;
	   int height;
}Rect;

typedef struct
{
    int width;
    int height;


}Size;

typedef struct Image
{
 	ImgPtr  imgPtr;
	int rows;
	int cols;
}Image;

typedef struct Mat8
{
	Mat8Ptr  mat8Ptr;
	int rows;
	int cols;
}Mat8;
typedef struct Mat32
{
	Mat32Ptr  mat32Ptr;
	int rows;
	int cols;
}Mat32;

typedef struct Mat64
{
	Mat64Ptr  mat64Ptr;
	int rows;
	int cols;
}Mat64;



typedef struct Sequence
{
	int       total; 
	Rect	  rectQueue[MAXSEQS];
	int		  neighbors[MAXSEQS];
	int		  tail;
}Sequence;


//Haar特征的数量
#define CV_HAAR_FEATURE_MAX  3    

/*************HidHaar to Caculation Feature***********************************/
typedef struct HidHaarFeature
{
    struct
    {
        sumtype *p0, *p1, *p2, *p3;
        int weight;
    }
    rect[CV_HAAR_FEATURE_MAX];
}HidHaarFeature;


typedef struct HidHaarTreeNode
{
    HidHaarFeature feature;
    int threshold;
    int left;
    int right;
}HidHaarTreeNode;


typedef struct HidHaarClassifier
{
    int count;
    //CvHaarFeature* orig_feature;

    HidHaarTreeNode* node;
    int* alpha;
	//HidHaarTreeNode node[MAXTREENODE];
    //int alpha[MAXALPHA];
}HidHaarClassifier;

typedef struct HidHaarStageClassifier
{
    int  count;
    int threshold;
    HidHaarClassifier* classifier;
	//HidHaarClassifier classifier[MAXCLASSIFER];
    int two_rects;
    
    struct HidHaarStageClassifier* next;
    struct HidHaarStageClassifier* child;
    struct HidHaarStageClassifier* parent;
}HidHaarStageClassifier;


typedef struct HidHaarClassifierCascade
{
    int  count;
    int  is_stump_based;
    int  has_tilted_features;
    int  is_tree;
    int window_area;
    Mat32* sum;
	Mat64* sqsum;
    HidHaarStageClassifier* stage_classifier;
	//HidHaarStageClassifier stage_classifier[MAXSTAGES];
    sqsumtype *pq0, *pq1, *pq2, *pq3;
    sumtype *p0, *p1, *p2, *p3;
	
    void** ipp_stages;
}HidHaarClassifierCascade;



/******************Haar Cascade*****************************************/
typedef struct HaarFeature
{
    int  tilted;
    struct
    {
        Rect r;
        int weight;
    } rect[CV_HAAR_FEATURE_MAX];
}HaarFeature;

typedef struct HaarClassifier
{
    int count;
    HaarFeature* haar_feature;
    int* threshold;
    int* left;
    int* right;
    int* alpha;
}HaarClassifier;

typedef struct HaarStageClassifier
{
    int  count;
    int threshold;
    HaarClassifier* classifier;
	
    int next;
    int child;
    int parent;
}HaarStageClassifier;


typedef struct HaarClassifierCascade
{
    int  flags;
    int  count;
    Size orig_window_size;
    Size real_window_size;
    int scale32x;
    HaarStageClassifier* stage_classifier;
    HidHaarClassifierCascade* hid_cascade;
}HaarClassifierCascade;



//typedef struct CvAvgComp
//{
//    Rect rect;
//    int neighbors;
//}
//CvAvgComp;


typedef struct Point
{
	int x;
	int y;
}Point;



/******************全局变量****************************************/
//cascade
extern HaarClassifierCascade *cascade_vec[cascade_num];
//extern HidHaarClassifierCascade hid_cascade;

//32bits cell Mat
extern int			  MatPool32[MaxMatNum][MAXROWS][MAXCOLS];
//8bits cell 
extern unsigned char  MatPool8[MaxMatNum][MAXROWS][MAXCOLS];

//8bits*3 cell 
extern unsigned char  ImgRGBPool8[MaxMatNum][RGBCHANNEL][MAXROWS][MAXCOLS];
//64bits float cell 
extern long long	 MatPool64[MaxMatNum][MAXROWS][MAXCOLS];



//分类器检测结果区域序列
extern Sequence result_seq;

/********************全局函数******************************************/
extern void ReadFaceCascade();
extern void ReadLayerConfig();
extern void HaarDetectObjects(Image* _img, char* storage, int scale_factor32x,
							  int min_neighbors, int flags, Size minSize,int tl_x,int tl_y,int beishu);//8.22

extern void Get_roi(Image* _img,const char *src,int tl_x,int tl_y,int roi_width,int roi_height);
void cv_LoadCvMat(  CvMat  *pFilter, uint32_t  nType,  uint32_t  nRows, uint32_t  nCols,  void  *pMask);
void DownSample(Image* pImage, int factor);
#endif
