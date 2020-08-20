#include "Haar.h"
#include "Util.h"
#include "stdio.h"
#include "string.h"
#include <math.h>
#include "load_config.h"
//#include "windows.h"
//#include "OpenCV.h"
/*******************Global************************************/

HaarClassifierCascade *cascade_vec[cascade_num];
//HidHaarClassifierCascade hid_cascade;
//32bits cell Mat
int		MatPool32[MaxMatNum][MAXROWS][MAXCOLS];
//8bits cell 
unsigned char  MatPool8[MaxMatNum][MAXROWS][MAXCOLS];

//8bits*3 cell 
unsigned char  ImgRGBPool8[MaxMatNum][RGBCHANNEL][MAXROWS][MAXCOLS];

//unsigned char  ImgRGBPool8_2[MaxMatNum][RGBCHANNEL][MAXROWS][MAXCOLS];

//unsigned char  ImgRGBPool8[MaxMatNum][RGBCHANNEL][MAXROWS][MAXCOLS];
//64bits  cell 
long long	 MatPool64[MaxMatNum][MAXROWS][MAXCOLS];

//候选区域坐标节点并查集
PTreeNode PTreeNodes[MAXPTREENODES];

//char HidCascade[MAXHIDCASCADE];

//分类器检测结果区域序列
Sequence result_seq;


int layerNum = 0;
layerConfig_t *layerConf;

char *HidCascade[3];


//==================================================================
//函数名：  IsEqual
//作者：    qiurenbo
//日期：    2014-10-1
//功能：    判断两个矩形是否邻接
//输入参数：_r1  _r2 候选区域矩形      
//返回值：  返回相似性（是否是邻接的矩形）
//修改记录：
//==================================================================
int IsEqual( const void* _r1, const void* _r2)
{
    const Rect* r1 = (const Rect*)_r1;
    const Rect* r2 = (const Rect*)_r2;
	int distance5x = r1->width ;//int distance = cvRound(r1->width*0.2);
	
    return r2->x*5 <= r1->x*5 + distance5x &&
		r2->x*5 >= r1->x*5 - distance5x &&
		r2->y*5 <= r1->y*5 + distance5x &&
		r2->y*5 >= r1->y*5 - distance5x &&
		r2->width*5 <= r1->width * 6 &&
		r2->width * 6 >= r1->width*5;
}

//==================================================================
//函数名：  ReadFaceCascade
//作者：    qiurenbo
//日期：    2014-10-1
//功能：    根据候选区域的相似性（IsEqual函数）建立并查集
//输入参数：seq  候选目标区域序列      
//返回值：  返回分类后的类别数 
//修改记录：
//==================================================================
int SeqPartition( const Sequence* seq )
{
	Sequence* result = 0;
    //CvMemStorage* temp_storage = 0;
    int class_idx = 0;
    

	memset(PTreeNodes, 0, MAXPTREENODES*sizeof(PTreeNode));
    
    int i, j;
   


    //建立以seq中元素为根节点的森林
    for( i = 0; i < seq->total; i++ )
		PTreeNodes[i].element = (char*)&seq->rectQueue[i];

	//遍历所有根节点
	for( i = 0; i < seq->total; i++ )
    {
        PTreeNode* node = &PTreeNodes[i];
		PTreeNode* root = node;
		//确保node中元素指针不为空
        if( !node->element )
            continue;
		
        //找到元素在树中的根结点
        while( root->parent )
            root = root->parent;
		
        for( j = 0; j < seq->total; j++ )
        {
            PTreeNode* node2 = &PTreeNodes[j];
            
			//确保1.node中元素指针不为空
			//    2.且不是同一个node结点
			//    3.且是相似区域
			// 若是相似区域，则合并元素
            if( node2->element && node2 != node &&
                IsEqual( node->element, node2->element))
            {
                PTreeNode* root2 = node2;
                
               //找到元素在树中的根结点
                while( root2->parent )
                    root2 = root2->parent;
				
				//合并的前提是不在一颗树中
                if( root2 != root )
                {
					//秩小的树归入秩大的树中
                    if( root->rank > root2->rank )
                        root2->parent = root;
					//秩相等的时候才改变树的秩
                    else
                    {
                        root->parent = root2;
                        root2->rank += root->rank == root2->rank;
                        root = root2;
                    }
                    //assert( root->parent == 0 );
					
                    // 路径压缩，子节点node2直接指向根节点
                    while( node2->parent )
                    {
                        PTreeNode* temp = node2;
                        node2 = node2->parent;
                        temp->parent = root;
                    }
					
                    // 路径压缩，子节点node直接指向根节点
                    node2 = node;
                    while( node2->parent )
                    {
                        PTreeNode* temp = node2;
                        node2 = node2->parent;
                        temp->parent = root;
                    }
                }
            }
		
        }
    }


	for( i = 0; i < seq->total; i++ )
    {
        PTreeNode* node = &PTreeNodes[i];
        int idx = -1;
        
        if( node->element )
        {
            while( node->parent )
                node = node->parent;
			
			//计算有几棵并查树，巧妙地利用取反避免重复计算
            if( node->rank >= 0 )
                node->rank = ~class_idx++;
            idx = ~node->rank;
        }
		
       
    }

    return class_idx;
}

void ReadLayerConfig()
{
	char *tmp_str1 = (char*)"./conf/layer_conf.txt";
	char *tmp_str2 = (char*)"./conf/layer_conf.txt";
	read_from_file(tmp_str1, tmp_str2, layerConf, layerNum);
}

static HidHaarClassifierCascade*
	CreateHidHaarClassifierCascade(HaarClassifierCascade* cascade, char *HidCascade);

//==================================================================
//函数名：  ReadFaceCascade
//作者：    qiurenbo
//日期：    2014-09-30
//功能：    读取Cascade文件
//输入参数：void      
//返回值：  void  
//修改记录：
//==================================================================
void ReadFaceCascade()
{
	for (int kk=0; kk<cascade_num; kk++) {
		int i;
		//load cascade
		cascade_vec[kk] = (HaarClassifierCascade*)HaarClassifierCascade_face_vec[kk];

		//load stages
		int stage_size = StageClassifier_face_vec[kk][0];
		HaarStageClassifier *stages ;
		stages = (HaarStageClassifier *)(StageClassifier_face_vec[kk]+1);

		//load classifier
		int classifier_size = Classifier_face1_vec[kk][0];
		HaarClassifier *cls ;
		cls = (HaarClassifier*) (Classifier_face1_vec[kk]+1);

		int class_info_size = class_info_vec[kk][0];
		int * cls_info ;
		cls_info = (int*)(class_info_vec[kk]+1);



		//link cascade with stages
		cascade_vec[kk]->stage_classifier = stages;
		//link stages，classifiers
		int offset=0;
		int offset_t=(sizeof(HaarFeature)/sizeof(int));
		int offset_l=offset_t+1;
		int offset_r=offset_t+2;
		int offset_a=offset_t+3;
		int offset_total=0;
		for(i=0;i<stage_size;++i)
		{
			(stages+i)->classifier = (cls+offset);
			offset +=(stages+i)->count;
		}

		offset_total = 5+ (sizeof(HaarFeature)/sizeof(int));
		//link classifiers and haar_featrue;
		for(i=0;i<classifier_size;++i)
		{
			HaarClassifier *cs= cls+i;

			cs->haar_feature = (HaarFeature*)(cls_info+i*offset_total);
			cs->threshold = (int*)(cls_info+i*offset_total+offset_t);
			cs->left =(int*)(cls_info+i*offset_total+offset_l);
			cs->right=(int*)(cls_info+i*offset_total+offset_r);
			cs->alpha=(int*)(cls_info+i*offset_total+offset_a);

		}
		HidCascade[kk] = (char*)malloc(MAXHIDCASCADE * sizeof(char));
		if( !cascade_vec[kk]->hid_cascade )
			CreateHidHaarClassifierCascade(cascade_vec[kk], HidCascade[kk]);
	}
}

//==================================================================
//函数名：  IntegralImage
//作者：    qiurenbo
//日期：    2014-09-26
//功能：    从矩阵池中获取rows * cols的矩阵 
//输入参数：mat		矩阵结构体地址
//			rows	待分配的行数
//			cols	待分配的列数
//			type    待分配的矩阵类型 
//			matIndex 从矩阵池中分配的矩阵序列（手动指定..）      
//返回值：  void  
//修改记录：
//==================================================================

void GetMat(void* mat, int rows, int cols, int type, int matIndex)
{
	switch(type)
	{
		case BITS8: 
				((Mat8*)mat)->rows = rows;
				((Mat8*)mat)->cols = cols;
				((Mat8*)mat)->mat8Ptr =  (Mat8Ptr)&MatPool8[matIndex];
				break;
			
		case BITS32: 
				((Mat32*)mat)->rows = rows;
				((Mat32*)mat)->cols = cols;
				((Mat32*)mat)->mat32Ptr =  (Mat32Ptr)&MatPool32[matIndex];
				break;

		case BITS64:
				((Mat64*)mat)->rows = rows;
				((Mat64*)mat)->cols = cols;
				((Mat64*)mat)->mat64Ptr =  (Mat64Ptr)&MatPool64[matIndex];
				break;
	}
}


//==================================================================
//函数名：  IntegralImage
//作者：    qiurenbo
//日期：    2014-09-26
//功能：    计算目标检测区域的积分图
//输入参数：src		待检测目标所在矩阵起始
//			srcstep 待检测区域列数
//			sum		积分图矩阵	(W+1)*(H+1)	
//			sumstep 积分图矩阵列数    
//			sqsum	平方和图矩阵 (W+1)*(H+1)	
//			sqsumstep 平方和图矩阵列数
//			size   待检测区域大小 W*H
//          
//          
//返回值：  void  
//修改记录：
//==================================================================
void IntegralImage(ImgPtr src, int srcstep,
				   Mat32Ptr sum, int sumstep,     
				   Mat64Ptr sqsum, int sqsumstep,
				   Size size)
{
	int s = 0;
	long long sq = 0;
	//移动指针到积分图的下一行,第一行全为0
	sum += sumstep + 1;     
	sqsum += sqsumstep + 1; 
	
	//y代表相对于输入检测矩阵起始第几行
	for(int y = 0; y < size.height; y++, src += srcstep,       
		sum += sumstep, sqsum += sqsumstep )    
	{   
		//sum和sqsum为(W+1)*(H+1)大小矩阵，故将第一列置为0
		sum[-1] = 0;                                        
		sqsum[-1] = 0;                                      
		
		for(int x = 0 ; x < size.width; x++ )    
		{                                                   
			int it = src[x];                           
			int t = (it);   
			
			//查表计算平方
			long long	tq =  CV_8TO16U_SQR(it);  
            //s代表行上的累加和
			s += t;  
			//sq代表行上的累加和
			sq += tq;                                       
			t = sum[x - sumstep] + s;                       
			tq = sqsum[x - sqsumstep] + sq;                 
			sum[x] = t;                                     
			sqsum[x] = (long long)tq;                                  
		}                                                   
    }                        
}

//==================================================================
//函数名：  Integral
//作者：    qiurenbo
//日期：    2014-09-26
//功能：    计算目标检测区域的积分图
//输入参数：image 图像
//			sumImage 积分图指针
//			sumSqImage 平方和图指针                 
//返回值：  void  
//修改记录：
//==================================================================
void Integral(Image* image, Mat32* sumImage, Mat64* sumSqImage)
{

	//取保地址空间已经分配，从数组中
	if (image == NULL || sumImage == NULL || sumSqImage == NULL)
		return;

    Image*src    =		(Image*)image;
    Mat32 *sum	 =		(Mat32*)sumImage;
    Mat64 *sqsum =		(Mat64*)sumSqImage;
   
	Size size;
	size.height = src->rows;
	size.width =  src->cols;

    IntegralImage(src->imgPtr, src->cols,
		sum->mat32Ptr, sum->cols,     
		sqsum->mat64Ptr, sqsum->cols,size);
    

}
//==================================================================
//函数名：  AlignPtr
//作者：    qiurenbo
//日期：    2014-10-03
//功能：	按algin字节对齐
//输入参数：ptr 要对齐的指针   
//			align 对齐的字节数           
//返回值：  void*   
//修改记录：
//==================================================================
//void* AlignPtr( const void* ptr, int align)
//{
//
//	return (void*)( ((unsigned int)ptr + align - 1) & ~(align-1) );
//} 
//==================================================================
//函数名：  CreateHidHaarClassifierCascade
//作者：    qiurenbo
//日期：    2014-09-28
//功能：    创建隐式积分图加快计算速度
//输入参数：cascade 级联分类器指针              
//返回值：  static HidHaarClassifierCascade*   返回一个隐式级联分类器指针
//修改记录：
//==================================================================
static HidHaarClassifierCascade*
CreateHidHaarClassifierCascade(HaarClassifierCascade* cascade, char *HidCascade)
{



	
	cascade->hid_cascade = (struct HidHaarClassifierCascade *)HidCascade;
	//分配栈空间
    HidHaarClassifierCascade* out = (struct HidHaarClassifierCascade *)HidCascade;
	const int icv_stage_threshold_bias = 419; //0.0001*(2^22)=419.4304

	HidHaarClassifier* haar_classifier_ptr;
    HidHaarTreeNode* haar_node_ptr;
    int i, j, l;
   
    int total_classifiers = 2135;
    int total_nodes = 0;
    
  
    int has_tilted_features = 0;
    int max_count = 0;




    /* 初始化HidCascade头 */
    out->count = cascade->count;
    out->stage_classifier = (HidHaarStageClassifier*)(out + 1);
	//out->stage_classifier = (HidHaarStageClassifier*)AlignPtr(out + 1, 4);
	//classifier起始地址
	haar_classifier_ptr = (HidHaarClassifier*)(out->stage_classifier + cascade->count);
	//haar_classifier_ptr = (HidHaarClassifier*)AlignPtr(out->stage_classifier + cascade->count, 4);
	//node起始地址
    //haar_node_ptr = (HidHaarTreeNode*)AlignPtr(haar_classifier_ptr + total_classifiers, 4);
	haar_node_ptr = (HidHaarTreeNode*)(haar_classifier_ptr + total_classifiers);
    out->is_stump_based = 1;
    out->is_tree = 0;

    // 用cascade初始化HidCascade
    for( i = 0; i < cascade->count; i++ )
    {

		//用cascades Stage初始化HidCascade的Stage
        HaarStageClassifier* stage_classifier = cascade->stage_classifier + i;
        HidHaarStageClassifier* hid_stage_classifier = out->stage_classifier + i;

        hid_stage_classifier->count = stage_classifier->count;
        hid_stage_classifier->threshold = stage_classifier->threshold - icv_stage_threshold_bias;
        //hid_stage_classifier->classifier = (struct HidHaarClassifier *)&HidClassifiers[i];
		 hid_stage_classifier->classifier = haar_classifier_ptr;
		//初始化为二特征，下面会根据真实的特征数至1或0（三特征）
        hid_stage_classifier->two_rects = 1;
		haar_classifier_ptr += stage_classifier->count;


		//Stage构成一颗退化的二叉树（单分支），每个结点最多只有一个孩子
        hid_stage_classifier->parent = (stage_classifier->parent == -1)
            ? NULL : out->stage_classifier + stage_classifier->parent;
        hid_stage_classifier->next = (stage_classifier->next == -1)
            ? NULL :  out->stage_classifier + stage_classifier->next;
        hid_stage_classifier->child = (stage_classifier->child == -1)
            ? NULL : out->stage_classifier + stage_classifier->child ;
        
		//判断该stage是否为树状结构（多分枝）
        out->is_tree |= hid_stage_classifier->next != NULL;


		//赋值classifer属性
        for( j = 0; j < stage_classifier->count; j++ )
        {
            HaarClassifier* classifier = stage_classifier->classifier + j;
            HidHaarClassifier* hid_classifier = hid_stage_classifier->classifier + j;
            int node_count = classifier->count;
			
            int* alpha_ptr = (int*)(haar_node_ptr + node_count);

			hid_classifier->count = node_count;
		    hid_classifier->node = haar_node_ptr;
            hid_classifier->alpha = alpha_ptr;
           
			//赋值node属性
            for( l = 0; l < node_count; l++ )
            {
                HidHaarTreeNode* node =  hid_classifier->node + l;
                HaarFeature* feature = classifier->haar_feature + l;
                memset( node, -1, sizeof(*node) );
                node->threshold = classifier->threshold[l];
                node->left = classifier->left[l];
                node->right = classifier->right[l];
				
				//对特征数目进行判断,若是三特征，则至two_rects为0
                if( (feature->rect[2].weight) == 0 ||
                    feature->rect[2].r.width == 0 ||
                    feature->rect[2].r.height == 0 )
                    memset( &(node->feature.rect[2]), 0, sizeof(node->feature.rect[2]) );
                else
                    hid_stage_classifier->two_rects = 0;
            }

			//赋值alpha
            memcpy( hid_classifier->alpha, classifier->alpha, (node_count+1)*sizeof(hid_classifier->alpha[0]));
			haar_node_ptr = (HidHaarTreeNode*)(alpha_ptr+node_count + 1);
                

			//判断cascade中的分类器是否是树桩分类器，只有根结点的决策树
            out->is_stump_based &= node_count == 1;
        }
    }


    //cascade->hid_cascade = out;
    //assert( (char*)haar_node_ptr - (char*)out <= datasize );

   



    return out;
}

//==================================================================
//函数名：  SetImagesForHaarClassifierCascade
//作者：    qiurenbo
//日期：    2014-09-29
//功能：    根据尺度调整Haar特征的大小和权重
//输入参数：cascade 级联分类器指针 
//			sum     积分图
//			sqsum   平方和积分图
//			scale32x 尺度             
//返回值：  无
//修改记录：
//==================================================================
void SetImagesForHaarClassifierCascade(HaarClassifierCascade* _cascade, Mat32* sum, Mat64* sqsum, int scale32x)
{



  
    HidHaarClassifierCascade* hidCascade;
    int coi0 = 0, coi1 = 0;
    int i;
    Rect equ_rect;
    int weight_scale;

	//根据尺度获取窗口大小
    _cascade->scale32x = scale32x;
    _cascade->real_window_size.width = (_cascade->orig_window_size.width * scale32x + 16)>>5 ;
    _cascade->real_window_size.height = (_cascade->orig_window_size.height * scale32x +16) >> 5;


	//设置隐式级联分类器的积分图
	hidCascade = _cascade->hid_cascade;
    hidCascade->sum = sum;
    hidCascade->sqsum = sqsum;

	//根据尺度设置积分图起始矩阵的位置
	equ_rect.x = equ_rect.y = (scale32x+16)>>5;    
    equ_rect.width = ((_cascade->orig_window_size.width-2)*scale32x + 16 ) >> 5;   //+0.5是为了四舍五入
    equ_rect.height = ((_cascade->orig_window_size.height-2)*scale32x + 16 ) >> 5;
    weight_scale = equ_rect.width*equ_rect.height;
    hidCascade->window_area = weight_scale; //矩形面积
	
	//获取积分图上起始矩阵四个像素的坐标
    hidCascade->p0 = sum->mat32Ptr + (equ_rect.y) * sum->cols+ equ_rect.x;
    hidCascade->p1 = sum->mat32Ptr + (equ_rect.y) * sum->cols + equ_rect.x + equ_rect.width;
    hidCascade->p2 = sum->mat32Ptr + (equ_rect.y + equ_rect.height) * sum->cols + equ_rect.x;
    hidCascade->p3 = sum->mat32Ptr + (equ_rect.y + equ_rect.height) * sum->cols + equ_rect.x + equ_rect.width;

	//获取平方和积分图上起始矩阵四个像素的坐标
	hidCascade->pq0 = sqsum->mat64Ptr + (equ_rect.y) * sqsum->cols+ equ_rect.x;
    hidCascade->pq1 = sqsum->mat64Ptr + (equ_rect.y) * sqsum->cols+ equ_rect.x + equ_rect.width;
    hidCascade->pq2 = sqsum->mat64Ptr + (equ_rect.y + equ_rect.height) * sqsum->cols+ equ_rect.x;
    hidCascade->pq3 = sqsum->mat64Ptr + (equ_rect.y + equ_rect.height) * sqsum->cols+ equ_rect.x + equ_rect.width;

	//遍历每个Classifer所使用的特征，对它们进行尺度放大，并将改变的值赋给HidCascade，隐式级联分类器
	for( i = 0; i < hidCascade->count; i++ )
    {
        int j, k,l;
        for( j = 0; j < hidCascade->stage_classifier[i].count; j++ )
        {
            for( l = 0; l < hidCascade->stage_classifier[i].classifier[j].count; l++ )
            {
                HaarFeature* feature = &_cascade->stage_classifier[i].classifier[j].haar_feature[0];
				
                HidHaarFeature* hidfeature = &hidCascade->stage_classifier[i].classifier[j].node[0].feature;
				int sum0 = 0, area0 = 0;
                Rect r[3];
				
				
				Rect tr;
                int correction_ratio;
				for( k = 0; k < CV_HAAR_FEATURE_MAX; k++ )
                {
                    if( !hidfeature->rect[k].p0 )
                        break;
					
                    r[k] = feature->rect[k].r;
					
					
					
					//左上角坐标和矩阵长宽都按尺度放大
					tr.x = (r[k].x * scale32x + 16) >> 5;
                    tr.width = (r[k].width * scale32x + 16) >> 5;
                    tr.y = ( r[k].y * scale32x + 16 ) >> 5;
                    tr.height = ( r[k].height * scale32x +16 ) >> 5;
					
					
                    correction_ratio = weight_scale;
					
					//设置矩阵四个顶点在积分图中的位置（为了计算特征方便）
					hidfeature->rect[k].p0 = sum->mat32Ptr + tr.y * sum->cols +  tr.x;
					hidfeature->rect[k].p1 = sum->mat32Ptr + tr.y * sum->cols +  tr.x + tr.width; 
					hidfeature->rect[k].p2 = sum->mat32Ptr + (tr.y + tr.height) *sum->cols +  tr.x; 
                    hidfeature->rect[k].p3 = sum->mat32Ptr + (tr.y + tr.height) *sum->cols +  tr.x + tr.width;
					
					//rect[1] = weight/area， 左移22位是为了避免浮点计算，将权值/检测窗口面积（不断扩大），降低权值
                    hidfeature->rect[k].weight = ((feature->rect[k].weight)<< NODE_THRESHOLD_SHIFT)/(correction_ratio);
					
                    if( k == 0 )
                        area0 = tr.width * tr.height;
					else
                        sum0 += hidfeature->rect[k].weight * tr.width * tr.height;
					
                }
				//rect[0].weight ,权重和特征矩形面积成反比
				hidfeature->rect[0].weight = (int)(-sum0/area0);
				
            } /* l */
        } /* j */
    }

	
};


//==================================================================
//函数名：  RunHaarClassifierCascade
//作者：    qiurenbo
//日期：    2014-09-30
//功能：    在指定窗口范围计算特征
//输入参数：_cascade	级联分类器指针 
//			pt			检测窗口左上角坐标
//			start_stage 起始stage下标   
//返回值：  <=0			未检测到目标或参数有问题
//			1			成功检测到目标
//修改记录：
//====================================================================
int RunHaarClassifierCascade( HaarClassifierCascade* _cascade, Point& pt, int start_stage )
{
                           
    int result = -1;


    int p_offset, pq_offset;
    int i, j;
    long long rectsum, variance_factor;
    int variance_norm_factor;
    HidHaarClassifierCascade* hidCascade;

	if (_cascade == NULL)
		return -1;
  

    hidCascade = _cascade->hid_cascade;
    if( !hidCascade )
		return -1;
	

	//确保矩形的有效性，并防止计算窗口出边界
	if( pt.x < 0 || pt.y < 0 ||
        pt.x + _cascade->real_window_size.width >= hidCascade->sum->cols-2 ||
        pt.y + _cascade->real_window_size.height >= hidCascade->sum->rows-2 )
        return -1;


	//计算特征点在积分图中的偏移，相当于移动窗口
    p_offset = pt.y * (hidCascade->sum->cols) + pt.x;
    pq_offset = pt.y * (hidCascade->sqsum->cols) + pt.x;


	//计算移动后整个窗口的特征值
    rectsum = calc_sum(*hidCascade,p_offset);//*cascade->inv_window_area;
    variance_factor = hidCascade->pq0[pq_offset] - hidCascade->pq1[pq_offset] -
                           hidCascade->pq2[pq_offset] + hidCascade->pq3[pq_offset];
    variance_factor = (variance_factor - (rectsum*rectsum + (hidCascade->window_area>>1))/hidCascade->window_area + (hidCascade->window_area>>1))/hidCascade->window_area;
    variance_norm_factor = int(sqrt(float(variance_factor))+0.5f);//qmath

    if( variance_norm_factor < 0 )
        variance_norm_factor = 1;
	//LARGE_INTEGER t1,t2,tc;
   	//QueryPerformanceFrequency(&tc);
	//QueryPerformanceCounter(&t1);

	//计算每个classifier的用到的特征区域的特征值
	for( i = start_stage; i < hidCascade->count; i++ )
	//for( i = start_stage; i < hidCascade->count; i++ )
	{
 		double stage_sum = 0;
		

		
		//if( hidCascade->stage_classifier[i].two_rects )
		//{
		for( j = 0; j < hidCascade->stage_classifier[i].count; j++ )
		{
		
			HidHaarClassifier* classifier = hidCascade->stage_classifier[i].classifier + j;
			HidHaarTreeNode* node = hidCascade->stage_classifier[i].classifier[j].node;
			double sum, t = node->threshold*variance_norm_factor, a, b;
			
			
			//计算Haar特征
			sum = calc_sum(node->feature.rect[0],p_offset) * node->feature.rect[0].weight;
			sum += calc_sum(node->feature.rect[1],p_offset) * node->feature.rect[1].weight;
			
			//两特征和三特征分开处理
			if( node->feature.rect[2].p0 )
				sum += calc_sum(node->feature.rect[2],p_offset) * node->feature.rect[2].weight;


			a = classifier->alpha[0];
			b = classifier->alpha[1];
			stage_sum += sum < t ? a : b;
		}
		
		if( stage_sum < hidCascade->stage_classifier[i].threshold )
		{
			return -i;
			
		}
	}
    
   //QueryPerformanceCounter(&t2);
   //printf("FeatureDetectTime:%fms\n",(t2.QuadPart - t1.QuadPart)*1000.0/tc.QuadPart);

  

    return 1;
}

//=======================================================================================
//函数名：  HaarDetectObjects
//作者：    qiurenbo
//日期：    2014-09-30
//功能：    在指定图片中查找目标
//输入参数： _img				图片指针	    
//			cascade				级联分类器指针 
//			start_stage			起始stage下标 
//			scale_factor32x		窗口变化尺度倍数 /32
//			min_neighbors		最小临界目标（min_neighbors个以上的候选目标的区域才是最后的目标区域）
//			minSize				目标最小的大小
//返回值：  <=0					未检测到目标或参数有问题
//			1					成功检测到目标
//修改记录：
//=========================================================================================
void HaarDetectObjects(Image* _img,
					char* storage, int scale_factor32x,
                    int min_neighbors, int flags, Size minSize,int tl_x,int tl_y,int beishu)//8.22
{
	
	
	//第一次分类用到的最大stage
	//第二次分类用到的起始stage
	int split_stage = 2;

   // ImgPtr stub, *img =  _img;
	Mat32		sum ;
	Mat64	    sqsum;
	Image	    tmp;

	//检测区域候选队列
	Sequence    seq[cascade_num];
	
	//结果候选队列
	Sequence    seq2;
	
	//并查集合并序列
	Sequence comps;

	for (int k=0; k<cascade_num; k++)
		memset(&seq[k], 0, sizeof(Sequence));
	memset(&comps, 0, sizeof(Sequence));
	memset(&seq2, 0, sizeof(Sequence));
	
    int i;
    
    int factor32x;
    int npass = 2;

    if( !cascade_vec[0] )
       return ;

	//获取积分图和平方和积分图的矩阵
	GetMat(&sum , _img->rows + 1, _img->cols + 1, BITS32, 0);
	GetMat(&sqsum, _img->rows + 1, _img->cols + 1, BITS64, 0);
	GetMat(&tmp, _img->rows, _img->cols, BITS8, 1);

	//将隐式级联分类器写入文件看是否正确
	//WriteCascadeToFile(cascade->hid_cascade);
	//LARGE_INTEGER t1,t2,tc;
	//QueryPerformanceFrequency(&tc);
	//QueryPerformanceCounter(&t1);
    //计算积分图
    Integral(_img, &sum, &sqsum);
	//QueryPerformanceCounter(&t2);
	//printf("SetImageFeatureRunTime:%fms\n",(t2.QuadPart - t1.QuadPart)*1000.0/tc.QuadPart);
	//WriteSumAndSqSumToFile(sum, sqsum);
	//LARGE_INTEGER t1,t2,tc,t4,t5;
	//LONGLONG SetImage = 0;
	//LONGLONG RunImage = 0;
   	//QueryPerformanceFrequency(&tc);
	//QueryPerformanceCounter(&t1);
	//QueryPerformanceCounter(&t2);
	//LONGLONG overhead = t2.QuadPart - t1.QuadPart;
	//	QueryPerformanceCounter(&t1);
	//int count = 0;
	//int count2 = 0;
	//不断调整窗口尺度，直到到达图像边缘(_img->cols-10) ||(_img->rows - 10)
	//并且确保尺度小于3倍(96)
    for( factor32x = 32; factor32x*cascade_vec[0]->orig_window_size.width < (_img->cols - 10)<<5 &&
		factor32x*cascade_vec[0]->orig_window_size.height < (_img->rows - 10)<<5
		&&factor32x<192;
	factor32x = (factor32x*scale_factor32x+16)>>5)
    {
        const int ystep32x = MAX(64, factor32x);

		//调整搜索窗口尺度
        Size win_size;
		win_size.height = (cascade_vec[0]->orig_window_size.height * factor32x + 16)>>5;
		win_size.width = (cascade_vec[0]->orig_window_size.width * factor32x + 16 )>>5;
		
       //pass指扫描次数，stage_offset指第二次扫描时从第几个stage开始
        int pass, stage_offset = 0;
		
		//确保搜索窗口在尺度放大后仍然在图像中
        int stop_height =  ( ((_img->rows - win_size.height)<<5)+ (ystep32x>>1) ) / ystep32x;
		
		//确保搜索窗口大于目标的最小尺寸
        if( win_size.width < minSize.width || win_size.height < minSize.height)
            continue;
		//QueryPerformanceFrequency(&tc);
		//QueryPerformanceCounter(&t1);
		//根据尺度设置隐式级联分类器中的特征和权重，并设置这些特征在积分图中的位置，以加速运算
        SetImagesForHaarClassifierCascade(cascade_vec[0], &sum, &sqsum, factor32x );
		//QueryPerformanceCounter(&t2);
		//SetImage = (t2.QuadPart - t1.QuadPart) - overhead;
		//printf("SetImageFeatureRunTime:%fms\n",(t2.QuadPart - t1.QuadPart)*1000.0/tc.QuadPart);


		//设置粗检测所使用的起始分类器
		cascade_vec[0]->hid_cascade->count = split_stage;
	
		
		//用检测窗口扫描两遍图像:
		//第一遍通过级联两个stage粗略定位目标大致区域，对候选区域进行标定（利用tmp矩阵）
		//第二遍对标定的候选区域进行完整筛选，将候选区域放置到队列中
		for( pass = 0; pass < npass; pass++ )
		{

			for( int _iy = 0; _iy < stop_height; _iy++ )
			{	
				//检测窗口纵坐标步长为2，保持不变
				int iy = (_iy*ystep32x+16)>>5;
				int _ix, _xstep = 1;
				
				//stop_width是指_ix迭代的上限，_ix还要*ystep32x才是真正的窗口坐标
				int stop_width =( ((_img->cols - win_size.width)<<5) +ystep32x/2) / ystep32x;
				unsigned char* mask_row = tmp.imgPtr + tmp.cols* iy;
				
				
				for( _ix = 0; _ix < stop_width; _ix += _xstep )
				{
					
					//检测窗口横坐标按步长为4开始移动，若没有检测到目标，则改变下一次步长为2
					int ix = (_ix*ystep32x+16)>>5; // it really should be ystep
				
					//当前检测窗口左上角坐标
					Point pt;
					pt.x = ix;
					pt.y = iy;

					//粗略检测
					if( pass == 0 )
					{
						
						int result = 0;
						_xstep = 2;
						//QueryPerformanceFrequency(&tc);
						//QueryPerformanceCounter(&t4);
						result = RunHaarClassifierCascade( cascade_vec[0], pt, 0 );
						//count++;
						//QueryPerformanceCounter(&t5);
						//RunImage += (t5.QuadPart - t4.QuadPart) - overhead;
						//printf("RunHaarRunTime:%fms\n",(t2.QuadPart - t1.QuadPart)*1000.0/tc.QuadPart);
						//
						if( result > 0 )
						{
							if( pass < npass - 1 )
								mask_row[ix] = 1;
						
						}
						//没有检测到改变步长为2（看ix的值）
						if( result < 0 )
							_xstep = 1;
					}
					//第二次检测先前粗定位的坐标
					else if( mask_row[ix] )
					{
						//写Mask文件看是否正确
						//WriteMaskRowToFile(tmp);
						/*FILE* fp;
						
						fp = fopen("..//MaskRowResult//maskRow2.txt","a+");
						fprintf(fp, "%d %d\n ", ix, iy);	
						fclose(fp);*/
						//QueryPerformanceFrequency(&tc);
						//QueryPerformanceCounter(&t4);
						
						int result = RunHaarClassifierCascade(cascade_vec[0], pt, stage_offset);
						//QueryPerformanceCounter(&t5);
						//RunImage += (t5.QuadPart - t4.QuadPart) - overhead;
						//count2++;
						//int result = 0;
						if( result > 0 )
						{
						
					
							seq[0].rectQueue[seq[0].tail].height = win_size.height;
							seq[0].rectQueue[seq[0].tail].width = win_size.width;
							seq[0].rectQueue[seq[0].tail].x = ix;
							seq[0].rectQueue[seq[0].tail].y = iy;
							seq[0].total++;
							seq[0].tail++;
						}
						else
							mask_row[ix] = 0;
				}
			}

		
        }

		//因为前两个stage在第一次检测的时候已经用过；
		//第二次检测的时候，从第3个stage开始进行完整的检测
		stage_offset = cascade_vec[0]->hid_cascade->count;
		//cascade->hid_cascade->count = cascade->count;
		cascade_vec[0]->hid_cascade->count = cascade_vec[0]->count > 4 ? 4 : cascade_vec[0]->count;
		//cascade->hid_cascade->count = 15;
	}
 }
	//cascade->hid_cascade->count = cascade->count;
	//QueryPerformanceCounter(&t2);
	//printf("HaarRunTime:%fms\n",(t2.QuadPart - t1.QuadPart)*1000.0/tc.QuadPart);

	int output_layer = 0;
	if (cascade_num >= 2) {
		for (int ii=0; ii<layerNum; ii++) {
			int in_layer = layerConf[ii].inputs;
			int out_layer = layerConf[ii].layerId;
			if (output_layer < out_layer)
				output_layer = out_layer;
			for (int k=0; k<seq[in_layer].total; k++) {
				Point pt;
				pt.x = seq[in_layer].rectQueue[k].x;
				pt.y = seq[in_layer].rectQueue[k].y;
				factor32x = 32 * seq[in_layer].rectQueue[k].width / cascade_vec[in_layer]->orig_window_size.width + 0.5;
				SetImagesForHaarClassifierCascade(cascade_vec[out_layer], &sum, &sqsum, factor32x);
				int result = RunHaarClassifierCascade(cascade_vec[out_layer], pt, 0);
				if (layerConf[ii].modelType == 0) {
					result = result > 0 ? 0 : 1;
				}
				if (result > 0) {
					seq[out_layer].rectQueue[seq[out_layer].tail] = seq[in_layer].rectQueue[k];
					seq[out_layer].total++;
					seq[out_layer].tail++;
				}
			}
		}
	}

	if( min_neighbors != 0 )
    {
	
        //将候选目标按相似度构成并查集
		//返回值代表并查集树的个数
        int ncomp = SeqPartition(&seq[output_layer]);
	
		
		
        //对相邻候选区域进行累加，为计算平均边界做准备
        for( i = 0; i < seq[output_layer].total; i++ )
        {
            Rect r1 = seq[output_layer].rectQueue[i];
			PTreeNode* node = &PTreeNodes[i];
			while(node->parent)
				node = node->parent;
            int idx = (node - PTreeNodes);
           
			
            comps.neighbors[idx]++;
			
            comps.rectQueue[idx].x += r1.x;
            comps.rectQueue[idx].y += r1.y;
            comps.rectQueue[idx].width += r1.width;
            comps.rectQueue[idx].height += r1.height;
        }

		// 计算平均目标边界
        for( i = 0; i < seq[output_layer].total; i++ )
        {
            int n = comps.neighbors[i];

			//只有满足最小临接的结果才是最终结果
            if( n >= min_neighbors )
            {
                Rect* rect = &seq2.rectQueue[seq2.tail];
                rect->x = (comps.rectQueue[i].x*2 + n)/(2*n);
                rect->y = (comps.rectQueue[i].y*2 + n)/(2*n);
				rect->width = (comps.rectQueue[i].width*2 + n)/(2*n);
                rect->height = (comps.rectQueue[i].height*2 + n)/(2*n);
                seq2.neighbors[seq2.tail] = comps.neighbors[i];
				seq2.tail++;
                seq2.total++;
            }
        }

		/*const char* p1="D:\\test.bmp";
		IplImage * img=cvCreateImage(cvSize(752,480),8,1);
		img->imageData = (char*)_img->imgPtr;
		cvSaveImage(p1,img);
		cvReleaseImage(&img);*/

		//从候选矩形中得到最大的矩形
        for( i = 0; i < seq2.total; i++ )
        {
			Rect r1 = seq2.rectQueue[i];
            int r1_neighbor = seq2.neighbors[i];
            int j, flag = 1;
			
            for( j = 0; j < seq2.total; j++ )
            {
                Rect r2 = seq2.rectQueue[j];
				int r2_neighbor = seq2.neighbors[j];
                int distance = (r2.width *2+5)/10;//cvRound( r2.rect.width * 0.2 );
				
                if( i != j &&
                    r1.x >= r2.x - distance &&
                    r1.y >= r2.y - distance &&
                    r1.x + r1.width <= r2.x + r2.width + distance &&
                    r1.y + r1.height <= r2.y + r2.height + distance &&
                    (r2_neighbor > MAX( 3, r1_neighbor ) || r1_neighbor < 3) )
                {
                    flag = 0;
                    break;
                }
            }
			
            if( flag )
            {
				r1.x=beishu*r1.x+tl_x;
				r1.y=beishu*r1.y+tl_y;
				r1.width=beishu*r1.width;
				r1.height=beishu*r1.height;
				result_seq.rectQueue[result_seq.tail] = r1;
				result_seq.tail++;
				result_seq.total++;
            }//8.22
        }
		
	}

	//printf("SetImageTime:%fms\n",SetImage*1000.0/tc.QuadPart);
	//printf("RunImageTime:%fms\n",RunImage*1000.0/tc.QuadPart);
}

void DownSample(Image* pImage, int factor)
{
	int i = 0;
	int j = 0;
	int counti = 0;
	int countj = 0;

	int step = pImage->cols / factor;
	for (i =0; i < pImage->rows; i+= factor)
	{
		countj++;
		for (j =0; j < pImage->cols; j += factor)
		{
			*(pImage->imgPtr + i*step/factor + j/factor) = *(pImage->imgPtr + i*pImage->cols + j);
			counti++;
		}
		counti = 0;
	}
	
	pImage->cols /= factor;
	pImage->rows /= factor;
}
void cv_LoadCvMat(
                      CvMat       *pFilter,
                      uint32_t     nType,
                      uint32_t    nRows,
                      uint32_t    nCols,
                      void        *pMask
                      )
{
	    pFilter->type = 0;
	    pFilter->rows = nRows;//480;
	    pFilter->step = nCols;//752;
//	    pFilter->height = IMG_ROWS;
	    pFilter->cols = nCols;//752;
	   // pFilter->hdr_refcount = 1;
	    //pFilter->refcount = 1;
//	    pFilter->width = IMG_COLS;

	    pFilter->data.ptr = (uchar *)pMask;
}
void Get_roi(Image* _img,const char * srcdata,int tl_x,int tl_y,int roi_width,int roi_height)
{
	uchar * dst_line;
	uchar * src_line;

	_img->cols=roi_width;
	_img->rows=roi_height;
	dst_line =_img->imgPtr;
	src_line = (uchar*)srcdata+tl_x+tl_y*752;
	for(int i=0;i<roi_height;i++)
	{
		for(int j=0;j<roi_width;j++)
			dst_line[j]=src_line[j];
		dst_line +=roi_width;
		src_line +=752;
	}
}
