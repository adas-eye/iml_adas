
#include "Haar.h"

//*************************************12.15版本**********************************
//**************************************************
//**************************
#define PI 3.141592653
#define Original_WIDTH 752
#define Original_HEIGHT 480

#define test_new_algothm 1
#define display_original_lines 0

#define only_near_car 1//1，0 效果好的。。。0，1效果不好的,   1,1最终的

#define add_near_car 1//

#define calib_y 13000//需被4整除
#define calib_multiple 2.5//需被4整除
#define calib_plus 6//需被4整除

#define IPM_WIDTH 108//需被4整除

#define ERROR_CONTROL 15 //9.28
#define GREDIENT_CONTROL 0.3 //9.28
#define FRONT_DISTANCE 6 //9.28

#define IPM_HEIGHT 84//需被2整除 //10.31

#define IPM_WIDTH_4x 270//需被4整除
#define IPM_HEIGHT_4x 252//需被2整除//9.17

#define CALIBRATE_PARA (IPM_WIDTH/90.0)
#define CHOOSELANE_PARA (IPM_WIDTH/64.0)

#define SEGMENT (9*CHOOSELANE_PARA)

#define hat_fearture 45

#define tracker_num 12

#define theta_low 55//11.2
#define theta_high 125

#define r_resolution 2
#define numrho int(IPM_WIDTH/2)//下位机可以直接改成54，initial_cos_sin函数里，可改成常数。可以以上需要保持一致！！！

#define numangle int(theta_high-theta_low)

#define edge_points_count IPM_WIDTH*IPM_HEIGHT/4

#define LOOP_NUM 50

//霍夫变换 分辨率为1，现在不支持为2.


//**************************
//**************************************************
//*************************************12.15版本**********************************

#define ROI_WIDTH_raw (int)((float)(752/zoom_parameter))

#define ROI_HEIGHT int(float(480*(1-roi_parameter)/zoom_parameter))
#define ROI_WIDTH (ROI_WIDTH_raw+8*((ROI_WIDTH_raw % 8)!=0)-(ROI_WIDTH_raw%8))
#define roi_parameter 0.45
#define zoom_parameter 6

//*************************************12.15版本**********************************
//**************************************************
//**************************
typedef struct Point2s
{
	short x;
	short y;
}Point2s;

typedef struct Line_IPM
{
   float angle;
   short rho;
   short val_1;
   short val_2;
   short val_3;
   short val_trust;
   float nsta;//9.28
   short type;
   bool if_valid;
   float cos;
   float sin;
   short val_track;//3.7
   int line_classification;

   Point2s P1,P2;
   Point2s p1,p2;

}Line_IPM;

typedef struct lane_tracker
{
   short state_value;
   Line_IPM pre_lane;
   short kind;
   short invalid_fps;
}lane_tracker;

//**************************
//**************************************************
//*************************************12.15版本**********************************

typedef struct Line
{
   short x1;
   short y1;
   short x2;
   short y2;
   short error;
   bool if_in_roi;

}Line;

struct Lane
{
	Image img_ROI;
	Rect ROI;
	Line pre_lane_left, pre_lane_right,cur_lane_left,cur_lane_right;
	//Vec<int, 5> cur_lane_left, cur_lane_right//,middle_lane,left, right,middle;
	//int fps;
	//int if_lane_f;
	//Mat32 Homo_Mat;    
	bool track_flag,B_flag;
	bool if_last_left_lane_valid;
	bool if_last_right_lane_valid;
	//int change_left,change_right;
	Point P1,P2,P3,P4;

	Line lines[50];

	Rect remove_roi;
};

extern void Canny_xy(Image src,Image dst,
	int low_thresh, int high_thresh,
	int aperture_size=3);

extern int remove_by_ROI(Line* _lines,int count);

extern int HoughLinesP_xy(Image _image, Line* _lines,
	short threshold,
	short minLineLength, short maxGap,short linesMax,float rho=1, float theta=PI/180);

extern int cal_error(Line* _lines,int count);

extern void rank_by_error(Line* _lines,int count);
extern void rank_by_abserror(Line* _lines,int count);

extern int merge(Line* lines,int count);

extern bool if_parallel(Line cur_lane_left,Line cur_lane_right);
extern void resize_and_threshold(unsigned char* src,int src_W,int src_H,unsigned char* dst,int dst_W,int dst_H,int threshold);

//*************************************12.15版本**********************************
//**************************************************
//**************************
extern void initial_sin_cos(float theta,float trho);
extern void GetMatrix(int*,float *);//3.7 314
//extern void GetMatrix_4x(int*,float *);//3.7 314
extern void ImageMatrix(int*t,unsigned char* dst,unsigned char* src);
//extern void ImageMatrix_4x(int*t,unsigned char* dst,unsigned char* src);
extern void HoughLines_accum_IPM(unsigned char* edge_image,int* accum,short height,short width,int y_start);

extern void cal_trust_mat(int* a1,int* a2,int* a3,int* b,int* c,Line_IPM& left_line,Line_IPM& middle_line, Line_IPM& right_line,
						int T1,int T_whole,int type1_val);

extern void choose_lane(Line_IPM left_line,Line_IPM middle_line,Line_IPM right_line,Line_IPM& choosed_left,Line_IPM& choosed_middle,Line_IPM& choosed_right);//9.28
extern void initialize_tracker(int i);
extern void initialize_line(Line_IPM* line);
extern bool judge_if_track_valid(lane_tracker track_para,Line_IPM line);
extern int track_lane(Line_IPM& left,Line_IPM& middle,Line_IPM& right);//9.28
extern int if_warn(Line_IPM left,Line_IPM right,int track_value);
extern int if_departure(int warn,int dep);
extern int new_warning(Line_IPM left,Line_IPM right,int track_value,float &s1,float &s2,float &s3,float &s4,int &departure);//0226版本增加新函数
extern void cal_LDW_perspective_mat(float D,Point xiaoshi,Point p3,Point p4,float *,float *,float);//0307新增，自动小范围调整矩阵。3.7//314
//extern void cal_LDW_perspective_mat_4x(float D,Point xiaoshi,Point p3,Point p4,float *,float);//0307新增，自动小范围调整矩阵。3.7//314
extern void cal_FCW_mat(float D,Point xiaoshi,Point p3,Point p4,float *,float);//314
extern int find_lane_near_edge_point(Line_IPM* line, Point* p1,Point* p2, Point* edge_points,unsigned char* edge_img,int *map_table);//330
extern void find_accurate_lane_position(Line_IPM* line, Point* edge_points,Point* right_edge,Point* right_edge2,unsigned char* edge_img,unsigned char* edge_img2,unsigned char* src_data,int *map_table,int *map_table2,int center_y,Point* P1,Point* P2,int& count,int& count2,int L_or_R,float* inv,short* max_pos1,short* min_pos1,short* max_pos2,short* min_pos2);//330
extern float cal_ZNCC_score(Point *poi,unsigned char* img_4x);
extern bool if_local_max(int i,int j,int* cur_pos);

void choose_lane2(Line_IPM line1,Line_IPM line2,Line_IPM line3,Line_IPM line4,Line_IPM& choosed_left,Line_IPM& choose_middle,Line_IPM& choosed_right);//11.24

void choose_lane3(Line_IPM line1,Line_IPM line2,Line_IPM line3,Line_IPM line4,Line_IPM* choosed_left,Line_IPM* choose_middle,Line_IPM* choosed_right);

extern void cal_trust_mat_gredient2(int* a,Line_IPM& line1,Line_IPM& line2,Line_IPM& line3,Line_IPM& line4,int type1_val,int T_whole);//11.24

extern void HoughLines_accum_IPM_gredient(unsigned char* edge_image, int* accum,short height,short width,int y_start,int valid_count,Point2s* edges);//8,2,1,0-30,150-180
extern void HoughLines_accum_IPM_gredient_near_car(unsigned char* edge_image, int* accum,short height,short width,int y_start,int valid_count,Point2s* edges);//8,2,1,0-30,150-180

extern void cal_trust_mat_gredient(int* a,Line_IPM& left_line,Line_IPM& middle_line, Line_IPM& right_line,int type1_val,int T_whole);//11.24

extern void judge_if_two_lane_valid(Line_IPM line1,Line_IPM line2,int* type,int* val,int pos);//11.24

extern void cal_LDW_perspective_mat_near_car(float D,Point xiaoshi,Point p3,Point p4,float *,float *,float);//0307新增，自动小范围调整矩阵。3.7//314

extern int WriteRawData(unsigned char *data,char *out_path);

void get_IPM_convolution(unsigned char * src_data,unsigned char * IPM_img,unsigned char * dst_data,int* valid_count,Point2s* edges,short* max_pos,short* min_pos,int* map_ta,int center_y,int if_limit);
//int get_calib_plus(); 
//extern void GetMatrix_near_car(int*,float *);//3.7 314
//extern void ImageMatrix_near_car(int*,float *);//3.7 314

//**************************
//**************************************************
//*************************************12.15版本**********************************
