#include "Haar.h"
//#include "Haar.cpp"
#include "stdio.h"
#include "Util.h"
#include "stdlib.h"
#include "cctv.hpp"
//#include <windows.h>
//#include "OpenCV.h"
//#include "Detect2.h"
//#include "test_score.h"
#include "lane.h"
#include <opencv2/opencv.hpp>  
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <cv.h>   //cv.h OpenCV的主要功能头文件，务必要；
#include <highgui.h>
#include "Detect3.h"

#define SHOW_IMAGE

extern int _width_min;
extern CtTracker_t _ct_tracker;
//#include <sys/time.h>

float s1=0,s2=0,s3=0,s4=0;//0226版本新增全局变量
int departurev=0;//0226版本新增全局变量
int warningnum=0;

struct timeval time_a, time_b;
double time_c;
double time_sum;
int time_cnt;

int main(int argc, char *argv[])
{
	if (argc != 2) {
		printf("Usage: ./iml_adas video_file!\n");
		exit(0);
	}
	//std::vector<cv::Vec4i> lines;
	int lane_test=0;
	int car_test=1;
	int k = 0;
	int count = 0;
	//int i_totle = 0;
	int num_totle = 10;
	//IplImage* historyimage = NULL;

#ifdef SHOW_IMAGE
	cvNamedWindow("src_without_tracking");
	cvNamedWindow("src");
#endif
	int if_adjust_LDW_mat=0;//控制是否需要小范围调整；
	int frame_num_for_calib=300;//调整时所需要的有效帧数，
	float  LDW_mat[9];
	float  inv_LDW_mat[9];

	float  LDW_mat_near_car[9];//4.20
	float  inv_LDW_mat_near_car[9];//12.4
	float  FCW_mat[9];//3.7 314
	float lane_width=3.7;//车道宽。
	float camera_offset=0;//车内看，相机在挡风玻璃左，值为负；右，为正，单位为厘米。
	//int center_x=431;//431,336
	//int center_y=226;
	Point src3,src4;
	int warn_dispay=0;
	int center_x=368;//364;
	int center_y=161;//158;

	Point xiaoshi_point;
	xiaoshi_point.x=center_x;
	xiaoshi_point.y=center_y;
	src3.x=2;
	src3.y=402;
	src4.x=724;
	src4.y=402;
	lane_width=float(366)/100;
	camera_offset=float(101-89)/2; 
	
	cal_LDW_perspective_mat(lane_width,xiaoshi_point,src3,src4,LDW_mat,inv_LDW_mat,camera_offset);//3.7    开头的矩阵也可以在此用点来标定。
	cal_LDW_perspective_mat_near_car(lane_width,xiaoshi_point,src3,src4,LDW_mat_near_car,inv_LDW_mat_near_car,camera_offset);//3.30    开头的矩阵也可以在此用点来标定。//326
	cal_FCW_mat(lane_width,xiaoshi_point,src3,src4,FCW_mat,camera_offset);//3.7    开头的矩阵也可以在此用点来标定。
	Image img;
	Image img_1;
	Image img_2;

	img.imgPtr = (ImgPtr)&ImgRGBPool8[0];

	img_1.imgPtr = (ImgPtr)&ImgRGBPool8[1];
	img_2.imgPtr = (ImgPtr)&ImgRGBPool8[2];

	ReadLayerConfig();
	ReadFaceCascade();
	tracker_create();
	//char* classifypath;
	//classifypath = "\\cjj0917.xml";
	//cascade = (HaarClassifierCascade*)cvLoad(classifypath, 0, 0, 0);
	Point PL1,PL2;
	Point PR1,PR2;
	Point pr1,pr2;  
	Point pl1,pl2;
	time_t t1,t2,tc;
	double Time = 0;

	//cvNamedWindow("src"); 
	//char *imgpath = (char*)"./smallstorage_08.19.10.43.avi";
	char *imgpath = argv[1];
	
	CvCapture *capture = cvCaptureFromAVI(imgpath);
	int totalframes = (int)cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_COUNT); 
	int beishu=1;  
	//for (int i = 5 ; i <= 100; i++)	
	uchar data[360960];

	initial_sin_cos(1,CV_PI / 180);//初始化运行时，只运行一次！**************************陈高灿
	//改
	//1.8 初始化sincos。k

	/*short ROI_width=752;
	short ROI_height=int(float(480*(1-roi_parameter)));
	short zoom_ROI_width=ROI_WIDTH_raw+8*((ROI_WIDTH_raw % 8)!=0)-(ROI_WIDTH_raw%8);
	short zoom_ROI_height=int(float(480*(1-roi_parameter)/zoom_parameter));*/
	int map_table[IPM_WIDTH*IPM_HEIGHT];
	int map_table_near_car[IPM_WIDTH*IPM_HEIGHT];
	//建立标定矩阵映射表
	unsigned char IPM_img[IPM_WIDTH*IPM_HEIGHT];
	unsigned char IPM_img_near_car[IPM_WIDTH*IPM_HEIGHT];

	GetMatrix(map_table,LDW_mat);//3.7，参数调整
	GetMatrix(map_table_near_car,LDW_mat_near_car);//3.7，参数调整
	//求查找表

	for(int i=0;i<tracker_num;i++)
	{
		initialize_tracker(i);
	}
	//Line_IPM lines[50];
	Line_IPM left_line={0,0,0,0,0,0,0,0,0,0,0};
	Line_IPM right_line={0,0,0,0,0,0,0,0,0,0,0};
	Line_IPM middle_line={0,0,0,0,0,0,0,0,0,0,0};

	Line_IPM line1={0,0,0,0,0,0,0,0,0,0,0};
	Line_IPM line2={0,0,0,0,0,0,0,0,0,0,0};
	Line_IPM line3={0,0,0,0,0,0,0,0,0,0,0};
	Line_IPM line4={0,0,0,0,0,0,0,0,0,0,0};

	Line_IPM choosed_left={0,0,0,0,0,0,0,0,0,0,0};
	Line_IPM choosed_right={0,0,0,0,0,0,0,0,0,0,0};
	Line_IPM choosed_middle={0,0,0,0,0,0,0,0,0,0,0};
	bool if_right_valid,if_left_valid;

	int accum4[int(numangle*numrho)];//表征车道线的梯度值
	int trust_val_mat[int(numangle*numrho)];
	int trust_val_mat_bak[int(numangle*numrho)];//信任值

	uchar resize_threshold[IPM_HEIGHT*IPM_WIDTH];//要考虑除尽的情况

	//12.16版本添加
	int if_departure=0;
	int final_warning=0;

	int warn_frame=0;
	//12.16版本添加

	int valid_cali_frame=0;
	float valid_cali_val=0;
	//12.16版本添加

	//生成测试图像！！！！！！！！！！！！！！12.16
	IplImage* src_test = cvCreateImage(cvSize(752,480), 8, 1);

	//uchar test_img[360960];
	//char *data_test=test_img;
	char *data_test=src_test->imageData;

	//for(int y=0;y<480;y++)
	//{
	//	for(int x=0;x<752;x++)
	//	{
	//		if((y+x*234/312-406<0) && (y+x*213/312-385>0))
	//			data_test[x]=200;
	//		else if((y-(x-312)*307/17-172<0) && (y-(x-312)*307/26-172>0))
	//			data_test[x]=200;
	//		else if((y-(x-312)*271/439-172<0) && (y-(x-312)*245/439-172>0))
	//			data_test[x]=200;
	//		else
	//			data_test[x]=50;
	//	}
	//	data_test=data_test+752;
	//}
	////生成测试图像！！！！！！！！！！！！！！12.16

	//cvShowImage("test",src_test);
	//int y=cvWaitKey(10000);

	int _cnt = 0;
	int valid_count=0;//11.27
	int valid_count_near_car=0;//11.27
	Point2s edge_points[edge_points_count];//11.27
	Point2s edge_points_near_car[edge_points_count];//11.27

	short max_pos[IPM_HEIGHT*IPM_WIDTH];
	short min_pos[IPM_HEIGHT*IPM_WIDTH];
	short max_pos_near_car[IPM_HEIGHT*IPM_WIDTH];
	short min_pos_near_car[IPM_HEIGHT*IPM_WIDTH];
	while (totalframes--)  
	{

		_cnt++;
		cvGrabFrame(capture);
		IplImage* imgframe = cvRetrieveFrame(capture, 0);
//		if(totalframes%2==1)
//			continue;

		if(totalframes >= 2230)
			continue;

		//车道线
		//IplImage* src = cvCreateImage(cvSize(imgframe->width,imgframe->height), 8, 1);
		//
		//cvCvtColor(imgframe,src,CV_BGR2GRAY);

		//char* src_data=(char *)src->imageData;//拿到src数据，以后都用这个
		//车道线

		//车辆
		IplImage* src = cvCreateImage(cvSize(imgframe->width,imgframe->height), 8, 1);
		//
		cvCvtColor(imgframe,src,CV_BGR2GRAY);
		char* src_data=(char *)(src->imageData);//拿到src数据，以后都用这个

		//车辆

		//char* src_data=(char *)first_img.data;
		//cv::imshow("read",first_img);

	//	char roidata[752*int(float(480*(1-roi_parameter)))];
		time_t t1, t2, tc;
		float time_use=0;
//		struct timeval start;
//		struct timeval end;
		//double Time = 0;
		time(&tc);
		time(&t1);

#if 1
		//****************************************************************************
		if(lane_test)
		{
			cvShowImage("read",src);

			//统计运行时间



			//***************************下位机至此开始*************************************//
			//***************************12.15版本*****************************************


			ImageMatrix(map_table,IPM_img,(uchar*)src_data);//求俯视图
			ImageMatrix(map_table_near_car,IPM_img_near_car,(uchar*)src_data);//求俯视图4.20

			cv::Mat gray_src_near_car=cv::Mat(IPM_HEIGHT,IPM_WIDTH,CV_8U);
			gray_src_near_car.data=IPM_img_near_car;


			cv::imshow("near_car",gray_src_near_car);
			//***************下面部分应使用adi函数（在注释内容里），当前没注释的为opencv版本*****************************
			//******************************************************
			//***************************12.15

			//uchar out1[IPM_HEIGHT*IPM_WIDTH];//结果存入此部分
			//int mask1={1,1,1,0,0,0,0,0,0};
			//adi_conv2D3by3_8(IPM_img,IPM_HEIGHT,IPM_WIDTH,mask1,out1,2); //移2位 除以4.

			//uchar out2[IPM_HEIGHT*IPM_WIDTH];//结果存入此部分
			//int mask2={0,0,0,1,1,1,0,0,0};
			//adi_conv2D3by3_8(IPM_img,IPM_HEIGHT,IPM_WIDTH,mask2,out2,2); //移2位 除以4.
			//
			//uchar out3[IPM_HEIGHT*IPM_WIDTH];//结果存入此部分
			//int mask3={0,0,0,0,0,0,1,1,1};
			//adi_conv2D3by3_8(IPM_img,IPM_HEIGHT,IPM_WIDTH,mask3,out3,2); //移2位 除以4.

			//uchar threshold_hat[IPM_HEIGHT*IPM_WIDTH];
			//short sub1,sub2,plus;
			//for(int i=0;i<IPM_HEIGHT*IPM_WIDTH;i++)
			//{
			//	short sub1=out2[i]-out1[i];
			//	short sub2=out2[i]-out3[i];
			//	if(sub1>=0 && sub2>=0)
			//	{
			//		plus=sub1+sub2;
			//		if(plus>(hat_fearture>>2))
			//			threshold_hat[i]=255;
			//		else
			//			threshold_hat[i]=0;
			//	}
			//	else
			//		threshold_hat[i]=0;
			//}
			//uchar *img_part=threshold_hat;


			//以下为opencv版的，求卷积。

			cv::Mat gray_src=cv::Mat(IPM_HEIGHT,IPM_WIDTH,CV_8U);
			gray_src.data=IPM_img;
			cv::Mat threshold_hat= cv::Mat(IPM_HEIGHT, IPM_WIDTH, CV_8U);
			cv::Mat threshold_hat_near_car= cv::Mat(IPM_HEIGHT, IPM_WIDTH, CV_8U);
#if test_new_algothm

#if only_near_car
			get_IPM_convolution((uchar*)src_data,IPM_img,threshold_hat.data,&valid_count,edge_points,max_pos,min_pos,map_table,center_y,1);
#endif
#if add_near_car
			get_IPM_convolution((uchar*)src_data,IPM_img_near_car,threshold_hat_near_car.data,&valid_count_near_car,edge_points_near_car,max_pos_near_car,min_pos_near_car,map_table_near_car,center_y,0);
#endif
#else
			for(int i=0;i<IPM_HEIGHT;i++)
#endif
			cv::Mat M_src(src);
			cv::Mat color_src(src);
			cv::cvtColor(M_src,color_src,CV_GRAY2BGR);
			uchar *img_part=threshold_hat.data;
#if test_new_algothm
			uchar *img_threshold_hat=threshold_hat.data;//9.23
			uchar *img_threshold_hat_near_car=threshold_hat_near_car.data;
#if only_near_car
			HoughLines_accum_IPM_gredient(img_threshold_hat,accum4,84,IPM_WIDTH,0,valid_count,edge_points);//8.26
#else
			memset(accum4,0,sizeof(int)*numangle*numrho);
#endif
#if add_near_car
			HoughLines_accum_IPM_gredient_near_car(img_threshold_hat_near_car,accum4,84,IPM_WIDTH,0,valid_count_near_car,edge_points_near_car);//8.26
#endif
#else 
				uchar *img_part1=img_part+0*IPM_WIDTH;//9.23
				//*******************************************************
				//**********************************************************************************
				//**************opencv版的求卷积结束，下面为通用部分！！**********************************12.15版本

				uchar *img_part1=img_part;
				uchar *img_part2=img_part+IPM_HEIGHT/3*IPM_WIDTH;
				uchar *img_part3=img_part+2*IPM_HEIGHT/3*IPM_WIDTH;

				HoughLines_accum_IPM(img_part1,accum1,IPM_HEIGHT/3,IPM_WIDTH,0);
				HoughLines_accum_IPM(img_part2,accum2,IPM_HEIGHT/3,IPM_WIDTH,IPM_HEIGHT/3);
				HoughLines_accum_IPM(img_part3,accum3,IPM_HEIGHT/3,IPM_WIDTH,2*IPM_HEIGHT/3);
#endif
				initialize_line(&left_line);
			initialize_line(&middle_line);
			initialize_line(&right_line);

			initialize_line(&line1);
			initialize_line(&line2);
			initialize_line(&line3);
			initialize_line(&line4);
			//初始化
			//if(totalframes>2500)
 		//		continue;
			

			//imshow("src2",M_src);


#if test_new_algothm
			//cal_trust_mat_gredient(accum4,left_line,middle_line,right_line,1,1);

			cal_trust_mat_gredient2(accum4,line1,line2,line3,line4,1,1);
#else 
			cal_trust_mat(accum1,accum2,accum3,trust_val_mat,trust_val_mat_bak,
				left_line,middle_line,right_line,
 				3,52,180);//4.20
#endif
			initialize_line(&choosed_left);
			initialize_line(&choosed_middle);
			initialize_line(&choosed_right);

			//choose_lane(left_line,middle_line,right_line,choosed_left,choosed_middle,choosed_right);
			choose_lane3(line1,line2,line3,line4,&choosed_left,&choosed_middle,&choosed_right);
			Point edge_pts[200];
			Point right_edge_pts[200];
			Point right_edge_pts2[200];//12.11
			if(choosed_left.if_valid)
			{
				Point PL1,PL2;
				int count=0,count2=0;
				//left_count=find_lane_near_edge_point(&choosed_left,&start_left,&start_right,edge_pts,threshold_hat.data,map_table);

				find_accurate_lane_position(&choosed_left,edge_pts ,right_edge_pts,right_edge_pts2,threshold_hat.data,threshold_hat_near_car.data,(uchar*)src_data,map_table,map_table_near_car,center_y,&PL1,&PL2,count,count2,0,inv_LDW_mat,max_pos,min_pos,max_pos_near_car,min_pos_near_car);

				for(int i=0;i<count;i++)
					cv::circle(color_src,cv::Point(right_edge_pts[i].x,right_edge_pts[i].y),1,cv::Scalar(0,0,255));

				for(int i=0;i<count2;i++)
					cv::circle(color_src,cv::Point(right_edge_pts2[i].x,right_edge_pts2[i].y),2,cv::Scalar(255,0,0),2);
		
				if(choosed_left.if_valid)
					cv::line(color_src,cv::Point(choosed_left.P1.x,choosed_left.P1.y),cv::Point(choosed_left.P2.x,choosed_left.P2.y),cv::Scalar(255,0,0),1);
			}

			if(choosed_middle.if_valid)
			{
				Point PL1,PL2;
				int count=0,count2=0;
				//left_count=find_lane_near_edge_point(&choosed_left,&start_left,&start_right,edge_pts,threshold_hat.data,map_table);
				 
				find_accurate_lane_position(&choosed_middle,edge_pts ,right_edge_pts,right_edge_pts2,threshold_hat.data,threshold_hat_near_car.data,(uchar*)src_data,map_table,map_table_near_car,center_y,&PL1,&PL2,count,count2,0,inv_LDW_mat,max_pos,min_pos,max_pos_near_car,min_pos_near_car);

				for(int i=0;i<count;i++)
					cv::circle(color_src,cv::Point(right_edge_pts[i].x,right_edge_pts[i].y),1,cv::Scalar(0,0,255));

				for(int i=0;i<count2;i++)   
					cv::circle(color_src,cv::Point(right_edge_pts2[i].x,right_edge_pts2[i].y),2,cv::Scalar(255,0,0),2);
				
				if(choosed_middle.if_valid)
					cv::line(color_src,cv::Point(choosed_middle.P1.x,choosed_middle.P1.y),cv::Point(choosed_middle.P2.x,choosed_middle.P2.y),cv::Scalar(255,0,0),1);
			}

			if(choosed_right.if_valid)
			{
				Point PL1,PL2;
				int count=0,count2=0;
				//left_count=find_lane_near_edge_point(&choosed_left,&start_left,&start_right,edge_pts,threshold_hat.data,map_table);

				find_accurate_lane_position(&choosed_right,edge_pts ,right_edge_pts,right_edge_pts2,threshold_hat.data,threshold_hat_near_car.data,(uchar*)src_data,map_table,map_table_near_car,center_y,&PL1,&PL2,count,count2,1,inv_LDW_mat,max_pos,min_pos,max_pos_near_car,min_pos_near_car);

				for(int i=0;i<count;i++)
					cv::circle(color_src,cv::Point(right_edge_pts[i].x,right_edge_pts[i].y),1,cv::Scalar(0,0,255));

				for(int i=0;i<count2;i++)
					cv::circle(color_src,cv::Point(right_edge_pts2[i].x,right_edge_pts2[i].y),2,cv::Scalar(255,0,0),2);
		
				if(choosed_right.if_valid)
					cv::line(color_src,cv::Point(choosed_right.P1.x,choosed_right.P1.y),cv::Point(choosed_right.P2.x,choosed_right.P2.y),cv::Scalar(255,0,0),1);
			}

			int _track=track_lane(choosed_left,choosed_middle,choosed_right);

				//生成测试图像！！！！！！！！！！！！！！12.16
			int warn=new_warning(choosed_left,choosed_right,_track,s1,s2,s3,s4,departurev);//0226版本添加
			final_warning=warn;
				//注意：warn=1，右侧报警，warn=-1，左侧报警！
				//warn=-2,没找到车道线
				//跟踪没达到要求，=-2;
				//-3，骑线。

				cv::Mat cwarn=cv::Mat(IPM_HEIGHT, IPM_WIDTH, CV_8U,cvScalarAll(0));//0226,使用了新图显示报警效果
			if(_track>=8&&choosed_left.if_valid && choosed_right.if_valid)// 双线模式0227
			{
				if(warn==1 ||warn_dispay>0)//右报警
				{
					if(warn_dispay==0)
						warn_dispay=20;
					else
						warn_dispay--;//存在bug
					//cv::line(cwarn,cv::Point(80,0),cv::Point(80,80),cvScalar(255,0,0,0),2);
					
					cv::line(color_src,cv::Point(650,50),cv::Point(650,250),cvScalar(0,0,255,0),15);
					warningnum=0;
				}
				if(warn == -1 || warn_dispay<0)//左报警
				{
					if(warn_dispay==0)
						warn_dispay=-20;
					else
						warn_dispay++;
					//cv::line(cwarn,cv::Point(10,0),cv::Point(10,80),cvScalar(255,0,0,0),2);
					cv::line(color_src,cv::Point(100,50),cv::Point(100,250),cvScalar(0,0,255,0),15);
					warningnum=0;
				}
			}
			else
				warn_dispay=0;


				//与if_departure联合，判断是否是 车道中行驶-偏离车道。骑线及切换判断如下：
				//由变道到骑线行驶只报第一次
				//刚切回来又切回去也只报第一次！
				//刚开始就骑线，不报警。不骑线，偏离的时候报警。

				//骑线时长时间找到的是左右双线，单角度不偏，长时间骑线，不会报警。

				//*********************下位机至此结束，后面为opencv版的上位机画图****************************
				//生成测试图像！！！！！！！！！！！！！！12.16
				//计算运行一帧的时间，下位机不要管。
				time(&t2);
				Time += difftime(t2, t1);
				//printf(":%fms\n", (t2.QuadPart - t1.QuadPart)*1000.0 / tc.QuadPart);
				//计算运行一帧的时间，下位机不要管。

				//*****************************************************
				//*****************************************************
				//*****************************************************
				//画图
#if 0
				cv::Mat show_lines= cv::Mat(IPM_HEIGHT, IPM_WIDTH, CV_8U,cvScalarAll(0));
				//show_lines
				cv::Mat color_lines;

				cv::cvtColor(show_lines,color_lines,CV_GRAY2BGR);

				cv::Point p1,p2;

				if(choosed_left.if_valid)
				{
					p1= cv::Point(choosed_left.nsta,0);
					p2= cv::Point((choosed_left.rho-80*choosed_left.cos)/choosed_left.sin,80);
					if(choosed_left.type==1)
						cv::line(color_lines,p1,p2,cvScalar(0,255,0,0),1);
					if(choosed_left.type==2)
						cv::line(color_lines,p1,p2,cvScalar(255,0,0,0),1);
				}

				if(choosed_right.if_valid)
				{
					p1= cv::Point(choosed_right.nsta,0);
					p2= cv::Point((choosed_right.rho-80*choosed_right.cos)/choosed_right.sin,80);
					if(choosed_right.type==1)
						cv::line(color_lines,p1,p2,cvScalar(0,255,0,0),1);
					if(choosed_right.type==2)
						cv::line(color_lines,p1,p2,cvScalar(255,0,0,0),1);
				}
				std::cout<<"warn_after: "<<final_warning<<std::endl;
				std::cout<<"dep："<<if_departure<<std::endl;
				std::cout<<"正在读取："<<totalframes<<"帧"<<std::endl;

				if(final_warning==-2)
					cv::line(color_lines, cv::Point(3,0), cv::Point(3,80),cvScalar(0,0,255,0),1);
				if(final_warning==2)
					cv::line(color_lines, cv::Point(IPM_WIDTH-2,0), cv::Point(IPM_WIDTH-2,80),cvScalar(0,0,255,0),1);

				imshow("resluts",color_src);

				int t=cvWaitKey(200);
				if(t==32)
					int mm=cvWaitKey(10000000);

#endif
				int line_src_copy=70*752+376-90-90;
				int line_IPM_img=0;
				for(int i=0;i<IPM_HEIGHT;i++)
				{
					for(int j=0;j<IPM_WIDTH;j++)
						src_data[line_src_copy+j]=IPM_img[line_IPM_img+j];
					line_src_copy +=Original_WIDTH;
					line_IPM_img+=IPM_WIDTH;
				}
			
				line_src_copy=70*752+376-45;
				line_IPM_img=0;
				for(int i=0;i<IPM_HEIGHT;i++)
				{ 
					for(int j=0;j<IPM_WIDTH;j++)
						src_data[line_src_copy+j]=img_part[line_IPM_img+j];
					line_src_copy +=Original_WIDTH;
					line_IPM_img+=IPM_WIDTH;
				}

				src_data[360959]=final_warning;
				src_data[360958]=warn;
				src_data[360957]=if_departure;

				if(choosed_right.if_valid)
					src_data[360956]=1;
				else
					src_data[360956]=0;
				src_data[360955]=choosed_right.nsta;
				src_data[360954]=choosed_right.angle;
				//src_data[360953]=right_line.rho;

				if(choosed_left.if_valid)
					src_data[360953]=1;
				else
					src_data[360953]=0;
				src_data[360951]=choosed_left.nsta;
				src_data[360950]=choosed_left.angle;
				pl1.x=pl1.y=pl2.x=pl2.y=0;
				int map_table_index;
				pl1.x=choosed_left.p1.x;
				pl1.y=choosed_left.p1.y;
				pl2.x=choosed_left.p2.x;
				pl2.y=choosed_left.p2.y;

				pr1.x=choosed_right.p1.x;
				pr1.y=choosed_right.p1.y;
				pr2.x=choosed_right.p2.x;
				pr2.y=choosed_right.p2.y;

				PL1.x=choosed_left.P1.x;
				PL1.y=choosed_left.P1.y;
				PL2.x=choosed_left.P2.x;
				PL2.y=choosed_left.P2.y;

				PR1.x=choosed_right.P1.x;
				PR1.y=choosed_right.P1.y;
				PR2.x=choosed_right.P2.x;
				PR2.y=choosed_right.P2.y;

				src_data[360949]=(char)pl1.x;
				src_data[360948]=(char)pl1.y;
				src_data[360947]=(char)pl2.x;
				src_data[360946]=(char)pl2.y;

				src_data[360945]=(char)pr1.x;
				src_data[360944]=(char)pr1.y;
				src_data[360943]=(char)pr2.x;
				src_data[360942]=(char)pr2.y;

				*(short*)(&src_data[360940])=(short)PL1.x;
				*(short*)(&src_data[360938])=(short)PL1.y;
				*(short*)(&src_data[360936])=(short)PL2.x;
				*(short*)(&src_data[360934])=(short)PL2.y;

				*(short*)(&src_data[360932])=(short)PR1.x;
				*(short*)(&src_data[360930])=(short)PR1.y;
				*(short*)(&src_data[360928])=(short)PR2.x;
				*(short*)(&src_data[360926])=(short)PR2.y;

				src_data[360924]=(char)choosed_left.val_track;
				src_data[360923]=(char)choosed_right.val_track;
				cv::Point shang_pl1,shang_pl2,shang_pr1,shang_pr2;
				cv::Point shang_PL1,shang_PL2,shang_PR1,shang_PR2;

				shang_pl1.x=src_data[360949];
				shang_pl1.y=src_data[360948];
				shang_pl2.x=src_data[360947];
				shang_pl2.y=src_data[360946];

				shang_pr1.x=src_data[360945];
				shang_pr1.y=src_data[360944];
				shang_pr2.x=src_data[360943];
				shang_pr2.y=src_data[360942];

				shang_PL1.x=*(short*)(&src_data[360940]);
				shang_PL1.y=*(short*)(&src_data[360938]);
				shang_PL2.x=*(short*)(&src_data[360936]);
				shang_PL2.y=*(short*)(&src_data[360934]);

				shang_PR1.x=*(short*)(&src_data[360932]);
				shang_PR1.y=*(short*)(&src_data[360930]);
				shang_PR2.x=*(short*)(&src_data[360928]);
				shang_PR2.y=*(short*)(&src_data[360926]);

				line_src_copy=70*752+376+90;
				for(int i=0;i<IPM_HEIGHT;i++)
				{ 
					for(int j=0;j<IPM_WIDTH;j++)
						src_data[line_src_copy+j]=0;
					line_src_copy +=Original_WIDTH;
				}

				cv::Mat color_lines= cv::Mat(IPM_HEIGHT, IPM_WIDTH, CV_8UC3,cvScalarAll(0));
				cv::Mat color_lines2= cv::Mat(IPM_HEIGHT, IPM_WIDTH, CV_8UC3,cvScalarAll(0));
				cv::Mat color_lines3= cv::Mat(IPM_HEIGHT, IPM_WIDTH, CV_8UC3,cvScalarAll(0));
				cv::Mat color_lines4= cv::Mat(IPM_HEIGHT, IPM_WIDTH, CV_8UC3,cvScalarAll(0));
				//cv::cvtColor(show_lines,color_lines,CV_GRAY2BGR);
				cv::Mat color_src_IPM;
				cv::cvtColor(gray_src,color_src_IPM,CV_GRAY2BGR);
				cv::imshow("gray_src",gray_src);
				cv::imshow("near222",gray_src_near_car);

				if(src_data[360953])
				{
					//cv::line(color_src,shang_pl1,shang_pl2,cvScalar(0,255,0,0),1);
					//if(choosed_left.val_track>=4)
					{
						if(final_warning==-1)
						{
							cv::line(color_src,cv::Point(shang_PL1.x,shang_PL1.y),cv::Point(shang_PL2.x,shang_PL2.y),cvScalar(0,0,255,0),8);
							cv::line(color_src_IPM,cv::Point(shang_pl1.x,pl1.y),cv::Point(shang_pl2.x,shang_pl2.y),cvScalar(0,0,255,0),1);
						}
						else
						{
							cv::line(color_src,cv::Point(shang_PL1.x,shang_PL1.y),cv::Point(shang_PL2.x,shang_PL2.y),cvScalar(0,255,0,0),1);
							cv::line(color_src_IPM,cv::Point(shang_pl1.x,pl1.y),cv::Point(shang_pl2.x,shang_pl2.y),cvScalar(0,255,0,0),1);
						}
					}
					/*else
					{
						if(final_warning==-2)
							cv::line(color_src,cv::Point(PL1.x,PL1.y),cv::Point(PL2.x,PL2.y),cvScalar(255,255,0,0),8);
						else
							cv::line(color_src,cv::Point(PL1.x,PL1.y),cv::Point(PL2.x,PL2.y),cvScalar(255,0,0,0),4);
					}*/
				}

				if(src_data[360956])
				{
					//cv::line(color_src,shang_pr1,shang_pr2,cvScalar(0,255,0,0),1);
					//if(choosed_right.val_track>=4)
					{
						if(final_warning==1)
						{
							cv::line(color_src,cv::Point(shang_PR1.x,shang_PR1.y),cv::Point(shang_PR2.x,shang_PR2.y),cvScalar(0,0,255,0),8);
							cv::line(color_src_IPM,cv::Point(shang_pr1.x,shang_pr1.y),cv::Point(shang_pr2.x,shang_pr2.y),cvScalar(0,0,255,0),1);
						}
						else
						{
							cv::line(color_src,cv::Point(shang_PR1.x,shang_PR1.y),cv::Point(shang_PR2.x,shang_PR2.y),cvScalar(0,255,0,0),1);
							cv::line(color_src_IPM,cv::Point(shang_pr1.x,shang_pr1.y),cv::Point(shang_pr2.x,shang_pr2.y),cvScalar(0,255,0,0),1);
						}
							
					}
					/*else
					{
		 				if(final_warning==2)
							cv::line(color_src,cv::Point(PR1.x,PR1.y),cv::Point(PR2.x,PR2.y),cvScalar(255,255,0,0),8);
						else
							cv::line(color_src,cv::Point(PR1.x,PR1.y),cv::Point(PR2.x,PR2.y),cvScalar(255,0,0,0),4);
					} */ 
				}

				cv::Mat r22;
				cv::resize(color_src_IPM,r22,cv::Size(IPM_WIDTH*3,IPM_HEIGHT*3));
				cv::imshow("ipm_LINES.bmp",color_src_IPM);
				//cv::imshow("read2",r22);
			
				//cv::imshow("read2",r22);
				cv::Mat r11;
				cv::resize(color_src,r11,cv::Size(288,180));
				cv::imshow("read",color_src);
			
				//cvReleaseImage(&imgframe);  
				//cvReleaseImage(&src);  
				//cvWaitKey(0);
				//cvShowImage("read",src);
	//			std::cout<<"正在读取："<<totalframes<<"帧"<<std::endl;
 				int t=cvWaitKey(1);
				//if(t==32)
				//	int mm=cvWaitKey(100000);
#endif			
		}

		if(car_test)
		{
			if (totalframes == 698)
				int a = 0;
			char out_path[256];
			sprintf(out_path, "..\\raw\\%d.raw",694-totalframes);
			memcpy(data, src->imageData, /*3**/(src->width*src->height)*sizeof(unsigned char));
//			gettimeofday(&start,NULL);
			//WriteRawData(data,out_path);

			Size minSize;
			minSize.height = 28;
			minSize.width =  28;

			Size minSize2;
			minSize2.height = 38;
			minSize2.width =  38;

			Size minSize4;
			minSize4.height = 50;
			minSize4.width =  50;

			//time(&tc);
		//	time(&t1);

			//************************第一次
			if(1)
			{
				beishu=4;
				int height_4=400;
				int width_4=600;
				int tl_4x=center_x-width_4/2;

				int tl_4y=center_y-100;
				if(height_4+tl_4y>480)
					tl_4y=(480-height_4)/2-2;

				Get_roi(&img,src->imageData,tl_4x,tl_4y,width_4,height_4);

				//CvRect roi_4=cvRect(tl_4x,tl_4y,img.cols,img.rows);
				//cvSetImageROI(src,roi_4);
				//IplImage *src_roi4=cvCreateImage(cvSize(img.cols,img.rows),8,1);
				//cvCopy(src,src_roi4,0);
				////src(roi);       
				//memcpy(src_roi4->imageData, img.imgPtr, img.rows*img.cols*sizeof(unsigned char));

				DownSample(&img, beishu);
				if(totalframes==680)
					int x=1;
				HaarDetectObjects(&img, NULL, 34, 1, 0, minSize4,tl_4x,tl_4y,beishu);//8.22

				//if(result_seq.total != 0)
				//{
				//	char path[100];
				//	for(int k=0;k<result_seq.total;k++)
				//	{
				//		Rect rect = result_seq.rectQueue[k];
				//		//cvRectangle(imgframe, cvPoint(beishu*rect.x+tl_4x,beishu* rect.y+tl_4y), cvPoint(beishu*rect.x + beishu*rect.width+tl_4x, beishu*rect.y + beishu*rect.height+tl_4y), cvScalar(0, 0, 255,0), 2,2,0);
				//	}
				//}
				cvRectangle(imgframe, cvPoint(tl_4x,tl_4y), cvPoint(beishu*img.cols+tl_4x, beishu*img.rows+tl_4y), cvScalar(0, 255, 255,0), 2,2,0);
			}
			//************************第一次

			//************************第二次
			if(1)
			{
				beishu=2;

				int height_2 =200;//120
				int width_2 =260;
				int tl_2x=center_x-width_2/2;
				int tl_2y=center_y-35;//140


				//CvRect roi_2=cvRect(tl_2x,tl_2y,img_1.cols,img_1.rows);
				//cvSetImageROI(src,roi_2);
				//IplImage *src_roi2=cvCreateImage(cvSize(img_1.cols,img_1.rows),8,1);
				//cvCopy(src,src_roi2,0);
				//memcpy(img_1.imgPtr, src_roi2->imageData, img_1.rows*img_1.cols*sizeof(unsigned char));

				Get_roi(&img_1,src->imageData,tl_2x,tl_2y,width_2,height_2);
				DownSample(&img_1, beishu);
				HaarDetectObjects(&img_1, NULL, 40, 1, 0, minSize2,tl_2x,tl_2y,beishu);//8.22

				//tl_2x=0;
				//tl_2y=0;
				//    cvRectangle(imgframe, cvPoint(175,180), cvPoint(175+img_1.cols*beishu, 180+img_1.   rows*beishu), cvScalar(255, 255, 0,0), 2,2,0);

				//if(result_seq.total != 0)
				//{
				//	char path[100];
				//	for(int k=0;k<result_seq.total;k++)
				//	{
				//		Rect rect = result_seq.rectQueue[k];
				//		//cvRectangle(imgframe, cvPoint(beishu*rect.x+tl_2x,beishu* rect.y+tl_2y), cvPoint(beishu*rect.x + beishu*rect.width+tl_2x, beishu*rect.y + beishu*rect.height+tl_2y), cvScalar(0, 255, 0,0), 2,2,0);
				//	}
				//}
				cvRectangle(imgframe, cvPoint(tl_2x,tl_2y), cvPoint(beishu*img_1.cols+tl_2x, beishu*img_1.rows+tl_2y), cvScalar(255, 0, 0,0), 2,2,0);
			}

			//************************第二次

			///************************第三次b
			if(1)
			{
				beishu=1;
				//int height_1 =50;//80            
				//int width_1 =200;
				//int tl_1x=center_x-width_1/2;
				//int tl_1y=center_y-20;  //150
				int height_1 =80;//80            
				int width_1 =132;
				int tl_1x=center_x-width_1/2;
				int tl_1y=center_y-20;  //150

				//CvRect roi_1=cvRect(tl_1x,tl_1y,img_2.cols,img_2.rows);  

				//cvSetImageROI(src,roi_1);

				//IplImage *src_roi=cvCreateImage(cvSize(img_2.cols,img_2.rows),8,1);
				//cvCopy(src,src_roi,0);
				//memcpy(img_2.imgPtr, src_roi->imageData, img_2.rows*img_2.cols*sizeof(unsigned char));

				Get_roi(&img_2,src->imageData,tl_1x,tl_1y,width_1,height_1);
				HaarDetectObjects(&img_2, NULL, 34, 2, 0, minSize,tl_1x,tl_1y,beishu);
				cvRectangle(imgframe, cvPoint(tl_1x,tl_1y), cvPoint(beishu*img_2.cols+tl_1x, beishu*img_2.rows+tl_1y), cvScalar(0, 0, 255,0), 2,2,0);
			}
			///************************第三次


			IplImage *imgframe2=cvCreateImage(cvSize(imgframe->width,imgframe->height), 8, 3);
			memcpy(imgframe2->imageData,imgframe->imageData,360960*3);
			if(result_seq.total != 0)
			{
				char path[100];
				for(int k=0;k<result_seq.total;k++)
				{
					Rect rect = result_seq.rectQueue[k];
					cvRectangle(imgframe2, cvPoint(rect.x,rect.y), cvPoint(rect.x+rect.width,rect.y+rect.height), cvScalar(0, 255, 0,0), 2,2,0);
				}
			}

			//track
//			CvMat *tmp_mat = cvCreateMat(src->height, src->width, CV_8U);
			struct CvMat tmp_mat;
			struct CtRectF detect_rect;
			detect_rect.x = 0;
			detect_rect.y = 0;
			detect_rect.width = 0;
			detect_rect.height = 0;
			std::vector<float> val_vec;

			if(result_seq.total != 0)
			{
			TIME_ST;
				for(int k = 0;k < result_seq.total; k++)
				{
					if (result_seq.rectQueue[k].width == 0 || result_seq.rectQueue[k].height == 0) {
						continue;
					}
					struct CvMat img_mat;
					struct CtRect tmp_rect;
					tmp_rect.x = result_seq.rectQueue[k].x;
					tmp_rect.y = result_seq.rectQueue[k].y;
					tmp_rect.width = result_seq.rectQueue[k].width;
					tmp_rect.height = result_seq.rectQueue[k].height;
					cv_LoadCvMat(&img_mat,0,MAXROWS,MAXCOLS, src->imageData);
					tracker_check((CtMat*)&img_mat, &tmp_rect, k);
				}
				tracker_check(val_vec);
			TIME_ED;
			}
			
			if(result_seq.total != 0)
			{
				int dis_min = 1 << 30;
				int idx = -1;
				for(int k = 0;k < result_seq.total; k++)
				{
					if (1) {
						CvRect rect;
						rect.x = result_seq.rectQueue[k].x;
						rect.y = result_seq.rectQueue[k].y;
						rect.width = result_seq.rectQueue[k].width;
						rect.height = result_seq.rectQueue[k].height;
						cvSetImageROI(src, rect);
						IplImage *tmp_img = cvCreateImage(cvSize(result_seq.rectQueue[k].width,
									result_seq.rectQueue[k].height), 8, 1);
						cvCopy(src, tmp_img);
						char tmp_name[1024];
						sprintf(tmp_name, "./img/%.2f_%d_%d.jpg", val_vec[k], totalframes, k);
						cvSaveImage(tmp_name, tmp_img);
						uchar *p = (uchar*)(tmp_img->imageData);
						//for (int ii = 0; ii < tmp_img->height; ii++) {
						//	for (int jj = 0; jj < tmp_img->widthStep; jj++) {
						//		printf("%d ", p[ii * tmp_img->widthStep + jj]);
						//	}
						//}
						//printf("\n");
						cvReleaseImage(&tmp_img);
					}

					if (val_vec[k] <= 5.0f || result_seq.rectQueue[k].width == 0 
							|| result_seq.rectQueue[k].height == 0) {
						continue;
					}
					int tmp_x = result_seq.rectQueue[k].x + result_seq.rectQueue[k].width / 2;
					int tmp_y = result_seq.rectQueue[k].y + result_seq.rectQueue[k].height / 2;
					int tmp_dis = (tmp_x - center_x) * (tmp_x - center_x) + (tmp_y - 480) * (tmp_y - 480);
					if (tmp_dis < dis_min) {
						struct CvMat img_mat;
						struct CtRect tmp_rect;
						tmp_rect.x = result_seq.rectQueue[k].x;
						tmp_rect.y = result_seq.rectQueue[k].y;
						tmp_rect.width = result_seq.rectQueue[k].width;
						tmp_rect.height = result_seq.rectQueue[k].height;
						cv_LoadCvMat(&img_mat,0,MAXROWS,MAXCOLS, src->imageData);
						int ret = tracker_check((CtMat*)&img_mat, &tmp_rect, k);
						if (ret == 0) {
							dis_min = tmp_dis;
							idx = k;
						}
					}
				}
				if (idx != -1) {
					detect_rect.x = result_seq.rectQueue[idx].x;
					detect_rect.y = result_seq.rectQueue[idx].y;
					detect_rect.width = result_seq.rectQueue[idx].width;
					detect_rect.height = result_seq.rectQueue[idx].height;
				}
			}

			int det_dis = -1;
			if (_ct_tracker.is_tracking != 0 && detect_rect.width > _width_min) {
				int img1_x = detect_rect.x + detect_rect.width / 2;
				int img1_y = detect_rect.y + detect_rect.height / 2;
				int img2_x = _ct_tracker.last_rect.x + _ct_tracker.last_rect.width / 2;
				int img2_y = _ct_tracker.last_rect.y + _ct_tracker.last_rect.height / 2;
				int dis1 = (img1_x - center_x) * (img1_x - center_x) + (img1_y - 480) * (img1_y - 480);
				int dis2 = (img2_x - center_x) * (img2_x - center_x) + (img2_y - 480) * (img2_y - 480);
				det_dis = dis1 - dis2;
			}

			int flag_match = 0;
			if (det_dis < 0 && detect_rect.width > _width_min && _ct_tracker.is_tracking != 0) {
			struct 	CtRectF rect1 = detect_rect;
			struct 	CtRectF rect2 = _ct_tracker.last_rect;
				int area1 = (rect1.width * rect1.height + rect2.width * rect2.height)/ 2;
				if (rect1.x + rect1.width > rect2.x + rect2.width) {
					rect1.width = (rect2.x + rect2.width - rect1.x);
				}
				if (rect1.y + rect1.height > rect2.y + rect2.height) {
					rect1.height = (rect2.y + rect2.height - rect1.y);
				}
				if (rect1.x < rect2.x) {
					rect1.width -= (rect2.x - rect1.x);
					rect1.x = rect2.x;
				}
				if (rect1.y < rect2.y) {
					rect1.height -= (rect2.y - rect1.y);
					rect1.y = rect2.y;
				}
				int area2 = rect1.width * rect1.height;
				float scale = area2 / (float)area1;
				if (scale > 0.5) {
					flag_match = 1;
				}
			}

			if (det_dis < 0 && detect_rect.width > _width_min && flag_match == 0) {
//				cvConvert(src, tmp_mat);
				cv_LoadCvMat(&tmp_mat,0,MAXROWS,MAXCOLS, src->imageData);
				if (_ct_tracker.is_tracking != 0) {
					tracker_release();
				}
				tracker_init(detect_rect, (CtMat*)&tmp_mat);
				struct CtRectF result = detect_rect;
				cvRectangle(imgframe, cvPoint(result.x,result.y),
					cvPoint(result.x+result.width,result.y+result.height),
					cvScalar(0,0,255,0),2,8,0);
			} else if (_ct_tracker.is_tracking != 0) {
			//	cvConvert(src, tmp_mat);
				cv_LoadCvMat(&tmp_mat,0,MAXROWS,MAXCOLS, src->imageData);
				struct CtRectF result;
				int ret = tracker_update((CtMat*)&tmp_mat, &result);
				if (ret == 0) {
					cvRectangle(imgframe, cvPoint(result.x,result.y),
						cvPoint(result.x+result.width,result.y+result.height),
						cvScalar(0,0,255,0),2,8,0);
				} else {
					tracker_release();
				}
			}

	
//			cvReleaseMat(&tmp_mat);
			//gettimeofday(&end,NULL);
			//time_use=(end.tv_sec-start.tv_sec)*1000000+(end.tv_usec-start.tv_usec);//微秒
			//printf("time_use is %.10f\n",time_use);
		/*	if(time_use>300000)
			{
				break;
			}*/
			//if(result_seq.total != 0)
			//{
			//	for(int k=0;k<result_seq.total;k++)
			//	{
			//		if (result_seq.rectQueue[k].width == 0)
			//			continue;
			//			Rect rect = result_seq.rectQueue[k];
			//		cvRectangle(imgframe, cvPoint(rect.x,rect.y), cvPoint(rect.x+rect.width,rect.y+rect.height), cvScalar(255, 0, 0,0), 2,2,0);
			//	}
			//}
			//if (_cnt == 500 || totalframes==0) {
			//get_score(_cnt);
			//break;
			//}

			char out_path1[256];
			sprintf(out_path1, "..\\kuang\\%d.bmp",694-totalframes);
			//cvSaveImage(out_path1,imgframe);
			//memcpy(data, src->imageData, /*3**/(src->width*src->height)*sizeof(unsigned char));
			//WriteRawData(data,out_path);
			//time(&t2);
			//Time += difftime(t2, t1);
		//	printf("%d:%ds\n",totalframes, (int)Time);
			//OutputResult(img, totalframes);
#ifdef SHOW_IMAGE
			cvShowImage("src_without_tracking",imgframe2);
			cvShowImage("tracking",imgframe);
#endif
			cvReleaseImage(&src);
			cvReleaseImage(&imgframe2);
			int t=cvWaitKey(1);
			//清楚result_queue结果队列
			CLR_RESULT_QUEUE();
			//Delay(1000);
		}
	}
	cvReleaseImage(&src_test);
	//printf("Avg:%fms\n",Time*1000.0/tc.QuadPart/totalframes);
	return 0;
}
