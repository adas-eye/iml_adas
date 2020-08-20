#include "Haar.h"
#include "lane.h"
//#include <windows.h>
#include <math.h>
#include <opencv2/opencv.hpp>  
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <cv.h>   //cv.h OpenCV的主要功能头文件，务必要；
#include <highgui.h>
#include "stdio.h"
#include "Util.h"
#include "stdlib.h"

//******************************************12.15版************************
//****************************************
//************************
float trigtab[2*numangle];
lane_tracker tracker[tracker_num];


//
//double  LDW_mat[9];
//double  inv_LDW_mat[9];
//
//double  FCW_mat[9];

//************************
//****************************************
//******************************************12.15版************************
//#define MAX(a,b)    a > b ? a : b
#define ABS(a) a>(-a)?a:(-a)

using namespace std;

#define buffer_count (ROI_WIDTH+2)*(ROI_HEIGHT+2) + (ROI_WIDTH+2) * 3 * sizeof(int)
//short dx_data[ROI_HEIGHT*ROI_WIDTH],dy_data[ROI_HEIGHT*ROI_WIDTH];//存储sobel计算结果

int trangle_x1=ROI_WIDTH/3;
int trangle_x2=2*ROI_WIDTH/3;
int trangle_y=ROI_HEIGHT/3;
float slope=trangle_x1/trangle_y;

int high_angle=110*numangle/180;
int low_angle=70*numangle/180;
IplImage* show_lines_1 = cvCreateImage(cvSize(numrho,numangle), 8, 1);
IplImage* show_lines_2 = cvCreateImage(cvSize(5*numrho,5*numangle), 8, 1);

//
int max_n;
int max_val;
int ttt;

int real_width;
//
float coff=0.7;//Y方向压缩系数 330



void initial_sin_cos(float rho, float theta)
{
	float irho = 1.0 / r_resolution;//此处为浮点，改成1.0 11.27
	float ang =0;
	int n = 0;
	for (; n < numangle; ang += theta, n++) 
	{
			//n=theta_low-n;
		trigtab[n * 2] = (float)(cos(ang+theta_low*PI/180) * irho);
		trigtab[n * 2 + 1] = (float)(sin(ang+theta_low*PI/180) * irho);
	}
	trigtab[70]=0;
	trigtab[71]=0.5;
}

 void GetMatrix(int* map_table,float* matin)//3.7 改成数组定义在main函数中，传值传过来
 {
	int i,j;
	//double  matin[9] = {
	//		    1.3926164874528061e+001, 2.6064539674623049e+001,
 //   -4.9536021505291174e+003 ,-1.4838709677624151e-001,
 //   1.5301762337997392e+001, 3.8243512544829076e+002,
 //   -7.1684587813930609e-004, 7.4051943755058192e-002, 1//207
	//		};

	double x,y,z;
	int xx,yy;

	memset(map_table,0,IPM_HEIGHT*IPM_WIDTH*sizeof(int));
	int start_j=(Original_WIDTH-IPM_WIDTH)/2;
	int end_j=(Original_WIDTH+IPM_WIDTH)/2;

	//int jj=Original_WIDTH/2;
	//int ii=-8;
	//double test_x=matin[0]*jj+matin[1]*ii+matin[2];
	//double test_y=matin[3]*jj+matin[4]*ii+matin[5];
	//double test_z=matin[6]*jj+matin[7]*ii+matin[8];

	// test_x=test_x/test_z;
	// test_y=test_y/test_z;

	for(i=0;i<IPM_HEIGHT;i++)
	{
		for(j=start_j;j<end_j;j++)//for(i=0;i<752;i++)
		{
			x=matin[0]*j+matin[1]*i+matin[2];
			y=matin[3]*j+matin[4]*i+matin[5];
			z=matin[6]*j+matin[7]*i+matin[8];
			xx = (x/z+0.5);
			yy = (y/z+0.5);
			if(xx >= 0 && xx < Original_WIDTH && yy >= 0 && yy < Original_HEIGHT)
			{
				map_table[i * IPM_WIDTH+j - start_j] = (xx) + (yy) * Original_WIDTH;
				//map_table[i + j * 752] = (752-xx) + (480-yy) * IMAGE_WIDTH;
			}
			else
			{
				map_table[i * IPM_WIDTH+j - start_j] = 0;//map_table[i + j * 752] = 0;
			}
		}
	}
 }

 void get_IPM_convolution(unsigned char* src_data,unsigned char* IPM_data,unsigned char* dst_data,int* valid_count_f,Point2s* edges,short* max_pos1,short* min_pos1,int* map_ta,int center_y,int if_limit)
 {
	unsigned char* line_hat=dst_data;
	unsigned char* ipmg_img=IPM_data;

	short* max_data=max_pos1;
	short* min_data=min_pos1;

	memset(line_hat,0,IPM_HEIGHT*IPM_WIDTH);
	memset(max_data,LOOP_NUM+1,IPM_HEIGHT*IPM_WIDTH*sizeof(short));
	memset(min_data,LOOP_NUM+1,IPM_HEIGHT*IPM_WIDTH*sizeof(short));

	int limit;
	if(if_limit)
		limit=float(IPM_HEIGHT-calib_plus)/calib_multiple;
	else
		limit=2;
	
	line_hat += limit*IPM_WIDTH;
	ipmg_img += limit*IPM_WIDTH;

	max_data += limit*IPM_WIDTH;
	min_data += limit*IPM_WIDTH;

	(*valid_count_f)=0;
	Point2s pt;

	for(int i=limit;i<IPM_HEIGHT-2;i++)
	{
	
		for(int j=2;j<IPM_WIDTH-2;j++)
		{
			//short plus11=ipmg_img[j]-ipmg_img[j-2];
			//short plus22=ipmg_img[j]-ipmg_img[j+2];
			short plus1=ipmg_img[j]-ipmg_img[j-2]+((ipmg_img[j-IPM_WIDTH]-ipmg_img[j-2-IPM_WIDTH]+ipmg_img[j+IPM_WIDTH]-ipmg_img[j-2+IPM_WIDTH])>>1);
			short plus2=ipmg_img[j]-ipmg_img[j+2]+((ipmg_img[j-IPM_WIDTH]-ipmg_img[j+2-IPM_WIDTH]+ipmg_img[j+IPM_WIDTH]-ipmg_img[j+2+IPM_WIDTH])>>1);

			if(ipmg_img[j]-ipmg_img[j-2]>0 && ipmg_img[j]-ipmg_img[j+2]>0 && plus2>0 && plus1>0 &&map_ta[(i-1)*IPM_WIDTH+j]!=0)
			{
				short plus3=((ipmg_img[j+1]+ipmg_img[j-1]-ipmg_img[j-IPM_WIDTH]-ipmg_img[j+IPM_WIDTH])>>1)+((ipmg_img[j-IPM_WIDTH+1]+ipmg_img[j+IPM_WIDTH-1]+ipmg_img[j-IPM_WIDTH-1]+ipmg_img[j+IPM_WIDTH+1])>>2)-ipmg_img[j];
				short plus =((plus1+plus2+plus3)>>1);//11.27

				if(plus<3)
					line_hat[j]=0;
				else
				{
					if(plus>255)//11.27
						line_hat[j]=255;
					else 
						line_hat[j]=plus;

					//*****************************耗时较长，下位机先不要加
					int src_index=map_ta[i*IPM_WIDTH+j];
					Point temp;
					temp.y=src_index/Original_WIDTH;
					temp.x=src_index%Original_WIDTH;

					//int loop_num=50;
					int max_val,max_pos, min_val,min_pos;
					int temp_val,temp1,temp2,temp_avg,th_val;
					max_val=max_pos=-10000;
					min_val=min_pos=10000;
					int loop=LOOP_NUM * float(temp.y-center_y)/(Original_HEIGHT-center_y);

					for(int k=-loop;k<=loop;k++)
					{
						if(temp.x+k<4||temp.x+k>Original_WIDTH-4)
							continue;
						temp1=src_data[src_index+k-4]+src_data[src_index+k-3]+src_data[src_index+k-2]+src_data[src_index+k-1];
						temp2=src_data[src_index+k+4]+src_data[src_index+k+3]+src_data[src_index+k+2]+src_data[src_index+k+1];
						temp_avg=((temp1+temp2)>>3);
						th_val=temp_avg*GREDIENT_CONTROL;

						temp_val=temp1-temp2;
						//if(temp_val<0)
						//	temp_val=0-temp_val;
						/*if(L_or_R==0)
							temp_val=2*src_data[src_index+j-2]+src_data[src_index+j-1]-src_data[src_index+j+1]-2*src_data[src_index+j+2];
						else if(L_or_R==1)
							temp_val=2*src_data[src_index+j+2]+src_data[src_index+j+1]-src_data[src_index+j-1]-2*src_data[src_index+j-2];*/

						if(temp_val>max_val)
						{
							max_val=temp_val;
							max_pos=k;
						}
						if(temp_val<min_val)
						{
							min_val=temp_val;
							min_pos=k;
						}
					}
					if(max_val<th_val || min_val >-1*th_val || min_pos>max_pos)
					{
						line_hat[j]=0;
						continue;
					}
					//*****************************耗时较长，下位机先不要加

					if((*valid_count_f)>edge_points_count)//11.27
						continue;//防止溢出
					else
					{
						pt.x=j;
						pt.y=i;

						edges[(*valid_count_f)]=pt;
						max_data[j]=max_pos;
						min_data[j]=min_pos;
						if(min_pos>10000)
							int x=1;
						(*valid_count_f)++;
					}
				}
			}
			else 
				line_hat[j]=0;
		}
		ipmg_img+=IPM_WIDTH;
		line_hat+=IPM_WIDTH;
		max_data+=IPM_WIDTH;
		min_data+=IPM_WIDTH;
	}
 }

 void find_accurate_lane_position(Line_IPM* line, Point* edge_points ,Point* right_edge,Point* right_edge2,
	 uchar* edge_img, uchar* edge_img_near_car,uchar* src_data,int *map_table,int *map_table_near_car,
	 int center_y,Point* P1,Point* P2,int& count,int& count2,
	 int L_or_R,float* inv,short* max_pos1,short* min_pos1,short* max_pos2,short* min_pos2)
 {
	int map_table_index;
	int edge_img_index;
	float sin_temp=line->sin;
	float cos_temp=line->cos;
	Point p1,p2,point_temp;
	//int edge_points_index=0;

	unsigned char repeated_flag[480];
	memset(repeated_flag,0,480);
	int loop_num=50;
	int x = 0, y = 0, x2 = 0, y2 = 0, xy = 0, w = 0;
	float dx2, dy2, dxy,tt;
	float avg_x=0,avg_y=0,avg_x2=0,avg_y2=0,avg_xy=0;
	Point temp;

	//寻找搜索起始点，结束点
	for(int i=0;i<IPM_HEIGHT;i++)
	{
		p1.x= int((float(line->rho)-i*cos_temp)/sin_temp+0.5);
		map_table_index=i*IPM_WIDTH+p1.x;
		if(map_table[map_table_index]!=0)
		{
			p1.y= i;
			break;
		}
	}

	p2.x= int((float(line->rho)-(IPM_HEIGHT-1)*cos_temp)/sin_temp+0.5);//9.23
	if(p2.x<0)
	{
		p2.x=0;
		p2.y=int(float(line->nsta)*sin_temp/cos_temp+0.5);
	}
	else if(p2.x>IPM_WIDTH-1)
	{
		p2.x=IPM_WIDTH-1;
		p2.y=int((IPM_WIDTH-1-float(line->nsta))*sin_temp/cos_temp+0.5);
		if(p2.y<0)
			p2.y=(-1)*p2.y;
	}
	else
		p2.y=IPM_HEIGHT-1;//
	//寻找搜索起始点，结束点

	////if(p2.y)
	//int black[IPM_WIDTH];
	//int white[IPM_WIDTH];

	//int temp_black=0,temp_white=0;//11.22
	//int black_count=0,white_count=0;//11.22
#if only_near_car
	float limit=float(IPM_HEIGHT-calib_plus)/calib_multiple;
	for(int i=0;i<=p2.y-p1.y;i++)
	{
		point_temp.y=p1.y+i;
		
		//if(point_temp.y<limit)
		//	continue;
		//删除一段//11.22

		point_temp.x=(p1.x-i*cos_temp/sin_temp+0.5);
		edge_img_index=IPM_WIDTH*point_temp.y+point_temp.x;

		//bool if_IPM_poi_valid=false;
		if(edge_img[edge_img_index]!=0)
		{
		//	if_IPM_poi_valid=true;

			//if(temp_black!=0)
			//{
			//	black[black_count]=temp_black;
			//	temp_black=0;
			//	black_count++;
			//}
			//temp_white++;//11.22
			/*edge_points[edge_points_index]=point_temp;
			edge_points_index++;*/
		}
		else if(point_temp.x>1 && edge_img[edge_img_index-1]!=0)
		{
			//if_IPM_poi_valid=true;
			point_temp.x-=1;
			edge_img_index-=1;

			//if(temp_black!=0)
			//{
			//	black[black_count]=temp_black;
			//	temp_black=0;
			//	black_count++;
			//}
			//temp_white++;//11.22
			/*edge_points[edge_points_index].x=point_temp.x-1;
			edge_points[edge_points_index].y=point_temp.y;
			edge_points_index++;*/
		}
		else if(point_temp.x<IPM_WIDTH-1 && edge_img[edge_img_index+1]!=0)
		{
			//if_IPM_poi_valid=true;
			point_temp.x+=1;
			edge_img_index+=1;

			//if(temp_black!=0)
			//{
			//	black[black_count]=temp_black;
			//	temp_black=0;
			//	black_count++;
			//}
			//temp_white++;//11.22
			/*edge_points[edge_points_index].x=point_temp.x+1;
			edge_points[edge_points_index].y=point_temp.y;
			edge_points_index++;*/
		}
		else
		{
			//if(temp_white!=0)
			//{
			//	white[white_count]=temp_white;
			//	temp_white=0;
			//	white_count++;
			//}
			//temp_black++;
			continue;//11.22
		}

		if(point_temp.y>2*IPM_HEIGHT/3)
			continue;//11.22

		int src_index=map_table[edge_img_index];
		temp.y=src_index/Original_WIDTH;
		temp.x=src_index%Original_WIDTH;

		if(repeated_flag[temp.y]==1)
			continue;

		right_edge[count].y=temp.y;
		if(L_or_R==0)
			right_edge[count].x=temp.x+max_pos1[edge_img_index];
		else
			right_edge[count].x=temp.x+min_pos1[edge_img_index];


		x += right_edge[count].x;
		y += right_edge[count].y;
		x2 += right_edge[count].x * right_edge[count].x;
		y2 += right_edge[count].y * right_edge[count].y;
		xy += right_edge[count].x * right_edge[count].y;
		count++;

		repeated_flag[temp.y]=1;
		//}
	}
#endif

	cout<<"count:"<<count<<"        ";
	//搜寻near_car的区域
#if add_near_car
	memset(repeated_flag,0,480);
	Point p3,p4,p_temp;
	p3.x=p1.x;
	p3.y=p1.y*calib_multiple+calib_plus;

	//p4.x=p2.x;
	//p4.y=p2.y*calib_multiple+calib_plus;

	if(p3.y>=0 && p3.y<IPM_HEIGHT)
	{
		for(int i=-calib_plus;i+p3.y<=IPM_HEIGHT-1;i++)
		{
			point_temp.y=p3.y+i;
			//删除一段//11.22

			point_temp.x=(p3.x-i*cos_temp/sin_temp/calib_multiple + 0.5);
			if(point_temp.x<0 || point_temp.x>IPM_WIDTH-1)
				continue;
			edge_img_index=IPM_WIDTH*point_temp.y+point_temp.x;

			//bool if_IPM_poi_valid=false;
			if(edge_img_near_car[edge_img_index]!=0)
			{
			//	if_IPM_poi_valid=true;

				//if(temp_black!=0)
				//{
				//	black[black_count]=temp_black;
				//	temp_black=0;
				//	black_count++;
				//}
				//temp_white++;//11.22
				/*edge_points[edge_points_index]=point_temp;
				edge_points_index++;*/
			}
			else if(point_temp.x>1 && edge_img_near_car[edge_img_index-1]!=0)
			{
				//if_IPM_poi_valid=true;
				point_temp.x-=1;
				edge_img_index-=1;

				//if(temp_black!=0)
				//{
				//	black[black_count]=temp_black;
				//	temp_black=0;
				//	black_count++;
				//}
				//temp_white++;//11.22
				/*edge_points[edge_points_index].x=point_temp.x-1;
				edge_points[edge_points_index].y=point_temp.y;
				edge_points_index++;*/
			}
			else if(point_temp.x<IPM_WIDTH-1 && edge_img_near_car[edge_img_index+1]!=0)
			{
				//if_IPM_poi_valid=true;
				point_temp.x+=1;
				edge_img_index+=1;

				//if(temp_black!=0)
				//{
				//	black[black_count]=temp_black;
				//	temp_black=0;
				//	black_count++;
				//}
				//temp_white++;//11.22
				/*edge_points[edge_points_index].x=point_temp.x+1;
				edge_points[edge_points_index].y=point_temp.y;
				edge_points_index++;*/
			}
			else
			{
				//if(temp_white!=0)
				//{
				//	white[white_count]=temp_white;
				//	temp_white=0;
				//	white_count++;
				//}
				//temp_black++;
				continue;//11.22
			}
			//continue;

			//if(point_temp.y>2*IPM_HEIGHT/3)
			//	continue;//11.22

			int src_index=map_table_near_car[edge_img_index];
			temp.y=src_index/Original_WIDTH;
			temp.x=src_index%Original_WIDTH;

			if(repeated_flag[temp.y]==1)
				continue;

			right_edge[count].y=temp.y;
			if(L_or_R==0)
				right_edge[count].x=temp.x+max_pos2[edge_img_index];
			else
				right_edge[count].x=temp.x+min_pos2[edge_img_index];


			x += right_edge[count].x;
			y += right_edge[count].y;
			x2 += right_edge[count].x * right_edge[count].x;
			y2 += right_edge[count].y * right_edge[count].y;
			xy += right_edge[count].x * right_edge[count].y;
			count++;

			repeated_flag[temp.y]=1;
			//}
		}
	}
#endif

	//此处需要添加额外步骤保证p1.y，maptable对应不为0，应在其实添加函数确定7的具体值！！
	
	//int bb=0,ww=0,b_total=0,w_total=0;
	//for(int i=0;i<black_count;i++)
	//{
	//	if(black[i]>=6)
	//		bb++;
	//	b_total+=black[i];
	//}//11.22

	//int max=0;
	//for(int i=0;i<white_count;i++)
	//{
	//	if(white[i]>max)
	//		max=white[i];
	//	w_total+=white[i];
	//}

	//if(bb>=2 && b_total>=16)
	//	line->line_classification=0;
	//else if(max>60)
	//	line->line_classification=1;//11.22

	cout<<"count:"<<count<<endl;

	if(count<2)
	{
		line->if_valid=false;
		return;
	}

	avg_x= double(x)/count;
	avg_y= double(y)/count;
	avg_x2= double(x2)/count;
	avg_y2= double(y2)/count;
	avg_xy=double(xy)/count;

	dx2 = avg_x2 - avg_x * avg_x;
	dy2 = avg_y2 - avg_y * avg_y;
	dxy = avg_xy - avg_x * avg_y;
	tt = (float) atan2( 2 * dxy, dx2 - dy2 ) / 2;
	float nx=sin(tt);
	float ny=cos(tt);

	float dist;
	for(int j = 0; j < count; j++ )
    {
        float xx, yy;

        xx = right_edge[j].x - avg_x;
        yy = right_edge[j].y - avg_y;

        dist = (float) fabs( nx * xx - ny * yy)*float(Original_HEIGHT-right_edge[j].y)/( right_edge[j].y-center_y);
		if(dist>ERROR_CONTROL)
		{
			x -= right_edge[j].x;
			y -= right_edge[j].y;
			x2 -= right_edge[j].x * right_edge[j].x;
			y2 -= right_edge[j].y * right_edge[j].y;
			xy -= right_edge[j].x * right_edge[j].y;
			continue;
		}
		right_edge2[count2]=right_edge[j];
		count2++;
    }

	if(count2<2)
	{
		line->if_valid=false;
		return;
	}

	avg_x= double(x)/count2;
	avg_y= double(y)/count2;
	avg_x2= double(x2)/count2;
	avg_y2= double(y2)/count2;
	avg_xy=double(xy)/count2;

	dx2 = avg_x2 - avg_x * avg_x;
	dy2 = avg_y2 - avg_y * avg_y;
	dxy = avg_xy - avg_x * avg_y;
	tt = (float) atan2( 2 * dxy, dx2 - dy2 ) / 2;

	//return count;
	nx=sin(tt);
	ny=cos(tt);

	float a=nx/ny;
	float b= avg_y - a*avg_x;

	//Point P1,P2;
	P1->y=479;
	P1->x=(479-b)/a;
	if(P1->x<0)
	{
		P1->x=0;
		P1->y=b;
	}
	else if(P1->x>Original_WIDTH-1)
	{
		P1->x=Original_WIDTH-1;
		P1->y=a*P1->x+b;
	}

	P2->y=center_y+50;
	P2->x=(P2->y-b)/a;

	float p1x,p1y,p2x,p2y;
	float xxx,yyy,zzz;

	xxx=inv[0]*P1->x+inv[1]*P1->y+inv[2];
	yyy=inv[3]*P1->x+inv[4]*P1->y+inv[5];
	zzz=inv[6]*P1->x+inv[7]*P1->y+inv[8];
	p1x = xxx/zzz-(376-IPM_WIDTH/2);
	p1y = yyy/zzz;

	xxx=inv[0]*P2->x+inv[1]*P2->y+inv[2];
	yyy=inv[3]*P2->x+inv[4]*P2->y+inv[5];
	zzz=inv[6]*P2->x+inv[7]*P2->y+inv[8];
	p2x = xxx/zzz-(376-IPM_WIDTH/2);
	p2y = yyy/zzz;

	line->P1.y=P1->y;
	line->P1.x=P1->x;
	line->P2.y=P2->y;
	line->P2.x=P2->x;

	line->p1.y=p1y;
	line->p1.x=p1x;
	line->p2.y=p2y;
	line->p2.x=p2x;

	line->angle=180-atan2(line->p2.y-line->p1.y,line->p2.x-line->p1.x)*180/3.1415927-theta_low;
	if(line->angle<0)
		line->angle=0;
	if(line->angle>=numangle)
		line->angle=numangle-1;

	//9.29*************测试
	float kkk=(p2x-p1x)/(p2y-p1y);
	if(L_or_R==0)
		line->nsta=p1x-(p1y+FRONT_DISTANCE)*kkk - 0.571*CHOOSELANE_PARA;//白线宽度按16的一半，8厘米算。8/14.3=0.571
	else if(L_or_R==1)
		line->nsta=p1x-(p1y+FRONT_DISTANCE)*kkk + 0.571*CHOOSELANE_PARA;//白线宽度按16的一半，8厘米算。
	//9.29*************测试
 }


 int find_lane_near_edge_point(Line_IPM* line, Point* p1,Point* p2, Point* edge_points,uchar* edge_img,int *map_table)
 {
	int map_table_index;
	int edge_img_index;
	float sin_temp=line->sin;
	float cos_temp=line->cos;
	Point point_temp;
	int edge_points_index=0;

	for(int i=0;i<IPM_HEIGHT;i++)
	{
		p1->x= int((float(line->rho)-i*cos_temp)/sin_temp+0.5);
		map_table_index=i*IPM_WIDTH+p1->x;
		if(map_table[map_table_index]!=0)
		{
			p1->y= i;
			break;
		}
	}

	p2->x= int((float(line->rho)-(IPM_HEIGHT-1)*cos_temp)/sin_temp+0.5);//9.23

	if(p2->x<0)
	{
		p2->x=0;
		p2->y=int(float(line->nsta)*sin_temp/cos_temp+0.5);
	}
	else if(p2->x>IPM_WIDTH-1)
	{
		p2->x=IPM_WIDTH-1;
		p2->y=int((IPM_WIDTH-1-float(line->nsta))*sin_temp/cos_temp+0.5);
		if(p2->y<0)
			p2->y=(-1)*p2->y;
	}
	else
		p2->y=IPM_HEIGHT-1;//

	for(int i=0;i<=p2->y-p1->y;i++)
	{
		point_temp.y=p1->y+i;
		point_temp.x=(p1->x-i*cos_temp/sin_temp+0.5);

		edge_img_index=IPM_WIDTH*point_temp.y+point_temp.x;

		if(edge_img[edge_img_index]!=0)
		{
			edge_points[edge_points_index]=point_temp;
			edge_points_index++;
		}
		
		if(point_temp.x>1 && edge_img[edge_img_index-1]!=0)
		{
			edge_points[edge_points_index].x=point_temp.x-1;
			edge_points[edge_points_index].y=point_temp.y;
			edge_points_index++;
		}

		if(point_temp.x<IPM_WIDTH-1 && edge_img[edge_img_index+1]!=0)
		{
			edge_points[edge_points_index].x=point_temp.x+1;
			edge_points[edge_points_index].y=point_temp.y;
			edge_points_index++;
		}
	}

	return edge_points_index;
 }

 float cal_ZNCC_score(Point *poi,uchar* img_4x)//4,9,4  111,145,111 ,均值129。车道线理想假设灰度,遍历左2，右2
 {
	 int lane_w=5;
	 int side=2;

	 int avg1=128+lane_w-2*side;

	 float coff_table[19];

	 for(int i=0;i<side;i++)
		 coff_table[i]=-2*lane_w;

	 for(int i=lane_w+side;i<lane_w+2*side;i++)
		 coff_table[i]=-2*lane_w;

	 for(int i=side;i<lane_w+side;i++)
		 coff_table[i]=4*side;

	 float sum=0;
	 for(int i=0;i<lane_w+2*side;i++)
		 sum+=coff_table[i]*coff_table[i];

	 int img_index=IPM_WIDTH_4x*poi->y+poi->x;
	 float ZNCC_temp=0,ZNCC_max=0;
	 float avg=0;
	 float diff_conv=0;
	 float diff_self=0;
	 

	 int shift=0;
	 int loop_num=lane_w/2+side;

	 for(int i=-2;i<=2;i++)
	 {
		 if(poi->x+i-loop_num>0 && poi->x+i+loop_num<IPM_WIDTH_4x)
		 {
			 //avg=img_4x[img_index];
			 for(int j=0-loop_num;j<=loop_num;j++)
			 {
				 avg+=img_4x[img_index+i+j];
			 }
			 avg/=(lane_w+2*side);

			 for(int j=0-loop_num;j<=loop_num;j++)
			 {
				 diff_self+=(img_4x[img_index+i+j]-avg)*(img_4x[img_index+i+j]-avg);
				 diff_conv+=(img_4x[img_index+i+j]-avg)*coff_table[j+loop_num];
			 }
			 float ttt=pow(sum*diff_self,0.5);
			 ZNCC_temp=diff_conv/ttt;
			 if(ZNCC_temp>ZNCC_max)
			 {
				 ZNCC_max=ZNCC_temp;
				 shift=i;
			 }
			 ZNCC_temp=avg=diff_conv=diff_self=0;
		 }			
	 }
	 cout<<"zncc:"<<ZNCC_max<<endl;
	 return ZNCC_max;
 }

 void GetMatrix_4x(int* map_table,float* matin)//3. 420
 {
	int i,j;
	//double  matin[9] = {
	//		    1.3926164874528061e+001, 2.6064539674623049e+001,
 //   -4.9536021505291174e+003 ,-1.4838709677624151e-001,
 //   1.5301762337997392e+001, 3.8243512544829076e+002,
 //   -7.1684587813930609e-004, 7.4051943755058192e-002, 1//207
	//		};

	double x,y,z;
	int xx,yy;

	memset(map_table,0,IPM_HEIGHT_4x*IPM_WIDTH_4x*sizeof(int));
	int start_j=(Original_WIDTH-IPM_WIDTH_4x)/2;
	int end_j=(Original_WIDTH+IPM_WIDTH_4x)/2;
	for(i=0;i<IPM_HEIGHT_4x;i++)
	{
		for(j=start_j;j<end_j;j++)//for(i=0;i<752;i++)
		{
			x=matin[0]*j+matin[1]*i+matin[2];
			y=matin[3]*j+matin[4]*i+matin[5];
			z=matin[6]*j+matin[7]*i+matin[8];
			xx = (x/z+0.5);
			yy = (y/z+0.5);
			if(xx >= 0 && xx < Original_WIDTH && yy >= 0 && yy < Original_HEIGHT)
			{
				map_table[i * IPM_WIDTH_4x+j - start_j] = (xx) + (yy) * Original_WIDTH;
				//map_table[i + j * 752] = (752-xx) + (480-yy) * IMAGE_WIDTH;
			}
			else
			{
				map_table[i * IPM_WIDTH_4x+j - start_j] = 0;//map_table[i + j * 752] = 0;
			}
		}
	}
 }

void ImageMatrix(int* map_table,uchar *dst,uchar *src)
{
	int j=0;
	for(int i=0;i<IPM_WIDTH*IPM_HEIGHT;i++)
	{
		if(map_table[i] == 0)
		{
			dst[i] = 0;
		}
		else
		{
			j=map_table[i];
			//if(j>360960)
			//	continue;
			dst[i] = src[j];
		}
	}
}

void ImageMatrix_4x(int* map_table,uchar *dst,uchar *src) //420
{
	int j=0;
	for(int i=0;i<IPM_WIDTH_4x*IPM_HEIGHT_4x;i++)
	{
		if(map_table[i] == 0)
		{
			dst[i] = 0;
		}
		else
		{
			j=map_table[i];
			//if(j>360960)
			//	continue;
			dst[i] = src[j];
		}
	}
}

bool if_local_max(int i,int j,int* cur_pos)
{
	for(int ii=-5;ii<=5;ii++)
	for(int jj=-5;jj<=5;jj++)
	{
		if(*cur_pos<*(cur_pos+ii*numrho+jj))
			return false;
	}
	return true;
}

void cal_trust_mat_gredient(int* a,Line_IPM& left,Line_IPM& middle, Line_IPM& right,int type1_val,int T_whole)
{
	int* a_line=a;
	float _nsta;

	for(int i=0;i<numangle;i++)
	{
		for(int j=0;j<numrho;j++)
		{
			if((*(a_line+j))>T_whole)//Line_IPM& right_line
			{
				_nsta=j/trigtab[i * 2+1]+FRONT_DISTANCE*trigtab[i * 2]/trigtab[i * 2+1];//9.28
				if(_nsta<IPM_WIDTH/2-SEGMENT && _nsta>-4 && (*(a_line+j))>left.val_trust) //左，信任值最大，且0-10米有边缘点 ////11.2
				{
					left.if_valid=true;
					left.angle=i;
					left.rho=j;
					/*left.val_1=*(a1_line+j);
					left.val_2=*(a2_line+j);
					left.val_3=*(a3_line+j);*/
					left.val_trust=(*(a_line+j));
					left.cos=trigtab[i * 2];
					left.sin=trigtab[i * 2+1];
					left.nsta=_nsta;

					if(((*(a_line+j))>type1_val))
					{
						left .type=1;
					}
					else
						left .type=2;//可能不是lane
				}
				else if(_nsta>IPM_WIDTH/2+SEGMENT && _nsta<IPM_WIDTH+4 && (*(a_line+j))>right.val_trust)//11.2
				{
					right.if_valid=true;
					right.angle=i;
					right.rho=j;
				/*	right.val_1=*(a1_line+j);
					right.val_2=*(a2_line+j);
					right.val_3=*(a3_line+j);*/
					right.val_trust=(*(a_line+j));
					right.cos=trigtab[i * 2];
					right.sin=trigtab[i * 2+1];
					right.nsta=_nsta;

					if((*(a_line+j))>type1_val )
					{
						right.type=1;
					}    
					else
						right.type=2;
				}
				else if((_nsta>=IPM_WIDTH/2-SEGMENT) && (_nsta<=IPM_WIDTH/2+SEGMENT) && (*(a_line+j)>middle.val_trust))//10.28
				{
 					middle.if_valid=true;
					middle.angle=i;
					middle.rho=j;
					/*middle.val_1=*(a1_line+j);
					middle.val_2=*(a2_line+j);
					middle.val_3=*(a3_line+j);*/
					middle.val_trust=(*(a_line+j));
					middle.cos=trigtab[i * 2];
					middle.sin=trigtab[i * 2+1];
					middle.nsta=_nsta;

					if((*(a_line+j))>type1_val )
					{
						middle.type=1;
					}
					else
						middle.type=2;
				}
			}
		}
		 a_line+=numrho;
		/* a2_line+=numrho;
		 a3_line+=numrho;
		 b_line+=numrho;
		 c_line+=numrho;*/
	}
}

void cal_trust_mat_gredient2(int* a,Line_IPM& line1,Line_IPM& line2, Line_IPM& line3,Line_IPM& line4,int type1_val,int T_whole)
{
	int* a_line=a;
	float _nsta;

	for(int i=0;i<numangle;i++)
	{
		for(int j=0;j<numrho;j++)
		{
			if((*(a_line+j))>T_whole)//Line_IPM& right_line
			{
				_nsta=j/trigtab[i * 2+1]+FRONT_DISTANCE*trigtab[i * 2]/trigtab[i * 2+1];//9.28
				if(_nsta<=IPM_WIDTH/4 && _nsta>-4 && (*(a_line+j))>line1.val_trust) //左，信任值最大，且0-10米有边缘点 ////11.2
				{
					line1.if_valid=true;
					line1.angle=i;
					line1.rho=j;
					/*left.val_1=*(a1_line+j);
					left.val_2=*(a2_line+j);
					left.val_3=*(a3_line+j);*/
					line1.val_trust=(*(a_line+j));
					line1.cos=trigtab[i * 2];
					line1.sin=trigtab[i * 2+1];
					line1.nsta=_nsta;

					if(((*(a_line+j))>type1_val))
					{
						line1.type=1;
					}
					else
						line1.type=2;//可能不是lane
				}
				else if(_nsta>IPM_WIDTH/4 && _nsta<=IPM_WIDTH/2 && (*(a_line+j))>line2.val_trust)//11.2
				{
					line2.if_valid=true;
					line2.angle=i;
					line2.rho=j;
				/*	right.val_1=*(a1_line+j);
					right.val_2=*(a2_line+j);
					right.val_3=*(a3_line+j);*/
					line2.val_trust=(*(a_line+j));
					line2.cos=trigtab[i * 2];
					line2.sin=trigtab[i * 2+1];
					line2.nsta=_nsta;

					if((*(a_line+j))>type1_val )
					{
						line2.type=1;
					}    
					else
						line2.type=2;
				}
				else if(_nsta>IPM_WIDTH/2 && _nsta<=IPM_WIDTH*3/4 && (*(a_line+j)>line3.val_trust))//10.28
				{
 					line3.if_valid=true;
					line3.angle=i;
					line3.rho=j;
					/*middle.val_1=*(a1_line+j);
					middle.val_2=*(a2_line+j);
					middle.val_3=*(a3_line+j);*/
					line3.val_trust=(*(a_line+j));
					line3.cos=trigtab[i * 2];
					line3.sin=trigtab[i * 2+1];
					line3.nsta=_nsta;

					if((*(a_line+j))>type1_val )
					{
						line3.type=1;
					}
					else
						line3.type=2;
				}
				else if(_nsta>IPM_WIDTH*3/4 && _nsta<IPM_WIDTH+4 && (*(a_line+j)>line4.val_trust))//10.28
				{
 					line4.if_valid=true;
					line4.angle=i;
					line4.rho=j;
					/*middle.val_1=*(a1_line+j);
					middle.val_2=*(a2_line+j);
					middle.val_3=*(a3_line+j);*/
					line4.val_trust=(*(a_line+j));
					line4.cos=trigtab[i * 2];
					line4.sin=trigtab[i * 2+1];
					line4.nsta=_nsta;

					if((*(a_line+j))>type1_val )
					{
						line4.type=1;
					}
					else
						line4.type=2;
				}
			}
		}
		 a_line+=numrho;
		/* a2_line+=numrho;
		 a3_line+=numrho;
		 b_line+=numrho;
		 c_line+=numrho;*/
	}
}

void cal_trust_mat(int* a1,int* a2,int* a3,int* b,int *c,
				  Line_IPM& left,Line_IPM& middle,Line_IPM& right,
				  int T1,int T_whole,int type1_val)
{
	int* a1_line=a1;
	int* a2_line=a2;
	int* a3_line=a3;
	int* b_line=b;
	int* c_line=c;

	int a11,a22,a33,bb,cc;
	float _nsta;
	for(int i=0;i<numangle;i++)
	{
		for(int j=0;j<numrho;j++)
		{
			a11=(*(a1_line+j));
			a22=(*(a2_line+j));
			a33=(*(a3_line+j));

			*(c_line+j)=(*(a2_line+j))*4+(*(a3_line+j));
			*(b_line+j)=(*(a1_line+j))*16+(*(c_line+j));

			bb=(*(b_line+j));
			cc=(*(c_line+j));

			if(a11>4 && a22>4 &&a33>4)
				(*(b_line+j))+=50;//0-10，10-20，20-30都有边缘点，则信任值+50

			if((*(b_line+j))>T_whole && (*(a1_line+j))>T1)//Line_IPM& right_line
			{
				_nsta=j/trigtab[i * 2+1]+FRONT_DISTANCE*trigtab[i * 2]/trigtab[i * 2+1];//9.28
				if(_nsta<IPM_WIDTH/2-SEGMENT && (*(b_line+j)>left.val_trust)) //左，信任值最大，且0-10米有边缘点 //10.28
				{
					left.if_valid=true;
					left.angle=i;
					left.rho=j;
					left.val_1=*(a1_line+j);
					left.val_2=*(a2_line+j);
					left.val_3=*(a3_line+j);
					left.val_trust=(*(b_line+j));
					left.cos=trigtab[i * 2];
					left.sin=trigtab[i * 2+1];
					left.nsta=_nsta;

					if(((*(b_line+j))>type1_val))
					{
						left .type=1;
					}
					else
						left .type=2;//可能不是车道线
				}
				else if(_nsta>IPM_WIDTH/2+SEGMENT && (*(b_line+j)>right.val_trust))//10.28
				{
					right.if_valid=true;
					right.angle=i;
					right.rho=j;
					right.val_1=*(a1_line+j);
					right.val_2=*(a2_line+j);
					right.val_3=*(a3_line+j);
					right.val_trust=(*(b_line+j));
					right.cos=trigtab[i * 2];
					right.sin=trigtab[i * 2+1];
					right.nsta=_nsta;

					if((*(b_line+j))>type1_val )
					{
						right.type=1;
					}    
					else
						right.type=2;
				}
				else if((_nsta>=IPM_WIDTH/2-SEGMENT) && (_nsta<=IPM_WIDTH/2+SEGMENT) && (*(b_line+j)>middle.val_trust))//10.28
				{
 					middle.if_valid=true;
					middle.angle=i;
					middle.rho=j;
					middle.val_1=*(a1_line+j);
					middle.val_2=*(a2_line+j);
					middle.val_3=*(a3_line+j);
					middle.val_trust=(*(b_line+j));
					middle.cos=trigtab[i * 2];
					middle.sin=trigtab[i * 2+1];
					middle.nsta=_nsta;

					if((*(b_line+j))>type1_val )
					{
						middle.type=1;
					}
					else
						middle.type=2;
				}
			}
		}
		 a1_line+=numrho;
		 a2_line+=numrho;
		 a3_line+=numrho;
		 b_line+=numrho;
		 c_line+=numrho;
	}
}

void choose_lane(Line_IPM left_line,Line_IPM middle_line,Line_IPM right_line,Line_IPM& choosed_left,Line_IPM& choose_middle,Line_IPM& choosed_right)//9.28
{
	int if_only_one_valid=(int)left_line.if_valid+(int)middle_line.if_valid+(int)right_line.if_valid;
	if(if_only_one_valid==1)
	{
		if(left_line.type==1)
			choosed_left=left_line;
		if(right_line.type==1)
			choosed_right=right_line;
		if(middle_line.type==1)
		{
			if(middle_line.nsta<IPM_WIDTH/2)
				choosed_left=middle_line;
			else
				choosed_right=middle_line;
		}
		return;
	}
	else if(if_only_one_valid==0)
	{
		return;
	}

	float angle_12,angle_23,angle_13;
	float width_12,width_23,width_13;//10.28
	bool valid_12,valid_23,valid_13;
	angle_12=angle_23=angle_13=width_12=width_23=width_13=0;
	valid_12=valid_23=valid_13=false;

	if(left_line.if_valid==1 && middle_line.if_valid==1)
	{
		angle_12=middle_line.angle-left_line.angle;
		width_12=middle_line.nsta-left_line.nsta;
		valid_12=(angle_12<=16) && (angle_12>=-16) && (width_12>17.5*CHOOSELANE_PARA) && (width_12<33*CHOOSELANE_PARA);//4.20。计算依据：29*coff,44*coff
	}
	if(middle_line.if_valid==1 && right_line.if_valid==1)
	{
		angle_23=right_line.angle-middle_line.angle;
		width_23=right_line.nsta-middle_line.nsta;
		valid_23=(angle_23<=16) && (angle_23>=-16) && (width_23>17.5*CHOOSELANE_PARA)&& (width_23<33*CHOOSELANE_PARA);//4.20
	}
	if(left_line.if_valid==1 && right_line.if_valid==1)
	{
		angle_13=right_line.angle-left_line.angle;
		width_13=right_line.nsta-left_line.nsta;
		valid_13=(angle_13<=16) && (angle_13>=-16) && (((width_13>17.5*CHOOSELANE_PARA) &&(width_13<33*CHOOSELANE_PARA))||(width_13>39*CHOOSELANE_PARA));////4.20。计算依据：29*coff,44*coff，56*coff
	}
	if((!valid_12) && (!valid_13) && (!valid_23))
	{
		if(left_line.type==1 && middle_line.type!=1 && right_line.type!=1)
			choosed_left=left_line;
		else if(middle_line.type==1 && left_line.type!=1 && right_line.type!=1)
		{
			if(middle_line.nsta>IPM_WIDTH/2)
				choosed_right=middle_line;
			else
				choosed_left=middle_line;
		}
		else if(right_line.type==1 && middle_line.type!=1 && left_line.type!=1)
			choosed_right=right_line;
		else 
			return;
	}
	else if(valid_12 && (!valid_13) && (!valid_23))
	{
		choosed_left=left_line;
		choosed_right=middle_line;
	}
	else if(valid_13 && (!valid_12) && (!valid_23))
	{
		choosed_left=left_line;
		choosed_right=right_line;
	}
	else if(valid_23 && (!valid_12) && (!valid_13))
	{
		choosed_left=middle_line;
		choosed_right=right_line;
	}
	else if(valid_12 && valid_13 && (!valid_23))
	{
		choosed_left=left_line;
		if(middle_line.val_trust>right_line.val_trust)
			choosed_right=middle_line;
		else
			choosed_right=right_line;
	}
	else if(valid_13 && valid_23 && (!valid_12))
	{
		choosed_right=right_line;
		if(middle_line.val_trust>left_line.val_trust)
			choosed_left=middle_line;
		else
			choosed_left=left_line;
	}
	else if(valid_12 && valid_23 && (!valid_13))
	{
		if((left_line.val_trust+middle_line.val_trust)>(middle_line.val_trust+right_line.val_trust))
		{
			choosed_left=left_line;
			choosed_right=middle_line;
		}
		else
		{
			choosed_left=middle_line;
			choosed_right=right_line;
		}
	}
	else if(valid_13 && valid_23 && valid_12)//0425 
	{
		choosed_left=left_line;
		choose_middle=middle_line;
		choosed_right=right_line;//9.28
	/*	int biggest_num;
		int second_num;
		if(left_line.val_trust>right_line.val_trust)
		{
			choosed_left=left_line;
			choosed_right=middle_line;
		}
		else
		{
			choosed_left=middle_line;
			choosed_right=right_line;
		}*/
	}//4.22 
}

void judge_if_two_lane_valid(Line_IPM line1,Line_IPM line2,int* type,int* val,int pos)//11.24
{
	float angle=line1.angle-line2.angle;
	float width=line2.nsta-line1.nsta;

	if((angle<=16) && (angle>=-16))
	{
		if(width>17.5*CHOOSELANE_PARA && width<32*CHOOSELANE_PARA)
		{
			type[pos]=1;
			val[pos]=line1.val_trust+line2.val_trust;
			return;
		}
		else if(width>35*CHOOSELANE_PARA && width<64*CHOOSELANE_PARA)
		{
			type[pos]=2;
			val[pos]=line1.val_trust+line2.val_trust;
			return;
		}
	}

	type[pos]=0;
	val[pos]=0;
}

void choose_lane3(Line_IPM line1,Line_IPM line2,Line_IPM line3,Line_IPM line4,Line_IPM* choosed_left,Line_IPM* choose_middle,Line_IPM* choosed_right)//9.28
{
	//12,23,34,   13, 24   14
	//type: 1 or 2 or 0
	//val:  gredient 相加
	int type[6];
	int val[6];

	judge_if_two_lane_valid(line1,line2,type,val,0);//12
	judge_if_two_lane_valid(line2,line3,type,val,1);//23
	judge_if_two_lane_valid(line3,line4,type,val,2);//34
	judge_if_two_lane_valid(line1,line3,type,val,3);//13
	judge_if_two_lane_valid(line2,line4,type,val,4);//24
	judge_if_two_lane_valid(line1,line4,type,val,5);//14

	int max=0,max_pos=0;
	for(int i=0;i<6;i++)
	{
		if(max<val[i])
		{
			max=val[i];
			max_pos=i;
		}
	}

	if(max==0)
		return;//没有符号条件的车道线!

	if(max_pos==0 && type[0]==1)//12
	{
		bool v1=(type[1]==1 && type[3]==2);//line3
		bool v2=(type[4]==1 && type[5]==2);//line4

		if(v1 && v2)
		{
			if(line3.val_trust>line4.val_trust)
				v2=false;
			else
				v1=false;
		}

		if(v1)//23,13
		{
			*choosed_left=line1;
			*choose_middle=line2;
			*choosed_right=line3;
		}
		else if(v2)//24,14
		{
			*choosed_left=line1;
			*choose_middle=line2;
			*choosed_right=line4;
		}
		else
		{
			*choosed_left=line1;
			*choosed_right=line2;
		}
	}
	else if(max_pos==1 && type[1]==1)//23
	{
		bool v1=(type[0]==1 && type[3]==2);//line1
		bool v2=(type[2]==1 && type[4]==2);//line4

		if(v1 && v2)
		{
			if(line1.val_trust>line4.val_trust)
				v2=false;
			else
				v1=false;
		}

		if(v1)//12,13
		{
			*choosed_left=line1;
			*choose_middle=line2;
			*choosed_right=line3;
		}
		else if(v2)//34,24
		{
			*choosed_left=line2;
			*choose_middle=line3;
			*choosed_right=line4;
		}
		else
		{
			*choosed_left=line2;
			*choosed_right=line3;
		}
	}
	else if(max_pos==2 && type[2]==1)//34
	{
		bool v1=(type[3]==1 && type[5]==2);//line1
		bool v2=(type[1]==1 && type[4]==2);//line2

		if(v1 && v2)
		{
			if(line1.val_trust>line2.val_trust)
				v2=false;
			else
				v1=false;
		}

		if(v1)//13,14
		{
			*choosed_left=line1;
			*choose_middle=line3;
			*choosed_right=line4;
		}
		else if(v2)//23,24
		{
			*choosed_left=line2;
			*choose_middle=line3;
			*choosed_right=line4;
		}
		else
		{
			*choosed_left=line3;
			*choosed_right=line4;
		}
	}
	else if(max_pos==3)//13
	{
		if(type[3]==1 && type[2]==1 && type[5]==2) //34,14
		{
			*choosed_left=line1;
			*choose_middle=line3;
			*choosed_right=line4;
		}
		else if(type[3]==2 && type[0]==1 && type[1]==1)//12,23
		{
			*choosed_left=line1;
			*choose_middle=line2;
			*choosed_right=line3;
		}
		else
		{
			*choosed_left=line1;
			*choosed_right=line3;
		}
	}
	else if(max_pos==4)//24
	{
		if(type[4]==1 && type[0]==1 && type[5]==2) //12,14
		{
			*choosed_left=line1;
			*choose_middle=line2;
			*choosed_right=line4;
		}
		else if(type[4]==2 && type[1]==1 && type[2]==1)//23,34
		{
			*choosed_left=line2;
			*choose_middle=line3;
			*choosed_right=line4;
		}
		else
		{
			*choosed_left=line2;
			*choosed_right=line4;
		}
	}
	else if(max_pos==5 && type[5]==2)//14
	{
		bool v1=(type[0]==1 && type[4]==1);//line2
		bool v2=(type[2]==1 && type[3]==1);//line3

		if(v1 && v2)
		{
			if(line2.val_trust>line3.val_trust)
				v2=false;
			else
				v1=false;
		}

		if(v1) //12,24
		{
			*choosed_left=line1;
			*choose_middle=line2;
			*choosed_right=line4;
		}
		else if(v2)//13,34
		{
			*choosed_left=line1;
			*choose_middle=line3;
			*choosed_right=line4;
		}
		else
		{
			*choosed_left=line1;
			*choosed_right=line4;
		}
	}
}

void choose_lane2(Line_IPM line1,Line_IPM line2,Line_IPM line3,Line_IPM line4,Line_IPM& choosed_left,Line_IPM& choose_middle,Line_IPM& choosed_right)//9.28
{
	//12,23,34,   13, 24   14
	//type: 1 or 2 or 0  
	//val:  gredient 相加
	int type[6];
	int val[6];

	judge_if_two_lane_valid(line1,line2,type,val,0);//12 
	judge_if_two_lane_valid(line2,line3,type,val,1);//23
	judge_if_two_lane_valid(line3,line4,type,val,2);//34
	judge_if_two_lane_valid(line1,line3,type,val,3);//13
	judge_if_two_lane_valid(line2,line4,type,val,4);//24
	judge_if_two_lane_valid(line1,line4,type,val,5);//14

	int max=0,max_pos=0;
	for(int i=0;i<6;i++)
	{
		if(max<val[i])
		{
			max=val[i];
			max_pos=i;
		}
	}

	if(max==0)
		return;//没有符号条件的车道线!

	if(max_pos==0 && type[0]==1)//12
	{
		bool v1=(type[1]==1 && type[3]==2);//line3
		bool v2=(type[4]==1 && type[5]==2);//line4

		if(v1 && v2)
		{
			if(line3.val_trust>line4.val_trust)
				v2=false;
			else
				v1=false;
		}

		if(v1)//23,13
		{
			choosed_left=line1;
			choose_middle=line2;
			choosed_right=line3;
		}
		else if(v2)//24,14
		{
			choosed_left=line1;
			choose_middle=line2;
			choosed_right=line4;
		}
		else
		{
			choosed_left=line1;
			choosed_right=line2;
		}
	}
	else if(max_pos==1 && type[1]==1)//23
	{
		bool v1=(type[0]==1 && type[3]==2);//line1
		bool v2=(type[2]==1 && type[4]==2);//line4

		if(v1 && v2)
		{
			if(line1.val_trust>line4.val_trust)
				v2=false;
			else
				v1=false;
		}

		if(v1)//12,13      
		{
			choosed_left=line1;
			choose_middle=line2;
			choosed_right=line3;
		}
		else if(v2)//34,24
		{
			choosed_left=line2;
			choose_middle=line3;
			choosed_right=line4;
		}
		else
		{
			choosed_left=line2;
			choosed_right=line3;
		}
	}
	else if(max_pos==2 && type[2]==1)//34
	{
		bool v1=(type[3]==1 && type[5]==2);//line1
		bool v2=(type[1]==1 && type[4]==2);//line2

		if(v1 && v2)
		{
			if(line1.val_trust>line2.val_trust)
				v2=false;
			else
				v1=false;
		}

		if(v1)//13,14   
		{
			choosed_left=line1;
			choose_middle=line3;
			choosed_right=line4;
		}
		else if(v2)//23,24
		{
			choosed_left=line2;
			choose_middle=line3;
			choosed_right=line4;
		}
		else
		{
			choosed_left=line3;
			choosed_right=line4;
		}
	}
	else if(max_pos==3)//13
	{
		if(type[3]==1 && type[2]==1 && type[5]==2) //34,14
		{
			choosed_left=line1;
			choose_middle=line3;
			choosed_right=line4;
		}
		else if(type[3]==2 && type[0]==1 && type[1]==1)//12,23
		{
			choosed_left=line1;
			choose_middle=line2;
			choosed_right=line3;
		}
		else
		{
			choosed_left=line1;
			choosed_right=line3;
		}
	}
	else if(max_pos==4)//24
	{
		if(type[4]==1 && type[0]==1 && type[5]==2) //12,14
		{
			choosed_left=line1;
			choose_middle=line2;
			choosed_right=line4;
		}
		else if(type[4]==2 && type[1]==1 && type[2]==1)//23,34
		{
			choosed_left=line2;
			choose_middle=line3;
			choosed_right=line4;
		}
		else
		{
			choosed_left=line2;
			choosed_right=line4;
		}
	}
	else if(max_pos==5 && type[5]==2)//14
	{
		bool v1=(type[0]==1 && type[4]==1);//line2
		bool v2=(type[2]==1 && type[3]==1);//line3

		if(v1 && v2)
		{
			if(line2.val_trust>line3.val_trust)
				v2=false;
			else
				v1=false;
		}

		if(v1) //12,24
		{
			choosed_left=line1;
			choose_middle=line2;
			choosed_right=line4;
		}
		else if(v2)//13,34
		{
			choosed_left=line1;
			choose_middle=line3;
			choosed_right=line4;
		}
		else
		{
			choosed_left=line1;
			choosed_right=line4;
		}
	}


	//int if_only_one_valid=(int)left_line.if_valid+(int)middle_line.if_valid+(int)right_line.if_valid;
	//if(if_only_one_valid==1)
	//{
	//	if(left_line.type==1)
	//		choosed_left=left_line;
	//	if(right_line.type==1)
	//		choosed_right=right_line;
	//	if(middle_line.type==1)
	//	{
	//		if(middle_line.nsta<IPM_WIDTH/2)
	//			choosed_left=middle_line;
	//		else
	//			choosed_right=middle_line;
	//	}
	//	return;
	//}
	//else if(if_only_one_valid==0)
	//{
	//	return;
	//}

	//float angle_12,angle_23,angle_13;
	//float width_12,width_23,width_13;//10.28
	//bool valid_12,valid_23,valid_13;
	//angle_12=angle_23=angle_13=width_12=width_23=width_13=0;
	//valid_12=valid_23=valid_13=false;

	//if(left_line.if_valid==1 && middle_line.if_valid==1)
	//{
	//	angle_12=middle_line.angle-left_line.angle;
	//	width_12=middle_line.nsta-left_line.nsta;
	//	valid_12=(angle_12<=16) && (angle_12>=-16) && (width_12>17.5*CHOOSELANE_PARA) && (width_12<33*CHOOSELANE_PARA);//4.20。计算依据：29*coff,44*coff
	//}
	//if(middle_line.if_valid==1 && right_line.if_valid==1)
	//{
	//	angle_23=right_line.angle-middle_line.angle;
	//	width_23=right_line.nsta-middle_line.nsta;
	//	valid_23=(angle_23<=16) && (angle_23>=-16) && (width_23>17.5*CHOOSELANE_PARA)&& (width_23<33*CHOOSELANE_PARA);//4.20
	//}
	//if(left_line.if_valid==1 && right_line.if_valid==1)
	//{
	//	angle_13=right_line.angle-left_line.angle;
	//	width_13=right_line.nsta-left_line.nsta;
	//	valid_13=(angle_13<=16) && (angle_13>=-16) && (((width_13>17.5*CHOOSELANE_PARA) &&(width_13<33*CHOOSELANE_PARA))||(width_13>39*CHOOSELANE_PARA));////4.20。计算依据：29*coff,44*coff，56*coff
	//}
	//if((!valid_12) && (!valid_13) && (!valid_23))
	//{
	//	if(left_line.type==1 && middle_line.type!=1 && right_line.type!=1)
	//		choosed_left=left_line;
	//	else if(middle_line.type==1 && left_line.type!=1 && right_line.type!=1)
	//	{
	//		if(middle_line.nsta>IPM_WIDTH/2)
	//			choosed_right=middle_line;
	//		else
	//			choosed_left=middle_line;
	//	}
	//	else if(right_line.type==1 && middle_line.type!=1 && left_line.type!=1)
	//		choosed_right=right_line;
	//	else 
	//		return;
	//}
	//else if(valid_12 && (!valid_13) && (!valid_23))
	//{
	//	choosed_left=left_line;
	//	choosed_right=middle_line;
	//}
	//else if(valid_13 && (!valid_12) && (!valid_23))
	//{
	//	choosed_left=left_line;
	//	choosed_right=right_line;
	//}
	//else if(valid_23 && (!valid_12) && (!valid_13))
	//{
	//	choosed_left=middle_line;
	//	choosed_right=right_line;
	//}
	//else if(valid_12 && valid_13 && (!valid_23))
	//{
	//	choosed_left=left_line;
	//	if(middle_line.val_trust>right_line.val_trust)
	//		choosed_right=middle_line;
	//	else
	//		choosed_right=right_line;
	//}
	//else if(valid_13 && valid_23 && (!valid_12))
	//{
	//	choosed_right=right_line;
	//	if(middle_line.val_trust>left_line.val_trust)
	//		choosed_left=middle_line;
	//	else
	//		choosed_left=left_line;
	//}
	//else if(valid_12 && valid_23 && (!valid_13))
	//{
	//	if((left_line.val_trust+middle_line.val_trust)>(middle_line.val_trust+right_line.val_trust))
	//	{
	//		choosed_left=left_line;
	//		choosed_right=middle_line;
	//	}
	//	else
	//	{
	//		choosed_left=middle_line;
	//		choosed_right=right_line;
	//	}
	//}
	//else if(valid_13 && valid_23 && valid_12)//0425 
	//{
	//	choosed_left=left_line;
	//	choose_middle=middle_line;
	//	choosed_right=right_line;//9.28
	///*	int biggest_num;
	//	int second_num;
	//	if(left_line.val_trust>right_line.val_trust)
	//	{
	//		choosed_left=left_line;
	//		choosed_right=middle_line;
	//	}
	//	else
	//	{
	//		choosed_left=middle_line;
	//		choosed_right=right_line;
	//	}*/
	//}//4.22 
}

int track_lane(Line_IPM &left,Line_IPM &middle,Line_IPM &right)
{
	int left_W,left_ang,right_W,right_ang;
	left_W=left_ang=right_W=right_ang=0;

	int left_kind=0;
	int right_kind=0;

	int left_tracker_num=-1;
	int middle_tracker_num=-1;
	int right_tracker_num=-1;

	int final_left_state=0;
	int final_right_state=0;
	int final_state;

	if(left.if_valid==1)
	{
		if((left.nsta<=IPM_WIDTH/2-SEGMENT))//9.28修改
			left_kind=1;
		else 
			left_kind=2;
	}
	else
		left_kind=0;

	if(right.if_valid==1)
	{
		if((right.nsta>=IPM_WIDTH/2+SEGMENT))//9.28修改
			right_kind=3;
		else
			right_kind=2;
	}
	else
		right_kind=0;
	//标记当前车道线是否位于middle的范围内

	if(left.if_valid==1)
	{
		for(int i=0;i<tracker_num;i++)
		{
			if(tracker[i].state_value>0 && left_tracker_num==-1)
			{
				if(left_kind==2)
				{
					if(judge_if_track_valid(tracker[i],left))
					{
						tracker[i].state_value+=1;
						if(tracker[i].state_value>6)
							tracker[i].state_value=6;
						tracker[i].pre_lane=left;
						tracker[i].kind=left_kind;
						tracker[i].invalid_fps=0;
						left_tracker_num=i;
						break;
					}
				}
				else if(left_kind==1 && (tracker[i].kind==1 || tracker[i].kind==2))
				{
					if(judge_if_track_valid(tracker[i],left))
					{
						tracker[i].state_value+=1;
						if(tracker[i].state_value>6)
							tracker[i].state_value=6;
						tracker[i].pre_lane=left;
						tracker[i].kind=left_kind;
						tracker[i].invalid_fps=0;
						left_tracker_num=i;
						break;
					}
				}
			}
		}
	}

	if(middle.if_valid==1)
	{
		for(int i=0;i<tracker_num;i++)
		{
			if(tracker[i].state_value>0 && middle_tracker_num==-1)
			{
				if(judge_if_track_valid(tracker[i],middle))
				{
					tracker[i].state_value+=1;
					if(tracker[i].state_value>6)
						tracker[i].state_value=6;
					tracker[i].pre_lane=middle;
					tracker[i].kind=2;
					tracker[i].invalid_fps=0;
					middle_tracker_num=i;
					break;
				}
			}
		}
	}

	if(right.if_valid==1)
	{
		for(int i=0;i<tracker_num;i++)
		{
			if(tracker[i].state_value>0 && right_tracker_num==-1)
			{
				if(right_kind==2)
				{
					if(judge_if_track_valid(tracker[i],right))
					{
						tracker[i].state_value+=1;
						if(tracker[i].state_value>6)
							tracker[i].state_value=6;
						tracker[i].pre_lane=right;
						tracker[i].kind=right_kind;
						tracker[i].invalid_fps=0;
						right_tracker_num=i;
						break;
					}
				}
				else if(right_kind==3 && (tracker[i].kind==3 || tracker[i].kind==2))
				{
					if(judge_if_track_valid(tracker[i],right))
					{
						tracker[i].state_value+=1;
						if(tracker[i].state_value>6)
							tracker[i].state_value=6;
						tracker[i].pre_lane=right;
						tracker[i].kind=right_kind;
						tracker[i].invalid_fps=0;
						right_tracker_num=i;
						break;
					}
				}
			}
		}
	}
	//寻找是否有上一帧的lane，接近此次的位置。

	for(int i=0;i<tracker_num;i++)
	{
		if(i!=right_tracker_num && i!=left_tracker_num && i!=middle_tracker_num &&tracker[i].state_value!=0)
		{
			tracker[i].invalid_fps+=1;
			if(tracker[i].invalid_fps>3)
				tracker[i].invalid_fps=3;

			tracker[i].state_value-=tracker[i].invalid_fps;
			if(tracker[i].state_value<=0)
			{
				initialize_tracker(i);
			}
		}
	}
	//将没有被跟踪车道线的值刷新

	if(left.if_valid==1 && left_tracker_num==-1)
	{
		for(int i=0;i<tracker_num;i++)
		{
			if(tracker[i].state_value==0)
			{
				tracker[i].invalid_fps=0;
				tracker[i].pre_lane=left;
				tracker[i].kind=left_kind;
				tracker[i].state_value=2;
				left_tracker_num=i;
				break;
			}
		}
	}
	if(right.if_valid==1 && right_tracker_num==-1)
	{
		for(int i=0;i<tracker_num;i++)
		{
			if(tracker[i].state_value==0)
			{
				tracker[i].invalid_fps=0;
				tracker[i].pre_lane=right;
				tracker[i].kind=right_kind;
				tracker[i].state_value=2;
				right_tracker_num=i;
				break;
			}
		}
	}
	if(middle.if_valid==1 && middle_tracker_num==-1)
	{
		for(int i=0;i<tracker_num;i++)
		{
			if(tracker[i].state_value==0)
			{
				tracker[i].invalid_fps=0;
				tracker[i].pre_lane=middle;
				tracker[i].kind=2;
				tracker[i].state_value=2;
				middle_tracker_num=i;
				break;
			}
		}
	}
	//若旧的跟踪数据都跟不上，则建立tracker供下一帧跟踪。

	if(middle_tracker_num!=-1)
		middle.val_track=tracker[middle_tracker_num].state_value;
	else
		middle.val_track=0;

	if(middle.if_valid==1)
	{
		if(middle.nsta<IPM_WIDTH/2)
		{
			left=middle;
			left_tracker_num=middle_tracker_num;
		}
		else
		{
			right=middle;
			right_tracker_num=middle_tracker_num;
		}
		//if(left.val_trust>right.val_trust)
		//{
		//	right=middle;
		//	right_tracker_num=middle_tracker_num;
		//}
		//else
		//{
		//	left=middle;
		//	left_tracker_num=middle_tracker_num;
		//}
	}
	
	if(left_tracker_num!=-1)
		left.val_track=final_left_state=tracker[left_tracker_num].state_value;
	else
		left.val_track=final_left_state=0;

	if(right_tracker_num!=-1)
		right.val_track=final_right_state=tracker[right_tracker_num].state_value;
	else
		right.val_track=final_right_state=0;

	//若left，right无数据，则跟踪状态值为0

	final_state=final_right_state+final_left_state;

	//排序：
	lane_tracker tmp;
	int flag=0;
	for(int i = 0; i < tracker_num; i++)  //外层循环控制循环次数
    {
        for(int j = 0; j < tracker_num-1-i; j++)    //内层循环控制每次循环里比较的次数。
        {
            if(tracker[j].state_value < tracker[j+1].state_value)
            {
                tmp = tracker[j];
                tracker[j] = tracker[j+1];
                tracker[j+1] = tmp;
                flag = 1;
            }
        }
    
        if(0 == flag)
            break;
    }
	
	return final_state;
}

bool judge_if_track_valid(lane_tracker track_para,Line_IPM line)
{
	int k=track_para.invalid_fps+1;
	float nsta_sub=track_para.pre_lane.nsta-line.nsta;
	float angle_sub=track_para.pre_lane.angle-line.angle;
	if((nsta_sub<=1.8*CHOOSELANE_PARA*k && nsta_sub>=-1.8*CHOOSELANE_PARA*k) && (angle_sub<=12 && angle_sub>=-12))//9.28
		return true;
	else
		return false;
	//判断当前车道线与上一帧的是否匹配
}

void initialize_tracker(int i)
{
	tracker[i].state_value=0;
	tracker[i].kind=0;
	tracker[i].invalid_fps=0;
	//初始化tracker
}

void initialize_line(Line_IPM* line)
{
	line->if_valid=0;
	line->type=0;
	line->val_trust=0;
	line->val_1=0;
	line->val_2=0;
	line->val_3=0;
	line->val_track=0;//3.7
}

int if_warn(Line_IPM choosed_left,Line_IPM choosed_right,int _track)//114改
{
	int warning=0;
	if(choosed_left.if_valid && choosed_right.if_valid)
	{//双线模式，需要tracker大于8
		if(_track>=8)
		{
			if(choosed_right.nsta-choosed_left.nsta<56)
			{
				//未剔除弯道误报
				//if(((choosed_right.nsta+choosed_left.nsta-IPM_WIDTH)>15) || ((choosed_right.nsta+choosed_left.nsta-IPM_WIDTH)<-15))//车道在车头中间。
				//{
				//	if(choosed_right.angle+choosed_left.angle-numangle<-8)//根据角度判断！！左偏还是右偏！//中间骑线，报不报警？
				//	{
				//		//right_warning++;
				//		warning=1;
				//	}
				//	else if(choosed_right.angle+choosed_left.angle-numangle>8)
				//		//left_warning++;
				//		warning=-1;
				//	else
				//		warning=-3;//骑线
				//}
				//未剔除弯道误报

				//提出弯道version 1
				//if(((choosed_right.nsta+choosed_left.nsta-IPM_WIDTH)>15) && choosed_right.angle+choosed_left.angle-numangle>8)
				//	warning=-1;
				//else if(((choosed_right.nsta+choosed_left.nsta-IPM_WIDTH)<-15) && choosed_right.angle+choosed_left.angle-numangle<-8)
				//	warning=1;
				//else if(((choosed_right.nsta+choosed_left.nsta-IPM_WIDTH)>15) || ((choosed_right.nsta+choosed_left.nsta-IPM_WIDTH)<-15))
				//	warning=-3;
				//剔除弯道version 1

				//2.18
				//剔除弯道version 2
				if((((choosed_right.nsta+choosed_left.nsta-IPM_WIDTH)>15) || ((choosed_right.nsta+choosed_left.nsta-IPM_WIDTH)<-23)) && choosed_right.angle+choosed_left.angle-numangle>5)//2.19,8改5
					warning=-1;
				else if((((choosed_right.nsta+choosed_left.nsta-IPM_WIDTH)<-15) || ((choosed_right.nsta+choosed_left.nsta-IPM_WIDTH)>23)) && choosed_right.angle+choosed_left.angle-numangle<-5)
					warning=1;
				else if(((choosed_right.nsta+choosed_left.nsta-IPM_WIDTH)>15) || ((choosed_right.nsta+choosed_left.nsta-IPM_WIDTH)<-15))
					warning=-3;
				//剔除弯道version 2
				//2.18

				//不要了
				//else if(((choosed_right.angle+choosed_left.angle-numangle)>30))//车道不在车头中间，但是偏离角度很大
				//	warning=-1;
				//else if((choosed_right.angle+choosed_left.angle-numangle)<-30)//车道不在车头中间，但是偏离角度很大
				//	warning=1;
				//不要了
				else
					warning=0;
			}
			else
				warning=0;
		}
		else
			warning=-2;//没跟踪到
	}
	else if(choosed_right.if_valid && (!choosed_left.if_valid))
	{//单线模式，需要tracker大于6
		if(_track>=6)
		{
			if((choosed_right.nsta-IPM_WIDTH/2)<6 && (choosed_right.nsta-IPM_WIDTH/2)>-6)//单线在车头中间位置，
			{
				if(choosed_right.angle-numangle/2<-3)//根据角度判断！！左偏还是右偏！
					warning=1;
				else if(choosed_right.angle-numangle/2>3)
					warning=-1;
				else
					warning=-3;//单线在车头中间，角度不偏
			}
			else if((choosed_right.nsta-IPM_WIDTH/2)>26)
				warning=-3;//另一侧的单线偏车头很多。骑线
			else if((choosed_right.nsta-IPM_WIDTH/2)>12 && (choosed_right.nsta-IPM_WIDTH/2)<22)
				warning=0;//nsta在距离中间线12-22范围内
			else
				warning=-4;
		}
		else
			warning=-2;
	}
	else if(choosed_left.if_valid && (!choosed_right.if_valid))
	{//单线模式，需要tracker大于6
		if(_track>=6)
		{
			if((choosed_left.nsta-IPM_WIDTH/2)<6 && (choosed_left.nsta-IPM_WIDTH/2)>-6)//单线在车头中间位置，
			{
				if(choosed_left.angle-numangle/2<-3)//根据角度判断！！左偏还是右偏！
					warning=1;
				else if(choosed_left.angle-numangle/2>3)
					warning=-1;
				else
					warning=-3;//中间线骑线
			}
			else if((choosed_left.nsta-IPM_WIDTH/2)<-26)
				warning=-3;//另外一边骑线
			else if((choosed_right.nsta-IPM_WIDTH/2)>-22 && (choosed_right.nsta-IPM_WIDTH/2)<-12)
				warning=-4;
			else
				warning=0;//nsta在距离中间线12-22范围内
		}
		else
			warning=-2;
	}
	else
		warning=-2;

	return warning;
}
int new_warning(Line_IPM choosed_left,Line_IPM choosed_right,int _track,float &s1,float &s2,float &s3,float &s4,int &departure)//0226版本新增函数实现
{
	int warning=0;
	if(choosed_left.if_valid && choosed_right.if_valid)
	{//双线模式，需要tracker大于8
		if(_track>=8)//&&choosed_left.val_track>=3&&choosed_right.val_track>=3)//0425增加跟踪数目
		{
	
			//先把检测到的点转换成当前车道中的点
			int lefttrack=0;//0811
			int righttrack=0;//0811
 			int anglemid=(choosed_left.angle+choosed_left.angle)/2;
  			float P1=0,P2=0;
  			float M=45;//IPM_WIDTH/2;
			/*float A1=(choosed_right.nsta)*1.43-2.86;
			float A2=(choosed_left.nsta)*1.43-2.86;*/
			float A1=(choosed_right.nsta-IPM_WIDTH/2+0.5)/CALIBRATE_PARA+45;
			float A2=(choosed_left.nsta-IPM_WIDTH/2+0.5)/CALIBRATE_PARA+45;

			
		/*	if(choosed_left.if_valid)
				std::cout<<"A1: "<<A1<<"     ";
			if(choosed_right.if_valid)
				std::cout<<"A2: "<<A2<<"     "<<std::endl;*/
			/*if(choosed_left.if_valid && choosed_right.if_valid)
			{
				std::cout<<"width: "<<choosed_right.nsta-choosed_left.nsta<<"     ";
				std::cout<<"middle: "<<(choosed_right.nsta+choosed_left.nsta)/2;
			}*/

			//0227版本——————————
			float T=0;
			if(A1-A2<56&&A1-A2>30)
				T=9;
			else
			{
				T=((A1-A2)/37.0*9);
			}
			//——————————————
			if(A1-A2<56)
			{
				if(A1>M&&M>=A2)
				{
					P1=A1;
					P2=A2;
					lefttrack=choosed_left.val_track;//0811
					righttrack=choosed_right.val_track;//0811
				}
				else
				{
					if(A2<A1&&A1<=M)
					{
						P1=A1+(A1-A2);
						P2=A1;
						lefttrack=choosed_right.val_track;//0811
						righttrack=choosed_left.val_track;//0811
					}
					else
					{
						if(M<A2&&A2<A1)
						{
							P1=A2;
							P2=A2-(A1-A2);
							righttrack=choosed_left.val_track;//0811
							lefttrack=choosed_right.val_track;//0811
						}
					}
				}
//				P1=P1+4;
//				P2=P2-2;
				if((s1<0.00001)&&(s2<0.00001))
				{
					s1=P1-M;
					s2=M-P2;
					if((s1<T+1)||(s2<T+1))//&&midflag==0)
					{
						s1=0;
						s2=0;
						s3=0;
						s4=0;
						return warning=0;
					}
				}
				else
				{
					s3=P1-M;
					s4=M-P2;
				}

				if(departure==1)
				{
					if((s3>T+2)&&(s4>T+2))//0227版本
					{
						departure=0;
						s1=0;
						s2=0;
						s3=0;
						s4=0;
						warning=0;
					}
				}
				else
				{
					if((s3>T)&&(s4>T))//0227版本
					{
						departure=0;
						s1=s3;
						s2=s4;
						s3=0;
						s4=0;
						warning=0;
					}
					else
					{
						if(!((s3<0.00001)&&(s4<0.00001)))
						{
							if((s3<=T)&&(s3>0)&&(departure==0))//0227版本
							{
								//if(righttrack>=2)//0811
								//{
									if((s1>s2)&&(s2<15)&&anglemid>numangle/2-5)//s2此条件小的话，限制会强
									{
										departure=1;
										warning=-1;//左偏
										s1=0;
										s2=0;
										s3=0;
										s4=0;
									}
									else
									{
										if((s1<s2)&&(s1<18)&&anglemid<numangle/2+5)
										{
											departure=1;
											warning=1;//右偏
											s1=0;
											s2=0;
											s3=0;
											s4=0;
										}
									}
								//}
							}
							else
							{
								if((s4<=T)&&(s4>0)&&(departure==0))//0227版本
								{
									//if(lefttrack>=2)//0811
									//{
										if((s1>s2)&&(s2<18)&&anglemid>numangle/2-5)
										{
											departure=1;
											warning=-1;
											s1=0;
											s2=0;
											s3=0;
											s4=0;
										}
										else
										{
											if((s1<s2)&&(s1<15)&&anglemid<numangle/2+5)
											{
												departure=1;
												warning=1;
												s1=0;
												s2=0;
												s3=0;
												s4=0;
											}
										}
									//}
								}
								else
								{
									//if(s4<0.000001&&lefttrack>=2)
									if(s4<0.000001)//改
									{
										if((s1<s2)&&(s1<18))
										{
											departure=1;
											warning=1;
											s1=0;
											s2=0;
											s3=0;
											s4=0;
										}
										else
										{
											if((s1>s2)&&(s2<15))
											{
												departure=1;
												warning=-1;
												s1=0;
												s2=0;
												s3=0;
												s4=0;
											}
										}
									}
									else
									{
										departure=0;//不应运行到此步骤，此步骤是压线但不报警
										warning=-3;
									}
								}
							}
						}
					}
				}
			}
			else
			{
//				departure=0;//0227版本
//				s1=0;//0227版本
//				s2=0;
//				s3=0;
//				s4=0;
				warning=0;

			}
		}
		else
		{
			departure=0;//0227版本
			s1=0;//0227版本
			s2=0;
			s3=0;
			s4=0;
			warning=-2;//没跟踪到
		}
	}

	if(warning==1 || warning==-1)
	{
		int L_nsta=abs(IPM_WIDTH/2-choosed_left.nsta);
		int R_nsta=abs(IPM_WIDTH/2-choosed_right.nsta);

		if((L_nsta<R_nsta && choosed_left.val_track<4) || (L_nsta>R_nsta && choosed_right.val_track<4))
 			warning=0;
	}//抑制误报   11.28!!!

	return warning;
}

void cal_LDW_perspective_mat_4x(float D,Point xiaoshi,Point p3,Point p4,float * X,float offset)//420
{
	//下位机
	float A[8][8];
	float b[8];
	//double X[8];

	float src_x[4],src_y[4],dst_x[4],dst_y[4];

	src_x[0]=float(xiaoshi.x)-0.5;//x0'
	src_y[0]=float(xiaoshi.y);
	src_x[1]=float(xiaoshi.x)+0.5;
	src_y[1]=float(xiaoshi.y);
	src_x[2]=float(p3.x);
	src_y[2]=float(p3.y);
	src_x[3]=float(p4.x);
	src_y[3]=float(p4.y);

	dst_x[0]=375.5-0.7*3*5*D+0.7*3*offset/10;//9.17
	dst_y[0]=6500*3;
	dst_x[1]=375.5+0.7*3*5*D+0.7*3*offset/10;//9.17
	dst_y[1]=6500*3;
	dst_x[2]=375.5-0.7*3*5*D+0.7*3*offset/10;//9.17
	dst_y[2]=0;//9.17
	dst_x[3]=375.5+0.7*3*5*D+0.7*3*offset/10;//9.17
	dst_y[3]=0;//9.17

	//for(int i=0;i<4;i++)
	//{
	//	dst_x[i]*=4;
	//	dst_y[i]*=4;
	//}

	float* mat1_data_ptr=A[0];
	memset(A[0],0,64*sizeof(float));
	for(int i=0;i<4;i++)
	{
		mat1_data_ptr[2*i*8]=dst_x[i];mat1_data_ptr[2*i*8+1]=dst_y[i];mat1_data_ptr[2*i*8+2]=1;
		mat1_data_ptr[2*i*8+6]=-dst_x[i]*src_x[i];mat1_data_ptr[2*i*8+7]=-dst_y[i]*src_x[i];

		mat1_data_ptr[2*i*8+11]=dst_x[i];mat1_data_ptr[2*i*8+12]=dst_y[i];mat1_data_ptr[2*i*8+13]=1;
		mat1_data_ptr[2*i*8+14]=-dst_x[i]*src_y[i];mat1_data_ptr[2*i*8+15]=-dst_y[i]*src_y[i];

		b[2*i]=src_x[i];
		b[2*i+1]=src_y[i];
	}

	int size=8;
	int i,j,k,p;     //计数器
    float Aik;     //正消过程用到的变量名
    float S;       //回代过程用到的变量名
	//int order[8]={1,2,3,4,5,6,7,8};
	float temp[8],temp_b;
	//int temp_order;

	for(k=0;k<size-1;k++)
    {
		p=k;
		while(A[p][k]==0)
		{
			p=p+1;
		}
		if(p!=k)
		{
			for(int o=0;o<8;o++)
			{
				temp[o]=A[k][o];
				A[k][o]=A[p][o];
				A[p][o]=temp[o];
			}
			//temp_order=order[k];
			//order[k]=order[p];
			//order[p]=temp_order;

			temp_b=b[k];
			b[k]=b[p];
			b[p]=temp_b;
		}
        for(i=k+1;i<size;i++)
        {
			Aik=A[i][k]/A[k][k];
			for(j=k;j<size;j++)
			{
				A[i][j]=A[i][j]-Aik*A[k][j];
			}
			b[i]=b[i]-Aik*b[k];
        }
    }

    X[size-1]=b[size-1]/A[size-1][size-1];
    for(k=size-2;k>=0;k--)
    {
        S=b[k];
        for(j=k+1;j<size;j++)
        {
            S=S-A[k][j]*X[j];
        }
        X[k]=S/A[k][k];
    }    //回代

	X[8]=1;
}


void cal_LDW_perspective_mat(float D,Point xiaoshi,Point p3,Point p4,float * X,float * inv_X,float offset)
{
	//下位机
	float A[8][8];
	float b[8];
	//double X[8];

	float src_x[4],src_y[4],dst_x[4],dst_y[4];

	src_x[0]=float(xiaoshi.x)-0.5;//x0'
	src_y[0]=float(xiaoshi.y);
	src_x[1]=float(xiaoshi.x)+0.5;
	src_y[1]=float(xiaoshi.y);
	src_x[2]=float(p3.x);
	src_y[2]=float(p3.y);
	src_x[3]=float(p4.x);
	src_y[3]=float(p4.y);

	dst_x[0]=375.5-CALIBRATE_PARA*5*D+CALIBRATE_PARA*offset/10;//x0,y0//9.17

	dst_x[1]=375.5+CALIBRATE_PARA*5*D+CALIBRATE_PARA*offset/10;//x1,y1//9.17
	
	dst_x[2]=375.5-CALIBRATE_PARA*5*D+CALIBRATE_PARA*offset/10;//9.17
	dst_y[2]=0;//8.26
	dst_x[3]=375.5+CALIBRATE_PARA*5*D+CALIBRATE_PARA*offset/10;//9.17
	dst_y[3]=0;//8.26

#if only_near_car
		dst_y[0]=calib_y;
		dst_y[1]=calib_y;
#else
		dst_y[0]=6500;
		dst_y[1]=6500;
#endif

	float* mat1_data_ptr=A[0];
	memset(A[0],0,64*sizeof(float));
	for(int i=0;i<4;i++)
	{
		mat1_data_ptr[2*i*8]=dst_x[i];mat1_data_ptr[2*i*8+1]=dst_y[i];mat1_data_ptr[2*i*8+2]=1;
		mat1_data_ptr[2*i*8+6]=-dst_x[i]*src_x[i];mat1_data_ptr[2*i*8+7]=-dst_y[i]*src_x[i];

		mat1_data_ptr[2*i*8+11]=dst_x[i];mat1_data_ptr[2*i*8+12]=dst_y[i];mat1_data_ptr[2*i*8+13]=1;
		mat1_data_ptr[2*i*8+14]=-dst_x[i]*src_y[i];mat1_data_ptr[2*i*8+15]=-dst_y[i]*src_y[i];

		b[2*i]=src_x[i];
		b[2*i+1]=src_y[i];
	}

	int size=8;
	int i,j,k,p;     //计数器
    float Aik;     //正消过程用到的变量名
    float S;       //回代过程用到的变量名
	//int order[8]={1,2,3,4,5,6,7,8};
	float temp[8],temp_b;
	//int temp_order;

	for(k=0;k<size-1;k++)
    {
		p=k;
		while(A[p][k]==0)
		{
			p=p+1;
		}
		if(p!=k)
		{
			for(int o=0;o<8;o++)
			{
				temp[o]=A[k][o];
				A[k][o]=A[p][o];
				A[p][o]=temp[o];
			}
			//temp_order=order[k];
			//order[k]=order[p];
			//order[p]=temp_order;

			temp_b=b[k];
			b[k]=b[p];
			b[p]=temp_b;
		}
        for(i=k+1;i<size;i++)
        {
			Aik=A[i][k]/A[k][k];
			for(j=k;j<size;j++)
			{
				A[i][j]=A[i][j]-Aik*A[k][j];
			}
			b[i]=b[i]-Aik*b[k];
        }
    }

    X[size-1]=b[size-1]/A[size-1][size-1];
    for(k=size-2;k>=0;k--)
    {
        S=b[k];
        for(j=k+1;j<size;j++)
        {
            S=S-A[k][j]*X[j];
        }
        X[k]=S/A[k][k];
    }    //回代

	X[8]=1;

	//以下9.29
	float a1=X[0];
	float b1=X[1];
	float c1=X[2];
	float a2=X[3];
	float b2=X[4];
	float c2=X[5];
	float a3=X[6];
	float b3=X[7];
	float c3=X[8];

	float inv_fenmu=a1*(b2*c3-c2*b3)-a2*(b1*c3-c1*b3)+a3*(b1*c2-c1*b2);
	inv_X[0]=(b2*c3-c2*b3)/inv_fenmu;
	inv_X[1]=(b3*c1-c3*b1)/inv_fenmu;
	inv_X[2]=(b1*c2-c1*b2)/inv_fenmu;

	inv_X[3]=(c2*a3-a2*c3)/inv_fenmu;
	inv_X[4]=(c3*a1-a3*c1)/inv_fenmu;
	inv_X[5]=(c1*a2-a1*c2)/inv_fenmu;

	inv_X[6]=(a2*b3-b2*a3)/inv_fenmu;
	inv_X[7]=(a3*b1-b3*a1)/inv_fenmu;
	inv_X[8]=(a1*b2-b1*a2)/inv_fenmu;
}

void cal_LDW_perspective_mat_near_car(float D,Point xiaoshi,Point p3,Point p4,float * X,float * inv_X,float offset)
{
	//下位机
	float A[8][8];
	float b[8];
	//double X[8];

	float src_x[4],src_y[4],dst_x[4],dst_y[4];

	src_x[0]=float(xiaoshi.x)-0.5;//x0'
	src_y[0]=float(xiaoshi.y);
	src_x[1]=float(xiaoshi.x)+0.5;
	src_y[1]=float(xiaoshi.y);
	src_x[2]=float(p3.x);
	src_y[2]=float(p3.y);
	src_x[3]=float(p4.x);
	src_y[3]=float(p4.y);

	dst_x[0]=375.5-CALIBRATE_PARA*5*D+CALIBRATE_PARA*offset/10;//x0,y0//9.17
	dst_y[0]=calib_y*calib_multiple;
	dst_x[1]=375.5+CALIBRATE_PARA*5*D+CALIBRATE_PARA*offset/10;//x1,y1//9.17
	dst_y[1]=calib_y*calib_multiple;
	dst_x[2]=375.5-CALIBRATE_PARA*5*D+CALIBRATE_PARA*offset/10;//9.17
	dst_y[2]=calib_plus;//8.26
	dst_x[3]=375.5+CALIBRATE_PARA*5*D+CALIBRATE_PARA*offset/10;//9.17
	dst_y[3]=calib_plus;//8.26

	float* mat1_data_ptr=A[0];
	memset(A[0],0,64*sizeof(float));
	for(int i=0;i<4;i++)
	{
		mat1_data_ptr[2*i*8]=dst_x[i];mat1_data_ptr[2*i*8+1]=dst_y[i];mat1_data_ptr[2*i*8+2]=1;
		mat1_data_ptr[2*i*8+6]=-dst_x[i]*src_x[i];mat1_data_ptr[2*i*8+7]=-dst_y[i]*src_x[i];

		mat1_data_ptr[2*i*8+11]=dst_x[i];mat1_data_ptr[2*i*8+12]=dst_y[i];mat1_data_ptr[2*i*8+13]=1;
		mat1_data_ptr[2*i*8+14]=-dst_x[i]*src_y[i];mat1_data_ptr[2*i*8+15]=-dst_y[i]*src_y[i];

		b[2*i]=src_x[i];
		b[2*i+1]=src_y[i];
	}

	int size=8;
	int i,j,k,p;     //计数器
    float Aik;     //正消过程用到的变量名
    float S;       //回代过程用到的变量名
	//int order[8]={1,2,3,4,5,6,7,8};
	float temp[8],temp_b;
	//int temp_order;

	for(k=0;k<size-1;k++)
    {
		p=k;
		while(A[p][k]==0)
		{
			p=p+1;
		}
		if(p!=k)
		{
			for(int o=0;o<8;o++)
			{
				temp[o]=A[k][o];
				A[k][o]=A[p][o];
				A[p][o]=temp[o];
			}
			//temp_order=order[k];
			//order[k]=order[p];
			//order[p]=temp_order;

			temp_b=b[k];
			b[k]=b[p];
			b[p]=temp_b;
		}
        for(i=k+1;i<size;i++)
        {
			Aik=A[i][k]/A[k][k];
			for(j=k;j<size;j++)
			{
				A[i][j]=A[i][j]-Aik*A[k][j];
			}
			b[i]=b[i]-Aik*b[k];
        }
    }

    X[size-1]=b[size-1]/A[size-1][size-1];
    for(k=size-2;k>=0;k--)
    {
        S=b[k];
        for(j=k+1;j<size;j++)
        {
            S=S-A[k][j]*X[j];
        }
        X[k]=S/A[k][k];
    }    //回代

	X[8]=1;

	//以下9.29
	float a1=X[0];
	float b1=X[1];
	float c1=X[2];
	float a2=X[3];
	float b2=X[4];
	float c2=X[5];
	float a3=X[6];
	float b3=X[7];
	float c3=X[8];

	float inv_fenmu=a1*(b2*c3-c2*b3)-a2*(b1*c3-c1*b3)+a3*(b1*c2-c1*b2);
	inv_X[0]=(b2*c3-c2*b3)/inv_fenmu;
	inv_X[1]=(b3*c1-c3*b1)/inv_fenmu;
	inv_X[2]=(b1*c2-c1*b2)/inv_fenmu;

	inv_X[3]=(c2*a3-a2*c3)/inv_fenmu;
	inv_X[4]=(c3*a1-a3*c1)/inv_fenmu;
	inv_X[5]=(c1*a2-a1*c2)/inv_fenmu;

	inv_X[6]=(a2*b3-b2*a3)/inv_fenmu;
	inv_X[7]=(a3*b1-b3*a1)/inv_fenmu;
	inv_X[8]=(a1*b2-b1*a2)/inv_fenmu;
}

void cal_FCW_mat(float D,Point xiaoshi,Point p3,Point p4,float * X,float offset)
{
	//cv::Mat mat1=cv::Mat(8, 8, CV_32F,cvScalarAll(0));
	//float* mat1_data=(float*)mat1.data;
	//
	////Point dst1,dst2,dst3,dst4,src1,src2,src3,src4;
	//float src_x[4],src_y[4],dst_x[4],dst_y[4];

	//src_x[0]=float(xiaoshi.x)-0.5;//x0'
	//src_y[0]=float(xiaoshi.y);
	//src_x[1]=float(xiaoshi.x)+0.5;
	//src_y[1]=float(xiaoshi.y);
	//src_x[2]=float(p3.x);
	//src_y[2]=float(p3.y);
	//src_x[3]=float(p4.x);
	//src_y[3]=float(p4.y);

	//dst_x[0]=375.5-5*D;//x0,y0
	//dst_y[0]=33000;
	//dst_x[1]=375.5+5*D;//x1,y1
	//dst_y[1]=33000;
	//dst_x[2]=375.5-5*D;
	//dst_y[2]=0;
	//dst_x[3]=375.5+5*D;
	//dst_y[3]=0;

	//for(int i=0;i<4;i++)
	//{
	//	mat1_data[2*i*8]=src_x[i];mat1_data[2*i*8+1]=src_y[i];mat1_data[2*i*8+2]=1;
	//	mat1_data[2*i*8+6]=-src_x[i]*dst_x[i];mat1_data[2*i*8+7]=-src_y[i]*dst_x[i];

	//	mat1_data[2*i*8+11]=src_x[i];mat1_data[2*i*8+12]=src_y[i];mat1_data[2*i*8+13]=1;
	//	mat1_data[2*i*8+14]=-src_x[i]*dst_y[i];mat1_data[2*i*8+15]=-src_y[i]*dst_y[i];
	//}
	//
	//double* mat1_data_ptr=A[0];
	//memset(A[0],0,64*sizeof(double));
	//cv::Mat mat1_inverse=mat1.inv();
	//float* mat1_inv_data=(float*)mat1_inverse.data;
	//for(int i=0;i<8;i++)
	//{
	//	FCW_mat[i]=dst_x[0]*mat1_inv_data[i*8]+dst_y[0]*mat1_inv_data[i*8+1]
	//	+dst_x[1]*mat1_inv_data[i*8+2]+dst_y[1]*mat1_inv_data[i*8+3]
	//	+dst_x[2]*mat1_inv_data[i*8+4]+dst_y[2]*mat1_inv_data[i*8+5]
	//	+dst_x[3]*mat1_inv_data[i*8+6]+dst_y[3]*mat1_inv_data[i*8+7];
	//}
	//FCW_mat[8]=1;

	//下位机


	//float mat1_data[64];
	//float mat1_inv_data[64];
	//float temp[64];

	float A[8][8];
	float b[8];
	float src_x[4],src_y[4],dst_x[4],dst_y[4];

	src_x[0]=float(xiaoshi.x)-0.5;//x0'
	src_y[0]=float(xiaoshi.y);
	src_x[1]=float(xiaoshi.x)+0.5;
	src_y[1]=float(xiaoshi.y);
	src_x[2]=float(p3.x);
	src_y[2]=float(p3.y);
	src_x[3]=float(p4.x);
	src_y[3]=float(p4.y);

	dst_x[0]=375.5-5*D+offset/10;//9.17
	dst_y[0]=33000;
	dst_x[1]=375.5+5*D+offset/10;//9.17
	dst_y[1]=33000;
	dst_x[2]=375.5-5*D+offset/10;//9.17
	dst_y[2]=0;//9.23
	dst_x[3]=375.5+5*D+offset/10;//9.17
	dst_y[3]=0;//9.23

	float* mat1_data_ptr=A[0];
	memset(A[0],0,64*sizeof(float));
	for(int i=0;i<4;i++)
	{
		mat1_data_ptr[2*i*8]=src_x[i];mat1_data_ptr[2*i*8+1]=src_y[i];mat1_data_ptr[2*i*8+2]=1;
		mat1_data_ptr[2*i*8+6]=-src_x[i]*dst_x[i];mat1_data_ptr[2*i*8+7]=-src_y[i]*dst_x[i];

		mat1_data_ptr[2*i*8+11]=src_x[i];mat1_data_ptr[2*i*8+12]=src_y[i];mat1_data_ptr[2*i*8+13]=1;
		mat1_data_ptr[2*i*8+14]=-src_x[i]*dst_y[i];mat1_data_ptr[2*i*8+15]=-src_y[i]*dst_y[i];
		b[2*i]=dst_x[i];
		b[2*i+1]=dst_y[i];
	}

	int size=8;
	int i,j,k,p;     //计数器
    float Aik;     //正消过程用到的变量名
    float S;       //回代过程用到的变量名
	//int order[8]={1,2,3,4,5,6,7,8};
	float temp[8],temp_b;
	//int temp_order;

	for(k=0;k<size-1;k++)
    {
		p=k;
		while(A[p][k]==0)
		{
			p=p+1;
		}
		if(p!=k)
		{
			for(int o=0;o<8;o++)
			{
				temp[o]=A[k][o];
				A[k][o]=A[p][o];
				A[p][o]=temp[o];
			}
			//temp_order=order[k];
			//order[k]=order[p];
			//order[p]=temp_order;

			temp_b=b[k];
			b[k]=b[p];
			b[p]=temp_b;
		}
        for(i=k+1;i<size;i++)
        {
			Aik=A[i][k]/A[k][k];
			for(j=k;j<size;j++)
			{
				A[i][j]=A[i][j]-Aik*A[k][j];
			}
			b[i]=b[i]-Aik*b[k];
        }
    }

    X[size-1]=b[size-1]/A[size-1][size-1];
    for(k=size-2;k>=0;k--)
    {
        S=b[k];
        for(j=k+1;j<size;j++)
        {
            S=S-A[k][j]*X[j];
        }
        X[k]=S/A[k][k];
    }    //回代

	X[8]=1;
}

//void HoughLines_accum_IPM(uchar* edge_image, int* accum,
//						 short height,short width,int y_start)//8,2,1,0-30,150-180
//{
//	int r,count;
//	int num;
//	float i,j;
//	Point2s pt;
//	const float* ttab= &trigtab[0];
//	uchar* data_line=edge_image;
//	
//	memset(accum,0,sizeof(int)*numangle*numrho);//改
//
//	//if(y_start==0)
//	//{
//	//	memset(show_lines_1->imageData,0,numrho*numangle);
//	//}//******************显示测试，下位机无视此句
//
//	for (pt.y = 0, count = 0; pt.y < height; pt.y++)
//	{
//		for (pt.x = 3; pt.x < width-3; pt.x++)
//		{
//			if(count>edge_points_count)
//				continue;//防止溢出
//			if (data_line[pt.x])
//			{
//				edge_points[count]=pt;
//				count++;
//			}
//		}
//		data_line+= width;
//	}
//
//	int idx =0;
//	Point2s *point = edge_points;
//	int* adata = NULL;
//	
// 	for (; count > 0; count--)
//	{
//		idx = count-1;
//		point = &(edge_points[idx]);
//		adata = accum;
//		i =(point->y)+y_start;//9.22
//		j =(point->x);
//
// 		for (int n= 0; n < numangle; n++)//
//		{
//			r = (int)(j * ttab[n * 2 + 1] + i *ttab[n * 2] +0.5);
//			//int num2=n*show_lines_1->widthStep+r;
//			if(r>IPM_WIDTH-1 || r<0)//2.19
//				continue;
//			else
//			{
//				num=n*numrho+r;
//				accum[num]+=1;
//			}
//		}
//	}
//
//		//cvShowImage("accum",show_lines_1);
//		//max_n=max_val=ttt=0;
//
//		//cvResize(show_lines_1,show_lines_2);
////		cvShowImage("accum",show_lines_2);
//		//CvPoint peak_out[50];
//		//cvMinMaxLoc(show_lines_1,0,0,0,peak_out);
////		cvWaitKey(5000);
//		/*int test=accum[69];
//		int test2=show_lines_1->imageData[69];*/
//
//					//show_lines_1->imageData[num2]+=10;
//				//if(show_lines_1->imageData[num2]>max_val)
//				//{	
//				//	max_val=show_lines_1->imageData[num2];
//				//	max_n=n;
//				//	ttt=r;
//				//}
//
//	//cv::HoughLinesP(threshold_hat,lines,1,CV_PI / 180,10,7,52);	
//		//show_lines_1.data=(uchar*)accum;
//}

void HoughLines_accum_IPM_gredient(uchar* edge_image, int* accum,
						 short height,short width,int y_start,int valid_count,Point2s* edge_points)//8,2,1,0-30,150-180
{
	int r;
	int num;
	int i,j;
	Point2s pt;
	const float* ttab= &trigtab[0];
	uchar* data_line=edge_image;
	
	memset(accum,0,sizeof(int)*numangle*numrho);//改

	//if(y_start==0)
	//{
	//	memset(show_lines_1->imageData,0,numrho*numangle);
	//}//******************显示测试，下位机无视此句

	//for (pt.y = 0, count = 0; pt.y < height; pt.y++)
	//{
	//	for (pt.x = 3; pt.x < width-3; pt.x++)
	//	{
	//		if(count>edge_points_count)
	//			continue;//防止溢出
	//		if (data_line[pt.x]>3)
	//		{
	//			edge_points[count]=pt;
	//			count++;
	//		}
	//	}
	//	data_line+= width;
	//}

	int idx =0;
	Point2s *point = edge_points;
	int* adata = NULL;
	
	float limit=float(IPM_HEIGHT-calib_plus)/calib_multiple;
	//int max=0;
 	for (; valid_count > 0; valid_count--)
	{
		idx = valid_count-1;
		point = &(edge_points[idx]);
		adata = accum;
		i =(point->y)+y_start;//9.22
		j =(point->x);

		//if(i-y_start<limit)
		//	continue;
		int temp=edge_image[(i-y_start)*IPM_WIDTH+j];

		float para2=1;
		if(temp>128)
		{
			para2=temp/255.0;
			para2=1-0.5*para2*para2;
		}

		//int kk;
		//if(i-y_start<28)
		//	kk=1;
		//else if(i-y_start<56)
		//	kk=3;
		//else
		//	kk=9;

		//if(temp>max)
		//	max=temp;

		//float para1=1;
		float para1=1;
		if(i-y_start<28)
			para1=1;
		else
			para1=0.6*(IPM_HEIGHT-float(i))/IPM_HEIGHT+0.1;

		//temp=pow(temp,0.9)/kk;
		temp=temp*para2*para1;

 		for (int n= 0; n < numangle; n++)//
		{

			//float test_fl,test_i,test;
			//int r1,r2;
			//if(n==35)
			//{
			//	float tmp_fl=0;
			//	int tmp_i=0;

			//	test_fl=float(j) * ttab[n * 2 + 1] + float(i) *tmp_fl;
			////	test_i=float(j) * ttab[n * 2 + 1] + float(i) *tmp_i;
			////	test_i=float(j) * 0.5;
			//	test_i=86.0 * 0.5;

			//	r1=int(test_fl);
			//	r2=int(test_i);


			//}
			//else
			//{
			r=float(j) * ttab[n * 2 + 1] + float(i) *ttab[n * 2];

			int num2=n*show_lines_1->widthStep+r;
			if(r>numrho-1 || r<0)//2.19
				continue;
			else
			{
				
				num=n*numrho+r;
				accum[num]+=temp;
			}
		}
	}

	//cout<<"max:  "<<max<<endl;
}

void HoughLines_accum_IPM_gredient_near_car(uchar* edge_image, int* accum,
						 short height,short width,int y_start,int valid_count,Point2s* edge_points)//8,2,1,0-30,150-180
{
	int r;
	int num;
	int i,j;
	Point2s pt;
	const float* ttab= &trigtab[0];
	uchar* data_line=edge_image;
	
	//memset(accum,0,sizeof(int)*numangle*numrho);//改

	int idx =0;
	Point2s *point = edge_points;
	int* adata = NULL;
	
 	for (; valid_count > 0; valid_count--)
	{
		idx = valid_count-1;
		point = &(edge_points[idx]);
		adata = accum;
		i =(point->y)+y_start;//9.22
		j =(point->x);

		/*if(i>42)
			continue;*/
		int temp=edge_image[(i-y_start)*IPM_WIDTH+j];

		float para2=1;
		if(temp>128)
		{
			para2=temp/255.0;
			para2=1-0.5*para2*para2;
		}

		float para1=1;
		if(i-y_start<28)
			para1=1;
		else
			para1=0.6*(IPM_HEIGHT-float(i))/IPM_HEIGHT+0.1;

		//temp=pow(temp,0.9)/kk;
		temp=temp*para2*para1;

 		for (int n= 0; n < numangle; n++)//
		{
			r=float(j) * ttab[n * 2 + 1] + float(i-calib_plus)/calib_multiple *ttab[n * 2];

			//int num2=n*show_lines_1->widthStep+r;
			if(r>numrho-1 || r<0)//2.19
				continue;
			else
			{
				num=n*numrho+r;
				accum[num]+=temp;
			}
		}
	}
	//cout<<"max:  "<<max<<endl;
}

//int HoughLinesP_xy(Image _image, Line* _lines,
//	short threshold,
//	short lineLength, short lineGap,short linesMax,float rho, float theta)
//{
////cv::Mat accum_t, mask;
////cv::vector<float> trigtab;
//int line_count=0;
//
////CvSeq* seq;
////CvSeqWriter writer;
//int width, height;
////int numangle, numrho;
//float ang;
//int r, n, count;
//Point2s pt;
//float irho = 1 / rho;
////CvRNG rng = cvRNG(-1);
//const float* ttab;
//uchar* mdata0;
//
////CV_Assert(CV_IS_MAT(image) && CV_MAT_TYPE(image->type) == CV_8UC1);
//
//width = _image.cols;
//height = _image.rows;
//
////accum_t.create(numangle, numrho, CV_32SC1);
////mask.create(height, width, CV_8UC1);
////trigtab.resize(numangle * 2);
////accum_t = cv::Scalar(0);
//int accum[int(numangle*numrho)];
//unsigned char mask[IPM_WIDTH*IPM_HEIGHT];
//memset(accum,0,sizeof(int)*numangle*numrho);//改
//
////for (ang = 0, n = 0; n < numangle; ang += theta, n++)
////{
////	trigtab[n * 2] = (float)(cos(ang) * irho);
////	trigtab[n * 2 + 1] = (float)(sin(ang) * irho);
////}
//ttab = &trigtab[0];
//mdata0 = mask;//改
//
////cvStartWriteSeq(CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), storage, &writer);起何作用？？
//
//// stage 1. collect non-zero image points
//const uchar* data = NULL;
//uchar* mdata = NULL;
//for (pt.y = 0, count = 0; pt.y < height; pt.y++)
//{
//	//const uchar* data = image->data.ptr + pt.y*image->step;
//	data = _image.imgPtr + pt.y*_image.cols;
//	mdata = mdata0 + pt.y*width;
//	for (pt.x = 0; pt.x < width; pt.x++)
//	{
//		if(count>1240)
//			continue;//防止溢出
//		if (data[pt.x])
//		{
//			mdata[pt.x] = (uchar)1;
//			//CV_WRITE_SEQ_ELEM(pt, writer);
//			edge_points[count]=pt;
//			count++;
//		}
//		else
//			mdata[pt.x] = 0;
//	}
//}//12.7优化。横的点，划定roi范围
////70%,80%,50%
//
////seq = cvEndWriteSeq(&writer);
////count = seq->total;
//
//	int idx =0;
//	int max_val = 0;
//	int max_n = 0;
//	Point2s *point = &(edge_points[idx]);
//	Point line_end[2] = { { 0, 0 }, { 0, 0 } };
//	float a, b;
//	int* adata = NULL;
//	int i, j, k, x0, y0, dx0, dy0, xflag;
//	int good_line;
//	const int shift = 16;
//// stage 2. process all the points in random order
//	for (; count > 0; count--)
//	{
//		// choose random point out of the remaining ones
//		idx = (rand())%count;
//		max_val = threshold - 1;
//		max_n = 0;
//		point = &(edge_points[idx]);
//		line_end[0].x=0;
//		line_end[0].y=0;
//		line_end[1].x=0;
//		line_end[1].y=0;
//		adata = &accum[0];
//		i =(int)(point->y);
//		j =(int)(point->x);
//
//		// "remove" it by overriding it with the last element
//		*point = edge_points[count-1];
//
//		// check if it has been excluded already (i.e. belongs to some other line)
//		if (!mdata0[i*width + j])
//			continue;
//
//		// update accum_tulator, find the most probable line
//
//		for (n = 0; n < low_angle; n++)//
//		{
//			int val = 0;
//			r = (int)(j * ttab[n * 2] + i * ttab[n * 2 + 1]+0.5);
//			r += ROI_WIDTH;
//			val = ++adata[r];
//
//			if (max_val < val)
//			{
//				max_val = val;
//				max_n = n;
//			}
//			adata += (int)(numrho);
//		}//改：1，角度限制，长度，积分空间改小 80%。。2. 角度分辨率降低2，50%。 40%，
//	//	int test_noun;
//		for (n = high_angle; n < numangle; n++)//
//		{
//			int val = 0;
//			r = (int)(j * ttab[n * 2] + i * ttab[n * 2 + 1]+0.5);
//			r += ROI_WIDTH;
//			val = ++adata[r];
//			if (max_val < val)
//			{
//				max_val = val;
//				max_n = n;
//			}
////			test_noun = (int)(numrho);
//			adata += (int)(numrho);
//		}
//
//		// if it is too "weak" candidate, continue with another point
//		if (max_val < threshold)
//			continue;
//
//		// from the current point walk in each direction
//		// along the found line and extract the line segment
//		a = -ttab[max_n * 2 + 1];
//		b = ttab[max_n * 2];
//		x0 = j;
//		y0 = i;
//		if (fabs(a) > fabs(b))
//		{
//			xflag = 1;
//			dx0 = a > 0 ? 1 : -1;
//			dy0 = (int)(b*(1 << shift) / fabs(a)+0.5);
//			y0 = (y0 << shift) + (1 << (shift - 1));
//		}
//		else
//		{
//			xflag = 0;
//			dy0 = b > 0 ? 1 : -1;
//			dx0 = (int)(a*(1 << shift) / fabs(b)+0.5);
//			x0 = (x0 << shift) + (1 << (shift - 1));
//		}
//
//		for (k = 0; k < 2; k++)
//		{
//
//			int gap = 0, x = x0, y = y0, dx = dx0, dy = dy0;
//			if (k > 0)
//				dx = -dx, dy = -dy;
//
//			// walk along the line using fixed-point arithmetics,
//			// stop at the image border or in case of too big gap
//
//			for (;; x += dx, y += dy)
//			{
//				uchar* mdata;
//
//				int i1, j1;
//				if (xflag)
//				{
//					j1 = x;
//					i1 = y >> shift;
//				}
//				else
//				{
//					j1 = x >> shift;
//					i1 = y;
//				}
//
//				if (j1 < 0 || j1 >= width || i1 < 0 || i1 >= height)
//					break;
//
//				mdata = mdata0 + i1*width + j1;
//
//				// for each non-zero point:
//				//    update line end,
//				//    clear the mask element
//				//    reset the gap
//				if (*mdata)
//				{
//					gap = 0;
//					line_end[k].y = i1;
//					line_end[k].x = j1;
//				}
//				else if (++gap > lineGap)
//					break;
//			}
//		}
//
//		good_line = abs(line_end[1].x - line_end[0].x) >= lineLength ||
//			abs(line_end[1].y - line_end[0].y) >= lineLength;
//
//		for (k = 0; k < 2; k++)
//		{
//
//			int x = x0, y = y0, dx = dx0, dy = dy0;
//			if (k > 0)
//				dx = -dx, dy = -dy;
//
//			// walk along the line using fixed-point arithmetics,
//			// stop at the image border or in case of too big gap
//			int i1, j1;
//			for (;; x += dx, y += dy)
//			{
//				uchar* mdata;
//
//
//				if (xflag)
//				{
//					j1 = x;
//					i1 = y >> shift;
//				}
//				else
//				{
//					j1 = x >> shift;
//					i1 = y;
//				}
//
//				mdata = mdata0 + i1*width + j1;
//
//				// for each non-zero point:
//				//    update line end,
//				//    clear the mask element
//				//    reset the gap
//				if (*mdata)
//				{
//					if (good_line)
//					{
//						adata = &accum[0];
//						for (n = 0; n < numangle; n++, adata += (int)(numrho))
//						{
//							r = (int)(j1 * ttab[n * 2] + i1 * ttab[n * 2 + 1]+0.5);
//							r += (numrho - 1) / 2;
//							adata[r]--;
//						}
//					}
//					*mdata = 0;
//				}
//
//				if (i1 == line_end[k].y && j1 == line_end[k].x)
//					break;
//			}
//		}
//
//		//if (max_n<high_angle && max_n>low_angle)
//		//		continue;
//
//		if (good_line)
//		{
//				_lines[line_count].x1=line_end[0].x;
//				_lines[line_count].y1=line_end[0].y;
//				_lines[line_count].x2=line_end[1].x;
//				_lines[line_count].y2=line_end[1].y;
//				line_count++;
//		}
//	}
//	return line_count;
//}

void resize_and_threshold(uchar* dataSrc,int src_W,int src_H,uchar* dataDst,int dst_W,int dst_H,int threshold)
{
		double scale_x = (double)src_W / dst_W;  
	double scale_y = (double)src_H / dst_H;

	//uchar* dataDst = matDst1.imgPtr;
	int stepDst = dst_W;
	//uchar* dataSrc = matSrc.imgPtr;
	int stepSrc = src_W;
	int iWidthSrc = src_W;
	int iHiehgtSrc = src_H;

	for (int j = 0; j < dst_W; ++j)
	{
		float fy = (float)((j + 0.5) * scale_y - 0.5);
		int sy = cvFloor(fy);
		fy -= sy;
		sy = MIN(sy, iHiehgtSrc - 2);
		sy = MAX(0, sy);

		short cbufy[2];
		cbufy[0] = cv::saturate_cast<short>((1.f - fy) * 2048);
		cbufy[1] = 2048 - cbufy[0];

		for (int i = 0; i < dst_W; ++i)
		{
			float fx = (float)((i + 0.5) * scale_x - 0.5);
			int sx = cvFloor(fx);
			fx -= sx;

			if (sx < 0) {
				fx = 0, sx = 0;
			}
			if (sx >= iWidthSrc - 1) {
				fx = 0, sx = iWidthSrc - 2;
			}

			short cbufx[2];
			cbufx[0] = cv::saturate_cast<short>((1.f - fx) * 2048);
			cbufx[1] = 2048 - cbufx[0];

			if((*(dataDst + i) = (*(dataSrc + sy*stepSrc + sx) * cbufx[0] * cbufy[0] + 
				*(dataSrc + (sy+1)*stepSrc + sx) * cbufx[0] * cbufy[1] + 
				*(dataSrc + sy*stepSrc + (sx+1)) * cbufx[1] * cbufy[0] + 
				*(dataSrc + (sy+1)*stepSrc + (sx+1)) * cbufx[1] * cbufy[1]) >> 22)>threshold)
					*(dataDst + i)=255;
			else
				*(dataDst + i)=0;
		}
		dataDst+=stepDst;
	}
}

//int HoughLinesP_xy(Image _image, Line* _lines,
//	short threshold,
//	short lineLength, short lineGap,short linesMax,float rho, float theta)
//{
////cv::Mat accum, mask;
////cv::vector<float> trigtab;
//int line_count=0;
//cv::MemStorage storage(cvCreateMemStorage(0));
//
////CvSeq* seq;
////CvSeqWriter writer;
//int width, height;
////int numangle, numrho;
//float ang;
//int r, n, count;
//Point2s pt;
//float irho = 1 / rho;
//CvRNG rng = cvRNG(-1);
//const float* ttab;
//uchar* mdata0;
//
////CV_Assert(CV_IS_MAT(image) && CV_MAT_TYPE(image->type) == CV_8UC1);
//
//width = _image.cols;
//height = _image.rows;
//
////accum.create(numangle, numrho, CV_32SC1);
////mask.create(height, width, CV_8UC1);
////trigtab.resize(numangle * 2);
////accum = cv::Scalar(0);
//std::memset(accum,0,sizeof(int)*numangle*numrho);//改
//
////for (ang = 0, n = 0; n < numangle; ang += theta, n++)
////{
////	trigtab[n * 2] = (float)(cos(ang) * irho);
////	trigtab[n * 2 + 1] = (float)(sin(ang) * irho);
////}
//ttab = &trigtab[0];
//mdata0 = mask;//改
//
////cvStartWriteSeq(CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), storage, &writer);起何作用？？
//
//// stage 1. collect non-zero image points
//
//for (pt.y = 0, count = 0; pt.y < height; pt.y++)
//{
//	//const uchar* data = image->data.ptr + pt.y*image->step;
//	const uchar* data = _image.imgPtr + pt.y*_image.cols;
//	uchar* mdata = mdata0 + pt.y*width;
//	for (pt.x = 0; pt.x < width; pt.x++)
//	{
//		if(count>1240)
//			continue;//防止溢出
//		if (data[pt.x])
//		{
//			mdata[pt.x] = (uchar)1;
//			//CV_WRITE_SEQ_ELEM(pt, writer);
//			edge_points[count]=pt;
//			count++;
//		}
//		else
//			mdata[pt.x] = 0;
//	}
//}//12.7优化。横的点，划定roi范围
////70%,80%,50%
//
////seq = cvEndWriteSeq(&writer);
////count = seq->total;
//
//// stage 2. process all the points in random order
//for (; count > 0; count--)
//{
//	// choose random point out of the remaining ones
//	int idx = std::rand()% count;
//	int max_val = threshold - 1, max_n = 0;
//	Point2s *point = &(edge_points[idx]);
//	Point line_end[2] = { { 0, 0 }, { 0, 0 } };
//	float a, b;
//	int* adata = &accum[0];
//	int i, j, k, x0, y0, dx0, dy0, xflag;
//	int good_line;
//	const int shift = 16;
//
//	i =int(point->y);
//	j =int(point->x);
//
//	// "remove" it by overriding it with the last element
//	*point = edge_points[count-1];
//
//	// check if it has been excluded already (i.e. belongs to some other line)
//	if (!mdata0[i*width + j])
//		continue;
//
//	// update accumulator, find the most probable line
//	for (n = 0; n < low_angle; n++, adata += int(numrho))//
//	{
//		r = int(j * ttab[n * 2] + i * ttab[n * 2 + 1]+0.5);
//		r += ROI_WIDTH;
//		int val = ++adata[r];
//
//		if (max_val < val)
//		{
//			max_val = val;
//			max_n = n;
//		}
//	}//改：1，角度限制，长度，积分空间改小 80%。。2. 角度分辨率降低2，50%。 40%， 
//
//	for (n = high_angle; n < numangle; n++, adata += int(numrho))//
//	{
//		r = int(j * ttab[n * 2] + i * ttab[n * 2 + 1]+0.5);
//		r += ROI_WIDTH;
//		int val = ++adata[r];
//		if (max_val < val)
//		{
//			max_val = val;
//			max_n = n;
//		}
//	}
//
//	// if it is too "weak" candidate, continue with another point
//	if (max_val < threshold)
//		continue;
//
//	// from the current point walk in each direction
//	// along the found line and extract the line segment
//	a = -ttab[max_n * 2 + 1];
//	b = ttab[max_n * 2];
//	x0 = j;
//	y0 = i;
//	if (fabs(a) > fabs(b))
//	{
//		xflag = 1;
//		dx0 = a > 0 ? 1 : -1;
//		dy0 = int(b*(1 << shift) / fabs(a)+0.5);
//		y0 = (y0 << shift) + (1 << (shift - 1));
//	}
//	else
//	{
//		xflag = 0;
//		dy0 = b > 0 ? 1 : -1;
//		dx0 = int(a*(1 << shift) / fabs(b)+0.5);
//		x0 = (x0 << shift) + (1 << (shift - 1));
//	}
//
//	for (k = 0; k < 2; k++)
//	{
//		int gap = 0, x = x0, y = y0, dx = dx0, dy = dy0;
//
//		if (k > 0)
//			dx = -dx, dy = -dy;
//
//		// walk along the line using fixed-point arithmetics,
//		// stop at the image border or in case of too big gap
//		for (;; x += dx, y += dy)
//		{
//			uchar* mdata;
//			int i1, j1;
//
//			if (xflag)
//			{
//				j1 = x;
//				i1 = y >> shift;
//			}
//			else
//			{
//				j1 = x >> shift;
//				i1 = y;
//			}
//
//			if (j1 < 0 || j1 >= width || i1 < 0 || i1 >= height)
//				break;
//
//			mdata = mdata0 + i1*width + j1;
//
//			// for each non-zero point:
//			//    update line end,
//			//    clear the mask element
//			//    reset the gap
//			if (*mdata)
//			{
//				gap = 0;
//				line_end[k].y = i1;
//				line_end[k].x = j1;
//			}
//			else if (++gap > lineGap)
//				break;
//		}
//	}
//
//	good_line = abs(line_end[1].x - line_end[0].x) >= lineLength ||
//		abs(line_end[1].y - line_end[0].y) >= lineLength;
//
//	for (k = 0; k < 2; k++)
//	{
//		int x = x0, y = y0, dx = dx0, dy = dy0;
//
//		if (k > 0)
//			dx = -dx, dy = -dy;
//
//		// walk along the line using fixed-point arithmetics,
//		// stop at the image border or in case of too big gap
//		for (;; x += dx, y += dy)
//		{
//			uchar* mdata;
//			int i1, j1;
//
//			if (xflag)
//			{
//				j1 = x;
//				i1 = y >> shift;
//			}
//			else
//			{
//				j1 = x >> shift;
//				i1 = y;
//			}
//
//			mdata = mdata0 + i1*width + j1;
//
//			// for each non-zero point:
//			//    update line end,
//			//    clear the mask element
//			//    reset the gap
//			if (*mdata)
//			{
//				if (good_line)
//				{
//					adata = &accum[0];
//					for (n = 0; n < numangle; n++, adata += int(numrho))
//					{
//						r = int(j1 * ttab[n * 2] + i1 * ttab[n * 2 + 1]+0.5);
//						r += (numrho - 1) / 2;
//						adata[r]--;
//					}
//				}
//				*mdata = 0;
//			}
//
//			if (i1 == line_end[k].y && j1 == line_end[k].x)
//				break;
//		}
//	}
//
//	//if (max_n<high_angle && max_n>low_angle)
//	//		continue;
//
//	if (good_line)
//	{
//			_lines[line_count].x1=line_end[0].x;
//			_lines[line_count].y1=line_end[0].y;
//			_lines[line_count].x2=line_end[1].x;
//			_lines[line_count].y2=line_end[1].y;
//			line_count++;
//	}
//	}
//	return line_count;
//}

//void Canny_xy(Image src,Image dst ,
//	int low_thresh, int high_thresh,
//	int aperture_size)
//{
//	unsigned char* src_data=src.imgPtr;
//	unsigned char* _dst=dst.imgPtr;
//	memset(dx_data,0,int(float(ROI_HEIGHT*ROI_WIDTH))*sizeof(short));
//	memset(dy_data,0,int(float(ROI_HEIGHT*ROI_WIDTH))*sizeof(short));
//
//	const int x_sobel[] = { -1, 0, 1, -2, 0, 2, -1, 0, 1 };
//	const int y_sobel[] = { -1, -2, -1, 0, 0, 0, 1, 2, 1 };
//	const int near_image[] = { -1-ROI_WIDTH, -ROI_WIDTH, 1-ROI_WIDTH, -1, 0, 1, ROI_WIDTH-1, ROI_WIDTH, ROI_WIDTH+1 };//188要改
//
//	for (int i = 0; i<ROI_HEIGHT; i++)
//	for (int j = 0; j<ROI_WIDTH; j++)
//	{
//		int count = i*ROI_WIDTH + j;
//
//		if (i != 0 && i != ROI_HEIGHT && j != 0 && j != ROI_WIDTH)
//		{
//			for (int ii = 0; ii<9; ii++)
//			{
//				int add = near_image[ii];
//				if (x_sobel[ii] != 0)
//					dx_data[count] += short(x_sobel[ii] * (int)(src_data[count + add]));
//				if (y_sobel[ii] != 0)
//					dy_data[count] += short(y_sobel[ii] * (int)(src_data[count + add]));
//			}
//		}
//	}
//	//adi_sobel_3x3_vert_i8(src_data,66,188,dy_data);
//	//adi_sobel_3x3_horz_i8(src_data,66,188,dx_data);
//	//int low = cvFloor(low_thresh);
//	//int high = cvFloor(high_thresh);
//
//	short mapstep = ROI_WIDTH + 2;
//	unsigned char buffer[int((ROI_WIDTH + 2)*(ROI_HEIGHT + 2) + (ROI_WIDTH + 2) * 3 * sizeof(int))];//188
//
//	int* mag_buf[3];
//	mag_buf[0] = (int*)(unsigned char*)buffer;
//	mag_buf[1] = mag_buf[0] + mapstep;//*1;//12.8优化，去掉1
//	mag_buf[2] = mag_buf[1] + mapstep;//*1;
//	memset(mag_buf[0], 0, /* cn* */mapstep*sizeof(int));
//
//	unsigned char* map = (unsigned char*)(mag_buf[2] + mapstep);//12.8优化，去掉1
//	memset(map, 1, mapstep);
//	memset(map + mapstep*int(ROI_HEIGHT + 1), 1, mapstep);
//
//	int maxsize = 2480;
//	unsigned char* stack[2480];
//	unsigned char **stack_top = &stack[0];
//	unsigned char **stack_bottom = &stack[0];
//
//	/* sector numbers
//	(Top-Left Origin)
//
//	1   2   3
//	*  *  *
//	* * *
//	0*******0
//	* * *
//	*  *  *
//	3   2   1
//	*/
//
////#define CANNY_PUSH(d)    *(d) = 2, *stack_top++ = (d)
////#define CANNY_POP(d)     (d) = *--stack_top
//
//	// calculate magnitude and angle of gradient, perfordinm non-maxima suppression.
//	// fill the map with one of the following values:
//	//   0 - the pixel might belong to an edge
//	//   1 - the pixel can not belong to an edge
//	//   2 - the pixel does belong to an edge 
//	short* _dx = dx_data;
//	short* _dy = dy_data;
//	short* _x = dx_data;
//	short* _y = dy_data;
//	int start_j=0;
//	int end_j=ROI_WIDTH;
//	for (int i = 0; i <= ROI_HEIGHT; i++)
//	{
//		int* _norm = mag_buf[(i > 0) + 1] + 1;
//
//		//short* _dx = &dx_data[i*ROI_WIDTH];//12.08乘法优化
//		//short* _dy = &dy_data[i*ROI_WIDTH];
//		if(i<trangle_y)
//		{
//			start_j=int(trangle_x1-i*slope);
//			end_j=int(trangle_x2+i*slope);
//		}
//		else
//		{
//			start_j=0;
//			end_j=ROI_WIDTH;
//		}
//
//		for (int j = start_j; j < end_j; j++)
//		{
//			int temp_x=int(_dx[j]);
//			int temp_y=int(_dy[j]);
//
//			if(temp_x<0)
//				temp_x=-temp_x;
//			if(temp_y<0)
//				temp_y=-temp_y;//求绝对值
//
//			_norm[j] = temp_x + temp_y;//ABS宏 不对！经过验证，有问题！！！
//		}
//		_norm[-1] = _norm[ROI_WIDTH] = 0;//12.8减少判断
//
//		if (i == ROI_HEIGHT)
//			memset(_norm - 1, 0, /* cn* */mapstep*sizeof(int));
//
//		// at the very beginning we do not have a complete ring
//		// buffer of 3 magnitude rows for non-maxima suppression
//		if (i == 0)
//			continue;
//
//		unsigned char* _map = map + mapstep*i + 1;
//		_map[-1] = _map[ROI_WIDTH] = 1;
//
//		int* _mag = mag_buf[1] + 1; // take the central row
//		short magstep1 = mag_buf[2] - mag_buf[1];
//		short magstep2 = mag_buf[0] - mag_buf[1];
//
//		int prev_flag = 0;
//		for (int j = start_j; j < end_j; j++)
//		{
//#define CANNY_SHIFT 15
//			const int TG22 = (int)(0.4142135623730950488016887242097*(1 << CANNY_SHIFT) + 0.5);
//
//			int m = _mag[j];
//
//			if (m > low_thresh)
//			{
//				int xs = _x[j];
//				int ys = _y[j];
//				int x = abs(xs);
//				int y = abs(ys) << CANNY_SHIFT;
//
//				int tg22x = x * TG22;
//
//				if (y < tg22x)
//				{
//					if (m > _mag[j - 1] && m >= _mag[j + 1]) goto __ocv_canny_push;
//				}
//				else
//				{
//					int tg67x = tg22x + (x << (CANNY_SHIFT + 1));
//					if (y > tg67x)
//					{
//						if (m > _mag[j + magstep2] && m >= _mag[j + magstep1]) goto __ocv_canny_push;
//					}
//					else
//					{
//						int s = (xs ^ ys) < 0 ? -1 : 1;
//						if (m > _mag[j + magstep2 - s] && m > _mag[j + magstep1 + s]) goto __ocv_canny_push;
//					}
//				}
//			}
//			prev_flag = 0;
//			_map[j] = 1;//xy-
//			continue;
//		__ocv_canny_push:
//			if (!prev_flag && m > high_thresh && _map[j - mapstep] != 2)
//			{
//				//CANNY_PUSH(_map + j);//xy
//				*(_map + j) = 2;
//				*stack_top++ = (_map + j);
//				prev_flag = 1;
//			}
//			else
//				_map[j] = 0;
//		}
//
//		// scroll the ring buffer
//		_mag = mag_buf[0];
//		mag_buf[0] = mag_buf[1];
//		mag_buf[1] = mag_buf[2];
//		mag_buf[2] = _mag;
//
//		_x += ROI_WIDTH;
//		_y += ROI_WIDTH;
//		_dx += ROI_WIDTH;
//		_dy += ROI_WIDTH;
//	}
//	// now track the edges (hysteresis thresholding)
//	while (stack_top > stack_bottom)
//	{
//		unsigned char* m;
//		//if ((stack_top - stack_bottom) + 8 > maxsize)
//		//	return;
//
//		//CANNY_POP(m);//xy
//		(m) = *--stack_top;
//
//		if (!m[-1])         
//		{
//			*(m - 1) = 2;
//			*stack_top++ = (m - 1);
//		}//CANNY_PUSH(m - 1);//xy
//		if (!m[1])         // CANNY_PUSH(m + 1);
//		{
//			*(m + 1) = 2;
//			*stack_top++ = (m + 1);
//		}
//		if (!m[-mapstep - 1]) //CANNY_PUSH(m - mapstep - 1);
//		{
//			*(m-mapstep - 1) = 2;
//			*stack_top++ = (m-mapstep - 1);
//		}
//		if (!m[-mapstep])   //CANNY_PUSH(m - mapstep);
//		{
//			*(m - mapstep) = 2;
//			*stack_top++ = (m - mapstep);
//		}
//		if (!m[-mapstep + 1])// CANNY_PUSH(m - mapstep + 1)
//		{
//			*(m - mapstep + 1) = 2;
//			*stack_top++ = (m - mapstep + 1);
//		}
//		if (!m[mapstep - 1])  //CANNY_PUSH(m + mapstep - 1);
//		{
//			*(m + mapstep - 1) = 2;
//			*stack_top++ = (m + mapstep - 1);
//		}
//		if (!m[mapstep])    //CANNY_PUSH(m + mapstep)
//		{
//			*(m + mapstep) = 2;
//			*stack_top++ = (m + mapstep);
//		}
//		if (!m[mapstep + 1])  //CANNY_PUSH(m + mapstep + 1);
//		{
//			*(m + mapstep + 1) = 2;
//			*stack_top++ = (m + mapstep + 1);
//		}
//	}
//
//	// the final pass, form the final image
//	const unsigned char* pmap = map + mapstep + 1;
//	unsigned char* pdst =_dst;
//	for (int i = 0; i < ROI_HEIGHT; i++, pmap += mapstep, pdst += ROI_WIDTH)
//	{
//		if(i<trangle_y)
//		{
//			start_j=int(trangle_x1-i*slope);
//			end_j=int(trangle_x2+i*slope);
//		}
//		else
//		{
//			start_j=0;
//			end_j=ROI_WIDTH;
//		}
//		for (int j = 0; j < start_j+1; j++)
//			pdst[j]=0;
//		for (int j = start_j+1; j < end_j-1; j++)
//			pdst[j] = (unsigned char)(-(pmap[j] >> 1));
//		for (int j = end_j-1; j < ROI_WIDTH; j++)
//			pdst[j]=0;
//	}
//}

int cal_error(Line* lines,int count)
{
	Line temp;
	int count_2=0;
	for (int i = 0; i < count; i++)
	{
		int x1, x2, y1, y2,error;
		x1 = lines[i].x1 + (float(lines[i].x1 - lines[i].x2) / float(lines[i].y1 - lines[i].y2))*(0 - lines[i].y1);
		x2 = lines[i].x1 + (float(lines[i].x1 - lines[i].x2) / float(lines[i].y1 - lines[i].y2))*(ROI_HEIGHT-1 - lines[i].y1);
		y1 = lines[i].y1 + (float(lines[i].y1 - lines[i].y2) / float(lines[i].x1 - lines[i].x2))*(0 - lines[i].x1);
		y2 = lines[i].y1 + (float(lines[i].y1 - lines[i].y2) / float(lines[i].x1 - lines[i].x2))*(ROI_WIDTH-1 - lines[i].x1);

		if (x1 < ROI_WIDTH*0.3 || x1 > ROI_WIDTH*0.7)
		{
			continue;
		}
		else if (x2 >= 0 && x2 <= ROI_WIDTH-1)
		{
			temp.error = x2-(0 + ROI_WIDTH-1) / 2;//error以x轴正向为正。
			temp.x1 = x1;
			temp.y1 = 0;
			temp.x2 = x2;
			temp.y2 = ROI_HEIGHT - 1;
		}
		else if (y1 >= 0 && y1 <= ROI_HEIGHT - 1)
		{
			temp.error = -(ROI_HEIGHT-1 - y1 + ROI_WIDTH / 2);
			temp.x1 = x1;
			temp.y1 = 0;
			temp.x2 = 0;
			temp.y2 = y1;
		}
		else if (y2 >= 0 && y2 <= ROI_HEIGHT - 1)
		{
			temp.error = (ROI_HEIGHT-1 - y2 +  ROI_WIDTH / 2);
			temp.x1 = x1;
			temp.y1 = 0;
			temp.x2 = ROI_WIDTH-1;
			temp.y2 = y2;
		}

		lines[count_2]=temp;
		count_2++;
	}
	return count_2;
}

int remove_by_ROI(Line* lines,int count)
{
	int count_2=0;
	Point P1,P4;

	P1.x=ROI_WIDTH * 3 / 8;
	P1.y=ROI_HEIGHT / 4;

	P4.x= ROI_WIDTH * 5 / 8;
	P4.y=ROI_HEIGHT *3 / 4;

	for (int i = 0; i < count; i++)
	{
		if ((lines[i].x1>=P1.x && lines[i].x1<=P4.x && lines[i].y1>=P1.y && lines[i].y1<=P4.y) || (lines[i].x2>=P1.x && lines[i].x2<=P4.x && lines[i].y2>=P1.y && lines[i].y2<=P4.y))
		{
			continue;
		}
		else
		{
			lines[count_2]=lines[i];
			count_2++;
		}
	}

	return count_2;
}

void rank_by_error(Line* _lines,int count)
{
	Line temp;
	bool flag=true;
	for (int i = 0; i < count; i++) 
	{
		flag = true;
		for (int j = 0; j < count - i - 1; j++) 
		{
			if (_lines[j].error > _lines[j + 1].error) 
			{
				temp = _lines[j];
				_lines[j] = _lines[j + 1];
				_lines[j + 1] = temp;
				flag = false;
			}
		}
		if (flag == true)
			break;
	}
}

int merge(Line* lines,int count)
{
	if(count<2)
		return count;
	Line temp;
	int diff;
	int count2=0;
	for (int i = 0; i < count; ) 
	{
		int add = 0;
		int threshold;
		if (lines[i].error>240/zoom_parameter || lines[i].error<240/zoom_parameter)
			threshold = 20/zoom_parameter;
		else
			threshold = 160/zoom_parameter;

		//if((i + add) < count && (i + add + 1) < count)
		//{
		//	diff=lines[i].error - lines[i+1].error;
		//	diff=diff>(-diff)?diff:(-diff);
		//}
		//else 
		//	diff=100;

		while ((i + add) < count && (i + add + 1) < count && (lines[i+add].error - lines[i+1+add].error)<threshold && (lines[i+add].error - lines[i+1+add].error>threshold*(-1)))
		{
			add++;
		}

		if (add != 0)
		{
			for (int p = 0; p < 5; p++)
			{
				for (int j = 0; j < add; j++)
				{
					//(*k)[p] = (*k)[p] + (*(k + j))[p];
					lines[i].x1+=lines[i+j].x1;
					lines[i].y1+=lines[i+j].y1;
					lines[i].x2+=lines[i+j].x2;
					lines[i].y2+=lines[i+j].y2;
					lines[i].error+=lines[i+j].error;
				}
				lines[i].x1=lines[i].x1/(add+1);
				lines[i].y1=lines[i].y1/(add+1);
				lines[i].x2=lines[i].x2/(add+1);
				lines[i].y2=lines[i].y2/(add+1);
				lines[i].error=lines[i].error/(add+1);
			}
		}

		lines[count2]=lines[i];
		i=i+add+1;
		count2++;
	}
	return count2;
}

void rank_by_abserror(Line* lines,int count)
{
	Line temp;
	bool flag=true;
	int abs1,abs2;
	int j1,j2;
	for (int i = 0; i < count; i++) 
	{
		flag = true;
		for (int j = 0; j < count - i - 1; j++) 
		{
			j1=lines[j].error;
			j2=lines[j+1].error;

			abs1=j1>(-j1)?j1:(-j1);
			abs2=j2>(-j2)?j2:(-j2);

			if (abs1>abs2) 
			{
				temp = lines[j];
				lines[j] = lines[j + 1];
				lines[j + 1] = temp;
				flag = false;
			}
		}
		if (flag == true)
			break;
	}
}

bool if_parallel(Line cur_lane_left,Line cur_lane_right)
{
	//车道线 逆矩阵
    float src2dst[9]={1.2810007251460834e-001,1.9817555978968571,
    -4.0169696206708363e+002,1.1588742625725000e-002,
    -1.4469163457415074e-001,1.2699457495683026e+002,
    1.4372564856530377e-004,5.2980024077946821e-003,
	-1.0007424129417866e+000};

	float temp;
	Line left_pers,right_pers;

	temp=src2dst[6]*(float)cur_lane_left.x1+src2dst[7]*(float)cur_lane_left.y1+src2dst[8];
	left_pers.x1=(src2dst[0]*(float)cur_lane_left.x1+src2dst[1]*(float)cur_lane_left.y1+src2dst[2])/temp;
	left_pers.y1=(src2dst[3]*(float)cur_lane_left.x1+src2dst[4]*(float)cur_lane_left.y1+src2dst[5])/temp;

	temp=src2dst[6]*(float)cur_lane_left.x2+src2dst[7]*(float)cur_lane_left.y2+src2dst[8];
	left_pers.x2=(src2dst[0]*(float)cur_lane_left.x2+src2dst[1]*(float)cur_lane_left.y2+src2dst[2])/temp;
	left_pers.y2=(src2dst[3]*(float)cur_lane_left.x2+src2dst[4]*(float)cur_lane_left.y2+src2dst[5])/temp;

	temp=src2dst[6]*(float)cur_lane_right.x1+src2dst[7]*(float)cur_lane_right.y1+src2dst[8];
	right_pers.x1=(src2dst[0]*(float)cur_lane_right.x1+src2dst[1]*(float)cur_lane_right.y1+src2dst[2])/temp;
	right_pers.y1=(src2dst[3]*(float)cur_lane_right.x1+src2dst[4]*(float)cur_lane_right.y1+src2dst[5])/temp;

	temp=src2dst[6]*(float)cur_lane_right.x2+src2dst[7]*(float)cur_lane_right.y2+src2dst[8];
	right_pers.x2=(src2dst[0]*(float)cur_lane_right.x2+src2dst[1]*(float)cur_lane_right.y2+src2dst[2])/temp;
	right_pers.y2=(src2dst[3]*(float)cur_lane_right.x2+src2dst[4]*(float)cur_lane_right.y2+src2dst[5])/temp;

	int lane_width1 = right_pers.x1 -left_pers.x1;
	int lane_width2 = right_pers.x2 -left_pers.x2;
	int e=lane_width1-lane_width2;

	if(!(lane_width2<45 && lane_width2>28))
		return false;

	if(lane_width1<-15)
        return false;

	float error_threshold=0.167/2*((left_pers.y1-left_pers.y2)+(right_pers.y1-right_pers.y2));//tan20度
	if(e>error_threshold || e<error_threshold*(-1))
		return false;

	return true;
}


int WriteRawData(unsigned char *data,char *out_path)
{
	FILE *out_file;
	out_file=fopen(out_path, "wb" );
	if(out_file==NULL)
		return 0;
	fwrite(data,1,360960,out_file);
	fclose(out_file);
	return 1;
}
