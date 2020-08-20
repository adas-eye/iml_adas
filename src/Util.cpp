#include "stdio.h"
//#include "Windows.h"
#include "Util.h"
#include "Haar.h"
int LoadTrueBitmapFile(const char * FileName,unsigned char *buf, int bufsize,int &X, int &Y)
{ 
	BITMAPFILEHEADER_U  bfheader;
	BITMAPINFOHEADER_U   binfohead;
	long  temp;
	unsigned char *Data,*line;
	FILE*fp;
	RGBQUAD_U    *Color;
	int i,j;
	fp=fopen(FileName,"rb");
	if(fp==NULL) 
	{
		//AfxMessageBox("IDS_FILEOPENERR");
		return 0;
	}
	size_t tmp = fread(&bfheader,sizeof(BITMAPFILEHEADER_U),1,fp);
	if(bfheader.bfType!=0x4d42) 
	{   
		//AfxMessageBox("IDS_NOTBMP");
		return 0;
	}
	tmp = fread(&binfohead,sizeof(BITMAPINFOHEADER_U),1,fp);
	X=(short)binfohead.biWidth;
	Y=(short)binfohead.biHeight;
	

	
	if(binfohead.biBitCount!=8 && binfohead.biBitCount!=24) 
	{
		//AfxMessageBox("IDS_NOT8OR24BITBMP");
		return 0;
	}
	if(binfohead.biCompression!=0) 
	{   
		//AfxMessageBox("IDS_BMPCOMPRESSED");
		return 0;
	}
	Data=buf;
	if(bufsize<(int)3*X*Y) return 0;
	if(Data==NULL) return 0;
	
	if(binfohead.biBitCount==24) 
    {for(i=Y-1;i>=0;i--)
	{size_t tmp = fread(&(Data[i*X*3]),X,3,fp);
	if((3*X)%4!=0)fseek(fp,(4-(3*X)%4) ,SEEK_CUR );
	}
    }
	else 
	{temp=256*4;
	Color=(RGBQUAD_U *)malloc(temp);
	if (Color==NULL) return 0;
	size_t tmp = fread(Color,temp,1,fp);
	line=(unsigned char *) malloc(X);
	if(line==NULL) return 0;
	for(i=Y-1;i>=0;i--)
	{size_t tmp = fread(line,X,1,fp); ///To Convert Image to Stand
	for(j=0;j<X;j++)
	{Data[(i*X+j)*3]=Color[line[j]].rgbBlue;
	Data[(i*X+j)*3+1]=Color[line[j]].rgbGreen;
	Data[(i*X+j)*3+2]=Color[line[j]].rgbRed;
    }
	if(X%4!=0)fseek(fp,4-X%4 ,SEEK_CUR );
	}
	free(line);
	free(Color); 
	}   
	fclose(fp);
	return 1;
}



int WriteBmp24bit(const char *fn,unsigned char *pImg,int w,int h)
{
	FILE *pf;
	pf=fopen(fn,"wb");
	if(pf==NULL) return 0;
	
	int X,Y,i;
	unsigned char *pNew;
	
	// crop image to 4*n
	X=((w+3)/4)*4; Y=h;
	//X=w; Y=h;
	pNew=new unsigned char[X*Y*3];
	for(i=0;i<Y;i++) ::memcpy(pNew+i*X*3,pImg+(h-1-i)*w*3,X*3);
	
	// init bmp file header
	BITMAPFILEHEADER_U bmh;
	int nOff=sizeof(BITMAPFILEHEADER_U)+sizeof(BITMAPINFOHEADER_U);
	bmh.bfType='B'+'M'*256;
	bmh.bfOffBits=nOff;
	bmh.bfSize=nOff+X*Y*1L;
	bmh.bfReserved1=0;
	bmh.bfReserved2=0;
	
	// init bitmap info header
	BITMAPINFOHEADER_U bmiHeader;
    bmiHeader.biSize=sizeof(BITMAPINFOHEADER_U); 
    bmiHeader.biWidth=X; 
    bmiHeader.biHeight=Y;
    bmiHeader.biPlanes=1; 
    bmiHeader.biBitCount=24; 
    bmiHeader.biCompression=0; 
    bmiHeader.biSizeImage=0; 
    bmiHeader.biXPelsPerMeter=0; 
    bmiHeader.biYPelsPerMeter=0; 
    bmiHeader.biClrUsed=0;
	bmiHeader.biClrImportant=0;
	
	// init pallete
	
	// write file
	int rtn=1;
	if(1!=fwrite((void*)&bmh,sizeof(BITMAPFILEHEADER_U),1,pf)) rtn=0;
	if(1!=fwrite((void*)&bmiHeader,sizeof(BITMAPINFOHEADER_U),1,pf)) rtn=0;
	if(1!=fwrite((void*)pNew,X*Y*3,1,pf)) rtn=0;
	fclose(pf);
	
	delete[] pNew;
	
	return rtn;
}
int WriteBmp24bit_Gray(const char *fn,unsigned char *pImg,int w,int h)
{
	BYTE *buf=new BYTE[w*h*3];
	int i;
	for(i=0;i<w*h;i++)buf[i*3]=buf[i*3+1]=buf[i*3+2]=pImg[i];
	int rtn=WriteBmp24bit(fn,buf,w,h);
	delete buf;
	return rtn;
}
void CvtRGB2GRAY(BYTE * src ,BYTE* dst , int X,int Y)
{
	BYTE * pRGB ,*pGray ,*pEnd = src+X*Y*3;
	int LUTR[256],LUTG[256],LUTB[256];
	int i;
	int gray;
	//查找表
	for(i=0;i<256;i++)
	{
		LUTR[i] = (i*38);
		LUTG[i] = (i*75);
		LUTB[i] = (i*15);
	}
	//
	for(pRGB=src,pGray=dst;pRGB<pEnd;)
	{
		gray = LUTR[*(pRGB++)];
		gray+= LUTG[*(pRGB++)];
		gray+= LUTB[*(pRGB++)];
		*(pGray++)=gray>>7;
	}
}


void OutputResult(Image &img, int index)
{
	if(result_seq.total != 0)
	{
		char path[100];
		Rect rect = result_seq.rectQueue[0];
		unsigned char *outputImg = (unsigned char *)malloc(rect.height*rect.width*sizeof(unsigned char));
		int offset = rect.y * img.cols +  rect.x;
		unsigned char *p = img.imgPtr + offset;
		for (int i = 0; i < rect.height; i++)
			for (int j = 0; j < rect.width; j++)
				outputImg[i*rect.width + j] = *(p+i*img.cols+j);
			
			sprintf(path,"..\\FaceResult\\%d.bmp", index);
			WriteBmp24bit_Gray(path,outputImg,rect.width,rect.height);
	}
	
}
void WriteSumAndSqSumToFile(const Mat32 sum, const Mat64 sqsum)
{
	FILE *fp;
	FILE *fp2;
    fp=fopen("..//InteralResult//sum.txt","wb+");
	fp2=fopen("..//InteralResult//sqsum.txt","wb+");
	if(fp!=NULL)
	{
		for (int i=0; i<sum.rows; i++)
		{
			for (int j=0; j<sum.cols; j++)
			{
				fprintf(fp,"%d ",*(((int*)sum.mat32Ptr)+i*sum.cols+j));  
				long long *p = (long long*)sqsum.mat64Ptr+i*sqsum.cols+j;
				fprintf(fp2,"%lld ",*(((long long*)sqsum.mat64Ptr)+i*sqsum.cols+j));	
			}
			
			fprintf(fp,"\n");  
			fprintf(fp2,"\n");  
		}
	}
	else 
		printf("fail to open file\n");


	fclose(fp);
	fclose(fp2);
}




void WriteCascadeToFile(HidHaarClassifierCascade* hidCascade)
{
	FILE *fp;
	
    fp=fopen("..//CascadeResult//cascade.txt","wb+");
	
	if(fp!=NULL)
	{
		
		fprintf(fp, "hidCascade->count:%d\n", hidCascade->count);
		// 用cascade初始化HidCascade 
		for(int i = 0; i < hidCascade->count; i++ )
		{
			
			HidHaarStageClassifier* hid_stage_classifier = &hidCascade->stage_classifier[i];
			fprintf(fp, "Stage:%d %d %d:\n ", hid_stage_classifier->count,hid_stage_classifier->threshold,
			hid_stage_classifier->two_rects);
			
			for(int j = 0; j < hid_stage_classifier->count; j++ )
			{
			
				HidHaarClassifier* hid_classifier = hid_stage_classifier->classifier+j;
				fprintf(fp, "Classifier:%d\n ",hid_classifier->count);
				
		
				
				for(int l = 0; l < hid_classifier->count; l++ )
				{
					HidHaarTreeNode* node = hid_classifier->node+l;
					fprintf(fp, "node:%d %d %d %d \n ",hid_classifier->count, node->threshold, node->left, node->right);
		
				}
				
			}
			
        }
		
		
	}

	
	
	fclose(fp);
	
} 
void WriteMaskRowToFile(Image tmp)
{
	FILE* fp;

	fp = fopen("..//MaskRowResult//maskRow.txt","wb+");

	
	if(fp!=NULL)
	{
		
	
		for(int i = 0; i < tmp.rows; i++ )
		{
			for(int j = 0; j < tmp.cols; j++ )
				fprintf(fp, "%d  ", *(tmp.imgPtr + i*tmp.cols + j));
	
			fprintf(fp, "\n");
		
		}
	}
	
	
	
	fclose(fp);
}

//int WriteRawData(unsigned char *data,char *out_path)
//{
//	FILE *out_file;
//	out_file=fopen(out_path, "wb" );
//	if(out_file==NULL)
//		return 0;
//	fwrite(data,1,360960,out_file);
//	fclose(out_file);
//	return 1;
//}




















