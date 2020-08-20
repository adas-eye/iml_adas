#ifndef _OPEN_CV_
#define _OPEN_CV_


// OpenCV
#include "cv.h"
#include "highgui.h"
#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"
#include "ml.h"


#ifdef _DEBUG

#pragma comment(lib,"opencv_ml249d.lib")
#pragma comment(lib,"opencv_core249d.lib")
#pragma comment(lib,"opencv_video249d.lib")
#pragma comment(lib,"opencv_legacy249d.lib")
#pragma comment(lib,"opencv_highgui249d.lib")
#pragma comment(lib,"opencv_imgproc249d.lib")
#pragma comment(lib,"opencv_features2d249d.lib")
#pragma comment(lib,"opencv_objdetect249d.lib")

#else

#pragma comment(lib,"opencv_core249.lib")
#pragma comment(lib,"opencv_video249.lib")
#pragma comment(lib,"opencv_legacy249.lib")
#pragma comment(lib,"opencv_highgui249.lib")
#pragma comment(lib,"opencv_imgproc249.lib")
#pragma comment(lib,"opencv_features2d249.lib")
#pragma comment(lib,"opencv_ml249.lib")
#pragma comment(lib,"opencv_objdetect249.lib")

#endif


//using namespace cv;


#endif