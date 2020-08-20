#ifndef _OPEN_CV_
#define _OPEN_CV_


// OpenCV
#include "cv.h"
#include "highgui.h"
#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"
#include "ml.h"


#ifdef _DEBUG

#pragma comment(lib,"opencv_ml243d.lib")
#pragma comment(lib,"opencv_core243d.lib")
#pragma comment(lib,"opencv_video243d.lib")
#pragma comment(lib,"opencv_legacy243d.lib")
#pragma comment(lib,"opencv_highgui243d.lib")
#pragma comment(lib,"opencv_imgproc243d.lib")
#pragma comment(lib,"opencv_features2d243d.lib")
#pragma comment(lib,"opencv_objdetect243d.lib")

#else

#pragma comment(lib,"opencv_core243.lib")
#pragma comment(lib,"opencv_video243.lib")
#pragma comment(lib,"opencv_legacy243.lib")
#pragma comment(lib,"opencv_highgui243.lib")
#pragma comment(lib,"opencv_imgproc243.lib")
#pragma comment(lib,"opencv_features2d243.lib")
#pragma comment(lib,"opencv_ml243.lib")
#pragma comment(lib,"opencv_objdetect243.lib")

#endif


//using namespace cv;


#endif