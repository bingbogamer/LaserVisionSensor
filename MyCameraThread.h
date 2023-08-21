#ifndef MYTHREAD_H
#define MYTHREAD_H

#include <QThread>
#include "MvCamera.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <vector>  
#include <string>  
#include <algorithm>  
#include <iostream>  
#include <iterator>  
#include <stdio.h>  
#include <stdlib.h>  
#include <ctype.h>  

using namespace std;
using namespace cv;

// MyThread 线程类
class MyCameraThread : public QThread
{
	Q_OBJECT

public:
	MyCameraThread();
	~MyCameraThread();

	void run()	override;						// 每30ms读取相机中的图像，并发送Display信号，img_display_label接收图像并显示


	void getCameraPtr(CMvCamera* camera) { cameraPtr = camera; }
	void getImagePtr(Mat* image) { imagePtr = image; }
	void getCameraIndex(int index) { cameraIndex = index; }
	void get_TriggerMode(int m_nTriggerMode) { TriggerMode = m_nTriggerMode; }

signals:
	void mess();		// send message 信号
	void singal_Camera_Display(const Mat* image, int index);

private:
	CMvCamera* cameraPtr;	
	cv::Mat* imagePtr;		// 输出图像缓存 
	int cameraIndex;		
	int TriggerMode;		// 触发模式 0:line0		1:line1		7:software
};

#endif // MYTHREAD_H

