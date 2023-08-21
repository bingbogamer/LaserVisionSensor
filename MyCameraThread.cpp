#include "MyCameraThread.h"

MyCameraThread::MyCameraThread()
	: cameraPtr(nullptr), imagePtr(nullptr), cameraIndex(0)
{
}

// 释放线程资源
MyCameraThread::~MyCameraThread()
{
	// 终止线程，线程可能会立即被终止也可能不会，这取决于操作系统的调度策略
	terminate();			
	if (cameraPtr != NULL) {
		delete cameraPtr;	// 释放相机资源
		cameraPtr = NULL;
	}
	if (imagePtr != NULL) {
		delete imagePtr;	// 释放图像资源
		imagePtr = NULL;
	}
}


// QThread实例（WorkThread）是属于创建该实例的线程的。比如在主线程中创建一个QThread，那么这个QThread实例本身属于主线程。
// 只有子线程 run()里面定义的变量、实例等是属于新线程的。
void MyCameraThread::run()
{
	if (cameraPtr == NULL) return;
	if (imagePtr == NULL) return;

	while (!isInterruptionRequested())
	{
		std::cout << "Thread_Trigger:" << cameraPtr->softTrigger() << std::endl;				// 相机发送软触发信号，并输出返回值
		std::cout << "Thread_Readbuffer:" << cameraPtr->ReadBuffer(*imagePtr) << std::endl;		// 读取相机中的图像   成功：返回0   失败：返回-1
		/* emit mess();*/
		emit singal_Camera_Display(imagePtr, cameraIndex);		// 发送信号，主线程接收图像imagePtr并显示
		msleep(20);		
	}
}
