#ifndef LASERVISIONSENSOR_H
#define LASERVISIONSENSOR_H

#include "ui_LaserVisionSensor.h"

#include <QtWidgets/QWidget>
#include <QMessageBox>
#include <QCloseEvent>
#include <QSettings>
#include <QDebug>
#include <QWidget>
#include <QValidator>
#include <qdatetime.h>

#include <opencv2\opencv.hpp>		
#include <fstream>
#include <iostream>
#include <ctime>
#include <sstream>

#include "MvCamera.h"
#include "MyCameraThread.h"

#include "T_Common.h"
#include "T_Lap.h"
//#include "T_SingleBevel.h"
//#include "T_Square.h"
//#include "T_V.h"

#define TRIGGER_SOURCE  7		// 触发源（默认软触发）
#define EXPOSURE_TIME   80000	// 曝光时间 us
#define FRAME           30
#define TRIGGER_ON      1		// 触发模式打开, 则 发送一次软触发，采集一次。
#define TRIGGER_OFF     0		// 触发模式关闭, 在 每帧开始，自动发送软触发？
#define CONTINUE_ON		1		// 连续采集开启
#define CONTINUE_OFF	0		
#define START_GRABBING_ON   1	// 开始取流
#define START_GRABBING_OFF  0	// 关闭取流
#define IMAGE_NAME_LEN    64
#define SAVE_PATH    "F:\\MVS_Data\\temp\\"		// 图片保存路径

using namespace std;
using namespace cv;

class LaserVisionSensor : public QWidget
{
	Q_OBJECT

public:
	LaserVisionSensor(QWidget* parent = nullptr);
	~LaserVisionSensor();

private:
	Ui::LaserVisionSensorClass ui;

public:
	CMvCamera* m_pcMyCamera[1];         // 相机指针对象 数组   MAX_DEVICE_NUM = 256

	MV_CC_DEVICE_INFO_LIST m_stDevList;	// 设备信息列表 结构体，用来存储设备列表

	cv::Mat* myImage_Camera;			// 输出图像缓存 _ 连续采集模式 + 单帧采集模式
	cv::Mat* myImage_ScreenShot;		// 抓取图像缓存

	int devices_num;
	string latestSaveName;				// 上一次保存的图像名字

	/*状态 Status*/
	bool  m_bOpenDevice;                        // 是否打开设备 | Whether to open device
	bool  m_bStartGrabbing;                     // 是否开始取流 | Whether to start grabbing
	int   m_nTriggerMode;						// 是否是 连续采集模式 | Trigger Mode
	int   m_bContinueStarted;                   // 开启过 连续采集图像
	MV_SAVE_IAMGE_TYPE   m_nSaveImageType;      // 保存图像格式 | Save Image Type

	MyCameraThread* Thread_Camera1 = NULL;		// 相机线程对象


private slots:
	void OnBnClickedEnumButton();				// 枚举设备  
	void OnBnClickedOpenButton();               // 打开设备 | Open device
	void OnBnClickedCloseButton();              // 关闭设备 | Close Devices

	// Qlable 显示图像
	void display_Camera(const Mat* imagePtr, int cameraIndex);				// 相机显示
	void display_ScreenShot();					// 抓图显示

	/* 图像采集 | Image Acquisition*/
	void OnBnClickedContinusModeRadio();        // 连续模式 | Continus Mode
	void OnBnClickedTriggerModeRadio();         // 触发模式 | Trigger Mode
	void OnBnClickedStartGrabbingButton();      // 开始采集 | Start Grabbing
	void OnBnClickedStopGrabbingButton();       // 结束采集 | Stop Grabbing

	/* 图像保存 | Image Save */
	void OnBnClickedSaveBmpButton();            // 保存bmp | Save bmp
	void OnBnClickedSaveJpgButton();            // 保存jpg | Save jpg
	void OnBnClickedSavePngButton();			// 保存Png | Save png

		/* 焊缝图像处理 */
	void OnBnClickedImageProcessButton();
	void WeldRecognizeCallback();

private:
	void OpenDevices();                    // 打开设备 | Open device
	void CloseDevices();                   // 关闭设备 | Close Device
	void SaveImage_Cv();				   // 保存图片 | Save Image
	void SaveImage_formBuffer();			


private slots:
	/* 设置、获取参数操作 */
	void SetTriggerMode(int m_nTriggerMode);    // 设置触发模式 | Set Trigger Mode
	int GetTriggerMode();

	void SetExposureTime();							// 设置曝光时间 | Set Exposure Time
	int GetExposureTime();							// 获取曝光时间 | Get Exposure Time
	//void SetGain();								// 设置增益 | Set Gain
	int GetGain();									// 获取增益 | Get Gain
	//void SetFrameRate();							// 设置帧率 | Set Frame Rate
	int GetFrameRate();								// 获取帧率 | Get Frame Rate

signals:
	void singal_Camera_Display(const Mat* image, int index);
	void singal_ScreenShot_Display(const Mat* image, int index);
};

#endif	// LASERVISIONSENSOR_H