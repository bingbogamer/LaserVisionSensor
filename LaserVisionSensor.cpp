#include "LaserVisionSensor.h"
// #include <thread>

// 主窗口 构造函数
LaserVisionSensor::LaserVisionSensor(QWidget* parent)
	: QWidget(parent)
	, devices_num(0)
	, m_nTriggerMode(TRIGGER_ON)			// 默认 触发模式打开
	, m_bStartGrabbing(START_GRABBING_ON)
	, m_bContinueStarted(CONTINUE_OFF)		// 连续采集 关闭
	, m_nSaveImageType(MV_Image_Bmp)
	, Thread_Camera1(new MyCameraThread)	// 相机线程对象
	, myImage_Camera(new cv::Mat())			// 显示图像对象
	, myImage_ScreenShot(new cv::Mat())		// 抓图图像对象
{
	ui.setupUi(this);

	/* 按键使能 */
	// 相机初始化控件
	ui.bntEnumDevices->setEnabled(true);
	ui.bntCloseDevices->setEnabled(false);
	ui.bntOpenDevices->setEnabled(false);
	ui.bntExposureTime->setEnabled(false);

	// 图像采集控件
	ui.rbnt_Continue_Mode->setEnabled(false);
	ui.rbnt_SoftTigger_Mode->setEnabled(false);
	ui.bntStartGrabbing->setEnabled(false);
	ui.bntStopGrabbing->setEnabled(false);
	ui.bntScreenShot->setEnabled(false);

	// 保存图像控件
	ui.bntSave_BMP->setEnabled(false);
	ui.bntSave_JPG->setEnabled(false);


	// Thread_Camera1	相机线程 在run()中发出 Display信号，被该主线程 接收，然后调用图像显示函数
	connect(Thread_Camera1, SIGNAL(singal_Camera_Display(const Mat*, int)), this, SLOT(display_Camera(const Mat*, int)));

	// 当前LaserVisionSensor对象所在线程 发出信号
	connect(this, SIGNAL(singal_Camera_Display(const Mat*, int)), this, SLOT(display_Camera(const Mat*, int)));
	// connect(this, SIGNAL(singal_ScreenShot_Display(const Mat*, int)), this, SLOT(display_ScreenShot(const Mat*, int)));

	// 抓图
	connect(ui.bntScreenShot, SIGNAL(clicked()), this, SLOT(display_ScreenShot()));

	// 图像处理，并显示
	connect(ui.btnStartProcess, SIGNAL(clicked()), this, SLOT(OnBnClickedImageProcessButton()));	// 创建新线程，做图像处理函数

	// 相机初始化
	connect(ui.bntEnumDevices, SIGNAL(clicked()), this, SLOT(OnBnClickedEnumButton()));
	connect(ui.bntOpenDevices, SIGNAL(clicked()), this, SLOT(OnBnClickedOpenButton()));
	connect(ui.bntCloseDevices, SIGNAL(clicked()), this, SLOT(OnBnClickedCloseButton()));
	connect(ui.bntExposureTime, SIGNAL(clicked()), this, SLOT(SetExposureTime()));

	// 图像采集
	connect(ui.rbnt_Continue_Mode, SIGNAL(clicked()), this, SLOT(OnBnClickedContinusModeRadio()));
	connect(ui.rbnt_SoftTigger_Mode, SIGNAL(clicked()), this, SLOT(OnBnClickedTriggerModeRadio()));
	connect(ui.bntStartGrabbing, SIGNAL(clicked()), this, SLOT(OnBnClickedStartGrabbingButton()));		// 采集开始，开始取流，启动线程
	connect(ui.bntStopGrabbing, SIGNAL(clicked()), this, SLOT(OnBnClickedStopGrabbingButton()));		// 采集停止，停止取流，回收线程

	connect(ui.bntSave_BMP, SIGNAL(clicked()), this, SLOT(OnBnClickedSaveBmpButton()));
	connect(ui.bntSave_JPG, SIGNAL(clicked()), this, SLOT(OnBnClickedSaveJpgButton()));
	connect(ui.bntSave_PNG, SIGNAL(clicked()), this, SLOT(OnBnClickedSavePngButton()));
}

LaserVisionSensor::~LaserVisionSensor()
{
	delete myImage_ScreenShot;	// 释放图像资源
}


/*************************************************** 定义槽函数 *************************************************** */
// 按下查找设备按钮:枚举 | Click Find Device button:Enumeration 
void LaserVisionSensor::OnBnClickedEnumButton()
{
	memset(&m_stDevList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));    // 初始化设备信息列表  
	int nRet = MV_OK;
	nRet = CMvCamera::EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &m_stDevList);     // 枚举子网内所有设备, 相机设备数量

	devices_num = m_stDevList.nDeviceNum;
	// unsigned char* ModelName = m_stDevList.pDeviceInfo[0]->SpecialInfo.stGigEInfo.chModelName;	// 型号名称
	if (devices_num > 0)
	{
		ui.bntOpenDevices->setEnabled(true);	// 设备数大于0，使能 打开设备按键
		QMessageBox::information(this, "Find Devices", "Find Devices Success !");
		// nDeviceNum 显示
		QString DeviceNum = QString("DeviceNum: %1").arg(devices_num);
		ui.DeviceNum->setText(DeviceNum);
	}
	else
	{
		QMessageBox::information(this, "Find Devices", "No device is detected. Check whether the device is correctly connected");	// 未发现设备，提示
	}
}


// 相机（列表）对象初始化	
// 创建句柄，即初始化 m_hDevHandle, 所有相机 设置为软触发
void LaserVisionSensor::OpenDevices()
{
	int nRet = MV_OK;
	// 创建相机指针对象 CMvCamera* m_pcMyCamera[MAX_DEVICE_NUM]
	for (unsigned int i = 0, j = 0; j < m_stDevList.nDeviceNum; j++, i++)
	{
		m_pcMyCamera[i] = new CMvCamera;
		// 相机对象初始化
		m_pcMyCamera[i]->m_pBufForDriver = NULL;			// 用于从驱动获取图像的缓存
		m_pcMyCamera[i]->m_pBufForSaveImage = NULL;			// 用于保存图像的缓存
		m_pcMyCamera[i]->m_nBufSizeForDriver = 0;			// ReadBuffer（）
		m_pcMyCamera[i]->m_nBufSizeForSaveImage = 0;		// ReadBuffer（）
		m_pcMyCamera[i]->m_nTLayerType = m_stDevList.pDeviceInfo[j]->nTLayerType;	// 赋值 每个相机的 设备传输层协议类型 

		nRet = m_pcMyCamera[i]->Open(m_stDevList.pDeviceInfo[j]);	//打开相机（创建句柄，即初始化 m_hDevHandle）
		m_pcMyCamera[i]->SetFloatValue("ExposureTime", EXPOSURE_TIME);	// 默认设置为80000us, 也就是 0.08s

		// 设置触发模式  关闭
		// 关闭触发后，自动在 每帧开始时发送一次软触发
		m_pcMyCamera[i]->setTriggerMode(TRIGGER_ON);			// 触发模式  TRIGGER_ON  1

		// 设置触发源 软触发
		m_pcMyCamera[i]->setTriggerSource(TRIGGER_SOURCE);		//0：Line0  1：Line1  7：Software
	}
}


// 点击打开相机
void LaserVisionSensor::OnBnClickedOpenButton()
{
	// 使能 "开始采集" 按键
	//ui->bntStartGrabbing->setEnabled(true);
	ui.bntOpenDevices->setEnabled(false);		// 点击 打开设备btn，触发此槽函数，并将打开设备按钮 关闭
	ui.bntEnumDevices->setEnabled(false);		// 点击 打开设备btn，触发此槽函数，并将打开设备按钮 关闭
	ui.bntCloseDevices->setEnabled(true);		// 关闭设备btn	使能 
	ui.rbnt_Continue_Mode->setEnabled(true);	// 连续采集模式 单选按钮 使能
	ui.rbnt_SoftTigger_Mode->setEnabled(true);	// 软触发模式	单选按钮 使能
	ui.bntExposureTime->setEnabled(true);

	ui.rbnt_Continue_Mode->setCheckable(true);	// 连续采集模式 单选按钮 设置可以选中
	// nDeviceNum 显示
	QString DeviceNum = QString("DeviceNum: %1").arg(devices_num);
	ui.DeviceNum->setText(DeviceNum);
	OpenDevices();								//所有 相机初始化（创建句柄，并设置为 软触发）
}



// 关闭设备 | Close Device
void LaserVisionSensor::CloseDevices()
{
	// 关闭并回收所有相机线程
	for (unsigned int i = 0; i < m_stDevList.nDeviceNum; i++)
	{
		// 关闭线程、相机
		if (Thread_Camera1->isRunning())	// 
		{
			// 发出中断请求后，MyThread::run()不再运行，图像展示窗口没有图像
			Thread_Camera1->requestInterruption();		// 该线程发出中断请求（Qthread成员函数）
			Thread_Camera1->wait();						// 线程无期限阻塞回收，直到子线程完成后退出
			m_pcMyCamera[0]->StopGrabbing();			// 停止取流
			// myThread_LeftCamera->~MyThread();		// 销毁线程
		}
		m_pcMyCamera[i]->Close();	// 销毁该相机对象的句柄
	}
	memset(&m_stDevList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));    // 初始化设备信息列表  

	// 关闭之后再枚举一遍 | Enumerate after close
	// 因为关闭之后相机文件句柄已经销毁了，如果再点击打开，需要再枚举一次设备，创建句柄
	int nRet = MV_OK;
	nRet = CMvCamera::EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &m_stDevList);     // 枚举子网内所有设备, 相机设备数量
	devices_num = m_stDevList.nDeviceNum;

	ui.DeviceNum->setText("Devices Closed");
}

// 按下关闭设备按钮：关闭设备 , 包含销毁句柄| Click Close button: Close Device
void LaserVisionSensor::OnBnClickedCloseButton()
{
	ui.bntOpenDevices->setEnabled(true);		// 打开设备btn 使能
	ui.bntCloseDevices->setEnabled(false);		// 关闭设备btn 失效
	ui.bntExposureTime->setEnabled(false);		// 关闭设备btn 失效
	// 图像采集控件
	ui.rbnt_Continue_Mode->setEnabled(false);	// 连续采集模式 单选btn 失效
	ui.rbnt_SoftTigger_Mode->setEnabled(false);	// 软触发模式  单选btn 失效
	ui.bntStartGrabbing->setEnabled(false);		// 开始连续采集 btn 失效
	ui.bntStopGrabbing->setEnabled(false);		// 关闭连续采集 btn 失效
	// 保存图像控件
	ui.bntSave_BMP->setEnabled(false);
	ui.bntSave_JPG->setEnabled(false);
	// 关闭设备，销毁线程
	CloseDevices();
}


// 采集开始
// 相机开始取流，相机线程开始运行，持续读取图像
void LaserVisionSensor::OnBnClickedStartGrabbingButton()
{
	// m_bContinueStarted = CONTINUE_ON;		// 为触发模式标记一下，切换触发模式 时先执行  停止采集图像函数

	int camera_Index = 0;

	// 连续采集模式下
	// 发送一次软触发，采集一次。开启线程，循环发送软触发
	if (m_bContinueStarted == CONTINUE_ON)
	{
		for (unsigned int i = 0; i < m_stDevList.nDeviceNum; i++)
		{
			camera_Index = i;
			if (camera_Index == 0)
			{
				m_pcMyCamera[i]->StartGrabbing();	// 自动传入该相机对象的句柄
				Thread_Camera1->getCameraPtr(m_pcMyCamera[0]);		// 线程获取相机指针
				Thread_Camera1->getImagePtr(myImage_Camera);		// 线程获取图像指针
				Thread_Camera1->getCameraIndex(0);					// 相机 Index==0

				// 如果线程没有运行（线程只是实例化，没有执行run()成员函数）
				if (!Thread_Camera1->isRunning())
				{
					// 每30ms 运行一次softTrigger()和ReadBuffer(*imagePtr), 传出图像指针myImage_Camera
					// 线程没有中断信号时，每30ms读取相机中的图像，并发送信号，img_display_label接收图像并显示
					Thread_Camera1->start();
				}
			}
			//if (camera_Index == 1)
		}
	}
	else if(m_bContinueStarted == CONTINUE_OFF)   // 单帧采集模式, 自动触发一次软触发，然后自动停止取流
	{
		// 所有相机开始取流
		for (unsigned int i = 0; i < m_stDevList.nDeviceNum; i++)
		{
			int nRet = MV_OK;

			m_pcMyCamera[i]->StartGrabbing();								// 自动传入该相机对象的句柄
			// nRet = m_pcMyCamera[i]->CommandExecute("TriggerSoftware");	// 发送一次 软触发信号
			nRet = m_pcMyCamera[i]->softTrigger();
			m_pcMyCamera[i]->ReadBuffer(*myImage_Camera);		// 保存到 myImage_Camera

			emit singal_Camera_Display(myImage_Camera, i);	// 显示在相机画面
			m_pcMyCamera[i]->StopGrabbing();					// 自动关闭取流
		}
	}
}

// 采集停止
// 停止取流，回收线程
void LaserVisionSensor::OnBnClickedStopGrabbingButton()
{
	ui.bntStartGrabbing->setEnabled(true);		// 开始连续采集按钮 使能
	ui.bntStopGrabbing->setEnabled(false);		// 停止连续采集按钮 失效

	// 连续采集模式下
	// 发送一次软触发，采集一次。开启线程，循环发送软触发
	if (m_bContinueStarted == CONTINUE_ON)
	{
		for (unsigned int i = 0; i < m_stDevList.nDeviceNum; i++)
		{
			//关闭相机
			if (Thread_Camera1->isRunning())
			{
				m_pcMyCamera[0]->StopGrabbing();			// 停止取流
				Thread_Camera1->requestInterruption();		// 线程请求中断信号
				Thread_Camera1->wait();						// 阻塞回收线程
			}
		}
	}
	else if (m_bContinueStarted == CONTINUE_OFF)   // 单帧采集模式，点击开始取流后就自动停止取流了
	{
		// 已经停止取流，无动作
	}
}

// 连续采集模式 单选按钮
void LaserVisionSensor::OnBnClickedContinusModeRadio()
{
	ui.bntStartGrabbing->setEnabled(true);	
	ui.bntStopGrabbing->setEnabled(true);	
	ui.bntScreenShot->setEnabled(true);

	m_bContinueStarted = CONTINUE_ON;
}


// 单帧采集 单选按钮 
void LaserVisionSensor::OnBnClickedTriggerModeRadio()
{
	for (unsigned int i = 0; i < m_stDevList.nDeviceNum; i++)
	{
		//关闭相机
		if (Thread_Camera1->isRunning())
		{
			m_pcMyCamera[0]->StopGrabbing();			// 停止取流
			Thread_Camera1->requestInterruption();		// 线程请求中断信号
			Thread_Camera1->wait();						// 阻塞回收线程
		}
	}
	m_bContinueStarted = CONTINUE_OFF;

	ui.bntStartGrabbing->setEnabled(true);
	ui.bntStopGrabbing->setEnabled(false);
	ui.bntScreenShot->setEnabled(true);	

	// m_nTriggerMode = TRIGGER_OFF;
	// for (unsigned int i = 0; i < m_stDevList.nDeviceNum; i++)
	//{
		//m_pcMyCamera[i]->setTriggerMode(m_nTriggerMode);	// 所有相机，设置 关闭触发模式
	//}
}

// imagePrt : RGB8格式
void LaserVisionSensor::display_Camera(const Mat* imagePrt, int cameraIndex)
{
	/* 将mat的BGR格式，转换成QT的RGB格式 */
	// cv::Mat rgb = *imagePrt;
	// cv::cvtColor(*imagePrt, rgb, CV_BGR2RGB);

	QImage QmyImage_Camera;
	if (myImage_Camera->channels() > 1)		// 3通道
	{
		QmyImage_Camera = QImage((const unsigned char*)(imagePrt->data), imagePrt->cols, imagePrt->rows, QImage::Format_RGB888);
	}
	else	// 单通道
	{
		QmyImage_Camera = QImage((const unsigned char*)(imagePrt->data), imagePrt->cols, imagePrt->rows, QImage::Format_Indexed8);
	}
	// 调整图像尺寸
	QmyImage_Camera = (QmyImage_Camera).scaled(ui.label_camera_display->size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
	// 显示在 label_camera_display 上
	ui.label_camera_display->setPixmap(QPixmap::fromImage(QmyImage_Camera));
}


// 抓图
void LaserVisionSensor::display_ScreenShot()
{
	// 保存当前图像缓存myImage_Camera 到 抓取图像缓存myImage_ScreenShot 中
	// myImage_Camera : RGB格式
	(*myImage_Camera).copyTo(*myImage_ScreenShot);  

	// 保存图像控件
	ui.bntSave_BMP->setEnabled(true);
	ui.bntSave_JPG->setEnabled(true);

	QImage QImage_ScreenShot;
	if (myImage_Camera->channels() > 1)		// 3通道
	{
		QImage_ScreenShot = QImage((const unsigned char*)(myImage_ScreenShot->data), myImage_ScreenShot->cols, myImage_ScreenShot->rows, QImage::Format_RGB888);
	}
	else	// 单通道
	{
		QImage_ScreenShot = QImage((const unsigned char*)(myImage_ScreenShot->data), myImage_ScreenShot->cols, myImage_ScreenShot->rows, QImage::Format_Indexed8);
	}

	QImage_ScreenShot = (QImage_ScreenShot).scaled(ui.label_screenshot_display->size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
	ui.label_screenshot_display->setPixmap(QPixmap::fromImage(QImage_ScreenShot));
}


// 保存bmp图片
void LaserVisionSensor::OnBnClickedSaveBmpButton()
{
	m_nSaveImageType = MV_Image_Bmp;
	SaveImage_Cv();
}

// 保存jpg图片
void LaserVisionSensor::OnBnClickedSaveJpgButton()
{
	m_nSaveImageType = MV_Image_Jpeg;
	SaveImage_Cv();
}

// 保存png图片
void LaserVisionSensor::OnBnClickedSavePngButton()
{
	m_nSaveImageType = MV_Image_Png;
	SaveImage_Cv();
}

// 保存图片 | Save Image
void LaserVisionSensor::SaveImage_Cv() 
{
	// myImage_ScreenShot：RGB格式，是否需要先转换成 BGR格式
	cv::Mat bgr;
	cv::cvtColor(*myImage_ScreenShot, bgr, CV_RGB2BGR); // Readbuffer传出的BGR8格式（用于OPENCV），但是QImage时RGB格式的，这里做转换

	time_t now = time(NULL);
	tm* t = localtime(&now);

	// 将信息输出到字符串流
	stringstream ss;
	ss << t->tm_year + 1900 - 2000 << "-" << t->tm_mon + 1 << "-" << t->tm_mday << "-" << t->tm_hour << "." << t->tm_min << "." << t->tm_sec;
	
	if (m_nSaveImageType == MV_Image_Bmp) {
		const string fullPath = SAVE_PATH + ss.str() + ".bmp";
		latestSaveName = fullPath;
		cv::imwrite(fullPath, bgr);
	}
	else if (m_nSaveImageType == MV_Image_Jpeg)
	{
		const string fullPath = SAVE_PATH + ss.str() + ".jpg";
		latestSaveName = fullPath;
		cv::imwrite(fullPath, bgr);
	}
	else if (m_nSaveImageType == MV_Image_Png)
	{
		// 压缩参数设置
		vector <int> compression_params;
		compression_params.push_back(IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(9);

		const string fullPath = SAVE_PATH + ss.str() + ".png";
		latestSaveName = fullPath;
		// cv::imwrite(fullPath.c_str(), bgr, compression_params);
		cv::imwrite(fullPath, bgr);
	}
}


// 保存图片 | Save Image`
void LaserVisionSensor::SaveImage_formBuffer()
{
	// 获取1张图 | Get one frame
	MV_FRAME_OUT_INFO_EX stImageInfo = { 0 };		// 输出帧的信息
	memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));

	unsigned int nDataLen = 0;
	int nRet = MV_OK;
	for (int i = 0; i < devices_num; i++)
	{
		// 仅在第一次保存图像时申请缓存，在 CloseDevice 时释放
		if (NULL == m_pcMyCamera[i]->m_pBufForDriver)
		{
			unsigned int nRecvBufSize = 0;
			unsigned int nRet = m_pcMyCamera[i]->GetIntValue("PayloadSize", &nRecvBufSize);	// 获取一帧数据大小

			m_pcMyCamera[i]->m_nBufSizeForDriver = nRecvBufSize;  // 一帧数据大小
			m_pcMyCamera[i]->m_pBufForDriver = (unsigned char*)malloc(m_pcMyCamera[i]->m_nBufSizeForDriver);
		}

		nRet = m_pcMyCamera[i]->GetOneFrameTimeout(m_pcMyCamera[i]->m_pBufForDriver, &nDataLen, m_pcMyCamera[i]->m_nBufSizeForDriver, &stImageInfo, 1000);
		if (MV_OK == nRet) {
			// ch:仅在第一次保存图像时申请缓存，在 CloseDevice 时释放
			if (NULL == m_pcMyCamera[i]->m_pBufForSaveImage)
			{
				// ch:BMP图片大小：width * height * 3 + 2048(预留BMP头大小)
				m_pcMyCamera[i]->m_nBufSizeForSaveImage = stImageInfo.nWidth * stImageInfo.nHeight * 3 + 2048;

				m_pcMyCamera[i]->m_pBufForSaveImage = (unsigned char*)malloc(m_pcMyCamera[i]->m_nBufSizeForSaveImage);

			}
			// ch:设置对应的相机参数 | en:Set camera parameter
			MV_SAVE_IMAGE_PARAM_EX stParam = { 0 };
			stParam.enImageType = m_nSaveImageType;							// 保存的图像类型 | MV_Image_Bmp \ MV_Image_Jpeg
			stParam.enPixelType = stImageInfo.enPixelType;					// 相机对应的像素格式 | Pixel format
			stParam.nBufferSize = m_pcMyCamera[i]->m_nBufSizeForSaveImage;  // 存储节点的大小 | Buffer node size
			stParam.nWidth = stImageInfo.nWidth;							// 相机对应的宽 | Width
			stParam.nHeight = stImageInfo.nHeight;							// 相机对应的高 | Height
			stParam.nDataLen = stImageInfo.nFrameLen;
			stParam.pData = m_pcMyCamera[i]->m_pBufForDriver;
			stParam.pImageBuffer = m_pcMyCamera[i]->m_pBufForSaveImage;
			stParam.nJpgQuality = 90;										// jpg编码，仅在保存Jpg图像时有效。保存BMP时SDK内忽略该参数
			// 将图像保存到  内存中
			nRet = m_pcMyCamera[i]->SaveImage(IN OUT & stParam);			// 将从设备采集到的原始图像数据  转换成JPEG或者BMP等格式  并存放在指定内存中


			char chImageName[IMAGE_NAME_LEN] = { 0 };
			if (MV_Image_Bmp == stParam.enImageType) {		// Bmp 格式
				if (i == 0) {
					//sprintf_s(chImageName, IMAGE_NAME_LEN, "Image_w%d_h%d_fn%03d_L.bmp", stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);
					//sprintf_s(chImageName, IMAGE_NAME_LEN, "%03d_L.bmp", stImageInfo.nFrameNum);
					sprintf_s(chImageName, IMAGE_NAME_LEN, "current_image.bmp", stImageInfo.nFrameNum);
				}
			}
			else if (MV_Image_Jpeg == stParam.enImageType) {	// Jpeg 格式
				if (i == 0) {
					//sprintf_s(chImageName, IMAGE_NAME_LEN, "Image_w%d_h%d_fn%03d_L.bmp", stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);
					sprintf_s(chImageName, IMAGE_NAME_LEN, "current_image.bmp", stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);
				}
			}

			FILE* fp = fopen(chImageName, "wb");	// 打开内存中的文件
			// 将buf中的数据 写入到文件	fwrite(buf ，size，count，fp)
			// 将指针buf 指向的内存中的数据  写入 fp指向的文件，写入的内存数据共有 count个，每个为size字节（读入数据类型的内存长度）
			fwrite(m_pcMyCamera[i]->m_pBufForSaveImage, 1, stParam.nImageLen, fp);
			//ui->label_debug->setText("save imgs");
			fclose(fp);
		}
	}
}

void LaserVisionSensor::OnBnClickedImageProcessButton() 
{
	// std::thread imageProcess(WeldRecognizeCallback, this);	// 报错
	// std::thread imageProcess = std::thread([this] { this->WeldRecognizeCallback(); });
	// std::thread imageProcess = std::thread(&LaserVisionSensor::WeldRecognizeCallback, this);
	std::thread imageProcess(std::bind(&LaserVisionSensor::WeldRecognizeCallback, this));
	imageProcess.detach();								// 线程分离
}

/* 焊缝图像处理，根据识别的焊缝类型，调用相应的图像处理函数 */
void LaserVisionSensor::WeldRecognizeCallback()
{
	// Mat frame = imread(latestSaveName);		// 读取最新保存的图像
	Mat frame = imread("./test.bmp");	//	./  项目工作目录：E:\projectCode\VS Repository\LaserVisionSensor
	if (frame.empty())
	{
		std::cout << "Could not read the image: " << "current_image.bmp" << std::endl;
		QMessageBox::information(this, "Weld Recognize", "Could not read the image");
	}

	double scale_down = 0.4;
	Mat sizeSrc;
	cv::resize(frame, sizeSrc, Size(), scale_down, scale_down, INTER_LINEAR);

	WeldCom Com(sizeSrc);
	if (Com.wledType == "V")
	{
		// Com.weldP1 = V::laser1_Extract(Com.laser1_OnWorkpiece);
		// Com.weldP2 = V::laser2_Extract(Com.laser2_OnWorkpiece);
	}
	else if (Com.wledType == "Lap")
	{
		Com.weldP1 = Lap::laser1_Extract(Com.laser1_OnWorkpiece);
		Com.weldP2 = Lap::laser2_Extract(Com.laser2_OnWorkpiece);
	}
	else if (Com.wledType == "SingleBevel")
	{
		// Com.weldP1 = SingleBevel::laser1_Extract(Com.laser1_OnWorkpiece);
		// Com.weldP2 = SingleBevel::laser2_Extract(Com.laser2_OnWorkpiece);
	}
	else if (Com.wledType == "Square")
	{
		// Com.weldP1 = Square::laser1_Extract(Com.laser1_OnWorkpiece);
		// Com.weldP2 = Square::laser2_Extract(Com.laser2_OnWorkpiece);
	}
	Com.weldLine(sizeSrc);

	//--------------------------------------- 显示数据 ---------------------------------------
	// 焊缝类型
	QString weldType = QString::fromStdString(Com.wledType);
	ui.label_weldType->setText(weldType);
	// 倾斜角度
	QString degree = QString::number(Com.degree, 'f', 2);
	ui.label_weldAngle->setText(degree);

	// 焊缝宽度
	ui.label_wledWidth->setText("laser1: ? , laser2: ? ");

	// 初始点坐标
	QString initialCoor = QString("P1:(%1,%2) P2:(%3,%4)").arg(Com.initP1.x).arg(Com.initP1.y).arg(Com.initP2.x).arg(Com.initP2.y);
	ui.label_initialCoor->setText(initialCoor);

	//--------------------------------------- 显示图像 ---------------------------------------
	Mat temp;
	if (sizeSrc.channels() == 4)
		cvtColor(sizeSrc, temp, COLOR_BGRA2RGB);
	else if (sizeSrc.channels() == 3)
		cvtColor(sizeSrc, temp, COLOR_BGR2RGB);

	QImage disImage = QImage((const unsigned char*)(temp.data), temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
	// scaled比例 —— IgnoreAspectRatio：忽略比例  
	disImage = disImage.scaled(ui.label_result_display->size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
	// 显示图像 在标签上
	ui.label_result_display->setPixmap(QPixmap::fromImage(disImage));
	// ui.label_result_display->setPixmap(QPixmap::fromImage(disImage.scaled(ui.label_detect_display->size(), Qt::KeepAspectRatio)));
}

// 设置曝光时间 | Get Exposure Time
void LaserVisionSensor::SetExposureTime(void)
{
	// 从标签获取曝光值
	QString ExposureTime = ui.ExposureTimeLine->text();
	float val = ExposureTime.toFloat();
	if (val < 10000) {
		QMessageBox::information(this, "SetExposureTime Error", "ExposureTime < 10000");
		return;
	}

	for (unsigned int i = 0, j = 0; j < m_stDevList.nDeviceNum; j++, i++)
	{
		m_pcMyCamera[i]->SetFloatValue("ExposureTime", val);	// 默认设置为80000us, 也就是 0.08s
	}
	return;
}

// 获取曝光时间 | Get Exposure Time
int LaserVisionSensor::GetExposureTime(void)
{
	return 0;
}

// 获取增益 | Get Gain
int LaserVisionSensor::GetGain(void)
{
	return 0;
}

// 获取帧率 | Get Frame Rate
int LaserVisionSensor::GetFrameRate(void)
{
	return 0;
}

// 获取触发模式 | Get Trigger Mode
int LaserVisionSensor::GetTriggerMode(void)
{
	return 0;
}

// 设置触发模式 | Set Trigger Mode
void LaserVisionSensor::SetTriggerMode(int m_nTriggerMode)
{
	return;
}



