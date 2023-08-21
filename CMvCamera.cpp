#include "MvCamera.h"

CMvCamera::CMvCamera()
{
	m_hDevHandle = MV_NULL;     // #define MV_NULL    0
}

CMvCamera::~CMvCamera()
{
	if (m_hDevHandle)
	{
		MV_CC_DestroyHandle(m_hDevHandle);  // 销毁设备句柄
		m_hDevHandle = MV_NULL;				// 避免悬空
	}
}

// 获取SDK版本号 
int CMvCamera::GetSDKVersion()
{
	return MV_CC_GetSDKVersion();       // 获取SDK版本号
}

// 枚举设备 | Enumerate Device
int CMvCamera::EnumDevices(unsigned int nTLayerType, MV_CC_DEVICE_INFO_LIST* pstDevList)
{
	return MV_CC_EnumDevices(nTLayerType, pstDevList);
}

// 判断设备是否可达 | Is the device accessible
bool CMvCamera::IsDeviceAccessible(MV_CC_DEVICE_INFO* pstDevInfo, unsigned int nAccessMode)
{
	return MV_CC_IsDeviceAccessible(pstDevInfo, nAccessMode);
}

// 打开设备
int CMvCamera::Open(MV_CC_DEVICE_INFO* pstDeviceInfo)
{
	if (MV_NULL == pstDeviceInfo)
	{
		return MV_E_PARAMETER;
	}

	if (m_hDevHandle)
	{
		return MV_E_CALLORDER;
	}
	// 成功，返回MV_OK；错误，返回错误码
	// 根据输入的设备信息，创建库内部必须的资源和初始化内部模块。通过该接口创建句柄，调用SDK接口，会默认生成SDK日志文件
	int nRet = MV_CC_CreateHandle(&m_hDevHandle, pstDeviceInfo);       // 传入设备信息结构体pstDeviceInfo，传出：m_hDevHandle
	if (MV_OK != nRet)
	{
		return nRet;    // 不成功，直接返回错误号
	}
	// 成功创建句柄，打开设备
	// 成功，返回MV_OK；错误，返回错误码 
	nRet = MV_CC_OpenDevice(m_hDevHandle);
	if (MV_OK != nRet)
	{
		MV_CC_DestroyHandle(m_hDevHandle);  // 如果打开设备失败，则销毁创建的句柄
		m_hDevHandle = MV_NULL;         // 将创建的句柄还原为
	}

	return nRet;    // 打开设备成功，返回MV_OK
}

// 关闭设备 
int CMvCamera::Close()
{
	if (MV_NULL == m_hDevHandle)
	{
		return MV_E_HANDLE;     // 错误或无效的句柄  
	}

	MV_CC_CloseDevice(m_hDevHandle);

	int nRet = MV_CC_DestroyHandle(m_hDevHandle);   // 销毁句柄
	m_hDevHandle = MV_NULL;

	return nRet;
}

// 判断相机是否处于连接状态
bool CMvCamera::IsDeviceConnected()
{
	return MV_CC_IsDeviceConnected(m_hDevHandle);   // 设备处于连接状态，返回true；没连接或失去连接，返回false
}

// 注册 图像数据回调
int CMvCamera::RegisterImageCallBack(void(__stdcall* cbOutput)(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser), void* pUser)
{
	return MV_CC_RegisterImageCallBackEx(m_hDevHandle, cbOutput, pUser);
}

// 开始取流（不采集图像）
int CMvCamera::StartGrabbing()
{
	return MV_CC_StartGrabbing(m_hDevHandle);   // 成功，返回MV_OK；错误，返回错误码
}

// 停止取流（不采集图像）
int CMvCamera::StopGrabbing()
{
	return MV_CC_StopGrabbing(m_hDevHandle);
}

// 主动获取一帧图像数据
// 使用内部缓存获取一帧图片（与MV_CC_Display不能同时使用）
int CMvCamera::GetImageBuffer(MV_FRAME_OUT* pFrame, int nMsec)
{
	return MV_CC_GetImageBuffer(m_hDevHandle, pFrame, nMsec);
}

// 释放图像缓存
int CMvCamera::FreeImageBuffer(MV_FRAME_OUT* pFrame)
{
	return MV_CC_FreeImageBuffer(m_hDevHandle, pFrame);
}

// 采用超时机制获取一帧图片，SDK内部等待直到有数据时返回 
int CMvCamera::GetOneFrameTimeout(unsigned char* pData, unsigned int* pnDataLen, unsigned int nDataSize, MV_FRAME_OUT_INFO_EX* pFrameInfo, int nMsec)
{
	if (NULL == pnDataLen)
	{
		return MV_E_PARAMETER;
	}

	int nRet = MV_OK;

	*pnDataLen = 0;

	nRet = MV_CC_GetOneFrameTimeout(m_hDevHandle, pData, nDataSize, pFrameInfo, nMsec);
	if (MV_OK != nRet)
	{
		return nRet;
	}

	*pnDataLen = pFrameInfo->nFrameLen;

	return nRet;
}


// 设置显示窗口句柄 
int CMvCamera::DisplayOneFrame(MV_DISPLAY_FRAME_INFO* pDisplayInfo)
{
	return MV_CC_DisplayOneFrame(m_hDevHandle, pDisplayInfo);
}

// 设置SDK内部图像缓存节点个数 | en:Set the number of the internal image cache nodes in SDK
int CMvCamera::SetImageNodeNum(unsigned int nNum)
{
	return MV_CC_SetImageNodeNum(m_hDevHandle, nNum);
}


// 获取设备信息，取流之前调用
int CMvCamera::GetDeviceInfo(MV_CC_DEVICE_INFO* pstDevInfo)
{
	return MV_CC_GetDeviceInfo(m_hDevHandle, pstDevInfo);
}

// 获取GEV相机的统计信息
int CMvCamera::GetGevAllMatchInfo(MV_MATCH_INFO_NET_DETECT* pMatchInfoNetDetect)
{
	if (MV_NULL == pMatchInfoNetDetect)
	{
		return MV_E_PARAMETER;
	}

	MV_CC_DEVICE_INFO stDevInfo = { 0 };
	GetDeviceInfo(&stDevInfo);
	if (stDevInfo.nTLayerType != MV_GIGE_DEVICE)
	{
		return MV_E_SUPPORT;
	}

	MV_ALL_MATCH_INFO struMatchInfo = { 0 };

	struMatchInfo.nType = MV_MATCH_TYPE_NET_DETECT;
	struMatchInfo.pInfo = pMatchInfoNetDetect;
	struMatchInfo.nInfoSize = sizeof(MV_MATCH_INFO_NET_DETECT);
	memset(struMatchInfo.pInfo, 0, sizeof(MV_MATCH_INFO_NET_DETECT));

	return MV_CC_GetAllMatchInfo(m_hDevHandle, &struMatchInfo);
}

// 获取U3V相机的统计信息 
int CMvCamera::GetU3VAllMatchInfo(MV_MATCH_INFO_USB_DETECT* pMatchInfoUSBDetect)
{
	if (MV_NULL == pMatchInfoUSBDetect)
	{
		return MV_E_PARAMETER;
	}

	MV_CC_DEVICE_INFO stDevInfo = { 0 };
	GetDeviceInfo(&stDevInfo);
	if (stDevInfo.nTLayerType != MV_USB_DEVICE)
	{
		return MV_E_SUPPORT;
	}

	MV_ALL_MATCH_INFO struMatchInfo = { 0 };

	struMatchInfo.nType = MV_MATCH_TYPE_USB_DETECT;
	struMatchInfo.pInfo = pMatchInfoUSBDetect;
	struMatchInfo.nInfoSize = sizeof(MV_MATCH_INFO_USB_DETECT);
	memset(struMatchInfo.pInfo, 0, sizeof(MV_MATCH_INFO_USB_DETECT));

	return MV_CC_GetAllMatchInfo(m_hDevHandle, &struMatchInfo);
}

// 获取和设置Int型参数，如 Width和Height，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
int CMvCamera::GetIntValue(IN const char* strKey, OUT unsigned int* pnValue)
{
	if (NULL == strKey || NULL == pnValue)
	{
		return MV_E_PARAMETER;
	}

	MVCC_INTVALUE stParam;
	memset(&stParam, 0, sizeof(MVCC_INTVALUE));
	int nRet = MV_CC_GetIntValue(m_hDevHandle, strKey, &stParam);
	if (MV_OK != nRet)
	{
		return nRet;
	}

	*pnValue = stParam.nCurValue;   // OUT ：当前值

	return MV_OK;
}


int CMvCamera::SetIntValue(IN const char* strKey, IN int64_t nValue)
{
	return MV_CC_SetIntValueEx(m_hDevHandle, strKey, nValue);
}

// 获取和设置Enum型参数，如 PixelFormat，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
int CMvCamera::GetEnumValue(IN const char* strKey, OUT MVCC_ENUMVALUE* pEnumValue)
{
	return MV_CC_GetEnumValue(m_hDevHandle, strKey, pEnumValue);
}

int CMvCamera::SetEnumValue(IN const char* strKey, IN unsigned int nValue)
{
	return MV_CC_SetEnumValue(m_hDevHandle, strKey, nValue);
}

int CMvCamera::SetEnumValueByString(IN const char* strKey, IN const char* sValue)
{
	return MV_CC_SetEnumValueByString(m_hDevHandle, strKey, sValue);
}

// 获取和设置Float型参数，如 ExposureTime和Gain，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
int CMvCamera::GetFloatValue(IN const char* strKey, OUT MVCC_FLOATVALUE* pFloatValue)
{
	return MV_CC_GetFloatValue(m_hDevHandle, strKey, pFloatValue);
}

int CMvCamera::SetFloatValue(IN const char* strKey, IN float fValue)
{
	return MV_CC_SetFloatValue(m_hDevHandle, strKey, fValue);
}

// 获取和设置Bool型参数，如 ReverseX，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
int CMvCamera::GetBoolValue(IN const char* strKey, OUT bool* pbValue)
{
	return MV_CC_GetBoolValue(m_hDevHandle, strKey, pbValue);
}

int CMvCamera::SetBoolValue(IN const char* strKey, IN bool bValue)
{
	return MV_CC_SetBoolValue(m_hDevHandle, strKey, bValue);
}

// 获取和设置String型参数，如 DeviceUserID，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件UserSetSave
int CMvCamera::GetStringValue(IN const char* strKey, MVCC_STRINGVALUE* pStringValue)
{
	return MV_CC_GetStringValue(m_hDevHandle, strKey, pStringValue);
}

int CMvCamera::SetStringValue(IN const char* strKey, IN const char* strValue)
{
	return MV_CC_SetStringValue(m_hDevHandle, strKey, strValue);
}

// 执行一次Command型命令，如 UserSetSave，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
int CMvCamera::CommandExecute(IN const char* strKey)
{
	return MV_CC_SetCommandValue(m_hDevHandle, strKey);
}

// 探测网络最佳包大小(只对GigE相机有效)
int CMvCamera::GetOptimalPacketSize(unsigned int* pOptimalPacketSize)
{
	if (MV_NULL == pOptimalPacketSize)
	{
		return MV_E_PARAMETER;
	}

	int nRet = MV_CC_GetOptimalPacketSize(m_hDevHandle);
	if (nRet < MV_OK)
	{
		return nRet;
	}

	*pOptimalPacketSize = (unsigned int)nRet;

	return MV_OK;
}

// 注册消息异常回调 | Register Message Exception CallBack
int CMvCamera::RegisterExceptionCallBack(void(__stdcall* cbException)(unsigned int nMsgType, void* pUser), void* pUser)
{
	return MV_CC_RegisterExceptionCallBack(m_hDevHandle, cbException, pUser);
}

// 注册单个事件回调 | Register Event CallBack
int CMvCamera::RegisterEventCallBack(const char* pEventName, void(__stdcall* cbEvent)(MV_EVENT_OUT_INFO* pEventInfo, void* pUser), void* pUser)
{
	return MV_CC_RegisterEventCallBackEx(m_hDevHandle, pEventName, cbEvent, pUser);
}

// 强制IP | Force IP
// 强制设置设备网络参数（包括IP、子网掩码、默认网关），强制设置之后将需要重新创建设备句柄，仅GigEVision设备支持
int CMvCamera::ForceIp(unsigned int nIP, unsigned int nSubNetMask, unsigned int nDefaultGateWay)
{
	return MV_GIGE_ForceIpEx(m_hDevHandle, nIP, nSubNetMask, nDefaultGateWay);
}

// 配置IP方式 | IP configuration method
// 发送命令设置设备的IP方式，如DHCP、LLA等，仅GigEVision设备支持。
int CMvCamera::SetIpConfig(unsigned int nType)
{
	return MV_GIGE_SetIpConfig(m_hDevHandle, nType);
}

// 置网络传输模式 | Set Net Transfer Mode
int CMvCamera::SetNetTransMode(unsigned int nType)
{
	return MV_GIGE_SetNetTransMode(m_hDevHandle, nType);
}

// 像素格式转换 | Pixel format conversion
int CMvCamera::ConvertPixelType(MV_CC_PIXEL_CONVERT_PARAM* pstCvtParam)
{
	return MV_CC_ConvertPixelType(m_hDevHandle, pstCvtParam);
}

// 保存图片, 存放到指定内存中 | save image
// 通过将接口可以将从设备采集到的原始图像数据  转换成JPEG或者BMP等格式  
// 并存放在指定内存中，然后用户可以将转换之后的数据 直接保存成图片文件
int CMvCamera::SaveImage(MV_SAVE_IMAGE_PARAM_EX* pstParam)
{
	return MV_CC_SaveImageEx2(m_hDevHandle, pstParam);
}

// 保存图片为文件 | Save the image as a file
int CMvCamera::SaveImageToFile(MV_SAVE_IMG_TO_FILE_PARAM* pstSaveFileParam)
{
	return MV_CC_SaveImageToFile(m_hDevHandle, pstSaveFileParam);
}


// 设置是否开启触发模式
int CMvCamera::setTriggerMode(unsigned int TriggerModeNum)
{
	//0：Off  1：On
	int tempValue = MV_CC_SetEnumValue(m_hDevHandle, "TriggerMode", TriggerModeNum);
	if (tempValue != 0) {
		return -1;      // 失败，返回-1
	}
	else {
		return 0;
	}
}

// 设置 触发源
int CMvCamera::setTriggerSource(unsigned int TriggerSourceNum)
{
	//0：Line0  1：Line1  7：Software
	int tempValue = MV_CC_SetEnumValue(m_hDevHandle, "TriggerSource", TriggerSourceNum);    // 设置属性"TriggerSource" 的值为 TriggerSourceNum
	if (tempValue != 0) {
		return -1;      // 失败，返回-1
	}
	else {
		return 0;
	}
}

// ************************************************************************************************
// 发送软触发
int CMvCamera::softTrigger()
{
	int tempValue = MV_CC_SetCommandValue(m_hDevHandle, "TriggerSoftware");
	if (tempValue != 0) {
		return -1;  // 失败，返回-1
	}
	else {
		return 0;   // 成功，返回0
	}
}

// 读取相机内部的取流数据缓存，并将数据转换成BGR8格式，保存到参入形参 image中 传出
// 成功返回0   失败：返回-1
int CMvCamera::ReadBuffer(cv::Mat& image)
{
	cv::Mat* getImage = new cv::Mat();  

	MVCC_INTVALUE stParam;
	memset(&stParam, 0, sizeof(MVCC_INTVALUE));

	// PayloadSize : 流通道上的每个图像传输的最大字节数（一帧数据的大小）
	// OUT：有关相机属性 结构体指针stParam （当前值、最大值、最小值）
	int retValue = MV_CC_GetIntValue(IN m_hDevHandle, IN "PayloadSize", IN OUT & stParam);
	if (retValue != MV_OK) {
		return -1; 
	}
	unsigned int nRecvBufSize = stParam.nCurValue;					// 接收缓存大小 = PayloadSize当前值
	unsigned char* pDate = (unsigned char*)malloc(nRecvBufSize);	// pDate ： 接收图像数据缓存      

	//	pDate: 指向接收的图像数据的 指针（获取的图像缓存保存在 应用层pDate中）
	//	stImageInfo: 图像信息结构体(宽、高、像素格式！！)
	MV_FRAME_OUT_INFO_EX stImageInfo = { 0 };
	retValue = MV_CC_GetOneFrameTimeout(IN m_hDevHandle, OUT pDate, IN nRecvBufSize, OUT & stImageInfo, 500);
	if (retValue != MV_OK)
	{
		return -1;
	}

	bool isMono;
	switch (stImageInfo.enPixelType)    // 像素格式  
	{
	case PixelType_Gvsp_Mono8:
	case PixelType_Gvsp_Mono10:
	case PixelType_Gvsp_Mono10_Packed:
	case PixelType_Gvsp_Mono12:
	case PixelType_Gvsp_Mono12_Packed:
		isMono = true;
		break;
	default:
		isMono = false;
		break;
	}

	// 输出图像缓存 m_pBufForSaveImage
	m_nBufSizeForSaveImage = stImageInfo.nWidth * stImageInfo.nHeight * 3 + 2048;			// 3通道 + 2048字节
	unsigned char* m_pBufForSaveImage = (unsigned char*)malloc(m_nBufSizeForSaveImage); 


	if (isMono)
	{
		*getImage = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC1, pDate);
	}
	else
	{   // 3通道图像
		// 转换图像格式为 PixelType_Gvsp_BGR8_Packed
		MV_CC_PIXEL_CONVERT_PARAM stConvertParam = { 0 };
		memset(&stConvertParam, 0, sizeof(MV_CC_PIXEL_CONVERT_PARAM));
		stConvertParam.nWidth = stImageInfo.nWidth;						
		stConvertParam.nHeight = stImageInfo.nHeight;				

		//stConvertParam.pSrcData = m_pBufForDriver;					//ch:输入数据缓存 | en:input data buffer
		stConvertParam.pSrcData = pDate;                                //ch:输入数据缓存 | en:input data buffer
		stConvertParam.nSrcDataLen = stImageInfo.nFrameLen;             //ch:输入数据大小 | en:input data size
		stConvertParam.enSrcPixelType = stImageInfo.enPixelType;        //ch:输入像素格式 | en:input pixel format

		// stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;  //ch:输出像素格式BGR8 | en:output pixel format  适用于OPENCV的图像格式
		stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;		//ch:输出像素格式RGB8 | en:output pixel format
		stConvertParam.pDstBuffer = m_pBufForSaveImage;    OUT          //ch:输出数据缓存 | en:output data buffer
		stConvertParam.nDstBufferSize = m_nBufSizeForSaveImage;			//ch:输出缓存大小 | en:output buffer size
		MV_CC_ConvertPixelType(m_hDevHandle, &stConvertParam);

		// 从输出数据缓存（转换成BGR8）
		*getImage = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, m_pBufForSaveImage);
	}

	(*getImage).copyTo(image);  // 获取到的图像，copy给传入的image
	(*getImage).release();      // 释放new出来的 图像指针  内存空间
	free(pDate);                // 释放接收缓存的 内存空间
	free(m_pBufForSaveImage);
	return 0;
}
