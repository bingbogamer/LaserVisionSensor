/************************************************************************/
/* 以C++接口为基础，对常用函数进行二次封装，方便用户使用                     */
/************************************************************************/

#ifndef _MV_CAMERA_H_
#define _MV_CAMERA_H_

#ifndef MV_NULL
#define MV_NULL    0
#endif

#include "MvCameraControl.h"       // 包含所有的include文件
#include <string.h>

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/types_c.h"

using namespace cv;

class CMvCamera
{
public:
    CMvCamera();
    ~CMvCamera();

    // 获取SDK版本号 
    static int GetSDKVersion();

    // 枚举设备 | Enumerate Device
    static int EnumDevices(unsigned int nTLayerType, MV_CC_DEVICE_INFO_LIST* pstDevList);

    // 判断设备是否可达 | Is the device accessible
    static bool IsDeviceAccessible(MV_CC_DEVICE_INFO* pstDevInfo, unsigned int nAccessMode);

    // 打开设备 | Open Device
    int Open(MV_CC_DEVICE_INFO* pstDeviceInfo);

    // 关闭设备 | Close Device
    int Close();

    // 判断相机是否处于连接状态 | Is The Device Connected
    bool IsDeviceConnected();

    // 注册图像数据回调 | Register Image Data CallBack
    int RegisterImageCallBack(void(__stdcall* cbOutput)(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser), void* pUser);

    // 开启抓图 | Start Grabbing
    int StartGrabbing();

    // 停止抓图 | Stop Grabbing
    int StopGrabbing();

    // 主动获取一帧图像数据 | Get one frame initiatively
    int GetImageBuffer(MV_FRAME_OUT* pFrame, int nMsec);

    // 释放图像缓存 | Free image buffer
    int FreeImageBuffer(MV_FRAME_OUT* pFrame);

    // 主动获取一帧图像数据 | Get one frame initiatively
    int GetOneFrameTimeout(unsigned char* pData, unsigned int* pnDataLen, unsigned int nDataSize, MV_FRAME_OUT_INFO_EX* pFrameInfo, int nMsec);

    // 显示一帧图像 | Display one frame image
    int DisplayOneFrame(MV_DISPLAY_FRAME_INFO* pDisplayInfo);

    // 设置SDK内部图像缓存节点个数 | Set the number of the internal image cache nodes in SDK
    int SetImageNodeNum(unsigned int nNum);

    // 获取设备信息 | Get device information
    int GetDeviceInfo(MV_CC_DEVICE_INFO* pstDevInfo);

    // 获取GEV相机的统计信息 | Get detect info of GEV camera
    int GetGevAllMatchInfo(MV_MATCH_INFO_NET_DETECT* pMatchInfoNetDetect);

    // 获取U3V相机的统计信息 | Get detect info of U3V camera
    int GetU3VAllMatchInfo(MV_MATCH_INFO_USB_DETECT* pMatchInfoUSBDetect);

    // 获取和设置Int型参数，如 Width和Height，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
    // int GetIntValue(IN const char* strKey, OUT MVCC_INTVALUE_EX* pIntValue);
    int GetIntValue(IN const char* strKey, OUT unsigned int* pnValue);
    int SetIntValue(IN const char* strKey, IN int64_t nValue);

    // 获取和设置Enum型参数，如 PixelFormat，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
    int GetEnumValue(IN const char* strKey, OUT MVCC_ENUMVALUE* pEnumValue);
    int SetEnumValue(IN const char* strKey, IN unsigned int nValue);
    int SetEnumValueByString(IN const char* strKey, IN const char* sValue);

    // 获取和设置Float型参数，如 ExposureTime和Gain，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
    int GetFloatValue(IN const char* strKey, OUT MVCC_FLOATVALUE* pFloatValue);
    int SetFloatValue(IN const char* strKey, IN float fValue);

    // 获取和设置Bool型参数，如 ReverseX，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
    int GetBoolValue(IN const char* strKey, OUT bool* pbValue);
    int SetBoolValue(IN const char* strKey, IN bool bValue);

    // 获取和设置String型参数，如 DeviceUserID，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件UserSetSave
    int GetStringValue(IN const char* strKey, MVCC_STRINGVALUE* pStringValue);
    int SetStringValue(IN const char* strKey, IN const char* strValue);

    // 执行一次Command型命令，如 UserSetSave，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
    int CommandExecute(IN const char* strKey);

    // 探测网络最佳包大小(只对GigE相机有效) 
    int GetOptimalPacketSize(unsigned int* pOptimalPacketSize);

    // 注册消息异常回调
    int RegisterExceptionCallBack(void(__stdcall* cbException)(unsigned int nMsgType, void* pUser), void* pUser);

    // 注册单个事件回调 
    int RegisterEventCallBack(const char* pEventName, void(__stdcall* cbEvent)(MV_EVENT_OUT_INFO* pEventInfo, void* pUser), void* pUser);

    // 强制IP | Force IP
    int ForceIp(unsigned int nIP, unsigned int nSubNetMask, unsigned int nDefaultGateWay);

    // 配置IP方式 | IP configuration method
    int SetIpConfig(unsigned int nType);

    // 设置网络传输模式 | Set Net Transfer Mode
    int SetNetTransMode(unsigned int nType);

    // 像素格式转换 | Pixel format conversion
    int ConvertPixelType(MV_CC_PIXEL_CONVERT_PARAM* pstCvtParam);

    // 保存图片 | save image
    int SaveImage(MV_SAVE_IMAGE_PARAM_EX* pstParam);

    // 保存图片为文件 | Save the image as a file
    int SaveImageToFile(MV_SAVE_IMG_TO_FILE_PARAM* pstParam);

    // 设置是否为触发模式
    int setTriggerMode(unsigned int TriggerModeNum);

    // 设置触发源
    int setTriggerSource(unsigned int TriggerSourceNum);

    // 软触发
    int softTrigger();

    // 读取buffer
    int ReadBuffer(cv::Mat& image);

public:
    void* m_hDevHandle;                        // 设备句柄
    unsigned int    m_nTLayerType;             // LayerType

public:
    // ReadBuffer() 接收缓存
    unsigned char*  m_pBufForSaveImage;        // 输出图像缓存
    unsigned int    m_nBufSizeForSaveImage;    

    unsigned char*  m_pBufForDriver;           // 从驱动获取图像的缓存
    unsigned int    m_nBufSizeForDriver;       
};

#endif      //_MV_CAMERA_H_

