#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <mutex>
#include <signal.h> // 处理信号

#include "MVviewer/IMVApi.h"
// #include "ImageConvert/VideoRender.h"



/**
 * @brief The CFrameInfo class
 */
class CFrameInfo
{
public:
  CFrameInfo()
  {
    m_pImageBuf = NULL;
    m_nBufferSize = 0;
    m_nWidth = 0;
    m_nHeight = 0;
    m_ePixelType = gvspPixelMono8;
    m_nPaddingX = 0;
    m_nPaddingY = 0;
    m_nTimeStamp = 0;
  }

  ~CFrameInfo()
  {
  }

public:
  unsigned char*	m_pImageBuf;    //　存储图像数据的内存地址
  int				m_nBufferSize;        //　图像数据的缓冲区大小
  int				m_nWidth;             //　图像的宽度（单位：像素）
  int				m_nHeight;            //　图像的高度（单位：像素）
  IMV_EPixelType	m_ePixelType;   //　图像像素的类型
  int				m_nPaddingX;          //　图像每行的填充字节数（单位：字节）
  int				m_nPaddingY;          //　图像每列的填充字节数（单位：字节）
  uint64_t		m_nTimeStamp;       //　图像帧的时间戳     
};

//枚举触发方式
enum ETrigType
{
  trigContinous = 0,	// 连续拉流
  trigSoftware = 1,	  // 软件触发
  trigLine = 2,		    // 外部触发
};

std::string                   m_currentCameraKey;   // 当前相机key | current camera key
IMV_HANDLE                    m_devHandle;	        // 相机句柄 | camera handle
image_transport::Publisher    m_pubimage_camInfo;
std::mutex                    m_pubimage_mutex;
std::vector<cv::Mat>          m_image_list;         // 

double                        m_time1,m_time2,m_time3;  // 调试 time
cv::Mat                       m_BGRImage;               // 

//设置曝光
bool SetExposeTime(double exposureTime);
//设置增益
bool SetGainRaw(double gainRaw);
//设置伽马
bool SetGamma(double gama);

//检测像机数、序列号
void CameraCheck();
//打开相机
bool CameraOpen();
//关闭相机
bool CameraClose();
//开始采集
bool CameraStart();
//停止采集
bool CameraStop();
//切换采集方式、触发方式 （连续采集、外部触发、软件触发）
void CameraChangeTrig(ETrigType trigType);
/**
 * @brief 回调函数 FrameCallback(), 在 CameraStart() 中被调用
 * @param [in] pFrame 由图像采集库（IMV）在捕获每一帧图像时生成并填充的
 * @param [in] pUser  在调用 IMV_AttachGrabbing() 时传入的用户数据。this 被作为 pUser 传递，这通常是一个指向类对象的指针（比如 CammerWidget），用于让回调函数访问类的成员变量或成员函数。
 */
static void FrameCallback(IMV_Frame* pFrame, void* pUser);
//发布ROS消息, 格式为 bgr8
void publish_img();


/************ 以下为函数实现 ************/

bool SetExposeTime(double exposureTime)
{
  if (!m_devHandle)
  {
    return false;
  }

  int ret = IMV_OK;

  ret = IMV_SetDoubleFeatureValue(m_devHandle, "ExposureTime", exposureTime);
  if (IMV_OK != ret)
  {
    printf("set ExposureTime value = %0.2f fail, ErrorCode[%d]\n", exposureTime, ret);
    return false;
  }

  return true;
}

bool SetGainRaw(double gainRaw)
{
  if (!m_devHandle)
  {
    return false;
  }

  int ret = IMV_OK;

  ret = IMV_SetDoubleFeatureValue(m_devHandle, "GainRaw", gainRaw);
  if (IMV_OK != ret)
  {
    printf("set GainRaw value = %0.2f fail, ErrorCode[%d]\n", gainRaw, ret);
    return false;
  }

  return true;
}

bool SetGamma(double gama)
{
  if (!m_devHandle)
  {
    return false;
  }

  int ret = IMV_OK;

  ret = IMV_SetDoubleFeatureValue(m_devHandle, "Gamma", gama);
  if (IMV_OK != ret)
  {
    printf("set GainRaw value = %0.2f fail, ErrorCode[%d]\n", gama, ret);
    return false;
  }

  return true;
}

void CameraCheck()
{
  IMV_DeviceList deviceInfoList;
  if (IMV_OK != IMV_EnumDevices(&deviceInfoList, interfaceTypeAll))
  {
    printf("Enumeration devices failed!\n");
    return;
  }

  // 打印相机基本信息（厂商序列号, 厂商, 设备型号, 设备序列号）
  for (unsigned int i = 0; i < deviceInfoList.nDevNum; i++)
  {
    printf("Camera[%d] Info :\n", i);
    printf("    key           = [%s]\n", deviceInfoList.pDevInfo[i].cameraKey);
    printf("    vendor name   = [%s]\n", deviceInfoList.pDevInfo[i].vendorName);
    printf("    model         = [%s]\n", deviceInfoList.pDevInfo[i].modelName);
    printf("    serial number = [%s]\n", deviceInfoList.pDevInfo[i].serialNumber);
  }

  if (deviceInfoList.nDevNum < 1)
  {
    printf("no camera.\n");
  //	msgBoxWarn(tr("Device Disconnected."));
  }
  else
  {
    //默认设置列表中的第一个相机为当前相机，其他操作比如打开、关闭、修改曝光都是针对这个相机。
    m_currentCameraKey = deviceInfoList.pDevInfo[0].cameraKey;
  }
}

bool CameraOpen()
{
  int ret = IMV_OK;

  if (m_currentCameraKey.length() == 0)
  {
    printf("open camera fail. No camera.\n");
    return false;
  }

  if (m_devHandle)
  {
    printf("m_devHandle is already been create!\n");
    return false;
  }

  std::string cameraKeyArray = m_currentCameraKey;
  const char* cameraKey = cameraKeyArray.data();

  ret = IMV_CreateHandle(&m_devHandle, modeByCameraKey, (void*)cameraKey);
  if (IMV_OK != ret)
  {
    printf("create devHandle failed! cameraKey[%s], ErrorCode[%d]\n", cameraKey, ret);
    return false;
  }

  // 打开相机
  // Open camera
  ret = IMV_Open(m_devHandle);
  if (IMV_OK != ret)
  {
    printf("open camera failed! ErrorCode[%d]\n", ret);
    return false;
  }

  return true;
}

bool CameraClose()
{
	if (!m_devHandle)
	{
		return false;
	}

	int ret = IMV_OK;

	if (!m_devHandle)
	{
		printf("close camera fail. No camera.\n");
		return false;
	}

	if (false == IMV_IsOpen(m_devHandle))
	{
		printf("camera is already close.\n");
		return false;
	}

	ret = IMV_Close(m_devHandle);
	if (IMV_OK != ret)
	{
		printf("close camera failed! ErrorCode[%d]\n", ret);
		return false;
	}

	ret = IMV_DestroyHandle(m_devHandle);
	if (IMV_OK != ret)
	{
		printf("destroy devHandle failed! ErrorCode[%d]\n", ret);
		return false;
	}

	m_devHandle = NULL;

	return true;
}

bool CameraStart()
{
  if (!m_devHandle)
  {
    return false;
  }

  int ret = IMV_OK;

  if (IMV_IsGrabbing(m_devHandle))
  {
    printf("camera is already grebbing.\n");
    return false;
  }

  // ret = IMV_AttachGrabbing(m_devHandle, FrameCallback, this);
  ret = IMV_AttachGrabbing(m_devHandle, FrameCallback, nullptr);
  if (IMV_OK != ret)
  {
    printf("Attach grabbing failed! ErrorCode[%d]\n", ret);
    return false;
  }

  ret = IMV_StartGrabbing(m_devHandle);
  if (IMV_OK != ret)
  {
    printf("start grabbing failed! ErrorCode[%d]\n", ret);
    return false;
  }

  return true;
}

bool CameraStop()
{
	if (!m_devHandle)
	{
		return false;
	}

	int ret = IMV_OK;
	if (!IMV_IsGrabbing(m_devHandle))
	{
		printf("camera is already stop grebbing.\n");
		return false;
	}

	ret = IMV_StopGrabbing(m_devHandle);
	if (IMV_OK != ret)
	{
		printf("Stop grabbing failed! ErrorCode[%d]\n", ret);
		return false;
	}

	// 清空显示队列 
	// clear display queue
	// CFrameInfo frameOld;
	// while (_displayFrameQueue.get(frameOld))
	// {
	// 	free(frameOld.m_pImageBuf);
	// 	frameOld.m_pImageBuf = NULL;
	// }

	// _displayFrameQueue.clear();

  // 遍历 vector 并释放每个 Mat 的内存
  for (auto &img : m_image_list)
  {
    img.release(); // 释放 cv::Mat 占用的内存
  }
  // 清空 vector
  m_image_list.clear();

	return true;
}

void CameraChangeTrig(ETrigType trigType)
{
  if (!m_devHandle)
  {
    return;
  }

  int ret = IMV_OK;

  // 1.连续采集
  if (trigContinous == trigType)
  {
    // 设置触发模式
    // set trigger mode
    ret = IMV_SetEnumFeatureSymbol(m_devHandle, "TriggerMode", "Off");
    if (IMV_OK != ret)
    {
      printf("set TriggerMode value = Off fail, ErrorCode[%d]\n", ret);
      return;
    }
  }

  // 2.软件触发
  else if (trigSoftware == trigType)
  {
    // 设置触发器
    // set trigger
    ret = IMV_SetEnumFeatureSymbol(m_devHandle, "TriggerSelector", "FrameStart");
    if (IMV_OK != ret)
    {
      printf("set TriggerSelector value = FrameStart fail, ErrorCode[%d]\n", ret);
      return;
    }

    // 设置触发模式
    // set trigger mode
    ret = IMV_SetEnumFeatureSymbol(m_devHandle, "TriggerMode", "On");
    if (IMV_OK != ret)
    {
      printf("set TriggerMode value = On fail, ErrorCode[%d]\n", ret);
      return;
    }

    // 设置触发源为软触发
    // set triggerSource as software trigger
    ret = IMV_SetEnumFeatureSymbol(m_devHandle, "TriggerSource", "Software");
    if (IMV_OK != ret)
    {
      printf("set TriggerSource value = Software fail, ErrorCode[%d]\n", ret);
      return;
    }
  }

  // 3.外部触发
  else if (trigLine == trigType)
  {
    // 设置触发器
    // set trigger
    ret = IMV_SetEnumFeatureSymbol(m_devHandle, "TriggerSelector", "FrameStart");
    if (IMV_OK != ret)
    {
      printf("set TriggerSelector value = FrameStart fail, ErrorCode[%d]\n", ret);
      return;
    }

    // 设置触发模式
    // set trigger mode
    ret = IMV_SetEnumFeatureSymbol(m_devHandle, "TriggerMode", "On");
    if (IMV_OK != ret)
    {
      printf("set TriggerMode value = On fail, ErrorCode[%d]\n", ret);
      return;
    }

    // 设置触发源为Line1触发
    // set trigggerSource as Line1 trigger
    ret = IMV_SetEnumFeatureSymbol(m_devHandle, "TriggerSource", "Line1");
    if (IMV_OK != ret)
    {
      printf("set TriggerSource value = Line1 fail, ErrorCode[%d]\n", ret);
      return;
    }

	  // 设置外触发为上升沿（下降沿为FallingEdge） 
	  // Set trigger activation to RisingEdge(FallingEdge in opposite) 
	  ret = IMV_SetEnumFeatureSymbol(m_devHandle, "TriggerActivation", "RisingEdge");
	  if (IMV_OK != ret)
	  {
	  	printf("Set triggerActivation value failed! ErrorCode[%d]\n", ret);
	  	return;
	  }

  }
}


static void FrameCallback(IMV_Frame* pFrame, void* pUser)
{

  if (pFrame == NULL)
	{
		printf("pFrame is NULL\n");
		return;
	}

  // m_time1 = ros::Time::now().toSec();
  // std::cout << "m_time1: " << m_time1 << std::endl;
  CFrameInfo frameInfo;
  frameInfo.m_nWidth = (int)pFrame->frameInfo.width;
  frameInfo.m_nHeight = (int)pFrame->frameInfo.height;
  frameInfo.m_nBufferSize = (int)pFrame->frameInfo.size;
  frameInfo.m_nPaddingX = (int)pFrame->frameInfo.paddingX;
  frameInfo.m_nPaddingY = (int)pFrame->frameInfo.paddingY;
  frameInfo.m_ePixelType = pFrame->frameInfo.pixelFormat;
  frameInfo.m_pImageBuf = (unsigned char *)malloc(sizeof(unsigned char) * frameInfo.m_nBufferSize);
  frameInfo.m_nTimeStamp = pFrame->frameInfo.timeStamp;

  // std::cout << "type: " << frameInfo.m_ePixelType << std::endl;

  // 内存申请失败，直接返回。无任何函数实现
  // memory application failed, return directly

  // 内存申请成功, 进行如下处理
  if (frameInfo.m_pImageBuf != nullptr)
  {
    // 初始化 m_BGRImage
    if (m_BGRImage.empty())
    {
      m_BGRImage.create(frameInfo.m_nHeight, frameInfo.m_nWidth, CV_8UC3);
    }

    /* 
    memcpy(dest, src, size) 是 C/C++ 中用于内存复制的标准库函数：
        dest（目标地址）：frameInfo.m_pImageBuf，即我们自己分配的内存。
        src（源地址）：pFrame->pData，即设备提供的原始图像数据。
        size（复制的大小）：frameInfo.m_nBufferSize，即本帧图像的字节数。
    */
    memcpy(frameInfo.m_pImageBuf, pFrame->pData, frameInfo.m_nBufferSize);

    /* 段错误原因：构造 cv::Mat 时使用的是外部内存指针，cv::Mat 并不会立即复制数据，而是直接引用这块内存。调用 free 后，这块内存就被释放了，但 BayGB8_Image 中仍然保存着已失效的指针。
     * 随后在调用 cvtColor(BayGB8_Image, m_BGRImage, cv::COLOR_BayerGR2BGR); 时，程序会尝试访问已释放的内存，从而引发段错误。 */
    cv::Mat BayGB8_Image(frameInfo.m_nHeight, frameInfo.m_nWidth, CV_8UC1, (unsigned char *)frameInfo.m_pImageBuf);
    cv::Mat BayGB8_Image_clone = BayGB8_Image.clone();
    // 释放内存. 这个非常重要，如果不加会导致内存爆炸！
    free(frameInfo.m_pImageBuf);

    m_pubimage_mutex.lock();
    // 后续使用 BayGB8_Image_clone 进行处理
    cvtColor(BayGB8_Image_clone, m_BGRImage, cv::COLOR_BayerGR2BGR);  // COLOR_BayerGR2BGR
    m_image_list.push_back(m_BGRImage);
    // m_time2 = ros::Time::now().toSec();
    // std::cout.precision(18);
    // std::cout << "m_time2: " << m_time2 <<std::endl;
    // std::cout << "m_time2 - m_time1: " << m_time2 - m_time1 << std::endl;
    m_pubimage_mutex.unlock();
  }
}

void publish_img()
{
  m_pubimage_mutex.lock();
  if(m_image_list.size() > 0)
  {
    cv::Mat image_temp = m_image_list.front();
    std::vector<cv::Mat>::iterator k = m_image_list.begin();
    m_image_list.erase(k);
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_temp).toImageMsg();
    ros::Time time_c = ros::Time::now();
    img_msg->header.stamp = time_c;
    m_pubimage_camInfo.publish(img_msg);

    // m_time3 = time_c.toSec();
    // std::cout.precision(18);
    // std::cout << "m_time3: " << m_time3 << std::endl;
    // std::cout << "m_time3 - m_time1: " << m_time3 - m_time1 << std::endl;
    if(m_image_list.size() > 20)
    {
      m_image_list.clear();
    }

  }
  m_pubimage_mutex.unlock();
  // std::cout << "m_image_list.size(): " << m_image_list.size() << std::endl;
}

void SigintHandler(int sig)
{
    ROS_WARN("Caught SIGINT, shutting down...");
    
    // 先释放相机资源
    CameraStop();
    CameraClose();

    // 关闭 ROS 进程
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "huaray_ros");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    m_pubimage_camInfo = it.advertise("/huaray/image_raw",1);

    signal(SIGINT, SigintHandler);

    CameraCheck();
    CameraOpen();
    CameraStart();
    
    SetExposeTime(32680.70);
    SetGainRaw(2.01);
    SetGamma(0.86);

    CameraChangeTrig(ETrigType::trigContinous);

    ros::Rate rate(150);  // 控制循环速率
    while(ros::ok())
    {
      publish_img();
      ros::spinOnce();
      rate.sleep();    // 适当延时，防止占用过高 CPU
    }

    // 退出时确保相机资源释放
    CameraStop();
    CameraClose();

    return 0;
}
