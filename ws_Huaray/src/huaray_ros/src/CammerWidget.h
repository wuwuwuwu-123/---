#ifndef _CAMMER_WIDGET_H__
#define _CAMMER_WIDGET_H__

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

#include "MVviewer/IMVApi.h"
#include "MessageQue.h"


/* 时间同步的共享内存部分 */
#include <unistd.h>
#include <chrono>

#include <iostream>
#include <fcntl.h>
#include <sys/ipc.h>
#include <sys/mman.h>

#include <math.h>

struct time_stamp {
  int64_t high;
  int64_t low;
};

// using namespace std;
/* 时间同步的共享内存部分 */


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
    unsigned char*	m_pImageBuf;          //　存储图像数据的内存地址
    int				m_nBufferSize;        //　图像数据的缓冲区大小
    int				m_nWidth;             //　图像的宽度（单位：像素）
    int				m_nHeight;            //　图像的高度（单位：像素）
    IMV_EPixelType	m_ePixelType;         //　图像像素的类型
    int				m_nPaddingX;          //　图像每行的填充字节数（单位：字节）
    int				m_nPaddingY;          //　图像每列的填充字节数（单位：字节）
    uint64_t		m_nTimeStamp;         //　图像帧的时间戳     
};


class CammerWidget
{

public:
    explicit CammerWidget();
    ~CammerWidget();

	//枚举触发方式
	enum ETrigType
	{
	  trigContinous = 0,	// 连续触发
	  trigSoftware = 1,	    // 软件触发
	  trigLine = 2,		    // 外部触发
	};

	std::string                         m_currentCameraKey;   // 当前相机key | current camera key
	IMV_HANDLE                          m_devHandle;	      // 相机句柄 | camera handle
	image_transport::CameraPublisher    m_pubimage_camInfo;

	TMessageQue<cv::Mat>				m_image_list;  // 全部队列操作已上锁
      
	double                              m_time1,m_time2,m_time3;  // 调试 time
	cv::Mat                             m_BGRImage;               // 

	std::string m_topicName;

	// 相机 SDK 参数
    int m_width, m_height, m_offsetX, m_offsetY;

    ETrigType m_trigType;
    double m_triggerDelay;

    int m_exposureAuto;
    double m_exposureTime;

    double m_gainRaw;
    int m_digitalShift;

    double m_gamma;
    int m_brightness;
    int m_saturation;

    std::string m_cam_frame_id;
	sensor_msgs::CameraInfoPtr m_camInfoPtr;; // 相机固定内参指针

	bool SetWidth(int Width);
	bool SetHeight(int Height);
	bool SetOffsetX(int OffsetX);
	bool SetOffsetY(int OffsetY);

	//切换采集方式、触发方式 （连续采集、外部触发、软件触发）
	void CameraChangeTrig(ETrigType trigType);
	// 设置触发延迟时间，单位 μs
	bool SetTriggerDelay(double TriggerDelay);

	//设置曝光模式
	bool SetExposureAuto(int ExposureAuto);
	//设置曝光时间
	bool SetExposeTime(double exposureTime); 

	//设置模拟增益
	bool SetGainRaw(double gainRaw); 
	//设置数字增益
	bool SetDigitalShift(int DigitalShift);

	//设置伽马校正因子
	bool SetGamma(double gama); 
	
	//设置亮度
	bool SetBrightness(int Brightness);
	//设置饱和度
	bool SetSaturation(int Saturation);

	//读取参数函数：从 ROS 参数服务器中获取参数，并保存到类成员变量
	bool loadCameraParameters(ros::NodeHandle &nh);
	//设置参数
	void setCameraParameters();
	//从 ROS 参数服务器中加载相机内参，并初始化 m_camInfoPtr
	void SetInitCameraInfo(ros::NodeHandle &nh);

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

	//发布ROS消息, 格式为 bgr8
	void publish_img(std_msgs::Header lidar_share_header);

};

#endif // _CAMMER_WIDGET_H__
