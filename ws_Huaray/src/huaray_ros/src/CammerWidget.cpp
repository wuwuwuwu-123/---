#include "CammerWidget.h"


/**
 * @brief 回调函数 FrameCallback(), 在 CameraStart() 中被调用。（注意：该函数不是 CammerWidget类 中的成员函数）
 * @param [in] pFrame 由图像采集库（IMV）在捕获每一帧图像时生成并填充的
 * @param [in] pUser  在调用 IMV_AttachGrabbing() 时传入的用户数据。this 被作为 pUser 传递，这通常是一个指向类对象的指针（比如 CammerWidget），用于让回调函数访问类的成员变量或成员函数。
 */
static void FrameCallback(IMV_Frame* pFrame, void* pUser)
{

  CammerWidget* pCammerWidget = (CammerWidget*)pUser;
  if (!pCammerWidget)
  {
  	printf("pCammerWidget is NULL!\n");
  	return;
  }

  std::cout << "********** FrameCallback() **********" << std::endl;
  std::cout.precision(18);
  pCammerWidget->m_time1 = ros::Time::now().toSec();
  std::cout << "-----debug-----m_time1: " << pCammerWidget->m_time1 << std::endl;

  CFrameInfo frameInfo;
  frameInfo.m_nWidth = (int)pFrame->frameInfo.width;
  frameInfo.m_nHeight = (int)pFrame->frameInfo.height;
  frameInfo.m_nBufferSize = (int)pFrame->frameInfo.size;
  frameInfo.m_nPaddingX = (int)pFrame->frameInfo.paddingX;
  frameInfo.m_nPaddingY = (int)pFrame->frameInfo.paddingY;
  frameInfo.m_ePixelType = pFrame->frameInfo.pixelFormat;
  frameInfo.m_pImageBuf = (unsigned char *)malloc(sizeof(unsigned char) * frameInfo.m_nBufferSize);
  frameInfo.m_nTimeStamp = pFrame->frameInfo.timeStamp;

  // std::cout << "-----debug-----type: " << frameInfo.m_ePixelType << std::endl; // type: 17301514, 即 gvspPixelMono8 格式

  // if 内存申请失败，直接返回。无任何函数实现
  // memory application failed, return directly

  // if 内存申请成功, 进行如下处理
  if (frameInfo.m_pImageBuf != nullptr)
  {
    // 初始化 m_BGRImage
    if (pCammerWidget->m_BGRImage.empty())
    {
      pCammerWidget->m_BGRImage.create(frameInfo.m_nHeight, frameInfo.m_nWidth, CV_8UC3);
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
	  frameInfo.m_pImageBuf = NULL;

    cvtColor(BayGB8_Image_clone, pCammerWidget->m_BGRImage, cv::COLOR_BayerGR2BGR);  // COLOR_BayerGR2BGR

    if (pCammerWidget->m_image_list.size() > 20)
    {
      ROS_WARN("警告: 队列消息数大于 20，删除旧帧！");
      cv::Mat frameOld;
      if ( pCammerWidget->m_image_list.get(frameOld) )
      {
        frameOld.release();
      }

    }

    /* 由于 cv::Mat 的默认拷贝是浅拷贝（共享数据），每次 push_back 的都是同一个底层数据的引用。
     * 如果后续又修改了 m_BGRImage 的内容，可能会导致已存入 vector 中的图像数据也发生变化或被覆盖。使用 clone() 解决该问题*/
    pCammerWidget->m_image_list.push_back(pCammerWidget->m_BGRImage.clone());

    pCammerWidget->m_time2 = ros::Time::now().toSec();
    std::cout.precision(18);
    std::cout << "-----debug-----m_time2: " << pCammerWidget->m_time2 <<std::endl;
    std::cout << "-----debug-----m_time2 - m_time1: " << pCammerWidget->m_time2 - pCammerWidget->m_time1 << std::endl;
  }
}

CammerWidget::CammerWidget():
	m_currentCameraKey("")
	, m_devHandle(NULL)
{
  m_camInfoPtr.reset(new sensor_msgs::CameraInfo());
}

CammerWidget::~CammerWidget()
{

}

void CammerWidget::SetInitCameraInfo(ros::NodeHandle &nh){
  
  nh.param("frame_id", m_cam_frame_id, std::string("huaray_camera_frame"));
  m_camInfoPtr->header.frame_id = m_cam_frame_id;

  // 图像尺寸
  m_camInfoPtr->width = m_width;
  m_camInfoPtr->height = m_height;

  // 畸变模型
  nh.param("distortion_model", m_camInfoPtr->distortion_model, std::string("plumb_bob"));

  // 读取畸变系数（D）：数组长度可变
  std::vector<double> D;
  if(nh.getParam("distortion_coefficients", D))
  {
    m_camInfoPtr->D = D;
  }
  else
  {
    ROS_WARN("Failed to load D");
  }

  // 读取相机内参矩阵（K）：需要 9 个元素
  std::vector<double> K;
  if(nh.getParam("camera_matrix", K) && K.size() == 9)
  {
    for (int i = 0; i < 9; ++i)
      m_camInfoPtr->K[i] = K[i];
  }
  else
  {
    ROS_WARN("Failed to load K or size is not 9");
  }

  // 读取校正矩阵（R）：需要 9 个元素
  std::vector<double> R;
  if(nh.getParam("rectification_matrix", R) && R.size() == 9)
  {
    for (int i = 0; i < 9; ++i)
      m_camInfoPtr->R[i] = R[i];
  }
  else
  {
    ROS_WARN("Failed to load R or size is not 9");
  }

  // 读取投影矩阵（P）：需要 12 个元素
  std::vector<double> P;
  if(nh.getParam("projection_matrix", P) && P.size() == 12)
  {
    for (int i = 0; i < 12; ++i)
      m_camInfoPtr->P[i] = P[i];
  }
  else
  {
    ROS_WARN("Failed to load P or size is not 12");
  }

  std::cout << "--------------------------------------------" << std::endl;
  std::cout << "CameraInfo Parameters:" << std::endl;
  std::cout << "    FrameID: " << m_camInfoPtr->header.frame_id << std::endl;
  std::cout << "    Width: " << m_camInfoPtr->width << std::endl;
  std::cout << "    Height: " << m_camInfoPtr->height << std::endl;
  std::cout << "    Distortion Model: " << m_camInfoPtr->distortion_model << std::endl;


  // 输出畸变系数 D
  std::cout << "  D: ";
  for (size_t i = 0; i < m_camInfoPtr->D.size(); ++i)
  {
    std::cout << m_camInfoPtr->D[i] << " ";
  }
  std::cout << std::endl;

  // 输出相机矩阵 K
  std::cout << "  K: ";
  for (int i = 0; i < 9; ++i)
  {
    std::cout << m_camInfoPtr->K[i] << " ";
  }
  std::cout << std::endl;

  // 输出校正矩阵 R
  std::cout << "  R: ";
  for (int i = 0; i < 9; ++i)
  {
    std::cout << m_camInfoPtr->R[i] << " ";
  }
  std::cout << std::endl;

  // 输出投影矩阵 P
  std::cout << "  P: ";
  for (int i = 0; i < 12; ++i)
  {
    std::cout << m_camInfoPtr->P[i] << " ";
  }
  std::cout << std::endl;
  
  std::cout << "--------------------------------------------" << std::endl;

}

bool CammerWidget::SetWidth(int Width){
  if (!m_devHandle)
  {
    return false;
  }

  int ret = IMV_OK;

  ret = IMV_SetIntFeatureValue(m_devHandle, "Width", Width);
  if (IMV_OK != ret)
  {
    printf("    set Width value = %d fail, ErrorCode[%d]\n", Width, ret);
    return false;
  }

  return true;
}

bool CammerWidget::SetHeight(int Height){
  if (!m_devHandle)
  {
    return false;
  }

  int ret = IMV_OK;

  ret = IMV_SetIntFeatureValue(m_devHandle, "Height", Height);
  if (IMV_OK != ret)
  {
    printf("    set Height value = %d fail, ErrorCode[%d]\n", Height, ret);
    return false;
  }

  return true;
}

bool CammerWidget::SetOffsetX(int OffsetX){
  if (!m_devHandle)
  {
    return false;
  }

  int ret = IMV_OK;

  ret = IMV_SetIntFeatureValue(m_devHandle, "OffsetX", OffsetX);
  if (IMV_OK != ret)
  {
    printf("    set OffsetX value = %d fail, ErrorCode[%d]\n", OffsetX, ret);
    return false;
  }

  return true;
}

bool CammerWidget::SetOffsetY(int OffsetY){
  if (!m_devHandle)
  {
    return false;
  }

  int ret = IMV_OK;

  ret = IMV_SetIntFeatureValue(m_devHandle, "OffsetY", OffsetY);
  if (IMV_OK != ret)
  {
    printf("    set OffsetY value = %d fail, ErrorCode[%d]\n", OffsetY, ret);
    return false;
  }

  return true;
}

bool CammerWidget::SetTriggerDelay(double TriggerDelay){
  if (!m_devHandle)
  {
    return false;
  }

  int ret = IMV_OK;

  ret = IMV_SetDoubleFeatureValue(m_devHandle, "TriggerDelay", TriggerDelay);
  if (IMV_OK != ret)
  {
    printf("    set TriggerDelay value = %0.2f fail, ErrorCode[%d]\n", TriggerDelay, ret);
    return false;
  }

  return true; 
}

bool CammerWidget::SetExposureAuto(int ExposureAuto){
  if (!m_devHandle)
  {
    return false;
  }

  int ret = IMV_OK;

  if (ExposureAuto == 0){
    ret = IMV_SetEnumFeatureSymbol(m_devHandle, "ExposureAuto", "Off");
    if (IMV_OK != ret){
      printf("set ExposureAuto value = Off fail, ErrorCode[%d]\n", ret);
      return false;
    }
  } 

  if (ExposureAuto == 1){
    ret = IMV_SetEnumFeatureSymbol(m_devHandle, "ExposureAuto", "Once");
    if (IMV_OK != ret){
      printf("set ExposureAuto value = Once fail, ErrorCode[%d]\n", ret);
      return false;
    }
  } 

  if (ExposureAuto == 2){
    ret = IMV_SetEnumFeatureSymbol(m_devHandle, "ExposureAuto", "Continuous");
    if (IMV_OK != ret){
      printf("set ExposureAuto value = Continuous fail, ErrorCode[%d]\n", ret);
      return false;
    }
  } 

  return true; 
}

bool CammerWidget::SetExposeTime(double exposureTime)
{
  if (!m_devHandle)
  {
    return false;
  }

  int ret = IMV_OK;

  ret = IMV_SetDoubleFeatureValue(m_devHandle, "ExposureTime", exposureTime);
  if (IMV_OK != ret)
  {
    printf("    set ExposureTime value = %0.2f fail, ErrorCode[%d]\n", exposureTime, ret);
    return false;
  }

  return true;
}

bool CammerWidget::SetGainRaw(double gainRaw)
{
  if (!m_devHandle)
  {
    return false;
  }

  int ret = IMV_OK;

  ret = IMV_SetDoubleFeatureValue(m_devHandle, "GainRaw", gainRaw);
  if (IMV_OK != ret)
  {
    printf("    set GainRaw value = %0.2f fail, ErrorCode[%d]\n", gainRaw, ret);
    return false;
  }

  return true;
}

bool CammerWidget::SetDigitalShift(int DigitalShift){
  if (!m_devHandle)
  {
    return false;
  }

  int ret = IMV_OK;

  ret = IMV_SetIntFeatureValue(m_devHandle, "DigitalShift", DigitalShift);
  if (IMV_OK != ret)
  {
    printf("    set DigitalShift value = %d fail, ErrorCode[%d]\n", DigitalShift, ret);
    return false;
  }

  return true;
}

bool CammerWidget::loadCameraParameters(ros::NodeHandle &nh){
    // 读取 Topic 名称
    nh.param<std::string>("TopicName", m_topicName, "/huaray/image_raw");

    // 读取图像格式参数
    nh.param("Width", m_width, 2592);
    nh.param("Height", m_height, 2048);
    nh.param("OffsetX", m_offsetX, 0);
    nh.param("OffsetY", m_offsetY, 0);

    // 读取触发模式和触发延迟
    int trigTypeInt;
    nh.param("TrigType", trigTypeInt, 0);
    switch(trigTypeInt)
    {
        case 0:
            m_trigType = trigContinous;
            break;
        case 1:
            m_trigType = trigSoftware;
            break;
        case 2:
            m_trigType = trigLine;
            break;
        default:
            m_trigType = trigContinous;
            break;
    }
    nh.param("TriggerDelay", m_triggerDelay, 0.0);

    // 读取曝光设置
    nh.param("ExposureAuto", m_exposureAuto, 0);
    nh.param("ExposureTime", m_exposureTime, 5000.0);

    // 读取增益设置
    nh.param("GainRaw", m_gainRaw, 15.0);
    nh.param("DigitalShift", m_digitalShift, 0);

    // 读取伽马校正因子
    double gamma;
    nh.param("Gamma", gamma, 0.6);
    m_gamma = gamma;

    // 读取亮度和饱和度
    nh.param("Brightness", m_brightness, 50);
    nh.param("Saturation", m_saturation, 50);

    return true;
}

void CammerWidget::setCameraParameters(){

  std::cout << "--------------------------------------------" << std::endl;
  std::cout << "Camera Parameters:" << std::endl;
  std::cout << "    TopicName: " << this->m_topicName << std::endl;
  std::cout << "    Width: " << this->m_width << std::endl;
  std::cout << "    Height: " << this->m_height << std::endl;
  std::cout << "    OffsetX: " << this->m_offsetX << std::endl;
  std::cout << "    OffsetY: " << this->m_offsetY << std::endl;
  std::cout << "    TrigType: " << static_cast<int>(m_trigType) << std::endl;
  std::cout << "    TriggerDelay: " << this->m_triggerDelay << std::endl;
  std::cout << "    ExposureAuto: " << this->m_exposureAuto << std::endl;
  std::cout << "    ExposureTime: " << this->m_exposureTime << std::endl;
  std::cout << "    GainRaw: " << this->m_gainRaw << std::endl;
  std::cout << "    DigitalShift: " << this->m_digitalShift << std::endl;
  std::cout << "    Gamma: " << this->m_gamma << std::endl;
  std::cout << "    Brightnes: " << this->m_brightness << std::endl;
  std::cout << "    Saturation: " << this->m_saturation << std::endl;
  std::cout << "--------------------------------------------" << std::endl;
  std::cout << "Please check for any errors !!!" << std::endl;

  this->SetWidth(this->m_width);
  this->SetHeight(this->m_height);
  this->SetOffsetX(this->m_offsetX);
  this->SetOffsetY(this->m_offsetY);

  this->CameraChangeTrig(this->m_trigType);
  this->SetTriggerDelay(this->m_triggerDelay);

  // this->SetExposureAuto(this->m_exposureAuto);
  this->SetExposeTime(this->m_exposureTime);

  this->SetGainRaw(this->m_gainRaw);
  this->SetDigitalShift(m_digitalShift); 

  this->SetGamma(this->m_gamma);

  this->SetBrightness(this->m_brightness); 
  this->SetSaturation(this->m_saturation); 

}

bool CammerWidget::SetGamma(double gama)
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

bool CammerWidget::SetBrightness(int Brightness){
  if (!m_devHandle)
  {
    return false;
  }

  int ret = IMV_OK;

  ret = IMV_SetIntFeatureValue(m_devHandle, "Brightness", Brightness);
  if (IMV_OK != ret)
  {
    printf("set Brightness value = %d fail, ErrorCode[%d]\n", Brightness, ret);
    return false;
  }

  return true;
}

bool CammerWidget::SetSaturation(int Saturation){
  if (!m_devHandle)
  {
    return false;
  }

  int ret = IMV_OK;

  ret = IMV_SetIntFeatureValue(m_devHandle, "Saturation", Saturation);
  if (IMV_OK != ret)
  {
    printf("set Saturation value = %d fail, ErrorCode[%d]\n", Saturation, ret);
    return false;
  }

  return true;
}

void CammerWidget::CameraCheck()
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

bool CammerWidget::CameraOpen()
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

bool CammerWidget::CameraClose()
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

bool CammerWidget::CameraStart()
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

  ret = IMV_AttachGrabbing(m_devHandle, FrameCallback, this);
//   ret = IMV_AttachGrabbing(m_devHandle, FrameCallback, nullptr);
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

bool CammerWidget::CameraStop()
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
	// clear display queue
	cv::Mat frameOld;
	while (m_image_list.get(frameOld))
	{
		frameOld.release(); // 释放 cv::Mat 占用的内存
	}

	m_image_list.clear();

	return true;
}

void CammerWidget::CameraChangeTrig(ETrigType trigType)
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


void CammerWidget::publish_img(std_msgs::Header lidar_share_header)
{
  // std::cout << "-----debug-----m_image_list.size(): " << m_image_list.size() << std::endl;

  cv::Mat image_temp;
  if(m_image_list.get(image_temp))
  {
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_temp).toImageMsg(); // 像素编码格式：bgr8
    
    // ros::Time time_now = ros::Time::now();

    /* 时间同步的共享内存部分 */
    img_msg->header.stamp = lidar_share_header.stamp;
    img_msg->header.frame_id = m_cam_frame_id;
    /* 时间同步的共享内存部分 */

    m_camInfoPtr->header.stamp = img_msg->header.stamp;
    m_pubimage_camInfo.publish(img_msg, m_camInfoPtr);

    /* 时间同步的共享内存部分 */
    //****************************** TEST ****************************
    std::string debug;
    debug = "  img stamp:  " + std::to_string(img_msg->header.stamp.toSec());
    ROS_ERROR(debug.c_str());
  
    std::string debug_info;
    debug_info = " info stamp:  " + std::to_string(m_camInfoPtr->header.stamp.toSec());
    ROS_ERROR(debug_info.c_str());
    //****************************************************************
    /* 时间同步的共享内存部分 */

    m_time3 = ros::Time::now().toSec();
    std::cout.precision(18);
    std::cout << "-----debug-----m_time3: " << m_time3 << std::endl;
    std::cout << "-----debug-----m_time3 - m_time2: " << m_time3 - m_time2 << std::endl;
    std::cout << "-----debug-----m_time3 - m_time1: " << m_time3 - m_time1 << std::endl;

    std::cout << "----------------------------------------------------------------------" << std::endl;
  }

}

