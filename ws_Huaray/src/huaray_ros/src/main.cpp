#include <signal.h> // 处理信号
#include "CammerWidget.h"

CammerWidget* widget;

/* 时间同步的共享内存部分 */
std_msgs::Header share_header;
time_stamp *pointt;
/* 时间同步的共享内存部分 */

void SigintHandler(int sig)
{
    ROS_WARN("Caught SIGINT, shutting down...");
    
    // 先释放相机资源
    widget->CameraStop();
    widget->CameraClose();

    delete widget; 

    munmap(pointt, sizeof(time_stamp));

    // 关闭 ROS 进程
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    setlocale(LC_CTYPE, "");

    /* 时间同步的共享内存部分 */
    const char *user_name = getlogin();
    std::string path_for_time_stamp = "/home/" + std::string(user_name) + "/3rdparty/driver/timeshare";
    const char *shared_file_name = path_for_time_stamp.c_str();

    // 以读写方式打开文件; 如果文件不存在，调用会失败，不会创建新文件。
    int fd = open(shared_file_name, O_RDWR);

    // 检查文件是否打开成功
    if (fd == -1) // 打开失败
    {
      std::cout << std::endl;
      ROS_ERROR("camera file open failed\n"); // 输出错误日志
    }
    else // 打开成功
    {
      std::cout << std::endl;
      ROS_ERROR("camera file open success. descriptor: %d\n", fd); // 输出成功的文件描述符
    }

    // 将文件的内容映射到内存
    // mmap: 将文件描述符 `fd` 的内容映射到虚拟地址空间
    pointt = (time_stamp *)mmap(NULL,                   // 自动分配映射的内存地址
                                sizeof(time_stamp),     // 映射内存大小
                                PROT_READ | PROT_WRITE, // 设置映射区域为可读可写
                                MAP_SHARED,             // 映射为共享模式，修改同步到文件      
                                fd,                     // 文件描述符
                                0);                     // 文件起始位置   
    /* 时间同步的共享内存部分 */

    ros::init(argc, argv, "huaray_ros");
    ros::NodeHandle nh;

    // **添加初始化**
    widget = new CammerWidget();    

    widget->loadCameraParameters(nh);

    image_transport::ImageTransport it(nh);
    widget->m_pubimage_camInfo = it.advertiseCamera(widget->m_topicName, 1);  // 发布原始数据（raw）

    signal(SIGINT, SigintHandler);

    widget->CameraCheck();
    widget->CameraOpen();

    // widget->SetExposeTime(32680.70);
    // widget->SetGainRaw(2.01);
    // widget->SetGamma(0.86);
    // widget->CameraChangeTrig(widget->ETrigType::trigContinous);
    // 注意：要在执行 CameraStart() 函数前完成各参数的设置
    widget->setCameraParameters();
    widget->SetInitCameraInfo(nh);

    widget->CameraStart();

    ros::Rate rate(10);  // 控制循环速率
    while(ros::ok())
    {
      std::cout << "         while(ros::ok())          " << std::endl;

      /* 时间同步的共享内存部分 */
      int64_t low = pointt->low;
      double time_pc = low / 1000000000.0; // 对于 Mid-360，时间单位 ns --> s
      ros::Time rcv_time = ros::Time(time_pc);
      //****************************** TEST ****************************
      // std::cout << "----------------------------------------------------------------------" << std::endl;
      // std::string debug_msg;
      // debug_msg = "  FrameTime:  " + std::to_string(rcv_time.toSec());
      // ROS_ERROR(debug_msg.c_str());
      //****************************************************************
  
      if (pointt != MAP_FAILED && rcv_time.toSec() != 0)
      {
          share_header.stamp = rcv_time; // use the lidar's time
      }
      else
      {
          share_header.stamp = ros::Time::now();
      }
      // std::cout << "-----debug-----share_header.stamp: " << share_header.stamp << std::endl;
      /* 时间同步的共享内存部分 */

      widget->publish_img(share_header);
      ros::spinOnce();
      rate.sleep();    // 适当延时，防止占用过高 CPU
    }

    // 退出时确保相机资源释放
    widget->CameraStop();
    widget->CameraClose();

    // **释放内存，防止内存泄漏**
    delete widget;  

    munmap(pointt, sizeof(time_stamp) * 1);

    return 0;
}
