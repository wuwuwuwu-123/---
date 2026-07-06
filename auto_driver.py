#!/usr/bin/python3
# coding=utf8
import subprocess
import rospy

def close_terminal_by_name(terminal_name):
    # 使用 wmctrl -l 来列出所有窗口和其标题
    wmctrl_process = subprocess.Popen(["wmctrl", "-l"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
    stdout, _ = wmctrl_process.communicate()

    # 在输出中查找指定名称的终端
    for line in stdout.splitlines():
        if terminal_name in line:
            # 获取终端的窗口 ID，并使用 wmctrl -i -c 命令关闭该终端
            window_id = line.split()[0]
            subprocess.run(["wmctrl", "-i", "-c", window_id])
            print("已关闭终端:", terminal_name)
            return

    # 如果未找到指定名称的终端
    print("未找到名称为", terminal_name, "的终端")

def step(commands):
    command_str = " && ".join(commands)  # 将命令列表连接为一个字符串，使用 && 分隔

    # subprocess.run() 函数用于执行外部命令。它会创建一个新的子进程，并等待子进程执行完成后返回。
    # gnome-terminal" 是要执行的命令，即打开一个新的终端
    # "--" 表示后面的参数将被传递给 gnome-terminal 命令
    # 使用 bash -c 执行命令
    rospy.sleep(0.5)
    subprocess.run(["gnome-terminal", "--", "bash", "-c", command_str]) # title 无效???
    # 添加标题："--title", str(i),

def step_name(commands):
    global i
    command_str = " && ".join(commands)  # 将命令列表连接为一个字符串，使用 && 分隔

    # subprocess.run() 函数用于执行外部命令。它会创建一个新的子进程，并等待子进程执行完成后返回。
    # gnome-terminal" 是要执行的命令，即打开一个新的终端
    # "--" 表示后面的参数将被传递给 gnome-terminal 命令
    # 使用 bash -c 执行命令
    rospy.sleep(0.5)
    subprocess.run(["gnome-terminal", "--title", str(i), "--", "bash", "-c", command_str]) # title 无效???
    # 添加标题："--title", str(i),
    i = i + 1

def start_roscore():
    command = "roscore"
    print("---启动{}".format(command))
    subprocess.Popen(["gnome-terminal", "--", "bash", "-c", command])

def kill_roscore():
    close_terminal_by_name("roscore http://WP:11311/")

i = 101 # 初始 title

if __name__ == '__main__':
    start_roscore() # 第一时间启动roscore
    rospy.sleep(2)
    

    rospy.init_node('auto_driver')
    rospy.loginfo('*****************************')

    # 雷达驱动
    command_lidar_drive = [
        "cd ~/3rdparty/driver/ws_livox",
        "source ./devel/setup.bash",
        "roslaunch livox_ros_driver2 msg_MID360.launch"
    ]
    # 相机驱动
    command_camera_drive = [
        "cd ~/3rdparty/driver/ws_Huaray",
        "source ./devel/setup.bash",
        "roslaunch huaray_ros huaray.launch use_rviz:=false"
    ]
    
    # RTK 驱动
    command_ins_drive = [
        "cd ~/test_ws",
        "source ./devel/setup.bash",
        "roslaunch ins570d_ros_driver ins570d.launch use_rviz_ins:=false"
    ]
    # show rviz
    command_show_rviz = [
        "cd ~/3rdparty/driver/ws_Huaray",
        "source ./devel/setup.bash",
        "roslaunch custommsg2rosmsg show_cloud_img.launch"
    ]
    # record
    command_record = [
        # "cd ~/", # rosbag 文件保存的路径
        "cd /media/wu/T9/bag",
        # "rosbag record -b 0 /huaray/image_raw /livox/lidar /livox/imu /ins570d/ins570d_enu /ins570d/ins570d_gps"
        "rosbag record -b 0 /huaray/image_raw /livox/lidar /livox/imu"
    ]


    rospy.set_param('flag', 0)       # 默认打开公共部分
    rospy.set_param('work_state', 1) # 设置无人车初始状态为空闲(0), 忙碌状态为(1)

    while True:
        r = rospy.Rate(1)                   # 1 Hz
        r.sleep()                           # 等待足够的时间，以满足之前设置的频率要求
        cur_flag = rospy.get_param("flag")    

        if(cur_flag == -1):
            print("ok!")
        
        elif(cur_flag == 0):   # 公共部分
            print("---打开雷达、相机、INS、驱动！---")
            rospy.set_param('flag', -1)

            rospy.sleep(0.5)
            step(command_lidar_drive)  # 
            rospy.sleep(5) 
            step(command_camera_drive) # 
            # rospy.sleep(5)
            # step(command_ins_drive) # 
            rospy.sleep(5)            
            step(command_show_rviz)  # 
            rospy.sleep(0.5)

            rospy.set_param('work_state', 0)
       
        elif(cur_flag == 1):
            print("开始 record!")
            rospy.set_param('work_state', 1)
            rospy.set_param('flag', -1)

            rospy.sleep(0.5)
            step(command_record)  # 
            rospy.sleep(0.5)

            rospy.set_param('work_state', 0)


        else:
            print(" floor 错误! ")






