#!/usr/bin/env python
import rosbag
from collections import deque

def main():
    # 请将此处替换为你的 bag 文件路径
    bag_file = '/home/wu/2025-03-02-20-21-47.bag' # /huaray/image_raw
    # bag_file = '/home/wu/下载/fast_livo2/Retail_Street.bag' # /left_camera/image
    
    # 分别为相机和雷达消息创建队列（仅保存 header.stamp 的时间戳）
    camera_queue = deque()
    lidar_queue = deque()
    
    # 打开 bag 文件，只读取 相机 和 雷达 两个话题
    
    with rosbag.Bag(bag_file, 'r') as bag:
        # for topic, msg, t in bag.read_messages(topics=['/huaray/image_raw', '/livox/lidar']): # 这里的变量 t 表示该条消息在 bag 文件中被记录的时间戳，即消息写入 bag 文件时的时间
        for topic, msg, _ in bag.read_messages(topics=['/huaray/image_raw', '/livox/lidar']):
            # 仅使用消息中的 header.stamp 字段，如果没有 header 则跳过该消息
            if not hasattr(msg, 'header'):
                continue
            stamp_sec = msg.header.stamp.to_sec()
            if topic == '/huaray/image_raw':
                camera_queue.append(stamp_sec)
            elif topic == '/livox/lidar':
                lidar_queue.append(stamp_sec)
    
    # 交替输出相机和雷达消息的 header.stamp 时间戳
    print("交替输出传感器消息中的 header.stamp 时间戳：")
    while camera_queue or lidar_queue:
        if camera_queue:
            cam_time = camera_queue.popleft()
            print("Camera timestamp:", cam_time)
        if lidar_queue:
            lidar_time = lidar_queue.popleft()
            print("Lidar timestamp:", lidar_time)

if __name__ == '__main__':
    main()

