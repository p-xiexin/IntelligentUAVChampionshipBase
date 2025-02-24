#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs  # 用于 tf2 与 geometry_msgs 的转换
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
from geometry_msgs.msg import TransformStamped
import tf2_sensor_msgs  # 用于转换 PointCloud2
from scipy.spatial.transform import Rotation as R
import numpy as np
from src.ego_planner_v2.src.Utils.uav_utils.scripts.tf_assist import odom_pub


# # 全局变量存储 IMU 的时间戳
# imu_timestamp = None
# tf_buffer = None
# tf_listener = None

# IMU 数据回调函数，更新全局时间戳
# def imu_callback(imu_msg):
#     global imu_timestamp
#     imu_timestamp = imu_msg.header.stamp
#     rospy.loginfo("IMU timestamp updated: %s", imu_timestamp)

# LiDAR 数据回调函数，更新 LiDAR 时间戳并发布
def lidar_callback(lidar_msg):
    # global imu_timestamp, tf_buffer

    # if imu_timestamp is None:
    #     rospy.logwarn("IMU timestamp not available yet.")
    #     return

    # 创建一个新的 PointCloud2 消息，更新时间戳
    # new_lidar_msg = lidar_msg
    # new_lidar_msg.header.stamp = imu_timestamp  # 更新为 IMU 时间戳

    # 坐标系转换：将 LiDAR 数据从 'base_link' 转换到 'map'
    try:
        # 获取从 'base_link' 到 'map' 的转换
        transform = tf_buffer.lookup_transform('world', 'lidar', rospy.Time(0), rospy.Duration(1.0))

        # 转换点云坐标系
        transformed_cloud = tf2_sensor_msgs.do_transform_cloud(lidar_msg, transform)

        # 发布转换后的点云数据
        lidar_pub.publish(transformed_cloud)
        rospy.loginfo("Published transformed LiDAR data with updated timestamp: %s", rospy.Time.now())

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr("Transform error: %s", str(e))
def odom_callback(odom_msg):
    # 获取里程计数据
    position = odom_msg.pose.pose.position
    orientation = odom_msg.pose.pose.orientation

    # 确保四元数不是零向量，避免出现除零错误
    if (orientation.x == 0 and orientation.y == 0 and orientation.z == 0 and orientation.w == 0):
        rospy.logwarn("Received invalid quaternion. Skipping transformation.")
        return

    # 将 NED 坐标转换为 ENU 坐标
    t_x = position.x  # NED North -> ENU East
    t_y = -position.y  # NED East -> ENU North
    t_z = -position.z  # NED Down -> ENU Up

    # 获取四元数并转换为旋转矩阵
    q = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])

    # 旋转矩阵：NED -> ENU
    ned_to_enu_rotation = R.from_matrix([[1, 0, 0],
                                         [0, -1, 0],
                                         [0, 0, -1]])
    ned_to_enu_quat = ned_to_enu_rotation.as_quat()  # 得到四元数 (x, y, z, w)
    ned_to_enu_quat_inv = R.from_quat(ned_to_enu_quat).inv().as_quat()
    q_enu = R.from_quat(ned_to_enu_quat) * q * R.from_quat(ned_to_enu_quat_inv)  # NED到ENU的四元数变换

    # 获取最终四元数
    final_rot = q_enu.as_quat()  # 得到四元数


    # 创建新的 Odometry 消息
    new_odom_msg = Odometry()
    new_odom_msg.header.stamp = rospy.Time.now()
    new_odom_msg.header.frame_id = "world"
    new_odom_msg.child_frame_id = "odom"

    # 设置转换后的位姿
    new_odom_msg.pose.pose.position.x = t_x
    new_odom_msg.pose.pose.position.y = t_y
    new_odom_msg.pose.pose.position.z = t_z

    # 设置转换后的四元数
    new_odom_msg.pose.pose.orientation.x = final_rot[0]
    new_odom_msg.pose.pose.orientation.y = final_rot[1]
    new_odom_msg.pose.pose.orientation.z = final_rot[2]
    new_odom_msg.pose.pose.orientation.w = final_rot[3]

    new_odom_msg.twist=odom_msg.twist

    # 发布新的 Odometry 消息
    odom_pub.publish(new_odom_msg)
    rospy.loginfo("Published transformed odometry data: %s", rospy.Time.now())
if __name__ == '__main__':
    # 初始化 ROS 节点
    rospy.init_node('imu_lidar_sync_node')

    # 初始化 tf2 缓存和监听器
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # 创建 IMU 数据订阅者
    # imu_sub = rospy.Subscriber('/airsim_node/drone_1/imu/imu', Imu, imu_callback)

    # 创建 LiDAR 数据订阅者
    lidar_sub = rospy.Subscriber('/airsim_node/drone_1/lidar', PointCloud2, lidar_callback)
    odom_sub= rospy.Subscriber('/eskf_odom', Odometry, odom_callback)
    # 创建一个新的 Publisher，用于发布修改后的 LiDAR 数据
    lidar_pub = rospy.Publisher('/drone_0_airsim_node/drone_1/lidar_updated', PointCloud2, queue_size=10)
    odom_pub=rospy.Publisher('/drone_0_airsim_node/drone_1/odom_updated', Odometry, queue_size=10)

    # 启动 ROS 循环
    rospy.spin()
