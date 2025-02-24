import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import numpy as np
from scipy.spatial.transform import Rotation as R


def publish_tf(odom_msg, header):
    """
    计算并发布 world -> body 和 body -> lidar 变换。
    从里程计消息中获取位置和方向数据，并转换为所需的坐标系。
    """

    # 创建 TF 变换广播器
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # === 发布 world -> body 变换 ===
    # 获取里程计数据
    position = odom_msg.pose.pose.position
    orientation = odom_msg.pose.pose.orientation

    # 确保四元数不是零向量，避免出现除零错误
    if (orientation.x == 0 and orientation.y == 0 and orientation.z == 0 and orientation.w == 0):
        rospy.logwarn("Received invalid quaternion. Skipping transformation.")
        return

    # 将 NED 坐标转换为 ENU 坐标
    t_x = position.x # NED North -> ENU East
    t_y = -position.y  # NED East -> ENU North
    t_z = -position.z  # NED Down -> ENU Up

    # 获取四元数并转换为旋转矩阵
    q = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])

    # 确保四元数已经归一化（如果不是单位四元数则进行归一化）
    q = R.from_quat(q.as_quat())

    rotation_matrix = q.as_matrix()

    # 将旋转矩阵转换为四元数
    q_new = R.from_matrix(rotation_matrix).as_quat()

    # ENU 到 NED 的旋转四元数（参考提供的 C++ 代码）
    # 1. 绕Z轴旋转180度，交换X和Y
    yaw = np.pi # 绕Z轴旋转180度
    enu_to_ned_q = R.from_euler('z', yaw).as_quat()

    # 2. 绕Y轴旋转180度，反转Z轴
    invert_z_q = R.from_euler('y', np.pi).as_quat()

    # 合并两个四元数
    enu_to_ned_q = R.from_quat(enu_to_ned_q) * R.from_quat(invert_z_q)

    # 将最终的四元数转化为 geometry_msgs::Quaternion 格式
    quat = enu_to_ned_q.as_quat()

    # 将原始的旋转四元数与坐标系转换四元数结合
    odom_rot = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])

    # 确保原始四元数已经归一化
    odom_rot = R.from_quat(odom_rot.as_quat())

    # 合并旋转：ENU到NED的转换四元数 * 原始姿态四元数
    final_rot = enu_to_ned_q * odom_rot

    # 创建 TransformStamped 对象
    odom_tf = TransformStamped()
    odom_tf.header.stamp = header.stamp
    odom_tf.header.frame_id = "world"
    odom_tf.child_frame_id = "body"

    # 设置变换的平移和旋转
    odom_tf.transform.translation.x = t_x
    odom_tf.transform.translation.y = t_y
    odom_tf.transform.translation.z = t_z

    odom_tf.transform.rotation.x = final_rot.as_quat()[0]
    odom_tf.transform.rotation.y = final_rot.as_quat()[1]
    odom_tf.transform.rotation.z = final_rot.as_quat()[2]
    odom_tf.transform.rotation.w = final_rot.as_quat()[3]

    odom_tf.transform.translation.x = odom_msg.pose.pose.position.x
    odom_tf.transform.translation.y = odom_msg.pose.pose.position.y
    odom_tf.transform.translation.z = odom_msg.pose.pose.position.z

    odom_tf.transform.rotation.x = odom_msg.pose.pose.orientation.x
    odom_tf.transform.rotation.y = odom_msg.pose.pose.orientation.y
    odom_tf.transform.rotation.z = odom_msg.pose.pose.orientation.z
    odom_tf.transform.rotation.w = odom_msg.pose.pose.orientation.w

    # 发送 world -> body 变换
    tf_broadcaster.sendTransform(odom_tf)

    # === 发布 body -> lidar 变换 ===
    lidar_tf = TransformStamped()
    lidar_tf.header.stamp = header.stamp
    lidar_tf.header.frame_id = "body"
    lidar_tf.child_frame_id = "lidar"

    # 假设 LiDAR 在 body 坐标系上方 50mm
    lidar_tf.transform.translation.x = 0.0
    lidar_tf.transform.translation.y = 0.0
    lidar_tf.transform.translation.z = 0.05  # LiDAR 50mm 上方

    # LiDAR 相对于 body 不存在旋转（假设对齐）
    lidar_tf.transform.rotation.x = 0.0
    lidar_tf.transform.rotation.y = 0.0
    lidar_tf.transform.rotation.z = 0.0
    lidar_tf.transform.rotation.w = 1.0

    # 发送 body -> lidar 变换
    tf_broadcaster.sendTransform(lidar_tf)


def odom_callback(odom_msg):
    """
    处理 /eskf_odom 话题的回调函数，获取里程计数据并发布 TF 变换。
    """

    # 创建 ROS header
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "world"  # 父坐标系为 world

    # 发布变换
    publish_tf(odom_msg, header)


def main():
    """初始化 ROS 节点并订阅 /eskf_odom 话题"""

    # 初始化 ROS 节点
    rospy.init_node('tf_broadcaster_eskf', anonymous=True)

    # 订阅 /eskf_odom 话题
    rospy.Subscriber("/drone_0_visual_slam/odom", Odometry, odom_callback)

    # 进入 ROS 事件循环
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
