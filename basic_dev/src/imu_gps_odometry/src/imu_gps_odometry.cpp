#include "imu_gps_odometry.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Vector3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf/transform_datatypes.h>

geometry_msgs::PoseStamped global_init;
nav_msgs::Odometry local_init;
bool slam_init = false;


int main(int argc, char** argv)
{
    // g_eskf_ptr = new ErrorStateKalmanFilter(-9.81083, 0.1, 0.1, 0.1, 0.0003158085227, 0.001117221, 0.5*10, 1.0, 0.00143, 0.0386);
    //(重力， P_位置不确定度_std, P_速度不确定度_std, P_角度不确定度_std, P_角速度bias不确定度_std, P_加速度bias不确定度_std,
    //gps位置测量噪声_std gpsz姿态测量噪声_std, imu角速度测量噪声_std, imu加速度测量噪声_std)
    g_eskf_ptr = new ErrorStateKalmanFilter(
        -9.81083,         // 重力
        0.1,             // 位置不确定度_std (米)
        0.1,              // 速度不确定度_std (m/s)
        0.1,              // 姿态不确定度_std (弧度)
        0.0003,            // 角速度零偏随机游走_std 
        0.0011,            // 加速度零偏随机游走_std
        0.01,              // GPS位置噪声_std (米)
        0.01,              // GPS姿态噪声_std
        0.002,              // IMU陀螺噪声_std (rad/s)
        0.048               // IMU加速度噪声_std (m/s²)
    );
    ros::init(argc, argv, "odometry");
    ros::NodeHandle n;

    g_eskf_odom_puber = n.advertise<nav_msgs::Odometry>("/eskf_odom", 1);
    ros::Subscriber odom_suber = n.subscribe<geometry_msgs::PoseStamped>("/airsim_node/drone_1/gps", 1, odom_local_ned_cb);//状态真值，用于赛道一
    // ros::Subscriber odom_suber = n.subscribe<geometry_msgs::PoseStamped>("/airsim_node/drone_1/debug/pose_gt", 1, odom_local_ned_cb);//状态真值，用于赛道一
    ros::Subscriber debug_suber = n.subscribe<geometry_msgs::PoseStamped>("/airsim_node/drone_1/debug/pose_gt", 1, posegt_cb);
    ros::Subscriber slam_suber = n.subscribe<nav_msgs::Odometry>("aft_mapped_to_init", 1, slam_cb);
    ros::Subscriber imu_suber = n.subscribe<sensor_msgs::Imu>("airsim_node/drone_1/imu/imu", 1, imu_cb);//imu数据odom_local_ned
    ros::Subscriber init_pose_suber = n.subscribe<geometry_msgs::PoseStamped>("/airsim_node/initial_pose", 1, init_pose_ned_cb);

    ros::Rate loop_rate(100);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    delete g_eskf_ptr;
    return 0;
}


void init_pose_ned_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(!g_eskf_ptr->m_isInitailed)
    {
        global_init = *msg;
        Eigen::Quaternion tmpq(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
        Eigen::Matrix4d r0 = Eigen::Matrix4d::Identity();
        r0.block<3,3>(0, 0) = tmpq.toRotationMatrix();
        r0.block<3,1>(0, 3) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        g_eskf_ptr->Init(r0, Eigen::Vector3d::Zero(),msg->header.stamp.toNSec());
        g_eskf_ptr->m_isInitailed = true;
    }
}

void odom_local_ned_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // ROS_INFO("Get odom_local_ned_cd\n  orientation: %f-%f-%f-%f\n  position: %f-%f-%f\n", 
    // msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, //姿态四元数
    // msg->pose.position.x, msg->pose.position.y,msg->pose.position.z);

    // g_eskf_ptr->correct(Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y,msg->pose.position.z), 
    //     Eigen::Quaterniond(msg->pose.orientation.w,msg->pose.orientation.x, msg->pose.orientation.y,msg->pose.orientation.z));
}

void imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    // ROS_INFO("Get imu data.\n %f %f %f \n %f %f %f", msg->angular_velocity.x, msg->angular_velocity.y,
    // msg->angular_velocity.z, msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    if(g_eskf_ptr->m_isInitailed)
    {
        Eigen::Vector3d pos, vel, angle_vel;
        Eigen::Quaterniond q;
        g_eskf_ptr->Predict(Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z), 
            Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z), 
            pos, vel, angle_vel, q, msg->header.stamp.toNSec());
        nav_msgs::Odometry msg2;

        msg2.header.stamp = msg->header.stamp;
        msg2.header.frame_id = "ned";
        msg2.pose.pose.position.x = pos.x();
        msg2.pose.pose.position.y = pos.y();
        msg2.pose.pose.position.z = pos.z();
        msg2.pose.pose.orientation.w = q.w();
        msg2.pose.pose.orientation.x = q.x();
        msg2.pose.pose.orientation.y = q.y();
        msg2.pose.pose.orientation.z = q.z();
        msg2.twist.twist.linear.x = vel.x();
        msg2.twist.twist.linear.y = vel.y();
        msg2.twist.twist.linear.z = vel.z();
        msg2.twist.twist.angular.x = angle_vel.x();
        msg2.twist.twist.angular.y = angle_vel.y();
        msg2.twist.twist.angular.z = angle_vel.z();

        g_eskf_odom_puber.publish(msg2);

        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;      
        transformStamped.header.stamp = msg->header.stamp;
        transformStamped.header.frame_id = "ned";
        transformStamped.child_frame_id = "eskf_out";
        transformStamped.transform.translation.x = pos.x();
        transformStamped.transform.translation.y = pos.y();
        transformStamped.transform.translation.z = pos.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        br.sendTransform(transformStamped);
    }
}

void posegt_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;      
    transformStamped.header.stamp = msg->header.stamp;
    transformStamped.header.frame_id = "ned";
    transformStamped.child_frame_id = "pose_gt";
    transformStamped.transform.translation.x = msg->pose.position.x;
    transformStamped.transform.translation.y = msg->pose.position.y;
    transformStamped.transform.translation.z = msg->pose.position.z;
    transformStamped.transform.rotation.x = msg->pose.orientation.x;
    transformStamped.transform.rotation.y = msg->pose.orientation.y;
    transformStamped.transform.rotation.z = msg->pose.orientation.z;
    transformStamped.transform.rotation.w = msg->pose.orientation.w;
    br.sendTransform(transformStamped);
}

void slam_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    if(!slam_init){
        local_init = *msg;
        slam_init = true;
    }
    else {
        tf::Vector3 t_trans(global_init.pose.position.x - local_init.pose.pose.position.x,
                            global_init.pose.position.y - local_init.pose.pose.position.y,
                            global_init.pose.position.z - local_init.pose.pose.position.z);
        tf::Quaternion q_local_init, q_global_init, q_trans;
        tf::quaternionMsgToTF(global_init.pose.orientation, q_global_init);
        tf::quaternionMsgToTF(local_init.pose.pose.orientation, q_local_init);
        q_trans = q_global_init * q_local_init.inverse();

        tf::Vector3 local_t(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        tf::Quaternion local_q;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, local_q);

        tf::Vector3 global_t = tf::Matrix3x3(q_trans) * local_t + t_trans + tf::Vector3(0, 0, 0.32);
        tf::Quaternion global_q = q_trans* local_q;
                    
        g_eskf_ptr->correct(Eigen::Vector3d(global_t.x(), global_t.y(), global_t.z()), 
        Eigen::Quaterniond(global_q.w(), global_q.x(), global_q.y(), global_q.z()));

        // static tf2_ros::TransformBroadcaster br;
        // geometry_msgs::TransformStamped transformStamped;      
        // transformStamped.header.stamp = msg->header.stamp;
        // transformStamped.header.frame_id = "ned";
        // transformStamped.child_frame_id = "slam_pose";
        // transformStamped.transform.translation.x = global_t.x();
        // transformStamped.transform.translation.y = global_t.y();
        // transformStamped.transform.translation.z = global_t.z();
        // transformStamped.transform.rotation.x = global_q.x();
        // transformStamped.transform.rotation.y = global_q.y();
        // transformStamped.transform.rotation.z = global_q.z();
        // transformStamped.transform.rotation.w = global_q.w();
        // br.sendTransform(transformStamped);
    }
}
