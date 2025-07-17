#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <chrono>
#include <pthread.h>
#include <boost/bind.hpp>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>

using namespace UNITREE_LEGGED_SDK;

class Custom {
public:
    UDP low_udp;
    UDP high_udp;

    HighCmd high_cmd = {0};
    HighState high_state = {0};

    LowCmd low_cmd = {0};
    LowState low_state = {0};

public:
    Custom() : low_udp(LOWLEVEL, 8091, "192.168.12.1", 8007),
               high_udp(8090, "192.168.12.1", 8082, sizeof(HighCmd), sizeof(HighState))
    {
        high_udp.InitCmdData(high_cmd);
        low_udp.InitCmdData(low_cmd);
    }

    void highUdpSend() {
        high_udp.SetSend(high_cmd);
        high_udp.Send();
    }

    void lowUdpSend() {
        low_udp.SetSend(low_cmd);
        low_udp.Send();
    }

    void lowUdpRecv() {
        low_udp.Recv();
        low_udp.GetRecv(low_state);
    }

    void highUdpRecv() {
        high_udp.Recv();
        high_udp.GetRecv(high_state);
    }
};

Custom custom;

Eigen::Vector3d vel_est = Eigen::Vector3d::Zero();
Eigen::Vector3d pos_est = Eigen::Vector3d::Zero();
ros::Time last_time;
ros::Subscriber sub_cmd_vel;
ros::Publisher pub_imu;
ros::Publisher pub_odom;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    custom.high_cmd = rosMsg2Cmd(msg);
    custom.high_cmd.velocity[0] = msg->linear.x;
    custom.high_cmd.velocity[1] = msg->linear.y;
    custom.high_cmd.yawSpeed = msg->angular.z;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "twist_sub");
    ros::NodeHandle nh;

    pub_imu = nh.advertise<sensor_msgs::Imu>("imu", 1);
    pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 1);
    sub_cmd_vel = nh.subscribe("cmd_vel", 1, cmdVelCallback);
    last_time = ros::Time::now();

    tf::TransformBroadcaster odom_broadcaster;
    LoopFunc loop_udpSend("high_udp_send", 0.002, 3, boost::bind(&Custom::highUdpSend, &custom));
    LoopFunc loop_udpRecv("high_udp_recv", 0.002, 3, boost::bind(&Custom::highUdpRecv, &custom));

    loop_udpSend.start();
    loop_udpRecv.start();

    ros::Rate loop_rate(1000);
    long count = 0;

    while (ros::ok()) {
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        if (dt <= 0.0 || !std::isfinite(dt)) continue;
        last_time = current_time;

        sensor_msgs::Imu msg_imu;
        msg_imu.header.seq = count;
        msg_imu.header.stamp = current_time;
        msg_imu.header.frame_id = "imu_link";

        msg_imu.orientation.w = custom.high_state.imu.quaternion[0];
        msg_imu.orientation.x = custom.high_state.imu.quaternion[1];
        msg_imu.orientation.y = custom.high_state.imu.quaternion[2];
        msg_imu.orientation.z = custom.high_state.imu.quaternion[3];

        tf::Quaternion q(
            msg_imu.orientation.x,
            msg_imu.orientation.y,
            msg_imu.orientation.z,
            msg_imu.orientation.w);
        q.normalize();

        if (!std::isfinite(q.x()) || !std::isfinite(q.y()) || !std::isfinite(q.z()) || !std::isfinite(q.w())) {
            ROS_WARN("Invalid quaternion detected, skipping loop.");
            continue;
        }

        double roll, pitch, yaw;
        tf::Matrix3x3 mat(q);
        mat.getRPY(roll, pitch, yaw);
        printf("RPY (deg): Roll=%.2f, Pitch=%.2f, Yaw=%.2f\n",
               roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);

        msg_imu.angular_velocity.x = custom.high_state.imu.gyroscope[0];
        msg_imu.angular_velocity.y = custom.high_state.imu.gyroscope[1];
        msg_imu.angular_velocity.z = custom.high_state.imu.gyroscope[2];

        msg_imu.linear_acceleration.x = custom.high_state.imu.accelerometer[0];
        msg_imu.linear_acceleration.y = custom.high_state.imu.accelerometer[1];
        msg_imu.linear_acceleration.z = custom.high_state.imu.accelerometer[2];

        pub_imu.publish(msg_imu);

        Eigen::Vector3d acc_local(
            msg_imu.linear_acceleration.x,
            msg_imu.linear_acceleration.y,
            msg_imu.linear_acceleration.z);

        if (!acc_local.allFinite()) {
            ROS_WARN("Invalid acceleration detected, skipping loop.");
            continue;
        }

        Eigen::Matrix3d R;
        R << mat.getRow(0).x(), mat.getRow(0).y(), mat.getRow(0).z(),
             mat.getRow(1).x(), mat.getRow(1).y(), mat.getRow(1).z(),
             mat.getRow(2).x(), mat.getRow(2).y(), mat.getRow(2).z();

        Eigen::Vector3d acc_world = R * acc_local;
        acc_world -= Eigen::Vector3d(0.0, 0.0, 9.81);

        for (int i = 0; i < 3; ++i) {
            if (std::abs(acc_world[i]) < 0.02)
                acc_world[i] = 0.0;
        }

        bool is_stationary = acc_world.norm() < 0.5 &&
            std::abs(custom.high_state.imu.gyroscope[0]) < 0.1 &&
            std::abs(custom.high_state.imu.gyroscope[1]) < 0.1 &&
            std::abs(custom.high_state.imu.gyroscope[2]) < 0.1;

        if (is_stationary) {
            vel_est = Eigen::Vector3d::Zero();
        } else {
            Eigen::Vector3d k1_v = acc_world;
            Eigen::Vector3d k2_v = acc_world;
            Eigen::Vector3d k3_v = acc_world;
            Eigen::Vector3d k4_v = acc_world;

            Eigen::Vector3d vel_next = vel_est + (dt / 6.0) * (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v);
            Eigen::Vector3d k1_p = vel_est;
            Eigen::Vector3d k2_p = vel_est + 0.5 * dt * k1_v;
            Eigen::Vector3d k3_p = vel_est + 0.5 * dt * k2_v;
            Eigen::Vector3d k4_p = vel_est + dt * k3_v;

            Eigen::Vector3d pos_next = pos_est + (dt / 6.0) * (k1_p + 2.0 * k2_p + 2.0 * k3_p + k4_p);

            vel_est = vel_next;
            pos_est = pos_next;
        }

        printf("Estimated Position: x=%.3f, y=%.3f, z=%.3f\n", pos_est[0], pos_est[1], pos_est[2]);

        nav_msgs::Odometry msg_odom;
        msg_odom.header.seq = count;
        msg_odom.header.stamp = current_time;
        msg_odom.header.frame_id = "odom";
        msg_odom.child_frame_id = "base_link";

        msg_odom.pose.pose.position.x = custom.high_state.position[0];
        msg_odom.pose.pose.position.y = custom.high_state.position[1];
        msg_odom.pose.pose.position.z = custom.high_state.position[2];
        msg_odom.pose.pose.orientation = msg_imu.orientation;
        msg_odom.twist.twist.linear.x = vel_est[0];
        msg_odom.twist.twist.linear.y = vel_est[1];
        msg_odom.twist.twist.linear.z = vel_est[2];

        std::cout << "Level flag: " << std::hex << int(custom.high_state.levelFlag) << std::endl;


        pub_odom.publish(msg_odom);

        geometry_msgs::TransformStamped odom_tf;
        odom_tf.header.stamp = current_time;
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_link";
        odom_tf.transform.translation.x = pos_est[0];
        odom_tf.transform.translation.y = pos_est[1];
        odom_tf.transform.translation.z = pos_est[2];
        odom_tf.transform.rotation = msg_imu.orientation;

        odom_broadcaster.sendTransform(odom_tf);

        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }

    return 0;
}
