#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <unitree_legged_sdk/unitree_legged_sdk.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <tf/transform_broadcaster.h>
#include <boost/bind.hpp>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>

using namespace UNITREE_LEGGED_SDK;

// Manual conversion from quaternion to rotation matrix
Eigen::Matrix3d quaternionToRotationMatrix(double w, double x, double y, double z) {
    Eigen::Matrix3d R;

    double xx = x * x;
    double yy = y * y;
    double zz = z * z;
    double xy = x * y;
    double xz = x * z;
    double yz = y * z;
    double wx = w * x;
    double wy = w * y;
    double wz = w * z;

    R(0, 0) = 1.0 - 2.0 * (yy + zz);
    R(0, 1) = 2.0 * (xy - wz);
    R(0, 2) = 2.0 * (xz + wy);

    R(1, 0) = 2.0 * (xy + wz);
    R(1, 1) = 1.0 - 2.0 * (xx + zz);
    R(1, 2) = 2.0 * (yz - wx);

    R(2, 0) = 2.0 * (xz - wy);
    R(2, 1) = 2.0 * (yz + wx);
    R(2, 2) = 1.0 - 2.0 * (xx + yy);

    return R;
}

class Custom {
public:
    UDP low_udp;
    UDP high_udp;

    HighState high_state = {0};
    LowCmd low_cmd = {0};
    LowState low_state = {0};

    Custom() : low_udp(LOWLEVEL, 8091, "192.168.123.10", 8007),
               high_udp(8090, "192.168.123.10", 8082, 0, sizeof(HighState)) {
        low_udp.InitCmdData(low_cmd);
    }

    void highUdpRecv() { high_udp.Recv(); high_udp.GetRecv(high_state); }
    void lowUdpRecv() { low_udp.Recv(); low_udp.GetRecv(low_state); }
    void lowUdpSend() { low_udp.SetSend(low_cmd); low_udp.Send(); }
};

Custom custom;

Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
Eigen::Vector3d position = Eigen::Vector3d::Zero();
Eigen::Vector3d accel_prev = Eigen::Vector3d::Zero();
ros::Publisher pub_imu;
ros::Publisher pub_odom;
ros::Publisher pub_joint_state;

void motorCmdCallback(const unitree_legged_msgs::MotorCmd::ConstPtr &msg, int motor_index) {
    if (motor_index < 0 || motor_index >= 12)
        return;

    custom.low_cmd.motorCmd[motor_index].mode = 0x0A;
    custom.low_cmd.motorCmd[motor_index].q = msg->q;
    custom.low_cmd.motorCmd[motor_index].dq = msg->dq;
    custom.low_cmd.motorCmd[motor_index].tau = msg->tau;
    custom.low_cmd.motorCmd[motor_index].Kp = msg->Kp;
    custom.low_cmd.motorCmd[motor_index].Kd = msg->Kd;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "low_high_interface");
    ros::NodeHandle nh;

    std::vector<ros::Subscriber> motor_subs;
    for (int i = 0; i < 12; ++i) {
        std::string topic = "/motor_" + std::to_string(i) + "/cmd";
        ros::Subscriber sub = nh.subscribe<unitree_legged_msgs::MotorCmd>(
            topic, 1, boost::bind(motorCmdCallback, _1, i));
        motor_subs.push_back(sub);
    }

    pub_imu = nh.advertise<sensor_msgs::Imu>("imu", 1);
    pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 1);
    pub_joint_state = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

    tf::TransformBroadcaster odom_broadcaster;

    LoopFunc loop_highRecv("high_udp_recv", 0.002, 3, boost::bind(&Custom::highUdpRecv, &custom));
    LoopFunc loop_lowRecv("low_udp_recv", 0.002, 3, boost::bind(&Custom::lowUdpRecv, &custom));
    LoopFunc loop_lowSend("low_udp_send", 0.002, 3, boost::bind(&Custom::lowUdpSend, &custom));

    loop_highRecv.start();
    loop_lowRecv.start();
    loop_lowSend.start();

    ros::Rate loop_rate(1000);
    ros::Time last_time = ros::Time::now();

    while (ros::ok()) {
        ros::Time current_time = ros::Time::now();

        // Publish IMU
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = current_time;
        imu_msg.header.frame_id = "imu_link";
        imu_msg.orientation.w = custom.low_state.imu.quaternion[0];
        imu_msg.orientation.x = custom.low_state.imu.quaternion[1];
        imu_msg.orientation.y = custom.low_state.imu.quaternion[2];
        imu_msg.orientation.z = custom.low_state.imu.quaternion[3];
        imu_msg.angular_velocity.x = custom.low_state.imu.gyroscope[0];
        imu_msg.angular_velocity.y = custom.low_state.imu.gyroscope[1];
        imu_msg.angular_velocity.z = custom.low_state.imu.gyroscope[2];
        pub_imu.publish(imu_msg);

        // Read acceleration in IMU frame
        Eigen::Vector3d accel_local;
        accel_local << custom.low_state.imu.accelerometer[0],
                       custom.low_state.imu.accelerometer[1],
                       custom.low_state.imu.accelerometer[2];

        imu_msg.linear_acceleration.x = accel_local[0];
        imu_msg.linear_acceleration.y = accel_local[1];
        imu_msg.linear_acceleration.z = accel_local[2];

        // Publish Odometry
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        // Transform position and rotation for odometry
        Eigen::Vector3d position(custom.low_state.imu.accelerometer[0], custom.low_state.imu.accelerometer[1], custom.low_state.imu.accelerometer[2]);
        odom_msg.pose.pose.position.x = position[0];
        odom_msg.pose.pose.position.y = position[1];
        odom_msg.pose.pose.position.z = position[2];
        
        Eigen::Matrix3d rot_matrix = quaternionToRotationMatrix(custom.low_state.imu.quaternion[3], custom.low_state.imu.quaternion[0], custom.low_state.imu.quaternion[1], custom.low_state.imu.quaternion[2]);
        Eigen::Quaterniond rotation(rot_matrix);
        odom_msg.pose.pose.orientation.w = rotation.w();
        odom_msg.pose.pose.orientation.x = rotation.x();
        odom_msg.pose.pose.orientation.y = rotation.y();
        odom_msg.pose.pose.orientation.z = rotation.z();
        
        pub_odom.publish(odom_msg);

        // Publish joint state
        sensor_msgs::JointState joint_msg;
        joint_msg.header.stamp = current_time;
        for (int i = 0; i < 12; i++) {
            joint_msg.name.push_back("motor_" + std::to_string(i));
            joint_msg.position.push_back(custom.low_state.motorState[i].q);
            joint_msg.velocity.push_back(custom.low_state.motorState[i].dq);
        }
        pub_joint_state.publish(joint_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


