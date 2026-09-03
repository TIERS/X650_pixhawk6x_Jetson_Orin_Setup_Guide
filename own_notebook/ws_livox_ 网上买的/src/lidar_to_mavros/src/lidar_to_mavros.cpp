#include <ros/ros.h> // ROS 头文件
#include <geometry_msgs/PoseStamped.h> // ROS 中用于传递坐标位置信息的消息类型
#include <nav_msgs/Odometry.h> // ROS 中用于传递里程计信息的消息类型
#include <iostream> // C++ 标准输入输出流
#include <tf/transform_datatypes.h> // ROS 中用于处理转换和旋转的数据类型
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // ROS 中用于将消息转换为几何数据类型的工具函数

using namespace std; // 使用标准命名空间

// 定义一个类
class vision_pose
{
public:
    // 构造函数
    vision_pose(const ros::NodeHandle &nh_, const ros::NodeHandle &nh_private_);
    double pi;
    // 定义一个结构体，用于存储姿态信息
    struct attitude
    {
        double pitch;
        double roll;
        double yaw;
    };

    attitude estimatedAttitude; // 存储估计的姿态信息
    attitude px4Attitude; // 存储 PX4 发送的姿态信息

    geometry_msgs::PoseStamped px4Pose; // 存储 PX4 发送的位置信息
    geometry_msgs::PoseStamped estimatedPose; // 存储估计的位置信息

    bool estimatedOdomRec_flag; // 用于标记是否已经接收到里程计信息
    bool firstPoseReceived; // 用于标记是否已经接收到第一次传入的坐标

    ros::Rate *rate; // ROS 定时器

    ros::NodeHandle nh; // ROS 节点句柄
    ros::NodeHandle nh_private; // 私有命名空间的 ROS 节点句柄

    ros::Subscriber px4Pose_sub; // PX4 位置信息的订阅者
    ros::Publisher vision_pose_pub; // 发布估计的位置信息
    ros::Subscriber odom_sub; // 订阅里程计信息

    // 回调函数，处理 PX4 发送的位置信息
    void px4Pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);

    // 回调函数，处理里程计信息
    void estimator_odom_cb(const nav_msgs::Odometry::ConstPtr &msg);

    // 开始执行主循环
    void start();
};

/* 构造函数 */
vision_pose::vision_pose(const ros::NodeHandle &nh_, const ros::NodeHandle &nh_private_) : nh(nh_), nh_private(nh_private_)
{
    // 初始化 pi 常数
    pi = 3.1415926;
    
    // 设置 ROS 定时器
    rate = new ros::Rate(30);

    // 订阅 PX4 发送的位置信息
    px4Pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &vision_pose::px4Pose_cb, this);
    
    // 订阅里程计信息
    odom_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 2, &vision_pose::estimator_odom_cb, this);
    
    // 发布估计的位置信息
    vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10);

    // 初始化标志变量
    estimatedOdomRec_flag = false;
    firstPoseReceived = false;

    // 初始化姿态信息
    estimatedAttitude.pitch = 0;
    estimatedAttitude.roll = 0;
    estimatedAttitude.yaw = 0;

    px4Attitude.pitch = 0;
    px4Attitude.roll = 0;
    px4Attitude.yaw = 0;
}

// 处理里程计信息的回调函数
void vision_pose::estimator_odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    // 如果是第一次接收到坐标，将其标记为零漂
    if (!firstPoseReceived)
    {
        estimatedPose.pose = msg->pose.pose;
        firstPoseReceived = true;
    }
    else
    {
        estimatedPose.pose = msg->pose.pose; // 复制姿态
    }

    // 将四元数转换为滚动-俯仰-偏航角度
    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    estimatedAttitude.pitch = pitch * 180 / pi;
    estimatedAttitude.roll = roll * 180 / pi;
    estimatedAttitude.yaw = yaw * 180 / pi;

    estimatedOdomRec_flag = true;
}

// 处理 PX4 发送的位置信息的回调函数
void vision_pose::px4Pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // 复制 PX4 发送的位置信息
    px4Pose.pose = msg->pose;

    // 将四元数转换为滚动-俯仰-偏航角度
    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    px4Attitude.pitch = pitch * 180 / pi;
    px4Attitude.roll = roll * 180 / pi;
    px4Attitude.yaw = yaw * 180 / pi;
}

// 主循环函数
void vision_pose::start()
{
    while (ros::ok())
    {
        // 如果没有接收到里程计信息，则输出提示
        if (estimatedOdomRec_flag == false)
        {
            cout << "\033[K"
                 << "\033[31m estimatedPose no receive!!! \033[0m" << endl;
        }
        else
        {
            // 发布估计的位置信息
            estimatedPose.header.stamp = ros::Time::now();
            vision_pose_pub.publish(estimatedPose);

            // 输出信息到控制台
            cout << "\033[K"
                 << "\033[32m vrpn ok !\033[0m" << endl;
            cout << "\033[K"
                 << "       estimatedPose                  px4Pose" << endl;
            cout << setiosflags(ios::fixed) << setprecision(7)
                 << "\033[K"
                 << "x      " << estimatedPose.pose.position.x << "\t\t" << px4Pose.pose.position.x << endl;
            cout << setiosflags(ios::fixed) << setprecision(7)
                 << "\033[K"
                 << "y      " << estimatedPose.pose.position.y << "\t\t" << px4Pose.pose.position.y << endl;
            cout << setiosflags(ios::fixed) << setprecision(7)
                 << "\033[K"
                 << "z      " << estimatedPose.pose.position.z << "\t\t" << px4Pose.pose.position.z << endl;
            cout << setiosflags(ios::fixed) << setprecision(7)
                 << "\033[K"
                 << "pitch  " << estimatedAttitude.pitch << "\t\t" << px4Attitude.pitch << endl;
            cout << setiosflags(ios::fixed) << setprecision(7)
                 << "\033[K"
                 << "roll   " << estimatedAttitude.roll << "\t\t" << px4Attitude.roll << endl;
            cout << setiosflags(ios::fixed) << setprecision(7)
                 << "\033[K"
                 << "yaw    " << estimatedAttitude.yaw << "\t\t" << px4Attitude.yaw << endl;
            cout << "\033[9A" << endl;
        }
        ros::spinOnce();
        rate->sleep();
    }
    cout << "\033[9B" << endl;
}

// 主函数
int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "lidar_to_mavros");
    ros::NodeHandle nh_("");
    ros::NodeHandle nh_private_("~");

    // 创建 vision_pose 对象并执行主循环
    vision_pose vision(nh_, nh_private_);
    vision.start();

    return 0;
}
