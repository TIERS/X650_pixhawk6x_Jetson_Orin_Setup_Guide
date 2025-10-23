#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <cmath>

using namespace std;

class vision_pose
{
public:
    vision_pose(const ros::NodeHandle &nh_, const ros::NodeHandle &nh_private_);
    void start();

private:
    // 回调：PX4 本地位置（保留以便对比打印）
    void px4Pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);

    // 从 tf 转 PoseStamped 并更新 estimatedPose/estimatedAttitude
    bool update_from_tf();

    static double angDiffDeg(double a_deg, double b_deg)
    {
        // 计算 a - b 并归一化到 [-180, 180)
        double d = a_deg - b_deg;
        d = std::fmod(d + 180.0, 360.0);
        if (d < 0) d += 360.0;
        return d - 180.0;
    }

private:
    struct attitude
    {
        double pitch{0.0};
        double roll{0.0};
        double yaw{0.0};
    };

    double pi{3.1415926};

    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    ros::Subscriber px4Pose_sub;
    ros::Publisher  vision_pose_pub;

    ros::Rate *rate{nullptr};

    attitude estimatedAttitude;
    attitude px4Attitude;

    geometry_msgs::PoseStamped px4Pose;
    geometry_msgs::PoseStamped estimatedPose;

    bool estimatedOdomRec_flag{false};

    // === tf 相关 ===
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener; // 注意初始化顺序要在构造函数初始化列表里
    std::string parent_frame{"odom"};
    std::string child_frame{"base_link"};
};

vision_pose::vision_pose(const ros::NodeHandle &nh_, const ros::NodeHandle &nh_private_)
    : nh(nh_), nh_private(nh_private_), tfListener(tfBuffer)
{
    rate = new ros::Rate(30);

    // 读取可配置的 tf 帧名
    nh_private.param<std::string>("parent_frame", parent_frame, "odom");
    nh_private.param<std::string>("child_frame",  child_frame,  "base_link");

    // 订阅 PX4 本地位置用于对比打印（可按需移除）
    px4Pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "mavros/local_position/pose", 10, &vision_pose::px4Pose_cb, this);

    // 发布基于 tf 的视觉位姿
    vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10);
}

void vision_pose::px4Pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    px4Pose = *msg;

    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    px4Attitude.pitch = pitch * 180.0 / pi;
    px4Attitude.roll  = roll  * 180.0 / pi;
    px4Attitude.yaw   = yaw   * 180.0 / pi;
}

bool vision_pose::update_from_tf()
{
    try
    {
        // 从 parent_frame 到 child_frame 的最新变换
        geometry_msgs::TransformStamped tf_msg =
            tfBuffer.lookupTransform(parent_frame, child_frame, ros::Time(0));

        // TransformStamped -> PoseStamped
        estimatedPose.header = tf_msg.header;
        estimatedPose.pose.position.x = tf_msg.transform.translation.x;
        estimatedPose.pose.position.y = tf_msg.transform.translation.y;
        estimatedPose.pose.position.z = tf_msg.transform.translation.z;
        estimatedPose.pose.orientation = tf_msg.transform.rotation;

        // 计算 RPY 以便打印
        tf2::Quaternion q;
        tf2::fromMsg(estimatedPose.pose.orientation, q);
        double r, p, y;
        tf2::Matrix3x3(q).getRPY(r, p, y);
        estimatedAttitude.pitch = p * 180.0 / pi;
        estimatedAttitude.roll  = r * 180.0 / pi;
        estimatedAttitude.yaw   = y * 180.0 / pi;

        estimatedOdomRec_flag = true;
        return true;
    }
    catch (const tf2::TransformException &ex)
    {
        // 第一次可能还没等到 tf，打印一次即可
        static bool warned = false;
        if (!warned)
        {
            ROS_WARN_STREAM("Waiting for TF [" << parent_frame << " -> " << child_frame << "]: " << ex.what());
            warned = true;
        }
        return false;
    }
}

void vision_pose::start()
{
    while (ros::ok())
    {
        ros::spinOnce();

        // 尝试从 tf 更新
        bool ok = update_from_tf();

        if (!ok || !estimatedOdomRec_flag)
        {
            cout << "\033[K\033[31m TF -> Pose no receive!!! \033[0m" << endl;
        }
        else
        {
            // 发布
            estimatedPose.header.stamp = ros::Time::now();
            estimatedPose.header.frame_id = parent_frame;
            vision_pose_pub.publish(estimatedPose);

            // 计算每时刻的差值（estimated - px4）
            double dx   = estimatedPose.pose.position.x - px4Pose.pose.position.x;
            double dy   = estimatedPose.pose.position.y - px4Pose.pose.position.y;
            double dz   = estimatedPose.pose.position.z - px4Pose.pose.position.z;
            double dpit = angDiffDeg(estimatedAttitude.pitch, px4Attitude.pitch);
            double drol = angDiffDeg(estimatedAttitude.roll,  px4Attitude.roll);
            double dyaw = angDiffDeg(estimatedAttitude.yaw,   px4Attitude.yaw);

            // 打印（依然保持 9 行）
            cout << "\033[K\033[32m tf ok !\033[0m" << endl;
            cout << "\033[K" << "       estimatedPose      px4Pose              diff(estim - px4)" << endl;

            cout << setiosflags(ios::fixed) << setprecision(7)
                << "\033[K" << "x      " << estimatedPose.pose.position.x << "\t\t"
                << px4Pose.pose.position.x << "\t\t" << dx << endl;

            cout << setiosflags(ios::fixed) << setprecision(7)
                << "\033[K" << "y      " << estimatedPose.pose.position.y << "\t\t"
                << px4Pose.pose.position.y << "\t\t" << dy << endl;

            cout << setiosflags(ios::fixed) << setprecision(7)
                << "\033[K" << "z      " << estimatedPose.pose.position.z << "\t\t"
                << px4Pose.pose.position.z << "\t\t" << dz << endl;

            cout << setiosflags(ios::fixed) << setprecision(7)
                << "\033[K" << "pitch  " << estimatedAttitude.pitch << "\t\t"
                << px4Attitude.pitch << "\t\t" << dpit << endl;

            cout << setiosflags(ios::fixed) << setprecision(7)
                << "\033[K" << "roll   " << estimatedAttitude.roll  << "\t\t"
                << px4Attitude.roll  << "\t\t" << drol << endl;

            cout << setiosflags(ios::fixed) << setprecision(7)
                << "\033[K" << "yaw    " << estimatedAttitude.yaw   << "\t\t"
                << px4Attitude.yaw   << "\t\t" << dyaw << endl;

            cout << "\033[9A" << endl;  // 维持原来的回滚行数

        }

        rate->sleep();
    }
    cout << "\033[9B" << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_to_mavros");
    ros::NodeHandle nh_("");
    ros::NodeHandle nh_private_("~");

    vision_pose node(nh_, nh_private_);
    node.start();
    return 0;
}