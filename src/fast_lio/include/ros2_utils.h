#ifndef FAST_LIO_ROS2_UTILS_H
#define FAST_LIO_ROS2_UTILS_H

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <memory>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
using PathMsg = nav_msgs::msg::Path;
using OdometryMsg = nav_msgs::msg::Odometry;
using OdomMsg = nav_msgs::msg::Odometry;
using PointMsg = geometry_msgs::msg::Point;
using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
using ImuMsg = sensor_msgs::msg::Imu;
using StringMsg = std_msgs::msg::String;
using PathPublisher = rclcpp::Publisher<PathMsg>::SharedPtr;
using OdomPublisher = rclcpp::Publisher<OdometryMsg>::SharedPtr;
using Pcl2Publisher = rclcpp::Publisher<PointCloud2Msg>::SharedPtr;
using TimeType = rclcpp::Time;
using RateType = rclcpp::Rate;
using Pcl2Msg = sensor_msgs::msg::PointCloud2;
using QuaternionMsg = geometry_msgs::msg::Quaternion;
using PoseStampedMsgConstPtr = PoseStampedMsg::ConstSharedPtr;
using ImuMsgConstPtr = ImuMsg::ConstSharedPtr;
using ImuMsgPtr = ImuMsg::SharedPtr;
using LivoxCustomMsg = livox_ros_driver2::msg::CustomMsg;
using LivoxCustomMsgConstPtr = LivoxCustomMsg::ConstSharedPtr;
using Pcl2MsgConstPtr = Pcl2Msg::ConstSharedPtr;
using LivoxMsg = LivoxCustomMsg;

inline rclcpp::Node::SharedPtr &get_ros_node()
{
    static rclcpp::Node::SharedPtr instance;
    return instance;
}

inline void init_ros_node(const rclcpp::Node::SharedPtr &node)
{
    get_ros_node() = node;
}

inline bool ros_ok()
{
    return rclcpp::ok();
}

inline void spin_once()
{
    if (get_ros_node()) {
        rclcpp::spin_some(get_ros_node());
    }
}

inline void ROS_PRINT_WARN(const char *format, ...)
{
    char message[1024];
    va_list args;
    va_start(args, format);
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    RCLCPP_WARN(rclcpp::get_logger("fast_lio"), "%s", message);
}

inline void ROS_PRINT_ERROR(const char *format, ...)
{
    char message[1024];
    va_list args;
    va_start(args, format);
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    RCLCPP_ERROR(rclcpp::get_logger("fast_lio"), "%s", message);
}

inline void ROS_PRINT_INFO(const char *format, ...)
{
    char message[1024];
    va_list args;
    va_start(args, format);
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    RCLCPP_INFO(rclcpp::get_logger("fast_lio"), "%s", message);
}

inline QuaternionMsg quaternion_from_rpy(double roll, double pitch, double yaw)
{
    tf2::Quaternion tf_q;
    tf_q.setRPY(roll, pitch, yaw);
    QuaternionMsg q;
    q.x = tf_q.x();
    q.y = tf_q.y();
    q.z = tf_q.z();
    q.w = tf_q.w();
    return q;
}

inline rclcpp::Time get_ros_time(double stamp_sec)
{
    const int32_t sec = static_cast<int32_t>(std::floor(stamp_sec));
    const uint32_t nanosec = static_cast<uint32_t>((stamp_sec - sec) * 1e9);
    return rclcpp::Time(sec, nanosec);
}

inline double get_ros_time_sec(const builtin_interfaces::msg::Time &stamp)
{
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

inline rclcpp::Time get_ros_now()
{
    return get_ros_node()->get_clock()->now();
}

template<typename PubType, typename MsgType>
inline void ros_publish(const PubType &publisher, const MsgType &message)
{
    publisher->publish(message);
}

template<typename T>
inline void rosparam_get(const std::string &name, T &value, const T &default_value)
{
    get_ros_node()->declare_parameter<T>(name, default_value);
    value = get_ros_node()->get_parameter(name).get_value<T>();
}

template<>
inline void rosparam_get<float>(const std::string &name, float &value, const float &default_value)
{
    get_ros_node()->declare_parameter<double>(name, static_cast<double>(default_value));
    value = static_cast<float>(get_ros_node()->get_parameter(name).as_double());
}

template<typename T, typename Callback>
inline typename rclcpp::Subscription<T>::SharedPtr create_subscriber(
    const std::string &topic, uint32_t queue_size, Callback callback)
{
    return get_ros_node()->create_subscription<T>(
        topic, rclcpp::QoS(rclcpp::KeepLast(queue_size)), callback);
}

template<typename T, typename Callback>
inline typename rclcpp::Subscription<T>::SharedPtr create_subscriber_qos(
    const std::string &topic, const rclcpp::QoS &qos, Callback callback)
{
    return get_ros_node()->create_subscription<T>(topic, qos, callback);
}

template<typename T>
inline typename rclcpp::Publisher<T>::SharedPtr create_publisher(
    const std::string &topic, uint32_t queue_size)
{
    return get_ros_node()->create_publisher<T>(
        topic, rclcpp::QoS(rclcpp::KeepLast(queue_size)));
}

template<typename T>
inline typename rclcpp::Publisher<T>::SharedPtr create_publisher_qos(
    const std::string &topic, const rclcpp::QoS &qos)
{
    return get_ros_node()->create_publisher<T>(topic, qos);
}

#endif
