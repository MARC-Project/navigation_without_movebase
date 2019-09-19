/*
接收二维码识别的话题消息，解析发送话题中的位置和姿态四元数信息，控制底盘机器人运动，跟踪二维码
*/
#include <string>
#include <vector>
#include <sstream>
#include <ros/ros.h>
#include <ros/types.h>
#include <ros/message_operations.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Quaternion.h>   // Deal with quaternion info
#include <geometry_msgs/Twist.h>   //运动信息发布
#include <nav_msgs/Odometry.h>    //里程计信息订阅
#include <cmath>

#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>   //二维码信息订阅
#include <tf/tf.h>  
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#define PI 3.14159265

using namespace std;

/*
 * Function: bezier_curve_generator
 * 
 
 */
void bezier_curve_generator(const ar_track_alvar_msgs::AlvarMarkers req)
{
     // 将接收到的消息打印出来
    if (!req.markers.empty())
    {
    /* TODO: rank the distance if seeing multiple ar markers */

    //二维码相对camera的位置关系
    // Note that z axis positive direction is the camera's facing direction,
    // and x axis positive direction is the camera's right hand side.
    ROS_INFO("I heard AR_marker Depth_z and Horizontal_offset_x :");
    float position_z=req.markers[0].pose.pose.position.z; //深度信息
    float position_x=req.markers[0].pose.pose.position.x; //水平偏移量
    ROS_INFO("I heard position z: [%f]",position_z);
    ROS_INFO("I heard position x: [%f]",position_x);


    //二维码相对camera的姿态关系四元数
    //ROS_INFO("I heard AR_marker Quaternion:");
    float quaternion_x=req.markers[0].pose.pose.orientation.x;
    float quaternion_y=req.markers[0].pose.pose.orientation.y;
    float quaternion_z=req.markers[0].pose.pose.orientation.z;
    float quaternion_w=req.markers[0].pose.pose.orientation.w;
    tf::Quaternion q (
    req.markers[0].pose.pose.orientation.x,
    req.markers[0].pose.pose.orientation.y,
    req.markers[0].pose.pose.orientation.z,
    req.markers[0].pose.pose.orientation.w);
    //ROS_INFO("I heard ar_quaternion_x: [%f]",q.x);
    //ROS_INFO("I heard ar_quaternion_y: [%f]",q.y);
    //ROS_INFO("I heard ar_quaternion_z: [%f]",q.z);
    //ROS_INFO("I heard ar_quaternion_w: [%f]",q.w);
    

    // changing quaternion into RPY angle
    //ROS_INFO("I heard RPY:");
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    //ROS_INFO("roll, pitch, yaw=%.2f degree  %.2f degree  %.2f degree" , roll*180/PI, pitch*180/PI, yaw*180/PI);
    //transfer RPY into useful target direction
    double target_direction = ((roll*180/PI) < 0) ? (-90 - roll*180/PI) : (270 - roll*180/PI) ;
    ROS_INFO("I heard target direction: [%f] degree", target_direction);

    /* TODO: create bezier curve according to x, z and target direction.*/
    geometry_msgs::Twist cmdvel_;
    }

}





int main(int argc, char **argv)
{
  // 初始化ROS节点
  ros::init(argc, argv, "spark_ar_marker");
  // 创建节点句柄
  ros::NodeHandle nhandle;
  ros::Subscriber arpose_sub;
  arpose_sub =nhandle.subscribe("ar_pose_marker", 1000, bezier_curve_generator);
    ros::Rate loop_rate(10);
  ros::spin();
  return 0;
}