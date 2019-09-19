#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#include <sensor_msgs/LaserScan.h>
#include <navigation_without_movebase/suduxifen.h>


#define PI 3.1415926





using namespace std;
  





class SubscribeAndPublish  
{  
public:  
    SubscribeAndPublish()  
    {  
        pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);      
        sub_state_ = n_.subscribe("/cmdvel_control", 1, &SubscribeAndPublish::callback_state, this);       
    }  
    

    void callback_state (const navigation_without_movebase::suduxifen::ConstPtr& msg)
    {

      if (msg->ar == 1)
      {
        geometry_msgs::Twist cmd_;

        if (msg->straight == 1)
        {
        /*
            测试 使其停止 判断开始执行开环控制的位置
        */
          //  ros::Duration(50).sleep();
          if (msg->straight_direction == 0)
          {
              cmd_.linear.x = 0.5;
              for(int i=0;i<58;i++)
              {
                  pub_.publish(cmd_);
                  ros::Duration(0.05).sleep();
              }
          }else // z正左转 z负右转 整体左转先右转后左转 整体右转先左转后右转
          {
              //先反向给预量
              cmd_.linear.x = 0.3;
              cmd_.angular.z = -msg->straight_direction;
              for(int i=0;i<10;i++)
              {
                  pub_.publish(cmd_);
                  ros::Duration(0.05).sleep();
              }
              //中间过渡
              cmd_.linear.x = 0.3;
              cmd_.angular.z = 0;
              for(int i=0;i<3;i++)
              {
                  pub_.publish(cmd_);
                  ros::Duration(0.05).sleep();
              }
              //再正向校准
              cmd_.linear.x = 0.3;
              cmd_.angular.z = msg->straight_direction;
              for(int i=0;i<25;i++)
              {
                  pub_.publish(cmd_);
                  ros::Duration(0.05).sleep();
              }
              //最后直行通过
              cmd_.linear.x = 0.5;
              cmd_.angular.z = 0;
              for(int i=0;i<40;i++)
              {
                  pub_.publish(cmd_);
                  ros::Duration(0.05).sleep();
              }
          }
            

        }else if (msg->straight == 0
        &&
        msg->forward == 1)
        {
            cmd_.linear.x = 0.5;
            pub_.publish(cmd_);
            ROS_INFO ("Moving forward.");
        }else if (msg->straight == 0
        &&
        msg->forward == 0
        )
        {
            cmd_.linear.x = 0.1;
            cmd_.angular.z = msg->left;

            pub_.publish (cmd_);
        }
      }else
      {
            geometry_msgs::Twist cmd_;
            int direction = 0;
            ros::Time time_ = ros::Time::now();

            if ( time_.sec&1 )
            {
                direction = 1;
                ROS_INFO("Finding AR, turn left.");
            }else
            {
                direction = -1;
                ROS_INFO("Finding AR, turn right.");
            }

            cmd_.linear.x = 0.05;


            cmd_.angular.z = 0.1*direction;
            pub_.publish (cmd_);
      }

    }




    private:  
      ros::NodeHandle n_;   
      ros::Publisher pub_;  
      ros::Subscriber sub_state_;   
      
    }; 
      






int main( int argc, char** argv)
{
    ros::init(argc, argv, "odom_point_node");
    
    SubscribeAndPublish SAPObject;  
   
    ros::spin();
    return 0;
}