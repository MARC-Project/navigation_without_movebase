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
#include <navigation_without_movebase/CvPoints.h>
#include <navigation_without_movebase/CvPoint.h>

#define PI 3.1415926

using namespace std;
  

class Bezier_local_planner
{  
public:  
    Bezier_local_planner()  
    {  
        pub_raw_vel = n_.advertise<geometry_msgs::Twist>("cmd_vel", 100);      
        sub_path = n_.subscribe("/path_points", 100, &Bezier_local_planner::velocity_generator, this);       
    }  
    
    ~Bezier_local_planner(){};

    void velocity_generator (const navigation_without_movebase::CvPoints::ConstPtr& msg);


private:  
    ros::NodeHandle n_;   
    ros::Publisher pub_raw_vel;  
    ros::Subscriber sub_path;   
    navigation_without_movebase::CvPoints points_buf;
      
    }; 



bool circleLeastFit(const std::vector<navigation_without_movebase::CvPoint> &points, double &center_x, double &center_y, double &radius)
{
     center_x = 0.0f;
     center_y = 0.0f;
     radius = 0.0f;
     if (points.size() < 3)
     {
         return false;
     }

     double sum_x = 0.0f, sum_y = 0.0f;
     double sum_x2 = 0.0f, sum_y2 = 0.0f;
     double sum_x3 = 0.0f, sum_y3 = 0.0f;
     double sum_xy = 0.0f, sum_x1y2 = 0.0f, sum_x2y1 = 0.0f;

     int N = points.size();
     for (int i = 0; i < N; i++)
     {
         double x = points[i].x;
         double y = points[i].y;
         double x2 = x * x;
         double y2 = y * y;
         sum_x += x;
         sum_y += y;
         sum_x2 += x2;
         sum_y2 += y2;
         sum_x3 += x2 * x;
         sum_y3 += y2 * y;
         sum_xy += x * y;
         sum_x1y2 += x * y2;
         sum_x2y1 += x2 * y;
     }

     double C, D, E, G, H;
     double a, b, c;

     C = N * sum_x2 - sum_x * sum_x;
     D = N * sum_xy - sum_x * sum_y;
     E = N * sum_x3 + N * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
     G = N * sum_y2 - sum_y * sum_y;
     H = N * sum_x2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;
     a = (H * D - E * G) / (C * G - D * D);
     b = (H * C - E * D) / (D * D - G * C);
     c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N;

     center_x = a / (-2);
     center_y = b / (-2);
     radius = sqrt(a * a + b * b - 4 * c) / 2;
     return true;
}


      
void Bezier_local_planner::velocity_generator (const navigation_without_movebase::CvPoints::ConstPtr& msg){

    if(!msg->points.empty()){
        points_buf.points = msg->points;
        double position_x = msg->position_x;
        double position_z = msg->position_z;
        double target_direction = msg->target_direction;
        vector<navigation_without_movebase::CvPoint> nearPoints;
        for(int i=0;i < 10; i++){
            nearPoints.push_back(msg->points[i]);
        }
        double a,b,r;
        circleLeastFit(nearPoints, a, b, r);
        ROS_INFO("Radius is: %f", r);
        geometry_msgs::Twist cmd_vel;
        if(position_z <= 1.0 && fabs(position_x) <= 0.05) {
            cmd_vel.linear.x = 0.5;
            cmd_vel.angular.z = 0;
        }
        else if(r >= 0.5 && position_x < 0){
            cmd_vel.linear.x = 0.2;
            cmd_vel.angular.z = 0.2 / r;
        }
        else if(r >= 0.5 && position_x >= 0){
            cmd_vel.linear.x = 0.2;
            cmd_vel.angular.z = -0.2 / r;
        }
        else if(r < 0.5 && position_x < 0) {
            cmd_vel.linear.x = 0.08;
            cmd_vel.angular.z = 0.08 / r;
        }
        else if(r < 0.5 && position_x >= 0) {
            cmd_vel.linear.x = 0.08;
            cmd_vel.angular.z = -0.08 / r;
        }
        pub_raw_vel.publish(cmd_vel);
    }
}







int main( int argc, char** argv)
{
    ros::init(argc, argv, "bezier_local_planner");
    
    Bezier_local_planner local_planner = Bezier_local_planner();  

    ros::Rate loop_rate(10);
    ros::spin();
    return 0;
}