/*
 * Node bezier_path: receive topic of QR code detection and obtain correct pose info. 
 *Output path as a series of two-dimensional coordinates.
 */
#include <string>
#include <vector>
#include <sstream>
#include <ros/ros.h>
#include <ros/types.h>
#include <ros/message_operations.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Quaternion.h>   
#include <geometry_msgs/Twist.h>  
#include <nav_msgs/Odometry.h>    
#include <cmath>

#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>   // QR code message
#include <tf/tf.h>  
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <navigation_without_movebase/CvPoints.h>

//#include "stdafx.h"
#include <stdio.h>
#include <iostream>
#define NUM_STEPS 100 // number of dots fitting the actual curve.
#define PI 3.14159265

using namespace std;

/*
 * Class: CvPoint
 *-------------------
 * A class that is used to represent a point (x,y). Notice that y direction is actually the
 * z direction of camera.
 */
class CvPoint
{
public:
    double x;
    double y;
    CvPoint()
    {
      x=0.0;
      y=0.0;
    }
    CvPoint(double a,double b)
    {
      x=a;
      y=b;
    }
 
}; 

/* Class: Bezier_global_planner
 * -----------------------------
 * This class is the node. It receives ar pose messages and publishes an array containing points
 * on the path.
 */

class Bezier_global_planner
{

public:
    Bezier_global_planner()  
    {  
        pub_path_dots = n_.advertise<navigation_without_movebase::CvPoints>("path_points", 1000);      
        sub_ar_marker = n_.subscribe("ar_pose_marker", 100, &Bezier_global_planner::bezier_curve_generator,this);       
    }  

    ~Bezier_global_planner(){};


    void bezier_curve_generator(const ar_track_alvar_msgs::AlvarMarkers req);

private:  
    ros::NodeHandle n_;   
    ros::Publisher pub_path_dots;  
    ros::Subscriber sub_ar_marker;   

};

/* 
 * Function: curve4(vector<CvPoint> &p,
    double x1, double y1,  
    double x2, double y2,   
    double x3, double y3,   
    double x4, double y4)
--------------------------------------------
    The core function that receive 4 points and create a vector of points of a cubic-bezier curve.
    Note that (x1,y1) and (x4,y4) are anchor points.
 */
void curve4(vector<CvPoint> &p,  
    double x1, double y1,   //Anchor1  
    double x2, double y2,   //Control1  
    double x3, double y3,   //Control2  
    double x4, double y4)   //Anchor2  
{  
    CvPoint tmp0(x1,y1);
    p.push_back(tmp0); 
    double dx1 = x2 - x1;  
    double dy1 = y2 - y1;  
    double dx2 = x3 - x2;  
    double dy2 = y3 - y2;  
    double dx3 = x4 - x3;  
    double dy3 = y4 - y3;  

    double subdiv_step  = 1.0 / (NUM_STEPS + 1);  
    double subdiv_step2 = subdiv_step*subdiv_step;  
    double subdiv_step3 = subdiv_step*subdiv_step*subdiv_step;  

    double pre1 = 3.0 * subdiv_step;  
    double pre2 = 3.0 * subdiv_step2;  
    double pre4 = 6.0 * subdiv_step2;  
    double pre5 = 6.0 * subdiv_step3;  

    double tmp1x = x1 - x2 * 2.0 + x3;  
    double tmp1y = y1 - y2 * 2.0 + y3;  

    double tmp2x = (x2 - x3)*3.0 - x1 + x4;  
    double tmp2y = (y2 - y3)*3.0 - y1 + y4;  

    double fx = x1;  
    double fy = y1;  

    double dfx = (x2 - x1)*pre1 + tmp1x*pre2 + tmp2x*subdiv_step3;  
    double dfy = (y2 - y1)*pre1 + tmp1y*pre2 + tmp2y*subdiv_step3;  

    double ddfx = tmp1x*pre4 + tmp2x*pre5;  
    double ddfy = tmp1y*pre4 + tmp2y*pre5;  

    double dddfx = tmp2x*pre5;  
    double dddfy = tmp2y*pre5;  

    int step = NUM_STEPS;  
 
	while(step--)  
	{  
		fx   += dfx;  
		fy   += dfy;  
		dfx  += ddfx;  
		dfy  += ddfy;  
		ddfx += dddfx;  
		ddfy += dddfy;  
		CvPoint tmp1(fx,fy);
		p.push_back(tmp1);  
	}  
    CvPoint tmp2(x4,y4);
    p.push_back(tmp2); 
}  



/*
 * Function: bezier_curve_generator(const ar_track_alvar_msgs::AlvarMarkers req)
 * --------------------------------
 * This function generates a vector of points representing the path.
 */
void Bezier_global_planner::bezier_curve_generator(const ar_track_alvar_msgs::AlvarMarkers req)
{
     
    if (!req.markers.empty())
    {
        /* TODO: rank the distance if seeing multiple ar markers */

        // obtain relative pose between QR code and camera
        // Note that z axis positive direction is the camera's facing direction,
        // and x axis positive direction is the camera's right hand side.
        ROS_INFO("I heard AR_marker Depth_z and Horizontal_offset_x :");
        float position_z=req.markers[0].pose.pose.position.z * 0.9; // depth info
        float position_x=req.markers[0].pose.pose.position.x * 0.9; // horizontal offset
        ROS_INFO("I heard position z: [%f]",req.markers[0].pose.pose.position.z);
        ROS_INFO("I heard position x: [%f]",req.markers[0].pose.pose.position.x);

        float quaternion_x=req.markers[0].pose.pose.orientation.x;
        float quaternion_y=req.markers[0].pose.pose.orientation.y;
        float quaternion_z=req.markers[0].pose.pose.orientation.z;
        float quaternion_w=req.markers[0].pose.pose.orientation.w;
        tf::Quaternion q (
        req.markers[0].pose.pose.orientation.x,
        req.markers[0].pose.pose.orientation.y,
        req.markers[0].pose.pose.orientation.z,
        req.markers[0].pose.pose.orientation.w);
        

        // changing quaternion into RPY angle
        //ROS_INFO("I heard RPY:");
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        //ROS_INFO("roll, pitch, yaw=%.2f degree  %.2f degree  %.2f degree" , roll*180/PI, pitch*180/PI, yaw*180/PI);
        //transfer RPY into useful target direction
        double target_direction = ((roll*180/PI) < 0) ? (-90 - roll*180/PI) : (270 - roll*180/PI) ;
        ROS_INFO("I heard target direction: [%f] degree", target_direction);

        // create bezier curve according to x, z and target direction.
        CvPoint point[4];
        point[0].x=0.0;
        point[0].y=0.0;
        point[1].x=0.0;
        point[1].y=0.4 * fabs(position_z) + 0.01 ;
        point[2].x= position_x - cos(target_direction * PI / 180) * 0.6 * (fabs(position_z) + 0.01) ;
        point[2].y= position_z - sin(target_direction * PI / 180) * 0.6 * (fabs(position_z) + 0.01) ;
        point[3].x=position_x;
        point[3].y=position_z;
        vector<CvPoint> curvePoint;
        curve4(curvePoint,
                point[0].x,point[0].y,
                point[1].x,point[1].y,
                point[2].x,point[2].y,
                point[3].x,point[3].y
                );

        // public curve dots
        navigation_without_movebase::CvPoint p_msg;
        navigation_without_movebase::CvPoints curve_msg;
        curve_msg.position_x = position_x;
        curve_msg.position_z = position_z;
        curve_msg.target_direction = target_direction;
        for(int i = 0; i < curvePoint.size(); i++){
            p_msg.x = curvePoint[i].x;
            p_msg.y = curvePoint[i].y;
            curve_msg.points.push_back(p_msg);
        }
        pub_path_dots.publish(curve_msg);

    }

}


/*
int main()
{
	CvPoint point[4];
	point[0].x=0.0;
	point[0].y=0.0;
	point[1].x=0;
	point[1].y=0.8;
	point[2].x=0.4;
	point[2].y=2-0.6 * 1.732;
	point[3].x=1;
	point[3].y=2;
	vector<CvPoint> curvePoint;
	curve4(curvePoint,
			point[0].x,point[0].y,
			point[1].x,point[1].y,
			point[2].x,point[2].y,
			point[3].x,point[3].y
			);
	int i=0;
	for(;i<curvePoint.size();i++)
	{
		cout<<curvePoint[i].x<<endl;
	}
    cout<<"-------------------------------------------------------------" << endl;
    for(int i = 0;i<curvePoint.size();i++)
	{
		cout<<curvePoint[i].y<<endl;
	}
	cout<<endl<<"点的个数："<<i<<endl;
	system("pause");
	return 0;
}*/



int main(int argc, char **argv)
{

  ros::init(argc, argv, "bezier_global_planner");

  Bezier_global_planner globalPlanner = Bezier_global_planner();
  ros::Rate loop_rate(10);
  ros::spin();
  return 0;
}
