#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#define	sBUFFERSIZE	15//send buffer size 串口发送缓存长度
#define	rBUFFERSIZE	27//receive buffer size 串口接收缓存长度
unsigned char s_buffer[sBUFFERSIZE];//发送缓存

typedef union{
	unsigned char cvalue[4];
	float fvalue;
}float_union;

serial::Serial ser;

void data_pack(const geometry_msgs::Twist& cmd_vel){
	//unsigned char i;
	float_union Vx,Vy,Ang_v;
	Vx.fvalue = cmd_vel.linear.x;
	Vy.fvalue = cmd_vel.linear.y;
	Ang_v.fvalue = cmd_vel.angular.z;
	
	memset(s_buffer,0,sizeof(s_buffer));
	//数据打包
	s_buffer[0] = 0xff;
	s_buffer[1] = 0xff;
	//Vx
	s_buffer[2] = Vx.cvalue[0];
	s_buffer[3] = Vx.cvalue[1];
	s_buffer[4] = Vx.cvalue[2];
	s_buffer[5] = Vx.cvalue[3];
	//Vy
	s_buffer[6] = Vy.cvalue[0];
	s_buffer[7] = Vy.cvalue[1];
	s_buffer[8] = Vy.cvalue[2];
	s_buffer[9] = Vy.cvalue[3];
	//Ang_v
	s_buffer[10] = Ang_v.cvalue[0];
	s_buffer[11] = Ang_v.cvalue[1];
	s_buffer[12] = Ang_v.cvalue[2];
	s_buffer[13] = Ang_v.cvalue[3];
	//CRC
	s_buffer[14] = s_buffer[2]^s_buffer[3]^s_buffer[4]^s_buffer[5]^s_buffer[6]^s_buffer[7]^
					s_buffer[8]^s_buffer[9]^s_buffer[10]^s_buffer[11]^s_buffer[12]^s_buffer[13];

	ser.write(s_buffer,sBUFFERSIZE);
	ROS_INFO("TARGET SENT.");
	
}


//订阅turtle1/cmd_vel话题的回调函数，用于显示速度以及角速度
void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel){
	ROS_INFO("I heard linear velocity: x-[%f],y-[%f],",cmd_vel.linear.x,cmd_vel.linear.y);
	ROS_INFO("I heard angular velocity: [%f]",cmd_vel.angular.z);
	std::cout << "Twist Received" << std::endl;	
	data_pack(cmd_vel);
}
int main (int argc, char** argv)
{
    ros::init(argc, argv, "my_serial_node");
    ros::NodeHandle nh;

	//订阅/turtle1/cmd_vel话题用于测试 $ rosrun turtlesim turtle_teleop_key
	ros::Subscriber write_sub = nh.subscribe("/cmd_vel",1000,cmd_vel_callback);
    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen())
	{
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }
	


}

