#include "ros/ros.h"
#include "serial/pose.h"
#include "serial/status.h"
#include "serial.h"
#include <cstdlib>

#include <chrono>
#include <thread>
#include <iostream>
using namespace std;



struct PoseData {
	int   distance;
	float yaw_angle;
	float pitch_angle;
};



class SerialNode
{
private:
	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::Subscriber sub;


	PoseData pd;


	Serial _serial;
	Test_send ts;
	Test_receive rec;

	int trans_flag;
	int WORKING_MODE;
	int ATTACK_COLOR;

public:
	SerialNode();
	~SerialNode();
	void serialCallback(const serial::pose::ConstPtr& msg);
	bool receiveRobotData();
	void sendCtrlData();
	bool spin();
};
