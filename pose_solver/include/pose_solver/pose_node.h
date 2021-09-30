#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <pose_solver/rect_data.h>
#include <pose_solver/pose.h>
#include <pose_solver/status.h>

#include <pose_solver/pnp_solver.h>
#include <pose_solver/trans_pose.h>

#include <iostream>
#include <chrono>
#include <thread>
#include <queue>
using namespace std;
using namespace cv;


struct RobotStatus {
	int mode;
	int blood;
	float power;
	float yaw_real_angle;
	float pitch_real_angle;
};


class PoseNode
{
private:
	ros::NodeHandle nh;
	ros::Subscriber sub;
	ros::Subscriber status_sub;
	ros::Publisher pub;

	RobotStatus robot_status;
	std::vector<float> rect_array;
	//std::queue<PoseData> pose_que;
	std::vector<cv::Point2f> rect_point;
	cv::Point3f posePoint;
	int pub_cnt;
	int armor_type;
	bool trans_flag;
	int WORKING_MODE;

	Input_coordinate pose_trans_in;
	OutAngles ctrl_angle;

public:
	PoseNode();
	~PoseNode();
	void poseCallback(const pose_solver::rect_data::ConstPtr& msg);
	void statusCallback(const pose_solver::status::ConstPtr& msg);
	bool spin();
	void convert2Coordinate(std::vector<float>& input_vec, std::vector<cv::Point2f>& point_vec);
	//bool getPoseData(std::vector<Point2f>& input_vec,PoseData& _pose);  
	//bool sendCtrlData(pose_solver::control_srv::Request &req,pose_solver::control_srv::Response &res);


};
