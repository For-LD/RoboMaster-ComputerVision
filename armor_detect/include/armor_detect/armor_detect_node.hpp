#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>


#include <armor_detect/rect_data.h>
#include <armor_detect/status.h>
#include <armor_detect/pose.h>
#include <iostream>

#include <chrono>
#include <thread>
#include <mutex>

#include <armor_detector.h>
#include <SentryDetector.h>
#include <getStream.hpp>
using namespace std;
using namespace cv;

enum ExecuteStatus { NO_TARGET, DETECTING, TRACKING };
enum YOLOStatus { YOLODETECT, NOYOLODETECT };

class DetectNode
{
private:
	ros::NodeHandle nh_;
	ros::Publisher rect_pub_;
	ros::Subscriber status_sub;
	ros::Subscriber pose_sub;

	int WORKING_MODE;
	int Attack_color;
	int pub_cnt;
	int debug_show_flag = 0;
	int save_img_flag = 0;
	int last_distance = 0;
	//bool capture_flag;

	//list<cv::Mat> img_list;
	cv::RotatedRect tmp_rect;
	std::vector<cv::RotatedRect> rect_v;




	SceneImage sceneImage;
	FinalPoints light_points;
	s_FinalPoints s_light_points;
	//RM_YOLO *rm_detector;
	//ArmorDetector armor_detector;
	struct SolverMsg {
		int   distance;
		float yaw_angle;
		float pitch_angle;
	}solver;


public:
	DetectNode();
	void statusCallback(const armor_detect::status::ConstPtr& msg);
	void solverCallback(const armor_detect::pose::ConstPtr& msg);
	bool spin();
	//int getStream();
	//bool detectInit();
	//void detectArmorLight(cv::Mat &cv_im,cv::RotatedRect &armor_light);


	~DetectNode();

};
