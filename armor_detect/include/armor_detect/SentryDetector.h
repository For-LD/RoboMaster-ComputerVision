#ifndef SENTRYDETECTOR_H
#define SENTRYDETECTOR_H

#include "opencv2/highgui/highgui.hpp"

#define POINT_DIST(p1,p2) std::sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y))
enum s_EnemyColor { rred = 0, bblue = 1 };

struct  s_FinalPoints
{
	std::vector<cv::Point2f> finalpoints;
	int CODE = -1;
	bool is_small_armor = true;
};


struct s_LeftRightLight {
	std::vector<cv::RotatedRect> lrect;
	std::vector<cv::RotatedRect> rrect;

};

struct SentryParam {
	uchar min_light_height;			//灯条最小高度
	uchar max_light_height;         //灯条最大高度

	int   min_light_delta_h;          //左右灯条的最小距离
	int   max_light_delta_h;             //左右灯条的最大距离
	uchar enemy_color;                 // 0 for red, otherwise blue
	uchar red_armor_threshold;
	uchar blue_armor_threshold;
	uchar  min_light_gray;//-----
	uchar max_light_delta_v;       //两灯条的高度差
	uchar max_light_delta_angle;   //两灯条的角度差

	uchar max_light_delta_height;


	SentryParam() {
		min_light_gray = 170;
		min_light_height = 3;//8
		max_light_height = 50;
		min_light_delta_h = 14;
		max_light_delta_h = 150;

		enemy_color = 0;
		red_armor_threshold = 60;
		blue_armor_threshold = 60;
		max_light_delta_v = 35;   //
		max_light_delta_angle = 30;  //
		max_light_delta_height = 15;
	}
};




class SentryDetector
{
public:
	SentryDetector(const std::string& filename, const std::string& template_path) {


		_filename = filename;
		_template_path = template_path;



	}

	void read_default(const cv::FileStorage& fs) {

		// for armor system
		fs["default_min_light_height"] >> armor_p.min_light_height;
		fs["default_max_light_height"] >> armor_p.max_light_height;
		fs["default_min_light_delta_h"] >> armor_p.min_light_delta_h;
		fs["default_max_light_delta_h"] >> armor_p.max_light_delta_h;
		fs["default_red_armor_threshold"] >> armor_p.red_armor_threshold;
		fs["default_blue_armor_threshold"] >> armor_p.blue_armor_threshold;
		fs["enemy_color"] >> armor_p.enemy_color;
		fs["default_min_light_gray"] >> armor_p.min_light_gray;
		fs["default_max_light_delta_v"] >> armor_p.max_light_delta_v;
		fs["default_max_light_delta_angle"] >> armor_p.max_light_delta_angle;
		fs["default_max_light_delta_height"] >> armor_p.max_light_delta_height;
		check();
	}

	void read_far(const cv::FileStorage& fs) {

		// for armor system
		fs["far_min_light_height"] >> armor_p.min_light_height;
		fs["far_max_light_height"] >> armor_p.max_light_height;
		fs["far_min_light_delta_h"] >> armor_p.min_light_delta_h;
		fs["far_max_light_delta_h"] >> armor_p.max_light_delta_h;
		fs["far_red_armor_threshold"] >> armor_p.red_armor_threshold;
		fs["far_blue_armor_threshold"] >> armor_p.blue_armor_threshold;
		fs["enemy_color"] >> armor_p.enemy_color;
		fs["far_min_light_gray"] >> armor_p.min_light_gray;
		fs["far_max_light_delta_v"] >> armor_p.max_light_delta_v;
		fs["far_max_light_delta_angle"] >> armor_p.max_light_delta_angle;
		fs["far_max_light_delta_height"] >> armor_p.max_light_delta_height;
		check();
	}

	void read_near(const cv::FileStorage& fs) {

		// for armor system
		fs["near_min_light_height"] >> armor_p.min_light_height;
		fs["near_max_light_height"] >> armor_p.max_light_height;
		fs["near_min_light_delta_h"] >> armor_p.min_light_delta_h;
		fs["near_max_light_delta_h"] >> armor_p.max_light_delta_h;
		fs["near_red_armor_threshold"] >> armor_p.red_armor_threshold;
		fs["near_blue_armor_threshold"] >> armor_p.blue_armor_threshold;
		fs["enemy_color"] >> armor_p.enemy_color;
		fs["near_min_light_gray"] >> armor_p.min_light_gray;
		fs["near_max_light_delta_v"] >> armor_p.max_light_delta_v;
		fs["near_max_light_delta_angle"] >> armor_p.max_light_delta_angle;
		fs["near_max_light_delta_height"] >> armor_p.max_light_delta_height;
		check();
	}


	void check() {
		SentryParam default_armor;
		if (armor_p.min_light_gray < 5)
			armor_p.min_light_gray = default_armor.min_light_gray;

		if (armor_p.min_light_height < 1)
			armor_p.min_light_height = default_armor.min_light_height;
		if (armor_p.max_light_height < 3)
			armor_p.max_light_height = default_armor.max_light_height;
		if (armor_p.min_light_delta_h < 5)
			armor_p.min_light_delta_h = default_armor.min_light_delta_h;
		if (armor_p.max_light_delta_h < 5)
			armor_p.max_light_delta_h = default_armor.max_light_delta_h;
		if (armor_p.max_light_delta_v < 5)
			armor_p.max_light_delta_v = default_armor.max_light_delta_v;
		if (armor_p.max_light_delta_angle < 5)
			armor_p.max_light_delta_angle = default_armor.max_light_delta_angle;
		if (armor_p.max_light_delta_height < 5)
			armor_p.max_light_delta_height = default_armor.max_light_delta_height;

		if (armor_p.red_armor_threshold < 5)
			armor_p.red_armor_threshold = default_armor.red_armor_threshold;
		if (armor_p.blue_armor_threshold < 5)
			armor_p.blue_armor_threshold = default_armor.blue_armor_threshold;

	}


	s_FinalPoints getTargetAera(cv::Mat& src, double distance);


public:
	void setImage(const cv::Mat& img);
	cv::RotatedRect adjustRRect(const cv::RotatedRect& rect);
	int templateDist(const cv::Mat& img);
	cv::RotatedRect boundingRRect(cv::RotatedRect& left, cv::RotatedRect& right);
	void findContour(std::vector<cv::RotatedRect>& final_rects, std::vector<double>& score, s_LeftRightLight& leftright_rect);
	cv::RotatedRect chooseTarget(std::vector<cv::RotatedRect>& final_rects, std::vector<double>& score, s_LeftRightLight& leftright_rect, cv::RotatedRect& lrect, cv::RotatedRect& rrect);
	bool makeRectSafe(cv::Rect& rect, cv::Size size) {
		if (rect.x < 0)
			rect.x = 0;
		if (rect.x + rect.width > size.width)
			rect.width = size.width - rect.x;
		if (rect.y < 0)
			rect.y = 0;
		if (rect.y + rect.height > size.height)
			rect.height = size.height - rect.y;
		if (rect.width <= 0 || rect.height <= 0)
			return false;
		return true;
	}



public:
	SentryParam armor_p;
	cv::Mat _binary_template_small; // small armor template binay image
	std::string _template_path;
	SentryParam _para;               // parameter of alg
	cv::Mat _src;                   // source image
	cv::Mat _max_color;             // binary image of sub between blue and red component
	cv::Mat _g;
	bool _is_small_armor;        //big or small armor,true is small
	std::string _filename;
};

#endif // SENTRYDETECTOR_H
