#ifndef ARMORDETECTOR_H
#define ARMORDETECTOR_H


#include "opencv2/highgui/highgui.hpp"
#include "rm.h"
#include <string>
#define POINT_DIST(p1,p2) std::sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y))


enum EnemyColor { RED = 0, BLUE = 1 };

struct ArmorParam {
	uchar min_light_height;			//灯条最小高度
	int   max_light_delta_h;
	int   min_light_delta_h;          //左右灯条的最小距离
	uchar enemy_color;                 // 0 for red, otherwise blue
	uchar red_armor_threshold;
	uchar blue_armor_threshold;
	uchar  min_light_gray;//-----
	uchar max_light_delta_v;       //两灯条的高度差
	uchar max_light_delta_angle;   //两灯条的角度差
	uchar max_light_delta_height;



	ArmorParam() {
		min_light_gray = 150;
		min_light_height = 4;//8
		min_light_delta_h = 14;
		max_light_delta_h = 500;
		enemy_color = 1;
		red_armor_threshold = 60;
		blue_armor_threshold = 60;
		max_light_delta_v = 40;   //
		max_light_delta_angle = 30;  //
		max_light_delta_height = 20;

	}
};

struct Final_rects {
	std::vector<cv::RotatedRect> rects;
	std::vector<int> idx;
};

struct LeftRightLight {
	std::vector<cv::RotatedRect> lrect;
	std::vector<cv::RotatedRect> rrect;

};

struct  FinalPoints
{
	std::vector<cv::Point2f> finalpoints;
	int CODE = -1;
	bool is_small_armor = true;
};

class ArmorDetector {
public:
	ArmorDetector(const std::string& filename, const std::string _tamplate_path) {
		_file_name = filename;





		tamplate_path = _tamplate_path;

	}


	void read_default(const cv::FileStorage& fs) {

		// for armor system
		fs["default_min_light_gray"] >> armor_p.min_light_gray;
		fs["default_min_light_height"] >> armor_p.min_light_height;
		fs["default_max_light_delta_h"] >> armor_p.max_light_delta_h;
		fs["default_min_light_delta_h"] >> armor_p.min_light_delta_h;
		fs["default_red_armor_threshold"] >> armor_p.red_armor_threshold;
		fs["default_blue_armor_threshold"] >> armor_p.blue_armor_threshold;
		fs["enemy_color"] >> armor_p.enemy_color;
		fs["default_max_light_delta_v"] >> armor_p.max_light_delta_v;
		fs["default_max_light_delta_angle"] >> armor_p.max_light_delta_angle;
		fs["default_max_light_delta_height"] >> armor_p.max_light_delta_height;



		check();

	}


	void read_far(const cv::FileStorage& fs) {

		// for armor system
		fs["far_min_light_gray"] >> armor_p.min_light_gray;
		fs["far_min_light_height"] >> armor_p.min_light_height;
		fs["far_max_light_delta_h"] >> armor_p.max_light_delta_h;
		fs["far_min_light_delta_h"] >> armor_p.min_light_delta_h;
		fs["far_red_armor_threshold"] >> armor_p.red_armor_threshold;
		fs["far_blue_armor_threshold"] >> armor_p.blue_armor_threshold;
		fs["enemy_color"] >> armor_p.enemy_color;
		fs["far_max_light_delta_v"] >> armor_p.max_light_delta_v;
		fs["far_max_light_delta_angle"] >> armor_p.max_light_delta_angle;
		fs["far_max_light_delta_height"] >> armor_p.max_light_delta_height;



		check();
	}


	void read_near(const cv::FileStorage& fs) {

		// for armor system
		fs["near_min_light_gray"] >> armor_p.min_light_gray;
		fs["near_min_light_height"] >> armor_p.min_light_height;
		fs["near_max_light_delta_h"] >> armor_p.max_light_delta_h;
		fs["near_min_light_delta_h"] >> armor_p.min_light_delta_h;
		fs["near_red_armor_threshold"] >> armor_p.red_armor_threshold;
		fs["near_blue_armor_threshold"] >> armor_p.blue_armor_threshold;
		fs["enemy_color"] >> armor_p.enemy_color;
		fs["near_max_light_delta_v"] >> armor_p.max_light_delta_v;
		fs["near_max_light_delta_angle"] >> armor_p.max_light_delta_angle;
		fs["near_max_light_delta_height"] >> armor_p.max_light_delta_height;



		check();
	}

	void check() {
		ArmorParam default_armor;
		if (armor_p.min_light_gray < 5)
			armor_p.min_light_gray = default_armor.min_light_gray;

		if (armor_p.min_light_height < 1)
			armor_p.min_light_height = default_armor.min_light_height;

		if (armor_p.max_light_delta_h < 5)
			armor_p.max_light_delta_h = default_armor.max_light_delta_h;

		if (armor_p.min_light_delta_h < 5)
			armor_p.min_light_delta_h = default_armor.min_light_delta_h;

		if (armor_p.red_armor_threshold < 5)
			armor_p.red_armor_threshold = default_armor.red_armor_threshold;
		if (armor_p.blue_armor_threshold < 5)
			armor_p.blue_armor_threshold = default_armor.blue_armor_threshold;

		if (armor_p.max_light_delta_v < 5)
			armor_p.max_light_delta_v = default_armor.max_light_delta_v;
		if (armor_p.max_light_delta_angle < 5)
			armor_p.max_light_delta_angle = default_armor.max_light_delta_angle;

		if (armor_p.max_light_delta_height < 5)
			armor_p.max_light_delta_height = default_armor.max_light_delta_height;
	}




	void initTemplate(const cv::Mat& _template_small);
	FinalPoints getTargetAera(cv::Mat& src, SceneImage& sceneImage, double distance);


public:
	/**
	 * @brief setImage Pocess the input (set the green component and sub of blue and red component)
	 * @param src
	 */
	void setImage(std::string color, const cv::Mat& img);

	/**
	 * @brief templateDist Compute distance between template and input image
	 * @param img input image
	 * @param is_smarect_pnp_solverll true if input image is a samll armor, otherwise, false
	 * @return distance
	 */

	int templateDist(const cv::Mat& img);
	/**
	 * @brief findContourInEnemyColor Find contour in _max_color
	 * @param left output left contour image (probably left lamp of armor)
	 * @param right output righe edge (probably right lamp of armor)
	 * @param contours_left output left contour
	 * @param contours_right output right contour
	 */

	void findContour(int ret_idx, Final_rects& final_rects, std::vector<double>& score, int yolo_score, LeftRightLight& leftright_rect);


	/**
	 * @brief chooseTarget Choose the most possible rectangle among all the rectangles
	 * @param rects candidate rectangles
	 * @return the most likely armor (RotatedRect() returned if no proper one)
	 */
	cv::RotatedRect chooseTarget(Final_rects& final_rects, std::vector<double>& score, int& idx, LeftRightLight& leftright_rect, cv::RotatedRect& lrect, cv::RotatedRect& rrect);


	/**
	 * @brief boundingRRect Bounding of two ratate rectangle (minumum area that contacts two inputs)
	 * @param left left RotatedRect
	 * @param right right RotatedRect
	 * @return minumum area that contacts two inputs
	 */
	cv::RotatedRect boundingRRect(cv::RotatedRect& left, cv::RotatedRect& right);



	/**
	 * @brief adjustRRect Adjust input angle
	 * @param rect input
	 * @return adjusted rotate rectangle
	 */
	cv::RotatedRect adjustRRect(const cv::RotatedRect& rect);
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

	bool broadenRect(cv::Rect& rect, int width_added, int height_added, cv::Size size) {
		rect.x -= width_added;
		rect.width += width_added * 2;
		rect.y -= height_added;
		rect.height += height_added * 2;
		return makeRectSafe(rect, size);
	}

public:
	ArmorParam armor_p;
	cv::Mat _binary_template_small; // small armor template binay image
	ArmorParam _para;               // parameter of alg
	cv::Mat _src;                   // source image
	cv::Mat _max_color;             // binary image of sub between blue and red component
	cv::Mat _g;
	bool _is_small_armor;        //big or small armor,true is small
	std::string tamplate_path;
	std::string _file_name;
};



#endif // ARMORDETECTOR_H
