#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>
#include <fstream>
#include <streambuf>
#include "rm.h"
#include "armor_detector.h"
using namespace std;

#define DEBUG
// test robot




class Test {
	std::string _file_name;
	std::string _enemy_color;
public:

	Test(const std::string& filename) {
		_file_name = filename;


	}

	void read_default(const cv::FileStorage& fs) {

		// for armor system

		fs["enemy_color"] >> _enemy_color;


	}

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

	inline bool outSide(cv::Mat& gray, cv::Point& p)
	{
		int h = gray.rows;
		int w = gray.cols;
		if (p.x < 0) return true;
		if (p.y < 0) return true;
		if (p.x >= w) return true;
		if (p.y >= h) return true;
		return false;
	}

	inline int getPixel(cv::Mat& gray, cv::Point& lt, cv::Point& rt, cv::Point& lb,
		float rx, float ry)
	{
		int x = lt.x * (1 - rx - ry) + rt.x * rx + lb.x * ry;
		int y = lt.y * (1 - rx - ry) + rt.y * rx + lb.y * ry;
		int val = gray.at<uchar>(y, x);
		return val;
	}

	inline float calcRational(cv::Mat& gray, cv::Point& lt, cv::Point& rt,
		cv::Point& lb, cv::Point& rb)
	{
		int h = gray.rows;
		int w = gray.cols;
		if (outSide(gray, lt))  return -1;
		if (outSide(gray, rt))  return -1;
		if (outSide(gray, lb))  return -1;
		if (outSide(gray, rb))  return -1;

		// thresh
		int v1 = getPixel(gray, lt, rt, lb, 0.5, 0);
		int v2 = getPixel(gray, lt, rt, lb, 0.5, 0.5);
		int v3 = getPixel(gray, lt, rt, lb, 0.5, 1);
		float thresh = (v1 + v2 + v3) / 3.f;
		int hit = 0;
		// light
		vector<float> lightPosX = { 0, 0, 0, 1, 1, 1 };
		vector<float> lightPosY = { 0, 0.5, 1, 0, 0.5, 1 };
		for (int i = 0; i < lightPosX.size(); i++) {
			int val = getPixel(gray, lt, rt, lb, lightPosX[i], lightPosY[i]);
			if (val > thresh) {
				hit++;
			}
		}
		// dark
		vector<float> darkPosX = { 0.2, 0.2, 0.2, 0.8, 0.8, 0.8 };
		vector<float> darkPosY = { 0, 0.5, 1, 0, 0.5, 1 };
		for (int i = 0; i < darkPosX.size(); i++) {
			int val = getPixel(gray, lt, rt, lb, darkPosX[i], darkPosY[i]);
			if (val < thresh) {
				hit++;
			}
		}
		float res = hit / 12.f;
		return res;
	}

	void genHorVer(float rad, float wSize, float hSize,
		int& horX, int& horY, int& verX, int& verY)
	{
		horX = cos(rad) * wSize + 0.5;
		horY = -sin(rad) * wSize + 0.5;
		verX = sin(rad) * hSize + 0.5;
		verY = cos(rad) * hSize + 0.5;
	}

	float calcDist(cv::Point p1, cv::Point p2)
	{
		float dx = p1.x - p2.x;
		float dy = p1.y - p2.y;
		return std::sqrt(dx * dx + dy * dy);
	}

	int sortByY(const cv::Point& p1, const cv::Point& p2)
	{
		return p1.y < p2.y;
	}

	void getEndPoint(vector<cv::Point>& pts, cv::Point& top, cv::Point& bottom)
	{
		sort(pts.begin(), pts.end(), [](const cv::Point& p1, const cv::Point& p2) {
			return p1.y < p2.y;
			});  // higher prior

		float maxDist = 0;
		for (int i = 0; i < pts.size(); i++) {
			for (int j = i + 1; j < pts.size(); j++) {
				float dist = calcDist(pts[i], pts[j]);
				if (dist > maxDist) {
					maxDist = dist;
					top = pts[i];
					bottom = pts[j];
				}
			}
		}
	}

	void getCorners(cv::Point top1, cv::Point bottom1, cv::Point top2, cv::Point bottom2,
		cv::Point& lt, cv::Point& rt, cv::Point& lb, cv::Point& rb)
	{
		if (top1.x > top2.x) {
			swap(top1, top2);
			swap(bottom1, bottom2);
		}
		lt = top1;
		rt = top2;
		lb = bottom1;
		rb = bottom2;
	}

	// todo: add angle penalization?

	float shapeSimi(cv::Point lt, cv::Point rt, cv::Point lb, cv::Point rb, float radio)
	{
		float vmx = lb.x - lt.x;
		float vmy = lb.y - lt.y;
		float hmx = vmy * radio;
		float hmy = -vmx * radio;
		//
		float td1 = calcDist(lt, rt);
		float td2 = calcDist(lt, cv::Point(lt.x + hmx, lt.y + hmy));
		float td3 = calcDist(rt, cv::Point(lt.x + hmx, lt.y + hmy));
		float s1 = max(0.f, 1.f - td3 / (max(td1, td2) + 0.00001f));

		float bd1 = calcDist(lb, rb);
		float bd2 = calcDist(lb, cv::Point(lb.x + hmx, lb.y + hmy));
		float bd3 = calcDist(rb, cv::Point(lb.x + hmx, lb.y + hmy));
		float s2 = max(0.f, 1.f - bd3 / (max(bd1, bd2) + 0.00001f));
		float res = s1 * s2;
		return res;
	}

	cv::Point mirrorPoint(cv::Point p)
	{
		cv::Point p2;
		p2.x = -p.x;
		p2.y = p.y;
		return p2;
	}

	float calcShapeSimi(cv::Point lt, cv::Point rt, cv::Point lb, cv::Point rb, float radio)
	{
		float res1 = shapeSimi(lt, rt, lb, rb, radio);
		// swap
		cv::Point lt2, rt2, lb2, rb2;
		lt2 = mirrorPoint(rt);
		rt2 = mirrorPoint(lt);
		lb2 = mirrorPoint(rb);
		rb2 = mirrorPoint(lb);
		float res2 = shapeSimi(lt2, rt2, lb2, rb2, radio);
		//
		float res = sqrt(res1 * res2);
		return res;
	}

	float calcPositionScore(cv::Mat& gray, cv::Point lt, cv::Point rt, cv::Point lb, cv::Point rb)
	{
		int h = gray.rows;
		int w = gray.cols;
		float dx = ((lt.x + rb.x) / 2 - w / 2) / (float)w;
		float dy = ((lt.y + rb.y) / 2 - h / 2) / (float)h;
		float corr = 1 - pow(dx, 2.f) - pow(dy, 2.f);
		return corr;
	}

	cv::Mat getLightMask(cv::Mat& gray)
	{
		cv::Mat equal;
		cv::equalizeHist(gray, equal);
		cv::Mat mask = equal > 200;

		// imshow("equal.jpg", equal);
		// imshow("gray.jpg", gray);
		 //imshow("mask.jpg", mask);
		// cv::waitKey(1);
		return mask;
	}

	bool lightSearch(cv::Mat& img,
		cv::Point& res_lt, cv::Point& res_rt, cv::Point& res_lb, cv::Point& res_rb,
		float& res_conf, int& res_type, bool& res_isRed)
	{
		int H = img.rows;
		int W = img.cols;
		if (H == 0 || W == 0) {
			return false;
		}
		res_type = -1;
		res_conf = -1;
		cv::Mat gray, equal;
		cv::cvtColor(img, gray, CV_BGR2GRAY);

		float ideaArea = 12000;
		float scale = sqrt(ideaArea / (float)(img.rows * img.cols));

		//  float scale = 2;
		cv::resize(gray, gray, cv::Size(), scale, scale);
		//
		cv::Ptr<cv::MSER> mser = cv::MSER::create(5, 10, 1000);
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Rect> box;
		mser->detectRegions(gray, contours, box);

		// #ifdef DEBUG
		   // display
		cv::Mat mserImg;
		cv::resize(img, mserImg, cv::Size(), scale, scale);
		for (int i = 0; i < contours.size(); i++)
		{
			cv::ellipse(mserImg, cv::fitEllipse(contours[i]), cv::Scalar(0, 255, 0));
			//     cv::imwrite("./mser.jpg", mserImg);
			//     cv::imshow("mser.jpg", mserImg);

		}
		//   cout << "## " <<  __LINE__ << endl;
		//   cv::imshow("mser.jpg", mserImg);
		//   cv::waitKey(0);
		//   cout << "## " << __LINE__ << endl;
		//   cv::imwrite("./gray.jpg", gray);
		// #endif

		cv::Mat mask = getLightMask(gray);
		std::vector<cv::Point> topVec;
		std::vector<cv::Point> bottomVec;
		for (int i = 0; i < contours.size(); i++)
		{
			cv::Point top, bottom;
			getEndPoint(contours[i], top, bottom);
			if (mask.at<uchar>(top) == 0) continue;
			if (mask.at<uchar>(bottom) == 0) continue;

			topVec.push_back(top);
			bottomVec.push_back(bottom);
		}

		if (contours.size() < 2) return false;

		vector<float> typeHWRatio = { 2.5, 3.3 };
		cv::Vec2i bestPair;

		for (int type = 0; type <= 1; type++)
		{
			for (int i = 0; i < topVec.size(); i++)
			{
				for (int j = i + 1; j < topVec.size(); j++)
				{
					cv::Point lt, rt, lb, rb;
					getCorners(topVec[i], bottomVec[i], topVec[j], bottomVec[j], lt, rt, lb, rb);
					float shapeScore = calcShapeSimi(lt, rt, lb, rb, typeHWRatio[type]);
					float posScore = calcPositionScore(gray, lt, rt, lb, rb);
					float rationalScore = calcRational(gray, lt, rt, lb, rb);
					float conf = rationalScore * shapeScore * posScore;
					if (conf > res_conf)
					{
						res_conf = conf;
						res_type = type;
						res_lt = lt / scale;
						res_rt = rt / scale;
						res_lb = lb / scale;
						res_rb = rb / scale;
						bestPair[0] = i;
						bestPair[1] = j;
					}
				}
			}
		}
		if (res_type < 0 || res_conf < 0)
		{
			return false;
		}

		// // identify color, blue or red
		// vector<cv::Point> contour0 = contours[bestPair[0]];
		// vector<cv::Point> contour1 = contours[bestPair[1]];
		// cv::Mat scaleImg;
		// cv::resize(img, scaleImg, cv::Size(), scale, scale);
		// int blueVote = 0;
		// int redVote = 0;
		// for (int y = 0; y < scaleImg.rows; y++){
		//   for (int x = 0; x < scaleImg.cols; x++){
		//     if ( cv::pointPolygonTest(contour0, cv::Point2f(y, x), false) > 0  
		//         ||  cv::pointPolygonTest(contour1, cv::Point2f(y, x), false) > 0 )
		//     {
		//        // todo
		//     }
		//   }
		// }
		return true;
	}

	FinalPoints getAera(Mat& src, SceneImage& sceneImage) {
		cv::FileStorage setting_fs(_file_name, cv::FileStorage::READ);

		FinalPoints finalPoints;
		string color;
		if (_enemy_color == RED)    color = "red";
		else     color = "blue";
		for (auto iter = sceneImage.m_object_list.begin(); iter != sceneImage.m_object_list.end();)
		{
			if (((*iter).m_label != color) && ((*iter).m_label != "last"))
			{
				iter = sceneImage.m_object_list.erase(iter);
			}
			else
			{
				iter++;
			}
		}
		if (sceneImage.m_object_list.size() < 1)
		{
			finalPoints.CODE = -1;
			cout << "can not find valid target!" << endl;
			return finalPoints;
		}
		if (sceneImage.m_object_list.size() < 1)
		{
			finalPoints.CODE = -1;
			cout << "can not find valid target!" << endl;
			return finalPoints;
		}

		int count = sceneImage.m_object_list.size();
		double x_offset, y_offset;
		cv::Point res_lt, res_rt, res_lb, res_rb;
		float res_conf, max_conf = -1;
		int res_type;
		bool isRed;
		int idx = -1;
		int x, y, w, h; string color; float yolo_score;
		for (int j = 0; j < count; j++) {

			w = sceneImage.m_object_list[j].m_w * 1.5;
			h = sceneImage.m_object_list[j].m_h * 1.5;

			x_offset = sceneImage.m_object_list[j].m_w * 0.25;
			y_offset = sceneImage.m_object_list[j].m_h * 0.25;
			x = sceneImage.m_object_list[j].m_x - x_offset;
			y = sceneImage.m_object_list[j].m_y - y_offset;
			yolo_score = sceneImage.m_object_list[j].m_score * 100;
			if (sceneImage.m_object_list[j].m_label == "red")
			{
				color = "red";
			}
			else if (sceneImage.m_object_list[j].m_label == "blue")
			{
				color = "blue";
			}
			else
			{
				color = "last";
			}
			Mat img;  Rect rect = Rect(x, y, w, h);
			cout << "----" << x << "y " << y << "d " << w << "ddf  " << h << endl;
			if (makeRectSafe(rect, Size(416, 416)) == false)
				continue;

			if (w < 3 || h < 3) { cout << "8888888888continue" << endl; continue; }

			img = src(rect);

			// imshow("ssss",img);
	 //	waitKey(1);
			bool flag = lightSearch(img, res_lt, res_rt, res_lb, res_rb, res_conf, res_type, isRed);
			cout << "flag" << flag << endl;
			cout << "res_conf" << res_conf << endl;
			if (flag == false || res_conf < 0.5) {
				continue;
			}
			if (res_conf > max_conf) {
				max_conf = res_conf;
				idx = j;

			}


		}



		if (!(res_rb.x == 0 || res_rb.y == 0) && idx >= 0) {

			if (sceneImage.m_object_list[idx].m_x - x_offset <= 0) {

				res_lt.x = res_lt.x;
				res_rt.x = res_rt.x;
				res_lb.x = res_lb.x;
				res_rb.x = res_rb.x;
			}

			else {

				res_lt.x = res_lt.x + sceneImage.m_object_list[idx].m_x - x_offset;
				res_rt.x = res_rt.x + sceneImage.m_object_list[idx].m_x - x_offset;
				res_lb.x = res_lb.x + sceneImage.m_object_list[idx].m_x - x_offset;
				res_rb.x = res_rb.x + sceneImage.m_object_list[idx].m_x - x_offset;
			}

			if (sceneImage.m_object_list[idx].m_y - y_offset <= 0)
			{
				res_lt.y = res_lt.y;
				res_rt.y = res_rt.y;
				res_lb.y = res_lb.y;
				res_rb.y = res_rb.y;
			}


			else {
				res_lt.y = res_lt.y + sceneImage.m_object_list[idx].m_y - y_offset;
				res_rt.y = res_rt.y + sceneImage.m_object_list[idx].m_y - y_offset;
				res_lb.y = res_lb.y + sceneImage.m_object_list[idx].m_y - y_offset;
				res_rb.y = res_rb.y + sceneImage.m_object_list[idx].m_y - y_offset;

			}

		}




		finalPoints.finalpoints.push_back(res_lb);
		finalPoints.finalpoints.push_back(res_lt);
		finalPoints.finalpoints.push_back(res_rt);
		finalPoints.finalpoints.push_back(res_rb);



		if (res_type == 1) {
			finalPoints.is_small_armor = false;

		}
		else {
			finalPoints.is_small_armor = true;
		}
		if (res_lt.x != 0 && res_lb.y != 0 && res_rt.x != 0 && res_rb.x != 0)
		{
			finalPoints.CODE = 0;
		}
		cout << __LINE__ << endl;
		/*
			line(src, res_lt,res_rt, CV_RGB(0, 0, 255), 1);
			line(src, res_rt, res_rb, CV_RGB(0, 0, 255), 1);
			line(src, res_rb,res_lb, CV_RGB(0, 0, 255), 1);
			line(src, res_lb,res_lt, CV_RGB(0, 0, 255), 1);
		   imshow("reererrer",src);
			 waitKey(0);
		*/
		return finalPoints;
	}



};
