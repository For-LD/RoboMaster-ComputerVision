#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>
#include <fstream>
#include <streambuf>
using namespace std;

#define DEBUG
// test robot

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
	sort(pts.begin(), pts.end(), sortByY);  // higher prior
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
float calcShapeSimi(cv::Point lt, cv::Point rt, cv::Point lb, cv::Point rb, float radio)
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

float calcPositionScore(cv::Mat& gray, cv::Point lt, cv::Point rt, cv::Point lb, cv::Point rb)
{
	int h = gray.rows;
	int w = gray.cols;
	float dx = ((lt.x + rb.x) / 2 - w / 2) / (float)w;
	float dy = ((lt.y + rb.y) / 2 - h / 2) / (float)h;
	float corr = 1 - pow(dx, 2.f) - pow(dy, 2.f);
	return corr;
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
	float scale = 2;
	cv::resize(gray, gray, cv::Size(), scale, scale);
	//
	cv::Ptr<cv::MSER> mser = cv::MSER::create(5, 5, 100);
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Rect> box;
	mser->detectRegions(gray, contours, box);

	// #ifdef DEBUG
	//   // display
	//   cv::Mat mserImg;
	//   cv::resize(img, mserImg, cv::Size(), scale, scale);
	//   for (int i = 0; i < contours.size(); i++)
	//   {
	//     cv::ellipse(mserImg, cv::fitEllipse(contours[i]), cv::Scalar(0, 255, 0));
	//     cv::imwrite("./mser.jpg", mserImg);
	//   }
	//   cv::imwrite("./gray.jpg", gray);
	// #endif

	std::vector<cv::Point> topVec;
	std::vector<cv::Point> bottomVec;
	for (int i = 0; i < contours.size(); i++)
	{
		cv::Point top, bottom;
		getEndPoint(contours[i], top, bottom);
		topVec.push_back(top);
		bottomVec.push_back(bottom);
	}

	vector<float> typeHWRatio = { 2, 3.3 };
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

int main()
{
	vector<string> imgList;
	FILE* fin = fopen("./ls.txt", "r");
	int n;
	fscanf(fin, "%d", &n);
	for (int i = 0; i < n; i++) {
		char imgPath[1024];
		fscanf(fin, "%s", imgPath);
		imgList.push_back(imgPath);
	}
	fclose(fin);

	for (int i = 0; i < imgList.size(); i++) {
		string imgPath = imgList[i];

		cv::Mat img = cv::imread(imgPath);
		cv::Point res_lt, res_rt, res_lb, res_rb;
		float res_conf;
		int res_type;
		bool isRed;

		bool flag = lightSearch(img, res_lt, res_rt, res_lb, res_rb, res_conf, res_type, isRed);
		if (!flag) {
			cout << imgPath << " not found" << endl;
			continue;
		}

		// display
		float scale = 4;
		cv::Mat showImg;
		cv::resize(img, showImg, cv::Size(), scale, scale);
		cv::Scalar color;
		if (isRed) {
			color = cv::Scalar(255, 0, 0);
		}
		else {
			color = cv::Scalar(0, 0, 255);
		}
		if (res_type == 0) {
			cv::line(showImg, cv::Point(0, 0), cv::Point(0, 20), color, 3);
		}
		else {
			cv::line(showImg, cv::Point(0, 0), cv::Point(20, 0), color, 3);
		}
		cv::circle(showImg, res_lt * scale, 1, color, 2);
		cv::circle(showImg, res_rt * scale, 1, color, 2);
		cv::circle(showImg, res_lb * scale, 1, color, 2);
		cv::circle(showImg, res_rb * scale, 1, color, 2);
		// cout << "res: " << res_conf << endl
		//      << res_lt << endl
		//      << res_rt << endl
		//      << res_lb << endl
		//      << res_rb << endl;

		string outPath = imgPath.replace(7, 5, "res");
		cv::imwrite(outPath, showImg);
	}

	return 0;
}
