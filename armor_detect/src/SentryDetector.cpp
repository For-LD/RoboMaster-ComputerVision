#include "SentryDetector.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <queue>
#include <algorithm>
#include "opencv2/opencv.hpp"
#include "math.h"

//#define SHOW_DEBUG_IMG
//#define COUT_LOG

#define SafeRect(rect, max_size) {if (makeRectSafe(rect, max_size) == false) continue;}

using namespace std;
using namespace cv;

RNG s_rng(12345);




void SentryDetector::setImage(const cv::Mat& img) {
	_src = img;

	_g.create(_src.size(), CV_8UC1);
	_max_color = cv::Mat(_src.size(), CV_8UC1, cv::Scalar(0));

	//    Mat srcGray;
	//    cv::cvtColor(img, srcGray,CV_RGB2GRAY);
	//    imshow("gray",srcGray);
	//    waitKey(0);
	//    threshold(srcGray,_max_color,210,255,CV_THRESH_BINARY);
#ifdef SHOW_DEBUG_IMG
	imshow("src", _src);
	waitKey(0);
#endif

	std::vector<Mat> channels;
	split(_src, channels);

	Mat red, blue;
	blue = channels[0];
	red = channels[2];
	_g = channels[1];


	uchar* ptr_g = _g.data;
	uchar* ptr_max_color = _max_color.data;
	uchar* ptr_b = blue.data;
	uchar* ptr_r = red.data;
	uchar* ptr_rd = red.data + _src.cols * _src.rows;


	if (_para.enemy_color == rred) {
		for (; ptr_r != ptr_rd; ++ptr_r, ++ptr_max_color, ++ptr_b, ++ptr_g) {
			uchar r = *ptr_r;
			uchar b = *ptr_b;
			uchar g = *ptr_g;
			if (r - b > _para.red_armor_threshold)

				*ptr_max_color = 255;
		}
	}
	else {
		for (; ptr_r != ptr_rd; ++ptr_r, ++ptr_max_color, ++ptr_b, ++ptr_g) {
			uchar r = *ptr_r;
			uchar b = *ptr_b;
			uchar g = *ptr_g;
			if (b - r > _para.blue_armor_threshold)
				*ptr_max_color = 255;
		}
	}



#ifdef SHOW_DEBUG_IMG

	cv::imshow("_max_color", _max_color);  //er zhi hua
	waitKey(0);
#endif
}

RotatedRect SentryDetector::adjustRRect(const RotatedRect& rect) {
	const Size2f& s = rect.size;

	if (s.width < s.height)
	{
		double angle = rect.angle; //dedao jiaodu
		angle = 90 - angle; //jiaodu zuocha
		angle = angle <= 0 ? angle + 180 : angle; //sheding
		if (angle - 90 < 0)
			angle = abs(angle - 90);
		else angle = -abs(angle - 90);

		if (angle < -90) {
			angle = 180 + angle;
		}
		return RotatedRect(rect.center, Size2f(s.width, s.height), angle);
	}
	else {
		double angle = rect.angle + 90.0; //dedao jiaodu
		angle = 90 - angle; //jiaodu zuocha
		angle = angle < 0 ? angle + 180 : angle; //sheding
		if (angle - 90 < 0)
			angle = abs(angle - 90);
		else angle = -abs(angle - 90);
		return RotatedRect(rect.center, Size2f(s.height, s.width), angle);
	}


}

cv::RotatedRect SentryDetector::boundingRRect(cv::RotatedRect& left, cv::RotatedRect& right) {
	const Point& pl = left.center, & pr = right.center;
	Point2f center = (pl + pr) / 2.0;
	cv::Size2f wh_l = left.size;
	cv::Size2f wh_r = right.size;
	float width = POINT_DIST(pl, pr);// - (wh_l.width + wh_r.width) / 2.0;
	float height = std::max(wh_l.height, wh_r.height);
	//float height = (wh_l.height + wh_r.height) / 2.0;
	float angle = std::atan2(right.center.y - left.center.y, right.center.x - left.center.x);
	return RotatedRect(center, Size2f(width, height), angle * 180 / CV_PI);
}

int SentryDetector::templateDist(const cv::Mat& img) {
	int dist = 0;
	const uchar threshold_value = _para.min_light_gray;//210-15
	int total_piexl = img.rows * img.cols;

	const uchar* p1 = _binary_template_small.data;//liangbianbai zhongjianhei


	const uchar* p2 = img.data;
	//    for(int i = 0; i < total_piexl;  ++i, p1+=1, p2+=3)
	//    {
	//        uchar v = (*p2) > threshold_value ? 255 : 0;
	//        dist += (*p1) == v ? 0 : 1;
	//    }
	std::vector<Mat> channels;
	Mat red, blue, g;
	split(img, channels);
	red = channels[2];
	g = channels[1];
	blue = channels[0];
	uchar* p = g.data;
	if (_para.enemy_color == rred)
		uchar* p = red.data;
	else
		uchar* p = blue.data;

	for (int i = 0; i < total_piexl; ++i, p1++, p++)
	{
		uchar v = (*p) > threshold_value ? 255 : 0;
		dist += (*p1) == v ? 0 : 1;
	}

	return dist;
}


void SentryDetector::findContour(vector<RotatedRect>& final_rects, std::vector<double>& score, s_LeftRightLight& leftright_rect) {

	//de dao erzhi tu he huidutu
	vector<vector<Point2i> > contours_br;
	vector<Vec4i> hierarchy;
	//er zhi tu zhao lun kuo
	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3)); //第一个参数MORPH_RECT表示矩形的卷积核，当然还可以选择椭圆形的、交叉型的
   // dilate(_max_color,_max_color,element);
	findContours(_max_color, contours_br, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	//bianli lunkuode zhizhen
	vector<vector<Point2i> >::const_iterator it = contours_br.begin();



	Mat contous = Mat::zeros(_max_color.size(), CV_8UC3);
	for (int i = 0; i < contours_br.size(); i++) {
		Scalar color = Scalar(s_rng.uniform(0, 255), s_rng.uniform(0, 255), s_rng.uniform(0, 255));  //������ɫ

		  //  putText(contous,to_string(i), contours_br[i][0], CV_FONT_HERSHEY_COMPLEX_SMALL, 0.5, color, 1);
		drawContours(contous, contours_br, i, color, 1, 8, vector<Vec4i>(), 0, Point());//}

	}


#ifdef SHOW_DEBUG_IMG
	imshow("lunkuo", contous);
	waitKey(0);

#endif

	vector<RotatedRect> rectss;

	while (it != contours_br.end())//bianli
	{
		RotatedRect rrect = minAreaRect(*it);
		Rect rect = cv::boundingRect(*it);
		if (rect.height <_para.min_light_height || rect.height>_para.max_light_height) {//min_light_height=8  xiaoyu8 paichu
			++it;
			// cout<<"min_height:"<<rect.height<<endl;
			continue;
		}
		rrect = adjustRRect(rrect);//xuan zhuan
		double angle = abs(rrect.angle); //dedao jiaodu

	   // float delta_angle = abs(angle - 90);

		if (angle < 45)  //xiaoyu30du
		{


			rectss.push_back(rrect);
			Point2f vertices[4];
			rrect.points(vertices);//结果框出来
			for (int i = 0; i < 4; i++)
			{
				line(contous, vertices[i], vertices[(i + 1) % 4], CV_RGB(0, 255, 0), 1);


			}
			Scalar color = Scalar(s_rng.uniform(0, 255), s_rng.uniform(0, 255), s_rng.uniform(0, 255));
			putText(contous, to_string(angle), vertices[0], CV_FONT_HERSHEY_COMPLEX_SMALL, 0.5, color, 1);



		}
		else {
			// cout<<"rect angle:"<<angle<<endl;
		}

#ifdef SHOW_DEBUG_IMG
		imshow("lunkuo", contous);
		waitKey(0);
#endif
		++it;

	}
	Mat s;
	_src.copyTo(s);


	sort(rectss.begin(), rectss.end(), [](const RotatedRect& rest1, const RotatedRect& rest2)
		{
			return rest1.center.x < rest2.center.x;
		});

	for (size_t i = 0; i < rectss.size(); ++i) {

		RotatedRect& rect_i = rectss[i];
		const Point& center_i = rect_i.center;
		float xi = center_i.x;
		float yi = center_i.y;

		for (size_t j = i + 1; j < rectss.size(); j++)
		{
			RotatedRect& rect_j = rectss[j];
			const Point& center_j = rect_j.center;
			float xj = center_j.x;
			float yj = center_j.y;
			float angle_j = rect_j.angle;
			float angle_i = rect_i.angle;
			float delta_x = abs(xj - xi);
			float delta_angle = abs(rect_j.angle - rect_i.angle);

			float delta_h = abs(yi - yj);
			if (delta_x<_para.max_light_delta_h && delta_x >_para.min_light_delta_h && delta_angle < _para.max_light_delta_angle && delta_h < _para.max_light_delta_v)        //zuoyou dengzhu zai shuiping weizhishang de zuixiao chazhi
			{

				//                cout<<"delta_angle:"<<delta_angle<<endl;

				//                cout<<"delta_h:"<<delta_h<<"delta_x"<<delta_x<<endl;

				RotatedRect rect = boundingRRect(rect_i, rect_j);//zuoyoude juxing



				Point2f vertices[4];
				rect.points(vertices);//结果框出来
				for (int i = 0; i < 4; i++)
				{
					line(s, vertices[i], vertices[(i + 1) % 4], CV_RGB(0, 255, 0), 1);
				}
#ifdef SHOW_DEBUG_IMG
				cout << "------------------------------------------------------------------" << endl;
				imshow("kuang", s);
				waitKey(0);
#endif
				//                rect = adjustRRect(rect);//xuan zhuan
				//                double angle = rect.angle; //dedao jiaodu
				//                angle = 90 - angle; //jiaodu zuocha
				//                angle = angle < 0 ? angle + 180 : angle; //sheding


								// the contour must be near-vertical  //||chui zhi




				//---------------------------------------------------------------------------------------------------------------------------
								// rotate the area

				double w = rect.size.width;
				double h = rect.size.height;
				int lamp_width = max((int)w / 12, 1);
				cv::Rect bounding_roi = rect.boundingRect();//bao wei
				bounding_roi.x -= w / 8;
				bounding_roi.width += w / 4;
				SafeRect(bounding_roi, _src.size());

				Point2f new_center = rect.center - Point2f(bounding_roi.x, bounding_roi.y);
				Mat roi_src = _src(bounding_roi);//src is source ,
				//imshow("_src",_src);
				Mat rotation = getRotationMatrix2D(new_center, rect.angle, 1);
				Mat rectify_target;
				cv::warpAffine(roi_src, rectify_target, rotation, bounding_roi.size());   //fangshe bianhuan
				// get the black board of the armor//
				cv::Point ul = Point(std::max(int(new_center.x - (w / 2.0)) + 1, 0), std::max((int)(new_center.y - h / 2.0), 0));
				cv::Point dr = Point(new_center.x + w / 2.0, new_center.y + h / 2.0);
				Rect roi_black = Rect(cv::Point(ul.x, ul.y), cv::Point(dr.x, dr.y));
				// get the left lamp and right lamp of the armor
				Rect roi_left_lamp = Rect(Point(max(0, ul.x - lamp_width), ul.y), Point(max(rectify_target.cols, ul.x), dr.y));
				Rect roi_right_lamp = Rect(Point(dr.x, ul.y), Point(min(dr.x + lamp_width, rectify_target.cols), dr.y));

				SafeRect(roi_left_lamp, rectify_target.size());
				SafeRect(roi_right_lamp, rectify_target.size());
				SafeRect(roi_black, rectify_target.size());

				// valid the gray value of black area
				Mat black_part;
				rectify_target(roi_black).copyTo(black_part);


				//                imshow("black_part",black_part);
				//                waitKey(0);

				Mat gray_mid;
				cv::cvtColor(black_part, gray_mid, CV_RGB2GRAY);
				//                  imshow("gray_mid",gray_mid);
				//                  waitKey(0);
				//                int black_side = min(_para.min_light_delta_h / 2, 4);//35/2,4
				//                Mat gray_mid_black(Size(roi_black.width - black_side * 2, roi_black.height), CV_8UC1);
				////cout<<"black_side:"<<black_side<<endl;









				Point p1(roi_left_lamp.x, roi_left_lamp.y);
				Point p2(roi_right_lamp.x + roi_right_lamp.width, roi_right_lamp.y + roi_right_lamp.height);
				// get the whole area of armor
				Rect armor_rect(p1, p2);
				SafeRect(armor_rect, rectify_target.size());
				Mat armor = rectify_target(armor_rect);
				cv::Size cur_size;
				cur_size = _binary_template_small.size();
				resize(armor, armor, cur_size);
#ifdef SHOW_DEBUG_IMG
				imshow("armor", armor);
				waitKey(0);
#endif
				//-------------------------------------------------------------------------------------------------------------------



				double template_dist_threshold = 0.3;      //--------------------------待调

			   // cv::Size cur_size;

			  //  resize(rRect, template_pip, cur_size);
			//    cv::cvtColor(template_pip, template_pip,CV_RGB2GRAY);
			 //   imshow("temo",template_pip);
			 //   waitKey(0);

				double dist = templateDist(armor);
				dist = dist / (cur_size.width * cur_size.height);



				if (dist > template_dist_threshold)//0.25 or 0.5
				{
#ifdef COUT_LOG

					cout << "refused 2: dist: " << dist << endl;
#endif
					continue;//---------
				}


				double delta_height;
				delta_height = abs(rect_i.size.height - rect_j.size.height);
				if (delta_height > _para.max_light_delta_height) {
#ifdef COUT_LOG

					cout << "refused 3: height_delta: " << delta_height << endl;
#endif
					continue;//---------
				}


				final_rects.push_back(rect);

				leftright_rect.lrect.push_back(rect_i);
				leftright_rect.rrect.push_back(rect_j);

				rect = adjustRRect(rect);
				double anglerect = abs(rect.angle);
				double delta_a = abs(rect_i.angle - rect_j.angle);


				// cout<<"rect.angle:"<<delta_a<<endl;

				double dd;
				dd = rect_i.size.area() - rect_j.size.area();
				double ss = (1 - dist) * 100 - delta_a * 4 - delta_height * 2;//-abs(dd);
#ifdef COUT_LOG
				cout << "delta_a:" << delta_a << endl;
				cout << "delta_height:" << delta_height << endl;
				cout << "score " << ss << endl;
#endif
				score.push_back(ss);

				//  }

#ifdef SHOW_DEBUG_IMG
				Point2f verticess[4];
				rect.points(verticess);//结果框出来
				for (int i = 0; i < 4; i++)
				{
					line(s, verticess[i], verticess[(i + 1) % 4], CV_RGB(0, 255, 0), 1);
				}
				rect_i.points(verticess);//结果框出来
				for (int i = 0; i < 4; i++)
				{
					line(s, verticess[i], verticess[(i + 1) % 4], CV_RGB(0, 255, 0), 1);
				}
				rect_j.points(verticess);//结果框出来
				for (int i = 0; i < 4; i++)
				{
					line(s, verticess[i], verticess[(i + 1) % 4], CV_RGB(0, 255, 0), 1);
				}
#endif
			}
		}


	}






#ifdef SHOW_DEBUG_IMG
	//  imshow("rects",s);
	 // waitKey(0);
#endif



}

cv::RotatedRect SentryDetector::chooseTarget(vector<RotatedRect>& final_rects, std::vector<double>& score, s_LeftRightLight& leftright_rect, cv::RotatedRect& lrect, cv::RotatedRect& rrect) {

	if (final_rects.size() < 1)//number is not enough
	{
		return RotatedRect();
	}
	//cout<<"size"<<final_rects.rects.size()<<endl;

	Mat ii;
	_src.copyTo(ii);

	double max_score = 0;
	int ret_idx = -1;
	const double small_armor_wh_threshold = 2.5;
	for (size_t i = 0; i < final_rects.size(); ++i)
	{
		RotatedRect& rect = final_rects[i];

		double w = rect.size.width * 1280 / 416;
		double h = rect.size.height * 1024 / 416;

		double wh_ratio = w / h;
		double max_wh_ratio = 6.7, min_wh_ratio = 1;

		rect = adjustRRect(rect);

		if (wh_ratio > max_wh_ratio || wh_ratio < min_wh_ratio) {
#ifdef COUT_LOG
			cout << "refused :wh_ratio:" << wh_ratio << endl;
#endif
			continue;//paichu bufuhe
		}

#ifdef SHOW_DEBUG_IMG
		Point2f vertices[4];
		rect.points(vertices);//结果框出来
		for (int i = 0; i < 4; i++)
		{
			line(ii, vertices[i], vertices[(i + 1) % 4], CV_RGB(0, 255, 0), 1);
		} imshow("rects", ii);
		waitKey(0);
#endif


		double s;
		s = score[i];
		if (s > max_score)
		{
			max_score = s;
			ret_idx = i;

		}


	}



	if (ret_idx == -1) {
		return RotatedRect();
	}
	else {
		RotatedRect frect;
		frect = final_rects[ret_idx];
		lrect = leftright_rect.lrect[ret_idx];
		rrect = leftright_rect.rrect[ret_idx];
		//cout<<"fjsidfjsoifosfe::"<<max_score<<endl;
		//if(frect.size.width/frect.size.height<small_armor_wh_threshold) _is_small_armor=true;


		//cout<<"width/height:"<<frect.size.width/frect.size.height<<endl;

		return frect;
	}

	return RotatedRect();

}


s_FinalPoints SentryDetector::getTargetAera(Mat& src, double distance) {

	cv::FileStorage setting_fs(_filename, cv::FileStorage::READ);
	if (distance > 0 && distance < 4) {
		read_near(setting_fs);
		setting_fs.release();
	}
	else if (distance >= 4) {
		read_far(setting_fs);
		setting_fs.release();
	}
	else {
		read_default(setting_fs);
		setting_fs.release();
	}

	_para = armor_p;

	vector<RotatedRect>  final_rects;
	s_LeftRightLight leftright_rect;
	vector<double>  score;


	Mat small_template_img = imread(_template_path);
	resize(small_template_img, _binary_template_small, Size(60, 25));



	//resize(src,src,Size(416,416));




	Mat img;
	src.copyTo(img);

	setImage(img);
	findContour(final_rects, score, leftright_rect);





	RotatedRect final_rect, lrect, rrect;

	final_rect = chooseTarget(final_rects, score, leftright_rect, lrect, rrect);

	Point2f lup, ldown, rup, rdown;
	Point2f leftvertices[4], rightvertices[4];
	lrect.points(leftvertices);
	rrect.points(rightvertices);
	s_FinalPoints finalPoints;


	lup = (leftvertices[1] + leftvertices[2]) / 2.0;
	ldown = (leftvertices[0] + leftvertices[3]) / 2.0;


	rup = (rightvertices[1] + rightvertices[2]) / 2.0;
	rdown = (rightvertices[0] + rightvertices[3]) / 2.0;

	finalPoints.finalpoints.push_back(ldown);
	finalPoints.finalpoints.push_back(lup);
	finalPoints.finalpoints.push_back(rup);
	finalPoints.finalpoints.push_back(rdown);

#ifdef SHOW_DEBUG_IMG
	Point2f vertices[4];
	final_rect.points(vertices);//结果框出来
	for (int i = 0; i < 4; i++)
	{
		line(src, vertices[i], vertices[(i + 1) % 4], CV_RGB(0, 255, 0), 1);
	}

	lrect.points(vertices);//结果框出来
	for (int i = 0; i < 4; i++)
	{
		line(src, vertices[i], vertices[(i + 1) % 4], CV_RGB(255, 0, 0), 1);
	}
	rrect.points(vertices);//结果框出来
	for (int i = 0; i < 4; i++)
	{
		line(src, vertices[i], vertices[(i + 1) % 4], CV_RGB(255, 0, 0), 1);
	}
#endif
	double ratio;
	ratio = ((rrect.center.x - lrect.center.x) * 1280) / (((rrect.size.height + lrect.size.height) / 2) * 1024);
	// cout<<"rrect.center.x-lrect.center.x:"<<(rrect.center.x-lrect.center.x)<<"rrect.size.height+lrect.size.height"<<rrect.size.height+lrect.size.height<<endl;
	// cout<<"ratio:"<<ratio<<endl;
	if (ratio > 4.1) {
		finalPoints.is_small_armor = false;
	}


	if (lup.x != 0 && ldown.x != 0 && lup.y != 0 && rup.x != 0) {
		finalPoints.CODE = 0;
	}

#ifdef SHOW_DEBUG_IMG
	line(src, lup, rup, CV_RGB(0, 0, 255), 1);
	line(src, rup, rdown, CV_RGB(0, 0, 255), 1);
	line(src, rdown, ldown, CV_RGB(0, 0, 255), 1);
	line(src, ldown, lup, CV_RGB(0, 0, 255), 1);
	imshow("reererrer", src);
	waitKey(0);
#endif
	return finalPoints;
}

