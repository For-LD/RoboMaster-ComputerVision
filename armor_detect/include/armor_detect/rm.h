#ifndef RM_H_
#define RM_H_
#include<opencv2/opencv.hpp>
#include<darknet.h>
#include<memory>
#include<thread>
#include<vector>
#include<string>
#include<iostream>
#include"image_opencv.hpp"
struct Object
{
	float m_score;
	std::string m_label;
	std::string m_ts;
	int m_x;
	int m_y;
	int m_w;
	int m_h;
};
struct SceneImage
{
	cv::Mat m_image;
	std::vector<Object> m_object_list;
	std::string m_ts;
};

class RM_YOLO
{
public:
	RM_YOLO(std::string& datacfg, std::string& cfg, std::string& weight, float thresh = 0.5);
	//RM_YOLO(){}
	~RM_YOLO() {}
	bool init(void);
	void detect(cv::Mat& img, std::string ts, SceneImage& sceneImage);
private:
	int m_gpu_index;
	std::string m_datacfg;
	std::string m_cfg;
	std::string m_weight;
	float m_thresh;
	float m_hier_thresh;
	float m_nms;
	list* m_options;
	char* m_name_list;
	char** m_names;
	image** m_alphabet;
	network* m_net;
	std::string m_save_path;
	cv::Mat m_cv_image;
};

void cleanData(SceneImage& sceneImage);
#endif
