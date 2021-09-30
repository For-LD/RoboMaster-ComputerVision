#include <armor_detect/armor_detect_node.hpp>

//#define SHOW_DETECT_RESULT

DetectNode::DetectNode()
{
	///////////////////////////////////////
	//detect_node parameter initialization
	cv::FileStorage debug_show_fs("src/armor_detect/model/debug.yaml", cv::FileStorage::READ);
	debug_show_flag = (int)debug_show_fs["show_debug"];
	save_img_flag = (int)debug_show_fs["save_img"];
	/////////////////////////////////
	pub_cnt = 0;
	WORKING_MODE = 0;
	solver = { 0 };


	//detectInit();

	//status_sub = nh_.subscribe("statusMsg", 1, &DetectNode::statusCallback,this);
	pose_sub = nh_.subscribe("PoseMsg", 1, &DetectNode::solverCallback, this);
	rect_pub_ = nh_.advertise<armor_detect::rect_data>("armorMsg", 10);
	std::thread t1(&DetectNode::spin, this);
	t1.detach();
}


bool DetectNode::spin()
{
	std::string datacfg = "src/armor_detect/model/voc.data";//
	std::string cfg = "src/armor_detect/model/yolov2-tiny-voc.cfg";//
	std::string weight = "src/armor_detect/model/test_yolov2-tiny-voc_final.weights";//
	std::string config_file_name = "src/armor_detect/model/opencv_detect_config.xml";
	std::string sentry_config_file_name = "src/armor_detect/model/sentry_detect_config.xml";
	std::string detect_template_path = "src/armor_detect/model/template.bmp";
	std::string template_path = "src/armor_detect/model/s_template.bmp";

	RM_YOLO rm_detector(datacfg, cfg, weight, 0.15);//
	rm_detector.init();//
	SceneImage sceneImage;//

	ArmorDetector armor_detector(config_file_name, detect_template_path);
	SentryDetector sentry_detector(sentry_config_file_name, template_path);
	armor_detect::rect_data send_msg;
	Mat image;
	Mat s_image;
	ExecuteStatus exe_status = DETECTING;
	YOLOStatus yolo_status = YOLODETECT;   //YOLODETECT
	DH_CAMERA camera(0, 8000);
	//camera.setAeState(false);
	//camera.setExposureTime(10000);
	ros::Rate loop_rate(100);
	int s_x = 0, s_y = 0;

	//VideoCapture cap("data/a.mp4");
	while (nh_.ok())
	{
		if (WORKING_MODE == 0)
		{
			cout << "==================" << endl;
			//image = imread("data/resize.jpg");2
			int ret = camera.ProcGetImage(image);

			//cap >> image;
			if (image.empty() || ret != 0)
			{
				ROS_ERROR("camera grab image exception! restart...");
				camera.closeCamera();
				camera.initCamera();
				camera.initGrabStream();
				continue;
			}

			if ((image.rows != 416) || (image.cols != 416))
			{
				cout << "resize..." << endl;
				resize(image, image, Size(416, 416));
			}
			double time = what_time_is_it_now();
			if (exe_status == DETECTING)
			{
				if (yolo_status == YOLODETECT)
				{
					cleanData(sceneImage);
					rm_detector.detect(image, "test", sceneImage);
					cout << "yolo detect done" << endl;
					if (sceneImage.m_object_list.size() <= 0) ROS_WARN("no target...");
				}




			}

			else if (exe_status == TRACKING)
			{
				ROS_WARN("tracking...");
				Object tmp_obj;

				tmp_obj.m_score = 0;
				tmp_obj.m_label = "last";
				tmp_obj.m_ts = "test";
				tmp_obj.m_x = (light_points.finalpoints[0].x + light_points.finalpoints[1].x) / 2;
				tmp_obj.m_y = (light_points.finalpoints[1].y + light_points.finalpoints[2].y) / 2;
				tmp_obj.m_w = (light_points.finalpoints[2].x + light_points.finalpoints[3].x) / 2 - tmp_obj.m_x;
				tmp_obj.m_h = (light_points.finalpoints[0].y + light_points.finalpoints[3].y) / 2 - tmp_obj.m_y;
				int rectOffset_w = (int)(tmp_obj.m_w / 3);
				int rectOffset_h = (int)(tmp_obj.m_h / 3);
				tmp_obj.m_x = ((tmp_obj.m_x - rectOffset_w) < 0 ? 0 : (tmp_obj.m_x - rectOffset_w));
				tmp_obj.m_y = ((tmp_obj.m_y - rectOffset_h) < 0 ? 0 : (tmp_obj.m_y - rectOffset_h));
				tmp_obj.m_w = ((tmp_obj.m_w + rectOffset_w * 2) > 416 ? 416 : (tmp_obj.m_w + rectOffset_w * 2));
				tmp_obj.m_h = ((tmp_obj.m_h + rectOffset_h * 2) > 416 ? 416 : (tmp_obj.m_h + rectOffset_h * 2));

				tmp_obj.m_w = ((tmp_obj.m_w + tmp_obj.m_x) > 416 ? (416 - tmp_obj.m_x) : tmp_obj.m_w);
				tmp_obj.m_h = ((tmp_obj.m_h + tmp_obj.m_y) > 416 ? (416 - tmp_obj.m_y) : tmp_obj.m_h);



				if (yolo_status == YOLODETECT) {
					cleanData(sceneImage);
					sceneImage.m_object_list.push_back(tmp_obj);
				}
				if (yolo_status == NOYOLODETECT) {
					s_image = image(Rect(tmp_obj.m_x, tmp_obj.m_y, tmp_obj.m_w, tmp_obj.m_h));
					s_x = tmp_obj.m_x;
					s_y = tmp_obj.m_y;


				}
			}

			if (sceneImage.m_object_list.size() > 0 || yolo_status == NOYOLODETECT)
			{

				if (exe_status == TRACKING)	last_distance = solver.distance;
				else				last_distance = 2000;

				if (yolo_status == YOLODETECT) {
					Mat d_img;
					image.copyTo(d_img);
					if (Attack_color == 3)	sceneImage.m_ts = "blue";
					else if (Attack_color == 4)	sceneImage.m_ts = "red";

					light_points = armor_detector.getTargetAera(d_img, sceneImage, 2000);
				}
				if (yolo_status == NOYOLODETECT) {


					cout << "no yolo detect" << endl;

					if (light_points.CODE == -1)
					{
						image.copyTo(s_image);

					}
					else {
						if (s_image.empty()) {
							image.copyTo(s_image);
						}
					}
					imshow("s_image", s_image);

					s_light_points = sentry_detector.getTargetAera(s_image, last_distance);
					light_points.CODE = s_light_points.CODE;
					light_points.is_small_armor = s_light_points.is_small_armor;
					if ((s_image.rows != 416) || (s_image.cols != 416)) {
						for (int i = 0; i < 4; i++) {

							s_light_points.finalpoints[i].x = ((s_light_points.finalpoints[i].x + s_x) > 416 ? 416 : (s_light_points.finalpoints[i].x + s_x));
							s_light_points.finalpoints[i].y = ((s_light_points.finalpoints[i].y + s_y) > 416 ? 416 : (s_light_points.finalpoints[i].y + s_y));
						}

					}
					light_points.finalpoints = s_light_points.finalpoints;

				}

				cout << "light_points.CODE:" << light_points.CODE << endl;
				if (light_points.CODE == 0)
				{
					pub_cnt++;
					send_msg.header.seq = pub_cnt;
					send_msg.header.stamp = ros::Time::now();
					//send_msg.header.frame_id = "1234";
					if (light_points.is_small_armor == true)
					{
						send_msg.armor_type = 0;
					}
					else
					{
						send_msg.armor_type = 1;
					}
					send_msg.point_data.resize(8);
					vector<cv::Point2f> p = light_points.finalpoints;
					send_msg.point_data[0] = p[0].x;
					send_msg.point_data[1] = p[0].y;
					send_msg.point_data[2] = p[1].x;
					send_msg.point_data[3] = p[1].y;
					send_msg.point_data[4] = p[2].x;
					send_msg.point_data[5] = p[2].y;
					send_msg.point_data[6] = p[3].x;
					send_msg.point_data[7] = p[3].y;

					rect_pub_.publish(send_msg);
					ROS_INFO("publish rect success!");
					//exe_status = TRACKING;
					exe_status = DETECTING;
				}
				else
				{
					exe_status = DETECTING;
				}
			}
			else
			{
				exe_status = DETECTING;
			}


			if (debug_show_flag == 1)
			{
				std::cout << "sceneImage.size:" << sizeof(sceneImage) << std::endl;
				std::cout << "execute time: " << what_time_is_it_now() - time << std::endl;
				for (int i = 0; i < sceneImage.m_object_list.size(); i++)
				{
					std::cout << "label: " << sceneImage.m_object_list[i].m_label << std::endl;
					std::cout << "score: " << sceneImage.m_object_list[i].m_score << std::endl;
					std::cout << "x: " << sceneImage.m_object_list[i].m_x << std::endl;
					std::cout << "y: " << sceneImage.m_object_list[i].m_y << std::endl;
					std::cout << "w: " << sceneImage.m_object_list[i].m_w << std::endl;
					std::cout << "h: " << sceneImage.m_object_list[i].m_h << std::endl;
					std::cout << std::endl;
					if (sceneImage.m_object_list[i].m_label != "last")
					{
						Rect yolo_rect(sceneImage.m_object_list[i].m_x, sceneImage.m_object_list[i].m_y, sceneImage.m_object_list[i].m_w, sceneImage.m_object_list[i].m_h);
						cv::rectangle(image, yolo_rect, Scalar(255, 0, 0), 2, LINE_8, 0);
					}

				}
				std::cout << "===========" << std::endl;
				if (light_points.CODE == 0)
				{
					for (int i = 0; i < 4; i++)
					{
						line(image, light_points.finalpoints[i], light_points.finalpoints[(i + 1) % 4], CV_RGB(255, 255, 255), 1);
					}
				}
				imshow("veiw", image);
				waitKey(1);
			}

		}
		else
		{
			ROS_WARN("No image subscribed!/image is empty!");
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	return true;
}

void DetectNode::statusCallback(const armor_detect::status::ConstPtr& msg)
{
	try
	{
		ROS_INFO("statusCallback...");
		cout << "WORKING_MODE:" << msg->ctrl_mode << endl;
		cout << "Attack_color:" << msg->task_mode << endl;
		WORKING_MODE = msg->ctrl_mode;

	}
	catch (std::exception& e)
	{
		ROS_ERROR("receive status failed...");
		cout << "error:" << e.what() << endl;
		return;
	}
}

void DetectNode::solverCallback(const armor_detect::pose::ConstPtr& msg)
{
	try
	{
		ROS_WARN("solverMsgCallback...");
		solver.distance = msg->distance;
		solver.yaw_angle = msg->yaw_angle;
		solver.pitch_angle = msg->pitch_angle;
		cout << "solver.distance:" << solver.distance << endl;
		cout << "solver.yaw_angle:" << solver.yaw_angle << endl;
		cout << "solver.pitch_angle:" << solver.pitch_angle << endl;

	}
	catch (std::exception& e)
	{
		ROS_ERROR("receive solverMsg feedback failed...");
		cout << "error:" << e.what() << endl;
		return;
	}
}


DetectNode::~DetectNode()
{

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "armor_detect");
	DetectNode obj;
	ros::spin();
	//obj.spin();
	return 0;
}




