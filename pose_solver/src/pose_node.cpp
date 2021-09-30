#include <pose_solver/pose_node.h>


PoseNode::PoseNode()
{
	trans_flag = false;
	pub_cnt = 0;
	pose_trans_in = {};
	ctrl_angle = {};

	pub = nh.advertise<pose_solver::pose>("PoseMsg", 1);
	status_sub = nh.subscribe("statusMsg", 1, &PoseNode::statusCallback, this);
	sub = nh.subscribe("armorMsg", 1, &PoseNode::poseCallback, this);
	//service = nh.advertiseService("providePose", &PoseNode::sendCtrlData,this);
	std::thread t(&PoseNode::spin, this);
	t.detach();
}

PoseNode::~PoseNode()
{

}

void PoseNode::poseCallback(const pose_solver::rect_data::ConstPtr& msg)
{
	try
	{
		ROS_INFO("poseCallback...");
		rect_array.clear();
		armor_type = msg->armor_type;
		rect_array = msg->point_data;

		rect_point.clear();
		convert2Coordinate(rect_array, rect_point);
		trans_flag = true;
		/*cout<<"header.seq:"<<msg->header.seq<<endl;
		cout<<"header.stamp:"<<msg->header.stamp<<endl;
		cout<<"header.frame_id:"<<msg->header.frame_id<<endl;
		cout<<"msg.angle:"<<msg->angle<<endl;
		for(std::vector<float>::iterator it = array.begin(); it != array.end(); ++it)
		{
			std::cout << *it << " ";
		}
		std::cout << std::endl;*/
		/*for(int i = 0;i < rect_point.size(); i++)
		{
			cout << "rect_point[i].x:" << rect_point[i].x << " rect_point[i].y:" << rect_point[i].y <<endl;
		}*/
	}
	catch (std::exception& e)
	{
		ROS_ERROR("receive failed...");
		cout << "error:" << e.what() << endl;
		return;
	}
}

bool PoseNode::spin()
{
	PNPSolver ps("model/pose_config.yaml");

	PShooting sh;
	Point2f po[4];

	pose_solver::pose p_msg;
	ros::Rate loop_rate(200);
	while (nh.ok())
	{
		if (trans_flag == true)
		{
			//ROS_INFO("PnP_solver success...");
//////////////////////////////////////////////////////////////////////////
			if (ps.Solve(rect_point, armor_type) == 0)
			{
				posePoint = ps.Position_OwInC;
				double yaw_angle = atan(fabs(posePoint.x) / fabs(posePoint.z)) * 180 / 3.14159;
				double pitch_angle = atan(fabs(posePoint.y) / fabs(posePoint.z)) * 180 / 3.14159;
				if (posePoint.x < 0)    yaw_angle = -yaw_angle;
				if (posePoint.y < 0)    pitch_angle = -pitch_angle;
				cout << "yaw_angle:" << yaw_angle << endl;
				cout << "pitch_angle:" << pitch_angle << endl;
				cout << "armor_type:" << armor_type << endl;
				cout << "x:" << posePoint.x << endl;
				cout << "y:" << posePoint.y << endl;
				cout << "z:" << posePoint.z << endl;

				pose_trans_in.x = posePoint.x;
				pose_trans_in.y = posePoint.y;
				pose_trans_in.z = posePoint.z;
				pose_trans_in.v0 = 22;
				sh.getShootAngles(pose_trans_in, ctrl_angle);
				cout << "after transport pose" << endl;
				cout << "yaw_angle:" << ctrl_angle.yaw_angle << endl;
				cout << "pitch_angle:" << ctrl_angle.pitch_angle << endl;

			}
			trans_flag = false;
			//////////////////////////////////////////////////////////////////////////

			if (fabs(posePoint.z) > 5500)
			{
				ROS_WARN("the emeny is too far!!!");
			}
			else
			{
				pub_cnt++;
				p_msg.header.seq = pub_cnt;
				p_msg.header.stamp = ros::Time::now();
				p_msg.pitch_angle = ctrl_angle.pitch_angle;
				p_msg.yaw_angle = ctrl_angle.yaw_angle;
				p_msg.distance = (int)posePoint.z;
				pub.publish(p_msg);
				cout << "p_msg.distance:" << p_msg.distance << endl;
				ROS_INFO("publish pose success!");
			}

		}
		else
		{
			ROS_WARN("No rectangle subsribed!");
		}
		//usleep(100000);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return true;
}

void PoseNode::convert2Coordinate(std::vector<float>& input_vec, std::vector<cv::Point2f>& point_vec)
{
	cv::Point2f p1, p2, p3, p4;

	p1.x = input_vec[0];
	p1.y = input_vec[1];
	p2.x = input_vec[2];
	p2.y = input_vec[3];
	p3.x = input_vec[4];
	p3.y = input_vec[5];
	p4.x = input_vec[6];
	p4.y = input_vec[7];
	point_vec.clear();
	point_vec.push_back(p1);
	point_vec.push_back(p2);
	point_vec.push_back(p3);
	point_vec.push_back(p4);

}

/*bool PoseNode::getPoseData(std::vector<Point2f>& input_vec,PoseData& _pose)
{

	////////////////////////////////////////////////
		caculate pose data;
	//////////////////////////////////////////////

	float pitch_angle = 1.2;
	float yaw_angle = 2.4;
	float distance = 18;

	_pose.pitch_angle = pitch_angle;
	_pose.yaw_angle = yaw_angle;
	_pose.distance = distance;
	//pose_vec.push_back(_pose);

	return true;
}*/

void PoseNode::statusCallback(const pose_solver::status::ConstPtr& msg)
{
	try
	{
		ROS_INFO("statusCallback...");
		cout << "task_mode:" << msg->task_mode << endl;

	}
	catch (std::exception& e)
	{
		ROS_ERROR("receive status failed...");
		cout << "error:" << e.what() << endl;
		return;
	}
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "pose_solver");

	PoseNode ps;
	ros::spin();

	return 0;
}
