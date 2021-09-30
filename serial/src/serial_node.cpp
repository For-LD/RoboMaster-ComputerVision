#include <serial/serial_node.h>


SerialNode::SerialNode()
{
	pd = { 0,0,0 };
	trans_flag = false;
	WORKING_MODE = 0;
	ATTACK_COLOR = 0;
	_serial.openPort();

	pub = nh.advertise<serial::status>("statusMsg", 1);
	sub = nh.subscribe("PoseMsg", 1, &SerialNode::serialCallback, this);

	std::thread t1(&SerialNode::spin, this);
	std::thread t2(&SerialNode::sendCtrlData, this);
	t1.detach();
	t2.detach();
}


SerialNode::~SerialNode()
{

}

void SerialNode::serialCallback(const serial::pose::ConstPtr& msg)
{
	try
	{
		ROS_INFO("serial callback...");
		//cout<<"msg_seq:"<<msg->header.seq;
		//cout<<"header.stamp:"<<msg->header.stamp<<endl;
		pd.distance = msg->distance;
		pd.yaw_angle = msg->yaw_angle;
		pd.pitch_angle = msg->pitch_angle;
		trans_flag = true;
		cout << "pd.distance:" << pd.distance << endl;
		cout << "pd.yaw_angle:" << pd.yaw_angle << endl;
		cout << "pd.pitch_angle:" << pd.pitch_angle << endl;
	}
	catch (std::exception& e)
	{
		ROS_ERROR("serialCallback failed!");
		cout << "error:" << e.what() << endl;
	}
}


bool SerialNode::receiveRobotData()
{
	/*receive and unpack the robot status data*/
	rec = _serial.receive();
	WORKING_MODE = rec.ctrl_mode;
	ATTACK_COLOR = rec.task_mode;
	cout << "WORKING_MODE:" << WORKING_MODE << endl;
	cout << "ATTACK_COLOR:" << ATTACK_COLOR << endl;
	return true;

}

bool SerialNode::spin()
{
	serial::status s_msg;
	ros::Rate loop_rate(1);
	while (nh.ok())
	{
		//receiveRobotData();
		if (WORKING_MODE == 1)
		{
			//publish status msg;
			s_msg.ctrl_mode = WORKING_MODE;
			s_msg.task_mode = ATTACK_COLOR;

			pub.publish(s_msg);
			ROS_INFO("publish robot status successed!");
		}
		//usleep(500000);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return true;
}

void SerialNode::sendCtrlData()
{
	while (1)
	{
		if (trans_flag == true && abs(pd.yaw_angle) < 20 && abs(pd.yaw_angle) < 12)
		{
			ts.pitch_angle = pd.pitch_angle;
			ts.yaw_angle = pd.yaw_angle;
			_serial.send(ts);
			ROS_WARN("send angles success!");
			trans_flag = false;
		}
		else
		{
			//cout<<"waitting ctrl data..."<<endl;
		}
		usleep(5000);
	}
}

int main(int argc, char** argv)
{
	/* code */
	ros::init(argc, argv, "serial_node");
	SerialNode a;
	ros::spin();

	return 0;
}
