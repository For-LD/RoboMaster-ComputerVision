#pragma once
#define X0 0 //coordinate x of gun in camera coordinate system
//#define Y0 43.27 //coordinate y of gun in camera coordinate system
#define Z0 30.73 //coordinate z of gun in camera coordinate system
#define Y0 40.91
//#define Z0 0
#define Pi 3.1415926;

//input coordinate of target point and initial velocity v0
struct Input_coordinate {
	float x;
	float y;
	float z;
	float v0;
};


struct OutAngles {
	float yaw_angle;
	float pitch_angle;
};

class PShooting
{
private:
	Input_coordinate input;
	OutAngles out_angle;
	double g = 9.8;

public:
	PShooting() {};
	void getShootAngles(Input_coordinate& input, OutAngles& out_angle); //each rotation angle is based on the positive direction of Z axis of gun coordinate system.

};
