#include "trans_pose.h"
#include "math.h"
#include <iostream>


using namespace std;


void PShooting::getShootAngles(Input_coordinate& input, OutAngles& out_angle)
{
	double Bx = (input.x - X0) / 1000;
	double By = (input.y - Y0) / 1000;
	double Bz = (input.z - Z0) / 1000;
	//double tmp=(0.02612667*input.z+45.7933)/1000;
	//By=By-tmp;
	//double tmp=(0.028498*input.z+56.43)/1000;
	//By=By-tmp;

	//double tmpy=(0.01621*input.z-9.33-5)/1000;
	//By=By-tmpy;
	//double tmpx=(0.001525*input.z+23.49)/1000;
	//Bx=Bx+tmpx;
	double tmpy = (0.001875 * input.z + 19.66) / 1000;
	By = By - tmpy;
	double tmpx = (0.00869 * input.z + 9.48 + 5) / 1000;
	Bx = Bx + tmpx;

	if (input.v0 <= 0)
	{
		out_angle.yaw_angle = NULL;
		out_angle.pitch_angle = NULL;
		cout << "invalid intput of v0" << endl;
	}
	else
	{
		out_angle.yaw_angle = (atan(Bx / Bz)) * 180 / Pi;

		double a = By;
		double b = sqrt(pow(Bx, 2) + pow(Bz, 2));
		double angle = 0;
		angle = (asin((a * pow(input.v0, 2) - g * pow(b, 2)) / (pow(input.v0, 2) * sqrt(pow(a, 2) + pow(b, 2)))) - atan(-a / b)) / 2;
		out_angle.pitch_angle = -angle * 180 / Pi;
	}

}





