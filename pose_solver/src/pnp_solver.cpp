#include "pnp_solver.h"


// 本类用于快速解决PNP问题，顺带解决空间绕轴旋转以及图像系、相机系、世界系三系坐标投影问题
// 调用顺序：
// 1.初始化本类
// 2.调用SetCameraMatrix(),SetDistortionCoefficients()设置好相机内参数与镜头畸变参数
// 3.向Points3D，Points2D中添加一一对应的特征点对
// 4.调用Solve()方法运行计算
// 5.从RoteM, TransM, W2CTheta等属性中提出结果
//

PNPSolver::PNPSolver(const std::string& config_path)
{
	//初始化输出矩阵
	vector<double> rv(3), tv(3);
	cv::Mat rvec(rv), tvec(tv);

	FileStorage fs(config_path, FileStorage::READ); //换成xml效果一样，yaml文件格式相对更直观

	CS.fx = (double)fs["fx"];
	CS.fy = (double)fs["fy"];
	CS.u0 = (double)fs["u0"];
	CS.v0 = (double)fs["v0"];
	CS.k_1 = (double)fs["k1"];
	CS.k_2 = (double)fs["k2"];
	CS.p_1 = (double)fs["p1"];
	CS.p_2 = (double)fs["p2"];
	CS.k_3 = (double)fs["k3"];
	fs.release();

	SetCameraMatrix(CS.fx, CS.fy, CS.u0, CS.v0);
	SetDistortionCoefficients(CS.k_1, CS.k_2, CS.p_1, CS.p_2, CS.k_3);

}
PNPSolver::PNPSolver(double fx, double fy, double u0, double v0, double k_1, double  k_2, double  p_1, double  p_2, double k_3)
{
	//初始化输出矩阵
	vector<double> rv(3), tv(3);
	cv::Mat rvec(rv), tvec(tv);
	SetCameraMatrix(fx, fy, u0, v0);
	SetDistortionCoefficients(k_1, k_2, p_1, p_2, k_3);
}

PNPSolver::~PNPSolver()
{
}

int PNPSolver::Solve(std::vector<Point2f>& rectPoint, int armorType)
{
	//数据校验
	if (camera_matrix.cols == 0 || distortion_coefficients.cols == 0)
	{
		printf("ErrCode:-1,相机内参数或畸变参数未设置！\r\n");
		return -1;
	}
	Point3f ldown, lup, rup, rdown;
	double light_width, light_height;
	if (armorType == 0)
	{
		light_width = 135;
		light_height = 55;
	}
	else if (armorType == 1)
	{
		light_width = 230;
		light_height = 55;
	}
	ldown = Point3f(-light_width / 2, light_height / 2, 0);
	lup = Point3f(-light_width / 2, -light_height / 2, 0);
	rup = Point3f(light_width / 2, -light_height / 2, 0);
	rdown = cv::Point3f(light_width / 2, light_height / 2, 0);

	Points3D.clear();

	Points3D.push_back(ldown);
	Points3D.push_back(lup);
	Points3D.push_back(rup);
	Points3D.push_back(rdown);


	Points2D = rectPoint;

	if (Points3D.size() != Points2D.size())
	{
		printf("ErrCode:-2，3D点数量与2D点数量不一致！\r\n");
		return -2;
	}

	solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, CV_ITERATIVE);

	double rm[9];
	RoteM = cv::Mat(3, 3, CV_64FC1, rm);
	Rodrigues(rvec, RoteM);
	double r11 = RoteM.ptr<double>(0)[0];
	double r12 = RoteM.ptr<double>(0)[1];
	double r13 = RoteM.ptr<double>(0)[2];
	double r21 = RoteM.ptr<double>(1)[0];
	double r22 = RoteM.ptr<double>(1)[1];
	double r23 = RoteM.ptr<double>(1)[2];
	double r31 = RoteM.ptr<double>(2)[0];
	double r32 = RoteM.ptr<double>(2)[1];
	double r33 = RoteM.ptr<double>(2)[2];
	TransM = tvec;

	//计算出相机坐标系的三轴旋转欧拉角，旋转后可以转出世界坐标系。
	//旋转顺序为z、y、x
	double thetaz = atan2(r21, r11) / CV_PI * 180;
	double thetay = atan2(-1 * r31, sqrt(r32 * r32 + r33 * r33)) / CV_PI * 180;
	double thetax = atan2(r32, r33) / CV_PI * 180;

	//相机系到世界系的三轴旋转欧拉角，相机坐标系照此旋转后可以与世界坐标系完全平行。
	//旋转顺序为z、y、x
	Theta_C2W.z = thetaz;
	Theta_C2W.y = thetay;
	Theta_C2W.x = thetax;

	//计算出世界系到相机系的三轴旋转欧拉角，世界系照此旋转后可以转出相机坐标系。
	//旋转顺序为x、y、z
	Theta_W2C.x = -1 * thetax;
	Theta_W2C.y = -1 * thetay;
	Theta_W2C.z = -1 * thetaz;


	/*************************************此处计算出相机坐标系原点Oc在世界坐标系中的位置**********************************************/

	/***********************************************************************************/
	/* 当原始坐标系经过旋转z、y、x三次旋转后，与世界坐标系平行，向量OcOw会跟着旋转 */
	/* 而我们想知道的是两个坐标系完全平行时，OcOw的值 */
	/* 因此，原始坐标系每次旋转完成后，对向量OcOw进行一次反相旋转，最终可以得到两个坐标系完全平行时的OcOw */
	/* 该向量乘以-1就是世界坐标系下相机的坐标 */
	/***********************************************************************************/

	//提出平移矩阵，表示从相机坐标系原点，跟着向量(x,y,z)走，就到了世界坐标系原点
	double tx = tvec.ptr<double>(0)[0];
	double ty = tvec.ptr<double>(0)[1];
	double tz = tvec.ptr<double>(0)[2];

	//x y z 为唯一向量在相机原始坐标系下的向量值
	//也就是向量OcOw在相机坐标系下的值
	double x = tx, y = ty, z = tz;
	Position_OwInC.x = x;
	Position_OwInC.y = y;
	Position_OwInC.z = z;
	//进行三次反向旋转
	CodeRotateByZ(x, y, -1 * thetaz, x, y);
	CodeRotateByY(x, z, -1 * thetay, x, z);
	CodeRotateByX(y, z, -1 * thetax, y, z);


	//获得相机在世界坐标系下的位置坐标
	//即向量OcOw在世界坐标系下的值
	Position_OcInW.x = x * -1;
	Position_OcInW.y = y * -1;
	Position_OcInW.z = z * -1;

	return 0;
}
