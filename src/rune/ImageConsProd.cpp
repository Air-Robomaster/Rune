/*
 * @Description: 
 * @Author: Chinwong_Leung
 * @Github: https://github.com/ZhanhongLiang
 * @Date: 2019-10-23 22:31:25
 * @LastEditors: Chinwong_Leung
 * @LastEditTime: 2019-11-07 19:47:08
 */

#include "../../include/rune/ImageConsProd.h"

#define COUT
#define CAP_VIDEO
#define RUNE_CAP_VIDEO
#define SERIAL_DEBUG

#define VIDEO_WIDTH  1280
#define VIDEO_HEIGHT 720
#define V_OFFSET 304
#define EXPOSURE (3000 + 2000)// 5000
#define SATURATION_BLUE 60
#define SATURATION_RED 100

#define EXPOSURE_RUNE 2000
#define RUNE_WIDTH  888
#define RUNE_HEIGHT 1000


#ifdef SERIAL_DEBUG
volatile unsigned int mode = 4;
volatile unsigned int sentry_mode = 1;
volatile unsigned int base_mode = 0;
#else
volatile unsigned int mode;
volatile unsigned int sentry_mode;
volatile unsigned int base_mode;
#endif

#define BUFFER_SIZE 1
volatile unsigned int prdIdx;
volatile unsigned int csmIdx;

struct ImageData
{
	Mat img;
	unsigned int frame;
};
ImageData Data[BUFFER_SIZE];

ImageConsProd::ImageConsProd(Settings * _settings, int fd_car)
{
	settings = _settings;
	fd2car = fd_car;
}

ImageConsProd::~ImageConsProd()
{
	
}

/**
 * @brief: 
 * @param {type} 
 * @return: 
 * @author: Chinwong_Leung
 */
void ImageConsProd::ImageProducer()
{
	VideoCapture cap("C:\\Users\\25212\\Documents\\Buff\\Buff.mp4");
	if (!cap.isOpened())
	{
		return;
		std::cout << "NO file!!" << std::endl;
	}
	while (1)
	{
		while (prdIdx - csmIdx >= BUFFER_SIZE);//等待图片产生的数目大于处理图片时，等待；
		cap >> Data[prdIdx % BUFFER_SIZE].img;
		Data[prdIdx % BUFFER_SIZE].frame++;
		++prdIdx;
	}
}

/**
 * @brief: 
 * @param {type} 
 * @return: 
 * @author: Chinwong_Leung
 */
void ImageConsProd::ImageConsumer()
{
    Buff_Detector find;
	Settings & setting = *settings;//?为什么要用setting
	float x = 100.0;
	float y = 100.0;
	float z = 100.0;
	float barrer_y = 100.0;
	//const char* file_path = "D:\\gitrepo\\Robobigsymbol\\src\\rune\\rune\\camera-RM3-06-640.xml";
	string file_path = "D:\\gitrepo\\Robobigsymbol\\src\\rune\\rune\\camera-RM3-06-640.xml";
	AngleSolver angleSolver(file_path/*, x, y, z, barrer_y*/);
	FileStorage fs(setting.intrinsic_file_480, FileStorage::READ);//读取文件

    //Mat cam_matrix_480, distortion_coeff_480;
	//fs["Camera_Matrix"] >> cam_matrix_480;
	//fs["Distortion_Coefficients"] >> distortion_coeff_480;
	
    Mat src;
    int frame_num = 0;
	if (!fs.isOpened())
	{
		cout << "Could not open the configuration file: \"" << setting.intrinsic_file_480 << "\"" << endl;
		return;
	}

    while (1)
	{
		while (prdIdx - csmIdx == 0);//判断他们两个不等于0
		Data[csmIdx%BUFFER_SIZE].img.copyTo(src);
		frame_num = Data[csmIdx % BUFFER_SIZE].frame;
		++csmIdx;

		RotatedRect rect;
		float angle_x = 0.0, angle_y = 0.0;
		float dist = 0.0;
		bool flag_ = true;
		//double send_data[3] = { 0 };

		if (mode == 4)
		{
			Point2f Pt = Point2f(0, 0);
			int status = 0;
			//这个是将之前solver_480传到angle_slover中，实现类中的传递数据；
			//angle_slover.SetSolver(&solver_480);
			//装甲板识别线程
			find.Detect(src, mode, Pt, status);
			angleSolver.GetRuneAngle(find.finalArmor2Angle);
			//rect.angle = find.finalArmor2Angle.angle;
			//rect.center = find.finalArmor2Angle.center;
			//rect.size = find.finalArmor2Angle.size;
			//std::cout << "AngleRect" << rect.angle << std::endl;
			//std::cout << "AngleCenter" << rect.center << std::endl;
			//std::cout << "AngleSize" << rect.size << std::endl;
			//angleSolver.GetRuneAngle(rect);

			//差的是rect的，角度解析线程,这里面解算会出现一系列问题
			// if (angle_slover.GetAngle(find.finalArmor2Angle, AngleSolverFactory::TargetType::TARGET_RUNE, angle_x, angle_y, dist) == true)
			// {
			// 	send_data[0] = (angle_x + offset_anlge_x) * 100;
			// 	send_data[1] = (angle_y + offset_anlge_y) * 100;
			// 	//send_data[3] = dist;
			// }
			// else
			// {
			// 	std::cout << "No Angles!!" << std::endl;
			// }
			// std::cout << "angle_x" << send_data[0] << std::endl;
			// std::cout << "angle_y" << send_data[1] << std::endl;
			// std::cout << "dist" << send_data[2] << std::endl;
		}
	}
}
