/*
 * @Description:
 * @Author: Chinwong_Leung
 * @Github: https://github.com/ZhanhongLiang
 * @Date: 2019-09-05 21:23:44
 * @LastEditors: Chinwong_Leung
 * @LastEditTime: 2019-11-24 20:22:12
 */
#define BUFF_DEBUG

#ifdef BUFF_DEBUG

#include <iostream>
#include <string>
#include <thread>
//#include "../../include/rune/ImageConsProd.h"
#include "../../include/filter/Filter.hpp"
#include "../../include/rune/AngleSolver.hpp"
#include "../../include/rune/Rune.h"
using namespace std;
string file = "C:/Users/25212/Documents/RM_BUFF/config/param_config.xml";
cv::Mat cameraMatrix1;
cv::Mat distCoeffs1;

int main() {
  detect::kalmanfilter::KalmanFilter kalmanFilter_(2, 4);
  detect::armor::Buff_Detector buff_Detector;
  detect::angle_solver::Flag flag1;
  detect::angle_solver::Settings settings_file(file);
  detect::angle_solver::AngleSolver angle(&settings_file, &flag1);

  angle.GetCamerParam();

  // Settings settings_file(file);
  // AngleSolver angleSlover(settings_file);
  // Settings settings("D:/gitrepo/Robobigsymbol/src/rune/rune/Rune.xml");
  // angleSlover.SetTargetSize(122, 233, AngleSolverFactory::TARGET_RUNE);
  cv::Mat frame, bin_img;
  cv::RotatedRect finalArmor_;
  int tSelect = 3;
  int status = 1;
  //   int mode = 0;
  //   float angle_x;
  //   float angle_y;
  //   float dist;
  //   Point2f kalmanFilterPoint = Point2f(0, 0);
  //   float anti_range = 0.2;
  //   // int mode = 1;

  //   VideoCapture capture;
  //   Point2f pt = Point2f(0, 0);
  //   Point2f anti_kalmanPoint(0, 0);
  //   int frameCount = 0;
  //   capture.open("C:/Users/25212/Documents/RM_BUFF/doc/red.avi");
  //   if (!capture.isOpened()) {
  //     std::cout << "could not find the mp4!!" << std::endl;
  //   } else {
  //     while (true) {
  //       capture >> frame;
  //       if (frame.empty()) {
  //         std::cout << "Not the frame!!" << std::endl;
  //         break;
  //       }
  //       // buff_Detector.Detect(frame, tSelect, pt, status, finalArmor_);
  //       buff_Detector.DetectII(frame, tSelect, pt, status);
  //       // std::cout << "pt.x: " << pt.x << std::endl;
  //       // std::cout << "pt.y: " << pt.y << std::endl;
  //       // angle.OnePointSolvePnP(pt, angle_x, angle_y);
  //       // angle.FourPointSolvePnP(finalArmor_, angle_x, angle_y, dist);
  //       // kalmanFilterPoint = kalmanFilter_.run(pt.x, pt.y);
  //       // if ((pt.x + anti_range * (pt.x - kalmanFilterPoint.x)) <=
  //       frame.cols ||
  //       //     (pt.x + anti_range * (pt.x - kalmanFilterPoint.x)) >= 0 ||
  //       //     (pt.y + anti_range * (pt.y - kalmanFilterPoint.y)) <=
  //       frame.rows ||
  //       //     (pt.y + anti_range * (pt.y - kalmanFilterPoint.y)) >= 0) {
  //       //   if (abs(pt.x - kalmanFilterPoint.x) > 3 &&
  //       //       abs(pt.y - kalmanFilterPoint.y > 3)) {
  //       //     anti_kalmanPoint.x = pt.x + anti_range * (pt.x);
  //       //     anti_kalmanPoint.y = pt.y + anti_range * (pt.y);
  //       //   } else {
  //       //     anti_kalmanPoint.x = pt.x;
  //       //     anti_kalmanPoint.y = pt.y;
  //       //   }
  //       // } else {
  //       //   anti_kalmanPoint.x = pt.x;
  //       //   anti_kalmanPoint.y = pt.y;
  //       // }
  //       // std::cout << "anti_kalmanPoint.x:" << anti_kalmanPoint.x <<
  //       std::endl;
  //       // std::cout << "anti_kalmanPoint.y:" << anti_kalmanPoint.y <<
  //       std::endl;
  //       // angle.GetRuneAngle(finalArmor);
  //       // angleSlover(buff_Detector.finalArmor2Angle,
  //       // angleSlover.TARGET_RUNE ,angle_x, angle_y, dist);
  //       frameCount++;
  //       std::cout << "frameCount:" << frameCount << std::endl;
  //     }
  //   }
  // }

  // // int main()
  // // {
  // // 	int fd = 1;
  // // 	string ParamConfig =
  // // "D:\\gitrepo\\Robobigsymbol\\src\\rune\\rune\\camera-RM3-06-640.xml";
  // // 	Settings setting_file(ParamConfig);

  // // 	ImageConsProd imagePro(&setting_file, fd);
  // // 	std::thread t1(&ImageConsProd::ImageProducer, imagePro);
  // // 	std::thread t2(&ImageConsProd::ImageConsumer, imagePro);

  // // 	t1.join();
  // // 	t2.join();

  // // 	//getchar();
  // // 	//system("pause");
  // // 	return 0;
  // // }
  // #endif  // BUFF_DEBUG