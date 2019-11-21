/*
 * @Description:
 * @Author: Chinwong_Leung
 * @Github: https://github.com/ZhanhongLiang
 * @Date: 2019-10-23 22:02:27
 * @LastEditors: Chinwong_Leung
 * @LastEditTime: 2019-11-16 18:34:30
 */

#ifndef ANGLESOLVER_H
#define ANGLESOLVER_H
#define MINECODE
#include "../header/Header.h"
#include "./Rune.h"
/**
 * @brief:
 * @param {type}
 * @return:
 * @author: Chinwong_Leung
 */

namespace detect {

namespace angle_solver {

#ifdef MINECODE

struct OtherParam {
  int angle_pitch;
  int angle_yaw;
  OtherParam() {
    angle_pitch = 0;
    angle_yaw = 0;
  }
};

class Settings;  //这个必须加，要不然报错，调用类的时候必须先声明类；

class Flag;  //这个必须加，要不然报错，调用类的时候必须先声明类；

class AngleSolver {
 public:
  //初始化程序
  AngleSolver(Settings *_settings, Flag *_flag) : settings_(_settings) {
    settings_ = _settings;
    flag = _flag;
    GetP3Point(0, Point2f(0, 0));
    Mat(objectPoints).convertTo(object_point_mat, CV_32F);
    Mat rvec(3, 1, DataType<double>::type);
    Mat tvec(3, 1, DataType<double>::type);
  }

  void SetSetings(Settings *_settings) { settings_ = _settings; }

  //大符角度解算
  void GetRuneAngle();
  void GetCamerParam();
  //得到世界坐标和图像的坐标点，这个是两个已知条件
  void GetP3Point(uint mode, Point2f offset_point);
  void GetP2Point(RotatedRect &rect, vector<cv::Point2f> &target2D,
                  Point2f offset_point);

  void GetRuneRect();

 public:
  Mat cameraMatrix, distCoeffs;
  Mat object_point_mat;
  vector<Point3f> objectPoints;
  vector<Point2f> projectedPoints;
  vector<Point2f> imagePoints;
  Mat rvec;
  Mat tvec;
  float height_world = 60.0;
  float overlap_dist = 100000.0;
  float barrel_ptz_offset_x = -0;
  float barrel_ptz_offset_y = -0;  // mm   + ptz is up barrel

  float ptz_camera_x = 0;     // +left
  float ptz_camera_y = 52.5;  // + camera is  ptz
  float ptz_camera_z = -135;  //-225;     // - camera is front ptz
  float scale = 0.99f;        // is calc distance scale not use pnp ,test

  // Kalman1 kalman;
  int f_ = 1500;
  float rune_h;
  float rune_distance;
  Settings *settings_;
  Flag *flag;
};
#endif

//扇子的类库,自己的位置和决定自己是否要更改切换位置
class Flag {
 public:
  struct FlagArmorParam {
    double flagArmorParamWidth;
    double flagArmorParamHeight;
    FlagArmorParam() {
      flagArmorParamWidth = 230.0;
      flagArmorParamHeight = 160.0;
    }
  };
  FlagArmorParam flagArmorParam;
};

/**
 * @brief:
 * @param {type}
 * @return:
 * @author: Chinwong_Leung
 */
class Settings {
 public:
  Settings(const std::string &filename) {
    FileStorage settingsReadFile(filename, FileStorage::READ);
    // FileStorage settingsWriteFile(filename, FileStorage::WRITE);
    FileRead(settingsReadFile);
    // FileWrite(settingsWriteFile);
    // settingsWriteFile.release();
    settingsReadFile.release();
  }
  //~Settings();
  void FileRead(
      const FileStorage &fs) {  // fs["rune_height"] >> runeParam.rune_height;
    // fs["rune_width"] >> runeParam.rune_width;

    fs["intrinsic_file_480"] >> intrinsic_file_480;
    fs["intrinsic_file_720"] >> intrinsic_file_720;

    fs["bullet_speed"] >> bullet_speed;
    fs["scale_z"] >> scale_z;
  }

  void FileWrite(FileStorage &fs) const {
    cvWriteComment(*fs, "\nParameter for Camera", 0);
    fs << "intrinsic_file_480" << intrinsic_file_480;
    fs << "intrinsic_file_720" << intrinsic_file_720;
    // fs << "exposure_time" << exposure_time;

    // for system mode
    cvWriteComment(*fs,
                   "\nParameter for Vision System Mode, 0(default) means for "
                   "armor detection mode, 1 means for rune system mode",
                   0);
    fs << "mode" << mode;

    cvWriteComment(*fs, "\nBullet speed (m/s)", 0);
    fs << "bullet_speed" << bullet_speed;

    cvWriteComment(*fs, "\nScale factor of Z", 0);
    fs << "scale_z" << scale_z;
  }

  struct RuneParam {
    float rune_height;
    float rune_width;

    RuneParam() {
      rune_height = 122;
      rune_width = 233;
    }
  };

 public:
  RuneParam runeParam;
  int mode = 0;
  std::string intrinsic_file_480;
  std::string intrinsic_file_720;
  double bullet_speed = 22.0;
  double scale_z = 1.0;
  // double scale_z_480 = 1.0;
  // std::string &filename;
};
}  // namespace angle_solver
}  // namespace detect
//官方代码修改
#ifndef MINECODE

#endif

#endif  // ANGLESOLVER_H
