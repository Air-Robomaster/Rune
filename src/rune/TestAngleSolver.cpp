#include "TestAngleSolver.h"
#include "math.h"
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

std::vector<cv::Point3f> AngleSolverParam::POINT_3D_OF_ARMOR_BIG =
    std::vector<cv::Point3f>{

        cv::Point3f(-117, -30, 0),  // tl
        cv::Point3f(117, -30, 0),   // tr
        cv::Point3f(117, 30, 0),    // br
        cv::Point3f(-117, 30, 0)    // bl
    };
std::vector<cv::Point3f> AngleSolverParam::POINT_3D_OF_RUNE =
    std::vector<cv::Point3f>{
        cv::Point3f(-370, -220, 0), cv::Point3f(0, -220, 0),
        cv::Point3f(370, -220, 0),  cv::Point3f(-370, 0, 0),
        cv::Point3f(0, 0, 0),       cv::Point3f(370, 0, 0),
        cv::Point3f(-370, 220, 0),  cv::Point3f(0, 220, 0),
        cv::Point3f(370, 220, 0)};

std::vector<cv::Point3f> AngleSolverParam::POINT_3D_OF_ARMOR_SMALL =
    std::vector<cv::Point3f>{
        cv::Point3f(-65, -35, 0),  // tl
        cv::Point3f(65, -35, 0),   // tr
        cv::Point3f(65, 35, 0),    // br
        cv::Point3f(-65, 35, 0)    // bl
    };

AngleSolver::AngleSolver() {
  for (int ll = 0; ll <= 3; ll++)
    target_nothing.push_back(cv::Point2f(0.0, 0.0));
}

AngleSolver::AngleSolver(const AngleSolverParam& AngleSolverParam) {
  _params = AngleSolverParam;
  _cam_instant_matrix = _params.CAM_MATRIX.clone();
  for (int ll = 0; ll <= 3; ll++)
    target_nothing.push_back(cv::Point2f(0.0, 0.0));
}

void AngleSolver::init(const AngleSolverParam& AngleSolverParam) {
  _params = AngleSolverParam;
  _cam_instant_matrix = _params.CAM_MATRIX.clone();
}

void AngleSolver::setTarget1(std::vector<cv::Point2f> objectPoints,
                             int objectType) {
  if (objectType == 1 || objectType == 2) {
    if (angle_solver_algorithm == 0 || angle_solver_algorithm == 2) {
      angle_solver_algorithm = 1;
      cout << "algorithm is reset to PNP Solution" << endl;
    }
    point_2d_of_armor = objectPoints;
    if (objectType == 1)
      enemy_type = 0;
    else
      enemy_type = 1;
    return;
  }
}

void AngleSolver::setTarget(const cv::Point2f Center_of_armor,
                            int objectPoint) {
  if (angle_solver_algorithm == 1 || angle_solver_algorithm == 2) {
    angle_solver_algorithm = 0;
    cout << "algorithm is reset to One Point Solution" << endl;
  }
  centerPoint = Center_of_armor;
  if (objectPoint == 3 || objectPoint == 4)
    is_shooting_rune = 1;
  else {
    is_shooting_rune = 0;
    _rune_compensated_angle = 0;
  }
}

void AngleSolver::getTarget2dPoinstion(const cv::RotatedRect& rect,
                                       vector<cv::Point2f>& target2d,
                                       const cv::Point2f& offset) {
  target2d.clear();
  Point2f vertices[4];
  rect.points(vertices);
  Point2f lu, ld, ru, rd;
  sort(vertices, vertices + 4,
       [](const Point2f& p1, const Point2f& p2) { return p1.x < p2.x; });
  if (vertices[0].y < vertices[1].y) {
    lu = vertices[0];
    ld = vertices[1];
  } else {
    lu = vertices[1];
    ld = vertices[0];
  }
  if (vertices[2].y < vertices[3].y) {
    ru = vertices[2];
    rd = vertices[3];
  } else {
    ru = vertices[3];
    rd = vertices[2];
  }

  target2d.clear();
  target2d.push_back(lu + offset);
  target2d.push_back(ru + offset);
  target2d.push_back(rd + offset);
  target2d.push_back(ld + offset);
}

/*
void AngleSolver::setTarget(const std::vector<cv::Point2f> runePoints)
{
angle_solver_algorithm = 2;
point_2d_of_rune = runePoints;
}
*/

#ifdef DEBUG

void AngleSolver::showPoints2dOfArmor() {
  cout << "the point 2D of armor is" << point_of_armor << endl;
}

void AngleSolver::showTvec() {
  cv::Mat tvect;
  transpose(_tVec, tvect);
  cout << "the current _tVec is:" << endl << tvect << endl;
}

void AngleSolver::showEnermyType() {
  cout << "EnermyType:" << enemy_type << endl;
}

void AngleSolver::showEDistance() {
  cout << "  _euclideanDistance is  " << _euclideanDistance / 1000 << "m"
       << endl;
}

void AngleSolver::showcenter_of_armor() {
  cout << "the center of armor is" << centerPoint << endl;
}

void AngleSolver::showAngle() {
  cout << "_xErr is  " << _xErr << "  _yErr is  " << _yErr << endl;
}

int AngleSolver::showAlgorithm() { return angle_solver_algorithm; }

void AngleSolver::showCamTrix() {
  cout << "CAM1" << _params.CAM_MATRIX.at<double>(2, 0);
}

#endif  // DEBUG

AngleSolver::AngleFlag AngleSolver::solve() {
  if (angle_solver_algorithm == 1) {
    if (enemy_type == 1)
      solvePnP(_params.POINT_3D_OF_ARMOR_BIG, point_2d_of_armor,
               _cam_instant_matrix, _params.DISTORTION_COEFF, _rVec, _tVec,
               false, CV_ITERATIVE);
    if (enemy_type == 0)
      solvePnP(_params.POINT_3D_OF_ARMOR_SMALL, point_2d_of_armor,
               _cam_instant_matrix, _params.DISTORTION_COEFF, _rVec, _tVec,
               false, CV_ITERATIVE);
    _tVec.at<double>(1, 0) -= _params.Y_DISTANCE_BETWEEN_GUN_AND_CAM;
    _xErr =
        atan(_tVec.at<double>(0, 0) / _tVec.at<double>(2, 0)) / 2 / CV_PI * 360;
    _yErr =
        atan(_tVec.at<double>(1, 0) / _tVec.at<double>(2, 0)) / 2 / CV_PI * 360;
    _euclideanDistance = sqrt(_tVec.at<double>(0, 0) * _tVec.at<double>(0, 0) +
                              _tVec.at<double>(1, 0) * _tVec.at<double>(1, 0) +
                              _tVec.at<double>(2, 0) * _tVec.at<double>(2, 0));
    if (_euclideanDistance >= 8500) {
      return TOO_FAR;
    }
    /*
    if (_x_time_points.size < 5)
    _x_time_points.push_back(cv::Point2f(framecount,_xErr)); else {
    _x_time_points[0] = _x_time_points[1];
    _x_time_points[1] = _x_time_points[2];
    _x_time_points[2] = _x_time_points[3];
    _x_time_points[3] = _x_time_points[4];
    _x_time_points[4] = cv::Point2f(framecount, _xErr);
    }
    if (_y_time_points.size < 5)
    _y_time_points.push_back(cv::Point2f(framecount, _yErr)); else {
    _y_time_points[0] = _y_time_points[1];
    _y_time_points[1] = _y_time_points[2];
    _y_time_points[2] = _y_time_points[3];
    _y_time_points[3] = _y_time_points[4];
    _y_time_points[4] = cv::Point2f(framecount, _yErr);
    }
    */
    return ANGLES_AND_DISTANCE;
  }
  if (angle_solver_algorithm == 0) {
    double x1, x2, y1, y2, r2, k1, k2, p1, p2, y_ture;
    x1 = (centerPoint.x - _cam_instant_matrix.at<double>(0, 2)) /
         _cam_instant_matrix.at<double>(0, 0);
    y1 = (centerPoint.y - _cam_instant_matrix.at<double>(1, 2)) /
         _cam_instant_matrix.at<double>(1, 1);
    r2 = x1 * x1 + y1 * y1;
    k1 = _params.DISTORTION_COEFF.at<double>(0, 0);
    k2 = _params.DISTORTION_COEFF.at<double>(1, 0);
    p1 = _params.DISTORTION_COEFF.at<double>(2, 0);
    p2 = _params.DISTORTION_COEFF.at<double>(3, 0);
    x2 = x1 * (1 + k1 * r2 + k2 * r2 * r2) + 2 * p1 * x1 * y1 +
         p2 * (r2 + 2 * x1 * x1);
    y2 = y1 * (1 + k1 * r2 + k2 * r2 * r2) + 2 * p2 * x1 * y1 +
         p1 * (r2 + 2 * y1 * y1);
    y_ture = y2 - _params.Y_DISTANCE_BETWEEN_GUN_AND_CAM / 1000;
    _xErr = atan(x2) / 2 / CV_PI * 360;
    _yErr = atan(y_ture) / 2 / CV_PI * 360;
    if (is_shooting_rune) _yErr -= _rune_compensated_angle;
    /*
    if (_x_time_points.size < 5)
    _x_time_points.push_back(cv::Point2f(framecount, _xErr)); else {
    _x_time_points[0] = _x_time_points[1];
    _x_time_points[1] = _x_time_points[2];
    _x_time_points[2] = _x_time_points[3];
    _x_time_points[3] = _x_time_points[4];
    _x_time_points[4] = cv::Point2f(framecount, _xErr);
    }
    if (_y_time_points.size < 5)
    _y_time_points.push_back(cv::Point2f(framecount, _yErr)); else {
    _y_time_points[0] = _y_time_points[1];
    _y_time_points[1] = _y_time_points[2];
    _y_time_points[2] = _y_time_points[3];
    _y_time_points[3] = _y_time_points[4];
    _y_time_points[4] = cv::Point2f(framecount, _yErr);
    }
    */
    return ONLY_ANGLES;
  }
  if (angle_solver_algorithm == 2) {
    std::vector<Point2f> runeCenters;
    std::vector<Point3f> realCenters;
    for (size_t i = 0; i < 9; i++) {
      if (point_2d_of_rune[i].x > 0 && point_2d_of_rune[i].y > 0) {
        runeCenters.push_back(point_2d_of_rune[i]);
        realCenters.push_back(_params.POINT_3D_OF_RUNE[i]);
      }
    }

    solvePnP(realCenters, runeCenters, _cam_instant_matrix,
             _params.DISTORTION_COEFF, _rVec, _tVec, false, CV_ITERATIVE);
    _tVec.at<double>(1, 0) -= _params.Y_DISTANCE_BETWEEN_GUN_AND_CAM;
    _xErr =
        atan(_tVec.at<double>(0, 0) / _tVec.at<double>(2, 0)) / 2 / CV_PI * 360;
    _yErr =
        atan(_tVec.at<double>(1, 0) / _tVec.at<double>(2, 0)) / 2 / CV_PI * 360;
    _euclideanDistance = sqrt(_tVec.at<double>(0, 0) * _tVec.at<double>(0, 0) +
                              _tVec.at<double>(1, 0) * _tVec.at<double>(1, 0) +
                              _tVec.at<double>(2, 0) * _tVec.at<double>(2, 0));
    if (_euclideanDistance >= 8500) {
      return TOO_FAR;
    }
    /*
    if (_x_time_points.size < 5)
    _x_time_points.push_back(cv::Point2f(framecount, _xErr)); else {
    _x_time_points[0] = _x_time_points[1];
    _x_time_points[1] = _x_time_points[2];
    _x_time_points[2] = _x_time_points[3];
    _x_time_points[3] = _x_time_points[4];
    _x_time_points[4] = cv::Point2f(framecount, _xErr);
    }
    if (_y_time_points.size < 5)
    _y_time_points.push_back(cv::Point2f(framecount, _yErr)); else {
    _y_time_points[0] = _y_time_points[1];
    _y_time_points[1] = _y_time_points[2];
    _y_time_points[2] = _y_time_points[3];
    _y_time_points[3] = _y_time_points[4];
    _y_time_points[4] = cv::Point2f(framecount, _yErr);
    }
    */
    return ANGLES_AND_DISTANCE;
  }
  return ANGLE_ERROR;
}

void AngleSolver::compensateOffset() {
  /* z of the camera COMS */
  const auto offset_z = 101.0;
  const auto& d = _euclideanDistance;
  const auto theta_y = _xErr / 180 * CV_PI;
  const auto theta_p = _yErr / 180 * CV_PI;
  const auto theta_y_prime =
      atan((d * sin(theta_y)) / (d * cos(theta_y) + offset_z));
  const auto theta_p_prime =
      atan((d * sin(theta_p)) / (d * cos(theta_p) + offset_z));
  const auto d_prime =
      sqrt(pow(offset_z + d * cos(theta_y), 2) + pow(d * sin(theta_y), 2));
  _xErr = theta_y_prime / CV_PI * 180;
  _yErr = theta_p_prime / CV_PI * 180;
  _euclideanDistance = d_prime;
}

void AngleSolver::compensateGravity() {
  const auto& theta_p_prime = _yErr / 180 * CV_PI;
  const auto& d_prime = _euclideanDistance;
  const auto& v = _bullet_speed;
  const auto theta_p_prime2 =
      atan((sin(theta_p_prime) - 0.5 * 9.8 * d_prime / pow(v, 2)) /
           cos(theta_p_prime));
  _yErr = theta_p_prime2 / CV_PI * 180;
}

void AngleSolver::angleLimit() {
  if (_xErr > 60)
    _xErr = 60;
  else if (_xErr < -60)
    _xErr = -60;
  if (_yErr > 30)
    _yErr = 30;
  else if (_yErr < -20)
    _yErr = -20;
}

//��������ڲν��б궨
void AngleSolver::setResolution(const cv::Size2i& image_resolution) {
  image_size = image_resolution;
  // cout << _cam_instant_matrix.at<double>(0, 2) << "  " <<
  // _cam_instant_matrix.at<double>(1, 2) << endl;
  _cam_instant_matrix.at<double>(0, 2) =
      _params.CAM_MATRIX.at<double>(0, 2) - (640 / 2 - image_size.width / 2);
  _cam_instant_matrix.at<double>(1, 2) =
      _params.CAM_MATRIX.at<double>(1, 2) - (480 / 2 - image_size.height / 2);
  _cam_instant_matrix.at<double>(0, 0) =
      _params.CAM_MATRIX.at<double>(0, 0) / (480 / image_size.height);
  _cam_instant_matrix.at<double>(1, 1) =
      _params.CAM_MATRIX.at<double>(1, 1) / (480 / image_size.height);
  // cout << _cam_instant_matrix.at<double>(0, 2) << "  " <<
  // _cam_instant_matrix.at<double>(1, 2) << endl; cout <<
  // _params.CAM_MATRIX.at<double>(0, 2) << "  " <<
  // _params.CAM_MATRIX.at<double>(1, 2) << endl; cout << endl;
}

void AngleSolver::setUserType(int usertype) { user_type = usertype; }

void AngleSolver::setEnemyType(int enemytype) { enemy_type = enemytype; }

// const cv::Vec2f AngleSolver::getCompensateAngle()
//{
//	return cv::Vec2f(_xErr, _yErr);
//}

void AngleSolver::setBulletSpeed(int bulletSpeed) {
  _bullet_speed = bulletSpeed;
}

const cv::Vec2f AngleSolver::getAngle() { return cv::Vec2f(_xErr, _yErr); }

double AngleSolver::getDistance() { return _euclideanDistance; }

// void AngleSolver::selectAlgorithm(const int t)
//{
//	if (t == 0 || t == 1)
//		angle_solver_algorithm = t;
//
//}

void AngleSolverParam::readFile() {
  cv::FileStorage fsread(
      "C:/Users/25212/Documents/RM_BUFF/config/angle_solver_params1.xml",
      cv::FileStorage::READ);
  if (!fsread.isOpened()) {
    std::cerr << "failed to open xml" << std::endl;
    return;
  }
  fsread["Y_DISTANCE_BETWEEN_GUN_AND_CAM"] >> Y_DISTANCE_BETWEEN_GUN_AND_CAM;

#ifdef DEBUG
  std::cout << Y_DISTANCE_BETWEEN_GUN_AND_CAM << std::endl;
#endif  // DEBUG

  fsread["CAMERA_MARTRIX_0"] >> CAM_MATRIX;
  fsread["DISTORTION_COEFF_0"] >> DISTORTION_COEFF;
}

//--------д������궨���ض�����
void AngleSolverParam::writeFile() {
  double cam[3][3] = {653.6612, 0, 323.7888, 0, 655.2624, 287.1174, 0, 0, 1};

  double dis[1][5] = {0.0431, -0.9899, 6.9654, 0.0000, 0.0000};

  cv::Mat K_(3, 3, CV_64FC1, cam);
  cv::Mat D_(1, 5, CV_64FC1, dis);

  cv::FileStorage fswrite("./angle_solver_params1.xml", cv::FileStorage::WRITE);

  if (!fswrite.isOpened()) {
    std::cerr << "failed to open xml" << std::endl;
    return;
  }
  fswrite << "Y_DISTANCE_BETWEEN_GUN_AND_CAM" << Y_DISTANCE_BETWEEN_GUN_AND_CAM;

  fswrite << "CAMERA_MARTRIX_0" << K_;
  fswrite << "DISTORTION_COEFF_0" << D_;
}

#ifdef DEBUG_WRITE

int main() {
  AngleSolverParam param;
  param.writeFile();
}
#endif  // DEBUG_WRITE
