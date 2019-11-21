/*
 * @Description:一阶卡尔曼滤波算法
 * @Author: Chinwong_Leung
 * @Github: https://github.com/ZhanhongLiang
 * @Date: 2019-11-16 23:00:48
 * @LastEditors: Chinwong_Leung
 * @LastEditTime: 2019-11-18 20:27:51
 */

#ifndef FILTER_H_
#define FILTER_H_
#include "../include/header/Header.h"
namespace detect {
namespace kalmanfilter {
class KalmanFilter {
 public:
  /**
   * @brief:
   * @param {type}
   * @return:
   * @author: Chinwong_Leung
   */
  //第一个步骤设定初始值和初始化参数
  //初始化过程
  KalmanFilter(int x, int y) : KF_(4, 2) {
    measurement_ = Mat::zeros(2, 1, CV_32F);
    KF_.transitionMatrix =
        (Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
    setIdentity(KF_.measurementMatrix,
                Scalar::all(1));  //测量矩阵，就是一维中的那个观察测量矩阵
    setIdentity(KF_.processNoiseCov,
                Scalar::all(1e-10));  //外界不确定因素，系统总误差;
    setIdentity(KF_.measurementNoiseCov, Scalar::all(1e-1));  //测量噪声方差矩阵
    setIdentity(KF_.errorCovPost, Scalar::all(1));  //跟新的最小方差，噪声方差
    KF_.statePost = (Mat_<float>(4, 1) << x, y, 0, 0);  //最总状态值
  }

  /**
   * @brief:反预测卡尔曼滤波
   * @param {type}
   * @return:
   * @author: Chinwong_Leung
   */
  //预测-->更新-->预测-->更新，这个是输出点
  //这个是迭代过程
  Point2f run(float x, float y) {
    Mat prediction = KF_.predict();  //预测值
    Point2f predict_pt =
        Point2f(prediction.at<float>(0), prediction.at<float>(1));
    measurement_.at<float>(0) = x;
    measurement_.at<float>(1) = y;
    KF_.correct(measurement_);  //更新过程
    // DrawKalmanPoint(predict_pt);
    return predict_pt;
  }

  void DrawKalmanPoint(Point2f &predictPoint) {
    // circle(showImg, predictPoint, 10, Scalar(255, 155, 168), 2);
    // imshow("showImg", showImg);
    std::cout << "predict_pt_x" << predictPoint.x << std::endl;
    std::cout << "predict_pt_y" << predictPoint.y << std::endl;
  }

 public:
  Mat showImg;

 private:
  Mat measurement_;  //观察值
  cv::KalmanFilter KF_;
};

/**
 * @brief: 一阶卡尔曼滤波
 * @param {type}
 * @return:
 * @author: Chinwong_Leung
 */
class KalmanFilter1 {
  KalmanFilter1() {
    F_ = 0.01f;
    K_ = 0.01f;
    P_ = 0.01f;
    X_ = 0.01f;
    R_ = 0.01f;
  }
  void SetKalmanParam(float F, float H, float B) {
    F_ = F;
    H_ = H;
    B_ = B;
  }

  void KalmanRun(float data) {
    //时间更新，预测
    xPre = X_ + B_;
    pPre = F_ * P_ + Q_;

    //状态更新，更新
    K_ = pPre / (pPre + R_);
    P_ = (1 - K_) * pPre;
    X_ = xPre + K_ * (data - xPre);
  }

 public:
  float F_;     //状态转移矩阵
  float K_;     //卡尔曼系数
  float xPre;   //预测值
  float xPost;  //状态值
  float P_;     //噪声协方差矩阵更新值
  float pPre;   //噪声协方差矩阵
  float X_;     //估计值
  float H_;     //测量值的观察矩阵
  float B_;
  float Q_;
  float R_;
};

/**
 * @brief:二阶卡尔曼滤波
 * @param {type}
 * @return:
 * @author: Chinwong_Leung
 */
class KalmanFilter2 {
  KalmanFilter2() {}
};

}  // namespace kalmanfilter
}  // namespace detect
#endif
