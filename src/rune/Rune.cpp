/*
 * @Description: 神符实现
 * @Author: Chinwong_Leung
 * @Github: https://github.com/ZhanhongLiang
 * @Date: 2019-09-05 20:23:33
 * @LastEditors: Chinwong_Leung
 * @LastEditTime: 2019-11-17 15:58:05
 */

#define BUFF_DEBUG
#include "../../include/rune/Rune.h"

using namespace cv;
using namespace std;
using namespace detect::angle_solver;

namespace detect {

namespace armor {

Buff_Detector::Buff_Detector() {}

/**
 * @brief: 二值化图像
 * @param src_img 原图
 * @param offset 偏差
 * @param bin_img灰度图
 * @return: True
 * @author: Chinwong_Leung
 */

bool Buff_Detector::SetBinary(const Mat src_img, Mat &bin_img, int bMode) {
  Mat gray_img, gray_binary_img, temp_binary_img;

  resize(src_img, bin_img, Size(src_img.cols / 2, src_img.rows / 2), 0, 0,
         INTER_LINEAR);
  resize(src_img, gray_img, Size(src_img.cols / 2, src_img.rows / 2), 0, 0,
         INTER_LINEAR);
  resize(src_img, gray_binary_img, Size(src_img.cols / 2, src_img.rows / 2), 0,
         0, INTER_LINEAR);
  resize(src_img, temp_binary_img, Size(src_img.cols / 2, src_img.rows / 2), 0,
         0, INTER_LINEAR);

  if (bMode == BGR) {
    //二值化图片
    cvtColor(src_img, gray_img, CV_BGR2GRAY);
    threshold(gray_img, gray_binary_img, 50, 255, THRESH_BINARY);
    imshow("bin_img1", gray_binary_img);
    // cvWaitKey(20);
    //红蓝通道相减
    std::vector<Mat> splited;
    split(src_img, splited);

    if (tSelect == BLUE_CLOCK || tSelect == BLUE_ANCLOCK) {
      subtract(splited[0], splited[2], temp_binary_img);
      threshold(temp_binary_img, temp_binary_img, 65, 255, THRESH_BINARY);  //60
    } else if (tSelect == RED_CLOCK || tSelect == RED_ANCLOCK) {
      subtract(splited[2], splited[0], temp_binary_img);
      dilate(temp_binary_img,temp_binary_img,getStructuringElement(MORPH_RECT, Size(3, 3)));
      imshow("subtract_red",temp_binary_img);
      threshold(temp_binary_img, temp_binary_img, 90, 255, THRESH_BINARY);   //135
    } else {
      return false;
    }
    imshow("dilate_before",temp_binary_img);
    dilate(temp_binary_img, temp_binary_img,
           getStructuringElement(MORPH_RECT, Size(5, 5)));  //腐蚀图片
    // morphologyEx(temp_binary_img, temp_binary_img, CV_MOP_CLOSE,
    // param.element); imshow("temp_binary_img", temp_binary_img);
    // cvWaitKey(20);
    // mask操作
    bin_img = gray_binary_img & temp_binary_img;  //相与
    imshow("bin_img",bin_img);
  }
  // HSV处理
  else if (bMode == HSV) {
    //先进行二值化处理
    cvtColor(src_img, gray_img, CV_BGR2GRAY);
    threshold(gray_img, gray_binary_img, 55, 255, THRESH_BINARY);
    // imshow("bin_img2", gray_binary_img);

    //
    Mat imgHsv;
    cvtColor(gray_binary_img, imgHsv, CV_BGR2HSV);

    if (tSelect == BLUE_CLOCK || tSelect == BLUE_ANCLOCK) {
      Mat temp_img;
      inRange(imgHsv, Scalar(0, 60, 60), Scalar(255, 255, 255),
              temp_img);  // inRange二值化的方法
      inRange(imgHsv, Scalar(156, 60, 80), Scalar(255, 255, 255),
              temp_binary_img);
      temp_binary_img = temp_img | temp_binary_img;  //或操作
    } else if (tSelect == RED_CLOCK || tSelect == RED_ANCLOCK) {
      inRange(imgHsv, Scalar(35, 46, 58), Scalar(255, 255, 255),
              temp_binary_img);
    }
    dilate(temp_binary_img, temp_binary_img,
           getStructuringElement(MORPH_RECT, Size(3, 3)));  //腐蚀操作
    bin_img = gray_binary_img & temp_binary_img;            //二值化图像
  }
  //灰度处理
  else if (bMode == GRAY) {
    cvtColor(src_img, gray_img, CV_BGR2GRAY);
    threshold(gray_img, gray_binary_img, 40, 255, THRESH_BINARY);
    bin_img = gray_binary_img;
    // imshow("GRAY", gray_binary_img);
  }
  //大津算法
  else if (bMode == OTSU) {
    cvtColor(src_img, gray_img, CV_BGR2GRAY);
    threshold(gray_img, temp_binary_img, 55, 255, THRESH_OTSU);
    bin_img = temp_binary_img;
    // imshow("OTUS_BIN_IMG", bin_img);
  } else {
    return false;
  }
  return true;
}

/**
 * @brief:视频调试作用
 * @param {type}
 * @return:
 * @author: Chinwong_Leung
 */
//#ifdef BUFF_DEBUG

//  bool Buff_Detector::InitCamera()
//  {
//  //#ifdef WINDOWS_DEBUG
//      capture.open("C:\\Users\\25212\\Documents\\Buff\\Buff.mp4");
//      if(!capture.isOpened())
//      {
//          cout << "can not find the mp4!!" << endl;
//          return false;
//      }
//      return true;
//      // #else
//      //     capture_m.setVideoFormat(720, 480, 1);
//      // 	capture_m.setExposureTime(0, 64);//settings->exposure_time);
//      // 	capture_m.startStream();
//      // 	capture_m.info();
//      // 	return true;
//      // //#endif
//  }
//#endif//WINDOWS_DEBUG

/**
 * @brief:得到装甲板中心点
 * @param src_img
 * @param bMode
 * @param data
 * @param offset
 * @return: true/false
 * @author: Chinwong_Leung
 */
bool Buff_Detector::GetArmorCenter(
    const Mat src_img, const int bMode, ArmorData &data,
    Point2f offset)  //, ArmorData &data, point2f
                     // offset)//传入的data参数可以同步修改
{
  //二值化图片
  Mat bin_img = Mat::zeros(src_img.size(), CV_8UC3);
  Mat drawing_img = Mat::zeros(src_img.size(), CV_8UC3);
  // resize(src_img, bin_img, Size(src_img.cols / 2, src_img.rows / 2), 0, 0,
  // INTER_LINEAR);
  if (SetBinary(src_img, bin_img, bMode) == false) {
    return false;
  }
  dilate(bin_img, bin_img, param.element);  //腐蚀之后的图像

  // imshow("bin_img", bin_img);

  // TODO找轮廓和扇叶
  vector<vector<Point>> contours;
  vector<Vec4i> hierachy;  //装甲板的向量轮廓查找

  contours.clear();
  hierachy.clear();

  //------------------异常处理-------------------------------
  try {
    findContours(bin_img, contours, hierachy, RETR_TREE,
                 CHAIN_APPROX_NONE);  //查找二值图的轮廓
    // findContours(bin_img, contours, hierachy, CV_RETR_CCOMP,
    // CHAIN_APPROX_SIMPLE);
  } catch (const std::out_of_range &oor) {
    std::cerr << "Out of range error:" << oor.what() << std::endl;
    // std::cout << "something just wrong!!" << std::endl;
    // contours.clear();
  }
  size_t contoursSize = contours.size();
  size_t hierachySize = hierachy.size();
  // int contours_size = int(contoursSize);
  // std::cout << "size:" << contoursSize << std::endl;

  //-------------------------第一次筛选------------------
  if (contoursSize == 0) {
    std::cout << "No Contours!" << std::endl;
    return false;
  } else {
    /*int idx = 0;
    for (;idx > 0;idx = hierachy[idx][0])
    {
            drawContours(bin_img,contours, idx,
    Scalar(rand()&255,rand()&255,rand()&255), CV_FILLED, 8, hierachy);
    }
    imshow("conoturs", bin_img);*/

    //剔除噪点；
    /*for (size_t i = 0; i < contoursSize;i++)
    {
            if (contourArea(contours[i]) < param.noise_point_area)
            {

            }
    }*/

    //计算子轮廓的数目
    // size_t findCount[contoursSize];//计数数组
    // vector<int>findCount;
    // int findCount(contours_size);

    ////----------------------计算出每个轮廓子轮廓数目----------------------
    // vector<size_t> findCount;
    // findCount.clear();//清零
    ////memset(findCount, 0, sizeof(findCount));
    // for (size_t i = 0; i < contoursSize; i++)
    //{
    //	try
    //	{
    //		findCount.push_back(hierachy[i][3]);//???可以加数组try进行异常调试？？
    //	}
    //	catch (...)
    //	{
    //		std::cout << "OUT OF RANGE！！" << std::endl;
    //	}
    //	/*if (i >= contoursSize || i < 0)
    //	{
    //		std::cout << "数组越界" << std::endl;
    //		break;
    //	}*/
    //	if (hierachy[i][3] != -1)//第i个轮廓的父轮廓不是空的
    //	{
    //		if (contourArea(contours[i]) >
    // param.noise_point_area)//第i个轮廓面积小于噪点面积
    //		{
    //			findCount[hierachy[i][3]]++;
    //		}
    //	}
    //}

    //-------------------------------这个是筛选出只有一个轮廓的箭头，多个箭头---------------------------
    //筛选出只有一个子轮廓的箭头

    vector<int> conIndex;  //轮廓序号
    conIndex.clear();
    // std::cout << "contoursize:" << contoursSize << std::endl;
    if (hierachy.size()) {
      // std::cout << "hiearchy:" << hierachySize << std::endl;
      for (size_t i = 0; i < contoursSize; i++) {
        drawContours(drawing_img, contours, i, Scalar(255, 255, 255), 1, 8,
                     hierachy, 0, Point());
        // imshow("drawing_img", drawing_img);
        // std::cout << "contours[i]:" <<
        // contours[static_cast<uint>(hierachy[i][3])].size() << std::endl;
        if (/*findCount[i] == 1*/ hierachy[i][3] != -1 ||
            /*contours[static_cast<uint>(hierachy[i][3])].size() == 1 &&*/
            contourArea(contours[i]) >
                param.noise_point_area)  //就是Vec4i的hiearchy中的子轮廓数为1
        {
          RotatedRect contoursRect = minAreaRect(contours[i]);
          // RotatedRect contoursElliseRect = fitEllipse(contours[i]);

          //子轮廓为1的箭头的宽高比
          float width = MAX(contoursRect.size.width, contoursRect.size.height);
          float height = MIN(contoursRect.size.width, contoursRect.size.height);
          float wh_ratio = width / height;
          float area = contourArea(contours[i]);
          //第一种筛选出风扇中最小宽高比
          bool wh_condition = wh_ratio < param.flabellum_whrio_max &&
                              wh_ratio > param.flabellum_whrio_min;
          //第二种筛选参数为风扇的面积最小值
          bool area_condition = area > param.flabellum_area_min;
          if (wh_condition && area_condition) {
            conIndex.push_back(i);
          }
        } else if (hierachy[i][3] == -1) {
          std::cout << "No father contours" << std::endl;
        } else {
          std::cout << "No Loop!" << std::endl;
        }
      }
      if (conIndex.size() == 0) {
        std::cout << "No flag's contours" << std::endl;
        return false;
      }
      // std::cout << "Condex:" << conIndex.size() << std::endl;
      // imshow("drawing_img", drawing_img);
      waitKey(20);
    }

    //	//----------------第二次筛选，筛选出一个只有一个箭头的轮廓-----------------

    int index = NO_TARGET;
    float min_area = MAX_NUM;  //根据分数进行最后权限筛选

    //分数制度筛选装甲板，装甲板的分数=装甲板长度*10＋面积,筛选出面积最小的箭头轮廓
    for (size_t i = 0; i < conIndex.size(); i++) {
      float finalLenght = arcLength(contours[conIndex[i]], true);
      float finalArea = contourArea(contours[conIndex[i]]);
      float score = finalArea + finalLenght * 10;  //最后的权分数,得分制;

      if (score < min_area) {
        min_area = score;
        index = conIndex[i];  //轮廓的序列号
      }
      // std::cout << "finalLenght:" << finalLenght << endl;
    }
    if (index == NO_TARGET) {
      std::cout << "No armor inside the arrow!!" << std::endl;
      return false;
    }
    ////
    /////--------------第三次筛选，在最后的一个箭头轮廓中找出装甲板--------------
    bool findFlag = false;
    cv::Rect final_rect =
        boundingRect(contours[index]);  //最后的轮廓，画出最小的矩形

    // std::cout << "final_rect_tl:" << final_rect.tl() << std::endl;
    // std::cout << "final_rect_br:" << final_rect.br() << std::endl;
    // std::cout << "final_rect_area:" << final_rect.area() << std::endl;
    // std::cout << "final_rect_width:" << final_rect.width << std::endl;
    // std::cout << "final_rect_height:" << final_rect.height << std::endl;

    rectangle(drawing_img, final_rect.tl(), final_rect.br(), Scalar(0, 0, 255),
              1, 8);  //画出final_rect的轮廓
    imshow("drawing_img_rect:", drawing_img);

    Mat finalRoi =
        bin_img(final_rect);  //从二值图中获取roi区域，也就是没有击打区域
    // imshow("finalSrc", finalRoi);//debug的时候观察图像
    ////Mat rectRoi =

    ////选取面积最大，得到最后的装甲板
    RotatedRect finalArmor;
    float max_area = 0;
    vector<vector<Point>> finalContours;
    vector<Vec4i> finalHierachy;

    finalContours.clear();
    finalHierachy.clear();

    findContours(
        finalRoi, finalContours, finalHierachy, RETR_TREE, CHAIN_APPROX_NONE,
        final_rect.tl());  //选取最小矩形的左上角的点,这里不加final_rect.tl()的话就会产生不画圆的现象

    if (finalHierachy.size()) {
      for (size_t i = 0; i < finalContours.size(); i++) {
        if (finalHierachy[i][3] != -1) {
          //用宽高比进行筛选，
          RotatedRect armorRect =
              minAreaRect(finalContours[i]);  //最后轮廓的最小矩形
          float final_width = MAX(armorRect.size.height, armorRect.size.width);
          float final_height = MIN(armorRect.size.height, armorRect.size.width);
          float final_wh_ratio = final_width / final_height;

          //用面积最小进行筛选
          float final_area = contourArea(finalContours[i]);

          //
          Point2f P[4];
          armorRect.points(P);
          vector<Point> P1;
          for (int i = 0; i < 4; i++) {
            P1.push_back(P[i]);
          }
          double rev =
              matchShapes(finalContours[i], P1, CONTOURS_MATCH_I1,
                          0.0);  //返回轮廓的最大相似程度，最大值为1，最小值为0

          bool final_wh_condition = final_wh_ratio < param.armor_whrio_max &&
                                    final_wh_ratio > param.armor_whrio_min;
          bool final_area_condition = final_area > param.armor_area_min;
          bool final_rect_condition = rev < param.armor_rev_thres;  //判断

          if (final_wh_condition && final_area_condition &&
              final_rect_condition)  //利用宽高比、面积比较和矩形画框进行筛选
          {
            for (int j = 0; j < 4; j++) {
              line(finalRoi, P[j] + Point2f(offset),
                   P[(j + 1) % 4] + Point2f(offset), Scalar(0, 0, 255), 3);
            }
            if (final_area > max_area) {
              max_area = final_area;
              finalArmor = armorRect;
              findFlag = true;
            }
          }
        }
        // imshow("finalrio", finalRoi);
      }
      if (findFlag = false) {
        std::cout << "NO ARMOR FILED!!" << std::endl;
        return false;
      }
    }

    //	//-----------------------象限确定-------------------------
    data.armorCenter =
        finalArmor.center + offset;  //装甲板中心点,存储进了data中，参数共同修改
    std::cout << "finalArmorRect:" << finalArmor2Angle.center
              << std::endl;  //参数和前面有区别，offset修改了补偿
    RotatedRect finalArrow = minAreaRect(contours[index]);  //箭头位置中心点
    Point2f arrowCenter = finalArrow.center;
    std::cout << "armorCenter:" << data.armorCenter << std::endl;
    // std::cout << "angles:" << finalArmor.angle << std::endl;
    // std::cout << "arrowCenter:" << arrowCenter << std::endl;

    float min = MIN(finalArmor.size.height, finalArmor.size.width);

    if (Distance(arrowCenter, data.armorCenter) <
        min * 0.8)  //用来进行筛选用的，箭头中心点到装甲板中心点的距离
    {
      // return false;
      data.isFind = false;
    } else {
      //没有加change_angle和getDection
      float tran_angle = 0.0;
      if (finalArmor.size.width > finalArmor.size.height) {
        tran_angle = 90 - fabs(finalArmor.angle);
      } else {
        tran_angle = fabs(finalArmor.angle);
      }
      data.angle = tran_angle;
      std::cout << "angles:" << data.angle << std::endl;
      /*std::cout << "finalArmor_width:" << finalArmor.size.width << std::endl;
      std::cout << "finalArmor_height:" << finalArmor.size.height << std::endl;
      std::cout << "arrowCenter.x:" << arrowCenter.x << std::endl;
      std::cout << "arrowCenter.y:" << arrowCenter.y << std::endl;
      std::cout << "finalArmor.x:" << finalArmor.center.x << std::endl;
      std::cout << "finalArmor.y:" << finalArmor.center.y << std::endl;*/

      if (finalArmor.center.x != 0 || finalArmor.center.y != 0) {
        if (tran_angle < 20) {
          if (finalArmor.size.width < finalArmor.size.height &&
              arrowCenter.x < finalArmor.center.x) {
            data.quadrant = 1;
          } else if (finalArmor.size.width > finalArmor.size.height &&
                     arrowCenter.x > finalArmor.center.x) {
            data.quadrant = 2;
          } else if (finalArmor.size.width < finalArmor.size.height &&
                     arrowCenter.x > finalArmor.center.x) {
            data.quadrant = 3;
          } else if (finalArmor.size.width > finalArmor.size.height &&
                     arrowCenter.x < finalArmor.center.x) {
            data.quadrant = 4;
          }
        } else if (tran_angle > 70) {
          if (finalArmor.size.width < finalArmor.size.height &&
              arrowCenter.y > finalArmor.center.y) {
            data.quadrant = 1;
          } else if (finalArmor.size.width > finalArmor.size.height &&
                     arrowCenter.y > finalArmor.center.y) {
            data.quadrant = 2;
          } else if (finalArmor.size.width < finalArmor.size.height &&
                     arrowCenter.y < finalArmor.center.y) {
            data.quadrant = 3;
          } else if (finalArmor.size.width > finalArmor.size.height &&
                     arrowCenter.y < finalArmor.center.y) {
            data.quadrant = 4;
          }
        } else {
          if (finalArmor.size.width < finalArmor.size.height &&
              arrowCenter.y > finalArmor.center.y &&
              arrowCenter.x < finalArmor.center.x) {
            data.quadrant = 1;
          } else if (finalArmor.size.width > finalArmor.size.height &&
                     arrowCenter.y > finalArmor.center.y &&
                     arrowCenter.x > finalArmor.center.x) {
            data.quadrant = 2;
          } else if (finalArmor.size.width < finalArmor.size.height &&
                     arrowCenter.y < finalArmor.center.y &&
                     arrowCenter.x > finalArmor.center.x) {
            data.quadrant = 3;
          } else if (finalArmor.size.width > finalArmor.size.height &&
                     arrowCenter.y < finalArmor.center.y &&
                     arrowCenter.x < finalArmor.center.x) {
            data.quadrant = 4;
          }
        }
      }
      std::cout << "data_quardrant:" << data.quadrant << std::endl;

      //------------转换为坐标系的角度-----------
      float tran2QuadrantAngle = 0.0;
      Change2Angle(data.quadrant, tran_angle, tran2QuadrantAngle);
      std::cout << "tran_angle:" << tran2QuadrantAngle << std::endl;
      //象限确定之后
      if (data.quadrant == 1) {
        data.RadiusCenter.x =
            data.armorCenter.x - param.radius * cos(data.angle * CV_PI / 180);
        data.RadiusCenter.y =
            data.armorCenter.y + param.radius * sin(data.angle * CV_PI / 180);
      } else if (data.quadrant == 2) {
        data.RadiusCenter.x =
            data.armorCenter.x + param.radius * cos(data.angle * CV_PI / 180);
        data.RadiusCenter.y =
            data.armorCenter.y + param.radius * sin(data.angle * CV_PI / 180);
      } else if (data.quadrant == 3) {
        data.RadiusCenter.x =
            data.armorCenter.x + param.radius * cos(data.angle * CV_PI / 180);
        data.RadiusCenter.y =
            data.armorCenter.y - param.radius * sin(data.angle * CV_PI / 180);
      } else if (data.quadrant == 4) {
        data.RadiusCenter.x =
            data.armorCenter.x - param.radius * cos(data.angle * CV_PI / 180);
        data.RadiusCenter.y =
            data.armorCenter.y - param.radius * sin(data.angle * CV_PI / 180);
      }
      //这里有异常，因为当在第四象限的时候会出现，RadiusCenter.x = 0
      /*else if(data.quadrant == 0)
      {
              data.RadiusCenter.x = data.armorCenter.x -
      param.radius*cos(data.angle * CV_PI / 180); data.RadiusCenter.y =
      data.armorCenter.y - param.radius*sin(data.angle * CV_PI / 180);
      }*/
      data.isFind = true;
    }
    //画出圆点和装甲板中心点，箭头中心点
    circle(debug_src_img, data.armorCenter, 5, Scalar(255, 255, 255), 2);
    circle(debug_src_img, data.RadiusCenter, 5, Scalar(255, 255, 0),
           2);  //这个东西解算有问题
    // circle(debug_src_img, arrowCenter, 5, Scalar(255, 0, 0), 2);
    circle(debug_src_img, data.RadiusCenter,
           Distance(data.armorCenter, data.RadiusCenter), Scalar(255, 155, 155),
           2);
    // std::cout << "Distance_RadiusCenter:" << Distance(data.armorCenter,
    // data.RadiusCenter) << std::endl;
    imshow("center", debug_src_img);

    std::cout << "data.RaiusCenter:" << data.RadiusCenter << endl;
  }
  return true;
}

/**
 * @brief:
 * @param src_img
 * @param bMode
 * @param data
 * @param offset
 * @return: true/false
 * @author: Chinwong_Leung
 */
void Buff_Detector::GetArmorRect(const RotatedRect &rect) {
  finalArmor2Angle = rect;
}

/**
 * @brief: 检测函数
 * @param pt为装甲板中心点
 * @param int &status状态
 * @return:
 * @author: Chinwong_Leung
 */
void Buff_Detector::Detect(const Mat frame, int Mode, Point2f &pt,
                           int &status)  //,Point2f &pt,int status)
{
  Point2f offset = Point2f(0, 0);
  Mat src = frame;
  SetImage(frame, src, offset);  // roi操作??
  // imshow("roi_process", src);

  if (sParam.debug) {
    debug_src_img = frame.clone();
    imshow("debug_src_img", debug_src_img);
  }
  cvWaitKey(20);
  tSelect = Mode;  //模式参数选择,提供接口
  if (tSelect == 1 || tSelect == 2 || tSelect == 3 || tSelect == 4)  //大符
  {
    ArmorData armordata;
    if (GetArmorCenter(src, param.bMode, armordata, offset) == false) {
      pt = Point2f(0, 0);
    } else {
      Point2f preCenter;
      if (Predict(armordata, preCenter, param.pMode) == false) {
        pt = Point2f(0, 0);
      } else {
        pt = preCenter;
      }
    }
    lastData = armordata;  //上一次的数据
    GetFrameClock();  //这个是判断神符顺逆时针、これは判断力です！！
    // std::cout << "preCenter:" << pt << std::endl;
    circle(debug_src_img, pt, 10, Scalar(255, 255, 0), 1, 8);
  } else if (tSelect == 5 || tSelect == 6) {
    ArmorData armordata;
    if (GetArmorCenter(src, param.bMode, armordata, offset)) {
    }
  } else {
    return;
  }
  // Mat src = src_img;
}

/**
 * @brief: 距离函数
 * @param Point2f x Point2f y
 * @return: value
 * @author: Chinwong_Leung
 */
float Buff_Detector::Distance(const Point2f pt1, const Point2f pt2) {
  return sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) +
              (pt1.y - pt2.y) * (pt1.y - pt2.y));
}

/**
 * @brief: 圆周拟合函数
 * @param vector<Point2f> &pt
 * @param Point2f R_center
 * @return: true/false
 * @author: Chinwong_Leung
 */
bool Buff_Detector::CircleFit(const vector<Point2f> &pt, Point2f &R_center) {
  float center_x;
  float center_y;
  float radius;

  if (pt.size() < 3) {
    return false;
  }

  double sum_x = 0.0f, sum_y = 0.0f;
  double sum_x2 = 0.0f, sum_y2 = 0.0f;
  double sum_x3 = 0.0f, sum_y3 = 0.0f;
  double sum_xy = 0.0f, sum_x1y2 = 0.0f, sum_x2y1 = 0.0f;

  int N = pt.size();
  for (int i = 0; i < N; i++) {
    double x = pt[i].x;
    double y = pt[i].y;
    double x2 = x * x;
    double y2 = y * y;
    sum_x += x;
    sum_y += y;
    sum_x2 += x2;
    sum_y2 += y2;
    sum_x3 += x2 * x;
    sum_y3 += y2 * y;
    sum_xy += x * y;
    sum_x1y2 += x * y2;
    sum_x2y1 += x2 * y;
  }

  double C, D, E, G, H;
  double a, b, c;

  C = N * sum_x2 - sum_x * sum_x;
  D = N * sum_xy - sum_x * sum_y;
  E = N * sum_x3 + N * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
  G = N * sum_y2 - sum_y * sum_y;
  H = N * sum_x2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;
  a = (H * D - E * G) / (C * G - D * D);
  b = (H * C - E * D) / (D * D - G * C);
  c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N;

  center_x = a / (-2);
  center_y = b / (-2);
  radius = sqrt(a * a + b * b - 4 * c) / 2;
  R_center = Point2f(center_x, center_y);
  return true;
}

/**
 * @brief: 预测函数
 * @param vector<Point2f> &pt
 * @param Point2f R_center
 * @return: true/false
 * @author: Chinwong_Leung
 */

bool Buff_Detector::Predict(const ArmorData data, Point2f &preCenter,
                            int pMode) {
  int countNum = 200;
  vector<Point2f> armorPrePoints;
  if (pMode == FIT_CIRCLE) {
    static int count = 0;
    if (count < countNum) {
      armorPrePoints.push_back(data.armorCenter);
      count++;
      return false;
    } else if (count == countNum) {
      Point2f center;
      CircleFit(armorPrePoints, center);  // center为圆点；
      // std::cout << "center:" << center << std::endl;
      // circle(debug_src_img, center, 5, Scalar(0, 0, 255), 1, 8);
      // imshow("Fit Circle", debug_src_img);
      float preAngle;
      //----------逆时针旋转--------
      if (tSelect == BLUE_ANCLOCK || tSelect == RED_ANCLOCK) {
        preAngle = param.preAngle;
      }
      //----------顺时针旋转---------
      else {
        preAngle = -param.preAngle;
      }
      double x = data.armorCenter.x - center.x;
      double y = data.armorCenter.y - center.y;
      // preCenter的公式问题，运行点不对
      preCenter.x = x * cos(preAngle) + y * sin(preAngle) + center.x;  //????
      preCenter.y = x * sin(preAngle) + y * cos(preAngle) + center.y;  //????

      circle(debug_src_img, preCenter, 5, Scalar(0, 0, 255), 1, 8);
      circle(debug_src_img, center, 5, Scalar(0, 255, 255), 1, 8);
      imshow("FIT_CIRCLE:", debug_src_img);
      std::cout << "preCenter_x:" << preCenter.x << std::endl;
      std::cout << "preCenter_y:" << preCenter.y << std::endl;
      std::cout << "center_x:" << center.x << std::endl;
      std::cout << "center_y:" << center.y << std::endl;
    }
  }
  //???这个还没有完全弄懂
  else if (pMode == TANGENT) {
    float preAngle = param.preAngle;
    // dis
    float dis = param.radius * tan(preAngle);  // dis 为118.41常数
    std::cout << "dis:" << dis << std::endl;
    float dis_x = dis * sin(data.angle * CV_PI / 180);
    float dis_y = dis * cos(data.angle * CV_PI / 180);

    std::cout << "dis_x:" << dis_x << std::endl;
    std::cout << "dis_y:" << dis_y << std::endl;

    Point2f tangent;

    if (tSelect == tSelect == BLUE_ANCLOCK || tSelect == RED_ANCLOCK) {
      if (data.quadrant == 1) {
        tangent.x = data.armorCenter.x - dis_x;
        tangent.y = data.armorCenter.y - dis_y;
      } else if (data.quadrant == 2) {
        tangent.x = data.armorCenter.x - dis_x;
        tangent.y = data.armorCenter.y + dis_y;
      } else if (data.quadrant == 3) {
        tangent.x = data.armorCenter.x + dis_x;
        tangent.y = data.armorCenter.y + dis_y;
      } else if (data.quadrant == 4) {
        tangent.x = data.armorCenter.x + dis_x;
        tangent.y = data.armorCenter.y - dis_y;
      } else {
        return false;
      }
      double x = tangent.x - data.armorCenter.x;
      double y = tangent.y - data.armorCenter.y;

      preCenter.x =
          x * cos(preAngle / 2) + y * sin(preAngle / 2) + data.armorCenter.x;
      preCenter.y =
          -x * sin(preAngle / 2) + y * cos(preAngle / 2) + data.armorCenter.y;
    } else {
      if (data.quadrant == 1) {
        tangent.x = data.armorCenter.x + dis_x;
        tangent.y = data.armorCenter.y + dis_y;
      } else if (data.quadrant == 2) {
        tangent.x = data.armorCenter.x + dis_x;
        tangent.y = data.armorCenter.y - dis_y;
      } else if (data.quadrant == 3) {
        tangent.x = data.armorCenter.x - dis_x;
        tangent.y = data.armorCenter.y - dis_y;
      } else if (data.quadrant == 4) {
        tangent.x = data.armorCenter.x - dis_x;
        tangent.y = data.armorCenter.y + dis_y;
      } else {
        return false;
      }
      // 绕装甲板旋转
      double x = tangent.x - data.armorCenter.x;
      double y = tangent.y - data.armorCenter.y;
      preCenter.x = x * cos(-preAngle / 2) + y * sin(-preAngle / 2) +
                    data.armorCenter.x;  //？？？
      preCenter.y = -x * sin(-preAngle / 2) + y * cos(-preAngle / 2) +
                    data.armorCenter.y;  //？？？？？
    }
    circle(debug_src_img, preCenter, 5, Scalar(0, 0, 255), 1, 8);
    std::cout << "preCenter" << preCenter << std::endl;
    imshow("TAGENT:", debug_src_img);
  }
  return true;
}

/**
 * @brief: roi范围的截图
 * @param const Rect rect
 * @param const Size size
 * @return: true/false
 * @author: Chinwong_Leung
 */

bool Buff_Detector::MakeRectSafe(const Rect rect, const Size size) {
  if (rect.x < 0) {
    return false;
  } else if (rect.x + rect.width > size.width) {
    return false;
  } else if (rect.y < 0) {
    return false;
  } else if (rect.y + rect.height > size.height) {
    return false;
  } else if (rect.width <= 0 || rect.height <= 0) {
    return false;
  }
  return true;
}

/**
 * @brief: 设置ROI
 * @param const src
 * @param const Size size
 * @return: true/false
 * @author: Chinwong_Leung
 */
bool Buff_Detector::SetImage(const Mat src, Mat &dect_src, Point2f &offset) {
  if (lastData.isFind == false) {
    dect_src = src;
  } else {
    float scale = 1.5;  //比例
    float lu_x = lastData.RadiusCenter.x - param.radius * scale;
    float lu_y = lastData.RadiusCenter.y - param.radius * scale;
    Rect2f rect(lu_x, lu_y, param.radius * 2 * scale, param.radius * 2 * scale);
    if (MakeRectSafe(rect, src.size()) == false) {
      dect_src = src;
      offset = Point2f(0, 0);
    } else {
      dect_src = src(rect);
      offset = rect.tl();
    }
  }
  return true;
}

/**
 * @brief: 将角度转换为坐标系的角度，0-360度
 * @param const int quadrant
 * @param const float angle
 * @param tran_angle
 * @return: true/false
 * @author: Chinwong_Leung
 */
bool Buff_Detector::Change2Angle(const int quadrant, const float angle,
                                 float &tran_angle) {
  if (quadrant == 1) {
    tran_angle = angle;
  } else if (quadrant == 2) {
    tran_angle = 90 + (90 - angle);
  } else if (quadrant == 3) {
    tran_angle = 180 + angle;
  } else if (quadrant == 4) {
    tran_angle = 270 + (90 - angle);
  } else if (quadrant == 0) {
    return false;
  }
  return true;
}

/**
 * @brief: 判断顺逆时针
 * @param const int quadrant
 * @param const float angle
 * @param tran_angle
 * @return: true/false
 * @author: Chinwong_Leung
 */
bool Buff_Detector::GetFrameClock() {
  static int times = 0;
  static vector<ArmorData> data;
  if (times < FRAME_NUM) {
    times++;
    data.push_back(lastData);
    return false;
  } else {
    // 记录角度和象限
    float angles[FRAME_NUM];
    memset(angles, 0, sizeof(angles));
    for (int i = 0; i < FRAME_NUM; ++i) {
      Change2Angle(data[i].quadrant, data[i].angle, angles[i]);
    }

    int positive = 0;
    int negetive = 0;
    //这里用了排序的思想，就是两两比较
    for (int j = 1; j < 3; ++j) {
      for (int i = 0; i < FRAME_NUM - j; ++i) {
        if ((angles[i] - angles[i + j]) > 0 ||
            (angles[i] - angles[i + j]) < -300) {
          positive++;
        } else if ((angles[i] - angles[i + j]) < 0 ||
                   (angles[i] - angles[i + j]) > 300) {
          negetive++;
        }
      }
    }
    if (positive > negetive) {
      dirFlag = true;
      cout << "CLOCK:" << positive << endl;
    } else if (positive < negetive) {
      dirFlag = false;
      cout << "ANTICLOCK:" << negetive << endl;
    }
    times = 0;
    data.clear();
    return true;
  }
  return true;
}

/**
 * @brief: 判断是否切换
 * @param const int quadrant
 * @param const float angle
 * @param tran_angle
 * @return: true/false
 * @author: Chinwong_Leung
 */

void Buff_Detector::IsCut(const ArmorData new_data, int &status) {
  if (new_data.isFind == true && lastData.isFind == true) {
    float new_tran_angle = 0.0;
    //新角度
    Change2Angle(new_data.quadrant, new_data.angle, new_tran_angle);

    //旧角度
    float last_tran_angle = 0.0;
    Change2Angle(lastData.quadrant, lastData.angle, last_tran_angle);

    if (new_data.quadrant == 4 && lastData.quadrant == 1) {
      last_tran_angle += 360;
    } else if (new_data.quadrant == 1 && lastData.quadrant == 4) {
      new_tran_angle += 360;
    }

    float dis = fabs(new_tran_angle - last_tran_angle);
    if (dis < 40) {
      status = 1;
    } else {
      status = 2;
      if (frame_cnt < param.cutLimitedTime) {
        status = 1;
      } else {
        frame_cnt = 0;
      }
    }
  }
}

void Buff_Detector::clear() {
  mode = 0;
  frame_cnt = 0;
  memset(&lastData, 0, sizeof(ArmorData));
  memset(&lostData, 0, sizeof(ArmorData));
}

}  // namespace armor
}  // namespace detect
