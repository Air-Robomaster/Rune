/*
 * @Description:
 * @Author: Chinwong_Leung
 * @Github: https://github.com/ZhanhongLiang
 * @Date: 2019-10-23 22:32:37
 * @LastEditors: Chinwong_Leung
 * @LastEditTime: 2019-11-13 09:45:40
 */
#pragma once
#include <iostream>
#include "../header/Header.h"
#include "./AngleSolver.hpp"
#include "./Rune.h"

#define EXPOSURE_RUNE 2000
#define RUNE_WIDTH 888
#define RUNE_HEIGHT 1000

#define BUFFER_SIZE 1

class ImageConsProd {
 public:
  ImageConsProd(Settings* _settings, int fd_car);
  ~ImageConsProd();

  // ImageData data[BUFFER_SIZE];
  void ImageProducer();
  void ImageConsumer();
  Settings* settings;
  int fd2car;
  void test();
};
