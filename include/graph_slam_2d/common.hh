/**
 * Global include header file 
 * author: Yuxuan Huang 
 */
#ifndef COMMON_H_
#define COMMON_H_

#include <vector>
#include <memory>
#include <iostream>
#include <string>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unordered_map>
#include <limits>
#include <termios.h>
#include <stdio.h>

//Third Party dependices 
//Eigen 
#include <Eigen/Core>
#include <Eigen/Dense>

//Sophus
#include <sophus/so2.hpp>
#include <sophus/se2.hpp>

//OpenCV, just to visualize the extracted line 
//#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>

#define PI 3.14159265
#define ERROR_MSG(msg) std::cerr<<"[ERROR] "<<msg<<std::endl
#endif