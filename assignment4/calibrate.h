//
//  calibrate.hpp
//  ComputerVisionProject4
//
//  Created by HJKBD on 4/10/17.
//  Copyright © 2017 HJKBD. All rights reserved.
//

#ifndef calibrate_hpp
#define calibrate_hpp

#include "global.h"

//camera calibration with OpenCV method
//@para image The input image
//@para CameraMatrix passed as reference, so camera internal paramters will be saved here
//@para distcoff passed as reference, so distortion coefficient will be saved here
//@return the undistorted image

bool CalibrateOpenCV (cv::Mat& img, cv::Mat& CameraMatrix, cv::Mat& distcoff);
bool CalibrateOpenCV2 (cv::Mat& img, cv::Mat& CameraMatrix, cv::Mat& distcoff);
//check if it is rotation matrix valid
bool isRotationMatrix(cv::Mat &R);

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R);
#endif /* calibrate_hpp */
