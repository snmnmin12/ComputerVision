//
//  calibration.hpp
//  ComputerVisionProject4
//
//  Created by HJKBD on 4/9/17.
//  Copyright © 2017 HJKBD. All rights reserved.
//

#ifndef calibration_hpp
#define calibration_hpp

#include "header.h"

//camera calibration with my method
//@para image The input image
//@return the camera matrix P
cv::Mat CameraCalibration(cv::Mat image);

//camera calibration with OpenCV method
//@para image The input image
//@return the camera matrix P
cv::Mat CalibrateOpenCV (cv::Mat img);

#endif /* calibration_hpp */
