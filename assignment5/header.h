//
//  header.h
//  ComputerVisionProject4
//
//  Created by HJKBD on 4/8/17.
//  Copyright © 2017 HJKBD. All rights reserved.
//

#ifndef header_h
#define header_h

#include <stdio.h>
#include <dirent.h>
#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
struct Point_Four {
    std::vector<cv::Point2f> src;
    std::vector<cv::Point2f> dst;
};
#endif /* header_h */
