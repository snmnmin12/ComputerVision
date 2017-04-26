//
//  util.h
//  ComputerVisionProject4
//
//  Created by HJKBD on 4/8/17.
//  Copyright © 2017 HJKBD. All rights reserved.
//

#ifndef util_h
#define util_h

#include "header.h"
#include <math.h>
#include <exception>

//define a data structure to hold image and world data
struct image_world_points {
    std::vector<cv::Point2d> image;
    std::vector<cv::Point3d> world;
};

typedef struct {
    int n_inliers;
    std::vector<cv::Point2d> src_inliers;
    std::vector<cv::Point2d> dst_inliers;
} inlier_struct;

typedef struct {
    std::vector<cv::Point2d> corners;
    cv::Size patternSize;
} pattern_struct;

typedef struct {
    std::vector<cv::Point2d> points;
    int cursor;
} line_struct;

//normalized 2-D data
//@para   image_points  The image world points
//@return transformation matrix for 2d points
cv::Mat normalize(const std::vector<cv::Point2d>& image_points);

//normalized 3-D data
//@para   world_points  The world points
//@return transformation matrix for 3d points
cv::Mat normalize(const std::vector<cv::Point3d>& world_points);


//apply normalization on 2d world points
//@para points   The image world points
//@para trans    transformation matrix
//@return normalized 2d points
std::vector<cv::Point2d>
normalize_points(const std::vector<cv::Point2d>& points, const cv::Mat& trans);

//apply normalization on 3d world points
//@para points   The world points
//@para trans    transformation matrix
//@return normalized 3d points
std::vector<cv::Point3d>
normalize_points(const std::vector<cv::Point3d>& points, const cv::Mat& trans);

//compute the camera matrix of a set of image/world point pairs with direct linear transfroma
//@param image_points  The image points
//@param world_points  The world points
//@return the calculated homograph matrix 3x4
cv::Mat dlt(const std::vector<cv::Point2d>& image_points,
    const std::vector<cv::Point3d>& world_points);
#endif /* util_h */
