//
//  util.h
//  ComputerVisionProject4
//
//  Created by HJKBD on 4/8/17.
//  Copyright © 2017 HJKBD. All rights reserved.
//

#ifndef fundamental_h
#define fundamental_h

#include "header.h"
#include <math.h>
#include <exception>

//define a data structure to hold image and world data
struct image_world_points {
    std::vector<cv::Point2d> image;
    std::vector<cv::Point3d> world;
};


//normalized 2-D data
//@para   image_points  The image world points
//@return transformation matrix for 2d points
cv::Mat normalize(const std::vector<cv::Point2f>& image_points);


//apply normalization on 2d world points
//@para points   The image world points
//@para trans    transformation matrix
//@return normalized 2d points
std::vector<cv::Point2d>
normalize_points(const std::vector<cv::Point2f>& points, const cv::Mat& trans);

cv::Mat Vector2SkewMatrix (const double v[3]);
void ComputeCameraMatrices (const cv::Mat& F, const cv::Mat& ep , cv::Mat& P, cv::Mat& Pp);

//get the 3D world point
cv::Point3f triangulate_to_3D(const cv::Point2f& i_point1, const cv::Point2f& i_point2,
                              const cv::Mat& P1, const cv::Mat& P2);
//save the 3D world into world points
std::vector<cv::Point3f> triangulate_to_3D(const std::vector<cv::Point2f>& i1_points, const std::vector<cv::Point2f>& i2_points,
                       const cv::Mat& P1, const cv::Mat& P2);

cv::Mat refine_F(const std::vector<cv::Point2f>& i1_points, const std::vector<cv::Point2f>& i2_points,
                 const cv::Mat& P1, const cv::Mat& P2, cv::Mat& P2_r, std::vector<cv::Point3f>& XW);
void Cost_Fun( const double *par, int m_dat, const void *data, double *fvec, int *info);
//compute epipolse;
void ComputeEpipoles (const cv::Mat& F, cv::Mat& e , cv::Mat& ep );

//compute the camera matrix of a set of image/world point pairs with direct linear transfroma
//@param image_points  The image points
//@param world_points  The world points
//@return the calculated homograph matrix 3x4
cv::Mat dlt(const std::vector<cv::Point2f>& i1_points,
    const std::vector<cv::Point2f>& i2_points);
#endif /* util_h */
