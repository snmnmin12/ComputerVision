//
//  gold_standard.hpp
//  ComputerVisionProject4
//
//  Created by HJKBD on 4/9/17.
//  Copyright © 2017 HJKBD. All rights reserved.
//

#ifndef gold_standard_hpp
#define gold_standard_hpp

#include "header.h"

/// Compute the camera matrix required to match a set of world points to a set
/// of image points with a gold standard algorithm. Start from a normalized
/// direct linear transform and refine the solution by minimizing the geometric
/// error.
/// @param _image_points The image points.
/// @param _world_points The world points.
/// @return A camera matrix that transforms the first set to the second.
cv::Mat
gold_standard(const std::vector<cv::Point2d>& _image_points,
              const std::vector<cv::Point3d>& _world_points);



void cost_function(const double* p, int m_data, const void *data, double *fvec, int *info);


/// Compute the geometric error resulting from imaging a world point with an
/// inexact camera matrix.
/// @param image  The measured image point.
/// @param world  The measured world point.
/// @param cameramatrix The applied camera matrix.
/// @return The geometric error.
double
geometric_error(const cv::Point2d& image,
                const cv::Point3d& world,
                const cv::Mat cameramatrix);

#endif /* gold_standard_hpp */
