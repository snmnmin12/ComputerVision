//
//  distortion.hpp
//  ComputerVisionProject4
//
//  Created by HJKBD on 4/10/17.
//  Copyright © 2017 HJKBD. All rights reserved.
//

#ifndef distortion_h
#define distortion_h
#include "header.h"
#include "lmmin.h"
#include "util.h"

using cv::Mat;
using cv::Point2d;
using cv::Point3d;
using cv::Size;
using std::vector;

Mat CalibrateDistortion (Mat);

void HLineFit (vector<Mat> &, Size, vector<Point2d>, const double *);

void VLineFit (vector<Mat> &, Size, vector<Point2d>, const double *);

void Evaluate_Distance( const double *, int, const void *, double *, int *);

void line_fit(const double *, const void *, double *);

double h_line_function (double, const double *);

double v_line_function (double, const double *);

double CalculateDistance (Mat, Mat);

double CalculateAngle (Mat, Mat);

Mat CorrectPoint (Point2d, const double *);

Mat InvCorrectPoint (Point2d, const double *);

Mat CorrectLine (Point2d, Point2d, const double *);

Mat DLT(vector<Point3d>, vector<Point2d>);

void ParBackup(const double *, double *, int);

void original_lines (vector<Mat> &, vector<Mat> &, Size, vector<Point2d> );

Mat MyUndistort(Mat, const double *);

#endif /* distortion_h */
