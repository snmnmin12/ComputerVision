//
//  p3p.hpp
//  ComputerVisionProject4
//
//  Created by HJKBD on 4/10/17.
//  Copyright © 2017 HJKBD. All rights reserved.
//

#ifndef p3p_hpp
#define p3p_hpp

#include "global.h"

//struct to store the camera internal paramters
struct p3p {
    double cx, cy,fx, fy;
};

//find the extrinsc parameters of camera with p3p, the inputs is 4 points in image and 4 points in the world
//camera intrinsci parameter
//@para camera The camera intrinsic parameter
//@para opoints The four points with repsect to the world coordinate system
//@para ipoints The four points on image plane
//@para R  The rotation matrix to be returned
//@para T  The translation matrix to be returned
bool solvep3p(const cv::Mat& camera, cv::Mat& R, cv::Mat& tvec, const std::vector<cv::Point3f>& opoints,
           const std::vector<cv::Point2f>& ipoints);

//for the given object coordinate in the world coordinate system and camera center coordinate system
//find all possible solutions of R and T, in total 4
//@para WorldP The given world coordinate
//@para CameraP The calculated camera coordinate from function findallsolutions
//@para Rs  All possible rotation matrix to be returned, maximum 4
//@para Ts  All possible translation matrix to be returned, maximum 4
int findallsolutions(const p3p& camera, std::vector<cv::Mat>& Rs, std::vector<cv::Mat>& t,
          const std::vector<cv::Point2d>&ipoints, const std::vector<cv::Point3d>& opoints);

/// Given 3D distances between three points and cosines of 3 angles at the apex, calculates
/// the lentghs of the line segments connecting projection center (P) and the three 3D points (A, B, C).
/// Returned distances are for |PA|, |PB|, |PC| respectively.
/// Only the solution to the main branch.
/// Reference : X.S. Gao, X.-R. Hou, J. Tang, H.-F. Chang; "Complete Solution Classification for the Perspective-Three-Point Problem"
/// IEEE Trans. on PAMI, vol. 25, No. 8, August 2003
/// \param lengths Lengths of line segments up to four solutions.
/// \param distances Distance between 3D points in pairs |BC|, |AC|, |AB|.
/// \param cosines Cosine of the angles /_BPC, /_APC, /_APB.
/// \returns Number of solutions.
/// WARNING: NOT ALL THE DEGENERATE CASES ARE IMPLEMENTED
int solve_for_lengths(std::vector<std::vector<double>>& lengths, const std::vector<double>& distances,
                      const std::vector<double> cosines);

//for the given object coordinate in the world coordinate system and camera center coordinate system
//find the rotation and translation relations to satisfy CameraPoint = R*WorldPoint + T
//@para WorldP The given world coordinate
//@para CameraP The calculated camera coordinate from function findallsolutions
//@para R  The rotation matrix to be returned
//@para T  The translation matrix to be returned
bool decompose(const std::vector<cv::Point3d>& WorldP,
           const std::vector<cv::Point3d>& CameraP,cv::Mat& R, cv::Mat& T);

#endif /* p3p_hpp */
