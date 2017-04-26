//
//  p3p.cpp
//  ComputerVisionProject4
//
//  Created by HJKBD on 4/10/17.
//  Copyright © 2017 HJKBD. All rights reserved.
//

#include "p3p.h"
#include "solve_polynom.h"

#include <cstring>
#include <cmath>


bool solvep3p(const cv::Mat& cameraMatrix, cv::Mat& R, cv::Mat& tvec,
                const std::vector<cv::Point3f>& _opoints, const std::vector<cv::Point2f>& _ipoints)
{
    //we use p3p, so 4 points required, 3 for find solution, the extra points used to select best solution
    assert(_opoints.size()==4 && _opoints.size()==_ipoints.size());
    
    //initialize the camera parameters;
    p3p camera = { cameraMatrix.at<double> (0, 2), cameraMatrix.at<double> (1, 2),
        cameraMatrix.at<double> (0, 0),cameraMatrix.at<double>(1, 1)};
    
    //we change the precision from float to double to have better accuracy
    //ipoints needs to be reprojected to the image plane because, we have to apply undistort function
    //in opencv, this function will change the coordinate of _ipoints to the camera coordinate, so change to image plane
    std::vector<cv::Point3d> opoints;
    std::vector<cv::Point2d> ipoints;
    for(int i = 0; i < 4; i++)
    {
        double ipointx = _ipoints[i].x*camera.fx + camera.cx;
        double ipointy = _ipoints[i].y*camera.fy + camera.cy;
        ipoints.push_back(cv::Point2d(ipointx,ipointy));
        opoints.push_back(cv::Point3d(_opoints[i]));
    }
    
    //collect all possible R and t, save it, if no solution ,return false
    std::vector<cv::Mat> Rs;
    std::vector<cv::Mat> ts;
    int n = findallsolutions(camera, Rs, ts, ipoints, opoints);
    if (n == 0)
        return false;
    
    //select best R, t pair according to the reproject error, the ideal one should has the least error
    int ns = 0;
    double min_reproj = 0;
    cv::Mat X3 = (cv::Mat_<double>(3,1) << opoints[3].x, opoints[3].y, opoints[3].z);
    for(int i = 0; i < n; i++) {
        cv::Mat X3P = Rs[i]*X3 + ts[i];
        double mu3p = camera.cx + camera.fx * X3P.at<double>(0) / X3P.at<double>(2);
        double mv3p = camera.cy + camera.fy * X3P.at<double>(1) /X3P.at<double>(2);
        cv::Point2d x3p(mu3p, mv3p);
        double reproj = cv::norm(ipoints[3] - x3p);
        if (i == 0 || min_reproj > reproj) {
            ns = i;
            min_reproj = reproj;
        }
    }
    
    //save results to R and tvec
    Rs[ns].copyTo(R);
    ts[ns].copyTo(tvec);
    
    //return true for success
    return true;
}

int findallsolutions(const p3p& camera, std::vector<cv::Mat>& Rs, std::vector<cv::Mat>& t,
          const std::vector<cv::Point2d>&ipoints, const std::vector<cv::Point3d>& opoints)
{
    std::vector<cv::Point3f>  n_ipoints;

    double cx = camera.cx, cy = camera.cy;
    double fx = camera.fx, fy = camera.fy;
    
    //shift the coordinate system to the camera center, and normalize it
    for (const auto & p : ipoints) {
        double mu =  (p.x - cx)/fx;
        double mv =  (p.y - cy)/fy;
        double norm = sqrt(mu * mu + mv * mv + 1);
        n_ipoints.emplace_back(mu/norm,mv/norm,1./norm);
    }
    
    //calculate distance
    std::vector<double> distances;
    distances.push_back(cv::norm(opoints[1]-opoints[2]));
    distances.push_back(cv::norm(opoints[0]-opoints[2]));
    distances.push_back(cv::norm(opoints[0]-opoints[1]));

    // Calculate angles
    std::vector<double> cosines;
    cosines.push_back(n_ipoints[1].dot(n_ipoints[2]));
    cosines.push_back(n_ipoints[0].dot(n_ipoints[2]));
    cosines.push_back(n_ipoints[0].dot(n_ipoints[1]));
    
    
    std::vector<std::vector<double>> lengths;
    int n = solve_for_lengths(lengths, distances, cosines);
    
    int nb_solutions = 0;
    for(int i = 0; i < n; i++) {
        //the calculated coordinate w.r.t to the camera center
        std::vector<cv::Point3d> P_Opoints;
        for (int j = 0; j < 3; j++) {
            P_Opoints.emplace_back(lengths[i][j]*n_ipoints[j]);
        }
        cv::Mat Ro, Tr;
        if(!decompose(opoints, P_Opoints, Ro, Tr))
            continue;
        Rs.push_back(Ro);
        t.push_back(Tr);

        nb_solutions++;
    }
    
    return nb_solutions;
}


int solve_for_lengths(std::vector<std::vector<double>>& lengths, const std::vector<double>& distances,
                      const std::vector<double> cosines)
{
    assert(distances.size() == 3 && distances.size() == cosines.size());
    
    double p = cosines[0] * 2;
    double q = cosines[1] * 2;
    double r = cosines[2] * 2;
    
    double inv_d22 = 1. / (distances[2] * distances[2]);
    double a = inv_d22 * (distances[0] * distances[0]);
    double b = inv_d22 * (distances[1] * distances[1]);
    
    double a2 = a * a, b2 = b * b, p2 = p * p, q2 = q * q, r2 = r * r;
    double pr = p * r, pqr = q * pr;
    
    // Check reality condition (the four points should not be coplanar)
    if (p2 + q2 + r2 - pqr - 1 == 0)
        return 0;
    
    double ab = a * b, a_2 = 2*a;
    
    double A = -2 * b + b2 + a2 + 1 + ab*(2 - r2) - a_2;
    
    // Check reality condition
    if (A == 0) return 0;
    
    double a_4 = 4*a;
    
    double B = q*(-2*(ab + a2 + 1 - b) + r2*ab + a_4) + pr*(b - b2 + ab);
    double C = q2 + b2*(r2 + p2 - 2) - b*(p2 + pqr) - ab*(r2 + pqr) + (a2 - a_2)*(2 + q2) + 2;
    double D = pr*(ab-b2+b) + q*((p2-2)*b + 2 * (ab - a2) + a_4 - 2);
    double E = 1 + 2*(b - a - ab) + b2 - b*p2 + a2;
    
    double temp = (p2*(a-1+b) + r2*(a-1-b) + pqr - a*pqr);
    double b0 = b * temp * temp;
    // Check reality condition
    if (b0 == 0)
        return 0;
    
    double real_roots[4];
    int n = solve_deg4(A, B, C, D, E,  real_roots[0], real_roots[1], real_roots[2], real_roots[3]);
    
    if (n == 0)
        return 0;
    
    int nb_solutions = 0;
    double r3 = r2*r, pr2 = p*r2, r3q = r3 * q;
    double inv_b0 = 1. / b0;
    
    // For each solution of x
    for(int i = 0; i < n; i++) {
        double x = real_roots[i];
        
        // Check reality condition
        if (x <= 0)
            continue;
        
        double x2 = x*x;
        
        double b1 =
        ((1-a-b)*x2 + (q*a-q)*x + 1 - a + b) *
        (((r3*(a2 + ab*(2 - r2) - a_2 + b2 - 2*b + 1)) * x +
          
          (r3q*(2*(b-a2) + a_4 + ab*(r2 - 2) - 2) + pr2*(1 + a2 + 2*(ab-a-b) + r2*(b - b2) + b2))) * x2 +
         
         (r3*(q2*(1-2*a+a2) + r2*(b2-ab) - a_4 + 2*(a2 - b2) + 2) + r*p2*(b2 + 2*(ab - b - a) + 1 + a2) + pr2*q*(a_4 + 2*(b - ab - a2) - 2 - r2*b)) * x +
         
         2*r3q*(a_2 - b - a2 + ab - 1) + pr2*(q2 - a_4 + 2*(a2 - b2) + r2*b + q2*(a2 - a_2) + 2) +
         p2*(p*(2*(ab - a - b) + a2 + b2 + 1) + 2*q*r*(b + a_2 - a2 - ab - 1)));
        
        // Check reality condition
        if (b1 <= 0)
            continue;
        
        double y = inv_b0 * b1;
        double v = x2 + y*y - x*y*r;
        
        if (v <= 0)
            continue;
        
        double Z = distances[2] / sqrt(v);
        double X = x * Z;
        double Y = y * Z;
        
        lengths.emplace_back(std::vector<double>{X, Y, Z});
        
        nb_solutions++;
    }
    
    return nb_solutions;
}

bool decompose(const std::vector<cv::Point3d>& WorldP, const std::vector<cv::Point3d>& CamerP,
           cv::Mat& R, cv::Mat& T)
{

    
    cv::Mat Camera_P(3,3,CV_64F);
    cv::Mat World_P(3,3,CV_64F);
    
    //make it as mat, so we can use mat mulipication
    for (int i = 0; i < 3; i++) {
        cv::Mat row1 = (cv::Mat_<double> (1,3) << CamerP[i].x, CamerP[i].y, CamerP[i].z);
        cv::Mat row2 = (cv::Mat_<double> (1,3) << WorldP[i].x, WorldP[i].y, WorldP[i].z);
        row1.copyTo(Camera_P.row(i));
        row2.copyTo(World_P.row(i));
    }

    // Centroids:
    cv::Mat C_W = (World_P.row(0) + World_P.row(1) + World_P.row(2)) / 3;
    cv::Mat C_C = (Camera_P.row(0)+ Camera_P.row(1) + Camera_P.row(2)) / 3;
    
    
    // Covariance matrix A:
    cv::Mat A = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat C = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat P = cv::Mat::zeros(3, 3, CV_64F);
    for (int i = 0; i < 3; i++ ){
        C_C.copyTo(C.row(i));
        C_W.copyTo(P.row(i));
    }
    //calculate covariance matrix A
    A = (World_P - P).t()*(Camera_P - C);

    //SVD decomposition
    cv::Mat U, D, V;
    cv::SVD::compute(A, D, U, V,cv::SVD::FULL_UV);
    R = V.t()*U.t();
    double s = cv::determinant(R);
    
    if (s < 0) {
        V.row(2) *= -1;
        R = V.t()*U.t();
    }
    
    T = -R*C_W.t() + C_C.t();
    //debug
    //std::cout << "The translation matrix is : \n" << T2 << std::endl;
    return true;
}

