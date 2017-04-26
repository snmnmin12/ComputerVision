//
//  util.cpp
//  ComputerVisionProject4
//
//  Created by HJKBD on 4/8/17.
//  Copyright © 2017 HJKBD. All rights reserved.
//

#include "util.h"


cv::Mat normalize(const std::vector<cv::Point2d>& points) {
    
    // Find the min, max, and average in each dimension.
    double  x_sum = 0;
    double  y_sum = 0;
    
    for(const auto& p : points) {
        
        x_sum += p.x;
        y_sum += p.y;
    }
    
    
    // Compute x, y translations based on the centroid.
    const double x = x_sum / points.size();
    const double y = y_sum / points.size();
    
    // Compute scale based on the size of the bounding box.
    double sumsquare = 0.0;
    for (const auto& p : points) {
        sumsquare += sqrt(pow(p.x-x,2.0)+pow(p.y-y,2.0));
    }
    double s = sqrt(2)*points.size()/sumsquare;
    
    // Compute the normalizing transform.
    cv::Mat transform = (cv::Mat_<double>(3, 3) << s, 0, -x * s,
                         0, s, -y * s,
                         0, 0, 1);
    
    // Debug.
    std::cout << "\nNormalizing transform (2d):\n" << transform << std::endl;
    
    return transform;

}

cv::Mat normalize(const std::vector<cv::Point3d>& points) {
    
    // Find the min, max, and average in each dimension.
    double x_sum = 0;
    double y_sum = 0;
    double z_sum = 0;
    
    for(const auto& p : points) {
        x_sum += p.x;
        y_sum += p.y;
        z_sum += p.z;
    }
    
    
    // Compute x, y translations based on the centroid.
    const double x = x_sum / points.size();
    const double y = y_sum / points.size();
    const double z = z_sum / points.size();
    
    // Compute scale based on the size of the bounding box.
    double sumsquare = 0.0;
    for (const auto& p : points) {
        sumsquare += sqrt(pow(p.x-x,2.0)+pow(p.y-y,2.0) + pow(p.z-z,2.0));
    }
    
    // Compute scale based on the size of the bounding box.
    const double s = std::sqrt(3.)*points.size()/sumsquare;
    
    // Compute the normalizing transform.
    cv::Mat transform = (cv::Mat_<double>(4, 4) << s, 0, 0, -x * s,
                         0, s, 0, -y * s,
                         0, 0, s, -z * s,
                         0, 0, 0, 1);
    // Debug.
    std::cout << "\nNormalizing transform (3d):\n" << transform << std::endl;
    
    return transform;
}



std::vector<cv::Point2d>
normalize_points(const std::vector<cv::Point2d>& points, const cv::Mat& trans)
{
    std::vector<cv::Point2d> output;
    output.reserve(points.size());
    
    for(const auto&p2d: points) {
        // Make a homogeneous representation of point i.
        cv::Mat_<double> p = (cv::Mat_<double>(3,1) << p2d.x, p2d.y, 1);
        
        // Transform p by _trans.
        p = trans * p;
        
        // Dehomogenize and store the result in output.
        output.emplace_back(p(0,0) / p(0,2), p(0,1) / p(0,2));
    }
    
    // Debug.
    std::cout << "\nNormalized points:";
    for(const auto& point : output)
        std::cout << "\n" << point;
    std::cout << std::endl;
    
    return output;
}

std::vector<cv::Point3d>
normalize_points(const std::vector<cv::Point3d>& points, const cv::Mat& trans)
{
    std::vector<cv::Point3d> output;
    output.reserve(points.size());
    
    for(const auto& p3d: points) {
        // Make a homogeneous representation of point i.
        cv::Mat_<double> p = (cv::Mat_<double>(4,1) << p3d.x, p3d.y, p3d.z, 1);
        
        // Transform p by _trans.
        p = trans * p;
        // Dehomogenize and store the result in output.
        output.emplace_back(p(0,0) / p(0,3), p(0,1) / p(0,3), p(0,2) / p(0,3));
    }
    
    // Debug.
    std::cout << "\nNormalized points:";
    for(const auto& point : output)
        std::cout << "\n" << point;
    std::cout << std::endl;
    
    return output;
}




cv::Mat dlt(const std::vector<cv::Point2d>& image_points,
            const std::vector<cv::Point3d>& world_points)
{
    if (image_points.size() != world_points.size())
        throw std::runtime_error("The image point size and wold point size doesn't match! but image point size: " + std::to_string(image_points.size()) + "world point size: " + std::to_string(world_points.size()));
    cv::Mat_<double> matrix(int(image_points.size()) * 2, 12);
    for (int i = 0; i < image_points.size(); i++) {
        const auto& imap = image_points[i];
        const auto& worp = world_points[i];
        cv::Mat_<double> even = (cv::Mat_<double>(1,12) <<
        worp.x, worp.y, worp.z, 1, 0, 0, 0, 0, -worp.x * imap.x,-worp.y * imap.x,-worp.z * imap.x,-imap.x);
        
        even.copyTo(matrix.row(i * 2));
        
        cv::Mat_<double> odd  = (cv::Mat_<double>(1, 12) <<
            0, 0, 0, 0, worp.x, worp.y, worp.z, 1, -worp.x * imap.y,
            -worp.y * imap.y, -worp.z * imap.y,-imap.y);
         odd.copyTo(matrix.row(i * 2 + 1));
    }
    
    //solve the dlt equation
    cv::Mat_<double> U,Sig, Vt;
    cv::SVD::compute(matrix, U, Sig, Vt, cv::SVD::FULL_UV);
    
    // The last row of Vt holds the parameters for our camera matrix. Reshape it
    // to a 3x4 matrix with the same number of channels.
    cv::Mat_<double> P = Vt.row(11);
    P = P.reshape(0, 3);
    // Debug.
    std::cout << "\nComputed camera matrix:\n" << P << std::endl;
    return P;
    
}
