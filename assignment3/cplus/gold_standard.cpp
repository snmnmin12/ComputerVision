//
//  gold_standard.cpp
//  ComputerVisionProject4
//
//  Created by HJKBD on 4/9/17.
//  Copyright © 2017 HJKBD. All rights reserved.
//

#include "gold_standard.h"
#include "util.h"
#include "lmmin.h"

void cost();
cv::Mat
gold_standard(const std::vector<cv::Point2d>& image_points,
              const std::vector<cv::Point3d>& world_points) {
    cv:: Mat mat;
    // Begin with normalized dlt.
    
    // Compute the normalizing transform for each image.
    const cv::Mat image_transform = normalize(image_points);
    const cv::Mat world_transform = normalize(world_points);
    
    // Normalize each set of points.
    const auto src_points = normalize_points(image_points,
                                                      image_transform);
    const auto des_points = normalize_points(world_points,
                                                      world_transform);
    
    
    // Compute the DLT using the normalized points.
    cv::Mat_<double> camera_matrix = dlt(src_points, des_points);
    
    int n_par = camera_matrix.cols* camera_matrix.rows;
    double par[n_par];
    for (int i = 0; i < n_par; i++)
        par[i] = camera_matrix[i/4][i%4];
    

    
    // Create the cost function.
    auto cost = [&src_points, &des_points](const double* p) {
        cv::Mat cameramatrix = cv::Mat::zeros(3, 4, CV_64F);
        for (int i = 0; i < 12; i++)
            cameramatrix.at<double>(i/4,i%4) = p[i];
        double cost = 0.;
        for(size_t i = 0; i < src_points.size(); ++i)
            cost += geometric_error(src_points[i], des_points[i], cameramatrix);
        return cost;
    };
    
    //initial error is
    double initial_error = cost(par);
    
    int m_data =5*int(src_points.size());
    
    image_world_points data = {src_points, des_points};
    lm_control_struct control = lm_control_double;
    lm_status_struct status;
    lmmin(n_par, par, m_data, &data, cost_function, &control, &status);
    
    
    //initial error is
    double final_error = cost(par);
    // After the iteration, par has the approached P values
    cv::Mat P = cv::Mat::zeros(3,4,CV_64F);
    for (int i=0; i<3; i++) {
        for (int j=0; j<4; j++) {
            P.at<double>(i,j)=par[i*4+j];
        }
    }
    
    std::cout << "\nMinimized camera_matrix:\n"
    << P << "\n"
    << "\nInitial geometric error: " << initial_error
    << "\nFinal geometric error: " << final_error
    << "\nImprovement: " << initial_error - final_error
    << " (" << std::setprecision(3)
    << 100 * (initial_error - final_error) / initial_error << "%)"
    << "\n" << std::endl;
    
    // Denormalize P to P_de
    cv::Mat P_de = image_transform.inv()*P*world_transform;
    
    // Normalized Matrix H
    std::cout << "\nFinal_camera_matrix:\n"
    << P_de << "\n";
    
    cv::Mat MycameraMatrix(3,3,CV_64F);
    cv::Mat R,T;
    decomposeProjectionMatrix(P_de,MycameraMatrix,R,T);
    MycameraMatrix=MycameraMatrix/MycameraMatrix.at<double>(2,2);
    
    std::cout << "The K matrix is : \n"
    << MycameraMatrix << "\n"
    << "The R matrix is : \n"
    << R << "\n"
    << "The T matrix is : \n"
    << T << "\n";
    
    return P_de;
    
}

void cost_function(const double* p, int m_data, const void *data, double *fvec, int *info) {
    image_world_points *D = (image_world_points*)data;
    
    cv::Mat cameramatrix = (cv::Mat_<double>(3, 4) << p[0], p[1], p[2],  p[3],
                            p[4], p[5], p[6],  p[7],
                            p[8], p[9], p[10], p[11]);
    
    size_t size = D->image.size();
    for(size_t i = 0; i < size; ++i) {
        fvec[i]  = geometric_error(D->image[i], D->world[i], cameramatrix);
    }
};

double
geometric_error(const cv::Point2d& image_point,
                const cv::Point3d& world_point,
                const cv::Mat cameramatrix) {
    cv::Mat ip = (cv::Mat_<double>(3,1) <<
        image_point.x, image_point.y, 1);
    
    cv::Mat wp = (cv::Mat_<double>(4,1) <<
                  world_point.x, world_point.y, world_point.z, 1);
    cv::Mat ipp = cameramatrix*wp;
    ipp = ipp/ipp.at<double>(2);
    return cv::norm(ip,ipp,cv::NORM_L2);
    
}
