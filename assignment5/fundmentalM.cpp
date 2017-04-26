//
//  util.cpp
//  ComputerVisionProject4
//
//  Created by HJKBD on 4/8/17.
//  Copyright © 2017 HJKBD. All rights reserved.
//

#include "fundmentalM.h"
#include "lmmin.h"

cv::Mat normalize(const std::vector<cv::Point2f>& points) {
    
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



std::vector<cv::Point2d>
normalize_points(const std::vector<cv::Point2f>& points, const cv::Mat& trans)
{
    std::vector<cv::Point2d> output;
    output.reserve(points.size());
    
    for(const auto&p2f: points) {
        // Make a homogeneous representation of point i.
        cv::Mat_<double> p = (cv::Mat_<double>(3,1) << p2f.x, p2f.y, 1.0);
        
        // Transform p by _trans.
        cv::Mat_<double> p1 = trans * p;
        
        // Dehomogenize and store the result in output.
        output.emplace_back(p1(0) / p1(2), p1(1) / p1(2));
    }
    
    // Debug.
//    std::cout << "\nNormalized points:";
//    for(const auto& point : output)
//        std::cout << "\n" << point;
//    std::cout << std::endl;
    
    return output;
}

cv::Mat dlt(const std::vector<cv::Point2f>& image1_points,
            const std::vector<cv::Point2f>& image2_points)
{
    if (image1_points.size() != image2_points.size())
        throw std::runtime_error("The image point size and wold point size doesn't match! but image point size: " + std::to_string(image1_points.size()) + "world point size: " + std::to_string(image2_points.size()));
   
    cv::Mat T1 = normalize(image1_points);
    cv::Mat T2 = normalize(image2_points);
    
    std::vector<cv::Point2d> n_points1 = normalize_points(image1_points, T1);
    std::vector<cv::Point2d> n_points2 = normalize_points(image2_points, T2);
    
    cv::Mat_<double> matrix(int(n_points1.size()), 9);
    for (int i = 0; i < image1_points.size(); i++) {
        const auto& p1 = n_points1[i];
        const auto& p2 = n_points2[i];
        cv::Mat_<double> row = (cv::Mat_<double>(1,9) <<
        p2.x*p1.x, p2.x*p1.y, p2.x, p2.y*p1.x, p2.y*p1.y, p2.y, p1.x, p1.y, 1);
        row.copyTo(matrix.row(i));
    }
    
    //solve the dlt equation
    cv::Mat_<double> U,Sig, Vt;
    cv::SVD::compute(matrix, U, Sig, Vt, cv::SVD::FULL_UV);
    
    // The last row of Vt holds the parameters for our camera matrix. Reshape it
    // to a 3x4 matrix with the same number of channels.
    cv::Mat_<double> F = Vt.row(8);
//    std::cout << "\nNew fundamental matrix is :\n" << matrix*F.t() << std::endl;
    F = F.reshape(0, 3);
//    std::cout << "\nNew fundamental matrix is :\n" << F << std::endl;
    //SVD of F again
    cv::SVD::compute(F, Sig,U, Vt, cv::SVD::FULL_UV);
    cv::Mat DD = cv::Mat::diag(Sig);
//    std::cout << "\ndiagnal matrix is :\n" << DD << std::endl;
    DD.at<double>(2,2) = 0;
    F = U*DD*Vt;
    // Debug.
//    std::cout << "\nNew fundamental matrix is :\n" << matrix*F.reshape(0,9) << std::endl;
    cv::Mat mat = T2.t()*F*T1;
//    std::cout << "\nNew fundamental matrix is :\n" << F << std::endl;
    mat /= mat.at<double>(2,2);
    return mat;
}


void ComputeEpipoles (const cv::Mat& F, cv::Mat& e , cv::Mat& ep )
{
    cv::Mat D, U, Vt;
    // F = U D V
    cv::SVD::compute(F, D, U, Vt, cv::SVD::FULL_UV);
    // e = t h e l a s t column o f VˆT, i . e . , l a s t row o f V
    // ep = t h e l a s t column o f UˆT, i . e . , l a s t row o f U
    cv::Mat e_ = Vt.row(2).t();
    e_.copyTo(e);
    U.col(2).copyTo(ep);
//    std::cout << "epipoles : \n" ;
//    std::cout << F*e << std::endl;
//    std::cout << ep.t()*F << std::endl;
}

cv::Mat Vector2SkewMatrix (const double v[3] )
{
    cv::Mat vx = cv::Mat::zeros(3, 3, CV_64F);
    vx.at<double>(0,1) = -v[2];
    vx.at<double>(0,2) =  v[1];
    vx.at<double>(1,0) =  v[2];
    vx.at<double>(1,2) = -v[0];
    vx.at<double>(2,0) = -v[1];
    vx.at<double>(2,1) = v[0];
    return vx;
}
void ComputeCameraMatrices (const cv::Mat& F, const cv::Mat& ep , cv::Mat& P, cv::Mat& Pp)
{
    cv::Mat epx, tmp;
    P = cv::Mat::eye(3, 4, CV_64F);
    Pp = cv::Mat::zeros(3, 4, CV_64F);
    const double* ptr = ep.ptr<double>();
    epx = Vector2SkewMatrix (ptr);
    tmp = epx*F;
    hconcat(tmp, ep, Pp);
}

std::vector<cv::Point3f> triangulate_to_3D(const std::vector<cv::Point2f>& i1_points, const std::vector<cv::Point2f>& i2_points,
                        const cv::Mat& P1, const cv::Mat& P2)
{
//    cv::Mat A = cv::Mat::zeros(4, 4, CV_64F);
//    cv::Mat row1, row2, row3, row4, D, U, Vt,X;
    std::vector<cv::Point3f> output;
    for (int i = 0; i < i1_points.size(); i++) {
        cv::Point3f point = triangulate_to_3D(i1_points[i], i2_points[i],P1, P2);
        output.push_back(point);
    }
    return output;
}

cv::Point3f triangulate_to_3D(const cv::Point2f& i_point1, const cv::Point2f& i_point2,
                                           const cv::Mat& P1, const cv::Mat& P2)
{
    cv::Mat A = cv::Mat::zeros(4, 4, CV_64F);
    cv::Mat row1, row2, row3, row4, D, U, Vt,X;
    std::vector<cv::Point3f> output;
    double x1 = i_point1.x;
    double y1 = i_point1.y;
    double x2 = i_point2.x;
    double y2 = i_point2.y;
    row1 = x1*P1.row(2) - P1.row(0);
    row2 = y1*P1.row(2) - P1.row(1);
    row3 = x2*P2.row(2) - P2.row(0);
    row4 = y2*P2.row(2) - P2.row(1);
    row1.copyTo(A.row(0));
    row2.copyTo(A.row(1));
    row3.copyTo(A.row(2));
    row4.copyTo(A.row(3));
    //        std::cout << A << std::endl;
    cv::SVD::compute(A, D, U, Vt);
    X = Vt.row(3);
//    std::cout << D << std::endl;
    //        std::cout << A*X.t() << std::endl;
    //        std::cout << X << std::endl;
    double X1 = X.at<double>(0);
    double X2 = X.at<double>(1);
    double X3 = X.at<double>(2);
    double X4 = X.at<double>(3);
    return cv::Point3f(X1/X4,X2/X4,X3/X4);
}

cv::Mat refine_F(const std::vector<cv::Point2f>& i1_points, const std::vector<cv::Point2f>& i2_points,
              const cv::Mat& P1, const cv::Mat& P2, cv::Mat& P2_r, std::vector<cv::Point3f>& XW)
{
    int rows = P2.rows;
    int cols = P2.cols;

    size_t num = i1_points.size();
    double pars[rows*cols + 3*num];
    double data[num*4];
    
    for (int i = 0; i < num; i++) {
        data[4*i]   = i1_points[i].x;
        data[4*i+1] = i1_points[i].y;
        data[4*i+2] = i2_points[i].x;
        data[4*i+3] = i2_points[i].y;
    }
    
    //fill the initial parameter array
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            pars[i*(rows+1) + j] = P2.at<double>(i,j);
    
    //fill in the world coordinate
    XW =  triangulate_to_3D(i1_points, i2_points, P1, P2);
    for(int i = 0; i < num; i++) {
        pars[rows*cols + 3*i] = XW[i].x;
        pars[rows*cols + 3*i+1] = XW[i].y;
        pars[rows*cols + 3*i+2] = XW[i].z;
    }

    lm_control_struct control = lm_control_double;
    lm_status_struct status;
    //cout << "my m_dat = "<< endl << " " << m_dat << endl << endl;
    double error[num*4];
    Cost_Fun(pars, int(num)*4, &data, error, nullptr);
    
    double errors = 0.0;
    for (int i = 0; i < num*4; i++)
        errors += pow(error[i],2);
    std::cout <<"The initial error is: "<< errors/20 <<std::endl;
    
    
    int n_par = (int)(rows*cols + 3*num);
    lmmin(n_par, pars, int(num)*4, &data, Cost_Fun, &control, &status);
    std::cout <<"The final error is: "<< pow(status.fnorm,2)/20<<std::endl;
    
    
    P2_r = (cv::Mat_<double>(3,4) << pars[0],pars[1],
                  pars[2],pars[3],pars[4],pars[5],pars[6],pars[7] ,pars[8],pars[9],pars[10],pars[11]);

    for (size_t i = 0; i < num; i++)
    {
        XW[i] = cv::Point3f(pars[12+3*i],pars[12+3*i+1],pars[12+3*i+2]);
    }
//    std::cout << "Refined P2_r is :"<< P2_r << std::endl;
    double t[3] =  {P2_r.at<double>(0,3),P2_r.at<double>(1,3),P2_r.at<double>(2,3)};
    cv::Mat e_cross = Vector2SkewMatrix (t);
//    std::cout << "e_cross is : "<< e_cross << std::endl;
    cv::Mat M = P2_r(cv::Range(0,3),cv::Range(0,3));
//    std::cout << "M is : "<< M << std::endl;
    cv::Mat F_r = e_cross*M;
//    std::cout << "Fr is :" << F_r << std::endl;
    // After the iteration, par has the approached P values
    return F_r;
}

void Cost_Fun( const double *par, int m_dat, const void *data, double *fvec, int *info ) {
    
    double*D = (double*) data;
    // Initialize Projection matrix P from parameters par
    cv::Mat P1 = cv::Mat::eye(3,4,CV_64F);
    cv::Mat P2 = (cv::Mat_<double>(3,4) << par[0],par[1],
                  par[2],par[3],par[4],par[5],par[6],par[7] ,par[8],par[9],par[10],par[11]);
    
//    std::cout <<"P1 is: " << P1 << std::endl;
//    std::cout <<"P2 is: " << P2 << std::endl;
    // limfit is not written for OpenCV. It is really hard to use.
    // However, the LM solver in OpenCV is not available to use either.
    cv::Point2f est1, est2;
    for (int i = 0; i < m_dat/4; i++) {
        cv::Mat X_world = (cv::Mat_<double>(4,1)<<par[12+3*i], par[12+3*i+1], par[12+3*i+2], 1.0);
        cv::Mat xp1 = P1*X_world;
        cv::Mat xp2 = P2*X_world;
        xp1 = xp1/xp1.at<double>(0,2);
        xp2 = xp2/xp2.at<double>(0,2);
        est1 = cv::Point2f(xp1.at<double>(0,0), xp1.at<double>(0,1));
        est2 = cv::Point2f(xp2.at<double>(0,0), xp2.at<double>(0,1));
        cv::Point2f p1(D[4*i],D[4*i+1]);
        cv::Point2f p2(D[4*i+2],D[4*i+3]);
        fvec[4*i]= (est1.x-p1.x);
        fvec[4*i+1]= (est1.y-p1.y);
        fvec[4*i+2] = (est2.x-p2.x);
        fvec[4*i+3] = (est2.y-p2.y);
    }
    
}
