//
//  calibrate.cpp
//  ComputerVisionProject4
//
//  Created by HJKBD on 4/10/17.
//  Copyright © 2017 HJKBD. All rights reserved.
//

#include "calibrate.h"



// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(cv::Mat &R)
{
    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());
    return  cv::norm(I, shouldBeIdentity) < 1e-6;
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R)
{
    
    assert(isRotationMatrix(R));
    
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
    
    bool singular = sy < 1e-6; // If
    
    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    double pitoangle = 180./CV_PI;
    return cv::Vec3f(x*pitoangle, y*pitoangle, z*pitoangle);
    
}


bool CalibrateOpenCV (cv::Mat& img, cv::Mat& cameraMatrix, cv::Mat& distortionCoefficients) {
    
    cv::Mat gray_img;
    cv::cvtColor(img, gray_img, CV_RGB2GRAY);
    
    int squareSize = 30;
    
    std::vector<cv::Point2f> corners, corners2;
    cv::Size patternSize(8,6);
    
    cv::Mat showimg;
    img.copyTo(showimg);
    //-- Step 1: Find the inner corner positions of the chessboard
    bool found = cv::findChessboardCorners(gray_img, patternSize, corners);
    std::vector<cv::Point3f> obj;
    if (found) {
        cornerSubPix(gray_img, corners, cv::Size(11,11), cv::Size(-1,-1),
                     cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        for (int i = 0; i < corners.size(); i++)
        {
            cv::putText(showimg,cv::format("%d", i),corners[i],cv::FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(0,255,0),1, CV_AA);
            std::cout << corners[i] <<", ";
        }
//        drawChessboardCorners(showimg, patternSize, cv::Mat(corners), found);
        for (int i=0; i < patternSize.height; i++) {
            for (int j=0; j < patternSize.width; j++) {
                obj.push_back(cv::Point3d((float)j * squareSize, (float)i * squareSize, 0));
            }
        }
    }else {
        std::cerr << "Fail to find chessborad!" << std::endl;
        return false;
    }
    
    //-- Step 2: Find the world coordinates and image coordinates of chessboard corners
    std::vector<std::vector<cv::Point3f> > object_points;
    std::vector<std::vector<cv::Point2f> > image_points;

    object_points.push_back(obj);
    image_points.push_back(corners);
    //-- Step 3: Camera calibration
    
    std::vector<cv::Mat> rotationVectors;
    std::vector<cv::Mat> translationVectors;
    
    int flags = 0;
    double error = calibrateCamera(object_points, image_points, img.size(), cameraMatrix,
                                 distortionCoefficients, rotationVectors, translationVectors,
                                 flags|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
    cv::Mat R,t;
    cv::Rodrigues(rotationVectors[0],R);
    t = translationVectors[0];
    cv::Vec3f angles = rotationMatrixToEulerAngles(R);
//    std::cout <<"The roation matrix in calibration step is: \n" << angles << "\n";
    std::cout <<"The translation vector in calibration step is : \n" << t << "\n";
    std::cout <<"The camera intrinsic is: \n" << cameraMatrix << "\n";
    std::cout <<"The distortion coefficient is: \n" << distortionCoefficients << "\n";
    
    cv::Mat E(3,4,CV_64F);
    cv::hconcat(R, t, E);
    cv::Mat P = cameraMatrix*E;
    P = P/P.at<double>(2,3);
//    std::cout << "Camera projection matrix P: " << P << std::endl<< std::endl;
    
    //Remove distortion
//    cv::Mat new_img;
//    cv::undistort(img, new_img, cameraMatrix, distortionCoefficients);
//    cv::imshow("Opencv Image", img);
//    cv::imwrite("OpenCV_undistorted_image.jpg", showimg);
    cv::imshow("Opencv", showimg);
    cv::waitKey();
    return true;
}

//bool CalibrateOpenCV2 (cv::Mat& img, cv::Mat& cameraMatrix, cv::Mat& distortionCoefficients) {
//    
//    cv::Mat gray_img;
//    cv::cvtColor(img, gray_img, CV_RGB2GRAY);
//    
//    int squareSize = 30;
//    
//    std::vector<cv::Point2f> corners1, corners2;
//    std::vector<cv::Point3f> obj1, obj2;
//    cv::Size patternSize(8,6);
//    
//    cv::Mat showimg;
//    img.copyTo(showimg);
//    //-- Step 1: Find the inner corner positions of the chessboard
//    bool found = cv::findChessboardCorners(gray_img, patternSize, corners1);
//    if (found) {
//        cornerSubPix(gray_img, corners1, cv::Size(11,11), cv::Size(-1,-1),
//                     cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
//        for (int i = 0; i < corners1.size(); i++)
//        {
//            cv::putText(showimg,cv::format("%d", i),corners1[i],cv::FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(0,255,0),1, CV_AA);
//            std::cout << corners1[i] <<", ";
//        }
//        drawChessboardCorners(showimg, patternSize, cv::Mat(corners1), found);
//        for (int i=0; i < patternSize.height; i++) {
//            for (int j=0; j < patternSize.width; j++) {
//                obj1.push_back(cv::Point3f((float)patternSize.width*squareSize+2.6, (float)i*squareSize, 2.6+(j+1)*squareSize));
//            }
//        }
//    }else {
//        std::cerr << "Fail to find chessborad!" << std::endl;
//        return false;
//    }
//
//    found = cv::findChessboardCorners(showimg, patternSize, corners2);
//    if (found) {
//        cornerSubPix(gray_img, corners2, cv::Size(11,11), cv::Size(-1,-1),
//                     cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
//        drawChessboardCorners(showimg, patternSize, cv::Mat(corners2), found);
//        for (int i = 0; i < corners2.size(); i++)
//        {
//            cv::putText(showimg,cv::format("%d", i),corners2[i],cv::FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(0,255,0),1, CV_AA);
//            std::cout << corners2[i] <<", ";
//        }
//        for (int i = 0; i < patternSize.height; i++) {
//            for (int j = 0; j < patternSize.width; j++) {
////                obj.push_back(cv::Point3f((float)patternSize.width*squareSize+2.6, (float)i*squareSize, 2.6+j*squareSize));
//                obj2.push_back(cv::Point3f((float)j * squareSize, (float)i * squareSize, 0));
//
//            }
//        }
//        
//    }else {
//        std::cerr << "Fail to find chessborad!" << std::endl;
//    }
//    
//    //-- Step 2: Find the world coordinates and image coordinates of chessboard corners
//    std::vector<std::vector<cv::Point3f> > object_points;
//    std::vector<std::vector<cv::Point2f> > image_points;
//    
//    img.copyTo(showimg);
//    std::vector<cv::Point2f> corners(corners2);
//    std::vector<cv::Point3f> obj(obj2);
//    for (int i = 0; i < corners1.size(); i++) {
//        corners.push_back(corners1[i]);
//        obj.push_back(obj1[i]);
//    }
//    
//    for (int i = 0; i < corners.size(); i++)
//    {
//        cv::putText(showimg,cv::format("%d", i),corners[i],cv::FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(0,255,0),1, CV_AA);
//    }
//    
//    
//    object_points.push_back(obj);
////    object_points.push_back(obj1);
//    image_points.push_back(corners);
////    image_points.push_back(corners1);
//    //-- Step 3: Camera calibration
//    
//    std::vector<cv::Mat> rotationVectors;
//    std::vector<cv::Mat> translationVectors;
//    
//    int flags = 0;
//    double error = calibrateCamera(object_points, image_points, img.size(), cameraMatrix,
//                    distortionCoefficients, rotationVectors, translationVectors,
//                    flags|CV_CALIB_USE_INTRINSIC_GUESS);
//    
////    double error = calibrateCamera(object_points, image_points, img.size(), cameraMatrix,
////                                   distortionCoefficients, rotationVectors, translationVectors,
////                                   flags|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
//    cv::Mat R,t;
//    cv::Rodrigues(rotationVectors[0],R);
//    t = translationVectors[0];
//    cv::Vec3f angles = rotationMatrixToEulerAngles(R);
//    std::cout <<"The roation matrix in calibration step is: \n" << angles << "\n";
//    std::cout <<"The translation vector in calibration step is : \n" << t << "\n";
//    std::cout <<"The camera intrinsic is: \n" << cameraMatrix << "\n";
//    std::cout <<"The distortion coefficient is: \n" << distortionCoefficients << "\n";
//    
//    cv::Mat E(3,4,CV_64F);
//    cv::hconcat(R, t, E);
//    cv::Mat P = cameraMatrix*E;
//    P = P/P.at<double>(2,3);
//    std::cout << "Camera projection matrix P: " << P << std::endl<< std::endl;
//    
//    //Remove distortion
//    //    cv::Mat new_img;
//    //    cv::undistort(img, new_img, cameraMatrix, distortionCoefficients);
//    //    cv::imshow("Opencv2 Image", img);
////        cv::imwrite("OpenCV2_undistorted_image.jpg", showimg);
////    cv::imshow("Opencv", showimg);
////    cv::waitKey();
//    return true;
//}

