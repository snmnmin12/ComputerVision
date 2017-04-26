//
//  calibration.cpp
//  ComputerVisionProject4
//
//  Created by HJKBD on 4/9/17.
//  Copyright © 2017 HJKBD. All rights reserved.
//

#include "calibration.h"
#include "gold_standard.h"

cv::Mat CameraCalibration(cv::Mat img) {
    // Conver image to gray-scale
    Mat mat;

    cv::Mat gray_img;
    cvtColor(img, gray_img, CV_RGB2GRAY);
    cv::Size patternSize(7,7);
    //Size patternSize(6,9);
    int squareSize = 26;
    //-- Step 1: Find the inner corner positions of the chessboard
    std::vector<cv::Point2d> corners;
    std::vector<cv::Point2f> corners_1, corners_2;
    bool found = findChessboardCorners(gray_img, patternSize, corners_1);
    if (found) {
        cv::cornerSubPix(gray_img, corners_1, cv::Size(11,11), cv::Size(-1,-1),
                     cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        //drawChessboardCorners(img, patternSize, Mat(corners_1), found);
        for (int i=0; i<corners_1.size(); i++) {
            cv::putText(img,cv::format("%d", i),corners_1[i],cv::FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(0,255,0),1, CV_AA);
            corners.push_back((cv::Point2d)corners_1[i]);
            //cout << "x " << corners_1[i].x << "  y  " << corners_1[i].y << endl;
        }
        //cout << corners_1.size() << endl;
    }
    else {
        std::cerr << "Fail to find chessboard!"  << std::endl;
    }
    
    found = findChessboardCorners(img, patternSize, corners_2);
    if (found) {
        cornerSubPix(gray_img, corners_2, cv::Size(11,11), cv::Size(-1,-1),
                     cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        //drawChessboardCorners(img, patternSize, Mat(corners_1), found);
        for (int i=0; i<corners_2.size(); i++) {
            cv::putText(img,cv::format("%d", i),corners_2[i],cv::FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(0,255,0),1, CV_AA);
            corners.push_back((cv::Point2d)corners_2[i]);
            //cout << "x " << corners_2[i].x << "  y  " << corners_2[i].y << endl;
        }
        //cout << corners_2.size() << endl;
    }
    else {
        std::cerr << "Fail to find chessboard!"  << std::endl;
    }
    
    
    //-- Step 2: Find the world coordinates and image coordinates of chessboard corners
    
    
    std::vector<cv::Point3d> obj;
    for (int i=0; i < patternSize.height; i++) {
        for (int j=0; j < patternSize.width; j++) {
            obj.push_back(cv::Point3f((double)j * squareSize, (double)i * squareSize, 0));
            //cout << "x " << j * squareSize << "  y  " << i * squareSize << "   z    " << 0 << endl;
            
        }
    }
    for (int j=0; j < patternSize.width; j++) {
        for (int i=0; i < patternSize.height; i++) {
            obj.push_back(cv::Point3d((double)patternSize.width*squareSize, (double)i*squareSize, (double)(-1.)*patternSize.width*squareSize+j*squareSize));
            //cout << "x " << patternSize.width*squareSize << "  y  " << i*squareSize << "   z    " << (-1.)*patternSize.width*squareSize+j*squareSize<< endl;
        }
    }
    //-- Step 3: Project world 3D points into the image points
    cv::Mat P = gold_standard(corners, obj);
    cv::Mat MycameraMatrix(3,3,CV_32FC1);
    cv::Mat R,T;
    
    decomposeProjectionMatrix(P,MycameraMatrix,R,T);
    MycameraMatrix=MycameraMatrix/MycameraMatrix.at<float>(2,2);
    
    std::cout << "P = "<< std::endl << " " << P << std::endl << std::endl;
    //cout << "MycameraMatrix = "<< endl << " " << MycameraMatrix << endl << endl;
    
    imshow("Calibration Image", img);
    imwrite("my_Calibration_image.jpg", img);
    return P;
}
cv::Mat CalibrateOpenCV (cv::Mat img) {
    
    cv::Mat img_c = img.clone();
    cv::Mat gray_img;
    cv::cvtColor(img, gray_img, CV_RGB2GRAY);
    int squareSize = 26;
    
    std::vector<cv::Point2f> corners;
    cv::Size patternSize(7,7);
//    cv::Size patternSize(8,6);
    
    //-- Step 1: Find the inner corner positions of the chessboard
    bool found = cv::findChessboardCorners(gray_img, patternSize, corners);
    if (found) {
        cornerSubPix(gray_img, corners, cv::Size(11,11), cv::Size(-1,-1),
                     cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        drawChessboardCorners(img, patternSize, cv::Mat(corners), found);
    }else {
        std::cerr << "Fail to find chessborad!" << std::endl;
    }
    
    //-- Step 2: Find the world coordinates and image coordinates of chessboard corners
    std::vector<std::vector<cv::Point3f> > object_points;
    std::vector<std::vector<cv::Point2f> > image_points;
    
    std::vector<cv::Point3f> obj;
    for (int i=0; i < patternSize.height; i++) {
        for (int j=0; j < patternSize.width; j++) {
            obj.push_back(cv::Point3d((float)j * squareSize, (float)i * squareSize, 0));
        }
    }
    object_points.push_back(obj);
    image_points.push_back(corners);
    
    //-- Step 3: Camera calibration
    
    std::vector<cv::Mat> rotationVectors;
    std::vector<cv::Mat> translationVectors;
    
    cv::Mat distortionCoefficients;
    cv::Mat cameraMatrix;
    
    int flags = 0;
    double rms = calibrateCamera(object_points, image_points, img.size(), cameraMatrix,
                                 distortionCoefficients, rotationVectors, translationVectors,
                                 flags|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);

    cv::Mat R,t;
    cv::Mat E(3,4,CV_64F);
    cv::Rodrigues(rotationVectors[0],R);
    cv::hconcat(R, translationVectors[0], E);
    cv::Mat P = cameraMatrix*E;
    P = P/P.at<double>(2,3);
    std::cout << "RMS: " << rms << std::endl;
    std::cout << "Camera projection matrix P: " << P << std::endl<< std::endl;
    //cout << "Distortion coefficients: " << distortionCoefficients << endl<< endl;
    //cout << "Extrinsic: " << E << "    "<< endl<< endl;
    //cout << "Intrinsic: " << cameraMatrix << endl<< endl;
    
    //Remove distortion
    cv::Mat new_img;
    cv::undistort(img_c, new_img, cameraMatrix, distortionCoefficients);
    cv::imshow("Opencv Image", new_img);
    cv::imwrite("OpenCV_undistorted_image.jpg", new_img);
    cv::waitKey();
    return new_img;
}

