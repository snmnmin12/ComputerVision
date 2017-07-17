////
////  main.cpp
////  ComputerVisionProject4
////
////  Created by HJKBD on 4/10/17.
////  Copyright © 2017 HJKBD. All rights reserved.
////
//
#include "global.h"
#include "calibrate.h"
#include "p3p.h"

int main(int argc, char** argv) {
    
    std::string imgfile = "../pic1.JPG";

//    std::string imgfile2 = "6.jpg";
    cv::Mat img = cv::imread(imgfile);
//    cv::Mat img2 = cv::imread(imgfile2);
    
    if (!img.data) {
        std::cout <<"The image can't be loaded!" <<std::endl;
    }
    //calibrate image from Opencv
    cv::Mat cameraMatrix;
    cv::Mat distortioncoefficient;
    bool success = CalibrateOpenCV(img, cameraMatrix, distortioncoefficient);
    
    if (!success)
        std::cout <<"Unable to find the camera parameters!" <<std::endl;
//    std::cout << "The calibrated camera is :\n"<< cameraMatrix << std::endl;
    
    cv::Size patternSize(8,6);

    
    //step 1 found points
    bool found;
    cv::Mat showimg;
    img.copyTo(showimg);
    cv::Mat gray_img;
    std::vector<cv::Point2f> corners;
    cv::cvtColor(img, gray_img, CV_RGB2GRAY);
    found = cv::findChessboardCorners(img, patternSize, corners);
    if (found) {
        cornerSubPix(gray_img, corners, cv::Size(11,11), cv::Size(-1,-1),
                     cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
//                drawChessboardCorners(showimg, patternSize, cv::Mat(corners), found);
    }else {
        std::cerr << "Fail to find chessborad!" << std::endl;
        return 0;
    }
    
    
    //find the four points in the image plane and world plane
    int squareSize = 30;
    std::vector<cv::Point3f> opoints = {{0,0,0}, {7*(float)squareSize,0,0},
        {0,5*(float)squareSize,0}, {7*(float)squareSize,5*(float)squareSize,0}};
    
//    std::vector<cv::Point3f> opoints = {{float(patternSize.width*squareSize+2.6), 0, float(2.6+squareSize)},
//        {float(patternSize.width*squareSize+2.6), 0, float(2.6+patternSize.width*squareSize)},
//        {float(patternSize.width*squareSize+2.6), float(7*squareSize), float(2.6+squareSize)},
//        {float(patternSize.width*squareSize+2.6), (float)7*squareSize, float(2.6+patternSize.width*squareSize)}};
    
    std::vector<cv::Point2f> ipoints = {{corners[0].x, corners[0].y}, {corners[7].x, corners[7].y},
        {corners[40].x, corners[40].y}, {corners[47].x, corners[47].y}};
    
    //show the four points on the images
    for (int i = 0; i < ipoints.size(); i++)
        cv::putText(showimg,cv::format("%c", 'A'+i),ipoints[i],cv::FONT_HERSHEY_SIMPLEX,1,CV_RGB(255,0,0),1, CV_AA);
    
    //undistor the points first, and then with our
    cv::Mat R, tvec;
    cv::Mat undistortedPoints;
    cv::undistortPoints(ipoints, undistortedPoints, cameraMatrix, distortioncoefficient);
    bool solved = solvep3p(cameraMatrix, R, tvec, opoints, undistortedPoints);
    if (!solved) {
        std::cout << "The p3p problem can't be solved easily!" << std::endl;
        return 0;
    }
    cv::Vec3f angles = rotationMatrixToEulerAngles(R);
    std::cout << "Rotation Matrix from p3p is :\n" <<
              angles << std::endl;
    std::cout << "Translation matrix from p3p is :\n" <<
     tvec << std::endl;
    
    
    //with opencv functions sovlePnp to find the values
    cv::Mat rec, tvec2, R2;
    cv::solvePnP(opoints, ipoints, cameraMatrix, distortioncoefficient, rec, tvec2,false,CV_P3P);
    cv::Rodrigues(rec, R2);
    angles = rotationMatrixToEulerAngles(R2);
    std::cout << "Rotation Matrix from pnp is :\n" <<
    angles << std::endl;
    std::cout << "Translation matrix from pnp is :\n" <<
    tvec2 << std::endl;
    
    
    //with opencv functions sovlePnp to find the values
    cv::solvePnP(opoints, ipoints, cameraMatrix, distortioncoefficient, rec, tvec2,false,CV_ITERATIVE);
    cv::Rodrigues(rec, R2);
    angles = rotationMatrixToEulerAngles(R2);
    std::cout << "Rotation Matrix from pnp iterative is :\n" <<
    angles << std::endl;
    std::cout << "Translation matrix from pnp iterative is :\n" <<
    tvec2 << std::endl;
    
    
    //with opencv functions sovlePnp to find the values
    cv::solvePnP(opoints, ipoints, cameraMatrix, distortioncoefficient, rec, tvec2,false,CV_EPNP);
    cv::Rodrigues(rec, R2);
    angles = rotationMatrixToEulerAngles(R2);
    std::cout << "Rotation Matrix from CV_EPNP is :\n" <<
    angles << std::endl;
    std::cout << "Translation matrix from CV_EPNP iterative is :\n" <<
    tvec2 << std::endl;
    
    //with opencv functions sovlePnp to find the values
//    cv::solvePnP(opoints, ipoints, cameraMatrix, distortioncoefficient, rec, tvec2,false,CV_P3P);
//    cv::Rodrigues(rec, R2);
//    angles = rotationMatrixToEulerAngles(R2);
//    std::cout << "Rotation Matrix from pnp is :\n" <<
//    angles << std::endl;
//    std::cout << "Translation matrix from pnp is :\n" <<
//    tvec2 << std::endl;
    
    //step
    //    std::string name = "test";
    //    cv::namedWindow(name);
    //    cv::imshow(name,undisimg);
    //    cv::waitKey();
    cv::imwrite("p3p_image.jpg", showimg);
    return 0;
}
