//
//  project2.hpp
//  ComputerVisionProject1
//  @author Mingmin Song
//  Created by HJKBD on 2/21/17.
//  Copyright © 2017 HJKBD. All rights reserved.
//
#ifndef project2_hpp
#define project2_hpp
#include "util.h"
//helper functions
Mat normalizeT(vector<int>& vec) {

    Mat T = Mat::zeros(3,3,CV_64FC1);
    //This is to compute the average center of x and y
    double meanx = 0, meany = 0;
    for (int i = 0; i < vec.size(); i +=2) {
        meanx += vec[i];
        meany += vec[i+1];
    }
    size_t len = vec.size()/2;
    meanx = meanx/(double)len;
    meany = meany/(double)len;
    
    double sumsquare = 0.0;
    for (int i = 0; i < vec.size(); i +=2) {
        sumsquare += sqrt(pow(vec[i]-meanx,2.0)+pow(vec[i+1]-meany,2.0));
    }
    //compute scale
    double s = sqrt(2)*len/sumsquare;
    //normalization matrix
    T.at<double> (0, 0) = s;
    T.at<double> (1, 1) = s;
    T.at<double> (2, 2) = 1.0;
    T.at<double> (0, 2) = -s*meanx;
    T.at<double> (1, 2) = -s*meany;
    return T;
}
Mat findH(vector<Point3d>& pts, vector<Point3d>& des) {
    // Declare variables
    int n = (int)pts.size();
    Mat H_vec, Htilde;
    Mat A;
    // We need to minimize ||Ah|| s.t. ||h|| = 1, with A being
    // 2n x 9, b being 8 X 1 and h being9x1
    // Fill A matrix
    for (int i = 0; i < n; i++) {
        Mat temp1  = (Mat_<double>(1,9) << 0, 0, 0, -pts[i].x, -pts[i].y, -1, des[i].y*pts[i].x, des[i].y*pts[i].y, des[i].y);
        Mat temp2  = (Mat_<double>(1,9) << pts[i].x, pts[i].y, 1, 0, 0, 0, -des[i].x*pts[i].x, -des[i].x*pts[i].y, -des[i].x);
        i+=2;
        A.push_back(temp1);
        A.push_back(temp2);
    }
    //cout << "A = " << A << endl;
    Mat d,u,ut;
    SVD::compute(A, d, u, ut);
    H_vec = ut(Range(ut.rows-1,ut.rows),Range::all());
    cout << H_vec << endl;
    //cout << "H_vec = " << H_vec << endl;
    // Reshape H vector into matrix form
    Htilde = H_vec.reshape(0, 3);
    cout << "Htilde = " << Htilde << endl;
    return Htilde;
}

void project2_question1(vector<int>&input1, vector<int>&input2) {
    //read images
    Mat image = imread(imagefile1);
    Mat image2 = imread(imagefile2);

    if (!image.data || !image2.data) {
        cout << "Can't find data!" << endl;
        return;
    }
    
    //processing input and draw points
    for (int i = 0; i < input1.size(); i+=2) {
        pts.push_back(Point3d(input1[i],input1[i+1],1));
        des.push_back(Point3d(input2[i],input2[i+1],1));
        circle(image, Point(input1[i],input1[i+1]), 4, red);
        circle(image2, Point(input2[i],input2[i+1]), 4, red);
    }
    //find original H
    Mat H = findH(pts,des);
    cout << " H= "<< H <<endl;
    //determine the size of transformed corner coordinate
    double minx = INFINITY, miny = INFINITY;
    vector<Point2d> in = {Point2d(0,0), Point2d(0,image.cols-1),Point2d(image.rows-1,0),Point2d(image.rows-1,image.cols-1)};
    vector<Point2d> out;
    perspectiveTransform(in, out, H);
    for (int i = 0; i < out.size(); i++){
        minx = min(minx, out[i].x);
        miny = min(miny,out[i].y);
    }
    double Htrdata[3][3] = {{1,0,-minx},{0,1,-miny},{0,0,1}};
    Mat Htr = Mat(3,3,CV_64FC1,Htrdata);
    
    //size of the output image
    Mat img_out = Mat(image2.rows,image.cols+image2.cols, image2.type());
    Mat image1 = imread(imagefile1);
    warpPerspective(image1, img_out,Htr*H, image.size()*4);
    Mat part(img_out,Rect(-minx,-miny,image2.cols,image2.rows));
    Mat temp = imread(imagefile2);
    temp.copyTo(part);
    
//    string title1 = "test1";
//    string title2 = "test2";
//    string title3 = "result";
//    namedWindow(title1);
//    imshow(title1,image);
//    namedWindow(title2);
//    imshow(title2,image2);
//    namedWindow(title3);
//    imshow(title3,img_out);
//    waitKey(0);
        imwrite("q1_1.png", image);
        imwrite("q1_2.png", image2);
        imwrite("q1_out.png", img_out);
    cout<<"Project finish"<<endl;
}
void project2_question2(vector<int>&input1, vector<int>&input2) {
    
    Mat image = imread(imagefile1);
    Mat image2 = imread(imagefile2);
    
    if (!image.data || !image2.data) {
        cout << "Can't find data!" << endl;
        return;
    }
    //normalize T1 and T2
    Mat T1 = normalizeT(input1);
    Mat T2 = normalizeT(input2);
    
//    cout << "T1 = " << T1 << endl;
//    cout << "T2 = " << T2 << endl;
    double cdata[40][9];
    
    //draw points
    for (int i = 0; i < input1.size(); i+=2) {
        circle(image, Point(input1[i],input1[i+1]), 4, red);
        circle(image2, Point(input2[i],input2[i+1]), 4, red);
    }
    
    //find corresponding corner points
    for (int i = 0; i < input1.size(); i +=2 ) {
        //Mat temp(Point3d(input1[i],input1[i+1],1));
        Mat point = T1*Mat(Point3d(input1[i],input1[i+1],1));
        point.at<double>(0) = point.at<double>(0) /point.at<double>(2);
        point.at<double>(1) = point.at<double>(1) /point.at<double>(2);
        pts.push_back(Point3d(point));
        point = T2*Mat(Point3d(input2[i],input2[i+1],1));
        point.at<double>(0) = point.at<double>(0) /point.at<double>(2);
        point.at<double>(1) = point.at<double>(1) /point.at<double>(2);
        des.push_back(Point3d(point));
    }
    Mat H = findH(pts,des);
    H = T2.inv()*H*T1;
    cout <<"H = "<< H <<endl;
    //translate the image
    //    vector<Point2d> in = {Point2d(0,0), Point2d(0,image.cols-1),Point2d(image.rows-1,0),Point2d(image.rows-1,image.cols-1)};
    vector<Point2d> in = {Point2d(0,0), Point2d(image.cols-1,0),Point2d(0,image.rows-1),Point2d(image.cols-1,image.rows-1)};
    vector<Point2d> out;
    double minx = INFINITY,miny = INFINITY, maxx =-INFINITY, maxy = -INFINITY ;
    perspectiveTransform(in, out, H);
    for (int i = 0; i < out.size(); i++){
        minx = min(minx, out[i].x);
        maxx = max(minx, out[i].x);
        miny = min(miny,out[i].y);
        maxy = max(miny,out[i].y);
    }
    double htr[3][3] = {{1,0,-minx},{0,1,-miny},{0,0,1}};
    
    //    double htr[3][3] = {{1,0,(double)(image2.cols)},{0,1,0},{0,0,1}};
    Mat htra = Mat(3,3,CV_64FC1, htr);
    cout << htra<<endl;
    //Mat img_out = Mat(image2.rows,image.cols+image2.cols, image2.type());
    Mat img_out = Mat(image2.rows-miny,image2.cols-minx, image2.type());
    // image = imread(imagefile1);
    warpPerspective(image, img_out, htra*H, img_out.size());
    Mat temp = imread(imagefile2);
    temp.copyTo(img_out(Rect(-minx,-miny,image2.cols,image2.rows)));
//    string title1 = "test1";
//    string title2 = "test2";
//    string title3 = "result";
//    namedWindow(title1,CV_WINDOW_NORMAL);
//    imshow(title1,image);
//    namedWindow(title2,CV_WINDOW_NORMAL);
//    imshow(title2,image2);
//    namedWindow(title3,CV_WINDOW_NORMAL);
//    imshow(title3,img_out);
//    waitKey(0);
    imwrite("q2_1.png", image);
    imwrite("q2_2.png", image2);
    imwrite("q2_out.png", img_out);
    cout<<"Project finish"<<endl;
}
#endif
