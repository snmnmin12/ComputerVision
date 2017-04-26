//
//  util.h
//  ComputerVisionProject1
//
//  Created by HJKBD on 2/27/17.
//  Copyright © 2017 HJKBD. All rights reserved.
//

#ifndef util_h
#define util_h
#include <stdio.h>
#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
# include <random>
#include <cstring>
# include <math.h>
#include <sstream>

using namespace std;
using namespace cv;
//get coordinate on the screen by clicking on the mouth
extern Mat image;
extern string title;
extern int cou;
extern vector<Point3d> pts;
extern vector<Point3d> des;
extern string imagefile1;
extern string imagefile2;
//
//bool flag = true;
extern Scalar green;
extern Scalar red;
//void onMouse( int event, int x, int y, int, void* ) {
//    if( event != CV_EVENT_LBUTTONDOWN )
//        return;
//    if (cou < 20)
//    {
//        Point3d pt = Point3d(x,y,1);
//        if(flag) pts.push_back(pt);
//        else   des.push_back(pt);
//        circle(image, Point(x,y), 4, red);
//        std::cout<<pt.x<<","<<pt.y;
//        if (cou < 19 ) cout <<",";
//        else cout << "\n";
//        //        std::cout<<cou+1<<": "<<"x="<<pt.x<<"\t y="<<pt.y<<"\n";
//        cou++;
//    }
//    imshow(title,image);
//}
//void getpoints(string& imagefile) {
//    image = imread(imagefile);
//    namedWindow(title);
//    imshow(title, image);
//    setMouseCallback(title, onMouse, 0);
//    waitKey(0);
//}
#endif /* util_h */
