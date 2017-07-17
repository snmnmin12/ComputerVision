//
//  main.cpp
//  ComputerVisionProject5
//
//  Created by HJKBD on 4/13/17.
//  Copyright © 2017 HJKBD. All rights reserved.
//

#include "header.h"
#include "fundmentalM.h"

using namespace std;
using namespace cv;


int cou = 0;
cv::Mat image;

Scalar red = {255, 0, 0};
string title = "CV";
void onMouse( int event, int x, int y, int, void* params) {
    if( event != CV_EVENT_LBUTTONDOWN )
        return;
    if (cou < 32)
    {
        vector<Point2f>* pts = (vector<Point2f>*)params;
        cv::Point2f pt = cv::Point2f(x,y);
        pts->push_back(pt);
        cv::circle(image, Point(x,y), 4, red);
        cou++;
    }
    imshow(title,image);
}
void getpoints(const std::string& im, void*params) {
    image = cv::imread(im);
//    namedWindow(title);
    imshow(title, image);
    setMouseCallback(title, onMouse, params);
    waitKey(0);
}

int main(int argc, const char * argv[]) {

    vector<Point2f> pts1 = {{142, 18}, {274, 30}, {147, 116}, {276, 113}, {150, 223}, {277, 200},{154, 320}, {276, 280},{163, 352}, {212, 334}, {254, 319}, 
    {293, 304}, {326, 292}, {200, 372}, {248, 353}, {291, 335}, {331, 320}, {361, 305}, {243, 395}, {294, 371}, {339, 351}, {373, 333}, {407, 316}};
    vector<Point2f> pts2 = {{179, 112}, {301, 100}, {179, 193}, {297, 197}, {177, 276}, {293, 300}, {176, 358}, {287, 395},{144, 374}, {175, 385}, 
    {212, 401}, {252, 417}, {300, 437}, {105, 384}, {135, 399}, {171, 415}, {212, 435}, {261, 457}, {59, 396}, {88, 414}, {124, 433}, {164, 454}, {214, 481}};

    assert(pts1.size() == pts2.size());
    
    
    string file1 = "../Picture1.png";
    string file2 = "../Picture2.png";
    Mat img1 = imread(file1);
    Mat img2 = imread(file2);
    if (!img1.data || !img2.data) {
        cerr << "Can't find files !"<< endl;
        return 0;
    }
    for (int i = 0; i < pts1.size(); i++) {
        cv::putText(img1,cv::format("%d", i),pts1[i],cv::FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(0,255,0),1, CV_AA);
        cv::circle(img1, pts1[i], 4, red);

        cv::putText(img2,cv::format("%d", i),pts2[i],cv::FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(0,255,0),1, CV_AA);
        cv::circle(img2, pts2[i], 4, red);
    }
    imwrite("Picture1marked.png", img1);
    imwrite("Picture2marked.png", img2);
    
    
    
    
    Mat F = dlt(pts1, pts2);
//    Mat F = findFundamentalMat(pts1, pts2);
//    debug
    cout <<"initial F is :\n"<< F << endl;
    
    double err = 0.0;
    for (int i = 0; i < pts1.size(); i++) {
        cv::Mat m1 = (cv::Mat_<double>(3,1) <<pts1[i].x, pts1[i].y, 1);
        cv::Mat m2 = (cv::Mat_<double>(1,3) <<pts2[i].x, pts2[i].y, 1);
        cv::Mat res = m2*F*m1;
        err += res.at<double>(0);
    }
    cout << "err is: "<<err << endl;
    
    
    cout << determinant(F) << endl;
//
    Mat e, ep;
    ComputeEpipoles(F, e, ep);
    cout << "epipoles in image 1 is: \n" << e << endl;
    cout << "epipoles in image 2 is: \n" << ep << endl;
    
    Mat P1, P2;
    ComputeCameraMatrices (F, ep , P1, P2);
    vector<Point3f> worldP;// =  triangulate_to_3D(pts1, pts2, P1, P2);
    
    cout << "P1 = \n" << P1 << endl;
    cout << "P2 = \n" << P2 << endl;
    
    cv::Mat F_r, P1_r, P2_r;
    F_r= refine_F(pts1, pts2,P1, P2, P2_r, worldP);
    F_r = F_r/F_r.at<double>(2,2);
    cout << "Refined F is: \n"<< F_r << endl;
    
    err = 0.0;
    for (int i = 0; i < pts1.size(); i++) {
        cv::Mat m1 = (cv::Mat_<double>(3,1) <<pts1[i].x, pts1[i].y, 1);
        cv::Mat m2 = (cv::Mat_<double>(1,3) <<pts2[i].x, pts2[i].y, 1);
        cv::Mat res = m2*F_r*m1;
        err += res.at<double>(0);
    }
    cout << "final err is: "<<err << endl;
    
    
    //output points to files
    ofstream of("3D.txt",ofstream::out);
    of << "[";
    for (int i = 0; i < worldP.size(); i++) {
        of << worldP[i].x <<", "<< worldP[i].y << ", "
        <<worldP[i].z;
        if (i != worldP.size()-1)
            of << ";\n";
    }
    of<<"]"<<endl;

//for selected points    
//    string file1 = "Picture2.png";
//    ofstream of(file1+"s.txt",ofstream::out);
//    vector<Point2f> pts;
//    Mat img = imread(file1);
//    getpoints(file1, &pts);
//    for (int i = 0; i < pts.size(); i++) {
//        cv::putText(img,cv::format("%d", i),pts[i],cv::FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(0,255,0),1, CV_AA);
//        cv::circle(img, pts[i], 4, red);
//        of << pts[i] <<", ";
//    }
//    of.close();
//    string name = "CV";
//    imshow(name, img);
//    Mat img1 = cv::imread(file1);
//    
//    cv::imwrite(file1+"marked.png", img);
//    waitKey();
//    return 0;
}
