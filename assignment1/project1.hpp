//
//  project1.hpp
//  ComputerVisionProject1
//
//  Created by HJKBD on 2/14/17.
//  Copyright © 2017 HJKBD. All rights reserved.
//

#ifndef project1_hpp
#define project1_hpp

#include <stdio.h>
#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

//Color definition
static Scalar green = {0,255, 0};
static Scalar red = {0, 0, 255};

//find H bsed on the found points selection
Mat findH(vector<int>&, std::vector<int>&);

//find Hp based on the 2 group of parellel lines
Mat homography_projective(vector<int>& paralle1);

//find Ha after removal of projective distortion
Mat homography_affine(Mat Hp, vector<int>& per1, vector<int>& per2);

//find one step H
Mat homography_onestep(vector<vector<int>>& input);

//transform image according to H
void transformimage(Mat image_in, Mat im_out, Mat h);

//combine findH and rectifyimage together
void fourpointsreticification(string&, Mat, vector<int>&);

//bilinear interpolation for no integer coordinate
Vec3b bilinear(Mat img, float x, float y);

//remove projective distortion;
void removeprojective(string&, vector<int>& parallel);

//remove affine distortion
void removeaffine(string&, Mat, vector<int>&per1, vector<int>&per2);


// one step method
void onestepmethod(string& imagefile, vector<vector<int>>& onestepinput);

//get coordinate on the screen by clicking on the mouth
void getpoints(string& imagefile);
void onMouse( int event, int x, int y, int, void* );
//This function is used to calculate the MH=R; so H = M^(-1)*R;
Mat findH(vector<int>& ptr1, vector<int>& ptr2)
{
    
    int dim = 8;
    Mat matrix;
    for (int i = 0; i < ptr1.size();) {
        Mat temp1  = (Mat_<float>(1,8) << ptr1[i], ptr1[i+1], 1, 0, 0, 0,-ptr1[i]*ptr2[i], -ptr1[i+1]*ptr2[i]);
        Mat temp2  = (Mat_<float>(1,8) << 0, 0, 0, ptr1[i], ptr1[i+1], 1, -ptr1[i]*ptr2[i+1], -ptr1[i+1]*ptr2[i+1]);
        i+=2;
        matrix.push_back(temp1);
        matrix.push_back(temp2);
    }
    
    //construct the right handside vector
    Mat right(1, dim, CV_32F);
    for(int i = 0; i < ptr2.size(); i++) {
        right.at<float>(2*i) = ptr2[2*i];
        right.at<float>(2*i+1) = ptr2[2*i+1];
    }
    //calculate the H
    Mat h0 = matrix.inv()*right.t();
    h0.push_back(float(1.0));
    h0 = h0.reshape(1, 3);
    return h0;
}


void transformimage(Mat im_in, Mat im_out, Mat h) {
    
    //first try to make the last element of h to 1
    cout<<"h= "<<h<<endl;
    Mat hi = h.inv();
    int rows = im_in.rows;
    int cols = im_in.cols;
    for (int y = 0; y < rows; y++)
        for (int x = 0; x < cols; x++) {
            Mat point =(Mat_<float>(3,1)<<x, y, 1);
            Mat npoint = hi*point;
            float oldx = npoint.at<float>(0)/npoint.at<float>(2);
            float oldy = npoint.at<float>(1)/npoint.at<float>(2);
            if (oldx >=0 && oldx < cols && oldy >= 0 && oldy < rows) {
                Vec3b color = bilinear(im_in, oldx, oldy);
                im_out.at<Vec3b>(y,x) = color;
            }
        }
}


void fourpointsreticification(string& img1, Mat h, vector<int>& pts_src) {
    
    //give name of the two pop up window
        string title = "test1";
        string title2 = "test2";
    
    Mat image = imread(img1);
    
    if (!image.data) {
        std::cout<<"Image does not exist" <<std::endl;
        return;
    }
    
    //output temp image
    Mat im_out = Mat(image.rows,image.cols,CV_8UC3);
    
    warpPerspective(image, im_out, h, im_out.size());
    //transformimage(image, im_out, h);
    circle(image, Point(pts_src[0],pts_src[1]), 8, red);
    circle(image, Point(pts_src[2],pts_src[3]), 8, red);
    circle(image, Point(pts_src[4],pts_src[5]), 8, red);
    circle(image, Point(pts_src[6],pts_src[7]), 8, red);
    
        namedWindow(title);
        imshow(title, image);
        namedWindow(title2);
        imshow(title2, im_out);
//        imwrite("../../new"+imagefile, im_out);
        waitKey(0);
}

Mat homography_projective(vector<int>& paralle1) {
    /*  v1------v2
     |       |
     |       |
     v3------v4
     */
    assert(paralle1.size() == 8);
    
    Vec3f v1(paralle1[0],paralle1[1], 1);
    Vec3f v2(paralle1[2],paralle1[3], 1);
    Vec3f v3(paralle1[4],paralle1[5], 1);
    Vec3f v4(paralle1[6],paralle1[7], 1);
    
    cout << "v1 = " << v1 <<endl;
    cout << "v2 = " << v2 <<endl;
    cout << "v3 = " << v3 <<endl;
    cout << "v4 = " << v4 <<endl;
    
    Vec3f l1 = v1.cross(v2);
    Vec3f l2 = v3.cross(v4);
    cout << "l1 = "<< l1 <<endl;
    cout << "l2 = "<< l2 <<endl;
    
    Vec3f l3 = v1.cross(v3);
    Vec3f l4 = v2.cross(v4);
    cout << "l3 = "<< l3 <<endl;
    cout << "l3 = "<< l4 <<endl;
    
    Vec3f P = l1.cross(l2);
    Vec3f Q = l3.cross(l4);
    cout << "P = "<< P <<endl;
    cout << "Q = "<< Q <<endl;
    
    Vec3f lineinf = P.cross(Q);
    cout << "Line = "<< lineinf << endl;
    
    lineinf(0) = lineinf(0)/lineinf(2);
    lineinf(1) = lineinf(1)/lineinf(2);
    lineinf(2) = 1.0;
    Mat Hp = Mat::eye(3,3, CV_32F);
    Hp.at<float>(2,0) = lineinf(0);
    Hp.at<float>(2,1) = lineinf(1);
    Hp.at<float>(2,2) = lineinf(2);
    
    return Hp;
}

Mat homography_affine(Mat Hp, vector<int>& per1, vector<int>& per2) {
    
    assert(per1.size()==8);
    assert(per2.size()==8);
    Vec3f l = Vec3f(per1[0],per1[1],1).cross(Vec3f(per1[2],per1[3],1));
    Vec3f m = Vec3f(per1[4],per1[5],1).cross(Vec3f(per1[6],per1[7],1));
    Vec3f o = Vec3f(per2[0],per2[1],1).cross(Vec3f(per2[2],per2[3],1));
    Vec3f n = Vec3f(per2[4],per2[5],1).cross(Vec3f(per2[6],per2[7],1));
    
    //correct the projective lines to affine lines
    Mat lmat = Hp.inv().t()*Mat(l, true);
    Mat mmat = Hp.inv().t()*Mat(m, true);
    Mat omat = Hp.inv().t()*Mat(o, true);
    Mat nmat = Hp.inv().t()*Mat(n, true);
    
    //convert the matrix to 3 dim vector
    for (int i = 0; i < 3; i++) {
        l(i) = lmat.at<float>(i);
        m(i) = mmat.at<float>(i);
        o(i) = omat.at<float>(i);
        n(i) = nmat.at<float>(i);
    }
    
    float mdata[2][2] = {{l(0)*m(0),l(0)*m(1)+l(1)*m(0)},
        {o(0)*n(0),o(0)*n(1)+o(1)*n(0)}};
    float bdata[2] = {-l(1)*m(1), -o(1)*n(1)};
    
    Mat matrix = Mat(2,2,CV_32F, mdata);
    Mat b = Mat(2,1,CV_32F,bdata);
    
    //solve M*S = b for s
    Mat s = matrix.inv()*b;
    cout<< "s = " << s <<endl;
    
    float sdata[2][2] = {{s.at<float>(0),s.at<float>(1)},
        {s.at<float>(1), 1}};
    
    Mat S = Mat(2,2,CV_32F,sdata);
    cout<<"S = "<< S << endl;
    
    //compute SVD of S
    Mat U,D2, D, Ut;
    SVD::compute(S, D2, U, Ut,  0);
    cout<< "U = "<< U <<endl;
    cout<< "Ut = "<< Ut <<endl;
    cout<< "D2 = "<< D2 <<endl;
    pow(D2,0.5, D);
    D = Mat::diag(D);
    
    //build A
    Mat A = U*D*U.inv();
    
    cout<<"A = " << A << endl;
    
    float hdata[3][3] = {{A.at<float>(0,0), A.at<float>(0,1),0},
        {A.at<float>(1,0),A.at<float>(1,1),0},{0,0,1}};
    Mat Ha(3,3,CV_32F,hdata);
    cout<<"Ha = "<<Ha<<endl;
    return Ha.clone();
}

Mat homography_onestep(vector<vector<int>>& input) {
    assert(input.size()== 5);
    
    //build the linear system Mx = 0
    Mat matrix(5,6,CV_32F);
    for (int i = 0; i < input.size(); i++) {
        Vec3f l = Vec3f(input[i][0],input[i][1],1).cross(Vec3f(input[i][2],input[i][3],1));
        Vec3f m = Vec3f(input[i][4],input[i][5],1).cross(Vec3f(input[i][6],input[i][7],1));
        cout <<"l = " << l << endl;
        cout <<"m = " << m << endl;
        matrix.at<float>(i, 0) = l(0)*m(0);
        matrix.at<float>(i, 1) = (l(0)*m(1) + l(1)*m(0))/2;
        matrix.at<float>(i, 2) = l(1)*m(1);
        matrix.at<float>(i, 3) = (l(0)*m(2) + l(2)*m(0))/2;
        matrix.at<float>(i, 4) = (l(1)*m(2) + l(2)*m(1))/2;
        matrix.at<float>(i, 5) = l(2)*m(2);
    }
    
    cout << "Matrx = " << matrix <<endl;
    Mat W,V,Vt;
    SVD::compute(matrix, W, V, Vt, 4);
    cout <<" Vt = " << Vt << endl;
    
    float cdata[3][3] = {{Vt.at<float>(5,0), Vt.at<float>(5,1)/2, Vt.at<float>(5,3)/2},
        {Vt.at<float>(5,1)/2, Vt.at<float>(5,2), Vt.at<float>(5,4)/2},
        {Vt.at<float>(5,3)/2, Vt.at<float>(5,4)/2, Vt.at<float>(5,5)}};
    
    //check if the last term in cdata is positive or not.
    if (cdata[2][2] < 0) {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                cdata[i][j] = -cdata[i][j];
    }
    
    Mat C = Mat(3,3,CV_32F,cdata);
    cout << "C infi = " << C <<endl;
    
    float sdata[2][2] = {{cdata[0][0],cdata[0][1]},{cdata[1][0],cdata[1][1]}};
    Mat S = Mat(2,2,CV_32F,sdata);
    
    float avdata[2][1] = {{cdata[0][2]},{cdata[1][2]}};
    
    Mat U,D2,D,Ut;
    SVD::compute(S, D2, U, Ut,  0);
    pow(D2,0.5, D);
    D = Mat::diag(D);
    //build A
    Mat A = U*D*U.inv();
    
    Mat b = Mat(2,1,CV_32F,&avdata);
    Mat v = A.inv()*b;
    
    cout<<"A = " << A << endl;
    
    Mat h = Mat::zeros(3, 3, CV_32F);
    
    for (int i = 0; i < A.cols; i++)
        for (int j = 0; j < A.rows; j++)
            h.at<float>(i,j) = A.at<float>(i,j);
    h.at<float>(0,2) = 0;
    h.at<float>(2,0) = v.at<float>(0);
    h.at<float>(2,1) = v.at<float>(1);
    h.at<float>(2,2) = float(1);
    
    cout << "H = "<< h <<endl;
    return h;
}

void removeprojective(string& img1, vector<int>& paralle1) {
    
    Mat im_in = imread(img1);
    Mat im_out(im_in.rows, im_in.cols, im_in.type());
    Mat Hp = homography_projective(paralle1);
    
    cout << Hp <<endl;
    
    //transfomraiton
    int ratio = 2;
    Size size(im_out.cols*ratio, im_out.rows*ratio);
    //transformimage(im_in, im_out, Hp);
    warpPerspective(im_in, im_out, Hp, size);
    
    string title = "test1";
    string title2 = "test2";
    
    circle(im_in, Point(paralle1[0],paralle1[1]), 3, red,-1);
    circle(im_in, Point(paralle1[2],paralle1[3]), 3, red,-1);
    circle(im_in, Point(paralle1[4],paralle1[5]), 3, red,-1);
    circle(im_in, Point(paralle1[6],paralle1[7]), 3, red,-1);
    line(im_in, Point(paralle1[0],paralle1[1]), Point(paralle1[2],paralle1[3]), green);
    line(im_in, Point(paralle1[4],paralle1[5]), Point(paralle1[6],paralle1[7]), green);
    //    line(im_in, Point(paralle1[6],paralle1[7]), Point(paralle1[0],paralle1[1]), green);
    line(im_in, Point(paralle1[0],paralle1[1]), Point(paralle1[4],paralle1[5]), green);
    line(im_in, Point(paralle1[2],paralle1[3]), Point(paralle1[6],paralle1[7]), green);
    
//    namedWindow(title);
//    imshow(title, im_in);
       imwrite("projective1.png", im_out);
//    namedWindow(title2);
//    imshow(title2, im_out);
//    waitKey(0);
    
}

void removeaffine(string& img1, Mat Hp, vector<int>&per1, vector<int>&per2) {
    
    Mat im_in = imread(img1);
    
    Mat im_out(im_in.rows, im_in.cols, im_in.type());
    Mat Ha = homography_affine(Hp,  per1, per2);
    cout << "Ha = " << Ha <<endl;
    //    transformimage(im_in, im_out, Hp*Ha.inv());
    int ratio = 1;
    Size size(im_out.cols*ratio, im_out.rows*ratio);
    warpPerspective(im_in, im_out, Hp*Ha.inv(), size);
    string title = "test1";
    string title2 = "test2";
    
    circle(im_in, Point(per1[0],per1[1]), 3, red,-1);
    circle(im_in, Point(per1[2],per1[3]), 3, red,-1);
    circle(im_in, Point(per1[4],per1[5]), 3, red,-1);
    circle(im_in, Point(per1[6],per1[7]), 3, red,-1);
    circle(im_in, Point(per2[0],per2[1]), 3, red,-1);
    circle(im_in, Point(per2[2],per2[3]), 3, red,-1);
    circle(im_in, Point(per2[4],per2[5]), 3, red,-1);
    circle(im_in, Point(per2[6],per2[7]), 3, red,-1);
    line(im_in, Point(per1[0],per1[1]), Point(per1[2],per1[3]), green);
    line(im_in, Point(per1[4],per1[5]), Point(per1[6],per1[7]), green);
    line(im_in, Point(per2[0],per2[1]), Point(per2[2],per2[3]), green);
    line(im_in, Point(per2[4],per2[5]), Point(per2[6],per2[7]), green);
    
    //    namedWindow(title);
    //    imshow(title, im_in);
    //    namedWindow(title2);
    //    imshow(title2, im_out);
    //    waitKey(0);
    //warpPerspective(im_in, im_out, L, im_out.size());
    //    imwrite("../../q2_affine"+imagefile, im_in);
       imwrite("affine1_out.png", im_out);
}


void onestepmethod(string& imagefile, vector<vector<int>>& inputs) {
    
    assert(inputs.size() == 5);
    assert(inputs[0].size() == 8);
    
    Mat im_in = imread(imagefile);
    float ratio = 2;
    Mat im_out(int(im_in.rows*ratio), int(im_in.cols*ratio), im_in.type());
    
    for (int i = 0; i < inputs.size(); i++) {
        line(im_in, Point(inputs[i][0],inputs[i][1]), Point(inputs[i][2],inputs[i][3]), green);
        line(im_in, Point(inputs[i][4],inputs[i][5]), Point(inputs[i][6],inputs[i][7]), green);
    }
    
    //get the one step h
    Mat hp = homography_onestep(inputs);
    //    transformimage(im_in, im_out, hp.inv());
    warpPerspective(im_in, im_out, hp.inv(), im_out.size());
//    imwrite("../../one_step_"+imagefile, im_in);
   imwrite("one_step_out.png", im_out);
//        string title = "test1";
//        string title2 = "test2";
//        namedWindow(title);
//        imshow(title, im_in);
//        namedWindow(title2);
//        imshow(title2, im_out);
//        waitKey(0);
}


Vec3b bilinear(Mat img, float x, float y) {
    Vec3b vec;
    int xlow = floor(x);
    int ylow = floor(y);
    Vec3b lt = img.at<Vec3b>(ylow, xlow);
    Vec3b rt = img.at<Vec3b>(ylow, xlow+1);
    Vec3b lb = img.at<Vec3b>(ylow+1, xlow);
    Vec3b rb = img.at<Vec3b>(ylow+1, xlow+1);
    for (int i = 0; i< 3; i++) {
        vec(i) = rb(i)*(x-xlow)*(y-ylow) +lb(i)*(xlow+1-x)*(y-ylow)
        + rt(i)*(x-xlow)*(ylow+1-y) + lt(i)*(xlow+1-x)*(ylow+1-y);
    }
    return vec;
}


void getpoints(string& imagefile) {
    Mat image = imread(imagefile);
    string title = "window";
    namedWindow(title);
    imshow(title, image);
    setMouseCallback(title, onMouse, 0);
    waitKey(0);
}

void onMouse( int event, int x, int y, int, void* ) {
    if( event != CV_EVENT_LBUTTONDOWN )
        return;
    Point pt = Point(x,y);
    std::cout<<"x="<<pt.x<<"\t y="<<pt.y<<"\n";
}

#endif /* project1_hpp */
