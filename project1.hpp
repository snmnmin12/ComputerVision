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

#endif /* project1_hpp */
