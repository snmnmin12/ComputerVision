//
//  main.cpp
//  ComputerVisionProject1
//
//  Created by HJKBD on 2/2/17.
//  Copyright © 2017 HJKBD. All rights reserved.
//

//#include "project1.hpp"
//
////global varaibles for imamge files names;
//string imagefile = "Picture1.png";
//
////original points and destination points for image a
//vector<int> inputpoints1 = {274, 32, 353, 41, 277, 295, 352, 269};
//vector<int> destipoints1 = {275, 30, 370, 30, 275, 300, 370,300};
//
//// four points of image b
//vector<int> inputpoints2 = {81,122,181,112,84,333,177,370};
//vector<int> destipoints2 = {81,122,200,122,81,375,200,375};
//
//int main(int arg, char** args){
//
//    //This is for question 1
////    Mat h1 = findH(inputpoints1, destipoints1);
////    cout <<"h1 = "<< h1 <<endl;
////    fourpointsreticification(imagefile, h1, inputpoints1);
//    
//    // This is for question 2 , four point with two parallel line pair
//    vector<int> parallel1 = inputpoints1;// {186,340,274,310,270,381,356,340};
//    // projective removal function
//    Mat hp = homography_projective(parallel1);
//    cout <<"hp = "<< hp <<endl;
//    removeprojective(imagefile, parallel1);
//    
//    //This is for question 3, remove affin distoration here for image a
//     vector<int> perp1 = {274, 32, 353, 41, 353, 41, 352, 110};
//     vector<int> perp2 = {274 ,32, 352,110, 353, 41, 275, 111};
//    // points for image b
////    vector<int> perp1 = {81,  122, 181, 112, 181, 112, 178, 190};
////    vector<int> perp2 = {81, 122, 178, 190, 181, 112, 81, 187};
//    //removeaffine(imagefile, hp, perp1, perp2);
//    
//    //This is for challenge question
//    //input points with 5 group of orthogonal lines
////    vector<vector<int>> onestepinput =
////                        {{274, 32, 353, 41,  353, 41, 352, 110},
////                         {274 ,32, 352, 110, 353, 41, 275, 111},
////                         {33, 186, 49,  184, 49, 184, 51, 208},
////                         {311,154, 313, 172,313, 172, 321, 170},
////                         {277,204, 276, 295, 276,295,352, 268},
////                        };
//    
//    vector<vector<int>> onestepinput =
//                        {{80,  124, 178, 190, 181, 112, 80, 188},
//                         {80,  124, 181, 112, 181, 112, 178, 190},
////                         {224, 92, 225, 80, 225, 80, 245, 77},
//                         {353,259,351,282,351,282,364,285},
//                         {298, 197, 301, 98,  301, 98,  390, 89},
//                         {129, 350, 178, 370, 178, 370, 179, 281},
//                        };
//    //onestepmethod(imagefile, onestepinput);
//    //This is to select foun point with mouse button, not used for now;
//    //getpoints(imagefile);
//    return 0;
//}
#include "project2.hpp"
#include "challenge.hpp"

string imagefile1 = "Picture1.png";
string imagefile2 = "Picture2.png";
string title = "window";
Scalar green = {0,255, 0};
Scalar red = {0, 0, 255};
//These are for project 2
vector<int> input1 = {536,137,563,131,582,126,609,118,632,112,658,107,540,176,563,171,584,166,609,159,
                        635,155,662,149,567,376,591,376,617,376,643,375,672,377,637,460,616,464,660,480};
vector<int> input2= {7,122,35,122,56,120,82,119,106,116,132,115,5,165,31,163,54,161,79,161,104,159,130,158,
                    14,377,40,375,66,375,91,373,118,374,66,457,44,465,84,476};
Mat image;
vector<Point3d> pts, des;
int cou = 0;

int main(int argc,char** argv) {
   //project2_question1(input1,input2);
    //project2_question2(input1,input2);
    double p = 0.99;
    int N = 200;
    challenge(input1, input2,p,N);
}
