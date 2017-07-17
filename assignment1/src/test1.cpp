
#include "project1.hpp"

//global varaibles for imamge files names;
string imagefile = "../Picture1.png";

//original points and destination points for image a
vector<int> inputpoints1 = {274, 32, 353, 41, 277, 295, 352, 269};
vector<int> destipoints1 = {275, 30, 370, 30, 275, 300, 370,300};

// four points of image b
vector<int> inputpoints2 = {81,122,181,112,84,333,177,370};
vector<int> destipoints2 = {81,122,200,122,81,375,200,375};

int main(int arg, char** args){

   //This is for question 1
//    Mat h1 = findH(inputpoints1, destipoints1);
//    cout <<"h1 = "<< h1 <<endl;
//    fourpointsreticification(imagefile, h1, inputpoints1);
   
   // This is for question 2 , four point with two parallel line pair
   vector<int> parallel1 = inputpoints1;// {186,340,274,310,270,381,356,340};
   // projective removal function
   Mat hp = homography_projective(parallel1);
   cout <<"hp = "<< hp <<endl;
   removeprojective(imagefile, parallel1);
   
   //This is for question 3, remove affin distoration here for image a
    vector<int> perp1 = {274, 32, 353, 41, 353, 41, 352, 110};
    vector<int> perp2 = {274 ,32, 352,110, 353, 41, 275, 111};
   // points for image b
   // vector<int> perp1 = {81,  122, 181, 112, 181, 112, 178, 190};
   // vector<int> perp2 = {81, 122, 178, 190, 181, 112, 81, 187};
   removeaffine(imagefile, hp, perp1, perp2);
   
   //This is for challenge question
   //input points with 5 group of orthogonal lines
   vector<vector<int>> onestepinput =
                       {{274, 32, 353, 41,  353, 41, 352, 110},
                        {274 ,32, 352, 110, 353, 41, 275, 111},
                        {33, 186, 49,  184, 49, 184, 51, 208},
                        {311,154, 313, 172,313, 172, 321, 170},
                        {277,204, 276, 295, 276,295,352, 268},
                       };
   
//    vector<vector<int>> onestepinput =
//                        {{80,  124, 178, 190, 181, 112, 80, 188},
//                         {80,  124, 181, 112, 181, 112, 178, 190},
// //                         {224, 92, 225, 80, 225, 80, 245, 77},
//                         {353,259,351,282,351,282,364,285},
//                         {298, 197, 301, 98,  301, 98,  390, 89},
//                         {129, 350, 178, 370, 178, 370, 179, 281},
//                        };
   onestepmethod(imagefile, onestepinput);
   //This is to select foun point with mouse button, not used for now;
   //getpoints(imagefile);
   return 0;
}

