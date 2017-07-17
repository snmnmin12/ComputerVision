
#include "project2.hpp"
#include "challenge.hpp"

string imagefile1 = "../Picture1.png";
string imagefile2 = "../Picture2.png";
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
    project2_question2(input1,input2);
}
