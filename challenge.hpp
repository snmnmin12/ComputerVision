//
//  chllenge.cpp
//  ComputerVisionProject1
// @authour  Mingmin Song
//  Created by HJKBD on 2/26/17.
//  Copyright © 2017 HJKBD. All rights reserved.
//
#ifndef challenge_hpp
#define challenge_hpp
#include "util.h"

//convert int to string
string convertint(int N) {
    stringstream c;
    c << N;
    return c.str();
}
void generateRandom(vector<Point2d>&input1, int cols, int rows, int N) {
    random_device rd ;
    for (int i = 0; i < N; i++) {
        mt19937 rng (rd());
        uniform_int_distribution <int> un1(0, cols);
        uniform_int_distribution <int> un2(0, rows);
        int x = un1(rng);
        int y = un2(rng);
        cout << "x = "<<x <<"\t";
        cout << "y = "<<y << endl;
        input1.push_back(Point2d(x,y));
    }
}

// Compute normalization matrix T
Mat normalizeT(vector<Point2d> vec) {
    // Declare variables
    Mat T = Mat::zeros(3, 3, CV_64FC1);
    double meanx = 0;
    double meany = 0;
    double s = 0;
    double sum = 0;
    int i;
    // Find means in x and y directions
    for (i = 0; i < (int) vec.size(); i++) {
        meanx += vec[i].x;
        meany += vec[i].y;
    }
    meanx /= (double) vec.size();
    meany /= (double) vec.size();
    // Find scale so that average distance of x’ to origin is sqrt(2)
    for (i = 0; i < (int) vec.size(); i++) {
        sum += sqrt(pow(vec[i].x - meanx, 2.0) + pow(vec[i].y - meany, 2.0));
    }
    //cout << "sum: " << sum << endl;
    s = (((double) vec.size()) * sqrt(2.0)) / sum;
    //cout << "scale: " << scale << endl;
    // Build normalization matrix T
    T.at<double> (0, 0) = s;
    T.at<double> (1, 1) = s;
    T.at<double> (2, 2) = 1.0;
    T.at<double> (0, 2) = -s * meanx;
    T.at<double> (1, 2) = -s * meany;
    return T;
}
// Apply normalized transformation T
Mat applyT(vector<Point2d> vec, Mat T) {
    // Declare variables
    int j;
    vector<Point3d> vec3d;
    Mat M_vec3d, M_vec3d_p;
    for (j = 0; j < (int) vec.size(); j++) {
        vec3d.push_back(Point3d(vec[j].x, vec[j].y, 1.0));
    }
    M_vec3d = Mat(vec3d, true);
    M_vec3d = M_vec3d.reshape(1, vec.size());
    //cout << "M_vec3d t = " << M_vec3d.t() << endl;
    M_vec3d_p = T * M_vec3d.t();
    //cout << "M_vec3d_p = " << M_vec3d_p << endl;
    // Normalize wrt third column
    M_vec3d_p.row(0) = M_vec3d_p.row(0).mul(1 / M_vec3d_p.row(2));
    M_vec3d_p.row(1) = M_vec3d_p.row(1).mul(1 / M_vec3d_p.row(2));
    return M_vec3d_p;
}
// Compute original homography H = inv(T2)*Htilde*T1
Mat findH(Mat Mvec1, Mat Mvec2) {
    // Declare variables
    int n = Mvec1.cols;
    int idx;
    Mat H_vec, Htilde;
    Mat A;
    Mat b(2 * n, 1, CV_64FC1);
    // We need to minimize ||Ah|| s.t. ||h|| = 1, with A being
    // 2n x 9, b being 8 X 1 and h being9x1
    // Fill A matrix
    for (idx = 0; idx < n; idx++) {
        double v1x = Mvec1.at<double> (0, idx);
        double v1y = Mvec1.at<double> (1, idx);
        double v2x = Mvec2.at<double> (0, idx);
        double v2y = Mvec2.at<double> (1, idx);
        
        Mat temp1  = (Mat_<double>(1,8) << 0, 0, 0, -v1x, -v1y, -1, v2y*v1x, v2y*v1y,v2y);
        Mat temp2  = (Mat_<double>(1,8) << v1x, v1y, 1, 0, 0, 0, -v2x*v1x, -v2x*v1y,-v2x);
        A.push_back(temp1);
        A.push_back(temp2);
        b.at<double> (2 * idx, 0) = -v2y;
        b.at<double> (2 * idx + 1, 0) = v2x;
    }
//    Mat d,u,ut;
//    SVD::compute(A, d, u, ut);
//    H_vec = ut(Range(ut.rows-1,ut.rows),Range::all());
    H_vec = (A.t() * A).inv() * A.t() * b;
    // Add h33 = 1
    H_vec.push_back(1.0);
    //cout << "H_vec = " << H_vec << endl;
    // Reshape H vector into matrix form
    Htilde = H_vec.reshape(0, 3);
//    cout << "A = " << A << endl;
    // Reshape H vector into matrix form
    Htilde = H_vec.reshape(0, 3);
//        cout << "Htilde = " << Htilde << endl;
    return Htilde;
}

//return sigma^2
double findSigma(vector<int>& input1, vector<int>& input2) {
    
    Mat T1 = normalizeT(input1);
    Mat T2 = normalizeT(input2);
    cout << "T1 = " << T1 <<  endl;
    cout << "T2 = " << T2 << endl;
    vector<Point2d> v1, v2;
    for (int i = 0; i < input1.size(); i += 2) {
        v1.push_back(Point2d(input1[i],input1[i+1]));
        v2.push_back(Point2d(input2[i],input2[i+1]));
    }
    
    Mat Mvec1 = applyT(v1, T1);
    Mat Mvec2 = applyT(v2, T2);
    Mat H = findH(Mvec1, Mvec2);
    cout << H << endl;
     H = T2.inv() * H * T1;
    cout << H << endl;
    double sigma = 0.0, dissum = 0.0;
    Mat x(3, 1, CV_64FC1);
    Mat xp(3, 1, CV_64FC1);
    Mat prod;
    vector<double> vecdis;
    for (int i = 0; i < (int) v1.size(); i++) {
        double distemp;
        // Setup x
        x.at<double> (0, 0) = v1[i].x;
        x.at<double> (1, 0) = v1[i].y;
        x.at<double> (2, 0) = 1.0;
        // Setup x’
        xp.at<double> (0, 0) = v2[i].x;
        xp.at<double> (1, 0) = v2[i].y;
        xp.at<double> (2, 0) = 1.0;
        // Compute distance d(Hx, x’)
        prod = H * x;
        prod = prod.mul(1 / prod.at<double> (2, 0));
        distemp = pow((int) (prod.at<double> (0, 0)) - v2[i].x, 2.0)
            + pow((int) (prod.at<double> (1, 0)) - v2[i].y, 2.0);
        prod = H.inv() * xp;
        prod = prod.mul(1 / prod.at<double> (2, 0));
        distemp += pow((int) (prod.at<double> (0, 0)) - v1[i].x, 2.0)
            + pow((int) (prod.at<double> (1, 0)) - v1[i].y, 2.0);
        vecdis.push_back(distemp);
        dissum += distemp;
    }
    dissum /= vecdis.size();
    for (int i = 0; i < vecdis.size(); i++) {
        sigma += pow(vecdis[i] - dissum, 2.0);
    }
    sigma = sigma/(vecdis.size()-1);
    return sigma;
}
// Compute distances between putative correspondences and find number of inliers
vector<Point2d> findNumInliers(vector<Point2d>& vec1, vector<Point2d>& vec2, Mat H, int distThres, double* distStd) {
    // Declare variables
    vector<Point2d> vecInliers;
    vector<double> vecDist;
    Mat prod;
    Mat x(3, 1, CV_64FC1);
    Mat xp(3, 1, CV_64FC1);
    vector<Point2d> v1, v2;
    int i;
    double distTemp, distSum;
    // Split vecSSD into world and image coordinates
    for (i = 0; i < (int) vec1.size(); i += 1) {
        v1.push_back(vec1[i]);
        v2.push_back(vec2[i]);
    }
    /*cout << "vecSSD = " << vecSSD << endl;
         cout << "v1 = " << v1 << endl;
         cout << "v2 = " << v2 << endl;*/
    // Check all correspondences
    distSum = 0.0;
    for (i = 0; i < (int) v1.size(); i++) {
        // Setup x
        x.at<double> (0, 0) = v1[i].x;
        x.at<double> (1, 0) = v1[i].y;
        x.at<double> (2, 0) = 1.0;
        // Setup x’
        xp.at<double> (0, 0) = v2[i].x;
        xp.at<double> (1, 0) = v2[i].y;
        xp.at<double> (2, 0) = 1.0;
        // Compute distance d(Hx, x’)
        prod = H * x;
        prod = prod.mul(1 / prod.at<double> (2, 0));
        distTemp = pow((int) (prod.at<double> (0, 0)) - v2[i].x, 2.0) + pow(
                                                                            (int) (prod.at<double> (1, 0)) - v2[i].y, 2.0);
        //cout << "prod: " << prod << endl;
        //cout << "v2[i]: " << v2[i] << endl;
        //cout << "distTemp: " << distTemp << endl;
        // Compute distance d(Hx, x’)
        prod = H.inv() * xp;
        prod = prod.mul(1 / prod.at<double> (2, 0));
        distTemp += pow((int) (prod.at<double> (0, 0)) - v1[i].x, 2.0) + pow(
                                                                             (int) (prod.at<double> (1, 0)) - v1[i].y, 2.0);
        //cout << "distTemp: " << distTemp << endl;
        // Check inlier threshold
        if (distTemp < distThres) {
            //cout << "i = " << i << endl;
            // Add inlier to filtered vector
            vecInliers.push_back(v1[i]);
            vecInliers.push_back(v2[i]);
            vecDist.push_back(distTemp);
            distSum += distTemp;
            // Add distance to distance vector
            }
    }
    *distStd = 0.0;
    distSum /= vecDist.size();
    for (i = 0; i < (int) vecDist.size(); i++) {
        *distStd += pow(vecDist[i] - distSum, 2.0);
    }
    *distStd /= (double) (vecDist.size() - 1);
    return vecInliers;
}


// Compute best homography using RANSAC algorithm
// Return vector<Point2d> vecInliers to return vector of inliers (just for drawing results)
Mat ransac(double p, double eps, vector<Point2d>&in1,vector<Point2d>&in2, vector<Point2d>&vecInliers,int distThres) {
    assert(in1.size()==in2.size());
    // Declare variables
    vector<Point2d> v1, v2, vec1, vec2;
    Mat T1, T2, H, Htilde, Hfinal;
    Mat Mvec1, Mvec2;
    int  idx, numInliers;
    long N = numeric_limits<int>::max(), Nupdate = 0, M = -1;
    vector<Point2d> vecInlinersCurrent;
    double distStd = 0;
    double distStdCurr;
    // Seed the random number generator
    srand(time(0));
    int count = 6;
    N = ceil(log(1 - p)/(log(1 - (pow((1 - eps), count)))));
    M = count*(1 - eps);
    if (in1.size() > 1000) N = 67422;
     // Iterate until we get the best homography
    while (N > Nupdate) {
        // Empty vector of correspondences
        vector<Point2d> vec1, vec2;
        v1 = in1; v2 = in2;
        // Get four random unique correspondences
        for (int j = 0; j < count; j++) {
            // Get random number
            idx = rand() % v1.size();
            // Get elements at random position
            vec1.push_back(v1[idx]);
            vec2.push_back(v2[idx]);
            // Delete elements at random position
            v1.erase(v1.begin() + idx);
            v2.erase(v2.begin() + idx);
        }
        // Build normalization matrix T such that x’=Tx
        T1 = normalizeT(vec1);
        T2 = normalizeT(vec2);
        //cout << "T1 = " << T1 << endl;
        //cout << "T2 = " << T2 << endl;
        // Apply normalization transformation
        Mvec1 = applyT(vec1, T1);
        Mvec2 = applyT(vec2, T2);
        // Compute homography H = inv(T2)*Htilde*T1
        Mat Htilde = findH(Mvec1, Mvec2);
//        cout << "Htitlede = " << Htilde << endl;
        H = T2.inv() * Htilde * T1;
        //cout << "H = " << H << endl;
        // Compute distances between putative correspondences
        // and find number of inliers
        vecInlinersCurrent = findNumInliers(in1, in2, H, distThres, &distStdCurr);
        // Update homography i f necessary
        numInliers = ((int) vecInlinersCurrent.size()) / 2;
        //cout << "numInliers: " << numInliers << endl;
        if (numInliers > M || (numInliers == M && distStdCurr < distStd)) {
            // Update homography
            M = numInliers;
            Hfinal = H.clone();
//            cout << "Hfinal = " << Hfinal << endl;
            // Update vector of inlier indices
            if (!vecInliers.empty()) {
                vecInliers.clear();
            }
            vecInliers = vecInlinersCurrent;
            distStd = distStdCurr;
            cout << "num inliers = " << vecInliers.size() / 2 << endl;
        }
        Nupdate++;
        // Update
    }
    cout << vecInliers << endl;
    cout << "num inliers = " << vecInliers.size() / 2 << endl;
    // Refine homography using the Levenberg-Marquardt method
    return Hfinal;
}

void challenge(vector<int>& input1, vector<int>& input2, double p = 0.9, int N = 200) {
    vector<Point2d> in,out;
    Mat im1 = imread(imagefile1);
    Mat im2 = imread(imagefile2);
    // find the standard deviation
    double sigma = findSigma(input1, input2);
    int distThres = 3.99*sigma; // RANSAC distance threshold
    
    //generate false corresponding points
    generateRandom(in, im1.cols, im1.rows, N);
    for(int i = 0; i < input1.size()-1;i+=2)
        in.push_back(Point2d(input1[i],input1[i+1]));
    generateRandom(out, im2.cols, im2.rows, N);
    for(int i = 0; i < input2.size()-1;i+=2)
        out.push_back(Point2d(input2[i],input2[i+1]));
    
    double eps = 1- (double)20/(double) (in.size());
    vector<Point2d> inliners;
    //recording time
    double t1, t2;
    t1=clock();
    Mat h = ransac( p, eps, in,out, inliners, distThres);
    t2=clock();
    double t =((double)t2-(double)t1)/CLOCKS_PER_SEC;
    
    cout << "Time counsumed" << t << endl;
    cout << "H = " << h << endl;
    //processing corner points
    //<Point2d> corner = {Point2d(0,0), Point2d(im1.cols-1,0),Point2d(0,im1.rows-1),Point2d(im1.cols-1,im1.rows-1)};
    vector<Point2d> corner = {Point2d(0,0), Point2d(im1.cols-1,0),Point2d(0,im1.rows-1),Point2d(im1.cols-1,im1.rows-1)};
    vector<Point2d> outcorner;
    double minx = INFINITY,miny = INFINITY, maxx =-INFINITY, maxy = -INFINITY ;
    perspectiveTransform(corner, outcorner, h);
    for (int i = 0; i < outcorner.size(); i++){
        minx = min(minx, outcorner[i].x);
        maxx = max(minx, outcorner[i].x);
        miny = min(miny,outcorner[i].y);
        maxy = max(miny,outcorner[i].y);
    }
    minx = min(minx,0.0);
    miny = min(miny,0.0);
    double htr[3][3] = {{1,0,-minx},{0,1,-miny},{0,0,1}};
    //    double htr[3][3] = {{1,0,(double)(image2.cols)},{0,1,0},{0,0,1}};
    Mat htra = Mat(3,3,CV_64FC1, htr);
    cout << htra<<endl;
    //Mat img_out = Mat(image2.rows,image.cols+image2.cols, image2.type());
    Mat img_out = Mat(im2.rows-miny,im2.cols-minx, im2.type());
    warpPerspective(im1, img_out,htra*h, img_out.size());
    Mat part(img_out,Rect(-minx,-miny,im2.cols,im2.rows));
    Mat temp = imread(imagefile2);
    temp.copyTo(part);
    
    string title1 = "test1";
    string title2 = "test2";
    string title3 = "result";
    namedWindow(title1,CV_WINDOW_NORMAL);
    imshow(title1,im1);
    namedWindow(title2,CV_WINDOW_NORMAL);
    imshow(title2,im2);
    namedWindow(title3,CV_WINDOW_NORMAL);
    imshow(title3,img_out);
    waitKey(0);
    imwrite("../../Image_2/challenge_out_"+convertint(N)+"_"+imagefile1, img_out);
}

#endif
