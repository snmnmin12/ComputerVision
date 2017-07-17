/*
 * @file distortion.cpp
 * @brief Calibrate camera lens distortion
 * @author Zhixing Li
 */

#include "header.h"
#include "distortion.h"
#include "calibration.h"

#define PI 3.14159265
// My function used to calibrate lens distortion of camera
Mat CalibrateDistortion (Mat img) {
    // Conver image to gray-scale
    Mat gray_img;
    cvtColor(img, gray_img, CV_RGB2GRAY);
    
    //-- Step 1: Find the inner corner positions of the chessboard
    vector<Point2f> corners;
    Size patternSize(7,7);
    //Size patternSize(6,9);
    // squareSize is the length of a square of chessboard in milimeter
    int squareSize = 25;
    // Use function provided by OpenCV to find all corners in chessboard
    bool found = findChessboardCorners(gray_img, patternSize, corners);
    if (found) {
        // refine the corners in subpixel level
        cornerSubPix(gray_img, corners, Size(11,11), Size(-1,-1),
                     TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        //drawChessboardCorners(img, patternSize, Mat(corners), found);
    }
    else {
        cerr << "Fail to find chessboard!"  << endl;
    }
    // Write numbers around found corners which is required in lab manual
    for (int i=0; i<corners.size(); i++) {
        putText(img,format("%d", i),corners[i],FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(0,255,0),1, CV_AA);
        //cout << corners[i].x << " " << corners[i].y << endl;
    }
    //-- Step 2: Find the world coordinates and image coordinates of chessboard corners
    // Object point -> points in real world coordinates
//    vector<vector<Point3f> > object_points;
    // Image point -> points in image pixel coordinates
//    vector<vector<Point2f> > image_points;
    // Assume Z = 0 for the chessboard plane, calculate object points for chessboard corners
    vector<Point3d> obj;
    for (int i=0; i < patternSize.height; i++) {
        for (int j=0; j < patternSize.width; j++) {
            obj.push_back(Point3d((double)j * squareSize, (double)i * squareSize, 0.));
        }
    }
//    object_points.push_back(obj);
//    image_points.push_back(corners);
    
    //-- Step 3: Parse k1 k2 p1 p2 k3 cx cy into par vector as requested by limfit
    int image_height = img.size().height;
    int image_width = img.size().width;
    
    int p_h = patternSize.height;
    int p_w = patternSize.width;
    
    // All horizontal and vertical lines of the chessboard
    //vector<Mat> h_lines;
    //vector<Mat> v_lines;
    // Calculate the initial lines by l = p_start x p_end
    // Hope these initial values could work in fitting, but it does not
    //original_lines (h_lines, v_lines, patternSize, corners);
    // Initial guess of par vector
    int n_par = 7;
    //int n_par = 4;
    double par[n_par];
    // Focal length estimation
    
    //double focal = (corners[1].x)/23.*29.;
    par[0]=-0.;		    // k1
    par[1]=0.;		    // k2
    par[2]=0.;			    // p1
    par[3]=0.;			    // p2
    par[4]=0.;			    // k3
    par[5]=image_width/2.;	    // cx
    par[6]=image_height/2.;	    // cy
    //par[7]=focal/1.;		    // fx
    //par[8]=focal/1.;		    // fy
    // Parse all pairs of keypoints into data as requested by limfit
    // m_dat = fvec size
    //int m_dat = 2*corners.size();
    //int m_dat = 9;
    int m_dat = p_h+p_w;
    
    vector<Point2d> corners2;
    for (const auto& p : corners)
        corners2.push_back(Point2d(p));
    //Initialize pattern data with initial lines and corners
    pattern_struct pattern_data ={corners2, patternSize};
    // Control parameters
    lm_control_struct control = lm_control_float;
    lm_status_struct status;
    //cout << "my m_dat = "<< endl << " " << m_dat << endl << endl;
    //
    // First Lmfit
    lmmin(n_par, par, m_dat, &pattern_data, Evaluate_Distance, &control, &status);
    double Err_0 = 2*status.fnorm;
    double Err = status.fnorm;
    double threshold = 1.e-7;
    // Here we try to make lmfit converge to a small error
    while (abs(Err-Err_0)>threshold) {
        Err_0 = Err;
        lmmin(n_par, par, m_dat, &pattern_data, Evaluate_Distance, &control, &status);
        Err = status.fnorm;
        cout << "Overall error = "<< endl << " " << status.fnorm << endl << endl;
        // If we use hvline method, then we need to update the lines for each fitting
        //HLineFit(pattern_data.h_lines,patternSize,corners,par);
        //VLineFit(pattern_data.v_lines,patternSize,corners,par);
    }
    // Show the lmfit message
    printf( "status after %d function evaluations:\n  %s\n",
           status.nfev, lm_infmsg[status.outcome] );
    // Parse the parameters into distortion matrix
    Mat D_M = Mat::zeros(1,5,CV_32FC1);
    for (int i=0; i<5; i++) {
        D_M.at<float>(0,i)=par[i];
    }
    // Parse the parameters into camera matrix
    Mat C_M = Mat::eye(3,3,CV_32FC1);
    //C_M.at<float>(0,0)=par[7];
    //C_M.at<float>(1,1)=par[8];
    C_M.at<float>(0,2)=par[5];
    C_M.at<float>(1,2)=par[6];
    
    // experiment to test my undistort function and OpenCV's
    // turns out they are the same
    
    
    
    cout << "My Camera Matrix = "<< endl << " " << C_M  << endl;
    cout << "My Distortion Matrix = "<< endl << " " << D_M  << endl;
    return D_M;
}

void Evaluate_Distance(const double *par, int m_dat, const void *data, double *fvec, int *info) {
    pattern_struct *D = (pattern_struct*)data;
    
    // For the lmfit's sake, we need to let it calculate the norm of Sampson's error
    // limfit is not written for OpenCV. It is really hard to use.
    // However, the LM solver in OpenCV is not available to use either.
    
    int p_h = D->patternSize.height;
    int p_w = D->patternSize.width;
    
    // Method 1: minimum distance to global vertical/horizontal lines
    // Initialize vector of lines based on input data
    // Each corner point has two point-to-line distances
    // One is to its horizontal line, the other is to its vertical line
    // Try to min sum(d_to_h^2+d_to_v^2) for all corner points
    /*
     for (int i=0; i< p_h; i++) {
     for (int j=0; j< p_w; j++) {
	    int cursor = j+p_w*i;
	    Mat C = CorrectPoint(D->corners[cursor],par);
	    //float d_to_h = CalculateDistance(C,D->h_lines[i]);
	    float d_to_v = CalculateDistance(C,D->v_lines[j]);
	    //cout << "my d_to_h = "<< endl << " " << d_to_h << endl << endl;
	    //cout << "my d_to_v = "<< endl << " " << d_to_v << endl << endl;
	    //fvec[2*cursor] = d_to_h;
	    //fvec[2*cursor+1] = d_to_v;
	    fvec[cursor] = d_to_v;
     }
     }
     */
    /*
     printf("obtained parameters:\n");
     for (int i=0; i<4; ++i ) {
     printf("  par[%i] = %12g\n", i, par[i]);
     }
     */
    /*
     // Method 2: minimum distance to local vertical/horizontal lines
     // This method uses a triple of points
     // and calculates the distance to line of its middle point
     // Try to min sum(d_h^2+d_v^2) for as many points as possible
     int number_h = 0;
     for (int i=0; i< p_h; i++) {
     for (int j=0; j< p_w-2; j++) {
	    Mat left_point = CorrectPoint(D->corners[j+p_w*i],par);
	    Mat middle_point = CorrectPoint(D->corners[j+p_w*i+1],par);
	    Mat right_point = CorrectPoint(D->corners[j+p_w*i+2],par);
	    Mat line = left_point.cross(right_point);
	    float d = CalculateDistance(middle_point,line);
     
	    fvec[number_h] = d;
	    number_h++;
     }
     }
     int number_v = number_h;
     for (int j=0; j< p_w; j++) {
     for (int i=0; i< p_h-2; i++) {
	    Mat up_point = CorrectPoint(D->corners[j+p_w*i],par);
	    Mat middle_point = CorrectPoint(D->corners[j+p_w*(i+1)],par);
	    Mat down_point = CorrectPoint(D->corners[j+p_w*(i+2)],par);
	    Mat line = up_point.cross(down_point);
	    float d = CalculateDistance(middle_point,line);
     
	    fvec[number_v] = d;
	    number_v++;
     }
     }
     */
    /*
     // Method 3: minimum the angles between local vertical/horizontal line
     // This method also uses a triple of points, which means two line segments
     // and calculates the angle between these two segments
     // Try to min sum(angle_h^2+angle_h_v^2) for as many points as possible
     int number_h = 0;
     for (int i=0; i< p_h; i++) {
     float angle_sum = 0;
     for (int j=0; j< p_w-2; j++) {
	    Mat line_1 = CorrectLine (D->corners[j+p_w*i],D->corners[j+p_w*i+1],par);
	    Mat line_2 = CorrectLine (D->corners[j+p_w*i+1],D->corners[j+p_w*i+2],par);
	    float angle = CalculateAngle(line_1, line_2);
	    angle_sum = angle_sum + angle*angle;
     }
     fvec[number_h] = angle_sum/(p_w-1);
     number_h++;
     
     }
     
     int number_v = number_h;
     for (int j=0; j< p_w; j++) {
     float angle_sum = 0;
     for (int i=0; i< p_h-2; i++) {
	    Mat line_1 = CorrectLine (D->corners[j+p_w*i],D->corners[j+p_w*(i+1)],par);
	    Mat line_2 = CorrectLine (D->corners[j+p_w*(i+1)],D->corners[j+p_w*(i+2)],par);
	    float angle = CalculateAngle(line_1, line_2);
	    angle_sum = angle_sum + angle*angle;
     }
     fvec[number_v] = angle_sum/(p_h-1);
     number_v++;
     
     }
     cout << number_v <<endl;
     */
    
    // Method 4: distortion error of each edge segment
    // This method comes from a paper "Make straight lines straight"
    // For each horizontal line and vertical line, it also calculates a sum of point-to-line distance
    // based on all points on the line or say should be on the line.
    // Then feed it to fvec to find the minimum.
    // As you can see, I even add more lines into it. But it still does not work out.
    
    
    vector<Point2d> line_points;
    int square = 0;
    if (p_w > p_h) {
        square = p_h;
    }
    else {
        square = p_w;
    }
    line_struct line_data={line_points,square};
    int count = 0;
    // Horizontal line segments
    for (int i=0; i< p_h; i++) {
        line_data.points.clear();
        for (int j=0; j< p_w; j++) {
            line_data.points.push_back(D->corners[j+p_w*i]);
        }
        line_data.cursor = count;
        count++;
        line_fit(par,&line_data,fvec);
    }
    
    // Vertical line segments
    for (int j=0; j< p_w; j++) {
        line_data.points.clear();
        for (int i=0; i< p_h; i++) {
            line_data.points.push_back(D->corners[j+p_w*i]);
        }
        line_data.cursor = count;
        count++;
        line_fit(par,&line_data,fvec);
    }
    /*
     for (int i=0; i< square-2; i++) {
     line_data.points.clear();
     for (int j=0; j< square-i; j++) {
	    line_data.points.push_back(D->corners[j+i+p_w*(j)]);
	    //cout << "cursor_1 =" << j+i+p_w*(j) << endl;
     }
     line_data.cursor = count;
     count++;
     line_fit(par,&line_data,fvec);
     }
     
     for (int i=1; i< square-2; i++) {
     line_data.points.clear();
     for (int j=0; j< square-i; j++) {
	    line_data.points.push_back(D->corners[j+p_w*(j+i)]);
	    //cout << "cursor_2 =" << j+p_w*(j+i) << endl;
     }
     line_data.cursor = count;
     count++;
     line_fit(par,&line_data,fvec);
     }
     
     for (int i=0; i< square-2; i++) {
     line_data.points.clear();
     for (int j=0; j< square-i; j++) {
	    line_data.points.push_back(D->corners[square-j-1-i+p_w*(j)]);
     }
     line_data.cursor = count;
     count++;
     line_fit(par,&line_data,fvec);
     }
     
     for (int i=1; i< square-2; i++) {
     line_data.points.clear();
     for (int j=0; j< square-i; j++) {
	    line_data.points.push_back(D->corners[square-j-1+p_w*(j+i)]);
     }
     line_data.cursor = count;
     count++;
     line_fit(par,&line_data,fvec);
     }
     //cout << "count = " << count << endl;
     */
    
}
// Line fit function for method 4
// Calculates the error based on all points on the line
void line_fit(const double *par, const void *line, double *fvec) {
    line_struct *L = (line_struct*)line;
    int cursor = L->cursor;
    int line_size = L->points.size();
    double sum_sqr_x = 0.;
    double sum_sqr_y = 0.;
    double sum_x = 0.;
    double sum_y= 0.;
    double sum_xy = 0.;
    for (int i=0; i< line_size; i++)  {
        Mat C = CorrectPoint(L->points[i],par);
        double x = C.at<float>(0,0);
        double y = C.at<float>(1,0);
        sum_x = sum_x + x;
        sum_y = sum_y + y;
        sum_sqr_x = sum_sqr_x + x*x;
        sum_sqr_y = sum_sqr_y + y*y;
        sum_xy = sum_xy + x*y;
    }
    double a = sum_sqr_x - sum_x * sum_x / line_size;
    double b = sum_xy - sum_x * sum_y / line_size;
    double c = sum_sqr_y - sum_y * sum_y / line_size;
    double alpha = a - c;
    double beta = alpha/(2.*sqrt(pow(alpha,2)+4.*pow(b,2)));
    double sin_phi = sqrt(1./2.-beta);
    double cos_phi = sqrt(1./2.+beta);
    double chi = a*pow(sin_phi,2) - 2.*abs(b)*sin_phi*cos_phi + c*pow(cos_phi,2);
    fvec[cursor] = sqrt(chi);
}
// Calculate original h/v lines for method 1
void original_lines (vector<Mat> &h_lines, vector<Mat> &v_lines, Size patternSize, vector<Point2d> corners) {
    int p_h = patternSize.height;
    int p_w = patternSize.width;
    for (int i=0; i< p_h; i++) {
        Mat start = (Mat_<double>(1,2)<<corners[p_w*i].x, corners[p_w*i].y);
        Mat end = (Mat_<double>(1,2) <<corners[p_w*(i+1)-1].x, corners[p_w*(i+1)-1].y);
        Mat h_line = start.cross(end);
        h_lines.push_back(h_line);
        //cout << "h_line = "<< endl << " " << h_line << endl << endl;
        //cout << "Its slope = "<< endl << " " << h_line.at<float>(0,0)/h_line.at<float>(1,0) << endl << endl;
        
    }
    for (int j=0; j< p_w; j++) {
        Mat start = (Mat_<double>(1,2)<<corners[j].x, corners[j].y);
        Mat end = (Mat_<double>(1,2) << corners[j+p_w*(p_h-1)].x, corners[j+p_w*(p_h-1)].y);
        Mat v_line = start.cross(end);
        v_lines.push_back(v_line);
        //cout << "v_line = "<< endl << " " << j << " " << j+p_w*(p_h-1) << endl << endl;
        //cout << "Its slope = "<< endl << " " << v_line.at<float>(1,0)/v_line.at<float>(0,0) << endl << endl;
    }
}
/*
 // Use lmcurve to fit a horizontal line based on the points
 void HLineFit (vector<Mat> &h_lines, Size patternSize, vector<Point2f> corners, const double *par) {
 int p_h = patternSize.height;
 int p_w = patternSize.width;
 int n_p = 3;
 int m_h = p_w;
 lm_control_struct control = lm_control_double;
 lm_status_struct status;
 for (int i=0; i< p_h; i++) {
	double x[m_h];
	double y[m_h];
	double par_h[3] = {h_lines[i].at<float>(0,0),h_lines[i].at<float>(1,0),h_lines[i].at<float>(2,0)};
	for (int j=0; j< p_w; j++) {
 Mat point = CorrectPoint(corners[j+p_w*i], par);
 x[j] = point.at<float>(0,0);
 y[j] = point.at<float>(1,0);
	}
	lmcurve(n_p, par_h, m_h, x, y, h_line_function, &control, &status);
	for (int k=0; k <n_p; k++) {
 h_lines[i].at<float>(k,0)=par_h[k];
	}
	//cout << "hline error = "<< endl << i << " " << status.fnorm << endl << endl;
 }
 //cout << "my h_line = "<< endl << " " << h_lines.size() << endl << endl;
 
 }
 // Use lmcurve to fit a vertical line based on the points
 void VLineFit (vector<Mat> &v_lines, Size patternSize, vector<Point2f> corners, const double *par) {
 int p_h = patternSize.height;
 int p_w = patternSize.width;
 int n_p = 3;
 int m_v = p_h;
 lm_control_struct control = lm_control_double;
 lm_status_struct status;
 for (int j=0; j< p_w; j++) {
	double x[m_v];
	double y[m_v];
	double par_v[3] = {v_lines[j].at<float>(0,0),v_lines[j].at<float>(1,0),v_lines[j].at<float>(2,0)};
	for (int i=0; i< p_h; i++) {
 Mat point = CorrectPoint(corners[j+p_w*i], par);
 x[i] = point.at<float>(0,0);
 y[i] = point.at<float>(1,0);
	}
	lmcurve( n_p, par_v, m_v, y, x, v_line_function, &control, &status );
	for (int k=0; k <n_p; k++) {
 v_lines[j].at<float>(k,0)=par_v[k];
	}
	//cout << "vline error = "<< endl << j << " " << status.fnorm << endl << endl;
 }
 //cout << "my v_line = "<< endl << " " << v_lines.size() << endl << endl;
 
 }
 */
// h line function for lmcurve's use
double h_line_function (double x, const double *p) {
    return (-(p[0]*x+p[2])/p[1]);
}
// v line function for lmcurve's use
double v_line_function (double y, const double *p) {
    return (-(p[1]*y+p[2])/p[0]);
}

// Calculate point-to-line distance
double CalculateDistance (Mat point, Mat line) {
    double r = sqrt(pow(line.at<float>(0,0),2)+pow(line.at<float>(1,0),2));
    //cout << "my line = "<< endl << " " << line << endl << endl;
    //cout << "my point = "<< endl << " " << point << endl << endl;
    double d = abs(point.dot(line))/r;
    //cout << "my d = "<< endl << " " << d  << endl << endl;
    return d;
}
// Calculate angle between two lines
double CalculateAngle (Mat line_1, Mat line_2) {
    double r = line_1.at<float>(1,0)*line_2.at<float>(1,0) +
    line_1.at<float>(0,0)*line_2.at<float>(0,0);
    double n_1 = sqrt(pow(line_1.at<float>(0,0),2)+pow(line_1.at<float>(1,0),2));
    double n_2 = sqrt(pow(line_2.at<float>(0,0),2)+pow(line_2.at<float>(1,0),2));
    double n = n_1 * n_2;
    double angle = acos (r/n) *180/PI;
    return angle;
}

// Correct the distorted points back to its actuall place
// Function verified √
Mat CorrectPoint (Point2d point, const double *par) {
    //double x = (point.x-par[5])/par[7];
    //double y = (point.y-par[6])/par[8];
    double x = (point.x-par[5]);
    double y = (point.y-par[6]);
    
    double r = pow(x,2)+pow(y,2);
    
    double new_x = x * (1+par[0]*r+par[1]*pow(r,2)+par[4]*pow(r,3))
    +(2.*par[2]*x*y+par[3]*(r+2.*pow(x,2)));
    double new_y = y * (1+par[0]*r+par[1]*pow(r,2)+par[4]*pow(r,3))
    +(2.*par[3]*x*y+par[2]*(r+2.*pow(y,2)));
    Mat p(3,1,CV_32FC1);
    //p.at<float>(0,0)=par[7]*new_x+par[5];
    //p.at<float>(1,0)=par[8]*new_y+par[6];
    p.at<float>(0,0)=new_x+par[5];
    p.at<float>(1,0)=new_y+par[6];
    p.at<float>(2,0)=1;
    
    return p;
}

Mat InvCorrectPoint (Point2d point, const double *par) {
    //double x = (point.x-par[5])/par[7];
    //double y = (point.y-par[6])/par[8];
    double x = (point.x-par[5]);
    double y = (point.y-par[6]);
    
    double r = pow(x,2)+pow(y,2);
    
    double new_x = (x-2.*par[2]*x*y+par[3]*(r+2.*pow(x,2))) / (1+par[0]*r+par[1]*pow(r,2)+par[4]*pow(r,3));
    double new_y = (y-2.*par[3]*x*y+par[2]*(r+2.*pow(y,2))) / (1+par[0]*r+par[1]*pow(r,2)+par[4]*pow(r,3));
    Mat p(3,1,CV_32FC1);
    p.at<double>(0,0)=new_x+par[5];
    p.at<double>(1,0)=new_y+par[6];
    p.at<double>(2,0)=1;
    
    return p;
}


// Find the line calculated based on corrected points
Mat CorrectLine (Point2d p_1, Point2d p_2, const double *par) {
    Mat line_start = CorrectPoint(p_1, par);
    Mat line_end = CorrectPoint(p_2, par);
    Mat line = line_start.cross(line_end);
    
    return line;
}
// Back up the parameters of min error
void ParBackup(const double *par, double *p, int n) {
    for (int i=0; i<n; i++) {
        p[i]=par[i];
    }
}
// My own warping function using image interpolition technique
// Function verified √
Mat MyUndistort(Mat img, const double *par) {
    Point2f point_old, point_new;
    Mat new_img = Mat::zeros(img.size(), img.type());
    int height = img.size().height;
    int width = img.size().width;
    double h, w;
    for (int j=0; j<height; j++) {
        for (int i=0; i<width; i++) {
            point_old.x = i;
            point_old.y = j;
            Mat point_mat = InvCorrectPoint(point_old,par);
            point_new = Point2d(point_mat.at<double>(0,0),point_mat.at<double>(0,1));
            h = point_new.x;
            w = point_new.y;
            
            if (int(h+1) >=height || int(w+1) >= width) {
                new_img.at<uchar>(j,i) = img.at<uchar>(int(w),int(h));
                
            }
            else {
                new_img.at<uchar>(j,i) = (int(w+1)-w)*(int(h+1)-h)*img.at<uchar>(int(w),int(h))+
                (int(w+1)-w)*(h-int(h))*img.at<uchar>(int(w),int(h+1))+
                (w-int(w))*(int(h+1)-h)*img.at<uchar>(int(w+1),int(h))+
                (w-int(w))*(h-int(h))*img.at<uchar>(int(w+1),int(h+1));
            }
        }
        
    }
    
    return new_img;
}

