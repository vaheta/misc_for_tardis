#include "opencv2/features2d/features2d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <signal.h>
#include <fstream>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <dirent.h>
#include <string>

using namespace cv;
using namespace std;


int main(int argc, char* argv[])
{
    int numBoards = atoi(argv[1]);
    int board_w = atoi(argv[2]);
    int board_h = atoi(argv[3]);

    Size board_sz = Size(board_w, board_h);
    int board_n = board_w*board_h;

    vector<vector<Point3f> > object_points;
    vector<vector<Point2f> > image_points_rgb, image_points_ir;
    vector<Point2f> corners_ir, corners_rgb;

    vector<Point3f> obj;
    double a = 27.7;
    for(int y=0; y<board_h; y++) 
    {
        for(int x=0; x<board_w; x++)
            obj.push_back(Point3f(x*a,y*a,0));
    }
    Mat img_rgb, img_ir, gray_ir, gray_rgb;

    int success = 0;

    bool found_rgb = false;
    bool found_ir = false;
    for (int i=1; i<=numBoards; i++){
        int k = 0;
        char fname[1024];
        sprintf(fname, "rgb/500196743142_%d.jpg", i);
        img_rgb = imread (fname, CV_LOAD_IMAGE_COLOR);
        cvtColor(img_rgb, gray_rgb, CV_BGR2GRAY);
        sprintf(fname, "ir/034670144547_%d.jpg", i);
        img_ir = imread (fname, CV_LOAD_IMAGE_COLOR);
        cvtColor(img_ir, gray_ir, CV_BGR2GRAY);

        found_rgb = findChessboardCorners (gray_rgb, board_sz, corners_rgb, CV_CALIB_CB_ADAPTIVE_THRESH);
        found_ir = findChessboardCorners (gray_ir, board_sz, corners_ir, CV_CALIB_CB_ADAPTIVE_THRESH);

        if (found_ir && found_rgb)
        {
            cout << i << " found on both images" << endl;
        }

        if (found_rgb)
        {
            cornerSubPix(gray_rgb, corners_rgb, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 100, 0.01));
            drawChessboardCorners(gray_rgb, board_sz, corners_rgb, found_rgb);
        }

        if (found_ir)
        {
            cornerSubPix(gray_ir, corners_ir, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 100, 0.01));
            drawChessboardCorners(gray_ir, board_sz, corners_ir, found_ir);
        }

        imshow("rgb", gray_rgb);
        imshow("ir", gray_ir);

        //k = waitKey(1);
        if (found_ir && found_rgb)
        {
            k = waitKey();
            cout << k << endl;
        }

        if (k == 1048608)
        {
            image_points_rgb.push_back(corners_rgb);
            object_points.push_back(obj);
            image_points_ir.push_back(corners_ir);
            cout << "added" << endl;
            success++;
        }
    }
    cout << "added " << success << " points in total" << endl;

    destroyAllWindows();

    printf("rgb Starting calibration\n");
    Mat intrinsic_rgb = Mat(3, 3, CV_32FC1);
    Mat distcoeffs_rgb;
    vector<Mat> rvecs_rgb, tvecs_rgb;

    intrinsic_rgb.at<float>(0, 0) = 1;
    intrinsic_rgb.at<float>(1, 1) = 1;
    
    double rep_er_rgb = calibrateCamera(object_points, image_points_rgb, img_rgb.size(), 
        intrinsic_rgb, distcoeffs_rgb, rvecs_rgb, tvecs_rgb);

    FileStorage fs1("rgb.yml", FileStorage::WRITE);
    fs1 << "CM" << intrinsic_rgb;
    fs1 << "D" << distcoeffs_rgb;

    cout << "rgb calibration is done; reprojecton error is = " << rep_er_rgb << endl;

    printf("ir Starting calibration\n");
    Mat intrinsic_ir = Mat(3, 3, CV_32FC1);
    Mat distcoeffs_ir;
    vector<Mat> rvecs_ir, tvecs_ir;

    intrinsic_ir.at<float>(0, 0) = 1;
    intrinsic_ir.at<float>(1, 1) = 1;
    
    double rep_er_ir = calibrateCamera(object_points, image_points_ir, img_ir.size(), 
        intrinsic_ir, distcoeffs_ir, rvecs_ir, tvecs_ir);

    FileStorage fs2("ir.yml", FileStorage::WRITE);
    fs2 << "CM" << intrinsic_ir;
    fs2 << "D" << distcoeffs_ir;

    cout << "ir calibration is done; reprojecton error is = " << rep_er_ir << endl;

    fs1.release();
    fs2.release();

    //Stereo calibration part starts here
    cout << "Starting Stereo Calibration" << endl;

    Mat R, T, E, F;
    double rep_er_stereo = stereoCalibrate (object_points, image_points_rgb, image_points_ir, 
        intrinsic_rgb, distcoeffs_rgb, intrinsic_ir, distcoeffs_ir, Size(0,0), 
        R, T, E, F, TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 100, 1e-6), 
        CV_CALIB_FIX_INTRINSIC | CV_CALIB_ZERO_TANGENT_DIST);
    
    FileStorage fs3("kinect.yml", FileStorage::WRITE);
    fs3 << "R" << R;
    fs3 << "T" << T;
    fs3 << "E" << E;
    fs3 << "F" << F;

    cout << "Stereo Calibration is done; reprojecton error is = " << rep_er_stereo << endl;

    fs3.release();
    fs3.

    return(0);
}
