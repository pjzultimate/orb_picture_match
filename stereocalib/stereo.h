//
// Created by pjz on 18-6-27.
//

#ifndef UNTITLED2_STEREO_H
#define UNTITLED2_STEREO_H
#include <opencv2/opencv.hpp>
//#include "opencv2/calib3d.hpp"
//#include "opencv2/imgcodecs.hpp"
//#include "opencv2/highgui.hpp"
//#include "opencv2/imgproc.hpp"
#include <string>
#include <vector>
using namespace cv;
using namespace std;
class stereocalibration {
public:
    void stereoinit(const vector<string>& imagelist);
    string file,file1;
    Size boardsize;
    float squareSize;
    bool useCalibrated=true;
private:
    vector<vector<Point2f>>imagepoints[2];
    vector<vector<Point3f>>objectpoints;
    vector<string>goodimagelist;
    Size imagesize;
    int i,j,k,nimages;
    bool found = false;
    Mat cameraMatrix[2],distCoeffs[2];
    Mat R, T, E, F;
    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];
    Mat rmap[2][2];

};


#endif //UNTITLED2_STEREO_H
