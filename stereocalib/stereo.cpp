//
// Created by pjz on 18-6-27.
//

#include "stereo.h"
void stereocalibration::stereoinit(const vector<string>& imagelist)
{
    if(!imagelist.empty())
    {
        cout<<"read success"<<endl;
    }
    //const int maxScale = 2;
    nimages=(int)imagelist.size()/2;
    cout<<nimages<<endl;
    imagepoints[0].resize(nimages);
    imagepoints[1].resize(nimages);
    for(i=j=0;i<nimages;i++)
    {
        for(k=0;k<2;k++)
        {
            const string& filename=imagelist[i*2+k];
            cout<<filename<<endl;
            Mat img;
            img = imread(filename,0);
            Mat tempimg;
            img.copyTo(tempimg);
            if(img.empty())
            {
                cout<<"image load error"<<endl;
            }
            if(imagesize == Size())
                imagesize=img.size();

            vector<Point2f>& corners = imagepoints[k][j];
           /* for( int scale = 1; scale <= maxScale; scale++ )
            {
                Mat timg;
                if( scale == 1 )
                    timg = img;
               else
                    resize(img, timg, Size(), scale, scale);
               found = findChessboardCorners(timg, boardsize, corners,
                                              CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
                if( found )
                {
                    if( scale > 1 )
                    {
                        Mat cornersMat(corners);
                        cornersMat *= 1./scale;
                    }
                    break;
                }
            }*/
            found = findChessboardCorners(img,boardsize,corners,CALIB_CB_ADAPTIVE_THRESH|CALIB_CB_NORMALIZE_IMAGE);
            cvtColor(img, tempimg, COLOR_GRAY2BGR);
            drawChessboardCorners(tempimg,boardsize,corners,found);
            imshow("corners",tempimg);
            //cvWaitKey(100);
            cornerSubPix(img,corners,Size(7,7),Size(-1,-1),
                         TermCriteria(TermCriteria::COUNT+TermCriteria::EPS,
                                      30, 0.01));
        }
        if(k==2)
        {
            goodimagelist.push_back(imagelist[i*2]);
            goodimagelist.push_back(imagelist[i*2+1]);
            j++;
        }

    }
    cout<<j<<"pairs have been successfully detected"<<endl;
    nimages=j;
    imagepoints[0].resize(nimages);
    imagepoints[1].resize(nimages);
    objectpoints.resize(nimages);
    for( i=0;i<nimages;i++)
    {
        for(j=0;j<boardsize.height;j++)
            for(k=0;k<boardsize.width;k++)
                objectpoints[i].push_back(Point3f(k*squareSize,j*squareSize,0));
    }
    cout<<"start stereo calibration"<<endl;

    cameraMatrix[0] = initCameraMatrix2D(objectpoints,imagepoints[0],imagesize,0);
    cameraMatrix[1] = initCameraMatrix2D(objectpoints,imagepoints[1],imagesize,0);

    double rms = stereoCalibrate(objectpoints, imagepoints[0], imagepoints[1],
                                 cameraMatrix[0], distCoeffs[0],
                                 cameraMatrix[1], distCoeffs[1],
                                 imagesize, R, T, E, F,
                                 CALIB_FIX_ASPECT_RATIO +
                                 CALIB_ZERO_TANGENT_DIST +
                                 CALIB_USE_INTRINSIC_GUESS +
                                 CALIB_SAME_FOCAL_LENGTH +
                                 CALIB_RATIONAL_MODEL +
                                 CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
                                 TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 100, 1e-5) );
    cout << "done with RMS error=" << rms << endl;
    double err = 0;
    int npoints = 0;
    vector<Vec3f> lines[2];
    for( i = 0; i < nimages; i++ )
    {
        int npt = (int)imagepoints[0][i].size();
        Mat imgpt[2];
        for( k = 0; k < 2; k++ )
        {
            imgpt[k] = Mat(imagepoints[k][i]);
            undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
            computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
        }
        for( j = 0; j < npt; j++ )
        {
            double errij = fabs(imagepoints[0][i][j].x*lines[1][j][0] +
                                imagepoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
                           fabs(imagepoints[1][i][j].x*lines[0][j][0] +
                                imagepoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
    cout << "average epipolar err = " <<  err/npoints << endl;
    FileStorage fs(file, FileStorage::WRITE);
    if( fs.isOpened() )
    {
        fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
           "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
        fs.release();
    }


    stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imagesize, R, T, R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, 0, imagesize, &validRoi[0], &validRoi[1]);

    fs.open(file1, FileStorage::WRITE);
    if( fs.isOpened() )
    {
        fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
        fs.release();
    }
    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imagesize, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imagesize, CV_16SC2, rmap[1][0], rmap[1][1]);
    Mat canvas;
    double sf;
    int w, h;
    if( !isVerticalStereo )
    {
        sf = 600./MAX(imagesize.width, imagesize.height);
        w = cvRound(imagesize.width*sf);
        h = cvRound(imagesize.height*sf);
        canvas.create(h, w*2, CV_8UC3);
    }
    else
    {
        sf = 300./MAX(imagesize.width, imagesize.height);
        w = cvRound(imagesize.width*sf);
        h = cvRound(imagesize.height*sf);
        canvas.create(h*2, w, CV_8UC3);
    }

    for( i = 0; i < nimages; i++ )
    {
        for (k = 0; k < 2; k++) {
            Mat img = imread(goodimagelist[i * 2 + k], 0), rimg, cimg;
            remap(img, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);
            //imshow("re",rimg);
            cvtColor(rimg, cimg, COLOR_GRAY2BGR);
            Mat canvasPart = !isVerticalStereo ? canvas(Rect(w * k, 0, w, h)) : canvas(Rect(0, h * k, w, h));
            resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
            if (useCalibrated) {
                Rect vroi(cvRound(validRoi[k].x * sf), cvRound(validRoi[k].y * sf),
                          cvRound(validRoi[k].width * sf), cvRound(validRoi[k].height * sf));
                rectangle(canvasPart, vroi, Scalar(0, 0, 255), 3, 8);
            }
        }

        if (!isVerticalStereo)
            for (j = 0; j < canvas.rows; j += 16)
                line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
        else
            for (j = 0; j < canvas.cols; j += 16)
                line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
        imshow("rectified", canvas);
        char c = (char) waitKey();
        if (c == 27 || c == 'q' || c == 'Q')
            break;


    }

}
//void stereo::readfile(const string& file, vector<string>& vfile) {}