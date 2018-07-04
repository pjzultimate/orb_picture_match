#include <pthread.h>
#include <opencv2/opencv.hpp>
#include <sys/time.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>

#include "loitorusbcam.h"
#include "loitorimu.h"
#include <time.h>
using namespace std;
using namespace cv;
double PI= 3.1415926535897932384626433832795;
float fx=461.146008,fy=461.332131,cx=398.192547,cy=266.527371;
void findo(Mat& img,Mat& imgd);


timeval left_stamp,right_stamp;

int main()
{
    cv::Mat img_left;
    cv::Mat img_right;
    img_left.create(cv::Size(752,480),CV_8U);
    img_right.create(cv::Size(752,480),CV_8U);
    img_left.data=new unsigned char[IMG_WIDTH_WVGA*IMG_HEIGHT_WVGA];
    img_right.data=new unsigned char[IMG_WIDTH_WVGA*IMG_HEIGHT_WVGA];
   visensor_load_settings("/home/pjz/Loitor_VISensor_Setups.txt");
   cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(1, cv::Size(5, 5));
  int a= visensor_Start_Cameras();
  if(a==0)
  {
      cout<<"open_cameras_success"<<endl;
  }
  int b=0;
   while(1)
   {
   visensor_get_stereoImg((char *)img_left.data,(char *)img_right.data,left_stamp,right_stamp);


   clahe->apply(img_left, img_left);
   clahe->apply(img_right, img_right);
   int x;
   //cout<<left_stamp.tv_sec<<":"<<left_stamp.tv_usec<<":"<<left_stamp.tv_sec-x<<endl;
   if((left_stamp.tv_sec-x)>-1)
   {
   x=left_stamp.tv_sec;
   imshow("left",img_left);
   //imshow("right",img_right);
   Mat imga,imga1,map1,map2,map11,map12;
   Size imgsize;
   Mat m,d,m1,d1;
   FileStorage fs("/home/pjz/untitled6/intrinsics.yml",FileStorage::READ);
   fs["M1"]>>m;
   fs["D1"]>>d;
   fs["M2"]>>m1;
   fs["D2"]>>d1;
   Mat R,T,R1,P1,R2,P2;
   FileStorage fs1("/home/pjz/untitled6/extrinsics.yml",FileStorage::READ);
   fs1["R"]>>R;
   fs1["T"]>>T;
   int numberofdisparities=80,sadwindowsize=7;
   Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);
//medianBlur(img,img,5);
//GaussianBlur(img_left,img_left,cv::Size(5,5),3,3);
//medianBlur(img2,img2,5);
//GaussianBlur(img_right,img_right,cv::Size(5,5),3,3);
imgsize = img_left.size();
Rect roi1,roi2;
Mat Q;
clock_t start = clock();
stereoRectify(m,d,m1,d1,imgsize,R,T,R1,R2,P1,P2,Q,CALIB_ZERO_DISPARITY,0,imgsize,&roi1,&roi2);
initUndistortRectifyMap(m,d,R1,P1,imgsize,CV_16SC2,map1,map2);
initUndistortRectifyMap(m1,d1,R2,P2,imgsize,CV_16SC2,map11,map12);
remap(img_left,imga,map1,map2,INTER_LINEAR);
remap(img_right,imga1,map11,map12,INTER_LINEAR);
imshow("wo",imga);
numberofdisparities = numberofdisparities > 0 ? numberofdisparities:((imgsize.width/8) + 15) & -16;
sgbm->setPreFilterCap(63);
int sgbmwindow = sadwindowsize > 0 ? sadwindowsize:3;
sgbm->setBlockSize(sgbmwindow);
int cn = imga.channels();
sgbm->setP1(8*cn*sgbmwindow*sgbmwindow);
sgbm->setP2(32*cn*sgbmwindow*sgbmwindow);
sgbm->setMinDisparity(0);
sgbm->setNumDisparities(numberofdisparities);
sgbm->setUniquenessRatio(10);
sgbm->setSpeckleWindowSize(100);//100
sgbm->setSpeckleRange(10);//10
sgbm->setDisp12MaxDiff(1);
sgbm->setMode(StereoSGBM::MODE_SGBM);
sgbm->setMinDisparity(StereoSGBM::MODE_SGBM);
Mat disp,disp8,de;
sgbm->compute(imga,imga1,disp);
disp=disp/16;
disp.convertTo(disp8,CV_8U);
//imshow("disp",disp);
clock_t end= clock();
cout<<"time:"<<(double)(end-start)/CLOCKS_PER_SEC<<endl;
//medianBlur(disp,disp,5);
//filterSpeckles(disp,ma
//blur(disp,disp,Size(5,5));
        reprojectImageTo3D(disp,de,Q);
 //      cout<<de.at<uchar>(100,100)[0]<<endl;
//        cout<<"chuanl:"<<de.channels()<<endl;
        for(int i=0;i<de.cols;i++)
            for(int j=0;j<de.rows;j++)
            {
                if(de.at<Vec3f>(j,i)[2]>0.57&&de.at<Vec3f>(j,i)[2]<2)
                {
                    if(abs(de.at<Vec3f>(j,i)[0])<0.3&&abs(de.at<Vec3f>(j,i)[1])<0.3)
                    {
                        if(de.at<Vec3f>(j,i)[1]>0&&de.at<Vec3f>(j,i)[1]>0.1)
                        {
                            imga.at<uchar>(j,i)=0;
                        }
                        else
                        {
                    double R=sqrt(de.at<Vec3f>(j,i)[0]*de.at<Vec3f>(j,i)[0]+de.at<Vec3f>(j,i)[2]*de.at<Vec3f>(j,i)[2]);
                    double angle=atan2(de.at<Vec3f>(j,i)[2],de.at<Vec3f>(j,i)[0])/PI*180;
                    //cout<<"angle:"<<angle<<endl;
                    //cout<<j<<";"<<i<<":"<<de.at<Vec3f>(j,i)<<endl;
                        }
                    }
                    else
                    {
                       imga.at<uchar>(j,i)=0;
                    }
                }
                else
                {
                    imga.at<uchar>(j,i)=0;
                }
                //cout<<i<<j<<":"<<(int)disp8.at<uchar>(j,i)<<endl;
            }

                             imshow("cdepth",de);
//disp.convertTo(disp8,CV_8U,255);

//cvNormalize(disp,disp8,0,255,CV_MINMAX);
//disp.convertTo(disp8,CV_8U,255/(numberofdisparities*16.));


imshow("left",imga);
//imshow("right",imga1);
//imshow("nom",disp8);
findo(imga,de);
//threshold(disp8,disp8,30,255,4);
imshow("tdisp8",disp8);
   char key=static_cast<char>(cvWaitKey(2));
   if(key==' ')
     break;
   else if(key=='s')
   {
         string fileleft=format("/home/pjz/left1/left%.2d.jpg",b);
         string fileright=format("/home/pjz/left1/right%.2d.jpg",b);
         imwrite(fileleft,img_left);
         imwrite(fileright,img_right);
         b++;
   }

   }

     }
return 0;
}
void findo(Mat &img, Mat &imgd)
{
    Mat dstImg=img.clone();
    threshold(img,img,0,255,THRESH_OTSU);
    imshow("er",img);
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
   findContours(img,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
    vector<Rect> boundRect(contours.size());

for(unsigned int i=0;i<contours.size();i++)
{

   if(contours[i].size()>500)
   {
    boundRect[i] = boundingRect(Mat(contours[i]));
    int x=boundRect[i].x+boundRect[i].width/2;
    int y=boundRect[i].y+boundRect[i].height/2;

    cout<<"x1;y1"<<boundRect[i].x<<";"<<boundRect[i].y<<";"<<imgd.at<Vec3f>(boundRect[i].y+30,boundRect[i].x+30)<<endl;
    circle(dstImg,Point(boundRect[i].x+30,boundRect[i].y+30),5,Scalar(255,0,255),3);
    cout<<"x2;y2"<<boundRect[i].x+boundRect[i].width<<";"<<boundRect[i].y<<";"<<imgd.at<Vec3f>(boundRect[i].y+30,boundRect[i].x+boundRect[i].width-30)<<endl;
    circle(dstImg,Point(boundRect[i].x+boundRect[i].width-30,boundRect[i].y+30),5,Scalar(255,0,255),3);
    cout<<x<<":"<<y<<":"<<imgd.at<Vec3f>(y,x)<<endl;
    circle(dstImg,Point(x,y),5,Scalar(255,0,255),3);
    cout<<"x3;y3"<<boundRect[i].x<<";"<<boundRect[i].y+boundRect[i].height<<";"<<imgd.at<Vec3f>(boundRect[i].y+boundRect[i].height,boundRect[i].x)<<endl;
    circle(dstImg,Point(boundRect[i].x,boundRect[i].y+boundRect[i].height),5,Scalar(255,0,255),3);
    cout<<"x4;y4"<<boundRect[i].x+boundRect[i].width<<";"<<boundRect[i].y+boundRect[i].height<<";"<<imgd.at<Vec3f>(boundRect[i].y+boundRect[i].height,boundRect[i].x+boundRect[i].width)<<endl;
    circle(dstImg,Point(boundRect[i].x+boundRect[i].width,boundRect[i].y+boundRect[i].height),5,Scalar(255,0,255),3);
    rectangle(dstImg, Point(boundRect[i].x, boundRect[i].y), Point(boundRect[i].x + boundRect[i].width, boundRect[i].y + boundRect[i].height), Scalar(255, 255, 255), 2, 8);
   }

}
 imshow("ce2",dstImg);
}
