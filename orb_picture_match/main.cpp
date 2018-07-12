#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

void OptimizeSeam(Mat& img1, Mat& trans, Mat& dst);

typedef struct
{
    Point2f left_top;
    Point2f left_bottom;
    Point2f right_top;
    Point2f right_bottom;
}four_corners_t;

four_corners_t corners;

void CalcCorners(const Mat& H, const Mat& src)
{
    double v2[] = { 0, 0, 1 };//左上角
    double v1[3];//变换后的坐标值
    Mat V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
    Mat V1 = Mat(3, 1, CV_64FC1, v1);  //列向量

    V1 = H * V2;
    //左上角(0,0,1)
    cout << "V2: " << V2 << endl;
    cout << "V1: " << V1 << endl;
    corners.left_top.x = v1[0] / v1[2];
    corners.left_top.y = v1[1] / v1[2];

    //左下角(0,src.rows,1)
    v2[0] = 0;
    v2[1] = src.rows;
    v2[2] = 1;
    V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
    V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
    V1 = H * V2;
    corners.left_bottom.x = v1[0] / v1[2];
    corners.left_bottom.y = v1[1] / v1[2];

    //右上角(src.cols,0,1)
    v2[0] = src.cols;
    v2[1] = 0;
    v2[2] = 1;
    V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
    V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
    V1 = H * V2;
    corners.right_top.x = v1[0] / v1[2];
    corners.right_top.y = v1[1] / v1[2];

    //右下角(src.cols,src.rows,1)
    v2[0] = src.cols;
    v2[1] = src.rows;
    v2[2] = 1;
    V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
    V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
    V1 = H * V2;
    corners.right_bottom.x = v1[0] / v1[2];
    corners.right_bottom.y = v1[1] / v1[2];

}
int main()
{
    //注意图像输入的顺序，先右后左
    Mat img1 = imread("/home/pjz/1.png");
    Mat img2 = imread("/home/pjz/2.png");
    imshow("test",img1);
    Mat img11,img22,imgmatch;
    cvtColor(img1,img11,CV_RGB2GRAY);
    cvtColor(img2,img22,CV_RGB2GRAY);
    Ptr<ORB> orb =ORB::create(2000);
    Ptr<xfeatures2d::SURF> surf=xfeatures2d::SURF::create(100);
    Ptr<xfeatures2d::SIFT> sift=xfeatures2d::SIFT::create(2000);
    vector<KeyPoint> kp1,kp2;
    Mat descrip1,descrip2;
    orb->detectAndCompute(img11,Mat(),kp1,descrip1);
    orb->detectAndCompute(img22,Mat(),kp2,descrip2);
    //surf->detectAndCompute(img11,Mat(),kp1,descrip1);//surf
    //surf->detectAndCompute(img22,Mat(),kp2,descrip2);//surf
    //sift->detectAndCompute(img11,Mat(),kp1,descrip1);//sift
    //sift->detectAndCompute(img22,Mat(),kp2,descrip2);//sift
    BFMatcher matcher ( NORM_HAMMING );
    vector<DMatch> matches;
    //Ptr<DescriptorMatcher> matcher=DescriptorMatcher::create("BruteForce");//surf
    //Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");//sift
    matcher.match(descrip1,descrip2,matches,Mat());
    //matcher->match(descrip1,descrip2,matches,Mat());//surf
     //matcher->match(descrip1,descrip2,matches,Mat());//sift
    //RANSAC操作：
    //1、根据matches中匹配的结果将两幅图像的特征点对齐，并将特征点的类型转化为float，为接下来做准备
    //2、通过求基础矩阵（F;调用函数findFundanmentalMat）得到特征点中匹配状态ransacstatus;
    //3、根据ransacstatus来剔除误匹配点，跟光流法跟踪特征点类似;
    vector<KeyPoint> m_kp1,m_kp2;
    for(int i=0;i<matches.size();i++)
    {
        m_kp1.push_back(kp1[matches[i].queryIdx]);
        m_kp2.push_back(kp2[matches[i].trainIdx]);
    }
    vector<Point2f> f_kp1,f_kp2;
    for(int i=0;i<matches.size();i++)
    {
        f_kp1.push_back(m_kp1[i].pt);
        f_kp2.push_back(m_kp2[i].pt);
    }
    vector<uchar> ransacstatus;
    Mat F=findFundamentalMat(f_kp1,f_kp2,ransacstatus,FM_RANSAC);
    vector<KeyPoint>mg_kp1,mg_kp2;
    vector<DMatch> good_matches;
    int index=0;
    for(int i=0;i<matches.size();i++)
    {
        if(ransacstatus[i]!=0)
        {
            mg_kp1.push_back(m_kp1[i]);
            mg_kp2.push_back(m_kp2[i]);
            matches[i].queryIdx=index;
            matches[i].trainIdx=index;
            good_matches.push_back(matches[i]);
            index++;
        }
    }
    //RANSAC结束;
            // 求得单应性矩阵H;
    Mat H=findHomography(f_kp1,f_kp2,CV_RANSAC);
    cout<<H<<endl;
    drawMatches(img1,mg_kp1,img2,mg_kp2,good_matches,imgmatch,Scalar::all(-1));
    imshow("match",imgmatch);
    //以上匹配和误匹配剔除结束
    CalcCorners(H,img1);//计算匹配区域
    Mat imagT1,imagT2;
    //进行仿射变换
    warpPerspective(img2,imagT1,H,Size(MAX(corners.right_top.x,corners.right_bottom.x),img2.rows));
    imshow("imgT1",imagT1);
    int dst_width = imagT1.cols;  //取最右点的长度为拼接图的长度
    int dst_height = img2.rows;

    Mat dst(dst_height, dst_width, CV_8UC3);
    dst.setTo(0);

   imagT1.copyTo(dst(Rect(0, 0, imagT1.cols, imagT1.rows)));
   img2.copyTo(dst(Rect(0, 0, img2.cols, img2.rows)));

   imshow("b_dst", dst);

//优化匹配裂缝
    OptimizeSeam(img2, imagT1, dst);


    imshow("dst", dst);
    //imwrite("dst.jpg", dst);
    cvWaitKey();
    return 0;

}
void OptimizeSeam(Mat& img1, Mat& trans, Mat& dst)
{
    int start = MIN(corners.left_top.x, corners.left_bottom.x);//开始位置，即重叠区域的左边界

    double processWidth = img1.cols - start;//重叠区域的宽度
    int rows = dst.rows;
    int cols = img1.cols; //注意，是列数*通道数
    double alpha = 1;//img1中像素的权重
    for (int i = 0; i < rows; i++)
    {
        uchar* p = img1.ptr<uchar>(i);  //获取第i行的首地址
        uchar* t = trans.ptr<uchar>(i);
        uchar* d = dst.ptr<uchar>(i);
        for (int j = start; j < cols; j++)
        {
            //如果遇到图像trans中无像素的黑点，则完全拷贝img1中的数据
            if (t[j * 3] == 0 && t[j * 3 + 1] == 0 && t[j * 3 + 2] == 0)
            {
                alpha = 1;
            }
            else
            {
                //img1中像素的权重，与当前处理点距重叠区域左边界的距离成正比，实验证明，这种方法确实好
                alpha = (processWidth - (j - start)) / processWidth;
            }

            d[j * 3] = p[j * 3] * alpha + t[j * 3] * (1 - alpha);
            d[j * 3 + 1] = p[j * 3 + 1] * alpha + t[j * 3 + 1] * (1 - alpha);
            d[j * 3 + 2] = p[j * 3 + 2] * alpha + t[j * 3 + 2] * (1 - alpha);

        }
    }

}
