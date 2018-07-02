#include <iostream>
#include <opencv2/opencv.hpp>
#include  "stereo.h"
int main()
{
    //string file = "/home/pjz/left";
vector<string> imagelists;
for(int i=1;i<14;i++)
{
    string left = format("/home/pjz/left1/left%.2d.jpg",i);
    cout<<left<<endl;
    imagelists.push_back(left);
    string right = format("/home/pjz/left1/right%.2d.jpg",i);
    cout<<right<<endl;
    imagelists.push_back(right);
}
    stereocalibration a;
    a.file = "/home/pjz/left1/intrinsics.yml";
    a.file1 = "/home/pjz/left1/extrinsics.yml";
    a.boardsize=Size(9,6);
    a.squareSize=30;
    a.stereoinit(imagelists);

    std::cout << "Hello, World!" << std::endl;
    return 0;
}