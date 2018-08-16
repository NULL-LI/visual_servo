//OPENCV INCLUDES
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "calibrationbase.h"
#include <fstream>
//STD C++ INCLUDES
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <sstream>
#include<setting.h>

using namespace std;
using namespace cv;


Mat cameraMatrix;
Mat distCoeffs;
Mat object_point;
Mat robotpose;
Settings  s;
Size imageSize;




const string CAMERA_DATA_FILE="camera_data.xml";



bool initSettings(int WIDTH,int HEIGHT,float SQUARESIZE);
Mat pose2transform(Mat t_q);
Mat std_pose2transform(Mat q_t);
int main(int argc, char** argv)
{

     if(argc!=2)
     { cout<<"numImg"<<endl;
     }

      int nImages=atoi(argv[1]);
     int bWidth=5;
     int bHeight=4;
     float square_size=25;

     if(!initSettings(bWidth,bHeight,square_size)) {
          cerr<<"inititalization false "<<endl;
          return -1;
     }

    char image_extrinsic[] = "image_extrinsic.m";
     ofstream fout( image_extrinsic);
     fout<<"image_extrinsic"<<"=[ ";

    char image_name[100];
    for (int i=1 ;i<=nImages;i++){

        sprintf(image_name,"frame%d.jpg",i);
        Mat image=imread(image_name);
        if(image.empty())
        {
            fout<<"  -1  "<<"  -1  "<<"  -1  "<<"  -1  "<<"  -1  "<<"  -1  "<<"  -1  "<<endl;
            continue;
        }
        Mat transform(4,4,CV_64F);
        if( calculate_extrinsic(image,&transform,object_point,cameraMatrix,distCoeffs,s))
       {  Mat quat = dcm2quat(transform(Range(0,3),Range(0,3)));

         fout<<quat.at<double>(0,0)<<" "<<quat.at<double>(0,1)<<" "<<quat.at<double>(0,2)<<" "<<quat.at<double>(0,3)<<" ";
         fout<<transform.at<double>(0,3)<<" "<<transform.at<double>(1,3)<<" "<<transform.at<double>(2,3)<<" ; "<<endl;
       }
       else
        {
            fout<<"  -1  "<<"  -1  "<<"  -1  "<<"  -1  "<<"  -1  "<<"  -1  "<<"  -1  ; "<<endl;
           }
    }

    cv::waitKey(0);
    cv::waitKey(0);
  }





bool  initSettings(int WIDTH,int HEIGHT,float SQUARESIZE)
{
    s.boardSize.width=WIDTH;
    s.boardSize.height=HEIGHT;
    s.squareSize=SQUARESIZE;
    s.patternToUse=1;
    s.input=" ";
    s.flipVertical=0;
    s.delay=100;
    s.nrFrames=25;
    s.aspectRatio=1;
    s.calibZeroTangentDist=1;
    s.calibFixPrincipalPoint=1;
    s.outputFileName=" ";
    s.bwritePoints=1;
    s.bwriteExtrinsics=1;
    s.showUndistorsed=1;
    s.inputType=Settings::IMAGE_LIST;

    s.flag = 0;
    if(s.calibFixPrincipalPoint) s.flag |= CV_CALIB_FIX_PRINCIPAL_POINT;
  //  cout<<"flag="<<s.flag<<endl;
    if(s.calibZeroTangentDist)   s.flag |= CV_CALIB_ZERO_TANGENT_DIST;
  //    cout<<"flag="<<s.flag<<endl;
    if(s.aspectRatio)            s.flag |= CV_CALIB_FIX_ASPECT_RATIO;
  //  cout<<"flag="<<s.flag<<endl;

   s. calibrationPattern = Settings::CHESSBOARD;
    if (s.calibrationPattern == Settings::NOT_EXISTING)
        {
            cerr << " Inexistent camera calibration mode: " <<s. patternToUse << endl;
            s.goodInput = false;
        }
    s.atImageList = 0;
    int mode = s.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;

    object_point=calcBoardCornerPositions(s.boardSize,s.squareSize);

    FileStorage fs(CAMERA_DATA_FILE,FileStorage::READ);
    fs["camera_matrix"]>>cameraMatrix;
    fs["distortion_coeffs"]>>distCoeffs;
    cout<<"Camera instrinsic matrix"<<endl;
    cout<<cameraMatrix<<endl;
    cout<<"camera distortion coefficients"<<endl;
    cout<<distCoeffs<<endl;
    fs.release();

    if(cameraMatrix.empty()) return false;
    if(distCoeffs.empty()) return false;


    return true;


}

