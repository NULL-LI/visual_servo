//OPENCV INCLUDES
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "calibrationbase.h"

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
#include <setting.h>
using namespace std;
using namespace cv;


Mat cameraMatrix;
Mat distCoeffs;
Mat object_point;
Mat robotpose;
Settings  s;
Size imageSize;

//Hand eye matrix
Extrinsic_Parameter E[50],Y[50],C[2450],D[2450];


const string CAMERA_DATA_FILE="camera_data.xml";
const string   ROBOT_POSE_FILENAME="robotpose.xml";
const string  RESULT_FILENAME="result_eye_in_hand.xml";
double calculate_value(Mat qx ,Mat tx,int NumOFCD);
bool initSettings(int WIDTH,int HEIGHT,float SQUARESIZE);
Mat calcBoardCornerPositions(Size boardSize, float squareSize);
Mat pose2transform(Mat t_q);
Mat std_pose2transform(Mat q_t);
int main(int argc, char** argv)
{

    ros::init(argc, argv, "eyeinhand_calib_node");
    ros::start();
        if(argc != 5){
            cerr<<"Error"<<endl;
            cout<<"Usage rosrun Hand_Eye_calibration  eyeinhand_calib_node number_of_images board_x_square_number board_y_square_number square_size"<<endl;
            return -1;
        }
     int nImages=atoi(argv[1]);
     int bWidth=atoi(argv[2]);
     int bHeight=atoi(argv[3]);
     float square_size=atof(argv[4]);

     if(!initSettings(bWidth,bHeight,square_size)) {
          cerr<<"inititalization false "<<endl;
          return -1;
     }


    char image_name[100];
    int  HEmatrix_count=0;

    for (int i=1 ;i<=nImages;i++){

        sprintf(image_name,"frame%d.jpg",i);
        Mat image=imread(image_name);
        Mat transform(4,4,CV_64F);
        if( calculate_extrinsic(image,&transform,object_point,cameraMatrix,distCoeffs,s))
        {


            E[HEmatrix_count].transform=transform;
            E[HEmatrix_count].k=findKFromTrans(E[HEmatrix_count].transform);
            E[HEmatrix_count].t=findTFromTrans(E[HEmatrix_count].transform);


            Mat t_q=robotpose(Range(i-1,i),Range(0,7));
            Y[HEmatrix_count].transform=std_pose2transform(t_q);
//            Y[HEmatrix_count].transform=pose2transform(t_q);  // replace
            cout<<Y[HEmatrix_count].transform<<endl;
            Y[HEmatrix_count].k=findKFromTrans(Y[HEmatrix_count].transform);
            Y[HEmatrix_count].t=findTFromTrans(Y[HEmatrix_count].transform);
            HEmatrix_count++;
        }
      }

    int CDcount=0;
    int resultrows;

    for(int i=0;i<HEmatrix_count;i++)
    {
        for (int j=0;j<HEmatrix_count;j++)
        {
                if((i!=j)&&(i<j))
                {
                C[CDcount].transform=E[i].transform*(E[j].transform.inv());
                C[CDcount].k=findKFromTrans(C[CDcount].transform);
                C[CDcount].t=findTFromTrans(C[CDcount].transform);

                D[CDcount].transform=Y[i].transform.inv()*(Y[j].transform);
                D[CDcount].k=findKFromTrans(D[CDcount].transform);
                D[CDcount].t=findTFromTrans(D[CDcount].transform);

                CDcount++;
                }
        }
    }

    resultrows=CDcount*(CDcount-1)/2;
    cout<<resultrows<<endl;
    Mat newResult(resultrows,8,CV_64F);
    int count=0;
    for(int i=0;i<CDcount;i++)
    {
        for (int j=0;j<CDcount;j++)
        {
                if((i!=j)&&(i<j))
                {
                    Mat R=calculate_R(i,j,C[i], C[j],D[i], D[j]);
                    Mat t=calculate_T(R,i,j,C[i], C[j],D[i], D[j]);
                    Mat quat=dcm2quat(R);
                     newResult.at<double>(count,0)=quat.at<double>(0,0);
                     newResult.at<double>(count,1)=quat.at<double>(0,1);
                     newResult.at<double>(count,2)=quat.at<double>(0,2);
                     newResult.at<double>(count,3)=quat.at<double>(0,3);

                     newResult.at<double>(count,4)=t.at<double>(0,0);
                     newResult.at<double>(count,5)=t.at<double>(1,0);
                     newResult.at<double>(count,6)=t.at<double>(2,0);
                     newResult.at<double>(count,7)=calculate_value(quat,t,CDcount);
                      count++;
            }
        }
    }

    //寻找最好的结果
     Mat bestresult(1,8,CV_64F);
     double bestvalue=1000000000000.0;
     int bestcount;

     for(int i=0;i<resultrows;i++)
     {
         if(newResult.at<double>(i,7)<1000000000000.0)
         {
           if(newResult.at<double>(i,7)<bestvalue)
           {
              bestresult.at<double>(0,0)=newResult.at<double>(i,0);
              bestresult.at<double>(0,1)=newResult.at<double>(i,1);
              bestresult.at<double>(0,2)=newResult.at<double>(i,2);
              bestresult.at<double>(0,3)=newResult.at<double>(i,3);
              bestresult.at<double>(0,4)=newResult.at<double>(i,4);
              bestresult.at<double>(0,5)=newResult.at<double>(i,5);
              bestresult.at<double>(0,6)=newResult.at<double>(i,6);
              bestresult.at<double>(0,7)=newResult.at<double>(i,7);
              bestvalue=newResult.at<double>(i,7);
              bestcount=i;
             }
         }
     }

     //开始创建最好结果
     Mat bestresult_q(1,4,CV_64F);
     Mat bestresult_t(1,3,CV_64F);
     bestresult_q.at<double>(0,0)=bestresult.at<double>(0,0);
     bestresult_q.at<double>(0,1)=bestresult.at<double>(0,1);
     bestresult_q.at<double>(0,2)=bestresult.at<double>(0,2);
     bestresult_q.at<double>(0,3)=bestresult.at<double>(0,3);

     bestresult_t.at<double>(0,0)=bestresult.at<double>(0,4);
     bestresult_t.at<double>(0,1)=bestresult.at<double>(0,5);
     bestresult_t.at<double>(0,2)=bestresult.at<double>(0,6);
     Mat trans_result=buildTransformMatrix_QT(bestresult_q,bestresult_t);      //best  result


     cout<<trans_result<<endl;
     FileStorage fs(RESULT_FILENAME,FileStorage::WRITE);
     fs<<"result"<<trans_result;
     fs.release();
     cout<<" optical result "<<endl;
     ee_link2camera_link(trans_result);
     

}
double calculate_value(Mat qx ,Mat tx,int NumOFCD)
{

//   cout<<"qx"<<endl<<qx<<endl;
//   cout<<"tx"<<endl<<tx<<endl;
  double  sumOfValue=0;


   for(int i=0;i<NumOFCD;i++)
   {

       Mat nc=C[i].k/norm(C[i].k);
       Mat nd=D[i].k/norm(D[i].k);

       Mat  ncq(1,4,CV_64F);
       Mat  ndq(1,4,CV_64F);
       for(int j =0;j<3;j++)
       {
          ncq.at<double>(0,j+1)=nc.at<double>(j,0);
           ndq.at<double>(0,j+1)=nd.at<double>(j,0);
        }
       ncq.at<double>(0,0)=0.0;
       ndq.at<double>(0,0)=0.0;
       Mat tc(1,4,CV_64F);
       Mat td(1,4,CV_64F);

       for(int j =0;j<3;j++)
       {

          tc.at<double>(0,j+1)=C[i].t.at<double>(j,0);
           td.at<double>(0,j+1)=D[i].t.at<double>(j,0);
       }

      tc.at<double>(0,0)=0.0;
      td.at<double>(0,0)=0.0;

      Mat Rd(3,3,CV_64F);
          Rodrigues(D[i].k,Rd);
      Mat temp=(Rd-Mat::eye(3,3,CV_64F))*tx;

      Mat tempt(1,4,CV_64F);
      tempt.at<double>(0,0)=0.0;
      tempt.at<double>(0,1)=temp.at<double>(0,0);
      tempt.at<double>(0,2)=temp.at<double>(1,0);
      tempt.at<double>(0,3)=temp.at<double>(2,0);
       sumOfValue=sumOfValue+squareofQuat(quatMulti(ndq,qx)-quatMulti(qx,ncq));
       sumOfValue=sumOfValue+squareofQuat(quatMulti(qx,tc)-quatMulti(tempt,qx)-quatMulti(td,qx));
   }
    sumOfValue=sumOfValue +1000*(1-squareofQuat(qx))*(1-squareofQuat(qx));
     return sumOfValue;

}
Mat std_pose2transform(Mat q_t)
{
    //mm q
    Mat t(1,3,CV_64F);
    Mat quat(1,4,CV_64F);

    for (int i=0;i<4;i++){
       quat.at<double>(0,i)=q_t.at<double>(0,i);
    }

    for (int i=0;i<3;i++){
       t.at<double>(0,i)=q_t.at<double>(0,i+4);
    }

    Mat transform= buildTransformMatrix_QT(quat,t);
    return transform;

}
Mat pose2transform(Mat t_q)
{
   //m q
   Mat t(1,3,CV_64F);
   Mat quat(1,4,CV_64F);

   for (int i=0;i<3;i++){
      t.at<double>(0,i)=t_q.at<double>(0,i)*1000;
   }
//   for (int i=0;i<4;i++){
//      quat.at<double>(0,i)=t_q.at<double>(0,i+3);
//   }

   quat.at<double>(0,0)=-t_q.at<double>(0,6);
   quat.at<double>(0,1)=t_q.at<double>(0,3);
   quat.at<double>(0,2)=t_q.at<double>(0,4);
   quat.at<double>(0,3)=t_q.at<double>(0,5);


   Mat transform= buildTransformMatrix_QT(quat,t);
   return transform;

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

    //read pose matrix

     FileStorage fspose(ROBOT_POSE_FILENAME,FileStorage::READ);
     fspose["robotpose"]>>robotpose;
     fspose.release();


    if(cameraMatrix.empty()) return false;
    if(distCoeffs.empty()) return false;
    if(robotpose.empty()) return false;

    return true;


}

