#include "calibrationbase.h"
#include <ros/ros.h>
#include <iostream>
#include <fstream>
//OPENCV INCLUDES
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{

  ros::init(argc, argv, "xml2txt_node");
  ros::start();


  if(argc!=3)
  {
      cerr<<" args : xml  dataname"<<endl;
      return -1;
  }
//
  const string FILENAME = argv[1];
  const string DATANAME = argv[2];

  FileStorage fs(FILENAME,FileStorage::READ);
  Mat data ;
  fs[DATANAME]>>data;

  char txtfilename[] = "data.m";

  ofstream fout(txtfilename);

  int rows = data.rows;
  int cols = data.cols;

  for (int i = 0; i<rows;i++)
  {
     for (int j = 0; j<cols;j++)
     {
         fout<<data.at<double>(i,j)<<" ";
     }

      fout<<" ;"<<endl;

  }

 fout.close();


}


