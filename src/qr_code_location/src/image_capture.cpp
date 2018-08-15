#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
using namespace std;
using namespace cv;
int main(int argc, char** argv)

{
  VideoCapture capture(1);
  namedWindow("hhh", CV_WINDOW_AUTOSIZE);
  int count = 0;
  while (true)
  {
    Mat frame;
    capture >> frame;
    imshow("hhh", frame);
    char key_board = waitKey(10);
    if (key_board == 32)
    {
      count++;
      stringstream image_index;
      image_index << count << ".jpg";
      imwrite(image_index.str(), frame);
      std::cout << "hello worlld" << std::endl;
    }
  }
  return 0;
}
