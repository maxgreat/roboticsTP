#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
  if(argc != 3) {
    cerr << "Usage : " << argv[0] << " image_left image_right" << endl;
    return 0;
  }
  
  // Open image from input file in grayscale
  Mat imgL = imread(argv[1], 0);
  Mat imgR = imread(argv[2], 0);


  //Compute stereo correspondence
  //StereoSGBM(Minimum possible disparity value,
  //	       int numDisparities, int SADWindowSize,
  //             int P1=0, int P2=0, int disp12MaxDiff=0,
  //             int preFilterCap=0, int uniquenessRatio=0,
  //             int speckleWindowSize=0, int speckleRange=0,
  //             bool fullDP=false);
  StereoSGBM stereo(0, 32, 7, 8*7*7, 32*7*7, 2, 0, 5, 100, 32, true);
  Mat disparity;
  stereo(imgL, imgR, disparity);

  //Convert disparity image to a 8bits image
  Mat display_disparity;
  disparity.convertTo(display_disparity, CV_8U);

  // Display images and wait for a key press
  imshow("left image", imgL);
  imshow("right image", imgR);
  imshow("disparity", display_disparity);
  waitKey();
  return 0;
}

