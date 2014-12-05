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
  imshow("left image", imgL);
  imshow("right image", imgR);


  //Question 2.1 - Computation of disparity data
  //Compute stereo correspondence -> disparity mat
  StereoSGBM stereo(0, 32, 7, 8*7*7, 32*7*7, 2, 0, 5, 100, 32, true);
  Mat disparity;
  stereo(imgL, imgR, disparity);

  //Convert disparity image to a 8bits image
  Mat disparity_d;
  disparity.convertTo(disparity_d, CV_8U);

  // Display images and wait for a key press
  imshow("disparity", disparity_d);
  
  waitKey();

  //Question 2.2 Road/obstacles segmentation in Cartesian Space
  //Remove pixel from the ground
  float z; //depth
  float Zo = 1.28; //height of the camera
  int vo = 156; //intrinsic camera parameter
  // int alpha = 410; //intrinsic camera parameter
  float b = 0.22; //stereo baseline
  
  //our parameters
  float ground = 0.2; //threshold for the ground
  float max_height = 2.5; //maximum height, points above this limit are removed
  for(int i=0; i<disparity_d.rows ; i++) {
    for(int j=0; j<disparity_d.cols ; j++) {
      //Height of this pixel
      z = Zo - ( ((i-vo)*b) / (disparity_d.at<unsigned char>(i,j)/16.0) );
      
      //We remove every point below the ground (the threshold) and above max_height
      if(z < ground || z >  max_height + ground) {
         disparity_d.at<unsigned char>(i,j) = 0;
      }
    }
  }
  imshow("disparity corrected", disparity_d);
  waitKey();
  
  
  //Question 2.3 - Road/obstacles segmentation in Disparity Space
  //Compute the v-disparity
  Mat v_disparity(disparity_d.cols, 32, CV_8UC1);
  
  for(int i=0; i<disparity_d.rows ; i++) {
    for(int j=0; j<disparity_d.cols ; j++) {
       int d = disparity_d.at<unsigned char>(i,j);
       if(d/16 > 0)
         v_disparity.at<unsigned char>(i,d) += 1;
    }
  }

  cout << "End compute v_disparity" << endl;

  //extraction of the road surface
  
  


  waitKey();







  return 0;
}

