#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <algorithm>
#include "tp_util.hpp"

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


  //Question 2.1 - Computation of disparity data -----------------------------------
  //Compute stereo correspondence -> disparity mat
  StereoSGBM stereo(0, 32, 7, 8*7*7, 32*7*7, 2, 0, 5, 100, 32, true);
  Mat disparity;
  stereo(imgL, imgR, disparity);
  Mat disparity2 = disparity.clone();

  //Convert disparity image to a 8bits image
  Mat disparity_d;
  disparity.convertTo(disparity_d, CV_8U);

  // Display images and wait for a key press
  imshow("disparity", disparity_d);


  //Question 2.2 Road/obstacles segmentation in Cartesian Space --------------------
  //Remove pixel from the ground
  float z; //depth
  float Zo = 1.28; //height of the camera
  int vo = 156; //intrinsic camera parameter
  // int alpha = 410; //intrinsic camera parameter
  float b = 0.22; //stereo baseline
  
  //our parameters
  float ground = 0.2; //threshold for the ground
  float max_height = 2.5; //maximum height, points above this limit are removed
  for(int i=0; i<disparity.rows ; i++) {
    for(int j=0; j<disparity.cols ; j++) {
      //Height of this pixel
      z = Zo - ( ((i-vo)*b) / (disparity.at<short>(i,j)/16.0) );
      
      //We remove every point below the ground (the threshold) and above max_height
      if(z < ground || z >  max_height + ground) {
         disparity.at<short>(i,j) = 0;
      }
    }
  }
  disparity.convertTo(disparity_d, CV_8U);
  imshow("disparity corrected", disparity_d);
  waitKey();
 
  
  //Question 2.3 - Road/obstacles segmentation in Disparity Space -------------
  //Compute the v-disparity
  Mat v_disparity(disparity2.rows, 32, CV_8UC1, Scalar::all(0));
  for(int v=0; v<disparity2.rows ; v++) {
    for(int u=0; u<disparity2.cols ; u++) {
       int d = disparity2.at<unsigned char>(v,u);
       if(d/16.0 > 0){
         v_disparity.at<unsigned char>(v,d/16.0) += 1;
       }
    }  
  }

  //imshow("v disparity", v_disparity);
  //waitKey();
  
  //Ransac Algorithm
  //Apply threshold to v-disparity
  Mat v_disp_thres;
  threshold(v_disparity, v_disp_thres, 60., 255, THRESH_TOZERO);
  //imshow("v_disp_thres", v_disp_thres);
  //Add points into vector
  std::vector<cv::Point2f> vec;
  for(int i=0; i<v_disp_thres.rows ; i++) {
    for(int j=0; j<v_disp_thres.cols ; j++) {
       int d = v_disp_thres.at<unsigned char>(i,j);
       if(d > 0){
         vec.push_back(Point2f(i,j));
       }
    }  
  }
  //FitLineRansac
  cv::Vec4f line;
  fitLineRansac(vec,line);
  
  
  //Compute the line equation
  
  //For each x coordinate, compute the y coordinate with the equation
  //and remove the point the actual y is below the computed y
  for(int i=0; i<disparity2.rows ; i++) {
    for(int j=0; j<disparity2.cols ; j++) {
       int d = disparity2.at<unsigned char>(i,j);
       double y = ((i-line[2])*line[1] + line[0]*line[3]) / line[0]; 
       if(y > d/16.0 + 0.2){
         disparity2.at<unsigned char>(i,j) = 0;
       }
    }  
  }
  disparity2.convertTo(disparity_d, CV_8U);
  imshow("Remove by v_disparity",disparity_d);
  
  
  
  //Question 2.4 - Clustering ----------------------------------------
  
  //dilate and erode the image
  int erosion_size = 10;
  Mat element = getStructuringElement( MORPH_RECT,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );
  
  Mat disparityFiltered;
  cv::erode(disparity, disparityFiltered, element);
  cv::dilate(disparityFiltered, disparityFiltered, element);
  
  disparityFiltered.convertTo(disparity_d, CV_8U);
  imshow("disparity filtered", disparity_d);
  
  
  //use segmentDisparity for clustering
  Mat dispSegmented = Mat::zeros(disparityFiltered.size(), CV_32S);
  int nb_seg = segmentDisparity(disparityFiltered, dispSegmented);
  
  //Show disparity segmented
  dispSegmented.convertTo(disparity_d, CV_8U);
  imshow("disparity Segmented", disparity_d);
  
  //Matrix for cluster representation
  //(vmin, vmax, umin, umax, nb_elem)
  Mat cluster = Mat(nb_seg, 5, CV_32S,Scalar::all(-1));
  for(int i=0; i < dispSegmented.rows; i++){ 
    for(int j=0 ; j < dispSegmented.cols ; j++){
        int seg = dispSegmented.at<int>(i,j);
        if(seg != 0){
        
            cluster.at<int>(seg, 4) += 1;
            if(cluster.at<int>(seg, 0) == -1){
                cluster.at<int>(seg,0) = i;
            } else if(cluster.at<int>(seg,0) > i){
                cluster.at<int>(seg,0) = i;
            }
            if(cluster.at<int>(seg,1) < i){
                cluster.at<int>(seg,1) = i;
            }
            if(cluster.at<int>(seg,2) == -1){
                cluster.at<int>(seg,2) = j;
            } else if(cluster.at<int>(seg,2) > j){
                cluster.at<int>(seg,2) = j;
            }
            if(cluster.at<int>(seg,3) < j){
                cluster.at<int>(seg,3) = j;
            }
        }
    }
  }
  
  for(int i = 0; i < nb_seg ; i++){
     if(cluster.at<int>(i,4) > 50){
         rectangle(disparityFiltered,Rect(cluster.at<int>(i,2),cluster.at<int>(i,0) , cluster.at<int>(i,3) - cluster.at<int>(i,2) , cluster.at<int>(i,1) - cluster.at<int>(i,0)),Scalar(255,0,0),1);
     }
  }
  
  disparityFiltered.convertTo(disparity_d, CV_8U);
  imshow("Bounding Boxes", disparity_d);
  waitKey();
  
  return 0;
}

