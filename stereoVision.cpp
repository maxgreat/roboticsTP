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
  
  
  
  
  
  
  
  
  
/*  
  //Question 2.3 - Road/obstacles segmentation in Disparity Space
  //Compute the v-disparity
  Mat v_disparity(disparity.rows, 32, CV_8UC1, Scalar::all(0));
  cout << "L'image a " << v_disparity.rows << " rows et " << v_disparity.cols << "cols" << endl;
  for(int v=0; v<disparity.rows ; v++) {
    for(int u=0; u<disparity.cols ; u++) {
       int d = disparity.at<unsigned char>(v,u);
       if(d/16.0 > 0){
         v_disparity.at<unsigned char>(v,d/16.0) += 1;
       }
    }  
  }

  imshow("v disparity", v_disparity);
  waitKey();
  //extraction of the road surface
  
  
  //Question 2.3.3 Ransac Algorithm
  //Apply threshold to v-disparity
  Mat v_disp_thres;
  threshold(v_disparity, v_disp_thres, 60., 255, THRESH_TOZERO);
  imshow("v_disp_thres", v_disp_thres);
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
  fitLineRansac(vec,line,1000,1.,7.);
  
  cout << "line " << line << endl;
  
  Mat disparity_d2;
  disparity.convertTo(disparity_d2, CV_8U);

  for(int i=0; i<disparity_d2.rows ; i++) {
    for(int j=0; j<disparity_d2.cols ; j++) {
      //We remove every point below the ground
      sign( (Bx-Ax)*(Y-Ay) - (By-Ay)*(X-Ax) )
      if(sign( (line(2)+line(0))*()) {
         disparity_d2.at<unsigned char>(i,j) = 0;
      }
    }
  }
  
 */ 
  
  
  
  //Question 2.4 - Clustering
  
  //int erosion_elem = 0;
  int erosion_size = 5;
  Mat element = getStructuringElement( MORPH_RECT,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );
  
  Mat disparityFiltered;
  cv::erode(disparity_d, disparityFiltered, element);
  cv::dilate(disparityFiltered, disparityFiltered, element);
  
  imshow("disparity transformed", disparityFiltered);
  waitKey();
  
 /* 
  
  Mat dispSegmented(disparityFiltered.size(), CV_8UC1);;
  int nb_seg = segmentDisparity(disparityFiltered, dispSegmented);
  
  cout << "On remplit les segments" << endl;
  vector<vector<Point> > segments(nb_seg, vector<Point>(0));
  for(int i=0; i < disparityFiltered.rows;i++){
    for(int j=0 ; j < disparityFiltered.cols ; j++){
        segments[dispSegmented.at<unsigned char>(i,j)].push_back(Point(i,j));
    }
  } 
  cout << "fin remplissage segment" << endl;
  cout << "Il y a en tout :" << nb_seg << " segments " << endl;
  vector<vector<Point> > seg;
  for(int i = 0 ; i< nb_seg;i++){
    if(segments[i].size() > 60){
        seg.push_back(segments[i]);
    }
  }
  cout << "il y a en tout :" << seg.size() << "segment" << endl;
  nb_seg = seg.size();
  

  vector<Point> means(nb_seg);
  vector<Vec4i> boundBoxes(nb_seg); //(vmin, vmax), (umin, umax)
  for(int i = 0; i < nb_seg; i++){
     means[i] = Point(0,0);
     boundBoxes[i] = Vec4i(segments[i][0].x,seg[i][0].y, seg[i][1].x, seg[i][1].y);
     
     for(unsigned int k = 0; k < segments[i].size(); k++){
        means[i] += seg[i][0];
        
        boundBoxes[i][0] = std::min(boundBoxes[i][0], seg[i][k].x);
        boundBoxes[i][1] = std::max(boundBoxes[i][1], seg[i][k].x);
        boundBoxes[i][2] = std::min(boundBoxes[i][2], seg[i][k].y);
        boundBoxes[i][3] = std::max(boundBoxes[i][3], seg[i][k].y);
     }
     means[i].x /= seg[i].size();
     means[i].y /= seg[i].size();
  }
  cout << "fin calcul mean et bounding" << endl;
  
  for(int i = 0; i < nb_seg ; i++){
     //disparityFiltered.at<unsigned char>(boundBoxes[i][0], boundBoxes[i][2]) = 255;
     disparityFiltered.at<unsigned char>(means[i].x, means[i].y) = 255;
  }
  imshow("disparity transformed", disparityFiltered);
  waitKey();
*/

  return 0;
}

