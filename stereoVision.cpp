#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <algorithm>
#include "tp_util.hpp"

using namespace cv;
using namespace std;


class shortPoint{
public:
	shortPoint(){}
	shortPoint(short i, short j){ 
		x = i; 
		y = j;
	}
	short x;
	short y;
};

class shortRect{
public:
	shortRect(){}
	shortRect(short i, short j, short k, short l){ 
		x1 = i; 
		y1 = j;
		x2 = k;
		y2 = l;
	}
	shortRect(shortPoint i, shortPoint j){ 
		x1 = i.x; 
		y1 = i.y;
		x2 = j.x;
		y2 = j.y;
	}
	short x1;
	short y1;
	short x2;
	short y2;
};




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
  for(short i=0; i<disparity.rows ; i++) {
    for(short j=0; j<disparity.cols ; j++) {
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
  
  //Compute the line equation
  
  //For each x coordinate, compute the y coordinate with the equation
  //and remove the point the actual y is below the computed y
  
  

  
  
  
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
  imshow("disparity transformed", disparity_d);
  waitKey();
  
  
  //use segmentDisparity for clustering
  Mat dispSegmented(disparityFiltered.size(), CV_16UC1);
  int nb_seg = segmentDisparity(disparityFiltered, dispSegmented);
  //imshow("dispSegmented", dispSegmented);
  //waitKey();
  
  dispSegmented.convertTo(disparity_d, CV_8U);
  imshow("dispSegmented", disparity_d);
  waitKey();
  
  //store every points in a vector
  vector<vector<shortPoint> > segments;
  for(short i = 0; i < nb_seg; i++){
  	vector<shortPoint> r;
  	segments.push_back(r);
  }
  
  for(short i=0; i < dispSegmented.rows;i++){ 
    for(short j=0 ; j < dispSegmented.cols ; j++){
        segments[dispSegmented.at<short>(i,j)].push_back(shortPoint(i,j));
    }
  } 
  
  
  //remove segments with size < 60 and the background
  vector<vector<shortPoint> > seg;
  for(short i = 1 ; i< nb_seg;i++){
    if(segments[i].size() > 50){
        seg.push_back(segments[i]);
    }
  }
  cout << "It give us :" << seg.size() << " segments " << endl;
  nb_seg = seg.size();
  
  //Compute the disparity means of a segment and the bounding box
  vector<double> means(nb_seg);
  vector<shortRect> boundBoxes(nb_seg); //(vmin, umin, vmax, umax)
  for(short i = 0; i < nb_seg; i++){
     means[i] = 0.0;
     boundBoxes[i] = shortRect(seg[i][0].x, seg[i][0].y, seg[i][0].x,seg[i][0].y);
     for(short k = 0; k < (short)seg[i].size(); k++){
        means[i] += disparityFiltered.at<short>(seg[i][k].x, seg[i][k].y); 
        
        boundBoxes[i].x1 = std::min(boundBoxes[i].x1, seg[i][k].x);
        boundBoxes[i].x2 = std::max(boundBoxes[i].x2, seg[i][k].x);
        boundBoxes[i].y1 = std::min(boundBoxes[i].y1, seg[i][k].y);
        boundBoxes[i].y2 = std::max(boundBoxes[i].y2, seg[i][k].y);
     }
     means[i] /= seg[i].size();
  }
  //Show every bounding box (one by one)
  for(short i = 0; i < nb_seg ; i++){
    
     for(short v = boundBoxes[i].x1; v < boundBoxes[i].x2; v++){
		 disparityFiltered.at<short>(v,boundBoxes[i].y1) = 1023;
		 disparityFiltered.at<short>(v,boundBoxes[i].y2) = 1023;
     }
     for(short u = boundBoxes[i].y1; u < boundBoxes[i].y2; u++){
		 disparityFiltered.at<short>(boundBoxes[i].x1,u) = 1023;
		 disparityFiltered.at<short>(boundBoxes[i].x2,u) = 1023;
     }
     
     Vec2i center = Point((boundBoxes[i].x1 + boundBoxes[i].x2)/2,(boundBoxes[i].y1 + boundBoxes[i].y2) /2);
     disparityFiltered.at<short>(center[0],center[1]) = 255;
     
     
     
  }
  disparityFiltered.convertTo(disparity_d, CV_8U);
  imshow("disparity segmented", disparity_d);
  waitKey();
  //imshow("disparity segmented", disparityFiltered);
  //waitKey();


  return 0;
}

