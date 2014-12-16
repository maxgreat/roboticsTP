#include <opencv2/opencv.hpp>
#include <string>
#include <opencv2/video/tracking.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>

using namespace cv;
using namespace std;


int main()
{
  //  Read lidar data from a file
  Mat lidar_data;
  string filename("lidarData.xml");
  FileStorage fs(filename, FileStorage::READ);
  fs["lidarData"]>> lidar_data;
  int nb_impacts = lidar_data.cols;
  int nb_frames = lidar_data.rows;

  //  extrinsic parameters of the lidar
  double lidar_pitch_angle = -1.*M_PI/180;
  double lidar_height = 0.47;

  //  parameters of the camera
  double uo = 256;
  double vo = 156;
  double alpha_u = 410;
  double alpha_v = 410;
  double camera_height = 1.28;
  double camera_ty = 1.8;
   
  //  define the parameters of the grid
  float x_min = -10.0;
  float x_max = 10.0;
  float y_min = 0;
  float y_max = 30.0;
  float x_step = 0.2;
  float y_step =  0.2;
  int nb_cells_x = (x_max-x_min)/x_step;
  int nb_cells_y = (y_max-y_min)/y_step;

  char key = 'a';
  int frame_nb = 0;
  
  //Region of Interest (ROI)
  float ROI_xmin = 4.0;
  float ROI_xmax = 7.5;
  float ROI_ymin = 9.0;
  float ROI_ymax = 11.0;
  
  
  //Initialize Kalman filter
  KalmanFilter K(4, 2);
  K.transitionMatrix = *(Mat_<float>(4,4) << 1,0,0.2,0,   0,1,0,0.2,  0,0,1,0,  0,0,0,1);
  K.measurementMatrix = *(Mat_<float>(2,4) << 1,0,0,0,   0,1,0,0);
  //setIdentity(K.measurementMatrix);  
  setIdentity(K.processNoiseCov);
  setIdentity(K.measurementNoiseCov);
  //setIdentity(K.errorCovPost, Scalar::all(.1));
  Mat prediction;
  Mat estimated;
  
  //Store every position
  vector<Mat> LPred;
  LPred.clear();
  vector<Mat> LEst;
  LEst.clear();
  
  
  while (key != 'q' && frame_nb != nb_frames)
  {
    //  Allocation/initialization of the grid
    Mat grid = Mat::zeros(Size(nb_cells_x, nb_cells_y), CV_32F);

    //  Read the stereo image
    ostringstream filename;
    filename<<"img/left_img_"<<frame_nb<<".png";
    Mat left_img = imread(filename.str(), 0);
    Mat left_display_img;
    cvtColor(left_img, left_display_img, CV_GRAY2RGB);

    float x_sum=0;
    float y_sum=0;
    int x_num = 0;
    int y_num = 0;
      
    //  Process all the lidar impacts
    for (int i=0; i<nb_impacts/2; ++i)
    {
      double x=lidar_data.at<double>(frame_nb, 2*i);
      double y=lidar_data.at<double>(frame_nb, 2*i+1);

      //  compute the grid
      if (x>x_min && x<x_max && y>y_min && y<y_max && y>0)
        grid.at<float>((y_max-(y-y_min))/y_step, (x-x_min)/x_step) = 0.3;
	
      //  display lidar impacts on stereo image
      if (x>x_min && x<x_max && y>y_min && y<y_max)
      {
        double z=camera_height -(lidar_height + sqrt(x*x+y*y)*sin(lidar_pitch_angle));
        int u=(int)uo+alpha_u*(x/(y+camera_ty));
        int v=(int)vo+alpha_v*(z/(y+camera_ty));
        if (u>0 && u<left_img.cols && v>0 && v<left_img.rows)
        {
          left_display_img.at<unsigned char>(v, 3*u) = 0;
          left_display_img.at<unsigned char>(v, 3*u+1) = 0;
          left_display_img.at<unsigned char>(v, 3*u+2) = 255;
        }
      }
      
      //detection of points in the ROI
      if (x>ROI_xmin && x<ROI_xmax && y>ROI_ymin && y<ROI_ymax)
      {
        x_sum += x;
        y_sum += y;
        x_num++;
        y_num++;
      }
		
    }
    
    //Draw the ROI
    float y = ROI_ymin;
    for(float x = ROI_xmin; x<ROI_xmax; x+= 0.2){
        grid.at<float>((y_max-(y-y_min))/y_step, (x-x_min)/x_step) = 1.0;
    
        Vec3b c = Vec3b(128,128,0);
        double z=camera_height -(lidar_height + sqrt(x*x+y*y)*sin(lidar_pitch_angle));
    	int u=(int)uo+alpha_u*(x/(y+camera_ty));
    	int v=(int)vo+alpha_v*(z/(y+camera_ty));
        left_display_img.at<Vec3b>(v,u) = c;  
    }
    y = ROI_ymax;
    for(float x = ROI_xmin; x<ROI_xmax; x+= 0.2){
        grid.at<float>((y_max-(y-y_min))/y_step, (x-x_min)/x_step) = 1.0;
        
        Vec3b c = Vec3b(128,128,0);
        double z=camera_height -(lidar_height + sqrt(x*x+y*y)*sin(lidar_pitch_angle));
    	int u=(int)uo+alpha_u*(x/(y+camera_ty));
    	int v=(int)vo+alpha_v*(z/(y+camera_ty));
        left_display_img.at<Vec3b>(v,u) = c;   
    }
    
    
    
    float x_mean = 0.0;
    float y_mean = 0.0;
    
    //if we detected point in the ROI
    if(x_num>0 || y_num >0){

        x_mean = x_sum/x_num;
    	y_mean = y_sum/y_num;

        if(frame_nb == 0){      
            K.statePost.at<float>(0) = x_mean;
            K.statePost.at<float>(1) = y_mean;
            K.statePost.at<float>(2) = 0;
            K.statePost.at<float>(3) = 0;
            prediction = K.predict();
        }
        else{
            Mat_<float> state(2,1);
            state(0) = x_mean;
            state(1) = y_mean;
            
            Mat estimated = K.correct(state);
            cout << "estimated : " << estimated.at<float>(0) << ", " << estimated.at<float>(1) << " - " << estimated.at<float>(2) << ", " << estimated.at<float>(3) << endl;

            //compute position in camera frame of the estimated mean
            float ex = estimated.at<float>(0);
            float ey = estimated.at<float>(1);
            double z=camera_height -(lidar_height + sqrt(ex*ex+ey*ey)*sin(lidar_pitch_angle));
            int u=(int)uo+alpha_u*(ex/(ey+camera_ty));
            int v=(int)vo+alpha_v*(z/(ey+camera_ty));

            //Display the mean
            if (u>0 && u<left_img.cols && v>0 && v<left_img.rows)
            {
              Vec3b c = Vec3b(0,255,0);
              left_display_img.at<Vec3b>(v,u) = c;
              left_display_img.at<Vec3b>(v+1,u) = c;
              left_display_img.at<Vec3b>(v,u+1) = c;
              left_display_img.at<Vec3b>(v+1,u+1) = c;
            }
            grid.at<float>((y_max-(ey-y_min))/y_step, (ex-x_min)/x_step) = 1.0;
            
            prediction = K.predict();
            LEst.push_back(estimated);
        }
        LPred.push_back(prediction);
        cout << "predict : " << prediction.at<float>(0) << ", " << prediction.at<float>(1) << " - " << prediction.at<float>(2) << ", " << prediction.at<float>(3) << endl;
        
        //compute mean position in camera frame
    	double z=camera_height -(lidar_height + sqrt(x_mean*x_mean+y_mean*y_mean)*sin(lidar_pitch_angle));
    	int u=(int)uo+alpha_u*(x_mean/(y_mean+camera_ty));
    	int v=(int)vo+alpha_v*(z/(y_mean+camera_ty));
    	
    	//Display the mean
    	if (u>0 && u<left_img.cols && v>0 && v<left_img.rows)
    	{
    	  Vec3b c = Vec3b(255,51,51);
    	  left_display_img.at<Vec3b>(v,u) = c;
    	  left_display_img.at<Vec3b>(v+1,u) = c;
    	  left_display_img.at<Vec3b>(v,u+1) = c;
    	  left_display_img.at<Vec3b>(v+1,u+1) = c;
    	}
    	cout<< "Computed mean: " << x_mean << " , " << y_mean << endl;
        grid.at<float>((y_max-(y_mean-y_min))/y_step, (x_mean-x_min)/x_step) = 1.0;
        
        
        //Move the ROI
        //compute the displacement
        ROI_xmin = LPred.back().at<float>(0) - 1.75;
        ROI_xmax = LPred.back().at<float>(0) + 1.75;
        ROI_ymin = LPred.back().at<float>(1) - 1;
        ROI_ymax = LPred.back().at<float>(1) + 1;
    
    } else {
        cerr << "!! no point in the ROI !!" << endl;
    }
    
    
    //   prepare the display of the grid
    Mat display_grid; //  to have a RGB grid for display
    grid.convertTo(display_grid, CV_8U, 255);
    cvtColor(display_grid, display_grid, CV_GRAY2RGB);
    
    Mat display_grid_large;// to have a large grid for display
    resize(display_grid, display_grid_large, Size(600,600));
	
    //Rect crop_left_image(10,10,200,200);	

    //  show images
    imshow("top view",  display_grid_large);
    imshow("left image", left_display_img);

    //  Wait for the user to press a key
    frame_nb++;
    key = waitKey( );
  }
  return 0;
}


