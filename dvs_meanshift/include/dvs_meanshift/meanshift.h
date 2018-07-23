/*
 * dvs_meanshift.h
 *
 *  Created on: Nov 2, 2016
 *      Author: fran
 */

#ifndef MEANSHIFT_H_
#define MEANSHIFT_H_

#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/photo/photo.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include "graph3d/segment.h"

#define TEST 0
#define KALMAN_FILTERING 1
#define BG_FILTERING 1


#define DVSW 128 //240
#define DVSH 128 //180

//#define DVSW 240
//#define DVSH 180

namespace dvs_meanshift
{

class Meanshift {
public:
  Meanshift(ros::NodeHandle & nh, ros::NodeHandle nh_private);
  virtual ~Meanshift();

private:
  ros::NodeHandle nh_;

  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);
  void eventsCallback_simple(const dvs_msgs::EventArray::ConstPtr& msg);

  //void assignClusterColor(std::vector<int> * positionClusterColor, int numClusters, std::vector<int> matches, std::vector<int> oldPositions);
  void assignClusterColor(std::vector<int> * positionClusterColor, int numClusters, std::vector<int> matches, std::vector<int> oldPositions, std::vector<int> activeTrajectories);
  //void createKalmanFilter();
  void createKalmanFilter(cv::KalmanFilter *kf, cv::Mat *state, cv::Mat *meas);

  int lastPositionClusterColor;

  cv::Mat camera_matrix_, dist_coeffs_;

  ros::Subscriber event_sub_;
  ros::Subscriber camera_info_sub_;

  image_transport::Publisher image_pub_;
  image_transport::Publisher image_segmentation_pub_;


  //DEBUG
  image_transport::Publisher image_debug1_pub_;
  image_transport::Publisher image_debug2_pub_;
  //NOT DEBUG


  cv::Mat last_image_;
  bool used_last_image_;

  enum DisplayMethod
  {
    GRAYSCALE, RED_BLUE
  } display_method_;

  //Meanshift params
  //computing image calibration
  double *outputFeatures;
  int mPixels, maxPixelDim;
  double *pixelsPtr;
  double *imagePtr;
  //double *positionsPtr;
  double *initializationPtr;
  int numRows, numCols, maxIterNum;
  float spaceDivider, timeDivider;
  char kernelFun[8];
  float tolFun;

  //Params for graph3d segmentation
  static const int MAX_NUM_CLUSTERS = 256;
  static const int MAX_NUM_TRAJECTORY_POINTS=16;
  std::vector<cv::Vec3b> RGBColors;
  std::vector<cv::Point> *allTrajectories; //It is an array of vectors of Points for storing trajectories (a maximum of 8 points should be stored)
  std::vector<int> counterTrajectories; //It stores the last position occupied for each trajectory in allTrajectories

  float sigma;
  int k;
  int min_region;
  int num_components;

  //Clusters
  std::vector<double> prev_clustCentX, prev_clustCentY, prev_clustCentZ;
  std::vector<int> prev_positionClusterColor;
  std::vector<int> clustColor;
  std::vector<int> prev_activeTrajectories;

  //Trajectories
  //cv::Mat trajectories=cv::Mat(DVSH, DVSW, CV_8UC3);
  cv::Mat trajectories;

  //Kalman filter parameters
  //cv::KalmanFilter kf;
  //cv::Mat state;
  //cv::Mat meas;

  std::vector<cv::KalmanFilter> vector_of_kf;
  std::vector<cv::Mat> vector_of_state;
  std::vector<cv::Mat> vector_of_meas;

  //bool foundBlobs;
  std::vector<bool> vector_of_foundBlobs;
  int notFoundBlobsCount;
  std::vector<bool> foundTrajectory;
  //bool foundTrajectory;
  int selectedTrajectory;
  //double ticks;
  std::vector<double> vector_of_ticks;

  //frame of times
  cv::Mat BGAFframe;
};

} // namespace

#endif // MEANSHIFT_H_
