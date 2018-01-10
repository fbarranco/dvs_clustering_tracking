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

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include "graph3d/segment.h"

#define DVSW 240 //240
#define DVSH 180 //180

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
  float sigma;
  int k;
  int min_region;
  int num_components;
};

} // namespace

#endif // MEANSHIFT_H_
