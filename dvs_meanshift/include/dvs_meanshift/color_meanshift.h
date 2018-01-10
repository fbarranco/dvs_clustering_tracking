/*
 * color_meanshift.h
 *
 *  Created on: Nov 2, 2016
 *      Author: fran
 */

#ifndef COLOR_MEANSHIFT_H_
#define COLOR_MEANSHIFT_H_

#include <ros/ros.h>
#include <opencv2/core/core.hpp>

void colorMeanShiftFilt(double *newPixelsPtr, const double *pixelsPtr, int mPixels, int maxPixelDim, const double *imagePtr, int numRows, int numCols, float spaceDivider, char *kernelFun, int maxIterNum, float tolFun);
//void colorMeanShiftFilt(double *newPixelsPtr, const double *pixelsPtr, int mPixels, int maxPixelDim, const double *imagePtr, int numRows, int numCols, float spaceDivider, char *kernelFun, int maxIterNum, float tolFun, double *newImagePtr);
void meanshiftCluster_Gaussian(cv::Mat dataPts, std::vector<double> *clusterCenterX, std::vector<double> *clusterCenterY, std::vector<double> *clusterCenterZ, std::vector<int> *point2Clusters, double bandwidth);

#endif /* COLOR_MEANSHIFT_H_ */
