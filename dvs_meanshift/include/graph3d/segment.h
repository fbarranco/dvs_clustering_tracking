/*
 * segment.h
 *
 *  Created on: Nov 3, 2016
 *      Author: fran
 */

#ifndef SEGMENT_H_
#define SEGMENT_H_

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>


void imadjust(const cv::Mat & src, cv::Mat & dst);
void im2segment(cv::Mat input, cv::Mat cv_image_output, float sigma, float k, int min_size, int *num_ccs);

#endif /* SEGMENT_H_ */
