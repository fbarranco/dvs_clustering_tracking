/*
 * image_reconst.h
 *
 *  Created on: Nov 3, 2016
 *      Author: fran
 */

#ifndef IMAGE_RECONST_H_
#define IMAGE_RECONST_H_

#include "opencv/cv.h"
#include "opencv/cxcore.h"
#include "opencv/highgui.h"
//#include "opencv2/core/core.hpp"


void dst(double *btest, double *bfinal, int h, int w);
void idst(double *btest, double *bfinal, int h, int w);
void transpose(double *mat, double *mat_t, int h, int w);

void getGradientX(cv::Mat &img, cv::Mat &gx);
void getGradientY(cv::Mat &img, cv::Mat &gy);
void lapx(cv::Mat &gx, cv::Mat &gxx);
void lapy(cv::Mat &gy, cv::Mat &gyy);

void poisson_solver(cv::Mat &img, cv::Mat &gxx, cv::Mat &gyy, cv::Mat &result);
void poisson_solver_jacobi(cv::Mat &img, cv::Mat &gxx, cv::Mat &gyy, cv::Mat &result);


//IPL Image versions (legacy)
void lapx( const IplImage *img, IplImage *gxx);
void lapy( const IplImage *img, IplImage *gyy);
void poisson_solver(const IplImage *img, IplImage *gxx , IplImage *gyy, cv::Mat &result);
#endif /* IMAGE_RECONST_H_ */
