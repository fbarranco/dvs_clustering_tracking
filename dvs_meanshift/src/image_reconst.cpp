/*
 * image_reconst.cpp
 *
 *  Created on: Nov 3, 2016
 *      Author: fran
 */

/*
#########################   Poisson Image Editing ############################

Copyright (C) 2012 Siddharth Kherada
Copyright (C) 2006-2012 Natural User Interface Group

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
Lesser General Public License for more details. "

##############################################################################
*/

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <complex>
#include "image_reconstruct/image_reconst.h"
#include <math.h>
#include <omp.h>

//using namespace std;
//using namespace cv;

#define pi 3.1416

#define ATF at<float>
#define AT3F at<Vec3f>
#define ATB at<uchar>
#define AT3B at<Vec3b>

/////////////////////Poisson Equation Solver by FFT/////////////////////
void dst(double *btest, double *bfinal, int h, int w)
{

	unsigned long int idx;

	cv::Mat temp = cv::Mat(2 * h + 2, 1, CV_32F);
	cv::Mat res = cv::Mat(h, 1, CV_32F);

	int p = 0;

	for (int i = 0; i<w; i++)
	{
		temp.ATF(0, 0) = 0.0;

		for (int j = 0, r = 1; j<h; j++, r++)
		{
			idx = j*w + i;
			temp.ATF(r, 0) = (float)btest[idx];
		}

		temp.ATF(h + 1, 0) = 0.0;

		for (int j = h - 1, r = h + 2; j >= 0; j--, r++)
		{
			idx = j*w + i;
			temp.ATF(r, 0) = (float)(-1 * btest[idx]);
		}

		cv::Mat planes[] = { cv::Mat_<float>(temp), cv::Mat::zeros(temp.size(), CV_32F) };

		cv::Mat complex1;
		merge(planes, 2, complex1);

		dft(complex1, complex1, 0, 0);

		cv::Mat planes1[] = { cv::Mat::zeros(complex1.size(), CV_32F), cv::Mat::zeros(complex1.size(), CV_32F) };

		split(complex1, planes1);

		std::complex<double> two_i = std::sqrt(std::complex<double>(-1));

		double fac = -2 * imag(two_i);

		for (int c = 1, z = 0; c<h + 1; c++, z++)
		{
			res.ATF(z, 0) = (float)(planes1[1].ATF(c, 0) / fac);
		}

		for (int q = 0, z = 0; q<h; q++, z++)
		{
			idx = q*w + p;
			bfinal[idx] = res.ATF(z, 0);
		}
		p++;
	}

}

void idst(double *btest, double *bfinal, int h, int w)
{
	int nn = h + 1;
	dst(btest, bfinal, h, w);
	#pragma omp parallel for
	for (int i = 0; i<h; i++)
	for (int j = 0; j<w; j++)
	{
		unsigned long int idx = i*w + j;
		bfinal[idx] = (double)(2 * bfinal[idx]) / nn;
	}

}

void transpose(double *mat, double *mat_t, int h, int w)
{

	cv::Mat tmp = cv::Mat(h, w, CV_32FC1);
	int p = 0;

	#pragma omp parallel for
	for (int i = 0; i < h; i++)
	{
		for (int j = 0; j < w; j++)
		{

			unsigned long int idx = i*(w)+j;
			tmp.ATF(i, j) = (float)(mat[idx]);
		}
	}
	cv::Mat tmp_t = tmp.t();

	unsigned long int idx;
	for (int i = 0; i < tmp_t.size().height; i++)
	for (int j = 0; j<tmp_t.size().width; j++)
	{
		idx = i*tmp_t.size().width + j;
		mat_t[idx] = tmp_t.ATF(i, j);
	}

}


void getGradientX(cv::Mat &img, cv::Mat &gx)
{
	int height = img.rows;
	int width = img.cols;
	cv::Mat cat = repeat(img, 1, 2);
	/*cat.col(width) = 0;*/
	cv::Mat roimat = cat(cv::Rect(1, 0, width, height));
	gx = roimat - img;
	gx.col(width - 1) = 0;
}

// calculate vertical gradient, gy(i, j) = img(i, j+1) - img(i, j)
void getGradientY(cv::Mat &img, cv::Mat &gy)
{
	int height = img.rows;
	int width = img.cols;
	cv::Mat cat = repeat(img, 2, 1);
	/*cat.row(height) = 0;*/
	cv::Mat roimat = cat(cv::Rect(0, 1, width, height));
	gy = roimat - img;
	gy.row(height - 1) = 0;
}

// calculate horizontal gradient, gxx(i+1, j) = gx(i+1, j) - gx(i, j)
void lapx(cv::Mat &gx, cv::Mat &gxx)
{
	int height = gx.rows;
	int width = gx.cols;
	cv::Mat cat = repeat(gx, 1, 2);
	/*cat.col(width - 1) = 0;*/
	cv::Mat roi = cat(cv::Rect(width - 1, 0, width, height));
	gxx = gx - roi;
	gxx.col(0) = 0;
}

// calculate vertical gradient, gyy(i, j+1) = gy(i, j+1) - gy(i, j)
void lapy(cv::Mat &gy, cv::Mat &gyy)
{
	int height = gy.rows;
	int width = gy.cols;
	cv::Mat cat = repeat(gy, 2, 1);
	/*cat.row(height - 1) = 0;*/
	cv::Mat roi = cat(cv::Rect(0, height - 1, width, height));
	gyy = gy - roi;
	gyy.row(0) = 0;
}



void poisson_solver(cv::Mat &img, cv::Mat &gxx, cv::Mat &gyy, cv::Mat &result){

	int w = img.cols;
	int h = img.rows;
	int channel = img.channels();

	unsigned long int idx;

	cv::Mat lap = gxx + gyy;

	cv::Mat bound(img);
	bound(cv::Rect(1, 1, w - 2, h - 2)) = 0;
	double *dir_boundary = new double[h*w];

#pragma omp parallel for
	for (int i = 1; i < h - 1; i++)
		for (int j = 1; j < w - 1; j++)
		{
			unsigned long int idx = i*w + j;
			if ( i == 1 || i == h - 2 || j == 1 || j == w - 2 )
				dir_boundary[idx] = (int)bound.ATF(i, (j + 1)) + (int)bound.ATF(i, (j - 1)) + (int)bound.ATF(i - 1, j) + (int)bound.ATF(i + 1, j);
			else dir_boundary[idx] = 0;

		}


	cv::Mat diff(h, w, CV_32FC1);

#pragma omp parallel for
	for (int i = 0; i<h; i++)
	{
		for (int j = 0; j<w; j++)
		{
			unsigned long int idx = i*w + j;
			diff.ATF(i, j) = (float)(lap.ATF(i, j) - dir_boundary[idx]);
		}
	}

	double *btemp = new double[(h - 2)*(w - 2)];
#pragma omp parallel for
	for (int i = 0; i < h - 2; i++)
	{
		for (int j = 0; j < w - 2; j++)
		{
			unsigned long int idx = i*(w - 2) + j;
			btemp[idx] = diff.ATF(i + 1, j + 1);

		}
	}

	double *bfinal = new double[(h - 2)*(w - 2)];
	double *bfinal_t = new double[(h - 2)*(w - 2)];
	double *denom = new double[(h - 2)*(w - 2)];
	double *fres = new double[(h - 2)*(w - 2)];
	double *fres_t = new double[(h - 2)*(w - 2)];


	dst(btemp, bfinal, h - 2, w - 2);

	transpose(bfinal, bfinal_t, h - 2, w - 2);

	dst(bfinal_t, bfinal, w - 2, h - 2);

	transpose(bfinal, bfinal_t, w - 2, h - 2);

	int cx = 1;
	int cy = 1;

	for (int i = 0; i < w - 2; i++, cy++)
	{
		for (int j = 0, cx = 1; j < h - 2; j++, cx++)
		{
			idx = j*(w - 2) + i;
			denom[idx] = (float)2 * cos(pi*cy / ((double)(w - 1))) - 2 + 2 * cos(pi*cx / ((double)(h - 1))) - 2;

		}
	}

	for (idx = 0; (int)idx < (w - 2)*(h - 2); idx++)
	{
		bfinal_t[idx] = bfinal_t[idx] / denom[idx];
	}


	idst(bfinal_t, fres, h - 2, w - 2);

	transpose(fres, fres_t, h - 2, w - 2);

	idst(fres_t, fres, w - 2, h - 2);

	transpose(fres, fres_t, w - 2, h - 2);


	img.convertTo(result, CV_8UC1);


	cv::Mat tmp = cv::Mat(h-2, w-2, CV_64F, fres_t);
	double min, max;
	cv::minMaxLoc(tmp, &min, &max);


	#pragma omp parallel for
	for (int i = 0; i < h - 2; i++)
	{
		for (int j = 0; j < w - 2; j++)
		{
			unsigned long int idx = i*(w - 2) + j;
			//if (fres_t[idx] < 0.0)
			//	result.ATB(i+1, j+1) = 0;
			//else if (fres_t[idx] > 255.0)
			//	result.ATB(i+1, j+1) = 255;
			//else
			//	result.ATB(i+1, j+1) = (int)fres_t[idx];
			result.ATB(i+1,j+1) = (int) (255*(fres_t[idx]-min)/(max-min));
		}
	}

}

void poisson_solver_jacobi(cv::Mat &img, cv::Mat &gxx, cv::Mat &gyy, cv::Mat &result) {
	int w = img.cols;
	int h = img.rows;
	int channel = img.channels();

	unsigned long int idx;

	cv::Mat lap = gxx + gyy;

	cv::Mat bound(img);
	bound(cv::Rect(1, 1, w - 2, h - 2)) = 0;
	double *dir_boundary = new double[h*w];

	#pragma omp parallel for
	for (int i = 1; i < h - 1; i++)
		for (int j = 1; j < w - 1; j++)
		{
			idx = i*w + j;
			if (i == 1 || i == h - 2 || j == 1 || j == w - 2)
				dir_boundary[idx] = (int)bound.ATF(i, (j + 1)) + (int)bound.ATF(i, (j - 1)) + (int)bound.ATF(i - 1, j) + (int)bound.ATF(i + 1, j);
			else dir_boundary[idx] = 0;

		}


	cv::Mat diff(h, w, CV_32FC1);
	#pragma omp parallel for
	for (int i = 0; i < h; i++)
	{
		for (int j = 0; j < w; j++)
		{
			idx = i*w + j;
			diff.ATF(i, j) = (float)(-lap.ATF(i, j) + dir_boundary[idx]);
		}
	}

	double *gtest = new double[(h - 2)*(w - 2)];
	#pragma omp parallel for
	for (int i = 0; i < h - 2; i++)
	{
		for (int j = 0; j < w - 2; j++)
		{
			idx = i*(w - 2) + j;
			gtest[idx] = diff.ATF(i + 1, j + 1);

		}
	}
	//Iteration begins
	cv::Mat U = cv::Mat::zeros(img.size(), CV_32FC3);
	int k = 0;
	while (k <= 3000){
		#pragma omp parallel for
		for (int i = 1; i < h - 1; i++)
		{
			for (int j = 1; j < w - 1; j++)
			{
				U.ATF(i, j) = (float)((U.ATF(i + 1, j) + U.ATF(i, j + 1) + U.ATF(i - 1, j) + U.ATF(i, j - 1) + diff.ATF(i, j)) / 4.0);
			}
		}
		k++;
	}

	img.convertTo(result, CV_8UC1);
	#pragma omp parallel for
	for (int i = 1; i < h - 1; i++)
	{
		for (int j = 1; j < w - 1; j++)
		{
			if (U.ATF(i,j) < 0.0)
				result.ATB(i , j ) = 0;
			else if (U.ATF(i, j) > 255.0)
				result.ATB(i, j) = 255;
			else
				result.ATB(i , j ) = (int)U.ATF(i, j);
		}
	}
}


//IplImage versions

void lapx( const IplImage *img, IplImage *gxx)
{
	int w = img->width;
	int h = img->height;
	int channel = img->nChannels;

	cvZero( gxx );
	for(int i=0;i<h;i++)
		for(int j=0;j<w-1;j++)
			for(int c=0;c<channel;++c)
			{
				CV_IMAGE_ELEM(gxx,float,i,(j+1)*channel+c) =
						(float)CV_IMAGE_ELEM(img,float,i,(j+1)*channel+c) - (float)CV_IMAGE_ELEM(img,float,i,j*channel+c);
			}
}
void lapy( const IplImage *img, IplImage *gyy)
{
	int w = img->width;
	int h = img->height;
	int channel = img->nChannels;

	cvZero( gyy );
	for(int i=0;i<h-1;i++)
		for(int j=0;j<w;j++)
			for(int c=0;c<channel;++c)
			{
				CV_IMAGE_ELEM(gyy,float,i+1,j*channel+c) =
					(float)CV_IMAGE_ELEM(img,float,(i+1),j*channel+c) - (float)CV_IMAGE_ELEM(img,float,i,j*channel+c);

			}
}

void poisson_solver(const IplImage *img, IplImage *gxx , IplImage *gyy, cv::Mat &result)
{

	int w = img->width;
	int h = img->height;
	int channel = img->nChannels;

	unsigned long int idx,idx1;

	IplImage *lap  = cvCreateImage(cvGetSize(img), 32, 1);

	for(int i =0;i<h;i++)
		for(int j=0;j<w;j++)
			CV_IMAGE_ELEM(lap,float,i,j)=CV_IMAGE_ELEM(gyy,float,i,j)+CV_IMAGE_ELEM(gxx,float,i,j);

	//cv::Mat bound(img);
	cv::Mat bound = cv::cvarrToMat(img);

	for(int i =1;i<h-1;i++)
		for(int j=1;j<w-1;j++)
		{
			bound.at<uchar>(i,j) = 0.0;
		}

	double *f_bp = new double[h*w];


	for(int i =1;i<h-1;i++)
		for(int j=1;j<w-1;j++)
		{
			idx=i*w + j;
			f_bp[idx] = -4*(int)bound.at<uchar>(i,j) + (int)bound.at<uchar>(i,(j+1)) + (int)bound.at<uchar>(i,(j-1))
					+ (int)bound.at<uchar>(i-1,j) + (int)bound.at<uchar>(i+1,j);
		}


	cv::Mat diff = cv::Mat(h,w,CV_32FC1);
	for(int i =0;i<h;i++)
	{
		for(int j=0;j<w;j++)
		{
			idx = i*w+j;
			diff.at<float>(i,j) = (CV_IMAGE_ELEM(lap,float,i,j) - f_bp[idx]);
		}
	}
	double *gtest = new double[(h-2)*(w-2)];
	for(int i = 0 ; i < h-2;i++)
	{
		for(int j = 0 ; j < w-2; j++)
		{
			idx = i*(w-2) + j;
			gtest[idx] = diff.at<float>(i+1,j+1);

		}
	}
	///////////////////////////////////////////////////// Find DST  /////////////////////////////////////////////////////

	double *gfinal = new double[(h-2)*(w-2)];
	double *gfinal_t = new double[(h-2)*(w-2)];
	double *denom = new double[(h-2)*(w-2)];
	double *f3 = new double[(h-2)*(w-2)];
	double *f3_t = new double[(h-2)*(w-2)];
	double *img_d = new double[(h)*(w)];

	dst(gtest,gfinal,h-2,w-2);

	transpose(gfinal,gfinal_t,h-2,w-2);

	dst(gfinal_t,gfinal,w-2,h-2);

	transpose(gfinal,gfinal_t,w-2,h-2);

	int cx=1;
	int cy=1;

	for(int i = 0 ; i < w-2;i++,cy++)
	{
		for(int j = 0,cx = 1; j < h-2; j++,cx++)
		{
			idx = j*(w-2) + i;
			denom[idx] = (float) 2*cos(pi*cy/( (double) (w-1))) - 2 + 2*cos(pi*cx/((double) (h-1))) - 2;

		}
	}

	for(idx = 0 ; idx < (w-2)*(h-2) ;idx++)
	{
		gfinal_t[idx] = gfinal_t[idx]/denom[idx];
	}


	idst(gfinal_t,f3,h-2,w-2);

	transpose(f3,f3_t,h-2,w-2);

	idst(f3_t,f3,w-2,h-2);

	transpose(f3,f3_t,w-2,h-2);

	for(int i = 0 ; i < h;i++)
	{
		for(int j = 0 ; j < w; j++)
		{
			idx = i*w + j;
			img_d[idx] = (double)CV_IMAGE_ELEM(img,uchar,i,j);
		}
	}
	for(int i = 1 ; i < h-1;i++)
	{
		for(int j = 1 ; j < w-1; j++)
		{
			idx = i*w + j;
			img_d[idx] = 0.0;
		}
	}
	int id1,id2;
	for(int i = 1,id1=0 ; i < h-1;i++,id1++)
	{
		for(int j = 1,id2=0 ; j < w-1; j++,id2++)
		{
			idx = i*w + j;
			idx1= id1*(w-2) + id2;
			img_d[idx] = f3_t[idx1];
		}
	}

	for(int i = 0 ; i < h;i++)
	{
		for(int j = 0 ; j < w; j++)
		{
			idx = i*w + j;
			if(img_d[idx] < 0.0)
				result.at<uchar>(i,j) = 0;
			else if(img_d[idx] > 255.0)
				result.at<uchar>(i,j) = 255.0;
			else
				result.at<uchar>(i,j) = img_d[idx];
		}
	}

}
