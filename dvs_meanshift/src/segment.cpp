/*
Copyright (C) 2006 Pedro Felzenszwalb

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

#include <cstdio>
#include <cstdlib>
#include "graph3d/segment.h"
#include "graph3d/image.h"
#include "graph3d/misc.h"
#include "graph3d/pnmfile.h"
#include "graph3d/segment-image.h"


/*#include <cstdio>
#include <cstdlib>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "graph3d/image.h"
#include "graph3d/misc.h"
#include "graph3d/pnmfile.h"
#include "graph3d/segment-image.h"


int main(int argc, char **argv) {
  if (argc != 6) {
    fprintf(stderr, "usage: %s sigma k min input(ppm) output(ppm)\n", argv[0]);
    return 1;
  }
  
  float sigma = atof(argv[1]);
  float k = atof(argv[2]);
  int min_size = atoi(argv[3]);
	
  printf("loading input image.\n");
  image<rgb> *input = loadPPM(argv[4]);
	
  printf("processing\n");
  int num_ccs; 
  image<rgb> *seg = segment_image(input, sigma, k, min_size, &num_ccs); 
  savePPM(seg, argv[5]);


  printf("got %d components\n", num_ccs);
  printf("done! uff...thats hard work.\n");

  return 0;
}*/



//void imadjust(const cv::Mat & src, cv::Mat & dst, int tol = 1, cv::Vec2i in = cv::Vec2i(0, 255), cv::Vec2i out = cv::Vec2i(0, 255))
void imadjust(const cv::Mat & src, cv::Mat & dst)
{
    // src : input CV_8UC1 image
    // dst : output CV_8UC1 imge
    // tol : tolerance, from 0 to 100.
    // in  : src image bounds
    // out : dst image buonds

	int in[2]; in[0]=0; in[1]=255;
	int out[2]; out[0]=0; out[1]=255;
	int tol =1;



    dst = src.clone();

    tol = std::max(0, std::min(100, tol));

    if (tol > 0)
    {
        // Compute in and out limits

        // Histogram
        std::vector<int> hist(256, 0);
        for (int r = 0; r < src.rows; ++r) {
            for (int c = 0; c < src.cols; ++c) {
                hist[src.at<uint8_t>(cv::Point(r,c))]++;
            }
        }

        // Cumulative histogram
        std::vector<int> cum = hist;
        for (int i = 1; i < hist.size(); ++i) {
            cum[i] = cum[i - 1] + hist[i];
        }

        // Compute bounds
        int total = src.rows * src.cols;
        int low_bound = total * tol / 100;
        int upp_bound = total * (100-tol) / 100;
        in[0] = distance(cum.begin(), lower_bound(cum.begin(), cum.end(), low_bound));
        in[1] = distance(cum.begin(), lower_bound(cum.begin(), cum.end(), upp_bound));

    }

    // Stretching
    float scale = float(out[1] - out[0]) / float(in[1] - in[0]);
    for (int r = 0; r < dst.rows; ++r)
    {
        for (int c = 0; c < dst.cols; ++c)
        {
            int vs = std::max(src.at<uint8_t>(cv::Point(r, c)) - in[0], 0);


            int vd = std::min(int(vs * scale + 0.5f) + out[0], out[1]);
            dst.at<uint8_t>(cv::Point(r, c)) = cv::saturate_cast<uchar>(vd);
        }
    }
}

//This function is just an interface to call the segment_image function
void im2segment(cv::Mat input, cv::Mat cv_image_output, float sigma, float k, int min_size, int *num_ccs)
{

	image<rgb> *inputIm=new image<rgb>(input.cols, input.rows);


	for (int y = 0; y < input.rows; y++) {
	    for (int x = 0; x < input.cols; x++) {
	      imRef(inputIm, x, y).r = input.at<uint8_t>(cv::Point(x, y));
	      imRef(inputIm, x, y).g = input.at<uint8_t>(cv::Point(x, y));
	      imRef(inputIm, x, y).b = input.at<uint8_t>(cv::Point(x, y));
	    }
	}

	//std::cout<<"I'm here!!"<<std::endl;
	//savePPM(inputIm, "/home/fran/input_image.ppm");

	image<rgb> *seg = segment_image(inputIm, sigma, k, min_size, num_ccs);

	//cv_image_output.encoding = "bgr8";
	//cv_image_output.image =cv::Mat(input.rows, input.cols, CV_8UC3);

	for (int y = 0; y < input.rows; y++) {
		for (int x = 0; x < input.cols; x++) {
		  //output.at<uint8_t>(cv::Point(x, y))=imRef(seg, x, y);
		  cv_image_output.at<cv::Vec3b>(cv::Point(x, y)) = ( \
		              cv::Vec3b(imRef(seg,x,y).b, imRef(seg,x,y).g, imRef(seg,x,y).r));
		}
	}


	/*image<uchar> *inputTmp=new image<uchar>(input.cols, input.rows);;
	for (int y = 0; y < input.rows; y++) {
		for (int x = 0; x < input.cols; x++) {
		  imRef(inputTmp, x, y) = input.at<uint8_t>(cv::Point(x, y));
		}
	}
	savePGM(inputTmp, "/home/fran/input_image.ppm");
	 */

	//printf("got %d components\n", num_ccs);
	//printf("done! uff...thats hard work.\n");
}

