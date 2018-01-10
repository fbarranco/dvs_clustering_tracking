
#include "dvs_meanshift/meanshift.h"
#include "dvs_meanshift/color_meanshift.h"
#include "image_reconstruct/image_reconst.h"
#include <time.h>

//remove this include, just for debugging
#include "graph3d/pnmfile.h"
#include <std_msgs/Float32.h>

#define min(x,y) ((x)<(y)?(x):(y))

double epsilon = 1E-10;
int counterGlobal = 0;

namespace dvs_meanshift {

void writeMatToFile(cv::Mat & m, const char* filename)
{
    std::ofstream fout(filename);

    if(!fout)
    {
        std::cout<<"File Not Opened"<<std::endl;  return;
    }

    for(int i=0; i<m.rows; i++)
    {
        for(int j=0; j<m.cols; j++)
        {
            fout<<m.at<double>(i,j)<<"\t";
        }
        fout<<std::endl;
    }
    fout.close();
}

class Parallel_pixel_cosine: public cv::ParallelLoopBody
{

public:

	Parallel_pixel_cosine(cv::Mat imgg) : img(imgg)
	{
	}

	void operator() (const cv::Range &r) const
	{
		for(int j=r.start; j<r.end; ++j)
		{
			double* current = const_cast<double*>(img.ptr<double>(j));
			double* last = current + img.cols;

			for (; current != last; ++current)
				*current = (double) cos((double) *current);
		}
	}

private:
    cv::Mat img;
};

void meshgrid(const cv::Mat &xgv, const cv::Mat &ygv, cv::Mat1i &X, cv::Mat1i &Y)
{
  cv::repeat(xgv.reshape(1,1), ygv.total(), 1, X);
  cv::repeat(ygv.reshape(1,1).t(), 1, xgv.total(), Y);
}

// helper function (maybe that goes somehow easier)
 void meshgridTest(const cv::Range &xgv, const cv::Range &ygv, cv::Mat1i &X, cv::Mat1i &Y)
{
  std::vector<int> t_x, t_y;
  for (int i = xgv.start; i <= xgv.end; i++) t_x.push_back(i);
  for (int i = ygv.start; i <= ygv.end; i++) t_y.push_back(i);

  meshgrid(cv::Mat(t_x), cv::Mat(t_y), X, Y);
}

Meanshift::Meanshift(ros::NodeHandle & nh, ros::NodeHandle nh_private) : nh_(nh)
{
  used_last_image_ = false;

  // get parameters of display method
  std::string display_method_str;
  nh_private.param<std::string>("display_method", display_method_str, "");
  display_method_ = (display_method_str == std::string("grayscale")) ? GRAYSCALE : RED_BLUE;

  // setup subscribers and publishers
  //event_sub_ = nh_.subscribe("events", 1, &Meanshift::eventsCallback, this);
  event_sub_ = nh_.subscribe("events", 1, &Meanshift::eventsCallback_simple, this);
  camera_info_sub_ = nh_.subscribe("camera_info", 1, &Meanshift::cameraInfoCallback, this);

  image_transport::ImageTransport it_(nh_);
  
  image_pub_ = it_.advertise("dvs_rendering", 1);
  image_segmentation_pub_ = it_.advertise("dvs_segmentation", 1);


  //DEBUGGING
  image_debug1_pub_ = it_.advertise("dvs_debug1", 1);
  image_debug2_pub_ = it_.advertise("dvs_debug2", 1);
  //NOT DEBUGGING

  //Initializing params for color meanshift
  //computing image calibration
  numRows=DVSH; numCols=DVSW;
  mPixels=numRows*numCols;  maxPixelDim=3;

  spaceDivider=7; timeDivider = 5;
  maxIterNum=1;
  tolFun=0.0001;
  kernelFun[0] = 'g'; kernelFun[1] = 'a'; kernelFun[2] = 'u'; kernelFun[3] = 's';
  kernelFun[4] = 's'; kernelFun[5] = 'i'; kernelFun[6] = 'a'; kernelFun[7] = 'n';

  //Initialize pointers
  outputFeatures = NULL; imagePtr = NULL; pixelsPtr = NULL;
  initializationPtr=new double[maxPixelDim*mPixels/4];

  /*for (int i = 0; i < numRows; i++)
  	for (int j = 0; j < numCols; j++)
  	{
  		initializationPtr[i*numCols + j]=j/spaceDivider;//x
  		initializationPtr[mPixels+i*numCols+j]=i/spaceDivider;//y
  		initializationPtr[2*mPixels+i*numCols+j]=0;//img

  	}*/
  for (int i = 0; i < numRows/2; i++)
	for (int j = 0; j < numCols/2; j++)
	{
		initializationPtr[i*numCols/2 + j]=j/spaceDivider;//x
		initializationPtr[mPixels/4+i*numCols/2+j]=i/spaceDivider;//y
		initializationPtr[2*mPixels/4+i*numCols/2+j]=0;//img

	}

  //Initialize params for graph3d segmentation
  sigma = 0.75;
  k = 500;
  min_region = (int)(numRows*numCols*0.01);
  num_components = 0;

  //Initialization of random seed
  srand(time(NULL));//Random seed initialization
  //asm("rdtsc\n"
  //    "mov edi, eax\n"
  //    "call   srand");
}

Meanshift::~Meanshift()
{
  delete [] initializationPtr;

  image_pub_.shutdown();  
  image_segmentation_pub_.shutdown();
}

void Meanshift::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  camera_matrix_ = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      camera_matrix_.at<double>(cv::Point(i, j)) = msg->K[i+j*3];

  dist_coeffs_ = cv::Mat(msg->D.size(), 1, CV_64F);
  for (int i = 0; i < msg->D.size(); i++)
    dist_coeffs_.at<double>(i) = msg->D[i];
}


void Meanshift::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  // only create image if at least one subscriber
  if (image_pub_.getNumSubscribers() > 0)
  {
    cv_bridge::CvImage cv_image;

    if (display_method_ == RED_BLUE)
    {
      cv_image.encoding = "bgr8";

      if (last_image_.rows == msg->height && last_image_.cols == msg->width)
      {
        last_image_.copyTo(cv_image.image);
        used_last_image_ = true;
      }
      else
      {
        cv_image.image = cv::Mat(msg->height, msg->width, CV_8UC3);
        cv_image.image = cv::Scalar(128, 128, 128);
      }

      for (int i = 0; i < msg->events.size(); ++i)
      {
        const int x = msg->events[i].x;
        const int y = msg->events[i].y;

        cv_image.image.at<cv::Vec3b>(cv::Point(x, y)) = (
            msg->events[i].polarity == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
      }
    }
    else
    {
      cv_image.encoding = "mono8";
      cv_image.image = cv::Mat(msg->height, msg->width, CV_8U);
      cv_image.image = cv::Scalar(128);

      cv::Mat on_events = cv::Mat(msg->height, msg->width, CV_8U);
      on_events = cv::Scalar(0);

      cv::Mat off_events = cv::Mat(msg->height, msg->width, CV_8U);
      off_events = cv::Scalar(0);

      // count events per pixels with polarity
      for (int i = 0; i < msg->events.size(); ++i)
      {
        const int x = msg->events[i].x;
        const int y = msg->events[i].y;

        if (msg->events[i].polarity == 1)
          on_events.at<uint8_t>(cv::Point(x, y))++;
        else
          off_events.at<uint8_t>(cv::Point(x, y))++;
      }

        // scale image
      cv::normalize(on_events, on_events, 0, 128, cv::NORM_MINMAX, CV_8UC1);
      cv::normalize(off_events, off_events, 0, 127, cv::NORM_MINMAX, CV_8UC1);

      cv_image.image += on_events;
      cv_image.image -= off_events;
    }
      image_pub_.publish(cv_image.toImageMsg());
  }

  if (image_segmentation_pub_.getNumSubscribers() > 0)
  {
	  cv_bridge::CvImage cv_segments;
	  //cv_bridge::CvImage cv_debug1;
	  //cv_bridge::CvImage cv_debug2;


	//Doing color meanshift
	cv::Mat timestamp_matrix = cv::Mat(numRows, numCols, CV_64F);
	timestamp_matrix = cv::Scalar(0.);
	cv::Mat timestamp_matrix_out;

	//Reading time of first event and saving it
	//ROS_ERROR("Reading values from timestamp_matrix_out: %d", (int) msg->events.size());
	uint64_t first_timestamp = msg->events[0].ts.toNSec();
	// count events per pixels with polarity
	for (int i = 0; i < msg->events.size(); ++i)
	{
		const int x = msg->events[i].x;
		const int y = msg->events[i].y;

		double event_timestamp =  (0.001*(double)(msg->events[i].ts.toNSec()-first_timestamp));//now in usecs

		if (msg->events[i].polarity == 1)
		  timestamp_matrix.at<double>(cv::Point(x, y))= event_timestamp*0.001;
		else
		  timestamp_matrix.at<double>(cv::Point(x, y))=-event_timestamp*0.001;
	}

	//First create the interface and
	//cv_debug1.image =result_image;
	//cv::Mat img_hist_equalized;
	//cv::equalizeHist(result_image, img_hist_equalized); //equalize the histogram
	//cv_debug1.image = img_hist_equalized;



	//POISSON SOLVER IMPLEMENTATION
	/*----------------------------------------------------------------------------* /
	cv::Mat debugIm1= cv::Mat(numRows, numCols, CV_8UC1);

	double min, max;
	cv::minMaxLoc(timestamp_matrix, &min, &max);

	cv::normalize(timestamp_matrix, debugIm1, 0, 255, cv::NORM_MINMAX, CV_8UC1);

	// Here, we are assuming that our image is a gradient already (gx and gy)
	cv::Mat gxx = cv::Mat(numRows, numCols, CV_64F);
	cv::Mat gyy = cv::Mat(numRows, numCols, CV_64F);

	cv::Mat tmp;
	debugIm1.convertTo(tmp, CV_64F);
	tmp = (tmp/255)*(max-min)+min;


	lapx(tmp, gxx);
	lapy(tmp, gyy);

	cv::Mat lapfnc = cv::Mat(numRows, numCols, CV_64F);
	cv::Mat fcos = cv::Mat(numRows, numCols, CV_64F);
	cv::Mat denom = cv::Mat(numRows, numCols, CV_64F);
	cv::Mat output = cv::Mat(numRows, numCols, CV_64F);
	lapfnc = gxx+gyy;
	cv::dct(lapfnc, fcos);

	cv::Mat1i XX, YY;
	cv::Mat XX_bis, YY_bis;
	meshgridTest(cv::Range(0,numCols-1), cv::Range(0, numRows-1), XX, YY);

	XX.convertTo(XX_bis, CV_64F); YY.convertTo(YY_bis, CV_64F);
	XX_bis = CV_PI*XX_bis/numCols; YY_bis=CV_PI*YY_bis/numRows;

	parallel_for_(cv::Range(0,XX_bis.rows), Parallel_pixel_cosine(XX_bis));
	parallel_for_(cv::Range(0,YY_bis.rows), Parallel_pixel_cosine(YY_bis));

	denom = 2*XX_bis-2 + 2*YY_bis-2;

	double first_elem =fcos.at<double>(cv::Point(0, 0));
	cv::divide(fcos, denom, fcos);
	fcos.at<double>(cv::Point(0, 0))=first_elem; // remove the Inf

	cv::idct(fcos, output);
	double min_bis, max_bis;
	cv::minMaxLoc(output, &min_bis, &max_bis);
	output = output - min_bis;
	cv::Mat result_image, output_norm;
	cv::normalize(output, output_norm, 0, 255, cv::NORM_MINMAX, CV_8UC1);

	cv::medianBlur(output_norm,result_image,3);
	/*----------------------------------------------------------------------------*/
	//Alternatively, one can use only the time
	cv::Mat result_image;
	cv::normalize(timestamp_matrix, result_image, 0, 255, cv::NORM_MINMAX, CV_8UC1);
	/*----------------------------------------------------------------------------*/

	//cv::Mat kk1;
	//result_image.convertTo(kk1, CV_64F);
	//writeMatToFile(kk1,"/home/fran/kk1.txt");

	//cv_debug1.encoding = "mono8";
	//cv_debug1.image = result_image;
	//image_debug1_pub_.publish(cv_debug1.toImageMsg());

	cv::Mat temporalIm, temporalImSubsampled;
	result_image.convertTo(temporalIm, CV_64F);
	temporalIm = temporalIm/timeDivider;
	cv::pyrDown(temporalIm, temporalImSubsampled, cv::Size(numCols/2, numRows/2));
	//Initialize positions (only once)
	outputFeatures=new double[maxPixelDim*mPixels/4];
	pixelsPtr=new double[maxPixelDim*mPixels/4];
	imagePtr=new double[mPixels/4];

	memcpy(pixelsPtr, initializationPtr, maxPixelDim*mPixels/4*sizeof(double)); //copy event the 0s image
	memcpy(pixelsPtr+2*mPixels/4, (void*)(temporalImSubsampled.data), mPixels/4*sizeof(double)); //copy event the 0s image
	memcpy(imagePtr, (void*)(temporalImSubsampled.data), mPixels/4*sizeof(double)); //copy event the 0s image

	//extract the features
	colorMeanShiftFilt(outputFeatures, pixelsPtr, mPixels/4, maxPixelDim, imagePtr, \
			numRows/2, numCols/2, spaceDivider, kernelFun, maxIterNum, tolFun);

	cv::Mat meanshiftIm;
	cv::Mat meanshiftIm_small = cv::Mat(numRows/2, numCols/2, CV_64F, outputFeatures+2*mPixels/4);
	meanshiftIm_small = meanshiftIm_small*timeDivider;
	cv::pyrUp(meanshiftIm_small, meanshiftIm, cv::Size(numCols, numRows));

	//cv::Mat kk = cv::Mat(numRows, numCols, CV_64F, imagePtr);
	//kk = kk*timeDivider;
	//writeMatToFile(kk,"/home/fran/imagePtr.txt");

	//do the segmentation
//	cv::Mat filtered_im = cv::Mat(numRows, numCols, CV_64F, outputFeatures+2*mPixels);
//	filtered_im = filtered_im*timeDivider;
	cv::Mat meanshiftImNormalized;
	//meanshift_im.convertTo(meanshift_im_normalized,CV_8UC1);
	cv::normalize(meanshiftIm, meanshiftImNormalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);

	//cv_debug2.encoding = "mono8";
	//cv_debug2.image = meanshift_im_normalized;
	//image_debug2_pub_.publish(cv_debug2.toImageMsg());

	//meanshift_im_normalized= cv::imread("/home/fran/mug.pgm", CV_LOAD_IMAGE_GRAYSCALE);

	//tmp has the values that are non-zero

	cv_segments.encoding = "bgr8";
	cv_segments.image =cv::Mat(numRows, numCols, CV_8UC3);
	//cv_segments.image =cv::Mat(128, 128, CV_8UC3);
	im2segment(meanshiftImNormalized, cv_segments.image, sigma, k, min_region, &num_components);
	image_segmentation_pub_.publish(cv_segments.toImageMsg());

	//cv::Mat mask;
	//mask = (abs(timestamp_matrix) > 1E-6); // Puts 255 wherever it is true
	//mask.convertTo(mask, CV_64F);
	//writeMatToFile(mask,"/home/fran/mask.txt");

	//cv::Mat finalOutput;
	//cv::bitwise_and(cv_segments.image, cv::Scalar(255,255,255), finalOutput, mask);
	//cv_segments.image = finalOutput;
	//image_debug2_pub_.publish(cv_segments.toImageMsg());
	//-------------------------------------------------------------------------------------

	delete[] imagePtr;
	delete [] outputFeatures;
	delete [] pixelsPtr;
  }
}

/*void Meanshift::eventsCallback_simple(const dvs_msgs::EventArray::ConstPtr& msg)
{
  // only create image if at least one subscriber
  if (image_pub_.getNumSubscribers() > 0)
  {
    cv_bridge::CvImage cv_image;

    if (display_method_ == RED_BLUE)
    {
      cv_image.encoding = "bgr8";

      if (last_image_.rows == msg->height && last_image_.cols == msg->width)
      {
        last_image_.copyTo(cv_image.image);
        used_last_image_ = true;
      }
      else
      {
        cv_image.image = cv::Mat(msg->height, msg->width, CV_8UC3);
        cv_image.image = cv::Scalar(128, 128, 128);
      }

      for (int i = 0; i < msg->events.size(); ++i)
      {
        const int x = msg->events[i].x;
        const int y = msg->events[i].y;

        cv_image.image.at<cv::Vec3b>(cv::Point(x, y)) = (
            msg->events[i].polarity == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
      }
    }
    else
    {
      cv_image.encoding = "mono8";
      cv_image.image = cv::Mat(msg->height, msg->width, CV_8U);
      cv_image.image = cv::Scalar(128);

      cv::Mat on_events = cv::Mat(msg->height, msg->width, CV_8U);
      on_events = cv::Scalar(0);

      cv::Mat off_events = cv::Mat(msg->height, msg->width, CV_8U);
      off_events = cv::Scalar(0);

      // count events per pixels with polarity
      for (int i = 0; i < msg->events.size(); ++i)
      {
        const int x = msg->events[i].x;
        const int y = msg->events[i].y;

        if (msg->events[i].polarity == 1)
          on_events.at<uint8_t>(cv::Point(x, y))++;
        else
          off_events.at<uint8_t>(cv::Point(x, y))++;
      }

        // scale image
      cv::normalize(on_events, on_events, 0, 128, cv::NORM_MINMAX, CV_8UC1);
      cv::normalize(off_events, off_events, 0, 127, cv::NORM_MINMAX, CV_8UC1);

      cv_image.image += on_events;
      cv_image.image -= off_events;
    }
      image_pub_.publish(cv_image.toImageMsg());
  }

  if (image_segmentation_pub_.getNumSubscribers() > 0)
  {
	  cv_bridge::CvImage cv_segments;

	  //Doing color meanshift
	  cv::Mat diffTimestampMatrix = cv::Mat(numRows, numCols, CV_64F, cv::Scalar::all(0.));
	  cv::Mat timestampMatrix = cv::Mat(numRows, numCols, CV_64F, cv::Scalar::all(0.));

	  uint64_t first_timestamp = msg->events[0].ts.toNSec();
	  double final_timestamp =  (1E-6*(double)(msg->events[(msg->events.size())-1].ts.toNSec()-first_timestamp));

	  // count events per pixels with polarity
	  cv::Mat data=cv::Mat(3, msg->events.size(), CV_64F, cv::Scalar::all(0.));
	  int counter = 0;

	  //std::ofstream foutX("/home/fran/xpos.txt");
	  //std::ofstream foutY("/home/fran/ypos.txt");
	  //std::ofstream foutTime("/home/fran/timestamp.txt");

	  for (int i = 0; i < msg->events.size(); i++)
	  {
		  const int x = msg->events[i].x;
		  const int y = msg->events[i].y;

		  double event_timestamp =  (1E-6*(double)(msg->events[i].ts.toNSec()-first_timestamp));//now in usecs

		  //if (timestampMatrix.at<double>(cv::Point(x,y))> epsilon)
		  //	  diffTimestampMatrix.at<double>(cv::Point(x, y)) = event_timestamp -timestampMatrix.at<double>(cv::Point(x, y));
		  //timestampMatrix.at<double>(cv::Point(x, y))= event_timestamp;

		  if (event_timestamp < 15)
		  {
			  data.at<double>(cv::Point(i, 0))= (double)x/numCols;
			  data.at<double>(cv::Point(i, 1))= (double)y/numRows;
			  data.at<double>(cv::Point(i, 2))= event_timestamp/final_timestamp;//normalized
			  counter++;
		  }
		  else
		  {
			  //ROS_ERROR_STREAM("I'm using "<<counter<<" out of "<<msg->events.size());
			  break;
		  }
		  //foutX<<x<<"\t"; foutY<<y<<"\t"; foutTime<<event_timestamp<<"\t";
	  }
	  //foutX.close(); foutY.close(); foutTime.close();

/*
	  cv::Mat data=cv::Mat(3, 66, CV_64F, cv::Scalar::all(0.));
	  int counter = 0;
	  std::ifstream foutX("/home/fran/xpos.txt");
	  std::ifstream foutY("/home/fran/ypos.txt");
	  std::ifstream foutTime("/home/fran/timestamp.txt");
	  final_timestamp = 9.963;
	  for (int i = 0; i < 66; i++)
	  {
		  double x, y, event_timestamp;
		  foutX>>x; foutY>>y; foutTime>>event_timestamp;
		  data.at<double>(cv::Point(i, 0))= x/numCols;
		  data.at<double>(cv::Point(i, 1))= y/numRows;
		  data.at<double>(cv::Point(i, 2))= event_timestamp/final_timestamp;
	  }
	  foutX.close(); foutY.close(); foutTime.close();
* /

	  //----------------------------------------------------------------------------
	  //Alternatively, one can use only the time
	  //cv::Mat result_image;
	  //cv::normalize(diffTimestampMatrix, result_image, 0, 255, cv::NORM_MINMAX, CV_8UC1);
	  //----------------------------------------------------------------------------

	  cv::Mat clusterCenters;
	  cv::Mat segmentation=cv::Mat(numRows, numCols, CV_8UC3);
	  segmentation = cv::Scalar(128,128,128);

	  std::vector<double> clustCentX, clustCentY, clustCentZ;
	  std::vector<int> point2Clusters;
	  double bandwidth = 0.2;
	  std::vector<cv::Vec3b> RGBColors;
	  cv::RNG rng(12345);

	  //ROS_ERROR_STREAM("Just before meanshift "<<msg->events.size());
	  meanshiftCluster_Gaussian(data, &clustCentX, &clustCentY, &clustCentZ, &point2Clusters, bandwidth);

	  //Now, draw the different segments with a color
	  for(int i=0; i<clustCentX.size(); i++)
		 RGBColors.push_back(cv::Vec3b((uchar)random(), (uchar)random(), (uchar)random()));

	  counter =0;
	  for (int i = 0; i < msg->events.size(); i++)
	  {
		  const int x = msg->events[i].x;
		  const int y = msg->events[i].y;
		  double ts =  (1E-6*(double)(msg->events[i].ts.toNSec()-first_timestamp));//now in usecs

		  if(ts<15)
		  {
		  	  segmentation.at<cv::Vec3b>(cv::Point(x,y))=RGBColors[point2Clusters[i]];
		  	  counter++;
		  }
		  else
		  {
			  break;
		  }
		  //ROS_ERROR_STREAM("segmentation["<<x<<","<<y<<"] = "<<RGBColors[point2Clusters[i]]);
	  }

	cv_segments.encoding = "bgr8";
	cv_segments.image =segmentation;
    image_segmentation_pub_.publish(cv_segments.toImageMsg());
    //std::cin.ignore();

	//Saving RGB image
	//if (counterGlobal>20)
	//{
	//	char filename[80];
	//	sprintf(filename,"/home/fran/RGB_%0d.png", counterGlobal+1);
	//	cv::imwrite(filename , segmentation);
	//	exit(0);
	//}
	//counterGlobal++;
	//-------------------------------------------------------------------------------------

  }
}*/

void Meanshift::eventsCallback_simple(const dvs_msgs::EventArray::ConstPtr& msg)
{
  // only create image if at least one subscriber
  if (image_pub_.getNumSubscribers() > 0)
  {
    cv_bridge::CvImage cv_image;

    if (display_method_ == RED_BLUE)
    {
      cv_image.encoding = "bgr8";

      if (last_image_.rows == msg->height && last_image_.cols == msg->width)
      {
        last_image_.copyTo(cv_image.image);
        used_last_image_ = true;
      }
      else
      {
        cv_image.image = cv::Mat(msg->height, msg->width, CV_8UC3);
        cv_image.image = cv::Scalar(128, 128, 128);
      }

      for (int i = 0; i < msg->events.size(); ++i)
      {
        const int x = msg->events[i].x;
        const int y = msg->events[i].y;

        cv_image.image.at<cv::Vec3b>(cv::Point(x, y)) = (
            msg->events[i].polarity == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
      }
    }
    else
    {
      cv_image.encoding = "mono8";
      cv_image.image = cv::Mat(msg->height, msg->width, CV_8U);
      cv_image.image = cv::Scalar(128);

      cv::Mat on_events = cv::Mat(msg->height, msg->width, CV_8U);
      on_events = cv::Scalar(0);

      cv::Mat off_events = cv::Mat(msg->height, msg->width, CV_8U);
      off_events = cv::Scalar(0);

      // count events per pixels with polarity
      for (int i = 0; i < msg->events.size(); ++i)
      {
        const int x = msg->events[i].x;
        const int y = msg->events[i].y;

        if (msg->events[i].polarity == 1)
          on_events.at<uint8_t>(cv::Point(x, y))++;
        else
          off_events.at<uint8_t>(cv::Point(x, y))++;
      }

        // scale image
      cv::normalize(on_events, on_events, 0, 128, cv::NORM_MINMAX, CV_8UC1);
      cv::normalize(off_events, off_events, 0, 127, cv::NORM_MINMAX, CV_8UC1);

      cv_image.image += on_events;
      cv_image.image -= off_events;
    }
      image_pub_.publish(cv_image.toImageMsg());
  }

  if (image_segmentation_pub_.getNumSubscribers() > 0)
  {
	  cv_bridge::CvImage cv_segments;

	  //Doing color meanshift
	  //cv::Mat diffTimestampMatrix = cv::Mat(numRows, numCols, CV_64F, cv::Scalar::all(0.));
	  //cv::Mat timestampMatrix = cv::Mat(numRows, numCols, CV_64F, cv::Scalar::all(0.));

	  uint64_t first_timestamp = msg->events[0].ts.toNSec();
	  double final_timestamp =  (1E-6*(double)(msg->events[(msg->events.size())-1].ts.toNSec()-first_timestamp));
	  int counter = 0;

	  int beginEvent = 0;
	  int packet = 1500;
	  //ROS_ERROR_STREAM("Before while");
	  while(beginEvent < msg->events.size())
	  {
		  //SELECT SMALL PACKETS OF MAXIMUM 1000 events
		  counter = 0;
		  //ROS_ERROR_STREAM("Computing events ["<<beginEvent<<","<<min(beginEvent+packet, msg->events.size())<<"]");

		  // count events per pixels with polarity
		  cv::Mat data=cv::Mat(3, min(packet, msg->events.size()-beginEvent), CV_64F, cv::Scalar::all(0.));

		  std::ofstream foutX("/home/fran/xpos.txt");
		  std::ofstream foutY("/home/fran/ypos.txt");
		  std::ofstream foutTime("/home/fran/timestamp.txt");

		  for (int i = beginEvent; i < min(beginEvent+packet, msg->events.size()); i++)
		  {
			  //SELECT SMALL PACKETS OF MAXIMUM 1000 events
			  const int x = msg->events[counter].x;
			  const int y = msg->events[counter].y;

			  double event_timestamp =  (1E-6*(double)(msg->events[counter].ts.toNSec()-first_timestamp));//now in usecs

			  data.at<double>(cv::Point(counter, 0))= (double)x/numCols;
			  data.at<double>(cv::Point(counter, 1))= (double)y/numRows;
			  data.at<double>(cv::Point(counter, 2))= event_timestamp/final_timestamp;//normalized
			  counter++;

			  foutX<<x<<"\t"; foutY<<y<<"\t"; foutTime<<event_timestamp<<"\t";
		  }
		  foutX.close(); foutY.close(); foutTime.close();

		  cv::Mat clusterCenters;
		  cv::Mat segmentation=cv::Mat(numRows, numCols, CV_8UC3);
		  segmentation = cv::Scalar(128,128,128);

		  std::vector<double> clustCentX, clustCentY, clustCentZ;
		  std::vector<int> point2Clusters;
		  double bandwidth = 0.2;
		  std::vector<cv::Vec3b> RGBColors;
		  cv::RNG rng(12345);

		  //clock_t start, end;
		  //double elapsed;
		  //start = clock();

		  meanshiftCluster_Gaussian(data, &clustCentX, &clustCentY, &clustCentZ, &point2Clusters, bandwidth);

		  //end = clock();
		  //elapsed = ((double) (end - start)) / CLOCKS_PER_SEC;
		  //ROS_ERROR_STREAM("Num. packets = "<<data.cols<<" elapsed time = "<<elapsed<<". Time/packet = "<<elapsed/data.cols);


		  //Now, draw the different segments with a color
		  for(int i=0; i<clustCentX.size(); i++)
			 RGBColors.push_back(cv::Vec3b((uchar)random(), (uchar)random(), (uchar)random()));

		  counter =0;
		  for (int i = beginEvent; i < min(beginEvent+packet, msg->events.size()); i++)
		  {
			  const int x = msg->events[counter].x;
			  const int y = msg->events[counter].y;
			  double ts =  (1E-6*(double)(msg->events[counter].ts.toNSec()-first_timestamp));//now in usecs

			  if(ts<15)
			  {
				  segmentation.at<cv::Vec3b>(cv::Point(x,y))=RGBColors[point2Clusters[counter]];
			  }
			  else
			  {
				  break;
			  }
			  counter++;
		  }

		  cv_segments.encoding = "bgr8";
		  cv_segments.image =segmentation;
		  image_segmentation_pub_.publish(cv_segments.toImageMsg());
		  //std::cin.ignore();

		  beginEvent +=packet;
	  }

	//std::cin.ignore();

	//Saving RGB image
	//if (counterGlobal>20)
	//{
	//	char filename[80];
	//	sprintf(filename,"/home/fran/RGB_%0d.png", counterGlobal+1);
	//	cv::imwrite(filename , segmentation);
	//	exit(0);
	//}
	//counterGlobal++;
	//-------------------------------------------------------------------------------------
  }
}




} // namespace
