
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
const int MAX_DISTANCE = 500; //actually the real distance is sqrt(MAX_DISTANCE)

bool firstevent = true;
double firsttimestamp;

//std::ofstream foutX("/home/fran/0_tmp/kk/shapes_rotation_totalevents.txt");
//std::ofstream fouttotalproc("/home/fran/0_tmp/kk/shapes_rotation_totalprocessing_f.txt");
//std::ofstream foutX("/home/fran/0_tmp/kk/shapes_rotation_2_totalevents.txt");
//std::ofstream fouttotalproc("/home/fran/0_tmp/kk/shapes_rotation_2_totalprocessing_f.txt");
//std::ofstream foutX("/home/fran/0_tmp/kk/shapes_translation_totalevents.txt");
//std::ofstream fouttotalproc("/home/fran/0_tmp/kk/shapes_translation_totalprocessing_f.txt");
//std::ofstream foutX("/home/fran/0_tmp/kk/shapes_translation_2_totalevents.txt");
//std::ofstream fouttotalproc("/home/fran/0_tmp/kk/shapes_translation_2_totalprocessing_f.txt");
//std::ofstream foutX("/home/fran/0_tmp/kk/shapes_6dof_totalevents.txt");
//std::ofstream fouttotalproc("/home/fran/0_tmp/kk/shapes_6dof_totalprocessing_f.txt");
//std::ofstream foutX("/home/fran/0_tmp/kk/shapes_6dof_2_totalevents.txt");
//std::ofstream fouttotalproc("/home/fran/0_tmp/kk/shapes_6dof_2_totalprocessing_f.txt");
//std::ofstream foutnumclust("/home/fran/0_tmp/kk/shapes_translation_numclusters_f.txt");
//std::ofstream foutnumclust("/home/fran/0_tmp/kk/shapes_rotation_numclusters_f.txt");
//std::ofstream foutnumclust("/home/fran/0_tmp/kk/shapes_6dof_numclusters_f.txt");
//std::ofstream foutnumclust("/home/fran/0_tmp/kk/shapes_6dof_2_numclusters_f.txt");
//std::ofstream foutnumclust("/home/fran/0_tmp/kk/shapes_rotation_2_numclusters_f.txt");
//std::ofstream foutnumclust("/home/fran/0_tmp/kk/shapes_translation_2_numclusters_f.txt");

#if KALMAN_FILTERING
std::ofstream foutX("/home/fran/0_tmp/kk/traj.txt");
//std::ofstream foutX_estim("/home/fran/0_tmp/kk/filt_traj.txt");
//std::ofstream foutT("/home/fran/0_tmp/kk/timetraj.txt");
#endif

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

void findClosestCluster(std::vector<int> * clustID, std::vector<double> clustX, std::vector<double> clustY,std::vector<double>clustX_old,std::vector<double> clustY_old)
{

	//for (int i = 0; i<clustX.size(); i++)
	//	ROS_ERROR_STREAM("ClusterCenter("<<i+1<<",:)=["<<clustX.at(i)<<","<<clustY.at(i)<<"];");
	//for (int i = 0; i<clustX_old.size(); i++)
	//	ROS_ERROR_STREAM("prev_ClusterCenter("<<i+1<<",:)=["<<clustX_old.at(i)<<","<<clustY_old.at(i)<<"];");

	std::vector<double> distvalue (clustX.size());
	double tmp;

	for(int i=0; i<clustX.size(); i++)
	{
		distvalue[i]=MAX_DISTANCE;
		(*clustID).push_back(-1); //no match when clustID is -1
		for(int j=0; j<clustX_old.size(); j++)
		{
			tmp= (clustX[i]*DVSW-clustX_old[j]*DVSW)*(clustX[i]*DVSW-clustX_old[j]*DVSW) + (clustY[i]*DVSH-clustY_old[j]*DVSH)*(clustY[i]*DVSH-clustY_old[j]*DVSH);
			if(tmp<distvalue[i])
			{
				distvalue[i]=tmp;
				(*clustID)[i]=j;
				//ROS_ERROR_STREAM("Cluster "<<i+1<<" same than old cluster "<<j+1<<". Distance = "<<distvalue[i]);
			}
		}
	}
}

/*
void Meanshift::assignClusterColor(std::vector<int> * positionClusterColor, int numClusters, std::vector<int> matches, std::vector<int> oldPositions)
{

	if (matches.size()==0) //no mask --> assing all new colors
	{
		for(int i=0; i<numClusters; i++)
		{
			(*positionClusterColor).push_back(lastPositionClusterColor);
			lastPositionClusterColor = lastPositionClusterColor+1;
		}
	}
	else
	{
		for(int i=0; i<numClusters; i++)
		{
			if(matches[i]==-1) //No match with previous clusters --> new cluster, new color
			{
				(*positionClusterColor).push_back(lastPositionClusterColor);
				lastPositionClusterColor = lastPositionClusterColor+1;
			}
			else //Match with previous cluster --> assign color of previous cluster
			{
				(*positionClusterColor).push_back(oldPositions[matches[i]]);
			}
		}

	}
}*/

void Meanshift::assignClusterColor(std::vector<int> * positionClusterColor, int numClusters, std::vector<int> matches, std::vector<int> oldPositions, std::vector<int> activeTrajectories)
{

	if (matches.size()==0) //no mask --> assing all new colors
	{
		for(int i=0; i<numClusters; i++)
		{
			(*positionClusterColor).push_back(lastPositionClusterColor);
			lastPositionClusterColor = lastPositionClusterColor+1;
		}
	}
	else
	{
		for(int i=0; i<numClusters; i++)
		{
			if(matches[i]==-1) //No match with previous clusters --> new cluster, new color
			{
				//Check if the new position is being used
				while ( activeTrajectories.end() != find (activeTrajectories.begin(), activeTrajectories.end(), (lastPositionClusterColor & (MAX_NUM_CLUSTERS-1))))
					lastPositionClusterColor++;

				(*positionClusterColor).push_back(lastPositionClusterColor);
				lastPositionClusterColor = lastPositionClusterColor+1;
			}
			else //Match with previous cluster --> assign color of previous cluster
			{
				(*positionClusterColor).push_back(oldPositions[matches[i]]);
			}
		}

	}
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

  //Write xposition, yposition, and image in the same vector
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

  //Create first the RGB values and then assign it to the clusters
  //(hopefully, this will keep some stability in the cluster colors along consecutive frames)
  //Assume we won't have more than 50 clusters
  for(int i=0; i<MAX_NUM_CLUSTERS; i++)
  {
	  RGBColors.push_back(cv::Vec3b((uchar)random(), (uchar)random(), (uchar)random()));
	  //counterTrajectories.push_back(0);//initialize counter trajectories to all zeros
  }
  counterTrajectories=std::vector<int>(MAX_NUM_CLUSTERS,0);

  lastPositionClusterColor=0;

  //Initializing trajectories
  trajectories = cv::Mat(numRows, numCols, CV_8UC3);
  trajectories = cv::Scalar(128,128,128);

  //Initialize trajectory matrix
  allTrajectories = new std::vector<cv::Point>[MAX_NUM_CLUSTERS];
  for(int i = 0; i < MAX_NUM_CLUSTERS; i++)
  {
	  allTrajectories[i] = std::vector<cv::Point>(MAX_NUM_TRAJECTORY_POINTS);
  }
  prev_activeTrajectories=std::vector<int>(MAX_NUM_CLUSTERS,0);

  //Init Kalman filter
  //createKalmanFilter();
  vector_of_kf=std::vector<cv::KalmanFilter>(MAX_NUM_CLUSTERS);
  vector_of_meas=std::vector<cv::Mat>(MAX_NUM_CLUSTERS);
  vector_of_state=std::vector<cv::Mat>(MAX_NUM_CLUSTERS);
  for(int i=0; i<vector_of_kf.size(); i++)
	  createKalmanFilter(&(vector_of_kf[i]), &(vector_of_state[i]), &(vector_of_meas[i]));

  selectedTrajectory = -1;
  //foundBlobs = false;
  vector_of_foundBlobs=std::vector<bool>(MAX_NUM_CLUSTERS,false);
  notFoundBlobsCount = 0;
  //foundTrajectory=false;
  foundTrajectory = std::vector<bool>(MAX_NUM_CLUSTERS, false);
  //ticks = 0;
  vector_of_ticks=std::vector<double> (MAX_NUM_CLUSTERS,0.0);

  //BG activity filtering
  BGAFframe = cv::Mat(numRows, numCols, CV_32FC1);
  BGAFframe =cv::Scalar(0.);
}

Meanshift::~Meanshift()
{
  delete [] initializationPtr;

#if KALMAN_FILTERING
  foutX.close();
  //foutX_estim.close();
  //foutT.close();
#endif
  image_pub_.shutdown();  
  image_segmentation_pub_.shutdown();
  image_debug1_pub_.shutdown();
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


void Meanshift::createKalmanFilter(cv::KalmanFilter *kf, cv::Mat *state, cv::Mat *meas)
{
	/*
	 * This implementation takes 6 states position, velocity, and dimensions of the bounding box
	int stateSize = 6;
	int measSize = 4;
	int contrSize = 0;

	unsigned int type = CV_32F;

	//cv::KalmanFilter kf(stateSize, measSize, contrSize, type);
	kf.init(stateSize, measSize, contrSize, type);

	cv::Mat state(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
	cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
	// [E_x,E_y,E_v_x,E_v_y,E_w,E_h]

	// Transition State Matrix A
	// Note: set dT at each processing step!
	// [ 1 0 dT 0  0 0 ]
	// [ 0 1 0  dT 0 0 ]
	// [ 0 0 1  0  0 0 ]
	// [ 0 0 0  1  0 0 ]
	// [ 0 0 0  0  1 0 ]
	// [ 0 0 0  0  0 1 ]
	cv::setIdentity(kf.transitionMatrix);

	// Measure Matrix H
	// [ 1 0 0 0 0 0 ]
	// [ 0 1 0 0 0 0 ]
	// [ 0 0 0 0 1 0 ]
	// [ 0 0 0 0 0 1 ]
	kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
	kf.measurementMatrix.at<float>(0) = 1.0f;
	kf.measurementMatrix.at<float>(7) = 1.0f;
	kf.measurementMatrix.at<float>(16) = 1.0f;
	kf.measurementMatrix.at<float>(23) = 1.0f;

	// Process Noise Covariance Matrix Q
	// [ Ex   0   0     0     0    0  ]
	// [ 0    Ey  0     0     0    0  ]
	// [ 0    0   Ev_x  0     0    0  ]
	// [ 0    0   0     Ev_y  0    0  ]
	// [ 0    0   0     0     Ew   0  ]
	// [ 0    0   0     0     0    Eh ]
	//cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
	kf.processNoiseCov.at<float>(0) = 1e-2;
	kf.processNoiseCov.at<float>(7) = 1e-2;
	kf.processNoiseCov.at<float>(14) = 5.0f;
	kf.processNoiseCov.at<float>(21) = 5.0f;
	kf.processNoiseCov.at<float>(28) = 1e-2;
	kf.processNoiseCov.at<float>(35) = 1e-2;

	// Measures Noise Covariance Matrix R
	cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
	*/


	/*
	 * This implementation is only for one kalman filter
	 */
	int stateSize = 4;
	int measSize = 2;
	int contrSize = 0;

	unsigned int type = CV_32F;

	kf->init(stateSize, measSize, contrSize, type);

	//cv::Mat state(stateSize, 1, type);  // [x,y,v_x,v_y]
	//cv::Mat meas(measSize, 1, type);    // [z_x,z_y]
	state->create(stateSize,1,type);
	meas->create(measSize,1,type);
	// [E_x,E_y,E_v_x,E_v_y]


	// Transition State Matrix A
	// Note: set dT at each processing step!
	// [ 1 0 dT 0 ]
	// [ 0 1 0  dT]
	// [ 0 0 1  0 ]
	// [ 0 0 0  1 ]
	cv::setIdentity(kf->transitionMatrix);

	// Measure Matrix H
	// [ 1 0 0 0 ]
	// [ 0 1 0 0 ]
	kf->measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
	kf->measurementMatrix.at<float>(0) = 1.0f;
	kf->measurementMatrix.at<float>(5) = 1.0f;

	// Process Noise Covariance Matrix Q
	// [ Ex   0   0     0     ]
	// [ 0    Ey  0     0     ]
	// [ 0    0   Ev_x  0     ]
	// [ 0    0   0     Ev_y  ]
	//cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
	kf->processNoiseCov.at<float>(0) = 1e-2;//original values
	kf->processNoiseCov.at<float>(5) = 1e-2;
	kf->processNoiseCov.at<float>(10) = 5.0f;
	kf->processNoiseCov.at<float>(15) = 5.0f;

	kf->processNoiseCov.at<float>(0) = 1e-2;
	kf->processNoiseCov.at<float>(5) = 1e-2;
	kf->processNoiseCov.at<float>(10) = 7.0f;
	kf->processNoiseCov.at<float>(15) = 7.0f;

	// Measures Noise Covariance Matrix R
	cv::setIdentity(kf->measurementNoiseCov, cv::Scalar(1e-1));
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

      //Write the total number of events
      //foutX<<msg->events.size()<<std::endl;
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

/*
  // only create image if at least one subscriber
    if (image_debug1_pub_.getNumSubscribers() > 0)
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

        if(firstevent)
        {
        	firsttimestamp = (1E-6*(double)(msg->events[0].ts.toNSec()));
        	firstevent = false;
        }

        double maxTs;
        int posx, posy;
        double usTime = 16.0;
        double ts;
        for (int i = 0; i < msg->events.size(); ++i)
        {
          const int x = msg->events[i].x;
          const int y = msg->events[i].y;
          ts = (1E-6*(double)(msg->events[i].ts.toNSec())) - firsttimestamp;
#if BG_FILTERING
		  //BGAFframe.at<float>(cv::Point(x,y))=(float)(1E-6*(double)(msg->events[i].ts.toNSec()));
          BGAFframe.at<float>(cv::Point(x,y))=0.;
		  maxTs = -1;
		  //ROS_ERROR_STREAM("NEW EVENT "<<x<<","<<y<<":"<<ts);
		  for(int ii=-1; ii<=1; ii++)
			  for(int jj=-1; jj<=1; jj++)
			  {
				  posx = x + ii;
				  posy = y + jj;
				  if(posx<0)
					  posx = 0;
				  if(posy<0)
					  posy=0;
				  if(posx>numRows-1)
					  posx = numRows-1;
				  if(posy>numCols-1)
					  posy = numCols-1;

				  if(BGAFframe.at<float>(cv::Point(posx,posy)) > maxTs)
					  maxTs = BGAFframe.at<float>(cv::Point(posx,posy));

				  //ROS_ERROR_STREAM(posx<<","<<posy<<":"<<BGAFframe.at<float>(cv::Point(posx,posy)));
			  }
		  //ROS_ERROR_STREAM("maxTs: "<<maxTs);
		  BGAFframe.at<float>(cv::Point(x,y))=ts;
		  //ROS_ERROR_STREAM(BGAFframe.at<float>(cv::Point(x,y)) - maxTs);

		  if(BGAFframe.at<float>(cv::Point(x,y)) >= (maxTs + usTime))
		  {
			  continue;
			  //ROS_ERROR_STREAM("HERE");
		  }

#endif
		  cv_image.image.at<cv::Vec3b>(cv::Point(x, y)) = (
              msg->events[i].polarity == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
        }
        //ROS_ERROR_STREAM("DONE--------------------------------------------");

        //Write the total number of events
        //foutX<<msg->events.size()<<std::endl;
      }
      image_debug1_pub_.publish(cv_image.toImageMsg());
    }
*/


  if (image_segmentation_pub_.getNumSubscribers() > 0)
  {
	  cv_bridge::CvImage cv_segments;

	  //Doing color meanshift
	  //cv::Mat diffTimestampMatrix = cv::Mat(numRows, numCols, CV_64F, cv::Scalar::all(0.));
	  //cv::Mat timestampMatrix = cv::Mat(numRows, numCols, CV_64F, cv::Scalar::all(0.));

	  uint64_t first_timestamp = msg->events[0].ts.toNSec();
	  double final_timestamp =  (1E-6*(double)(msg->events[(msg->events.size())-1].ts.toNSec()-first_timestamp));
	  std::vector<bool> activeEvents(msg->events.size());

	  int counterIn = 0;
	  int counterOut = 0;

	  int beginEvent = 0;
	  int packet;
	  if(DVSW==240)
		packet = 1800;
	  else
		packet = 1500;
	  //ROS_ERROR_STREAM("Before while");


	  while(beginEvent < msg->events.size())
	  {
		  //if(msg->events.size()>15e3)
			  //ROS_ERROR_STREAM("BEGIN packet "<<beginEvent);
		  //SELECT SMALL PACKETS OF MAXIMUM 1500 events
		  counterIn = 0;
		  counterOut = 0;
		  //ROS_ERROR_STREAM("Computing events ["<<beginEvent<<","<<min(beginEvent+packet, msg->events.size())<<"]");
		  //ROS_ERROR_STREAM("SIZE of current package is "<<msg->events.size()<<"];");

		  // count events per pixels with polarity
		  cv::Mat data=cv::Mat(3, min(packet, msg->events.size()-beginEvent), CV_64F, cv::Scalar::all(0.));

		  //std::ofstream foutX("/home/fran/xpos.txt");
		  //std::ofstream foutY("/home/fran/ypos.txt");
		  //std::ofstream foutTime("/home/fran/timestamp.txt");

		  //Filter events
		  //cv::Mat BGAFframe = cv::Mat(numRows, numCols, CV_32FC1);
		  //frame =cv::Scalar(-1);

		  if(firstevent)
		  {
			firsttimestamp = (1E-6*(double)(msg->events[10].ts.toNSec()));
			firstevent = false;
		  }

		  double maxTs;
		  int posx, posy;
		  double usTime = 10.0;
		  double ts;



		  for (int i = beginEvent; i < min(beginEvent+packet, msg->events.size()); i++)
		  {
			  //SELECT SMALL PACKETS OF MAXIMUM 1000 events
			  const int x = msg->events[counterIn].x;
			  const int y = msg->events[counterIn].y;

			  double event_timestamp =  (1E-6*(double)(msg->events[counterIn].ts.toNSec()-first_timestamp));//now in usecs

			  ts = (1E-6*(double)(msg->events[i].ts.toNSec())) - firsttimestamp;
#if BG_FILTERING
			  //BGAFframe.at<float>(cv::Point(x,y))=(float)(1E-6*(double)(msg->events[i].ts.toNSec()));
			  BGAFframe.at<float>(cv::Point(x,y))=0.;
			  maxTs = -1;
			  //calculate maximum in a 3x3 neighborhood
			  for(int ii=-1; ii<=1; ii++)
				  for(int jj=-1; jj<=1; jj++)
				  {
					  posx = x + ii;
					  posy = y + jj;
					  if(posx<0)
						  posx = 0;
					  if(posy<0)
						  posy=0;
					  if(posx>numRows-1)
						  posx = numRows-1;
					  if(posy>numCols-1)
						  posy = numCols-1;

					  if(BGAFframe.at<float>(cv::Point(posx,posy)) > maxTs)
						  maxTs = BGAFframe.at<float>(cv::Point(posx,posy));
				  }

			  BGAFframe.at<float>(cv::Point(x,y))=ts;
			  //if nothing happened in usTime, remove the event
			  if(BGAFframe.at<float>(cv::Point(x,y)) >= (maxTs + usTime))
			  {
				  activeEvents.at(counterIn)=false;
				  counterIn++;
			  }
			  else
			  {
#endif
				  data.at<double>(cv::Point(counterOut, 0))= (double)x/numCols;
				  data.at<double>(cv::Point(counterOut, 1))= (double)y/numRows;
				  //data.at<double>(cv::Point(counter, 2))= event_timestamp/final_timestamp;//normalized
				  double tau = 10000;
				  data.at<double>(cv::Point(counterOut, 2))= exp(-(final_timestamp-event_timestamp)/tau);//normalized
				  activeEvents.at(counterIn)=true;
				  counterIn++;
				  counterOut++;
#if BG_FILTERING
			  }
#endif



			  //foutX<<x<<"\t"; foutY<<y<<"\t"; foutTime<<event_timestamp<<"\t";
		  }
		  double last_timestamp =  (1E-6*(double)(msg->events[counterIn-1].ts.toNSec()));//now in usecs
		  //foutX.close(); foutY.close(); foutTime.close();

		  cv::Mat clusterCenters;
		  cv::Mat segmentation=cv::Mat(numRows, numCols, CV_8UC3);
		  segmentation = cv::Scalar(128,128,128);

		  cv::Mat traj = cv::Mat(numRows, numCols, CV_8UC3);
		  traj = cv::Scalar(128,128,128);

		  std::vector<double> clustCentX, clustCentY, clustCentZ;
		  std::vector<int> point2Clusters;
		  std::vector<int> positionClusterColor;
		  std::vector<int> assign_matches;


		  //double bandwidth = 0.05;
		  double bandwidth = 0.15;
		  //double bandwidth = 0.15;
		  //double bandwidth = 0.20;
		  //double bandwidth = 0.25;
		  //double bandwidth = 0.50;
		  //cv::RNG rng(12345);

/*		  if(counterGlobal==0)
		  {
			  //clock_t start, end;
			  //double elapsed;
			  //start = clock();
			  //clustCentZ_old = clustCentZ;

			  meanshiftCluster_Gaussian(data, &clustCentX, &clustCentY, &clustCentZ, &point2Clusters, bandwidth);

			  assignClusterColor(&positionClusterColor, clustCentX.size(), assign_matches, prev_positionClusterColor); //assign new colors to clusters

			  prev_clustCentX = clustCentX;
			  prev_clustCentY = clustCentY;
			  prev_positionClusterColor = positionClusterColor;
			  //std::vector<int> checkvector (clustCentX.size(), -1);
			  //ROS_ERROR_STREAM("Total number of clusters is "<<clustCentX.size());


			  //end = clock();
			  //elapsed = ((double) (end - start)) / CLOCKS_PER_SEC;
			  //ROS_ERROR_STREAM("Num. packets = "<<data.cols<<" elapsed time = "<<elapsed<<". Time/packet = "<<elapsed/data.cols);

			  //Moved to initialization
			  //Now, draw the different segments with a color
			  //for(int i=0; i<clustCentX.size(); i++)
			  //	 RGBColors.push_back(cv::Vec3b((uchar)random(), (uchar)random(), (uchar)random()));

			  counter =0;
			  for (int i = beginEvent; i < min(beginEvent+packet, msg->events.size()); i++)
			  {
				  const int x = msg->events[counter].x;
				  const int y = msg->events[counter].y;
				  double ts =  (1E-6*(double)(msg->events[counter].ts.toNSec()-first_timestamp));//now in usecs

				  //if(ts<15)
				  //{
					  //segmentation.at<cv::Vec3b>(cv::Point(x,y))=RGBColors[point2Clusters[counter]];
					  //segmentation.at<cv::Vec3b>(cv::Point(x,y))=RGBColors[(positionClusterColor[point2Clusters[counter]])%MAX_NUM_CLUSTERS];
					  segmentation.at<cv::Vec3b>(cv::Point(x,y))=RGBColors[(positionClusterColor[point2Clusters[counter]])&MAX_NUM_CLUSTERS]; //cheaper than % (mod operation)
				  //}
				  //else
				  //	  break;
				  counter++;
			  }
		  }
		  else
		  {

*/

		  	  //fouttotalproc<<min(packet, msg->events.size()-beginEvent)<<std::endl;

			  meanshiftCluster_Gaussian(data, &clustCentX, &clustCentY, &clustCentZ, &point2Clusters, bandwidth);

			  //Assign new color or use color from previous clusters?
			  //if(counterGlobal>0)//not the first time
			  findClosestCluster(&assign_matches, clustCentX, clustCentY, prev_clustCentX, prev_clustCentY); //no match when clustID is -1

			  //assignClusterColor(&positionClusterColor, clustCentX.size(), assign_matches, prev_positionClusterColor); //assign new colors to clusters
			  assignClusterColor(&positionClusterColor, clustCentX.size(), assign_matches, prev_positionClusterColor, prev_activeTrajectories); //assign new colors to clusters

			  //foutnumclust<<clustCentX.size()<<std::endl;

			  int tmpColorPos;
			  float estimClustCenterX=-1., estimClustCenterY=-1.;
			  //std::vector<int> activeTrajectories(MAX_NUM_CLUSTERS,0);
			  std::vector<int> activeTrajectories;

			  if(point2Clusters.size()>400) //update positions only when there is enough change (>25%)
			  {
				  for(int i=0; i<clustCentX.size(); i++)
				  {
					  estimClustCenterX=-1., estimClustCenterY=-1.;

					  tmpColorPos = (positionClusterColor[i])&(MAX_NUM_CLUSTERS-1);

					  //Check if the new position is being used
					  /*int idx = 0;
					  for (int idx=0; idx<prev_activeTrajectories.size(); idx++)
						  if(tmpColorPos == prev_activeTrajectories[idx])
							  break;*/
					  std::vector<int>::iterator it;
					  it = find (prev_activeTrajectories.begin(), prev_activeTrajectories.end(), tmpColorPos);

					  if(it==prev_activeTrajectories.end())//if the element tmpcolorpos is not found in the vector
					  {
						  						  //Initialize the trajectory (just the counter)
						  //allTrajectories[tmpColorPos] = std::vector<cv::Point>(MAX_NUM_TRAJECTORY_POINTS);
						  counterTrajectories[tmpColorPos]=0;

#if KALMAN_FILTERING
						  foundTrajectory[tmpColorPos] = false;
						  createKalmanFilter(&(vector_of_kf[tmpColorPos]), &(vector_of_state[tmpColorPos]), &(vector_of_meas[tmpColorPos]));
						  vector_of_foundBlobs[tmpColorPos]=false;
						  vector_of_ticks[tmpColorPos]=0.0;
#endif
					  }

					  /*
					  if(prev_activeTrajectories[tmpColorPos]==0) //TODO: when the cluster is tracked all the time, it is active when I go back with &MAX_NUM_CLUSTERS
					  {
						  //Initialize the trajectory (just the counter)
						  //allTrajectories[tmpColorPos] = std::vector<cv::Point>(MAX_NUM_TRAJECTORY_POINTS);
						  counterTrajectories[tmpColorPos]=0;
					  }*/

#if KALMAN_FILTERING
					  if(counterTrajectories[tmpColorPos]>25 & !foundTrajectory[tmpColorPos])
					  {
						  //selectedTrajectory = tmpColorPos;
						  foundTrajectory[tmpColorPos] = true;
					  }
					  //if(foundTrajectory[tmpColorPos] & selectedTrajectory == tmpColorPos) //More than 25 points in the trajectory, start w/ KF
					  if(foundTrajectory[tmpColorPos]) //More than 25 points in the trajectory, start w/ KF
					  {
							//double precTick = ticks;
							//ticks = (double) cv::getTickCount();
							//double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds

						    /* In case we use only 1 kalman filter
						    if((last_timestamp - ticks) > 1)
						    {
						    	double precTick = ticks;
								ticks = last_timestamp;
								double dT = (ticks - precTick)/1000; //seconds

								if (foundBlobs)
								{
									// >>>> Matrix A
									kf.transitionMatrix.at<float>(2) = dT;
									kf.transitionMatrix.at<float>(7) = dT;
									// <<<< Matrix A
									state = kf.predict();
									estimClustCenterX = state.at<float>(0);
									estimClustCenterY = state.at<float>(1);
								}

								meas.at<float>(0) = clustCentX[i]*DVSW;
								meas.at<float>(1) = clustCentY[i]*DVSH; //[z_x, z_y]

								if (!foundBlobs) // First detection!
								{
									// >>>> Initialization
									kf.errorCovPre.at<float>(0) = 1; // px
									kf.errorCovPre.at<float>(5) = 1; // px
									kf.errorCovPre.at<float>(10) = 2;
									kf.errorCovPre.at<float>(15) = 2;

									state.at<float>(0) = meas.at<float>(0);
									state.at<float>(1) = meas.at<float>(1);
									state.at<float>(2) = 0;
									state.at<float>(3) = 0; //[z_x, z_y, v_x, v_y]
									// <<<< Initialization

									kf.statePost = state;

									foundBlobs = true;
							    }
							    else
								   kf.correct(meas); // Kalman Correction

								if(estimClustCenterX>=0.) //initialized to -1
								{
									//ROS_ERROR_STREAM("Estimated point = ["<<estimClustCenterX<<", "<<estimClustCenterY<<"];");
									//ROS_ERROR_STREAM("Measured point = ["<<clustCentX[i]<<", "<<clustCentY[i]<<"];");
									cv::circle(trajectories, cv::Point(clustCentX[i]*DVSW, clustCentY[i]*DVSH), 2, cv::Scalar( 0, 0, 255 ),-1,8);
									cv::circle(trajectories, cv::Point(estimClustCenterX, estimClustCenterY), 2, cv::Scalar( 255, 0, 0),-1,8);
									foutX<<clustCentX[i]*DVSW<<" "<<clustCentY[i]*DVSH<<std::endl;
									foutX_estim<<estimClustCenterX<<" "<<estimClustCenterY<<std::endl;
									foutT<<dT<<std::endl;
								}
						    }
						    */

						  if((last_timestamp - vector_of_ticks[tmpColorPos]) > 1)
							{
								double precTick = vector_of_ticks[tmpColorPos];
								vector_of_ticks[tmpColorPos] = last_timestamp;
								double dT = (vector_of_ticks[tmpColorPos] - precTick)/1000; //seconds
								//if (foundBlobs)
								if(vector_of_foundBlobs[tmpColorPos])
								{
									// >>>> Matrix A
									vector_of_kf[tmpColorPos].transitionMatrix.at<float>(2) = dT;
									vector_of_kf[tmpColorPos].transitionMatrix.at<float>(7) = dT;
									// <<<< Matrix A
									vector_of_state[tmpColorPos] = vector_of_kf[tmpColorPos].predict();
									estimClustCenterX = vector_of_state[tmpColorPos].at<float>(0);
									estimClustCenterY = vector_of_state[tmpColorPos].at<float>(1);
								}
								vector_of_meas[tmpColorPos].at<float>(0) = clustCentX[i]*DVSW;
								vector_of_meas[tmpColorPos].at<float>(1) = clustCentY[i]*DVSH; //[z_x, z_y]


								//if (!foundBlobs) // First detection!
								if(!vector_of_foundBlobs[tmpColorPos])
								{
									// >>>> Initialization
									vector_of_kf[tmpColorPos].errorCovPre.at<float>(0) = 1; // px
									vector_of_kf[tmpColorPos].errorCovPre.at<float>(5) = 1; // px
									vector_of_kf[tmpColorPos].errorCovPre.at<float>(10) = 2;
									vector_of_kf[tmpColorPos].errorCovPre.at<float>(15) = 2;

									vector_of_state[tmpColorPos].at<float>(0) = vector_of_meas[tmpColorPos].at<float>(0);
									vector_of_state[tmpColorPos].at<float>(1) = vector_of_meas[tmpColorPos].at<float>(1);
									vector_of_state[tmpColorPos].at<float>(2) = 0;
									vector_of_state[tmpColorPos].at<float>(3) = 0; //[z_x, z_y, v_x, v_y]
									// <<<< Initialization

									vector_of_kf[tmpColorPos].statePost = vector_of_state[tmpColorPos];

									//foundBlobs = true;
									vector_of_foundBlobs[tmpColorPos]=true;
								}
								else
									vector_of_kf[tmpColorPos].correct(vector_of_meas[tmpColorPos]); // Kalman Correction

								if(estimClustCenterX>=0.) //initialized to -1
								{
									//ROS_ERROR_STREAM("Estimated difference = ["<<abs(estimClustCenterX/DVSW -clustCentX[i]) <<", "<<abs(estimClustCenterY/DVSH-clustCentY[i])<<"];");
									//ROS_ERROR_STREAM("Estimated point = ["<<estimClustCenterX<<", "<<estimClustCenterY<<"];");
									//ROS_ERROR_STREAM("Measured point = ["<<clustCentX[i]<<", "<<clustCentY[i]<<"];");
									//cv::circle(trajectories, cv::Point(clustCentX[i]*DVSW, clustCentY[i]*DVSH), 2, cv::Scalar( 0, 0, 255 ),-1,8);
									//cv::circle(trajectories, cv::Point(estimClustCenterX, estimClustCenterY), 2, cv::Scalar( 255, 0, 0),-1,8);

									//foutX<<tmpColorPos<<" "<<clustCentX[i]*DVSW<<" "<<clustCentY[i]*DVSH<<" "<<estimClustCenterX<<" "<<estimClustCenterY<<std::endl;

									//foutX_estim<<estimClustCenterX<<" "<<estimClustCenterY<<std::endl;
									//foutT<<dT<<std::endl;
									//foutX<<"Estimated difference = ["<<abs(estimClustCenterX/DVSW -clustCentX[i]) <<", "<<abs(estimClustCenterY/DVSH-clustCentY[i])<<"];"<<std::endl;

									// !!!!!!CAREFUL ****************************************************************************************
									//ACTIVATE THIS !!!!!
									//clustCentX[i] = estimClustCenterX/DVSW;
									//clustCentY[i] = estimClustCenterY/DVSH;
									// CAREFUL ****************************************************************************************
								}
							}
					  }
#endif

					  cv::Point end(clustCentX[i]*DVSW, clustCentY[i]*DVSH);
/*
#if KALMAN_FILTERING
					  //frame, event, colorCluster, X,Y, filtX, filtY
					  foutX<<counterGlobal+1<<" "<<(double)(1E-6*(msg->events[beginEvent].ts.toNSec()-msg->events[0].ts.toNSec()))<<" " <<tmpColorPos<<" "<<clustCentX[i]*DVSW<<" "<<clustCentY[i]*DVSH<<" "<<estimClustCenterX<<" "<<estimClustCenterY<<std::endl;
#endif
*/

					  allTrajectories[tmpColorPos][(counterTrajectories[tmpColorPos]) & (MAX_NUM_TRAJECTORY_POINTS-1)]=end;
					  counterTrajectories[tmpColorPos]++;
					  //activeTrajectories[tmpColorPos]=1;
					  activeTrajectories.push_back(tmpColorPos);
				  }

				  prev_clustCentX = clustCentX;
				  prev_clustCentY = clustCentY;
				  prev_positionClusterColor = positionClusterColor;
				  prev_activeTrajectories = activeTrajectories;

				  trajectories = cv::Scalar(128,128,128);
				  int first=1;
				  for(int i=0; i<activeTrajectories.size(); i++)
				  {
					int tmpval = activeTrajectories[i];
					if (counterTrajectories[tmpval]>1)
					{
						if(counterTrajectories[tmpval]<=MAX_NUM_TRAJECTORY_POINTS)
						{
							for(int j=1; j<counterTrajectories[tmpval]; j++) //instead of % I use &(power of 2 -1): it is more efficient
								cv::line( trajectories, allTrajectories[tmpval][j-1], allTrajectories[tmpval][j], cv::Scalar( RGBColors[tmpval].val[0], RGBColors[tmpval].val[1], RGBColors[tmpval].val[2]), 2, 1);
						}
						else
						{
							int j=1;
							int pos = counterTrajectories[tmpval];
							while(j<MAX_NUM_TRAJECTORY_POINTS)
							{
								cv::line( trajectories, allTrajectories[tmpval][pos & (MAX_NUM_TRAJECTORY_POINTS-1)], allTrajectories[tmpval][(pos+1)&(MAX_NUM_TRAJECTORY_POINTS-1)], cv::Scalar( RGBColors[tmpval].val[0], RGBColors[tmpval].val[1], RGBColors[tmpval].val[2]), 2, 1);
								pos = pos+1;
								j++;
							}
						}
					}

				  }
			  }

			  trajectories.copyTo(segmentation); //Always keep trajectories

/*
#if TEST
			  char filename[80];
			  sprintf(filename,"/home/fran/0_tmp/kk/RGB_%0d.png", counterGlobal+1);
			  cv::imwrite(filename , segmentation);

			  sprintf(filename,"/home/fran/0_tmp/kk/RGB_%0d.txt", counterGlobal+1);
			  std::ofstream foutallT(filename);

			  for(int k=0; k<activeTrajectories.size(); k++)
			  	if(counterTrajectories[activeTrajectories[k]]>0)
				{
					foutallT<<"ClusterID "<<activeTrajectories[k]<<" with "<<counterTrajectories[activeTrajectories[k]]<<" elems: = [";
					for(int j=0; j<counterTrajectories[activeTrajectories[k]]; j++) //instead of % I use &(power of 2 -1): it is more efficient
					{
						foutallT<<"("<<allTrajectories[activeTrajectories[k]][j].x<<", "<<allTrajectories[activeTrajectories[k]][j].y<<"), ";
					}
						foutallT<<"];\n";
				}

			  foutallT.close();
#endif
*/

#if TEST
			  char filename1[80];
			  sprintf(filename1,"/home/fran/0_tmp/kk/rgb_%0d.txt", counterGlobal+1);
			  std::ofstream foutX(filename1);
#endif
			  counterIn =0;
			  counterOut=0;
			  for (int i = beginEvent; i < min(beginEvent+packet, msg->events.size()); i++)
			  {
				  if(activeEvents.at(counterIn)) //Label it only if it is not noise
				  {
					  const int x = msg->events[counterIn].x;
					  const int y = msg->events[counterIn].y;
					  double ts =  (1E-6*(double)(msg->events[counterIn].ts.toNSec()-first_timestamp));//now in usecs

					  //if(ts<15) //This is just for painting, we are processing all events
					  //{
						  //segmentation.at<cv::Vec3b>(cv::Point(x,y))=RGBColors[(positionClusterColor[point2Clusters[counter]])%MAX_NUM_CLUSTERS];
						  segmentation.at<cv::Vec3b>(cv::Point(x,y))=RGBColors[(positionClusterColor[point2Clusters[counterOut]])&(MAX_NUM_CLUSTERS-1)]; //cheaper than % (mod operation)
	#if TEST
						  //foutX<<x<<" "<<y<<" "<<ts<<" "<<msg->events[counter].polarity<<'\n';
						  foutX<<x<<'\t'<<'\t'<<y<<'\t'<<ts<<'\t'<<(int)(msg->events[counter].polarity)<<'\t'<<(int)((positionClusterColor[point2Clusters[counter]])&(MAX_NUM_CLUSTERS-1))<<'\n';
	#endif
						  counterOut++;
				  }
				  //}
				  //else
				  //{
				  //	  break;
				  //}
				  counterIn++;
			  }


			  char filename2[80];
			  sprintf(filename2,"/home/fran/0_tmp/kk/rgb_%0d.png", counterGlobal+1);
			  cv::imwrite(filename2, segmentation);


//		  }


		  cv_segments.encoding = "bgr8";
		  cv_segments.image = segmentation;
		  image_segmentation_pub_.publish(cv_segments.toImageMsg());

		  //std::cin.ignore();

		  beginEvent +=packet;

//#if TEST
		  //char filename[80];
		  //sprintf(filename,"/home/fran/0_tmp/kk/rgb_%0d.jpg", counterGlobal+1);
		  //cv::imwrite(filename , segmentation);
		  //sprintf(filename,"/home/fran/0_tmp/kk/cc_%0d.txt", counterGlobal+1);
		  //std::ofstream foutcc(filename);
		  //for (int i = 0; i<clustCentX.size(); i++)
		  //	  foutcc<<"ClusterCenter("<<i+1<<",:)=["<<clustCentX.at(i)*DVSW<<","<<clustCentY.at(i)*DVSH<<"];"<<std::endl;
		  //foutcc.close();
//#endif
		  //Saving RGB image
/*
			if (counterGlobal>0)
			{
				//std::cin.ignore();
				//for (int i = 0; i<clustCentX.size(); i++)
				//	ROS_ERROR_STREAM("ClusterCenter("<<i+1<<",:)=["<<clustCentX.at(i)<<","<<clustCentY.at(i)<<","<<clustCentZ.at(i)<<"];");

				//std::cout<<"clustperevent=["<<point2Clusters[0];
				//for (int i = 1; i<point2Clusters.size(); i++)
				//	std::cout<<","<<point2Clusters[i];
				//std::cout<<"];"<<std::endl;

				char filename[80];
				sprintf(filename,"/home/fran/0_tmp/kk/RGB_%0d.png", counterGlobal+1);
				cv::imwrite(filename , segmentation);
				//exit(0);

				//char filename[80];
				//sprintf(filename,"/home/fran/0_tmp/kk/RGB_%0d.txt", counterGlobal+1);
				//std::ofstream foutX(filename);
				//foutX<<counter<<"\n";
				//foutX.close();
			}
*/


		  /*if(msg->events.size()>15e3)
		  {
			  char filename[80];
			  sprintf(filename,"/home/fran/0_tmp/kk/RGB_%0d.png", counterGlobal+1);
			  cv::imwrite(filename , segmentation);
			  exit(0);
		  }*/
		  counterGlobal++;
	  }
  }
}




} // namespace
