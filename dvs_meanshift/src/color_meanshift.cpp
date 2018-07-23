#include <cstring>
#include <cmath>
#include <cstdlib>
#include "dvs_meanshift/color_meanshift.h"

#define max(x,y) ((x)>(y)?(x):(y))
#define min(x,y) ((x)<(y)?(x):(y))

#define _2D_to_linear(row, col, maxCol) ((row)*(maxCol)+(col))
//#define _3D_to_linear(row, col, color, maxRow, maxCol) ((maxRow)*(maxCol)*(color)+(row)*(maxCol)+(col))

//#define _2D_to_linear(row, col, maxRow) ((col)*(maxRow)+(row))
//#define _3D_to_linear(row, col, color, maxRow, maxCol) ((maxRow)*(maxCol)*(color)+(col)*(maxRow)+(row))
//void colorMeanShiftFilt_helper_C(int nlhs, mxArray *plhs[], int nrhs, const mxArray  *prhs[] )

/*void colorMeanShiftFilt(double *newPixelsPtr, const double *pixelsPtr, int mPixels, int maxPixelDim, const double *imagePtr, int numRows, int numCols, float spaceDivider, char *kernelFun, int maxIterNum, float tolFun, double *newImagePtr)
{
	// Copy the original pixel values to the new pixel matrix
	memcpy(newPixelsPtr, pixelsPtr, maxPixelDim*mPixels*sizeof(double));

	// Copy the original image to a new image
	memcpy(newImagePtr, imagePtr, mPixels*sizeof(double));

	// Help variables
	double  x,y;
	int     rX, rY;
	int     neighX, neighY;
	bool    bRepeat=false;

	double  *delta      = (double *) calloc(maxPixelDim,sizeof(double));
	double  *dX         = (double *) calloc(maxPixelDim,sizeof(double));
	double  *neighVec   = (double *) calloc(maxPixelDim,sizeof(double));
	double  deltaGrad2;

	// Create an array that indicates if we reached the end of the optimization for each pixel
	double *optimizedPointArray=new double[mPixels];

	for (int iterNum=0; iterNum<maxIterNum; iterNum++)
	{
		bRepeat=false;

		for (int pointNum=0; pointNum<mPixels; pointNum++)
		{
			if (optimizedPointArray[pointNum]==1) // The optimization for this pixel has reached the threshold
				continue;

			x=newPixelsPtr[_2D_to_linear(0, pointNum, maxPixelDim)]*spaceDivider;
			y=newPixelsPtr[_2D_to_linear(1, pointNum, maxPixelDim)]*spaceDivider;

			// Reset the delta variables
			delta[0]=0; delta[1]=0; delta[2]=0;
			dX[0]=0;    dX[1]=0;    dX[2]=0;

			//ROS_ERROR("y = %d, x = %d, pointNum = %d, spaceDivider = %g", (int)floor(y), (int)floor(x), pointNum, spaceDivider);

			double sumF=0;
			double dist2;


			// Find the neighbors around point (rX,rY)
			//for (neighX=max(0, floor(x)-1-spaceDivider); neighX<min(numCols, ceil(x)+spaceDivider); neighX++)
			for (neighX=max(0, floor(x)-spaceDivider); neighX<min(numCols, ceil(x)+spaceDivider); neighX++)
			{
				//for (neighY=max(0,floor(y)-1-spaceDivider); neighY<min(numRows, ceil(y)+spaceDivider); neighY++)
				for (neighY=max(0,floor(y)-spaceDivider); neighY<min(numRows, ceil(y)+spaceDivider); neighY++)
				{
					neighVec[0]=(neighX)/spaceDivider;
					neighVec[1]=(neighY)/spaceDivider;
					neighVec[2]=newImagePtr[neighY*numCols+neighX];


					delta[0]=newPixelsPtr[_2D_to_linear(0,pointNum,maxPixelDim)]-neighVec[0];
					delta[1]=newPixelsPtr[_2D_to_linear(1,pointNum,maxPixelDim)]-neighVec[1];
					delta[2]=newPixelsPtr[_2D_to_linear(2,pointNum,maxPixelDim)]-neighVec[2];

					dist2 = delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2];

					double kres=(exp(-dist2/2.0));

					dX[0] += kres*neighVec[0];
					dX[1] += kres*neighVec[1];
					dX[2] += kres*neighVec[2];

					sumF+=kres;

					//ROS_ERROR("newImagePtr[%d, %d]= %g", (int)(neighX), (int)(neighY), neighVec[2]);
				}
			}

			deltaGrad2=0;


			dX[0]=(sumF==0)?0:dX[0]/sumF; //check for NaNs
			dX[1]=(sumF==0)?0:dX[1]/sumF; //check for NaNs
			dX[2]=(sumF==0)?0:dX[2]/sumF; //check for NaNs

			double tmp0=newPixelsPtr[_2D_to_linear(0, pointNum, maxPixelDim)]-dX[0];
			double tmp1=newPixelsPtr[_2D_to_linear(1, pointNum, maxPixelDim)]-dX[1];
			double tmp2=newPixelsPtr[_2D_to_linear(2, pointNum, maxPixelDim)]-dX[2];

			//double tmp1=newPixelsPtr[_2D_to_linear(i, pointNum, mPixels)]-dX[i];
			deltaGrad2 = tmp0*tmp0 + tmp1*tmp1 + tmp2*tmp2;
			newPixelsPtr[_2D_to_linear(0, pointNum, maxPixelDim)]=dX[0];
			newPixelsPtr[_2D_to_linear(1, pointNum, maxPixelDim)]=dX[1];
			newPixelsPtr[_2D_to_linear(2, pointNum, maxPixelDim)]=dX[2];

			//ROS_ERROR("newImagePtr[%d, %d]= %g", (int)pointNum/numCols, (int)pointNum%numCols, dX[0]);

			// Find the corresponding row and column of the point
			newImagePtr[pointNum]=dX[2];

			//ROS_ERROR("newImagePtr[%d, %d] = %g", pointCol, pointRow, dX[0]);

			if (deltaGrad2<tolFun)
				optimizedPointArray[pointNum]=1;
			else
				bRepeat=true;

		} // end pointNum loop

		if (bRepeat==false)
			break;

	} // end for(iterNum) loop

	// CLEAN-UP
	//delete[] newImagePtr;
	delete[] optimizedPointArray;
	delete(delta);
	delete(dX);
	delete(neighVec);
}*/

/*void colorMeanShiftFilt(double *newPixelsPtr, const double *pixelsPtr, int mPixels, int maxPixelDim, const double *imagePtr, int numRows, int numCols, float spaceDivider, char *kernelFun, int maxIterNum, float tolFun)
{
	// Copy the original pixel values to the new pixel matrix
	memcpy(newPixelsPtr, pixelsPtr, maxPixelDim*mPixels*sizeof(double));

	// Copy the original image to a new image
	double *newImagePtr=new double[mPixels];
	memcpy(newImagePtr, imagePtr, mPixels*sizeof(double));

	// Help variables
	double  x,y;
	int     neighX, neighY;
	bool    bRepeat=false;

	double  *delta      = (double *) calloc(maxPixelDim,sizeof(double));
	double  *dX         = (double *) calloc(maxPixelDim,sizeof(double));
	double  *neighVec   = (double *) calloc(maxPixelDim,sizeof(double));
	double  deltaGrad2;

	// Create an array that indicates if we reached the end of the optimization for each pixel
	double *optimizedPointArray=new double[mPixels];

	for (int iterNum=0; iterNum<maxIterNum; iterNum++)
	{
		bRepeat=false;
		for (int pointNum=0; pointNum<mPixels; pointNum++)
		{
			if (optimizedPointArray[pointNum]==1) // The optimization for this pixel has reached the threshold
				continue;

			x=newPixelsPtr[_2D_to_linear(0, pointNum, mPixels)]*spaceDivider;
			y=newPixelsPtr[_2D_to_linear(1, pointNum, mPixels)]*spaceDivider;

			delta[0]=0; delta[1]=0; delta[2]=0;
			dX[0]=0; dX[1]=0; dX[2]=0;

			//ROS_ERROR("y = %d, x = %d, pointNum = %d, spaceDivider = %g", (int)floor(y), (int)floor(x), pointNum, spaceDivider);

			double sumF=0;
			double dist2, kres;

			// Find the neighbors around point (rX,rY)
			for (neighX=max(0, floor(x)-1-spaceDivider); neighX<min(numCols, ceil(x)+spaceDivider); neighX++)
			{
				for (neighY=max(0,floor(y)-1-spaceDivider); neighY<min(numRows, ceil(y)+spaceDivider); neighY++)
				{
					neighVec[0]=(neighX+1)/spaceDivider;
					neighVec[1]=(neighY+1)/spaceDivider;
					neighVec[2]=newImagePtr[neighY*numCols+neighX];

					delta[0]=newPixelsPtr[_2D_to_linear(0,pointNum,mPixels)]-neighVec[0];
					delta[1]=newPixelsPtr[_2D_to_linear(1,pointNum,mPixels)]-neighVec[1];
					delta[2]=newPixelsPtr[_2D_to_linear(2,pointNum,mPixels)]-neighVec[2];

					dist2 = delta[0]*delta[0]+delta[1]*delta[1]+delta[2]*delta[2];

					kres =(exp(-dist2/2.0));

					dX[0]+= kres*neighVec[0];
					dX[1]+= kres*neighVec[1];
					dX[2]+= kres*neighVec[2];
					sumF +=kres;
				}
			}

			// Find the corresponding row and column of the point
			int pointRow = pointNum/numCols;
			int pointCol = pointNum%numCols;

			dX[0]=(sumF==0)?0:dX[0]/sumF; //check for NaNs
			dX[1]=(sumF==0)?0:dX[1]/sumF; //check for NaNs
			dX[2]=(sumF==0)?0:dX[2]/sumF; //check for NaNs
			//double tmp1=newPixelsPtr[_2D_to_linear(i, pointNum, maxPixelDim)]-dX[i];
			double tmp1=newPixelsPtr[_2D_to_linear(0, pointNum, mPixels)]-dX[0];
			double tmp2=newPixelsPtr[_2D_to_linear(1, pointNum, mPixels)]-dX[1];
			double tmp3=newPixelsPtr[_2D_to_linear(2, pointNum, mPixels)]-dX[2];

			deltaGrad2=tmp1*tmp1+tmp2*tmp2+tmp3*tmp3;
			newPixelsPtr[_2D_to_linear(0, pointNum, mPixels)]=dX[0];
			newPixelsPtr[_2D_to_linear(1, pointNum, mPixels)]=dX[1];
			newPixelsPtr[_2D_to_linear(2, pointNum, mPixels)]=dX[2];
			//newImagePtr[pointRow*numCols+pointCol]=dX[2];
			newImagePtr[pointNum]=dX[2];

			if (deltaGrad2<tolFun)
				optimizedPointArray[pointNum]=1;
			else
				bRepeat=true;

		} // end pointNum loop

		if (bRepeat==false)
			break;

	} // end for(iterNum) loop


	// CLEAN-UP
	delete[] newImagePtr;
	delete[] optimizedPointArray;
	delete(delta);
	delete(dX);
	delete(neighVec);
}*/

/*void meanshiftCluster_Gaussian(cv::Mat dataPts, std::vector<double> *clusterCenterX, std::vector<double> *clusterCenterY, std::vector<double> *clusterCenterZ, std::vector<int> *point2Clusters, double bandwidth)
{
	//Initialization
	int numPts = dataPts.cols;
	int numDim = dataPts.rows;
	int numClust = 0;
	double bandSq = bandwidth*bandwidth;
	cv::Range rng = cv::Range(1,numPts);

	cv::Mat onesAux = cv::Mat::ones(1, dataPts.cols, dataPts.type());

	std::vector<int> initPtInds;
	for (int i = rng.start; i <= rng.end; i++) initPtInds.push_back(i);
	double stopThresh = 1E-3*bandwidth; 						//when mean has converged
	int numInitPts = numPts; 								//track if a points been seen already
	cv::Mat beenVisitedFlag = cv::Mat(1, numPts, CV_32S, cv::Scalar::all(0)); 	//number of points to posibaly use as initilization points
	cv::Mat clusterVotes; 		//used to resolve conflicts on cluster membership

	double lambda = 10.;
	double myMeanX, myMeanY, myMeanZ, myOldMeanX, myOldMeanY, myOldMeanZ, totalWeightX, totalWeightY, totalWeightZ;
	int stInd;

	//std::vector<double> clusterCenterX, clusterCenterY, clusterCenterZ;

	int tempInd;

	while (numInitPts>0)
	{
		tempInd = (rand()%numInitPts);								//pick a random seed point
		stInd = initPtInds[tempInd];								//use this point as start of mean
		myMeanX = dataPts.at<double>(cv::Point(stInd, 0));				//intilize mean to this points location
		myMeanY = dataPts.at<double>(cv::Point(stInd, 1));
		myMeanZ = dataPts.at<double>(cv::Point(stInd, 2));
		cv::Mat thisClusterVotes = cv::Mat(1, numPts, CV_32S, cv::Scalar::all(0)); 	//used to resolve conflicts on cluster membership
		//ROS_ERROR_STREAM("Before getting into while myMean = ["<<myMeanX<<", "<<myMeanY<<", "<<myMeanZ<<"]");
		cv::Mat_<double> myMean = ( cv::Mat_<double>(3, 1) << myMeanX, myMeanY, myMeanZ);



		while(true)
		{
			cv::Mat dataX, dataY, dataZ;
			dataX.push_back(dataPts.row(0));
			dataY.push_back(dataPts.row(1));
			dataZ.push_back(dataPts.row(2));

			cv::Mat diffX = dataX - myMeanX;
			cv::Mat diffY = dataY - myMeanY;
			cv::Mat diffZ = dataZ - myMeanZ;

			ROS_ERROR_STREAM("Original = "<<diffX);
			ROS_ERROR_STREAM("onesAux.rows = "<<onesAux.rows<<" cols = "<<onesAux.cols);
			ROS_ERROR_STREAM("dataPts.rows = "<<dataPts.rows<<" cols = "<<dataPts.cols);
			ROS_ERROR_STREAM("myMean.rows = "<<myMean.rows<<" cols = "<<myMean.cols);

			cv::Mat diff = dataPts - myMean*onesAux;
			diffX.push_back(diff.row(0));
			diffY.push_back(diff.row(1));
			diffZ.push_back(diff.row(2));

			ROS_ERROR_STREAM("Matricial "<<diff.row(0));
			exit(0);

			cv::Mat diffXX = diffX.mul(diffX);
			cv::Mat diffYY = diffY.mul(diffY);
			cv::Mat diffZZ = diffZ.mul(diffZ);

			cv::Mat sqDistToAll = diffXX + diffYY + diffZZ;												//dist squared from mean to all points still active
			cv::Mat inInds = (sqDistToAll < bandSq); 									// Puts 255 wherever it is true
			//ROS_ERROR_STREAM("This is the size "<<inInds.size());
			//ROS_ERROR_STREAM("This is the size "<<thisClusterVotes.size());
			//inInds.convertTo(inInds, CV_8UC1);
			//cv::add(thisClusterVotes, cv::Mat::ones(1, numPts, CV_32S), thisClusterVotes, inInds); 			//add a vote for all the in points belonging to this cluster
			cv::add(thisClusterVotes, cv::Scalar(1), thisClusterVotes, inInds); 			//add a vote for all the in points belonging to this cluster
			//ROS_ERROR_STREAM("thisclusterVotes = "<<inInds);
			//ROS_ERROR_STREAM("thisclusterVotes = "<<thisClusterVotes);
			//ROS_ERROR_STREAM("thisclusterVotes = "<<sqDistToAll);
			//exit(0);

			cv::Mat selectedDataX, selectedDataY, selectedDataZ;
			//cv::bitwise_and(dataX, cv::Scalar(255), selectedDataX, inInds);
			//cv::bitwise_and(dataY, cv::Scalar(255), selectedDataY, inInds);
			//cv::bitwise_and(dataZ, cv::Scalar(255), selectedDataZ, inInds);
			//ROS_ERROR_STREAM("Before: dataX ="<<dataX);
			dataX.setTo(0., inInds==0);
			dataY.setTo(0., inInds==0);
			dataZ.setTo(0., inInds==0);
			//ROS_ERROR_STREAM("inInds ="<<inInds);
			//ROS_ERROR_STREAM("After: dataX ="<<dataX);

			double numOfOnes = (double) (cv::sum(inInds)[0])/255; //inInds is active if inInds[i]==255
			//ROS_ERROR_STREAM("Num of Ones ="<<numOfOnes);
			//ROS_ERROR_STREAM("inInds ="<<inInds);
			//exit(0);
			myOldMeanX = myMeanX; myOldMeanY = myMeanY; myOldMeanZ = myMeanZ;
			myMeanX = cv::sum(dataX)[0]/numOfOnes;
			myMeanY = cv::sum(dataY)[0]/numOfOnes;
			myMeanZ = cv::sum(dataZ)[0]/numOfOnes;


			//ROS_ERROR_STREAM("inInds="<<inInds);
			//ROS_ERROR_STREAM("After: dataX="<<dataX);

			diffX = dataX - myMeanX;
			diffY = dataY - myMeanY;
			diffZ = dataZ - myMeanZ;

			diffXX = diffXX.mul(diffXX);
			diffYY = diffYY.mul(diffYY);
			diffZZ = diffZZ.mul(diffZZ);

			cv::Mat weightX, weightY, weightZ;
			cv::exp(diffXX/lambda, weightX);
			cv::exp(diffYY/lambda, weightY);
			cv::exp(diffZZ/lambda, weightZ);
			//cv::Mat wDataX = weightX.mul(selectedDataX);
			//cv::Mat wDataY = weightY.mul(selectedDataY);
			//cv::Mat wDataZ = weightZ.mul(selectedDataZ);

			weightX.setTo(0., inInds==0); //Again, because the exp and the x - u make invalid values non-zero (inInds)
			weightY.setTo(0., inInds==0);
			weightZ.setTo(0., inInds==0);

			cv::Mat wDataX = weightX.mul(dataX);
			cv::Mat wDataY = weightY.mul(dataY);
			cv::Mat wDataZ = weightZ.mul(dataZ);
			totalWeightX = cv::sum(weightX)[0];
			totalWeightY = cv::sum(weightY)[0];
			totalWeightZ = cv::sum(weightZ)[0];
			myMeanX = cv::sum(wDataX)[0]/totalWeightX;
			myMeanY = cv::sum(wDataY)[0]/totalWeightY;
			myMeanZ = cv::sum(wDataZ)[0]/totalWeightZ;

			//cv::add(beenVisitedFlag, cv::Scalar(1.f), beenVisitedFlag, inInds);
			beenVisitedFlag.setTo(1, inInds);
			//ROS_ERROR_STREAM("myMean = ["<<myMeanX<<", "<<myMeanY<<", "<<myMeanZ<<"]");
			//exit(0);
			// if mean doesn't move much stop this cluster
			if((myMeanX-myOldMeanX)*(myMeanX-myOldMeanX) + (myMeanY-myOldMeanY)*(myMeanY-myOldMeanY) + (myMeanZ-myOldMeanZ)*(myMeanZ-myOldMeanZ) < stopThresh)
			{
				//check for merge possibilities
				int mergeWith = -1;
				double distToOther;
				for(int cN = 0; cN<numClust; cN++) //Careful!! cN goes from 1 to numClust!!!
				{
					double distToOther = (myMeanX - (*clusterCenterX)[cN])*(myMeanX - (*clusterCenterX)[cN]) + (myMeanY - (*clusterCenterY)[cN])*(myMeanY - (*clusterCenterY)[cN]) + (myMeanZ - (*clusterCenterZ)[cN])*(myMeanZ - (*clusterCenterZ)[cN]); //distance from posible new clust max to old clust max
					//ROS_ERROR_STREAM("Distance to others "<<distToOther<<" and cN "<<cN);
					//ROS_ERROR_STREAM("myMean = ["<<myMeanX<<", "<<myMeanY<<", "<<myMeanZ<<"]");
					//ROS_ERROR_STREAM("clusterCenter = ["<<clusterCenterX[cN]<<", "<<clusterCenterY[cN]<<", "<<clusterCenterZ[cN]<<"]");
					if(distToOther < bandwidth/2)                    //if its within bandwidth/2 merge new and old
					{
						mergeWith = cN;
						break;
					}
				}

				if(mergeWith > -1)
				{
					(*clusterCenterX)[mergeWith] = 0.5*(myMeanX+(*clusterCenterX)[mergeWith]);
					(*clusterCenterY)[mergeWith] = 0.5*(myMeanY+(*clusterCenterY)[mergeWith]);
					(*clusterCenterZ)[mergeWith] = 0.5*(myMeanZ+(*clusterCenterZ)[mergeWith]);

					cv::Mat newClusterVotes = cv::Mat(clusterVotes.row(mergeWith)) + thisClusterVotes;
					newClusterVotes.copyTo(clusterVotes.row(mergeWith));
				}
				else
				{
					numClust                     = numClust+1;                   //increment clusters
					(*clusterCenterX).push_back(myMeanX);                       //record the mean
					(*clusterCenterY).push_back(myMeanY);
					(*clusterCenterZ).push_back(myMeanZ);

					clusterVotes.push_back(thisClusterVotes);                           // add the new row for the new cluster
				}
				break;
			}
		}

		std::vector<int> newInitPtInds;								//we can initialize with any of the points not yet visited
		for(int i=0; i<numPts; i++)
			if (beenVisitedFlag.at<int>(cv::Point(i, 0)) == 0)
				newInitPtInds.push_back(i);
		initPtInds = newInitPtInds;
		numInitPts = initPtInds.size();                   				//number of active points in set

		//ROS_ERROR_STREAM("Num. of values still un clustered "<<numInitPts);
		//ROS_ERROR_STREAM("beenVisitedFlag "<<beenVisitedFlag);
		//for(int i=0; i<initPtInds.size(); ++i)
		//  std::cerr<<initPtInds[i] << ' ';
	}
	//exit(0);
	//[val,data2cluster] = max(clusterVotes,[],1);                //a point belongs to the cluster with the most votes

	cv::Mat TclusterVotes = clusterVotes.t();

	double min, max;
	for(int i=0; i<TclusterVotes.rows; i++)
	{
		cv::minMaxIdx(cv::Mat(TclusterVotes.row(i)), &min, &max);
		(*point2Clusters).push_back((int)max);
	}

	//ROS_ERROR_STREAM("Inside "<<(*point2Clusters).size());
	//ROS_ERROR_STREAM("Number of clusters " <<numClust<<"and "<<clusterCenterX.size());
}*/



void meanshiftCluster_Gaussian(cv::Mat dataPts, std::vector<double> *clusterCenterX, std::vector<double> *clusterCenterY, std::vector<double> *clusterCenterZ, std::vector<int> *point2Clusters, double bandwidth)
{
	//Initialization
	int numPts = dataPts.cols;
	int numDim = dataPts.rows;
	int numClust = 0;
	double bandSq = bandwidth*bandwidth;
	cv::Range rng = cv::Range(0,numPts-1);

	cv::Mat onesAux = cv::Mat::ones(1, dataPts.cols, dataPts.type());

	std::vector<int> initPtInds;
	for (int i = rng.start; i <= rng.end; i++) initPtInds.push_back(i);
	double stopThresh = 1E-3*bandwidth; 						//when mean has converged
	int numInitPts = numPts; 								//track if a points been seen already
	cv::Mat beenVisitedFlag = cv::Mat(1, numPts, CV_32S, cv::Scalar::all(0)); 	//number of points to posibaly use as initilization points
	cv::Mat clusterVotes; 		//used to resolve conflicts on cluster membership

	double lambda = 10.;
	double myMeanX, myMeanY, myMeanZ, myOldMeanX, myOldMeanY, myOldMeanZ, totalWeightX, totalWeightY, totalWeightZ;
	int stInd;

	//std::vector<double> clusterCenterX, clusterCenterY, clusterCenterZ;


	int tempInd;

	while (numInitPts>0)
	{
		//ROS_ERROR_STREAM("iniPtInds");
		//for(int i=0; i<initPtInds.size(); i++)
		//	std::cerr<<initPtInds[i]<<" ";
		//std::cerr<<std::endl;


		tempInd = (rand()%numInitPts);								//pick a random seed point
		//ROS_ERROR_STREAM("numInitPts ="<<numInitPts);

		//std::cin.ignore();
		stInd = initPtInds[tempInd];								//use this point as start of mean
		/**********REMOVE THIS******************************/
		//stInd = initPtInds[numInitPts-1];
		/**********REMOVE THIS******************************/
		myMeanX = dataPts.at<double>(cv::Point(stInd, 0));				//intilize mean to this points location
		myMeanY = dataPts.at<double>(cv::Point(stInd, 1));
		myMeanZ = dataPts.at<double>(cv::Point(stInd, 2));
		cv::Mat thisClusterVotes = cv::Mat(1, numPts, CV_32S, cv::Scalar::all(0)); 	//used to resolve conflicts on cluster membership
		//ROS_ERROR_STREAM("Before getting into while myMean = ["<<myMeanX<<", "<<myMeanY<<", "<<myMeanZ<<"]");
		cv::Mat_<double> myMean = ( cv::Mat_<double>(3, 1) << myMeanX, myMeanY, myMeanZ);


		while(true)
		{
			cv::Mat myDataPts;
			dataPts.copyTo(myDataPts);

			//ROS_ERROR_STREAM("Original = "<<diffX);
			//ROS_ERROR_STREAM("onesAux.rows = "<<onesAux.rows<<" cols = "<<onesAux.cols);
			//ROS_ERROR_STREAM("dataPts.rows = "<<dataPts.rows<<" cols = "<<dataPts.cols);
			//ROS_ERROR_STREAM("myMean.rows = "<<myMean.rows<<" cols = "<<myMean.cols);

			cv::Mat diff = myDataPts - myMean*onesAux;
			cv::Mat diffdiff = diff.mul(diff);
			cv::Mat sqDistToAll = diffdiff.row(0) +diffdiff.row(1) + diffdiff.row(2);	//dist squared from mean to all points still active

			cv::Mat inInds = (sqDistToAll < bandSq); 									// Puts 255 wherever it is true
			cv::add(thisClusterVotes, cv::Scalar(1), thisClusterVotes, inInds); 			//add a vote for all the in points belonging to this cluster
			cv::Mat mask3 = cv::repeat(inInds==0, 3, 1);

			//ROS_ERROR_STREAM("BEFORE myMean = "<<myMean);
			//ROS_ERROR_STREAM("sqDistToAll = "<<sqDistToAll);
			//ROS_ERROR_STREAM("diffX = "<<diff.row(0));
			//ROS_ERROR_STREAM("diffdiffX = "<<diffdiff.row(0));
			//ROS_ERROR_STREAM("thisClusterVotes = "<<thisClusterVotes);
			//ROS_ERROR_STREAM("dataPts = "<<dataPts);

			myDataPts.setTo(0, mask3);
			double numOfOnes = (double) (cv::sum(inInds)[0])/255; //inInds is active if inInds[i]==255

			myOldMeanX = myMeanX; myOldMeanY = myMeanY; myOldMeanZ = myMeanZ;

			myMeanX = cv::sum(myDataPts.row(0))[0]/numOfOnes;
			myMeanY = cv::sum(myDataPts.row(1))[0]/numOfOnes;
			myMeanZ = cv::sum(myDataPts.row(2))[0]/numOfOnes;

			myMean = ( cv::Mat_<double>(3, 1) << myMeanX, myMeanY, myMeanZ);

			//ROS_ERROR_STREAM("MIDDLE myMean = "<<myMean);

			diff = myDataPts - myMean*onesAux;
			diffdiff = diff.mul(diff);

			cv::Mat weight;
			cv::exp(diffdiff/lambda, weight);

			weight.setTo(0., mask3); //Again, because the exp and the x - u make invalid values non-zero (inInds)
			cv::Mat wData = weight.mul(myDataPts);

			totalWeightX = cv::sum(weight.row(0))[0];
			totalWeightY = cv::sum(weight.row(1))[0];
			totalWeightZ = cv::sum(weight.row(2))[0];
			myMeanX = cv::sum(wData.row(0))[0]/totalWeightX;
			myMeanY = cv::sum(wData.row(1))[0]/totalWeightY;
			myMeanZ = cv::sum(wData.row(2))[0]/totalWeightZ;

			myMean = ( cv::Mat_<double>(3, 1) << myMeanX, myMeanY, myMeanZ);
			//ROS_ERROR_STREAM("AFTER: myMean = "<<myMeanX<<","<<myMeanY<<","<<myMeanZ);
			//exit(0);

			beenVisitedFlag.setTo(1, inInds);
			//ROS_ERROR_STREAM("beenVisitedFlag = "<<beenVisitedFlag);

			// if mean doesn't move much stop this cluster
			//ROS_ERROR_STREAM("Norm = "<<(myMeanX-myOldMeanX)*(myMeanX-myOldMeanX) + (myMeanY-myOldMeanY)*(myMeanY-myOldMeanY) + (myMeanZ-myOldMeanZ)*(myMeanZ-myOldMeanZ));
			if((myMeanX-myOldMeanX)*(myMeanX-myOldMeanX) + (myMeanY-myOldMeanY)*(myMeanY-myOldMeanY) + (myMeanZ-myOldMeanZ)*(myMeanZ-myOldMeanZ) < stopThresh*stopThresh)
			{
				//check for merge posibilities
				//ROS_ERROR_STREAM("Dentro!! ");
				int mergeWith = -1;
				double distToOther;
				for(int cN = 0; cN<numClust; cN++) //Careful!! cN goes from 1 to numClust!!!
				{
					double distToOther = (myMeanX - (*clusterCenterX)[cN])*(myMeanX - (*clusterCenterX)[cN]) + (myMeanY - (*clusterCenterY)[cN])*(myMeanY - (*clusterCenterY)[cN]) + (myMeanZ - (*clusterCenterZ)[cN])*(myMeanZ - (*clusterCenterZ)[cN]); //distance from posible new clust max to old clust max
					//ROS_ERROR_STREAM("Dentro!! ");
					//ROS_ERROR_STREAM("distToOther " <<distToOther);
					if(distToOther < (bandwidth/2)*(bandwidth/2))  //if its within bandwidth/2 merge new and old
					{
						mergeWith = cN;
						break;
					}
				}

				if(mergeWith > -1)
				{
					//ROS_ERROR_STREAM("Merging cluster with #"<<mergeWith);
					//ROS_ERROR_STREAM("NumClust = "<<numClust);
					//exit(0);

					(*clusterCenterX)[mergeWith] = 0.5*(myMeanX+(*clusterCenterX)[mergeWith]);
					(*clusterCenterY)[mergeWith] = 0.5*(myMeanY+(*clusterCenterY)[mergeWith]);
					(*clusterCenterZ)[mergeWith] = 0.5*(myMeanZ+(*clusterCenterZ)[mergeWith]);

					cv::Mat newClusterVotes = cv::Mat(clusterVotes.row(mergeWith)) + thisClusterVotes;
					newClusterVotes.copyTo(clusterVotes.row(mergeWith));

					//ROS_ERROR_STREAM("clusterVotes = "<<clusterVotes);
					//exit(0);
				}
				else
				{
					//ROS_ERROR_STREAM("Creating new cluster");
					(*clusterCenterX).push_back(myMeanX);                       //record the mean
					(*clusterCenterY).push_back(myMeanY);
					(*clusterCenterZ).push_back(myMeanZ);
					numClust = numClust+1;                   //increment clusters

					clusterVotes.push_back(thisClusterVotes);                           // add the new row for the new cluster

					//ROS_ERROR_STREAM("clusterVotes = "<<clusterVotes);
					//ROS_ERROR_STREAM("ClusterCenterX = "<<(*clusterCenterX)[numClust-1]);
					//ROS_ERROR_STREAM("ClusterCenterY = "<<(*clusterCenterY)[numClust-1]);
					//ROS_ERROR_STREAM("ClusterCenterZ = "<<(*clusterCenterZ)[numClust-1]);
					//exit(0);
				}
				break;
			}
		}

		std::vector<int> newInitPtInds;								//we can initialize with any of the points not yet visited
		for(int i=0; i<numPts; i++)
			if (beenVisitedFlag.at<int>(cv::Point(i, 0)) == 0)
				newInitPtInds.push_back(i);
		initPtInds = newInitPtInds;
		numInitPts = initPtInds.size();                   				//number of active points in set
	}

	cv::Mat TclusterVotes = clusterVotes.t();
	cv::Point minLoc;
	cv::Point maxLoc;
	double min, max;
	for(int i=0; i<TclusterVotes.rows; i++)
	{
		cv::minMaxLoc(cv::Mat(TclusterVotes.row(i)), &min, &max, &minLoc, &maxLoc);
		(*point2Clusters).push_back(maxLoc.x);
	}

	//ROS_ERROR_STREAM("Number of clusters " <<numClust<<" and "<<(*clusterCenterX).size());
	//ROS_ERROR_STREAM("Tclusters rows " <<TclusterVotes.rows<<" cols "<<TclusterVotes.cols);
}

void colorMeanShiftFilt(double *newPixelsPtr, const double *pixelsPtr, int mPixels, int maxPixelDim, const double *imagePtr, int numRows, int numCols, float spaceDivider, char *kernelFun, int maxIterNum, float tolFun)
{
	// Copy the original pixel values to the new pixel matrix
	memcpy(newPixelsPtr, pixelsPtr, maxPixelDim*mPixels*sizeof(double));

	// Copy the original image to a new image
	double *newImagePtr=new double[mPixels];
	memcpy(newImagePtr, imagePtr, mPixels*sizeof(double));

	// Help variables
	double  x,y,val;
	int     neighX, neighY;
	bool    bRepeat=false;

	double  *delta      = (double *) calloc(maxPixelDim,sizeof(double));
	double  *dX         = (double *) calloc(maxPixelDim,sizeof(double));
	double  *neighVec   = (double *) calloc(maxPixelDim,sizeof(double));
	double  deltaGrad2;

	// Create an array that indicates if we reached the end of the optimization for each pixel
	double *optimizedPointArray=new double[mPixels];

	for (int iterNum=0; iterNum<maxIterNum; iterNum++)
	{
		bRepeat=false;

		cv::Mat XX=cv::Mat(numRows, numCols, CV_64F, newPixelsPtr);
		cv::Mat YY=cv::Mat(numRows, numCols, CV_64F, newPixelsPtr+mPixels);
		cv::Mat MM=cv::Mat(numRows, numCols, CV_64F, newPixelsPtr+2*mPixels);

		for (int pointNum=0; pointNum<mPixels; pointNum++)
		{
			if (optimizedPointArray[pointNum]==1) // The optimization for this pixel has reached the threshold
				continue;

			x=newPixelsPtr[_2D_to_linear(0, pointNum, mPixels)];
			y=newPixelsPtr[_2D_to_linear(1, pointNum, mPixels)];
			val=newPixelsPtr[_2D_to_linear(2, pointNum, mPixels)];

			//ROS_ERROR("y = %d, x = %d, pointNum = %d, spaceDivider = %g", (int)floor(y), (int)floor(x), pointNum, spaceDivider);
			double sumF;

			int x_coord = max(0, floor(x*spaceDivider)-1-spaceDivider);
			int y_coord = max(0, floor(y*spaceDivider)-1-spaceDivider);
			int width = min(2*spaceDivider+1, numCols-x_coord);
			int height= min(2*spaceDivider+1, numRows-y_coord);

			cv::Mat submatXX = XX(cv::Rect(x_coord, y_coord, width, height)).clone();
			cv::Mat submatYY = YY(cv::Rect(x_coord, y_coord, width, height)).clone();
			cv::Mat submatMM = MM(cv::Rect(x_coord, y_coord, width, height)).clone();

			cv::Mat delta_XX, delta_YY, delta_MM, dist2_matrix, kres_matrix;
			delta_XX = x - submatXX;
			delta_YY = y - submatYY;
			delta_MM = val - submatMM;
			dist2_matrix = delta_XX.mul(delta_XX) + delta_YY.mul(delta_YY) + delta_MM.mul(delta_MM);
			cv::exp(dist2_matrix/(-2.0), kres_matrix); //kres =(exp(-dist2/2.0));

			dX[0] = cv::sum(kres_matrix.mul(submatXX))[0];//cv::sum() returns a cvScalar, to get the value we need .val(0)
			dX[1] = cv::sum(kres_matrix.mul(submatYY))[0];
			dX[2] = cv::sum(kres_matrix.mul(submatMM))[0];
			sumF = cv::sum(kres_matrix)[0];

			// Find the corresponding row and column of the point

			dX[0]=(sumF==0)?0:dX[0]/sumF; //check for NaNs
			dX[1]=(sumF==0)?0:dX[1]/sumF; //check for NaNs
			dX[2]=(sumF==0)?0:dX[2]/sumF; //check for NaNs

			double tmp1=newPixelsPtr[_2D_to_linear(0, pointNum, mPixels)]-dX[0];
			double tmp2=newPixelsPtr[_2D_to_linear(1, pointNum, mPixels)]-dX[1];
			double tmp3=newPixelsPtr[_2D_to_linear(2, pointNum, mPixels)]-dX[2];

			deltaGrad2=tmp1*tmp1+tmp2*tmp2+tmp3*tmp3;
			newPixelsPtr[_2D_to_linear(0, pointNum, mPixels)]=dX[0];
			newPixelsPtr[_2D_to_linear(1, pointNum, mPixels)]=dX[1];
			newPixelsPtr[_2D_to_linear(2, pointNum, mPixels)]=dX[2];

			newImagePtr[pointNum]=dX[2];

			if (deltaGrad2<tolFun)
				optimizedPointArray[pointNum]=1;
			else
				bRepeat=true;

		} // end pointNum loop

		if (bRepeat==false)
			break;

	} // end for(iterNum) loop


	// CLEAN-UP
	delete[] newImagePtr;
	delete[] optimizedPointArray;
	delete(delta);
	delete(dX);
	delete(neighVec);
}



