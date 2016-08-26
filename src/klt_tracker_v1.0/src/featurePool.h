/**
* CMU MSUAV Project (Micro and Small Unmanned Aerial Vehicle)
*          
* \file  featurePool.h
* \brief header file for featurePool.cxx (CFeature2DPool for CPU version)
* \date  15-Jun-2007
*
* \author Myung Hwangbo (myung@cs.cmu.edu)
*
* Copyright (c) 2007-2009 Myung Hwangbo
* Robotics Institute, Carnegie Mellon University
*
*/

#ifndef __FEATURE_POOL_H__
#define __FEATURE_POOL_H__

#define	__GPU_DEFINED
//#define LARGE_INTEGER long int
 
#include <iostream>
#include <iomanip> 
#include <fstream>

#include <vector>
#include <list>
#include <map>
#include <set>

#include <cv.h>

//#include "ctimer.h"
#include "featureTrack.h"
#include "imageAlign.h"
#include "cameraModel.h"

#define MAX_NUM_FEATURES	1024

using namespace std;

class CImageAlign;
class CImagePyramid;

////////////////////////////////////////////////////////////////////////////////////////////
//!
//! \brief Class for managing a set of KLT point feature trackers
//!
//! Note that external feature selection module is used.
//!
//! \see CFeature2D, CFeature2DTrack, CFeatureTrackStat, and CFeatureCorrespondence
//! \see cvGoodFeaturesToTrack() in opencv for selecting new point features
//!
class CFeature2DPool : public std::vector<CFeature2DTrack*>
{
public:
	//! Constructor
	CFeature2DPool(CvSize imgSize, int templSize, int pyrLevel, int maskSize, bool gpu, CCameraModel* pCameraModel = NULL);

	//! Destructor
	~CFeature2DPool();

	//! Get the total number of tracking features so far.
	int  Size() { return (int)size(); }

	//! Add a new incoming image
	int AddImage(int frame, IplImage* img);

	//! Get an image
	const IplImage* GetImage(int frame = -1);

	//! Purge old images
	int PurgeImage(int frame, int lag_dist);

	//! Add a new feature point of a given location with additional information.
	int  Add(float x, float y, float saliency, int frame, CImagePyramid* newPyr);
	
	//! Purge the old features that have lost before a lag time.
	int  Purge(int frame, int lag_dist);

	//! Track all the features that are currently active in the pool.
	void Track(int frame, int timestamp, IplImage* gimgf, double* Hmg = NULL);

	//! Select new features.
	int  Select(int frame, int min_threshold, int max_num, IplImage* gimgf,  bool verbose = false);

	//! Set the feature referred by id as an outlier.
	void SetOutlier(int id, int inout);

	//! Get all the feature declared as outlier.
	int  GetOutlier(vector<int>& vid);

	//! Check whether the feature of a given ID is outlier or not.
	bool IsOutlier(int id);

	//! Get all the features that are alive at a given frame
	list<int>& GetLastIDs();
	list<int>& GetActiveIDs(int frame = -1);

	//! Save the tracking results
	void SaveToFile(int timestamp, const char* fname, double time, int width, int height);
	
	//! Get the statistical results of tracking.
	CFeatureTrackStat& GetStat(int frame = -1);

	CFeature2DTrack* GetFeatureTrackPtr(int id);
	CFeature2DTrack& GetFeatureTrack(int id);
	CFeature2D*		 GetFeaturePtr(int id, int frame = -1 /*last*/);
	CFeature2D&		 GetFeature(int id, int frame = -1 /*last*/);

public:
	//! Set the mask area of the image which are excluded when new features are selected.
	void ComputeSelectionMask(int templ_size, int mask_size, const list<int>& activeID, IplImage* img_mask);

	CvSize m_imgSize;						//!< input image size
	int m_templateSize;						//!< template size
	int m_maskSize;							//!< feature mask size in feature selection
	int m_pyrLevel;							//!< pyramid level ( =< MAX_PYRAMMID_LEVEL in imageAlign.h)
	bool b_gpu;								//!< flag for GPU use

	int	m_frame;							//!< current frame number
	IplImage* m_img;						//!< current image
	CImagePyramid* m_pyr;					//!< current pyramid image
	map<int, IplImage*> m_mapImage;			//!< image storage

	list<int> m_listLastID;					//!< last active feature IDs
	list<int> m_listActiveID;				//!< current active feature IDs
	list<int> m_listLostID;					//!< current lost feature IDs
	map<int, list<int> > m_mapActiveID;		//!< history of active feature IDs
	map<int, list<int> > m_mapLostID;		//!< history of lost feature IDs

	CFeatureTrackStat m_stat;				//!< current tracking statistics
	map<int, CFeatureTrackStat> m_mapStat;	//!< history of tracking statistics
	
	map<int, int> m_mapOutliers;			//!< outlier map of all the features

	CCameraModel* m_pCameraModel;			//!< camera model to get normalized coordinates of features

	IplImage* m_imgEig;						//!< memory used in feature selection \see cvGoodFeaturesToTrack() in opencv
	IplImage* m_imgTmp;						//!< memory used in feature selection \see cvGoodFeaturesToTrack() in opencv
	IplImage* m_imgMask;					//!< memory used in feature selection \see cvGoodFeaturesToTrack() in opencv

// GPU assisted version
public:
	int	 Select_GPU(int frame, int min_threshold, int max_num, IplImage* img, bool verbose = false);
	void Track_GPU(int frame, IplImage* img, double* Hmg = NULL);

	//! CUDA support
	float m_cuda_fx[MAX_NUM_FEATURES], m_cuda_fy[MAX_NUM_FEATURES];	// for using a GPU
	int m_cuda_index_map_global[MAX_NUM_FEATURES];					// index mapping from cuda mem to global mem
	int m_cuda_index_map_lost[MAX_NUM_FEATURES];					// index of lost features of cuda mem
	int m_cuda_index_map[MAX_NUM_FEATURES];							// index map from new features to cuda mem

	float m_cuda_motion[8*MAX_NUM_FEATURES];
	float m_cuda_residual[MAX_NUM_FEATURES];
};


////////////////////////////////////////////////////////////////////////////////////////////
// inline functions
//
inline std::ostream& operator <<(std::ostream& out, const IplImage* img)
{
	for (int i = 0; i < img->height; i++)  {
		for (int j = 0; j < img->width; j++)  {
			CvScalar s = cvGet2D(img, i, j);
			out << s.val[0] << "\t";
		}
		out << endl;
	}
	return out;
}

inline std::ostream& operator <<(std::ostream& out, const CvMat* mat)
{
	if (mat->cols > 8 && mat->cols > mat->rows) {
		out << "Transposed" << endl;
		for (int j = 0; j < mat->cols; j++)  {
			for (int i = 0; i < mat->rows; i++)  {
				out << setprecision(6) << setiosflags(ios::fixed) << setw(12);
				out << cvmGet(mat, i, j);
			}
			out << endl;
		}
	}
	else {
		for (int i = 0; i < mat->rows; i++)  {
			for (int j = 0; j < mat->cols; j++)  {
				out << setprecision(6) << setiosflags(ios::fixed) << setw(12);
				out << cvmGet(mat, i, j);
			}
			out << endl;
		}
	}
	return out;
}



#endif
