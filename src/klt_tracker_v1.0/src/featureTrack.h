/**
* CMU MSUAV Project (Micro and Small Unmanned Aerial Vehicle)
*          
* \file  featureTrack.h
* \brief header file for featureTrack.cxx (CFeature2D and CFeature2DTrack)
* \date  12-Jun-2006
*
* \author Myung Hwangbo (myung@cs.cmu.edu)
*
* Copyright (c) 2006-2009 Myung Hwangbo
* Robotics Institute, Carnegie Mellon University
*
*/

#ifndef __FEATURE_TRACK_H__
#define __FEATURE_TRACK_H__

#include <iostream>
#include <iomanip> 
#include <fstream>
#include <vector>

#include <cv.h>
#include "imageAlign.h"
#include "cameraModel.h"
//#include "ctimer.h"

using namespace std;

class CImageAlign;
class CImagePyramid;

////////////////////////////////////////////////////////////////////////////////////////////
//!
//! \brief Class for 2D image point feature
//!
class CFeature2D
{
public:
	//! Constructor
	CFeature2D(float x, float y, int frame, int status = 0, 
			   float rms = 0, float rms_In = 0, float ncc = 0, float shrink = 0, float bias = 0,
			   float nx = 0, float ny = 0, float x0 = 0, float y0 = 0) 
		: m_x(x), m_y(y), m_frame(frame), m_status(status),
		  m_rms(rms), /*matthew ---> */ m_rms_In(rms_In),/* <--- matthew*/ m_ncc(ncc), m_shear(shrink), m_bias(bias),
		  m_nx(nx), m_ny(ny), m_x0(x0), m_y0(y0) {};

	//! Destructor
	~CFeature2D() {};

public:
	float	m_x0, m_y0;	//!< Initial feature position in pixel coordinate
	float	m_x,  m_y;	//!< Current feature position in pixel coordinate
	float   m_nx, m_ny;	//!< Feature position in a normalized coordinate

	int		m_frame;	//!< frame counter
	int		m_status;	//!< image alignment status

	float	m_rms;		//!< root mean squared error
	float   m_rms_In;	//!!!!!!MATTHEW ADDED THIS IN!!!!!!! (original rms error from imu)
	float	m_ncc;		//!< normalized correlation
	float	m_shear;	//!< sheared amount
	float	m_bias;		//!< bias
};


////////////////////////////////////////////////////////////////////////////////////////////
//!
//! \brief Class for a sigle KLT point feature tracker
//! \see CFeature2D, CImageAlign, and CCameraModel
//!
class CFeature2DTrack : public std::vector<CFeature2D*>
{
public:
	//! Constructor
	CFeature2DTrack();

	//! Constructor
	CFeature2DTrack(float x, float y, float saliency, int frame, int id, int tsize, 
		CImagePyramid* templatePyr, CCameraModel* pCameraModel);

	//! Destructor
	~CFeature2DTrack();

	//! Free the memory used in image alignment.
	void Free();

	//! Run the KLT and provide a predictive camera motion if available.
	int  Run(int frame, CImagePyramid *newPyr, CvMat* ff_motion = NULL);

	//! Update the template in the KLT.
	void UpdateTemplate();

	//! Get the pointer of feature2D at a given frame.
	CFeature2D* GetFeaturePtr(int frame = -1 /*last*/);

	//! Get the reference of feature2D at a given frame.
	CFeature2D& GetFeature(int frame = -1 /*last*/);

	int GetStartFrame()		{ return m_start_frame; }
	int GetEndFrame()		{ return m_end_frame; }
	int GetCurrentFrame()	{ return m_cur_frame; }
	int GetID()				{ return m_id; }

	void SetStartFrame(int frame)	{ m_start_frame = frame; }
	void SetEndFrame(int frame)		{ m_end_frame = frame; }
	void SetCurrentFrame(int frame)	{ m_cur_frame = frame; }

	float GetSaliency()		{ return m_saliency; }

	int   GetStatus();
	float GetRMSError();
	float GetNCC();
	float GetShear();

	CImageAlign* GetImageAligner() { return m_pAligner; }

private:
	float m_saliency;

	int m_start_frame;	//!< starting frame of the feature
	int m_end_frame;	//!< ending frame of the feature
	int m_cur_frame;	//!< current frame of the feature

	int m_id;			//!< unique ID number tagged with the feature
	int	m_status;		//!< tracking status

	CImageAlign* m_pAligner;
	CCameraModel* m_pCameraModel;
};


////////////////////////////////////////////////////////////////////////////////////////////
//!
//! \brief Class for recording feature tracking results
//!
class CFeatureTrackStat
{
public: 
	CFeatureTrackStat() {
		frame = 0;
		n_success = n_total = n_new = 0;
		time_track = time_select = 0;
	}

public:
	int frame;				//!< frame number
	int n_success;			//!< the number of successful trackers
	int n_total;			//!< total number of trackers
	int n_new;				//!< the number of the newly started trackers
	float flow[2];			//!< optical flow
	float time_track;		//!< time used in KLT tracking
	float time_select;		//!< time used in selecting new features
};

#endif
