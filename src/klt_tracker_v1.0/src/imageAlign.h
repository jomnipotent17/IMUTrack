/**
* CMU MSUAV Project (Micro and Small Unmanned Aerial Vehicle)
*          
* \file  trackDisplay.cxx
* \brief header file for imageAlign.cxx
* \date  12-Jun-2006
*
* \author Myung Hwangbo (myung@cs.cmu.edu) and Qifa Ke
*
* Copyright (c) 2006-2009 Myung Hwangbo and and Qifa Ke
* Robotics Institute, Carnegie Mellon University
*
*/

#ifndef __IMAGE_ALIGH_H__
#define __IMAGE_ALIGH_H__

#include <iostream>
#include <iomanip>
#include <fstream>

#include <vector>
#include <list>

//#include <opencv2/opencv.hpp>	//TODO make sure this works
#include <cv.h>
#include "featureTrack.h"

using namespace std;

#define	MAX_PYRAMID_LEVEL	5

typedef IplImage* IplImagePtr;
class CFeature2DTrack;

typedef struct {
	float fp[2];
	float param[8];
} ALIGN_DATA;

////////////////////////////////////////////////////////////////////////////////////////////
//!
//! \brief Class for image pyramid
//!
class CImagePyramid
{
public:
	CImagePyramid(const IplImage* src, int level) : m_maxLevel(level)
	{
		m_pyr = new IplImagePtr[m_maxLevel];
		m_pyr[0] = cvCloneImage(src);

        for (int i=1; i < m_maxLevel; i++) {
			CvSize size = cvGetSize(m_pyr[i-1]);
			m_pyr[i] = cvCreateImage(cvSize(size.width/2, size.height/2), IPL_DEPTH_32F, 1);
			cvPyrDown(m_pyr[i-1], m_pyr[i], CV_GAUSSIAN_5x5);
		}
	}
	~CImagePyramid() {
		for (int i = 0; i < m_maxLevel; i++) cvReleaseImage(&m_pyr[i]);
		delete [] m_pyr;
	}

	IplImage* operator[](int i)	{
		return (i < m_maxLevel) ? m_pyr[i] : NULL;
	}

	int GetMaxLevel() { return m_maxLevel; }
	int GetWidth()    { return m_pyr[0]->width; }
	int GetHeight()   { return m_pyr[0]->height; }

private:
	int m_maxLevel;
	IplImagePtr *m_pyr;
};

////////////////////////////////////////////////////////////////////////////////////////////
//!
//! \brief Class for image alignment (KLT tracker)
//!
class CImageAlign
{
public:
	//! Algnment method used at each pyramid level
	enum ALIGNMENT_METHOD_ENUM {
		ALIGN_TRANSLATION = 0,
		ALIGN_AFFINE_PHOTO
	};

	//! Algnment status at each pyramid level
	enum ALIGNMENT_STATUS_ENUM {
		TRACK_NULL = 0x0000,
		TRACK_NEW = 0x0001,
		TRACK_SUCCESS = 0x0010,
		TRACK_SUCCESS_TEMPL_UPDATE = 0x0030,
		TRACK_FAIL = 0x0100,
		TRACK_FAIL_ALIGNMENT = 0x0300,
		TRACK_FAIL_BOUND = 0x0500,
		TRACK_FAIL_RMS = 0x0900,
		TRACK_FAIL_SHRINK = 0x1100,
		TRACK_FAIL_NCC = 0x3100,
	};

public:
	//! Constructor
	CImageAlign(CFeature2DTrack* p);

	//! Destructor
	~CImageAlign();

	//! Initialize the parameters and allocate the memories.
    bool	Initialize(CvPoint c, int wsize, CImagePyramid* Ipyr);

	//! Reset alignment parameters and status
	void	ResetStatus();

	//! Update the template with its gradient and Hessian together.
	void	UpdateTemplate();

	//! Align a given pyramid image to the template in multiple steps.
	int		AlignPyramid(CImagePyramid* Ipyr);

	//! Compensate alignment steps from a given known camera motion.
	void	PredictFromCameraMotion(CvMat* motion);
	void    PredictNextFeaturePoint(CvMat* motion, float p[2]);

	//! Update current alignment status and check whether if the template needs updated.
	int		UpdateStatus();
	
	void	GetTrackedPoint(float p[2]) { p[0] = m_fp[0],  p[1] = m_fp[1]; };
	void	GetGuessPoint  (float p[2]) { p[0] = m_fp0[0], p[1] = m_fp0[1]; };
	void	GetLastPoint   (float p[2]) { p[0] = m_lp[0],  p[1] = m_lp[1]; };
	void	GetInitialPoint(float p[2]) { p[0] = m_c[0],   p[1] = m_c[1]; };
	void    GetFourCorners(float p[4][2]);

	float	GetRMSError()	{ return m_RMS_error; };
	float 	GetRMSErrorInitial() { return m_RMS_error_INITIAL; };
	float	GetNCC()		{ return m_NCC; };
	float	GetShear()      { return m_shear; };
	float	GetBias()		{ return m_bias; };
	int		GetStatus()		{ return m_status; };
	
	void	Read(const char *fname);
	CFeature2DTrack* GetPtrFeature2DTrack() { return m_pFeature2DTrack; };
	
protected:
	void	ComputeHessian(int level, CvMat* H, CvMat* invH);
	void	ComputeB(int level, CvMat* bm);
	void	ComputeGradient(int level);
	float	ComputeError(int level);
	float	ComputeUVcorrection(const CvRect& rect, CvMat* old_motion, CvMat* new_motion);

	void	WarpBackImage(IplImage* src, IplImage* dst, CvRect r, CvMat* motion);
	void	WarpPoint(CvMat* M, float p[2], float w[2]);
	void	WarpPoint(CvMat* M, CvMat* p, CvMat* w);

	bool	Align(int level, ALIGNMENT_METHOD_ENUM mode, IplImage* I_whole);
	void	UpdateMotion(int level, float m_step_size, CvMat* motion);
	
	void	ScaleMotionUp(int level, CvMat* motion);
	void	ScaleMotionDown(int level, CvMat* motion);

	float	ComputeNormalizedCorrelation(const IplImage* T, IplImage* I);
	float	ComputeShearAmount(CvMat* motion);
	bool	IsInRange(const float& x, const float& y);
	
private:
	CvMat* m_motion;

	CvMat* t1, *t2, *g2;
	CvMat* dM, *dM_t, *dM_i, *dM_c;

	int pyrLevel;

	IplImage* m_imgGx[MAX_PYRAMID_LEVEL];
	IplImage* m_imgGy[MAX_PYRAMID_LEVEL];
	IplImage* m_imgWm[MAX_PYRAMID_LEVEL];

	float m_sum_weight[MAX_PYRAMID_LEVEL];
	float m_sum_werr[MAX_PYRAMID_LEVEL];
	float m_sum_werr2[MAX_PYRAMID_LEVEL];
	
	IplImage* m_imgT[MAX_PYRAMID_LEVEL];
	IplImage* m_imgI[MAX_PYRAMID_LEVEL];
	IplImage* m_imgErr[MAX_PYRAMID_LEVEL];
	IplImage* m_imgWerr[MAX_PYRAMID_LEVEL];
	IplImage* m_imgDx[MAX_PYRAMID_LEVEL];
	IplImage* m_imgDy[MAX_PYRAMID_LEVEL];
	IplImage* m_imgDx2[MAX_PYRAMID_LEVEL];
	IplImage* m_imgDy2[MAX_PYRAMID_LEVEL];
	IplImage* m_imgDxy[MAX_PYRAMID_LEVEL];

	int hw[MAX_PYRAMID_LEVEL];
	CvSize m_templ_size[MAX_PYRAMID_LEVEL];
	CvRect m_roi[MAX_PYRAMID_LEVEL];
	CvPoint m_center[MAX_PYRAMID_LEVEL];
	float m_c[2], m_lp[2], m_fp[2], m_fp0[2];

	int m_modelSize[2];
	CvMat* m_Hessian[MAX_PYRAMID_LEVEL];
	CvMat* m_HessianInv[MAX_PYRAMID_LEVEL];
	CvMat* m_b[MAX_PYRAMID_LEVEL];
	CvMat* m_x[MAX_PYRAMID_LEVEL];
	CvMat* m_x0[MAX_PYRAMID_LEVEL];

	bool m_reset;
	int  m_id;
		
	float m_bias;
	float m_RMS_error_old;
	float m_RMS_error;
	float m_RMS_error_INITIAL;	//MATTHEW ADDED THIS
	float m_uv_correct;

	float m_photo_delta;
	float m_photo_lambda;
	float m_photo_scale;

	int   m_status;
	float m_RMS_error_threshold;
	float m_NCC, m_NCC_threshold;
	float m_shear, m_shear_threshold;
	bool  m_inRange;
	float m_range_lower[2], m_range_upper[2];

	ALIGNMENT_METHOD_ENUM m_alignMode[MAX_PYRAMID_LEVEL];
	CFeature2DTrack* m_pFeature2DTrack;
};


#endif
