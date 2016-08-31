/**
* CMU MSUAV Project (Micro and Small Unmanned Aerial Vehicle)
*          
* \file  featureTrack.h
* \brief implementation of CFeature2D and CFeature2DTrack
* \date  12-Jun-2006
*
* \author Myung Hwangbo (myung@cs.cmu.edu)
*
* Copyright (c) 2006-2009 Myung Hwangbo
* Robotics Institute, Carnegie Mellon University
*
*/

#include "featureTrack.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//! CFeature2DTrack class
//!
CFeature2DTrack::CFeature2DTrack()
{
	reserve(100);
}

CFeature2DTrack::CFeature2DTrack(float x, float y, float saliency, int frame, int id, int tsize,
								 CImagePyramid* templatePyr, CCameraModel* pCameraModel)
{
	reserve(100);

	m_saliency = saliency;

	m_start_frame = frame;
	m_cur_frame   = frame; 
	m_end_frame   = frame;
	
	m_id = id;
	m_pCameraModel = pCameraModel;

	// Obtain the normalized coordinate of a pixel
	float px[2] = {x, y}, nx[2] = {0.0, 0.0};
	if (m_pCameraModel != NULL) {
		m_pCameraModel->NormalizeFromTable(px[0], px[1], nx[0], nx[1]);
	}

	// Append a new track
	int status = 0;
	float rms = 0.0f;
	float ncc = 0.0f;
	float shrink = 0.0f;
	float bias = 0.0f;
	push_back(new CFeature2D(px[0], px[1], frame, status, rms, ncc, shrink, bias, nx[0], nx[1]));

	// Create a new image aligner
	m_pAligner = NULL;
	if (templatePyr != NULL) {
		m_pAligner = new CImageAlign(this);
		m_pAligner->Initialize(cvPoint((int)x, (int)y), tsize, templatePyr);
	}
}

CFeature2DTrack::~CFeature2DTrack()
{
	if (m_pAligner) delete m_pAligner;
	for (int i=0; i < (int)size(); i++) delete at(i);
}

void CFeature2DTrack::Free() 
{
	if (m_pAligner) delete m_pAligner;
	m_pAligner = NULL;
}

int CFeature2DTrack::GetStatus()		
{
	if (m_pAligner == NULL) return -1;
	return m_pAligner->GetStatus(); 
}

float CFeature2DTrack::GetRMSError()
{
	if (m_pAligner == NULL) return -1.0f;
	return m_pAligner->GetRMSError(); 
}

float CFeature2DTrack::GetNCC()			
{
	if (m_pAligner == NULL) return -1.0f;
	return m_pAligner->GetNCC(); 
}

float CFeature2DTrack::GetShear()		
{
	if (m_pAligner == NULL) return -1.0f;
	return m_pAligner->GetShear(); 
}

int CFeature2DTrack::Run(int frame, CImagePyramid *newPyr, CvMat* ff_motion)
{
	m_pAligner->ResetStatus();
	m_pAligner->PredictFromCameraMotion(ff_motion);		//fp0 is set within this function


	m_pAligner->AlignPyramid(newPyr);					//here, the tracking actually occurs

	m_pAligner->UpdateStatus();


	int   status = m_pAligner->GetStatus();
	float rms    = m_pAligner->GetRMSError();
	float rms_In = m_pAligner->GetRMSErrorInitial();
	float ncc    = m_pAligner->GetNCC();
	float shrink = m_pAligner->GetShear();
	float bias   = m_pAligner->GetBias();
/*
	cout << "rms error before:: " << rms_In << endl;
	cout << "rms error after: " << rms << endl;

	if(rms > rms_In)
	{
		cout << "rms Error: " << rms_In << " -> " << rms << endl; 
	}
*/

	float px[2] = {0, 0};
	float qx[2] = {0, 0};
	float nx[2] = {0, 0};

	// Compute a normalized coordinate
	m_pAligner->GetTrackedPoint(px);
	m_pAligner->GetGuessPoint(qx);
	if (m_pCameraModel != NULL) {
		m_pCameraModel->NormalizeFromTable(px[0], px[1], nx[0], nx[1]);
	}

	// Add a new tracked point								   /*Matthew Add*/
	push_back(new CFeature2D(px[0], px[1], frame, status, rms, /**/rms_In/**/, ncc, shrink, bias, nx[0], nx[1], qx[0], qx[1]));

	if (!(status & CImageAlign::TRACK_FAIL)) {
		m_cur_frame = frame; 
		m_end_frame = frame;
	}

	return status;
}

void CFeature2DTrack::UpdateTemplate()
{
	m_pAligner->UpdateTemplate();
}

CFeature2D* CFeature2DTrack::GetFeaturePtr(int frame)
{
	int k = (frame == -1) ? m_end_frame-m_start_frame : frame-m_start_frame;
	int m = (int)size();
	if (k < 0 || k >= m) return NULL;

	return at(k);
}

CFeature2D& CFeature2DTrack::GetFeature(int frame)
{
	int k = (frame == -1) ? m_end_frame-m_start_frame : frame-m_start_frame;
	int m = (int)size();
	assert(k >= 0 && k < m);
	return (*at(k));
}
