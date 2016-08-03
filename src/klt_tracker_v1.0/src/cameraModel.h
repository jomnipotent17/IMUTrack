/**
* CMU MSUAV Project (Micro and Small Unmanned Aerial Vehicle)
*          
* \file  cameraModel.h
* \brief header file for cameraModel.cxx (CCameraModel)
* \date  12-Jun-2006
*
* \author Myung Hwangbo (myung@cs.cmu.edu) and Qifa Ke
*
* Copyright (c) 2006-2009 Myung Hwangbo and Qifa Ke
* Robotics Institute, Carnegie Mellon University
*
*/

#ifndef __CAMERA_MODEL_H__
#define __CAMERA_MODEL_H__

#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

//! \brief Struct for camera intrinsic parameters
typedef struct {
	int width;			//!< image width
	int height;			//!< image height
	float fc[2];		//!< focal length in x and y
	float cc[2];		//!< camera center
	float skew;			//!< skew value
	float kc[5];		//!< distortion coefficients up to fifth order
	float KK[3][3];		//!< 3x3 camera calibratio matrix
	float Ki[3][3];		//!< inverse of a 3x3 camera calibratio matrix
} CAMERA_INTRINSIC_PARAM;

//!
//! \brief Class for a pin-hole camera model
//!
class CCameraModel
{
public:
	//! Constructor from the struct of camera intrinsic parameters
	CCameraModel(const CAMERA_INTRINSIC_PARAM& p);

	//! Constructor from an undistorted perfect pin-hole camera
	CCameraModel(int w, int h, float vfov, const char *lut_fname);

	//! Constructor from a set of camera intrinsic parameters
	CCameraModel(int w, int h, float fc0, float fc1, float cc0, float cc1, float s, 
				float kc0, float kc1, float kc2, float kc3, float kc4, 
				const char *lut_fname = NULL);

	//! Constructor from the camera parameter file
	CCameraModel(const char* param_fname, const char *lut_fname = NULL);

	//! Destructor
	~CCameraModel();

	int   GetWidth()	{ return m_cam.width; };
	int   GetHeight()	{ return m_cam.height; };
	float GetFx()		{ return m_cam.fc[0]; };
	float GetFy()		{ return m_cam.fc[1]; };
	void  GetIntrinsicParams(CAMERA_INTRINSIC_PARAM& p)       { p = m_cam; }
	void  SetIntrinsicParams(const CAMERA_INTRINSIC_PARAM& p) { m_cam = p; }

	double* GetKK(double KK[9]);
	double* GetKi(double Ki[9]);

	bool IsValid(float px, float py);

	//! Compute the normalized coordinate of given pixel values from a precomputed look-up table
	bool NormalizeFromTable(float px, float py, float& nx, float& ny);
	bool NormalizeFromTable(const vector<float>& px, const vector<float>& py, vector<float>& nx, vector<float>& ny);
	bool NormalizeFromTable(int num, float *px, float *py, float *nx, float *ny);
	bool NormalizeFromTable(int num, float *pxy, float *nxy);
	
	//! Compute a camera calibration matrix from a given camera intrinsic parameters
	void ComputeCameraMatrix(CAMERA_INTRINSIC_PARAM& p);

	//! Compute and store the normalized coordinates of all the image pixels
	void ComputeNormalizeTable(const char* fname = NULL);
	
	bool SaveNormalizeTable(const char* fname);
	bool LoadNormalizeTable(const char* fname);

protected:
	void Normalize(const int px, const int py, float& nx, float& ny);
	void Unnormalize(float nx, float ny, int& px, int& py);
	void Antidistortion(float dx, float dy, float& nx, float& ny);
	void Distortion(float nx, float ny, float& dx, float& dy);

	bool ReadCameraParameter(const char* fname, CAMERA_INTRINSIC_PARAM& p);
	
private:
	bool b_normalTable;				//!< A flag for using the anti-distortion look-up table
	int  m_index_max;				//!< The byte size of the look-up table
		
	CAMERA_INTRINSIC_PARAM m_cam;	//!< Camera intrinsic parameters
	float *m_pNormalTable;			//!< A pointer for the anti-distortion look-up Table
};
				

#endif
