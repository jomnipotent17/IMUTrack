/**
* CMU MSUAV Project (Micro and Small Unmanned Aerial Vehicle)
*          
* \file  imageAlign.cxx
* \brief implementation of CImagePyramid and CImageAlign classes
* \date  12-Jun-2006
*
* \author Myung Hwangbo (myung@cs.cmu.edu) and Qifa Ke
*
* Copyright (c) 2006-2009 Myung Hwangbo and Qifa Ke
* Robotics Institute, Carnegie Mellon University
*
*  - Do not forget to recompute the gradient and Hessian whenever the template is updated.
*  - The frequency of template update should be carefully considered for speed-up purpose.
*  - Now, a quarter of execution time_msec is consumed in computing gradient and Hessian.
*  - The inverse of affine motion can be computed manually. 
*    It is much better not using an expensive general matrix inversion routine such as cvInvert().
*/

/*
 -----------------------------------------------------------------------
 Description on the coordinate system used in the image warping function
 -----------------------------------------------------------------------

 First the warping function w(x;p) uses the pixel coordinate.
 The origin is the top-left corner of the image and the x-axis goes right along the width 
 and the y-axis goes down along the height. 

	T(x) = I(w(x;p)

 For image warping, we uses ippiWarpAffineBack(). It is also based on the pixel coordinate.
 The new image "I" warps back to "Iw" in order to compare with the template "T". The pixel 
 coordinates (x',y') in the warped image "Iw" are obtained from the following equations:

	Iw(x') = Iw(w^-1(x;p)) = I(x)
	
	c00*x¡Ç + c01*y¡Ç + c02 = x
	c10*x¡Ç + c11*y¡Ç + c12 = y

 Since "Iw" has the template size in our implementation, translational offset is needed to 
 warp the image I to a correct area where "Iw" exists. It shifts the coordinate so that
 "Iw" is at the top-left corner of the image.

	w' = w o T(r.x, r.y)

 The feature point also uses the same pixel coordinate above. The trajectory of the feature
 location "x(i)" can be computed from the starting position "x0" and the current warping
 parameter "p(i)".

	T(x0) = I(w(x0; p(i))) --> x(i) = w(x0; p(i))
	                                = A*x0 + b

 The inverse compositional method is to find "dp" such that the following equality holds

	I(w(x;p(i)) = T(w(x;dp)) = T(x)
	
	I(x' = w(x0;p(i)) = T(x0)  --->  x' = w(x0; p(i))  ==>  x' = w(w^-1(x;dp); p(i))
	T(x0 = w(x;dp)) = T(x0)    --->  x0 = w(x; dp)

 
 Inverse compositional method vs. Forward compositional method

    +---+   w^-1(dp)
	|   |  +---+        w(p(i))            w(p(i+1)) = w(p(i)) o w^-1(dp)
    +---+  |   |                 +---+    
    T      +---+                /   /      w(dp) is a warping from T(i) to I(i+1)/I(i)
           I(i+1)/I(i)         +---+       described in T
                              I(i+1)



    +---+     w(p(i))
	|   |              +---+  w(dp)        w(p(i+1)) = w(dp) o w(p(i))
    +---+             /   /    +---+    
    T                +---+    /   /        w(dp) is a warping from I(i) to I(i+1)
                    I(i)     +---+         described in I(i)
                             I(i+1)


 Note that "dp" computed from the Hessian inverse and b_vector is represented in the local 
 template coordinate. It needs to be transformed into the pixel coordinate in which the 
 warping function "w(x)" is described. In the affine case,

	x0_b' = A(dp) x_b -->  P_w x0_w' = A(dp) P_w x_w
	                  -->  x0_w' = (P_w^-1 A(dp) P_w) x_w
	
 Therefore, A(dp)_w = P_w^-1 A(dp) P_w
*/

#include <cmath>
#include <limits>

#include "imageAlign.h"

//! [Caution for index arguments]
//!  Pay attention to how indices(i,j) are working in CV_IMAGE_ELEM(i, j).
//!  Remember that "i" indicates row which is "y" coordinate in the gradient image we are generating.

#define	CV_PMAT_ELEM(mat, elemtype, row, col) \
			(((elemtype*)((mat)->data.ptr + (mat)->step*(row)))[(col)])

#define CV_IMAGE_ELEM_IDX(image, elemtype, idx)       \
    ((elemtype*)((image)->imageData + idx))


#define CLAMP(x, y) ((x > 0) ? min(x, y) : max(x, -y));

#define __IPP

#ifdef __IPP
#include <ipp.h>
typedef Ipp32f PDTYPE;
#define	IPL_DEPTH_PROC				IPL_DEPTH_32F
#define	IppsThreshold_LT_I			ippsThreshold_LT_32f_I
#define	IppsDivC_I					ippsDivC_32f_I
#define	IppsAbs						ippsAbs_32f
#define	IppsAdd_I					ippsAdd_32f_I
#define	IppsSub						ippsSub_32f
#define	IppsSubC_I					ippsSubC_32f_I
#define	IppsMulC_I					ippsMulC_32f_I
#define	IppsMul						ippsMul_32f
#define	IppsSum(src, len, sum)		ippsSum_32f(src, len, sum, ippAlgHintFast)
#define	IppsDotProd					ippsDotProd_32f
#define	IppsZero					ippsZero_32f
#define	IppsSet						ippsSet_32f
#define	IppsMalloc					ippsMalloc_32f
#define	IppsFree					ippsFree
#define	IppiFilterRow				ippiFilterRow_32f_C1R
#define	IppiFilterColumn			ippiFilterColumn_32f_C1R
#else
typedef float PDTYPE;
#define	IPL_DEPTH_PROC				32
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
CImageAlign::CImageAlign(CFeature2DTrack* p) 
						: m_pFeature2DTrack(p) 
{

}

bool CImageAlign::Initialize(CvPoint c, int wsize, CImagePyramid* Ipyr)
{
	int i;
	pyrLevel = Ipyr->GetMaxLevel();

	m_id = m_pFeature2DTrack->GetID();

	//! 1. Set alignment parameters
	m_reset = true;
		
	m_bias = 0.0f;
	m_RMS_error_old = 0.0f;
	m_RMS_error = 0.0f;
	m_uv_correct = 0.0f;

	m_photo_delta = 0.0f;
	m_photo_lambda = 1.0f;
	m_photo_scale = 1.0f / 16.0f;
	
	m_modelSize[ALIGN_TRANSLATION]  = 2;
	m_modelSize[ALIGN_AFFINE_PHOTO] = 8;

	//! 2. Set status variables
	m_status = TRACK_NEW;
	m_NCC = 0.0f;
	m_shear = 0.0f;
	m_inRange = true;

	//! 3. Set threshold values and frequently used constants
	m_RMS_error_threshold = 20.0f;
	m_NCC_threshold = 0.5f;
	m_shear_threshold = 0.3f;
	m_range_lower[0] = (float)wsize/4;
	m_range_lower[1] = (float)wsize/4;
	m_range_upper[0] = Ipyr->GetWidth()  - m_range_lower[0];
	m_range_upper[1] = Ipyr->GetHeight() - m_range_lower[1];

	//! 4. Compute the template-related constants.
	m_c[0] = m_lp[0] = m_fp[0] = m_fp0[0] = (float)c.x;
	m_c[1] = m_lp[1] = m_fp[1] = m_fp0[1] = (float)c.y;

	//for (i=0; i < MAX_PYRAMID_LEVEL; i++) hw[i] = wsize >> (i+1);
	for (i=0; i < pyrLevel; i++) hw[i] = wsize >> 1;

	for (i=0; i < pyrLevel; i++) {
		m_center[i] = cvPoint(c.x >> i, c.y >> i);
		m_roi[i] = cvRect(m_center[i].x - hw[i], m_center[i].y - hw[i], 2*hw[i] + 1, 2*hw[i] + 1);
		m_templ_size[i] = cvSize(2*hw[i] + 1, 2*hw[i] + 1);
	}

	//! 5. Memory Allocation
	for (i=0; i < pyrLevel; i++) {
		//! Template and warped image
		m_imgI[i] = cvCreateImage(m_templ_size[i], IPL_DEPTH_PROC, 1);
		m_imgErr[i]  = cvCreateImage(m_templ_size[i], IPL_DEPTH_PROC, 1);
		m_imgWerr[i]   = cvCreateImage(m_templ_size[i], IPL_DEPTH_PROC, 1);
		m_imgT[i] = cvCreateImage(m_templ_size[i], IPL_DEPTH_PROC, 1);

		//! Gradient
		m_imgGx[i] = cvCreateImage(m_templ_size[i], IPL_DEPTH_PROC, 1);
		m_imgGy[i] = cvCreateImage(m_templ_size[i], IPL_DEPTH_PROC, 1);
		m_imgWm[i] = cvCreateImage(m_templ_size[i], IPL_DEPTH_PROC, 1);
		m_sum_weight[i] = m_sum_werr[i] = m_sum_werr2[i] = 0.0f;

		//! Hessian
		m_alignMode[i] = (i < 2) ? 	ALIGN_AFFINE_PHOTO : ALIGN_TRANSLATION;
		//m_alignMode[i] = ALIGN_AFFINE_PHOTO;

		int msize = m_modelSize[m_alignMode[i]];
		m_Hessian[i]    = cvCreateMat(msize, msize, CV_32FC1);
		m_HessianInv[i] = cvCreateMat(msize, msize, CV_32FC1);
        m_b[i]          = cvCreateMat(msize, 1, CV_32FC1);
		m_x[i]          = cvCreateMat(msize, 1, CV_32FC1);
		m_x0[i]    = cvCreateMat(msize, 1, CV_32FC1);

		m_imgDx[i]  = cvCreateImage(m_templ_size[i], IPL_DEPTH_PROC, 1);
		m_imgDy[i]  = cvCreateImage(m_templ_size[i], IPL_DEPTH_PROC, 1);
		m_imgDx2[i] = cvCreateImage(m_templ_size[i], IPL_DEPTH_PROC, 1);
		m_imgDy2[i] = cvCreateImage(m_templ_size[i], IPL_DEPTH_PROC, 1);
		m_imgDxy[i] = cvCreateImage(m_templ_size[i], IPL_DEPTH_PROC, 1);
	}
	//! The motion matrix
	m_motion = cvCreateMat(4, 4, CV_32F);
	cvSetIdentity(m_motion);

	t1 = cvCreateMat(4, 4, CV_32F);
	t2 = cvCreateMat(4, 4, CV_32F);
	g2 = cvCreateMat(4, 4, CV_32F);

	dM   = cvCreateMat(4, 4, CV_32F);
	dM_t = cvCreateMat(4, 4, CV_32F);
	dM_i = cvCreateMat(4, 4, CV_32F);
	dM_c = cvCreateMat(4, 4, CV_32F);

	//! 6. Crop the template image
	for (i=0; i < pyrLevel; i++) {
		cvSetImageROI((*Ipyr)[i], m_roi[i]);
		cvCopyImage((*Ipyr)[i], m_imgT[i]);
	}

	//! 7. Compute the image gradient for all pyramid levels
	for (i=0; i < pyrLevel; i++) {
		ComputeGradient(i);
	}

	//! 8. Compute the image Hessian for all pyramid levels
	for (i=0; i < pyrLevel; i++) 
	{
		PDTYPE p, q;
		for (int x=0; x < m_templ_size[i].width; x++) {
			for (int y=0; y < m_templ_size[i].height; y++) {
				p = (PDTYPE)(x - m_templ_size[i].width/2);
				q = (PDTYPE)(y - m_templ_size[i].height/2);
				CV_IMAGE_ELEM(m_imgDx[i], PDTYPE, y, x)  = p;
				CV_IMAGE_ELEM(m_imgDy[i], PDTYPE, y, x)  = q;
				CV_IMAGE_ELEM(m_imgDx2[i], PDTYPE, y, x) = p*p;
				CV_IMAGE_ELEM(m_imgDy2[i], PDTYPE, y, x) = q*q;
				CV_IMAGE_ELEM(m_imgDxy[i], PDTYPE, y, x) = p*q;
			}
		}

		ComputeHessian(i, m_Hessian[i], m_HessianInv[i]);
	}

	if( cvGetErrStatus() < 0 ) {
		cout << "Some error occurred." << endl;
	}

	return true;
}

CImageAlign::~CImageAlign()
{
	for (int i=0; i < pyrLevel; i++) {
		cvReleaseImage(&m_imgGx[i]);
		cvReleaseImage(&m_imgGy[i]);
		cvReleaseImage(&m_imgWm[i]);

		cvReleaseImage(&m_imgT[i]);
		cvReleaseImage(&m_imgI  [i]);
		cvReleaseImage(&m_imgErr[i]);
		cvReleaseImage(&m_imgWerr[i]);
		cvReleaseImage(&m_imgDx[i]);
		cvReleaseImage(&m_imgDy[i]);
		cvReleaseImage(&m_imgDx2[i]);
		cvReleaseImage(&m_imgDy2[i]);
		cvReleaseImage(&m_imgDxy[i]);
		
		cvReleaseMat(&m_Hessian[i]);
		cvReleaseMat(&m_HessianInv[i]);
		cvReleaseMat(&m_b[i]);
		cvReleaseMat(&m_x[i]);
		cvReleaseMat(&m_x0[i]);
	}
	
	cvReleaseMat(&m_motion);
	cvReleaseMat(&t1);
	cvReleaseMat(&t2);
	cvReleaseMat(&g2);

	cvReleaseMat(&dM);
	cvReleaseMat(&dM_t);
	cvReleaseMat(&dM_i);
	cvReleaseMat(&dM_c);
}

void CImageAlign::ResetStatus()
{
	m_reset = true;
		
	m_bias = 0.0f;
	m_RMS_error_old = 0.0f;
	m_RMS_error = 10000.0f;
	m_uv_correct = 0.0f;

	//m_photo_delta = 0.0f;
	//m_photo_lambda = 1.0f;
}

void CImageAlign::UpdateTemplate()
{
	for (int i=pyrLevel-1; i >= 0; i--) {
		cvCopy(m_imgI[i], m_imgT[i]);
		ComputeGradient(i);
		ComputeHessian(i, m_Hessian[i], m_HessianInv[i]);
	}
}

void CImageAlign::PredictNextFeaturePoint(CvMat* motion, float p[2])
{
	if (motion != NULL) {
		CvMat* mm = cvCreateMat(4, 4, CV_32F);
		cvMatMul(motion, m_motion, mm);
		WarpPoint(mm, m_c, p);
		cvReleaseMat(&mm);
	}
	else {
		p[0] = m_c[0];
		p[1] = m_c[1];
	}
}

void CImageAlign::PredictFromCameraMotion(CvMat* motion)
{
	if (motion != NULL) {
		float p0[2], p1[2];

		// Homography
		PredictNextFeaturePoint(motion, p0);

		// Affine enforcement
		CvMat* A = cvCreateMat(4, 4, CV_32F);
		cvSetIdentity(A);

		cvCopy(motion, A);
		cvScale(A, A, cvmGet(A, 3, 3));
		cvmSet(A, 3, 0, 0);
		cvmSet(A, 3, 1, 0);

		PredictNextFeaturePoint(A, p1);

		// Compensate the difference between H*x and A*x + b
		cvmSet(A, 0, 3, cvmGet(A, 0, 3) + p0[0] - p1[0]);
		cvmSet(A, 1, 3, cvmGet(A, 1, 3) + p0[1] - p1[1]);

		/*
		cvmSet(A, 0, 3, p0[0] - m_fp[0]);
		cvmSet(A, 1, 3, p0[1] - m_fp[1]);
		*/

		cvMatMul(A, m_motion, m_motion);
	}

	WarpPoint(m_motion, m_c, m_fp0);
	
	/*
	if (motion != NULL) {
		cvMatMul(motion, m_motion, m_motion);
	}
	WarpPoint(m_motion, m_c, m_fp0);
	*/
}

void CImageAlign::GetFourCorners(float p[4][2])
{
	float c[4][2] = {{m_c[0] - hw[0], m_c[1] - hw[0]},
					 {m_c[0] + hw[0], m_c[1] - hw[0]},
					 {m_c[0] + hw[0], m_c[1] + hw[0]},
					 {m_c[0] - hw[0], m_c[1] + hw[0]}};

	for (int i=0; i < 4; i++) {
		WarpPoint(m_motion, c[i], p[i]);
	}
}

#define	MIN_UV_CHANGE		0.04
#define	MAX_UV_CHANGE		10
#define	MAX_STEPS_LEVEL		10

int CImageAlign::AlignPyramid(CImagePyramid* Ipyr)
{
	m_status = TRACK_NULL;
	m_lp[0] = m_fp[0];
	m_lp[1] = m_fp[1];

	int iter_count = 0;
	for (int i = Ipyr->GetMaxLevel()-1; i >=0; i--) {
		bool converged = false;
		int	iter_level = 0;

		//cout << endl << "Pyramid Level: " << i << endl;
		while(!converged && (iter_level < MAX_STEPS_LEVEL)) {
			//cout << "Align Pyramid LINE: " << __LINE__ << endl;
			//cout << "iter_level = " << iter_level << endl;
			Align(i, m_alignMode[i], (*Ipyr)[i]);
			//cout << "Align Pyramid LINE: " << __LINE__ << endl;
			converged = (m_uv_correct < MIN_UV_CHANGE);
			iter_level++;
			iter_count++;
		}

		m_reset = true;
	}

	WarpPoint(m_motion, m_c, m_fp);

	return iter_count;
}

//TODO check out Align
bool CImageAlign::Align(int level, ALIGNMENT_METHOD_ENUM mode, IplImage* I_whole)
{
	ScaleMotionDown(level, m_motion);
	
	CvMat* motion_old  = cvCloneMat(m_motion);
	CvMat* motion_prev = cvCloneMat(m_motion);

	int max_inner_iter = 10;
	for (int iter = 0;; iter++) 
	{
		//cout << "	iter= " << iter << endl;
		if ((iter > 0 && m_uv_correct <= MAX_UV_CHANGE) || m_reset) {
			m_reset = false;
			//cout << "Align ln: " << __LINE__ << endl;
			WarpBackImage(I_whole, m_imgI[level], m_roi[level], m_motion);
			//cout << "Align ln: " << __LINE__ << endl;
			m_RMS_error = ComputeError(level);
			if( (iter == 0) && (level == 2) )
			{
				//cout << "	Error (iter=" << iter << "): " << m_RMS_error << "               WE WANT THIS ONE" << endl;
				m_RMS_error_INITIAL = m_RMS_error;
			}
			else
			{
				//cout << "	Error (iter=" << iter << "): " << m_RMS_error << endl; 
			}
		}
		if (iter == 0) {
			m_RMS_error_old = m_RMS_error;
		}
		else if (m_RMS_error < m_RMS_error_old && m_uv_correct < MAX_UV_CHANGE) {
			break;
		}
		else if (iter > max_inner_iter || m_uv_correct < MIN_UV_CHANGE) {
			if (m_uv_correct > MAX_UV_CHANGE) {
				cvCopy(motion_old, m_motion);
				m_RMS_error = m_RMS_error_old;
				m_uv_correct = 0.0f;
			}
			break;
		}

		if (iter == 0) {
			ComputeB(level, m_b[level]);
		}
		else {
			cvCopy(motion_old, m_motion);
		}

		float step_size = 1.0f / (1 << iter);

		cvCopy(m_motion, motion_prev);
		UpdateMotion(level, step_size, m_motion);
		m_uv_correct = ComputeUVcorrection(m_roi[level], motion_prev, m_motion);
	}

	cvReleaseMat(&motion_old);
	cvReleaseMat(&motion_prev);

	ScaleMotionUp(level, m_motion);

	return true;
}

void CImageAlign::UpdateMotion(int level, float step_size, CvMat* motion)
{
	int i;
	float s = step_size;

	cvSetIdentity(dM);
	cvSetIdentity(dM_i);

	float **M = new float*[dM->rows];
	for (i=0; i < dM->rows; i++) M[i] = (float *)(dM->data.fl + i*dM->step/sizeof(float));

	float **Mi = new float*[dM_i->rows];
	for (i=0; i < dM_i->rows; i++) Mi[i] = (float *)(dM_i->data.fl + i*dM_i->step/sizeof(float));

	float *x0 = (float *)(m_x0[level]->data.fl);

	//! Solve the linear equation of motion from a first-order approximation
	if (s >= 0.9) {
		cvMatMul(m_HessianInv[level], m_b[level], m_x[level]);	//! x = H^(-1) * b
		cvCopy(m_x[level], m_x0[level]);
	}

	for (i=0; i < m_modelSize[m_alignMode[level]]; i++)
		x0[i] = CLAMP(x0[i], 1e3f);

	if (m_alignMode[level] >= ALIGN_TRANSLATION) {
		M[0][3] += s*x0[0];
		M[1][3] += s*x0[1];
	}

	if (m_alignMode[level] >= ALIGN_AFFINE_PHOTO) {
		M[0][0] += s*x0[2];
		M[1][0] += s*x0[3];
		M[0][1] += s*x0[4];
		M[1][1] += s*x0[5];
		m_photo_delta += m_photo_lambda * (s*x0[7])/m_photo_scale;
		m_photo_lambda *= (1.0f + s*x0[6]);

		//m_photo_delta  = CLAMP(m_photo_delta, 1e3);
		//m_photo_lambda = CLAMP(m_photo_lambda, 1e3);
	}

	// Coordinate Transformation
	cvSetIdentity(t1);
	cvSetIdentity(t2);
	cvmSet(t1, 0, 3, -m_center[level].x);
	cvmSet(t1, 1, 3, -m_center[level].y);
	cvmSet(t2, 0, 3,  m_center[level].x);
	cvmSet(t2, 1, 3,  m_center[level].y);

	// motion = motion * (t2 * dM^(-1) * t1)
	if (0) {
		cvInvert(dM, dM_i, CV_SVD);
	}
	else {
		float det = M[0][0]*M[1][1] - M[1][0]*M[0][1];
		Mi[0][0] =  M[1][1]/det;
		Mi[0][1] = -M[0][1]/det;
		Mi[1][0] = -M[1][0]/det;
		Mi[1][1] =  M[0][0]/det;
		Mi[0][3] = -(Mi[0][0]*M[0][3] + Mi[0][1]*M[1][3]);
		Mi[1][3] = -(Mi[1][0]*M[0][3] + Mi[1][1]*M[1][3]);
	}

	cvMatMul(t2, dM_i, dM_t);
	cvMatMul(dM_t, t1, dM_c);
	cvMatMul(motion, dM_c, motion);

	delete [] M;
	delete [] Mi;
}

float CImageAlign::ComputeUVcorrection(const CvRect& rect, CvMat* old_motion, CvMat* new_motion)
{
	//! Warp four corners
	int i;
	float w1[4][2], w2[4][2], s = 0.0;
    float px[2] = {(float)rect.x, (float)(rect.x + rect.width  - 1)};
	float py[2] = {(float)rect.y, (float)(rect.y + rect.height - 1)};
    
	for (i = 0; i < 4; i++) {
		float p[2] = {px[(i==1)||(i==2)], py[i >> 1]};
		WarpPoint(old_motion, p, w1[i]);
		WarpPoint(new_motion, p, w2[i]);
		
		p[0] = w1[i][0] - w2[i][0];
		p[1] = w1[i][1] - w2[i][1];
		s += (p[0]*p[0] + p[1]*p[1]);
    }

	return sqrt(s/8.0f);
}

void CImageAlign::ScaleMotionUp(int level, CvMat* motion)
{
	if (level == 0) return;
	float s = (float)(1 << level);
	
	CV_MAT_ELEM(*motion, float, 0, 3) *= s;
	CV_MAT_ELEM(*motion, float, 1, 3) *= s;
	CV_MAT_ELEM(*motion, float, 3, 0) /= s;
	CV_MAT_ELEM(*motion, float, 3, 1) /= s;
}

void CImageAlign::ScaleMotionDown(int level, CvMat* motion)
{
	if (level == 0) return;
	float s = (float)(1 << level);

	CV_MAT_ELEM(*motion, float, 0, 3) /= s;
	CV_MAT_ELEM(*motion, float, 1, 3) /= s;
	CV_MAT_ELEM(*motion, float, 3, 0) *= s;
	CV_MAT_ELEM(*motion, float, 3, 1) *= s;
}

void CImageAlign::WarpPoint(CvMat* M, float p[2], float w[2])
{
	int i;
    float **A = new float*[M->rows];
	for (i=0; i < M->rows; i++) 
		A[i] = (float *)(M->data.fl + i*M->step/sizeof(float));

	float q[4], s;
	for (i=0; i < 4; i++) {
	    q[i] = A[i][0] * p[0] + A[i][1] * p[1] + A[i][2] * 0.0f + A[i][3] * 1.0f;
	}
	
	s = (q[3] > 0.0f) ? 1.0f/q[3] : (q[3] < 0.0f) ? -1.0f/q[3] : 1.0f;
	w[0] = s*q[0];
	w[1] = s*q[1];

	delete [] A;
}

void CImageAlign::WarpPoint(CvMat* M, CvMat* p, CvMat* w)
{
	cvMatMul(M, p, w);
	float s = CV_MAT_ELEM(*w, float, 3, 0);
	if (s == 0.0) return;

	CV_MAT_ELEM(*w, float, 0, 0) /= s;
	CV_MAT_ELEM(*w, float, 1, 0) /= s;
	CV_MAT_ELEM(*w, float, 2, 0) /= s;
	CV_MAT_ELEM(*w, float, 3, 0) = 1.0;
}

// The origin of the coordinate the motion is described in is the top left of a tracker window.
// Note that, however, the ippiWarp() has the top left corner of a whole image as an origin
//
void CImageAlign::WarpBackImage(IplImage* I_whole, IplImage* I, CvRect r, CvMat* motion)
{
	//cout << "Warp ln: " << __LINE__ << endl;
	cvSetIdentity(t2);
	CV_MAT_ELEM(*t2, float, 0, 3) = (float)(r.x);
	CV_MAT_ELEM(*t2, float, 1, 3) = (float)(r.y);
	cvMatMul(motion, t2, g2);

	//above creates t2 as identity (t2 is a 4x4 matrix)
	//then sets 2 values tx and ty
	//then it multiplies motion by t2 and gets g2 (another 4x4)

	//cout << "Warp ln: " << __LINE__ << endl;
	
    //! Set up the 3x3 coefficients matrix
    double coeffs[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            coeffs[i][j] = cvmGet(g2, i+(i==2), j+(j==2));
        }
    }
	//TODO coeffs matrix
	//Mat* coeffsCV = cvCreateMat(3,3, CV_32F);	
	
		//Example from OpenCV Documentation
       /*
	*double m[3][3] = {{a, b, c}, {d, e, f}, {g, h, i}};
	*Mat M = Mat(3, 3, CV_64F, m).inv();
	*/

	cv::Mat coeffsCV = cv::Mat(3,3, CV_64F, coeffs).inv();	

/*
	Mat coeffsCV(3,3,CV_32F)
	for(int i=0; i <3; i++) {
		for(int j=0; j < 3; j++) {
			cvmSet(   coeffsCV, i, j,  coeffs[i][j]  );
		}
	}
	const CvMat* mapMatrix = coeffsCV;
*/

	//cout << "Warp ln: " << __LINE__ << endl;
	//! Get a warped-back image I from I_whole
	//Ipp32f* src = (Ipp32f *)I_whole->imageData;
	//const Mat* src = (Mat*)I_whole->imageData;
	cv::Mat src(I_whole);

	
	int srcStep = I_whole->widthStep;
	int dstStep = I->widthStep;
	
	

	IppiSize size = {I_whole->width, I_whole->height};
	IppiRect src_roi = {0, 0, I_whole->width, I_whole->height};
	IppiRect dst_roi = {0, 0, I->width, I->height};

	//Ipp32f* dst = (Ipp32f *)I->imageData;
	//Mat* dst = (CvMat*)I->imageData;
	cv::Mat dst(I);
	
	//cout << "Warp ln: " << __LINE__ << endl;

	//make the size variable for the output image 
	//cvSize dsize;

	int dWidth = (int) I->width;
	int dHeight = (int) I->height;
	CvSize dsize;
	dsize.height = dHeight;
	dsize.width = dWidth;

//OpenCV function (but we want the c version of this I presume, don't understand why...
	// src - input image
	// dst - output image that has the size dsize and the same type as src
	// M - 3x3 transformation matrix
	// dsize - size of the output image
	// flags - combination of interpolation methods (INTER_LINEAR or INTER_NEAREST) and the optional flag WARP_INVERSE_MAP, that sets M as the inverse transformation (dst -> src)
	// borderMode - pixel extrapolation method (BORDER_CONSTANT or BORDER_REPLICATE).
	// borderValue - value in case of a constant border; by default, it equals 0.
	//void warpPerspective(InputArray src, OutputArray dst, InputArray M, Size dsize, int flags=INTER_LINEAR, int borderMode=BORDER_CONSTANT, const Scalar& borderValue=Scalar())

	//warpPerspective(src, dst, coeffsCV, dsize, (CV_INTER_LINEAR) | (CV_WARP_INVERSE_MAP), IPL_BORDER_CONSTANT, cv::Scalar()); 
	warpPerspective(src, dst, coeffsCV, dsize, (CV_INTER_LINEAR)                        , IPL_BORDER_CONSTANT, cv::Scalar()); 


	//cvWarpPerspective(src, dst, mapMatrix, (CV_INTER_LINEAR) + (CV_WARP_INVERSE_MAP), cvScalarAll(0)); 
	//cout << "Warp ln: " << __LINE__ << endl;

//so the only thing that is changed in the end is the source image which remains src which is an Ipp32f*, we may need to cast that differently

//the other parameters indicate 

//what have you not used from the IPP function?
//  srcSize   srcStep  srcRoi    dstStep   dstRoi

//IPP Function
	//pSrc - Pointer to the source image origin. An array of separate pointers to each plane in case of data in planar format.
	//srcSize - Size in pixels of the source image (of the IppiRect type).
	// srcStep - Distance in bytes between starts of consecutive lines in the source image buffer.
	//srcRoi - Region of interest in the source image.
	//pDst - Pointer to the destination image origin. An array of separate pointers to each plane in case of data in planar format.
	//dstStep - Distance in bytes between starts of consecutive lines in the destination image buffer.
	//dstRoi - Region of interest in the destination image (of the IppiRect type).
	//coeffs - The perspective transform coefficients.
	//interpolation - Specifies the interpolation mode. Use one of the following values:
		// IPPI_INTER_NN - nearest neighbor interpolation
		// IPPI_INTER_LINEAR - linear interpolation
		// IPPI_INTER_CUBIC - cubic interpolation

	//IppStatus ippiWarpPerspectiveBack_<mod>(const Ipp<datatype>* pSrc, IppiSize srcSize, int srcStep, IppiRect srcRoi, Ipp<datatype>* pDst, int dstStep, IppiRect dstRoi, const double coeffs[3][3], int interpolation);

    //ippiWarpPerspectiveBack_32f_C1R(src, size, srcStep, src_roi, dst, dstStep, dst_roi, coeffs, IPPI_INTER_LINEAR);	//SEG FAULT HERE

}

float CImageAlign::ComputeError(int level)
{
	int len = (m_imgT[level]->width)*(m_imgT[level]->height);
	
	PDTYPE sum;
	PDTYPE* T  = (PDTYPE *)(m_imgT[level]->imageData);
	PDTYPE* I  = (PDTYPE *)(m_imgI[level]->imageData);
	PDTYPE* E  = (PDTYPE *)(m_imgErr[level]->imageData);
	PDTYPE* Ew = (PDTYPE *)(m_imgWerr[level]->imageData);
	PDTYPE* W  = (PDTYPE *)(m_imgWm[level]->imageData);

	
	if ((m_alignMode[level] == ALIGN_AFFINE_PHOTO) &&
	   !(fabs(m_photo_delta) < 0.00001f && fabs(m_photo_lambda - 1.0f) < 0.0000001f)) 
	{
		IppsSubC_I(m_photo_delta, I, len);
		IppsThreshold_LT_I(I, len, 0.0f);
		IppsDivC_I(m_photo_lambda, I, len);
	}

	// I_w' = 1/lambda*(I_w - delta)
	// E = I_w' - T - m_bias
	IppsSub(T, I, E, len);
//	IppsSubC_I(m_bias, E, len);

	// Ew = Err*W
	IppsMul(E, W, Ew, len);
	IppsSum(Ew, len, &sum);
	m_sum_werr[level] = (float)sum;
	
	// werr2 = Err*Err*W
	IppsDotProd(Ew, E, len, &sum);
	m_sum_werr2[level] = (float)sum;
	
	// RMS_error
    return sqrt(m_sum_werr2[level]/m_sum_weight[level]);
}

void CImageAlign::ComputeGradient(int level)
{
	int len = (m_imgT[level]->width)*(m_imgT[level]->height);

	PDTYPE* T  = (PDTYPE *)(m_imgT[level]->imageData);
	PDTYPE* Gx = (PDTYPE *)(m_imgGx[level]->imageData);
	PDTYPE* Gy = (PDTYPE *)(m_imgGy[level]->imageData);
	PDTYPE* W  = (PDTYPE *)(m_imgWm[level]->imageData);

	IppsZero(Gx, len);
	IppsZero(Gy, len);
	IppsZero(W, len);

	//! Gx(i, j) = 0.5*( I(i+1, j) - I(i-1, j) )
	//! Gy(i, j) = 0.5*( I(i, j+1) - I(i, j-1) )
	Ipp32f kernel[3] = {0.5, 0.0, -0.5};
	IppiSize roi = {m_imgT[level]->width - 2, m_imgT[level]->height - 2};
	//int offset = m_imgT[level]->widthStep + sizeof(Ipp32f);
	int offset = m_imgT[level]->width + 1;
	int widthStep = m_imgT[level]->widthStep;

	IppiFilterRow   (T + offset, widthStep, 
					 Gx + offset, widthStep, roi, kernel, 3, 1);

	IppiFilterColumn(T + offset, widthStep, 
					 Gy + offset, widthStep, roi, kernel, 3, 1);

	int mode = 1;
	if (mode == 0) {
		//! W = |Gx| + |Gy|
		PDTYPE* tmp = IppsMalloc(len);
		IppsAbs(Gx, tmp, len);
		IppsAbs(Gy, W, len);
		IppsAdd_I(tmp, W, len);
		IppsMulC_I(0.25f, W, len);
		IppsFree(tmp);
	}
	else {
		//! W = Gaussian distribution
		IppsSet(1.0f, W, len);

		int w = m_templ_size[level].width;
		int h = m_templ_size[level].height;
		float sigma = (float)(w/2)*(w/2);
		for (int i=0; i < w; i++) {
			for (int j=0; j < h; j++) {
				float d2 = (float)(i - w/2)*(i - w/2) + (j - h/2)*(j - h/2);
				W[i*w + j] = 10*exp(-d2/sigma);
			}
		}
	}

	PDTYPE sum;
	IppsSum(W, len, &sum);
	m_sum_weight[level] = (float)sum;
}

/*///////////////////////////////////////////////////////////////////////////////////
A[8][8]
=
0		1		2			3			4			5			6			7
gx2		Gx*Gy	x*gx2		x*Gx*Gy		y*gx2		y*Gx*Gy		I*Gx		Gx
		gy2		x*Gx*Gy		x*gy2		y*Gx*Gy		y*gy2		I*Gy		Gy
				x*x*gx2		x*x*Gx*Gy	x*y*gx2		x*y*Gx*Gy	x*I*Gx		x*Gx
							x*x*gy2		x*y*Gx*Gy	x*y*gy2		x*I*Gy		x*Gy
										y*y*gx2		y*y*Gx*Gy	y*I*Gx		y*Gx
                                                   	y*y*gy2		y*I*Gy		y*Gy
                                                                I2			I
																			w
= [grad(I) dW/dp]^T [grad(I) dW/dp] 
   where  [grad(I) dW/dp] = [Gx  Gy  x*Gx  x*Gy  y*Gx  y*Gy  I  1]

A[1][2] = A[0][3]	A[1][4] = A[0][5]
A[3][4] = A[2][5]
*/
void CImageAlign::ComputeHessian(int level, CvMat* H, CvMat* invH)
{
	int i, j, msize = m_modelSize[m_alignMode[level]];
	
	PDTYPE **A = new PDTYPE* [msize];
	for (i=0; i < msize; i++) A[i] = new PDTYPE [msize];

	PDTYPE* T   = (PDTYPE *)(m_imgT[level]->imageData);
	PDTYPE* Gx  = (PDTYPE *)(m_imgGx[level]->imageData);
	PDTYPE* Gy  = (PDTYPE *)(m_imgGy[level]->imageData);
	PDTYPE* W   = (PDTYPE *)(m_imgWm[level]->imageData);
	PDTYPE* dx  = (PDTYPE *)(m_imgDx[level]->imageData);
	PDTYPE* dy  = (PDTYPE *)(m_imgDy[level]->imageData);
	PDTYPE* dxy = (PDTYPE *)(m_imgDxy[level]->imageData);
	PDTYPE* dx2 = (PDTYPE *)(m_imgDx2[level]->imageData);
	PDTYPE* dy2 = (PDTYPE *)(m_imgDy2[level]->imageData);
	
	int len = (m_imgT[level]->width) * (m_imgT[level]->height);
	PDTYPE* Gwx  = IppsMalloc(len);
	PDTYPE* Gwy  = IppsMalloc(len);
	PDTYPE* Gwx2 = IppsMalloc(len);
	PDTYPE* Gwy2 = IppsMalloc(len);
	PDTYPE* Gwxy = IppsMalloc(len);
	PDTYPE* TGwx = IppsMalloc(len);
	PDTYPE* TGwy = IppsMalloc(len);
	PDTYPE* Tw   = IppsMalloc(len);
	PDTYPE* tmp  = IppsMalloc(len);
	
	// Precompute weighted gradients
	IppsMul(W, Gx,  Gwx, len);
	IppsMul(W, Gy,  Gwy, len);
	IppsMul(Gx, Gwx, Gwx2, len);
	IppsMul(Gy, Gwy, Gwy2, len);
	IppsMul(Gx, Gwy, Gwxy, len);

	if (m_alignMode[level] >= ALIGN_TRANSLATION) {
		IppsSum(Gwx2, len, &A[0][0]);	// A[0][0] = sum(w*gx2)
		IppsSum(Gwxy, len, &A[0][1]);	// A[0][1] = sum(w*gxy)
		IppsSum(Gwy2, len, &A[1][1]);	// A[1][1] = sum(w*gy2)
		// Symmetry
		A[1][0] = A[0][1];
	}

	if (m_alignMode[level] >= ALIGN_AFFINE_PHOTO) 
	{
		IppsMul(T, Gwx, TGwx, len);
		IppsMul(T, Gwy, TGwy, len);
		IppsMul(T, W,  Tw,  len);

		//-------------------------------------------------------------------
		IppsDotProd(dx, Gwx2, len, &A[0][2]);		// A[0][2] = sum(dx*Gwx2)
		IppsDotProd(dx, Gwxy, len, &A[0][3]);		// A[0][3] = sum(dx*Gwxy)
		IppsDotProd(dy, Gwx2, len, &A[0][4]);		// A[0][4] = sum(dy*Gwx2)
		IppsDotProd(dy, Gwxy, len, &A[0][5]);		// A[0][5] = sum(dy*Gwxy)
		IppsSum(TGwx, len, &A[0][6]);				// A[0][6] = sum(T*Gwx)
		IppsSum(Gwx,  len, &A[0][7]);				// A[0][7] = sum(Gwx)

		//-------------------------------------------------------------------
		IppsDotProd(dx, Gwy2, len, &A[1][3]);		// A[1][3] = sum(dx*Gwy2)
		IppsDotProd(dy, Gwy2, len, &A[1][5]);		// A[1][5] = sum(dy*Gwy2)
		IppsSum(TGwy, len, &A[1][6]);				// A[1][6] = sum(T*Gwy)
		IppsSum(Gwy,  len, &A[1][7]);				// A[1][7] = sum(Gwy)

		//-------------------------------------------------------------------
		IppsDotProd(dx2, Gwx2, len, &A[2][2]);		// A[2][2] = sum(dx2*Gwx2)
		IppsDotProd(dx2, Gwxy, len, &A[2][3]);		// A[2][3] = sum(dx2*Gwxy)
		IppsDotProd(dxy, Gwx2, len, &A[2][4]);		// A[2][4] = sum(dxy*Gwx2)
		IppsDotProd(dxy, Gwxy, len, &A[2][5]);		// A[2][5] = sum(dxy*Gwxy)
		IppsDotProd(dx,  TGwx, len, &A[2][6]);		// A[2][6] = sum(dx*T*Gwx)
		IppsDotProd(dx,  Gwx,  len, &A[2][7]);		// A[2][7] = sum(dx*Gwx)

		//-------------------------------------------------------------------
		IppsDotProd(dx2, Gwy2, len, &A[3][3]);		// A[3][3] = sum(dx2*Gwy2)
		IppsDotProd(dxy, Gwy2, len, &A[3][5]);		// A[3][5] = sum(dxy*Gwy2)
		IppsDotProd(dx,  TGwy, len, &A[3][6]);		// A[3][6] = sum(dx*T*Gwy)
		IppsDotProd(dx,  Gwy,  len, &A[3][7]);		// A[3][7] = sum(dx*Gwy)

		//-------------------------------------------------------------------
		IppsDotProd(dy2, Gwx2, len, &A[4][4]);		// A[4][4] = sum(dy2*Gwx2)
		IppsDotProd(dy2, Gwxy, len, &A[4][5]);		// A[4][5] = sum(dy2*Gwxy)
		IppsDotProd(dy,  TGwx, len, &A[4][6]);		// A[4][6] = sum(dy*T*Gwx)
		IppsDotProd(dy,  Gwx,  len, &A[4][7]);		// A[4][7] = sum(dy*Gwx)

		//-------------------------------------------------------------------
		IppsDotProd(dy2, Gwy2, len, &A[5][5]);		// A[5][5] = sum(dy2*Gwy2)
		IppsDotProd(dy,  TGwy, len, &A[5][6]);		// A[5][6] = sum(dy*T*Gwy)
		IppsDotProd(dy,  Gwy,  len, &A[5][7]);		// A[5][7] = sum(dy*Gwy)

		//-------------------------------------------------------------------
		IppsDotProd(T, Tw, len, &A[6][6]);			// A[6][6] = sum(T*Tw)
		IppsSum(Tw, len, &A[6][7]);					// A[6][7] = sum(Tw)		
		A[7][7] = (PDTYPE)m_sum_weight[level];		// A[7][7] = sum(W);

		//-------------------------------------------------------------------
		A[1][2] = A[0][3];	
		A[1][4] = A[0][5];	
		A[3][4] = A[2][5];

		// Symmetric matrix
		for (i = 1; i < 8; i++) {
			for (j = 0; j < i; j++) {
				A[i][j] = A[j][i];
			}
		}
	}

	cvSetZero(H);
	if (m_alignMode[level] >= ALIGN_AFFINE_PHOTO) {
		// Photo scale
		for (i=0; i < 7; i++) {
			for (j=0; j < 7; j++) {
				CV_MAT_ELEM(*H, float, i, j) = (float)A[i][j] * m_photo_scale*m_photo_scale;
			}
			CV_MAT_ELEM(*H, float, i, 7) = (float)A[i][7] * m_photo_scale;
			CV_MAT_ELEM(*H, float, 7, i) = (float)A[7][i] * m_photo_scale;
		}
		CV_MAT_ELEM(*H, float, 7, 7) = (float)A[7][7];
	}
	else {
		for (i=0; i < msize; i++) {
			for (j=0; j < msize; j++) {
				CV_MAT_ELEM(*H, float, i, j) = (float)A[i][j];
			}
		}
	}
	cvInvert(H, invH, CV_SVD_SYM);

#ifdef __DEBUG__
	{
		cout << "Hessian (" << level << ")" << endl;
		for (int i=0; i < H->rows; i++)  {
			for (int j=0; j < H->cols; j++)  {
				cout << setprecision(1) << setiosflags(ios::fixed) << setw(10);
				cout << cvmGet(H, i, j);
			}
			cout << endl;
		}
		cout << endl;
	}
#endif

	// Free unused memories
	for (i=0; i < msize; i++) delete [] A[i];
	delete [] A;

	IppsFree(Gwx);
	IppsFree(Gwy);
	IppsFree(Gwx2);
	IppsFree(Gwy2);
	IppsFree(Gwxy);
	IppsFree(TGwx);
	IppsFree(TGwy);
	IppsFree(Tw);
	IppsFree(tmp);
}

void CImageAlign::ComputeB(int level, CvMat* bm)
{
	int i, msize = m_modelSize[m_alignMode[level]];
	PDTYPE *b = new PDTYPE [msize];

	int len = (m_imgT[level]->width) * (m_imgT[level]->height);
	PDTYPE* EwGx = IppsMalloc(len);
	PDTYPE* EwGy = IppsMalloc(len);

	PDTYPE* T  = (PDTYPE *)(m_imgT[level]->imageData);
	PDTYPE* Ew = (PDTYPE *)(m_imgWerr[level]->imageData);
	PDTYPE* Gx = (PDTYPE *)(m_imgGx[level]->imageData);
	PDTYPE* Gy = (PDTYPE *)(m_imgGy[level]->imageData);
	PDTYPE* dx = (PDTYPE *)(m_imgDx[level]->imageData);
	PDTYPE* dy = (PDTYPE *)(m_imgDy[level]->imageData);

	if (m_alignMode[level] >= ALIGN_TRANSLATION) {
		IppsMul(Ew, Gx, EwGx, len);	IppsSum(EwGx, len, &b[0]);	// b[0] = sum(Ew*Gx)
		IppsMul(Ew, Gy, EwGy, len);	IppsSum(EwGy, len, &b[1]);	// b[1] = sum(Ew*Gy)
	}

	if (m_alignMode[level] >= ALIGN_AFFINE_PHOTO) {
		IppsDotProd(dx, EwGx, len, &b[2]);			// b[2] = sum(Dx*EwGx)
		IppsDotProd(dx, EwGy, len, &b[3]);			// b[3] = sum(Dx*EwGy)
		IppsDotProd(dy, EwGx, len, &b[4]);			// b[4] = sum(Dy*EwGx)
		IppsDotProd(dy, EwGy, len, &b[5]);			// b[5] = sum(Dy*EwGy)
		IppsDotProd(Ew, T, len, &b[6]);				// b[6] = sum(Ew*I)
		IppsSum(Ew, len, &b[7]);					// b[7] = sum(Ew)
	}
	
	cvSetZero(bm);
	if (m_alignMode[level] >= ALIGN_AFFINE_PHOTO) {
		// Photo scale
		for (int i=0; i < 7; i++) 
			CV_MAT_ELEM(*bm, float, i, 0) = (float)b[i] * m_photo_scale*m_photo_scale;
		CV_MAT_ELEM(*bm, float, 7, 0) = (float)b[7] * m_photo_scale;
	}
	else {
		for (i=0; i < msize; i++)
			CV_MAT_ELEM(*bm, float, i, 0) = (float)b[i];
	}

#ifdef __DEBUG__
	{
		cout << "B (" << level << ")" << endl;
		for (int i=0; i < bm->rows; i++)  {
			cout << setprecision(1) << setiosflags(ios::fixed) << setw(10);
			cout << cvmGet(bm, i, 0);
		}
		cout << endl << endl;
	}
#endif

	delete [] b;
	IppsFree(EwGx);
	IppsFree(EwGy);
}

/////////////////////////////////////////////////////////////////////////////////////////////
//
int CImageAlign::UpdateStatus()
{
	m_bias += (m_sum_werr[0]/m_sum_weight[0]);

	m_NCC = ComputeNormalizedCorrelation(m_imgT[0], m_imgI[0]);
	m_shear = ComputeShearAmount(m_motion);

	bool inr = (m_fp[0] > m_range_lower[0]) && (m_fp[0] < m_range_upper[0]) && 
			   (m_fp[1] > m_range_lower[1]) && (m_fp[1] < m_range_upper[1]);
	bool rms = m_RMS_error < m_RMS_error_threshold;
	bool sha = m_shear > m_shear_threshold;
	bool ncc = m_NCC > m_NCC_threshold;

	// Check failure modes
	if (!inr) {
		m_status |= TRACK_FAIL_BOUND;
	}
	if (!rms) {
		m_status |= TRACK_FAIL_RMS;
	}
	if (!sha) {
		m_status |= TRACK_FAIL_SHRINK;
	}
	if (!ncc) {
		m_status |= TRACK_FAIL_NCC;
	}

	m_RMS_error = CLAMP(m_RMS_error, 1e3f);
	m_bias = CLAMP(m_bias, 1e3f);
	m_NCC = CLAMP(m_NCC, 1e3f);
	m_shear = CLAMP(m_shear, 1e3f);

	if (m_status & TRACK_FAIL) return m_status;

	// Is it a good time to update the template?
	m_status = TRACK_SUCCESS;
	rms = m_RMS_error > 0.5f*m_RMS_error_threshold;
	sha = m_shear < 2.0f*m_shear_threshold;
	ncc = m_NCC < 0.5f;

	//if (rms || sha || ncc)
	if (sha || ncc)
		m_status = TRACK_SUCCESS_TEMPL_UPDATE;

	return m_status;
}

float CImageAlign::ComputeNormalizedCorrelation(const IplImage* T, IplImage* I)
{
	assert(T->width == I->width && T->height == I->height); 
	int border = 4;
/*	
	// This is not working well ... I don't know why.
	int offset = T->widthStep*border + sizeof(Ipp32f)*border;
	Ipp32f corr;
	IppiSize roi = {T->width - 2*border, T->height - 2*border};
	
	ippiCrossCorrValid_NormLevel_32f_C1R(
		(Ipp32f *)(T->imageData + offset), T->widthStep, roi,
		(Ipp32f *)(I->imageData + offset), I->widthStep, roi,
		(Ipp32f *)&corr, sizeof(Ipp32f));
	
	return corr;
*/

	int i, j;
	int height = T->height;
	int width  = T->width;
	
	float avg1 = 0.0f, avg2 = 0.0f;
	float n = (float)((height-2*border)*(width-2*border));
	
	for (j=border; j < height - border; j++) {
		for (i=border; i < width - border; i++) {
			avg1 += (float)CV_IMAGE_ELEM(T, PDTYPE, j, i);
			avg2 += (float)CV_IMAGE_ELEM(I, PDTYPE, j, i);
		}
	}
	avg1 /= n;
	avg2 /= n;

	// compute the correlation
	float corr = 0.0f, n1 = 0.0f, n2 = 0.0f;
	float p1, p2;
	
	for (j=border; j < height - border; j++) {
		for (i=border; i < width - border; i++) {
			p1 = (float)CV_IMAGE_ELEM(T, PDTYPE, j, i);
			p2 = (float)CV_IMAGE_ELEM(I, PDTYPE, j, i);
			corr += (p1 - avg1) * (p2 - avg2);
			n1   += (p1 - avg1) * (p1 - avg1); 
			n2   += (p2 - avg2) * (p2 - avg2); 
		}
	}

	float d = sqrt(n1 * n2);
	return fabs( corr/(d + (d == 0.0)) );
}

float CImageAlign::ComputeShearAmount(CvMat* motion)
{
	// Warp four corners and compute its area
	int i;
	float w[4][2], area1, area2 = 0.0f;
    float px[2] = {(float)m_roi[0].x, (float)(m_roi[0].x + m_roi[0].width  - 1)};
	float py[2] = {(float)m_roi[0].y, (float)(m_roi[0].y + m_roi[0].height - 1)};
    
	for (i=0; i < 4; i++) {
		float p[2] = {px[(i==1)||(i==2)], py[i >> 1]};
		WarpPoint(motion, p, w[i]);
    }

	for (i=0; i < 4; i++) {
		int j = (i+1) % 4;
		area2 += w[i][0] * w[j][1] - w[j][0] * w[i][1]; 
	}

	area2 = 0.5f * area2;
	area2 = (area2 < 0.0f) ? -area2 : area2;

	area1 = (float)m_roi[0].width * m_roi[0].height;

	float m1, m2;
	if (area1 > area2) {
		m1 = area2;
		m2 = area1;
	}
	else {
		m1 = area1;
		m2 = area2;
	}

	if (m2 == 0.0f) return 0.0;

	return m1/m2;
}
