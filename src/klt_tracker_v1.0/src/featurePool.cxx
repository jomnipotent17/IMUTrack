/**
* CMU MSUAV Project (Micro and Small Unmanned Aerial Vehicle)
*          
* \file  featurePool.h
* \brief implementation of CFeature2D, CFeature2DTrack, and CFeature2DPool
* \date  15-Jun-2007
*
* \author Myung Hwangbo (myung@cs.cmu.edu) and Jun-Sik Kim (kimjs@cs.cmu.edu)
*
* Copyright (c) 2007-2009 Myung Hwangbo and Jun-Sik Kim
* Robotics Institute, Carnegie Mellon University
*
*/

#include <ipp.h>
#include <algorithm>
#include "featurePool.h"

#include <cuda_runtime.h>
#include "cuda_feature_select.h"

#define CUDA_SAFE_CALL(call) {                                         \
	cudaError err = call;                                              \
	if (cudaSuccess != err) {                                          \
		fprintf(stderr, "Cuda error in file '%s' in line %i : %s.\n",  \
			__FILE__, __LINE__, cudaGetErrorString( err) );            \
		exit(EXIT_FAILURE);                                            \
    }} 
    
extern "C" int InitCUDA(int imageWidth, int imageHeight, int pyrLevel, int templateSize, int maxIteration, int maxLinesearch);
extern "C" bool ExitCUDA(void);
extern cudaArray *Image1[5], *Image2[5];
extern "C" void Caller_Tracker_Registration(float* fx, float *fy, int numfeatures, int *indexmap);
extern "C" void Caller_Align(cudaArray *Image2, float* motion, float *residual, float *vec_invH);

////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//! CFeature2DPool class
//!
CFeature2DPool::CFeature2DPool(CvSize imgSize, int templSize, int pyrLevel, int maskSize, bool gpu, CCameraModel* pCameraModel)
	: m_imgSize(imgSize), m_templateSize(templSize), m_pyrLevel(pyrLevel), m_maskSize(maskSize), b_gpu(gpu), m_pCameraModel(pCameraModel)
{
	reserve(50000);

	m_frame = 0;
	m_pyr = NULL;

	// Memory alloc for OPENCV feature selection, see cvGoodFeaturesToTrack()
	m_imgEig  = cvCreateImage(m_imgSize, IPL_DEPTH_32F, 1);
	m_imgTmp  = cvCreateImage(m_imgSize, IPL_DEPTH_32F, 1);
	m_imgMask = cvCreateImage(m_imgSize, IPL_DEPTH_8U, 1);

	if (b_gpu)
	{
		for (int i=0;i < MAX_NUM_FEATURES;i++) {
			m_cuda_index_map_global[i] = -1;	// index mapping from cuda mem to global mem
			m_cuda_index_map_lost[i] = 1;		// index of lost features of cuda mem (lost 1, tracked 0)
			m_cuda_index_map[i] = 0;			// index map from new features to cuda mem
		}

		m_pyrLevel = InitCUDA(m_imgSize.width, m_imgSize.height, m_pyrLevel, templSize, 3, 5);
	}
}

CFeature2DPool::~CFeature2DPool()
{
	for (int i=0; i < (int)size(); i++) delete at(i);
	
	map<int, IplImage*>::iterator im = m_mapImage.begin();
	for ( ; im != m_mapImage.end(); im++) {
		cvReleaseImage(&im->second);
	}

	// Memory release for OPENCV feature selection, see cvGoodFeaturesToTrack()
	cvReleaseImage(&m_imgEig);
	cvReleaseImage(&m_imgTmp);
	cvReleaseImage(&m_imgMask);

	if (b_gpu) {
		ExitCUDA();
	}

	//if (m_pyr) delete m_pyr; // added by HB
}

int CFeature2DPool::AddImage(int frame, IplImage* img)
{
	m_frame = frame;
	m_img = img;
	m_mapImage[frame] = img;

	// Create a new pyramid image
	if (m_pyr) delete m_pyr;
	m_pyr = new CImagePyramid(img, m_pyrLevel);

	return (int)m_mapImage.size();
}

int CFeature2DPool::PurgeImage(int frame, int lag_dist)
{
	map<int, IplImage*>::iterator im;

	for (im = m_mapImage.begin(); im != m_mapImage.end(); im++) {
		if (im->first < frame - lag_dist) {
			cvReleaseImage(&im->second);
			m_mapImage.erase(im++);
		}
	}

	return (int)m_mapImage.size();
}

const IplImage* CFeature2DPool::GetImage(int frame)
{
	if (frame < 0) return m_mapImage[m_frame]; // last image
	else {
		map<int, IplImage*>::const_iterator im = m_mapImage.find(frame);
		if (im == m_mapImage.end()) return NULL;
		else return im->second;
	}
}

CFeature2DTrack* CFeature2DPool::GetFeatureTrackPtr(int id)
{
	if (id < 0 || id >= (int)size()) return NULL;
	return (*this)[id];
}

CFeature2DTrack& CFeature2DPool::GetFeatureTrack(int id)
{
	assert(id >= 0 && id < (int)size());
	return *(*this)[id];
}

CFeature2D* CFeature2DPool::GetFeaturePtr(int id, int frame)
{
	if (id < 0 || id >= (int)size()) return NULL;
	return (*this)[id]->GetFeaturePtr(frame);
}

CFeature2D& CFeature2DPool::GetFeature(int id, int frame)
{
	assert(id >= 0 && id < (int)size());
	return (*this)[id]->GetFeature(frame);
}

void CFeature2DPool::SetOutlier(int id, int inout)
{
	m_mapOutliers[id] = inout;
}

int CFeature2DPool::GetOutlier(vector<int>& vid)
{
	vid.clear();
	map<int, int>::const_iterator im;
	for (im = m_mapOutliers.begin(); im != m_mapOutliers.end(); im++) {
		if (im->second == 1) {
			vid.push_back(im->first);
		}
	}

	return (int)vid.size();
}

bool CFeature2DPool::IsOutlier(int id)
{
	map<int, int>::const_iterator im = m_mapOutliers.find(id);
	if (im == m_mapOutliers.end()) {
		return false;
	}
	else {
		return (im->second == 1);
	}
}

void CFeature2DPool::ComputeSelectionMask(int templ_size, int mask_size, const list<int>& activeID, IplImage* img_mask)
{
    //if (!img_mask) return;
    
	int width  = img_mask->width;
	int height = img_mask->height;

	int hw = templ_size >> 1;

	//int x_border = (hw+1) << (MAX_PYRAMID_LEVEL - 1);
	//int y_border = (hw+1) << (MAX_PYRAMID_LEVEL - 1);
	int x_border = (hw+1) << (m_pyrLevel - 1);
	int y_border = (hw+1) << (m_pyrLevel - 1);

	int wx = mask_size;
	int wy = mask_size;

	int bottom_offset = 0;

	cvSetZero(img_mask);
	cvSetImageROI(img_mask, cvRect(x_border, y_border, m_imgSize.width - 2*x_border, m_imgSize.height - 2*y_border - bottom_offset));
	cvSet(img_mask, cvScalar(255));

	// Generate the mask from current feature points
	//cout << "size = " << m_listActiveID.size() << endl;
	for (list<int>::const_iterator it = activeID.begin(); it != activeID.end(); it++) 
	{
		int id = *it;
		CFeature2D* fp = GetFeaturePtr(id);	//cout << id << endl;

		CvPoint p1, p2;
		p1.x = int(fp->m_x - wx/2 + 0.5);	p1.y = int(fp->m_y - wy/2 + 0.5);
		p2.x = int(fp->m_x + wx/2 + 0.5);	p2.y = int(fp->m_y + wy/2 + 0.5);

		p1.x = max(0, min(p1.x, width -1));
		p1.y = max(0, min(p1.y, height-1));
		p2.x = max(0, min(p2.x, width -1));
		p2.y = max(0, min(p2.y, height-1));
		if (p1.x == p2.x || p1.y == p2.y) continue;

		cvSetImageROI(img_mask, cvRect(p1.x, p1.y, p2.x-p1.x, p2.y-p1.y));
		cvSetZero(img_mask);
	}

	// Don't forget to reset the ROI of imgFeatureMask before running cvGoodFeaturesToTrack()
	cvResetImageROI(img_mask);
}

int CFeature2DPool::Select(int frame, int min_threshold, int max_num, IplImage* gimgf, bool verbose)
{
	assert(min_threshold <= max_num && frame >= 0);

	int f_num = (int)m_listActiveID.size();
	if (f_num >= min_threshold) return 0;
	
	int n_new = max_num - f_num;
	int block_size = 3;
	double min_distance = 20;
	double quality_level = 0.005;

	//! Select new features in the area where no other features exist nearby.
	CvPoint2D32f* corners = new CvPoint2D32f[n_new];
	ComputeSelectionMask(m_templateSize, m_maskSize, m_listActiveID, m_imgMask); 
	cvGoodFeaturesToTrack(gimgf, m_imgEig, m_imgTmp, corners, &n_new, quality_level, min_distance, m_imgMask, block_size);

	//! Add new features to the feature pool
	for (int i=0; i < n_new; i++) {
		float saliency = CV_IMAGE_ELEM(m_imgEig, float, (int)corners[i].y, (int)corners[i].x);
		int nid = Add(corners[i].x, corners[i].y, saliency, frame, m_pyr);
	}
	delete [] corners;

	if (0) {
		cout << "CFeature2DPool::Select() : frame = " << frame 
			<< ", f_num = " << f_num << ", new_num = " << n_new << ", total_num = " << (int)m_listActiveID.size() << endl;
	}

	m_stat.time_select = 0;
	m_stat.n_new = n_new;
	m_mapStat[frame] = m_stat;

	//! Update activeID map so that newly selected features are included.
	m_mapActiveID[frame] = m_listActiveID;

	return n_new;
}

int CFeature2DPool::Add(float x, float y, float saliency, int frame, CImagePyramid* newPyr)
{
	//! 1. A unique ID is assigned to a new feature tracker
	int new_id = (int)size();

	//! 2. Create a new feature tracker
	CFeature2DTrack *pTrack = new CFeature2DTrack(x, y, saliency, frame, new_id, m_templateSize, newPyr, m_pCameraModel);

	//! 3. Append to the feature pool
	push_back(pTrack);

	//! 4. Append the new ID to the active feature list
	m_listActiveID.push_back(new_id);

	return new_id;
}

int CFeature2DPool::Purge(int frame, int lag_dist)
{
	PurgeImage(frame, lag_dist);

	//! Note that it purges features that are lost at least "lag_dist" ago.
	//! Be cautious that the purged features are not completely deleted from the feature pool
	//! because erasing elements in std::vector container is expensive.
	//! Instead the memories that they have used are deallocated.

	int num_purged = 0;
	
	list<int>::iterator it = m_listLostID.begin();
	while (it != m_listLostID.end()) 
	{
		int id = (int)(*it);
		int d = frame - at(id)->GetEndFrame();
		if (d > lag_dist) {
			at(id)->~CFeature2DTrack();
			at(id) = NULL;
			it = m_listLostID.erase(it);

			num_purged++;
		}
		else {
			++it;
		}
	}

	return num_purged;
}

void CFeature2DPool::Track(int frame, int timestamp, IplImage* gimgf, double* Hmg)
{
	//! Tracking, updating template, and refreshing active ID list happen at the same time.
	//! This is a single thread version that aligns all the active features sequentially.

	//! Predictive Homography
	CvMat* H = NULL;
	if (Hmg != NULL) {
		H = cvCreateMat(4, 4, CV_32F);
		cvSetIdentity(H);
		for (int i=0; i < 3; i++) {
			for (int j=0; j < 3; j++) {
				cvmSet(H, (i==2) ? 3 : i, (j==2) ? 3 : j, Hmg[i + 3*j]/Hmg[8]);
			}
		}
	}

	//! Copy feature ID
	m_listLastID = m_listActiveID;

	list<int>::iterator it = m_listActiveID.begin();
	int n_total = (int)m_listActiveID.size();
	int n_success = 0;
	
	while (it != m_listActiveID.end())
	{
		CFeature2DTrack* pTracker = GetFeatureTrackPtr((int)(*it));
		// A new feature
		if (pTracker->GetStartFrame() == frame) continue;

		//cout << "track line: " << __LINE__ << endl;
		//cout << "frame #: " << frame << endl;
		/*
		if(frame == 192) {
			cout << "m_pyr = " << m_pyr << endl;
			cout << "H = " << H << endl;
			cout << "pTracker = " << pTracker << endl;
		}*/
		// Run KLT tracker
		//cout << endl << "///////////////////////   Before Run    ///////////////////////" << endl;
		int status = pTracker->Run(frame, m_pyr, H);	//TODO Seg Faulting here!
/*
		if(status & CImageAlign::TRACK_FAIL)
			cout << "Lost Tracking!" << endl; 
		cout << endl << "_______________________   After Run     _______________________" << endl;
*/

		//cout << "track line: " << __LINE__ << endl;
		// Failure in tracking
		if (status & CImageAlign::TRACK_FAIL) {
			m_listLostID.push_back((int)(*it));
			it = m_listActiveID.erase(it);
		}
		
		else {
			if (status & CImageAlign::TRACK_SUCCESS) {
				if (status == CImageAlign::TRACK_SUCCESS_TEMPL_UPDATE) {
					pTracker->UpdateTemplate();
				}
				n_success++;
			}
			else {
				cout << "Error, invalid status = " << status << endl;
			}
			++it;
		}
	}

	
	m_stat.time_select = 0;
	m_stat.n_new = 0;
	m_stat.frame = frame;
	m_stat.n_success = n_success;
	m_stat.n_total = n_total;
	m_stat.time_track = 0;
		

	m_mapStat[frame] = m_stat;
	m_mapActiveID[frame] = m_listActiveID;

	if (H != NULL) cvReleaseMat(&H);
}

list<int>& CFeature2DPool::GetLastIDs()
{
	return m_listLastID;
}

list<int>& CFeature2DPool::GetActiveIDs(int frame)
{
	if (frame == -1) return m_listActiveID;
	else {
		return m_mapActiveID[frame];
	}
}

CFeatureTrackStat& CFeature2DPool::GetStat(int frame)
{
	if (frame == -1) return m_stat;
	else {
		return m_mapStat[frame];
	}
}

void CFeature2DPool::SaveToFile(int timestamp, const char* fname, double time)
{
	static ofstream fsave(fname);
	
	CFeatureTrackStat stat = GetStat();
	//fsave << stat.n_new << "," << stat.n_success << "," << stat.n_total << endl; //first try with log file


	//slight modification to original (added commas for the csv, not sure if I need them in general though)
	fsave << stat.frame << "," << timestamp << endl;
	fsave << stat.n_new << "," << stat.n_success << "," << stat.n_total << endl;
	fsave << setprecision(2) << setiosflags(ios::fixed) << stat.time_select*1e3 << ",";
	fsave << setprecision(2) << setiosflags(ios::fixed) << stat.time_track*1e3 << endl;
	fsave << setprecision(10) << time << endl;
	
	list<int> listID = GetActiveIDs();
	fsave << listID.size() << endl;
	
	for (list<int>::iterator it = listID.begin(); it != listID.end(); it++)
	{
		int id = *it;
		CFeature2DTrack* pTrack = GetFeatureTrackPtr(id);
		if (pTrack == NULL) continue;

		int n = (int)pTrack->size();
		if (n < 1) continue;

		CFeature2D* fp = pTrack->back();
		fsave << id << ",";
		fsave << setprecision(2) << fp->m_x << ",";
		fsave << setprecision(2) << fp->m_y << ",";
		fsave << setprecision(5) << fp->m_nx << ",";
		fsave << setprecision(5) << fp->m_ny << ",";
		fsave << setprecision(5) << fp->m_rms_In << ",";
		fsave << setprecision(5) << fp->m_rms;
		fsave << endl;
	}


	//original
/*	fsave << stat.frame << "  " << timestamp << endl;
	fsave << stat.n_new << "  " << stat.n_success << "  " << stat.n_total << endl;
	fsave << setprecision(2) << setiosflags(ios::fixed) << stat.time_select*1e3 << "  ";
	fsave << setprecision(2) << setiosflags(ios::fixed) << stat.time_track*1e3 << endl;
	
	list<int> listID = GetActiveIDs();
	fsave << listID.size() << endl;
	
	for (list<int>::iterator it = listID.begin(); it != listID.end(); it++)
	{
		int id = *it;
		CFeature2DTrack* pTrack = GetFeatureTrackPtr(id);
		if (pTrack == NULL) continue;

		int n = (int)pTrack->size();
		if (n < 1) continue;

		CFeature2D* fp = pTrack->back();
		fsave << id << "\t";
		fsave << setprecision(2) << setiosflags(ios::fixed) << setw(12) << fp->m_x;
		fsave << setprecision(2) << setiosflags(ios::fixed) << setw(12) << fp->m_y;
		fsave << setprecision(5) << setiosflags(ios::fixed) << setw(12) << fp->m_nx;
		fsave << setprecision(5) << setiosflags(ios::fixed) << setw(12) << fp->m_ny;
		fsave << endl;
	}
*/

}

int CFeature2DPool::Select_GPU(int frame, int min_threshold, int max_num, IplImage* img, bool verbose)
{
	assert(min_threshold <= max_num && frame >= 0);

	int f_num = (int)m_listActiveID.size();
	if (f_num >= min_threshold) return 0;
	
	int n_new = max_num - f_num;
	int block_size = 3;
	double min_distance = 20;
	double quality_level = 0.005;

	// Select new features in the area where no current feature exists.
	CvPoint2D32f* corners = new CvPoint2D32f[n_new];
	ComputeSelectionMask(m_templateSize, m_maskSize, m_listActiveID, m_imgMask);

	CUDA_SAFE_CALL( cudaMemcpy2DToArray(Image1[0], 0, 0, img->imageData, img->width*sizeof(float), 
										img->width*sizeof(float), img->height, cudaMemcpyHostToDevice));

	cvGoodFeaturesToTrack_GPU(img, m_imgEig, m_imgTmp, corners, &n_new, quality_level, min_distance, m_imgMask, block_size);
//	cvGoodFeaturesToTrack(img, m_imgEig, m_imgTmp, corners, &n_new, quality_level, min_distance, m_imgMask, block_size);
	
	int free_index = 0;
	for (int i=0; i < n_new; i++) 
	{
		float saliency = CV_IMAGE_ELEM(m_imgEig, float, (int)corners[i].y, (int)corners[i].x);
		int nid = Add(corners[i].x, corners[i].y, saliency, frame, NULL);
		while ((m_cuda_index_map_lost[free_index] == 0) && (free_index < MAX_NUM_FEATURES)) free_index++;

		m_cuda_index_map[i]= free_index;
		m_cuda_index_map_global[free_index] = nid;
		m_cuda_index_map_lost[free_index] = 0;
		m_cuda_fx[free_index] = corners[i].x;
		m_cuda_fy[free_index] = corners[i].y;

		free_index++;
	}
	delete [] corners;
	
	for (int i=n_new; i < MAX_NUM_FEATURES; i++) m_cuda_index_map[i] = 0;

	Caller_Tracker_Registration(m_cuda_fx, m_cuda_fy, n_new, m_cuda_index_map);

	// Echo
	if (verbose) {
		cout << "CFeature2DPool::Select() : frame = " << frame 
			<< ", f_num = " << f_num << ", new_num = " << n_new << ", total_num = " << m_mapActiveID[frame].size() << endl;
	}

	m_stat.frame = frame;
	m_stat.time_select = 0;
	m_stat.n_new = n_new;
	m_mapStat[frame] = m_stat;

	// Update activeID map so that newly selected features are included.
	m_mapActiveID[frame] = m_listActiveID;

	return n_new;
}

void CFeature2DPool::Track_GPU(int frame, IplImage* img, double* Hmg)
{
	if (this->size() == 0) {
		return;
	}

	CUDA_SAFE_CALL( cudaMemcpy2DToArray(Image2[0], 0, 0, 
		img->imageData, img->width*sizeof(float), img->width*sizeof(float), img->height, cudaMemcpyHostToDevice));

	float vec_invH[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
	if (Hmg != NULL) {
		for (int i=0; i < 9; i++) vec_invH[i] = (float)Hmg[i];
	}

	Caller_Align(Image2[0], m_cuda_motion, m_cuda_residual, vec_invH);
	
	list<int>::iterator it = m_listActiveID.begin();
	int n_total = (int)m_listActiveID.size();
	int n_success = 0;

	float failthres = 400.0f;
	int boundary = 13;
		int count_fail_alignment=0;
		int count_fail_bound=0;

	for (int i=0; i < MAX_NUM_FEATURES; i++)
	{
		int id = m_cuda_index_map_global[i];
		if (id == -1) continue;

		CFeature2DTrack* pTracker = GetFeatureTrackPtr(id);

		int status = 0;
		float rms = 0.0f, ncc = 0.0f, shrink = 0.0f, bias = 0.0f;


		float px[2], nx[2] = {0.0f, 0.0f};
		px[0] = m_cuda_motion[0*MAX_NUM_FEATURES+i] + m_cuda_fx[i];
		px[1] = m_cuda_motion[1*MAX_NUM_FEATURES+i] + m_cuda_fy[i];

		if (m_pCameraModel != NULL)
			m_pCameraModel->NormalizeFromTable(px[0], px[1], nx[0], nx[1]);

		if (m_cuda_residual[i] > failthres) {
			status = CImageAlign::TRACK_FAIL_ALIGNMENT;
		}
		else if ((px[0] < boundary) || (px[0] > img->width - boundary) ||
				 (px[1] < boundary) || (px[1] > img->height- boundary)) {
			status = CImageAlign::TRACK_FAIL_BOUND;
		}
		else {
			status = CImageAlign::TRACK_SUCCESS;
		}
		
		pTracker->push_back(new CFeature2D(px[0], px[1], frame, status, rms, ncc, shrink, bias, nx[0], nx[1]));

		if (status & CImageAlign::TRACK_FAIL) {
			//m_listLostID.push_back((int)(*it));
			//it = m_listActiveID.erase(it);

			m_listLostID.push_back(id);
			m_listActiveID.remove(id);
			m_cuda_index_map_lost[i] = 1;
			m_cuda_index_map_global[i] = -1;
		}
		else {
			pTracker->SetCurrentFrame(frame);
			pTracker->SetEndFrame(frame);
			n_success++;
		}
	}

	m_stat.time_select = 0;
	m_stat.n_new = 0;

	m_stat.frame = frame;
	m_stat.n_success = n_success;
	m_stat.n_total = n_total;
	m_stat.time_track = 0;
	
	//cout << "fail_bound = " << count_fail_bound ;
	//cout << "    count_fail_alignment = " << count_fail_alignment << endl;

	m_mapStat[frame] = m_stat;
	m_mapActiveID[frame] = m_listActiveID;
}
