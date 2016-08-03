/**
* CMU MSUAV Project (Micro and Small Unmanned Aerial Vehicle)
*          
* \file  trackDisplay.cxx
* \brief display feature tracking using OpenCV highGUI
* \date  15-Apr-2009
*
* \author Myung Hwangbo (myung@cs.cmu.edu)
*
* Copyright (c) 2009 Myung Hwangbo
* Robotics Institute, Carnegie Mellon University
*
*/

#include "featurePool.h"
#include "display_tracker.h"

#define	NUM_OUTPUT	1

using namespace std;

extern CFeature2DPool* featurePool;
extern int img_width, img_height;

static CvFont cv_font = cvFont(0.8);
static vector<IplImage*> imgOut(NUM_OUTPUT);
static float sg = 1;

void display_track_prev(IplImage* out);
void convert_float_image_color(const IplImage* gimgf, IplImage* cimg);

void init_track_display(double scale)
{
	sg = (float)scale;

	CvSize size = cvSize((int)(scale*img_width), (int)(scale*img_height));
		
	for (int i=0; i < (int)imgOut.size(); i++) 
	{
		imgOut[i] = cvCreateImage(size, IPL_DEPTH_8U, 3);

		int x = (i % 3)*(size.width  + 10);
		int y = (i / 3)*(size.height + 40);
	
		char name[255];
		sprintf(name, "out_image_%02d", i);
		cvNamedWindow(name, CV_WINDOW_AUTOSIZE);	
		cvMoveWindow(name, x, y);
	}
}

void close_track_display()
{
	char name[255];
	for (int i=0; i < (int)imgOut.size(); i++) {
		sprintf(name, "out_image_%02d", i);
		cvDestroyWindow(name);
		cvReleaseImage(&imgOut[i]);
	}
}

void run_track_display()
{
	convert_float_image_color(featurePool->GetImage(), imgOut[0]);
	display_track_prev(imgOut[0]);

	char name[255];
	for (int i=0; i < (int)imgOut.size(); i++) {
		sprintf(name, "out_image_%02d", i);
		cvShowImage(name, imgOut[i]);
	}
}

void display_track_prev(IplImage* out)
{
	int hw = 1;
	float scale = (float)(out->width)/img_width;
	CvPoint p0, p1, q1;

	CFeatureTrackStat& stat = featurePool->GetStat();
	int frame = stat.frame;
	if (frame == 0) return;

	char str[256];
	sprintf(str, "Frame %03d: %3d / %3d, %4.1f%%", stat.frame, (int)stat.n_success, (int)stat.n_total, 100.0*stat.n_success/stat.n_total);
	cvRectangle(out, cvPoint(0, 14), cvPoint(240, 0), CV_RGB(0,0,0), CV_FILLED);
	cvPutText  (out, str, cvPoint(8, 12), &cv_font, CV_RGB(200, 200, 200));

	list<int> prevID = featurePool->GetActiveIDs(frame-1);
	for (list<int>::iterator it = prevID.begin(); it != prevID.end(); it++)
	{	
		CFeature2DTrack* pTrack = featurePool->GetFeatureTrackPtr(*it);
		if (pTrack == NULL || pTrack->empty()) continue;

		int n = (int)pTrack->size();
		CFeature2D* p_fp = pTrack->GetFeaturePtr(frame-1);
		CFeature2D* c_fp = pTrack->GetFeaturePtr(frame);
		
		p0.x = (int)(scale*(p_fp->m_x) + 0.5f);
		p0.y = (int)(scale*(p_fp->m_y) + 0.5f);
		p1.x = (int)(scale*(c_fp->m_x) + 0.5f);
		p1.y = (int)(scale*(c_fp->m_y) + 0.5f);

		q1.x = (int)(scale*(c_fp->m_x0) + 0.5f);
		q1.y = (int)(scale*(c_fp->m_y0) + 0.5f);

		// Failure
		if (c_fp->m_status & CImageAlign::TRACK_FAIL) {
			//cvCircle(out, p1, hw, CV_RGB(0,  0,  255), 2); // Blue
			//cvLine(out, p1, p0, CV_RGB(64, 64, 64), 1);
		}
		// Success
		else {
			cvLine(out, p1, p0, CV_RGB(255, 255, 255), 2);
			cvCircle(out, p1, hw, CV_RGB(255, 0, 0), 4); // Red
		}

	}

	// For debugging CUDA
	//CvPoint l1,l2,l3,l4;
	//int tsize = 11;
	//for (int iii=0;iii<100;iii++)
	//{
	//	float px = featurePool->m_cuda_fx[iii] + featurePool->m_cuda_motion[iii+0*MAX_NUM_FEATURES];
	//	float py = featurePool->m_cuda_fy[iii] + featurePool->m_cuda_motion[iii+1*MAX_NUM_FEATURES];
	//	
	//	l1.x = scale*(px+ tsize*featurePool->m_cuda_motion[iii+2*MAX_NUM_FEATURES]+tsize*featurePool->m_cuda_motion[iii+4*MAX_NUM_FEATURES]);
	//	l1.y = scale*(py+ tsize*featurePool->m_cuda_motion[iii+3*MAX_NUM_FEATURES]+tsize*featurePool->m_cuda_motion[iii+5*MAX_NUM_FEATURES]);
	//	l2.x = scale*(px+ tsize*featurePool->m_cuda_motion[iii+2*MAX_NUM_FEATURES]-tsize*featurePool->m_cuda_motion[iii+4*MAX_NUM_FEATURES]);
	//	l2.y = scale*(py+ tsize*featurePool->m_cuda_motion[iii+3*MAX_NUM_FEATURES]-tsize*featurePool->m_cuda_motion[iii+5*MAX_NUM_FEATURES]);
	//	l3.x = scale*(px- tsize*featurePool->m_cuda_motion[iii+2*MAX_NUM_FEATURES]-tsize*featurePool->m_cuda_motion[iii+4*MAX_NUM_FEATURES]);
	//	l3.y = scale*(py- tsize*featurePool->m_cuda_motion[iii+3*MAX_NUM_FEATURES]-tsize*featurePool->m_cuda_motion[iii+5*MAX_NUM_FEATURES]);
	//	l4.x = scale*(px- tsize*featurePool->m_cuda_motion[iii+2*MAX_NUM_FEATURES]+tsize*featurePool->m_cuda_motion[iii+4*MAX_NUM_FEATURES]);
	//	l4.y = scale*(py- tsize*featurePool->m_cuda_motion[iii+3*MAX_NUM_FEATURES]+tsize*featurePool->m_cuda_motion[iii+5*MAX_NUM_FEATURES]);
	//	
	//	cvLine(out, l1, l2, CV_RGB(255, 126, 126), 1);
	//	cvLine(out, l2, l3, CV_RGB(255, 126, 126), 1);
	//	cvLine(out, l3, l4, CV_RGB(255, 126, 126), 1);
	//	cvLine(out, l4, l1, CV_RGB(255, 126, 126), 1);
	//}

	char fname[255];
	sprintf(fname, "%04d.bmp", stat.frame);
	//cvSaveImage(fname,out);
}

void convert_float_image_color(const IplImage* gimgf, IplImage* cimg)
{
	if (gimgf == NULL) return;

	CvSize size = cvSize(gimgf->width, gimgf->height);
	IplImage* gimg = cvCreateImage(size, IPL_DEPTH_8U, 1);
	cvConvert(gimgf, gimg);

	if (gimgf->width == cimg->width && gimgf->height == cimg->height) {
		cvCvtColor(gimg, cimg, CV_GRAY2RGB);
	}
	else {
		IplImage* cimg_tmp = cvCreateImage(size, IPL_DEPTH_8U, 3);
		cvCvtColor(gimg, cimg_tmp, CV_GRAY2RGB);
		cvResize(cimg_tmp, cimg);
		cvReleaseImage(&cimg_tmp);
	}

	cvReleaseImage(&gimg);
}
