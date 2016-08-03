/**
* CMU MSUAV Project (Micro and Small Unmanned Aerial Vehicle)
*          
* \file  playback.h
* \brief header file of playback.cxx
* \date  01-Nov-2009
*
* \author Myung Hwangbo Myung Hwangbo (myung@cs.cmu.edu)
*
* Copyright (c) 2009 Myung Hwangbo
* Robotics Institute, Carnegie Mellon University
*
*/

#ifndef __PLAYBACK_H__
#define __PLAYBACK_H__

#include <cv.h>

void smooth_image_float(const IplImage* img, IplImage* gimgf, int w);
int playback(const char* fname, int t_offset, IplImage* gimgf);
bool compute_homography_IMU(int ptime, int ctime, const char* fname, const char* shape_fname, 
							const double* KK, const double* Ki, double H[9], int sequence, int imuAid);

#endif
