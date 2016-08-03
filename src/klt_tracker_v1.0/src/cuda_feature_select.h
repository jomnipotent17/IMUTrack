/**
* CMU MSUAV Project (Micro and Small Unmanned Aerial Vehicle)
*
* \file  cuda_feature_select.h
* \brief header file for cuda_feature_select.cxx
* \date  15-Sept-2008
*
* \author Jun-Sik Kim (kimjs@cs.cmu.edu)
*
*/

#ifndef __CUDA_SELECT_H__
#define __CUDA_SELECT_H__

void cvGoodFeaturesToTrack_GPU(IplImage* gimgf, void* eigImage, void* tempImage,
			CvPoint2D32f* corners, int *corner_count, double quality_level, double min_distance,
			const void* maskImage = NULL, int block_size = 3, int use_harris = 0, double harris_k = 0.04);

#endif
