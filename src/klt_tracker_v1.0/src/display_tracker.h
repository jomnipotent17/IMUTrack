/**
* CMU MSUAV Project (Micro and Small Unmanned Aerial Vehicle)
*          
* \file  trackDisplay.h
* \brief header file of trackDisplay.cxx
* \date  15-Apr-2009
*
* \author Myung Hwangbo (myung@cs.cmu.edu)
*
* Copyright (c) 2009 Myung Hwangbo
* Robotics Institute, Carnegie Mellon University
*
*/

#ifndef __TRACK_DISPLAY_H__
#define __TRACK_DISPLAY_H__

#include <cv.h>
#include <highgui.h>

void init_track_display(double scale = 1);
void run_track_display();
void close_track_display();

#endif