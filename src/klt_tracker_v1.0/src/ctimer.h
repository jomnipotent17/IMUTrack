/**
* MSUAV Project (Micro and Small Unmanned Aerial Vehicle)
*          
* \file  ctimer.h
* \brief High-precision timer function class (CTimer)
* \date  06-Jun-2010
*
* \author Myung Hwangbo, Robotics Institute, Carnegie Mellon University
* \modified by Jun-Sik Kim, Robotics Institute, Carnegie Mellon University
*           for measuring time both on Windows and on Linux
*
* Copyright (c) 2010 Myung Hwangbo
* Robotics Institute, Carnegie Mellon University
*
*/

#ifndef __CTIMER_H__
#define __CTIMER_H__

#ifndef _WIN32
#include <sys/time.h>
#endif
#include <iostream>
#include <iomanip>
#include <fstream>

#define	SYNC_FILENAME	"/sync_timer.txt"

class CTimer
{

public:
	CTimer(bool sync = false) 
	{
#ifdef _WIN32
		QueryPerformanceFrequency((LARGE_INTEGER*)&freq);
#endif
		Start();
		if (sync) Sync();
	}
	~CTimer(void) {}

	void Start(void) {
#ifndef _WIN32
		struct timeval s;
		gettimeofday(&s, NULL);
		t_start = s.tv_sec + 1E-6*s.tv_usec;
#else
		QueryPerformanceCounter((LARGE_INTEGER*)&start);
		t_start = (double)start/(double)freq;
#endif
	}

	double Stop(void) {
#ifndef _WIN32
		struct timeval s;
		gettimeofday(&s, NULL);
		t_stop = s.tv_sec + 1E-6*s.tv_usec;
		return (t_stop - t_start);
#else
		QueryPerformanceCounter((LARGE_INTEGER*)&stop);
		t_stop = (double)stop/(double)freq;;
		return (t_stop-t_start);
#endif
	}

	void Sync() {
		std::ifstream fin(SYNC_FILENAME);
		if (fin.fail()) {
			Write();
		}
		else {
			fin >> t_start;
			std::cout << "CTimer::Sync() t_start = " << std::setprecision(15) << t_start << std::endl;
		}
	}

	void Write() {
		std::ofstream fout(SYNC_FILENAME);
		if (!fout.fail()) {
			std::cout << "CTimer::Write() t_start = " << std::setprecision(15) << t_start << std::endl;
			fout << std::setprecision(15) << t_start;
		}
	}

	double GetStartTime() {
		return t_start;
	}

	double GetStopTime() {
		return t_stop;
	}
	
private:
	double t_start, t_stop;

#ifdef _WIN32
	__int64 freq, start, stop;
#endif

};
#endif
