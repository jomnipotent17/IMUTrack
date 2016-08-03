/**
* CMU MSUAV Project (Micro and Small Unmanned Aerial Vehicle)
*          
* \file  main_tracker.cxx
* \brief main function of IMU-aided KLT feature tracker
* \date  18-Apr-2008
*
* \author Myung Hwangbo (myung@cs.cmu.edu) and Jun-Sik Kim (kimjs@cs.cmu.edu)
*
* Copyright (c) 2006-2009 Myung Hwangbo and Jun-Sik Kim
* Robotics Institute, Carnegie Mellon University
*
*/

#ifdef _WIN32
#include <windows.h>
#include <conio.h>
#endif

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>

#include <string>
#include <ipp.h>

#include "ctimer.h"
#include "featurePool.h"
#include "playback_tracker.h"
#include "display_tracker.h"

//#ifdef _WIN32
//#pragma comment(lib, "affineTracker.lib")
//#pragma comment(lib, "cv.lib")
//#pragma comment(lib, "cvcam.lib")
//#pragma comment(lib, "cxcore.lib")
//#pragma comment(lib, "highgui.lib")
//#pragma comment(lib, "ippi.lib")
//#pragma comment(lib, "ipps.lib")
//#pragma comment(lib, "cudart.lib")
//#pragma comment(lib, "cutil32.lib")
//#endif

using namespace std;

CFeature2DPool* featurePool = NULL;

int min_feature = 400;
int max_feature = 400;
int templ_size = 11;
int mask_size = 10;
int pyramid_level = 3;

double disp_scale = 1.0;
int img_width = 640;
int img_height = 480;
int t_offset = 0;
char track_log_name[256];
char imu_shape_name[256];
char imu_list_name[256];
char img_list_name[256];
char camera_param_name[256];
char camera_table_name[256];

//double disp_scale = 1.0;
//int img_width = 640;
//int img_height = 480;
//int t_offset = -20;
//char track_log_name[256] = "track.log";
//char imu_shape_name[256] = "../data/desk_imu_shape.txt";
//char imu_list_name[256] = "../data/desk_imu.log";
//char img_list_name[256] = "../data/desk_img.log";
//char camera_param_name[256] = "../data/desk_camera_param.txt";
//char camera_table_name[256] = "normalTable.txt";

//double disp_scale = 1.5;
//int img_width = 320;
//int img_height = 240;
//int t_offset = -20;
//char track_log_name[256] = "track.log";
//char imu_shape_name[256] = "../data/aerial_imu_shape.txt";
//char imu_list_name[256] = "../data/aerial_imu.log";
//char img_list_name[256] = "../data/aerial_img.log";
//char camera_param_name[256] = "../data/aerial_camera_param.txt";
//char camera_table_name[256] = "normalTable.txt";

bool b_display = true;
bool b_gpu = false;
bool b_imu = true;
bool b_stop = false;
bool b_paused = false;
bool b_log = true;
bool b_run = true;

void exit_handler();
bool read_input_arg(int argc, char* argv[]);
bool read_config_file(const char* fname);

int main(int argc, char* argv[])
{
	atexit(exit_handler);
	if (!read_input_arg(argc, argv)) return 0;

	CTimer timer;
	double t0 = 0, t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0;
	double KK[9], Ki[9];

	CvSize img_size = cvSize(img_width, img_height);
	CCameraModel* cam = new CCameraModel(camera_param_name, camera_table_name);
	featurePool = new CFeature2DPool(img_size, templ_size, pyramid_level, mask_size, b_gpu, cam);
	
	if (b_display) init_track_display(disp_scale);

	//
	// Main loop
	//
	int ctime = 0, ptime = 0, frame = 0;
	while(b_run)
	{
		if (b_display) {
			char ch = (char)cvWaitKey(5);
			if (ch == 'q') break;
			else if (ch == 'c') system("cls");
			else if (ch > 0) b_paused = !b_paused;
		}
		
		if (b_paused) {
			continue; //Sleep(10);
		}

		//
		// Step 1: Read the data
		//
		IplImage* gimgf = cvCreateImage(img_size, IPL_DEPTH_32F, 1);
		ctime = playback(img_list_name, t_offset, gimgf);
		if (ctime < 0) break;
		featurePool->AddImage(frame, gimgf);								
		t0 = timer.Stop();

		//
		// Step 2: Predict the camera rotation from the IMU
		//
		double H[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
		if (b_imu && ptime != 0) {
			if (!compute_homography_IMU(ptime, ctime, imu_list_name, imu_shape_name, 
					cam->GetKK(KK), cam->GetKi(Ki), H)) break;
		}

		//
		// Step 3: Track and select the features
		//
		if (b_gpu) {
			featurePool->Track_GPU(frame, gimgf, H);							t1 = timer.Stop();
			featurePool->Select_GPU(frame, min_feature, max_feature, gimgf);	t2 = timer.Stop();
		}
		else {
			featurePool->Track(frame, ctime, gimgf, H);							t1 = timer.Stop();
			featurePool->Select(frame, min_feature, max_feature, gimgf, false);	t2 = timer.Stop();
		}

		featurePool->Purge(frame, 10);											t3 = timer.Stop();
		if (b_log) featurePool->SaveToFile(0, track_log_name);
		
		//
		// Step 4: Display
		//
		 if (b_display) run_track_display();									t4 = timer.Stop();

		CFeatureTrackStat& stat = featurePool->GetStat();
		printf("%3d, %8d %3d | %3d = %3d/%3d %3d | %5.1f = %5.1f + %5.1f + %5.1f, %5.1f\n",
			stat.frame, ctime-t_offset, (ctime-ptime), stat.n_total, stat.n_success, stat.n_total - stat.n_success, stat.n_new,
			(t3 - t0)*1e3, (t1 - t0)*1e3, (t2 - t1)*1e3, (t3 - t2)*1e3, (t0 - t5)*1e3);

		//
		// Step 3: Update
		//
		if (b_stop) b_paused = true;

		ptime = ctime;															t5 = timer.Stop();
		frame++;

	}

	if (b_display) close_track_display();
	if (cam) delete cam;

	return 0;
}

void exit_handler()
{
	printf("exit() is called. \n");
    b_run = false;

	if (featurePool) delete featurePool;
}

bool read_input_arg(int argc, char* argv[])
{
	for (int i=1; i < argc; i++) 
	{
		if      (strncmp(argv[i], "-log",   4) == 0)  b_log = true;
		else if (strncmp(argv[i], "-gpu",   4) == 0)  b_gpu = true;
		else if (strncmp(argv[i], "-stop",  5) == 0)  b_stop = true;
		else if (strncmp(argv[i], "-f",     2) == 0) {
			if ((i+1) >= argc) return false;
			if (!read_config_file(argv[i+1])) return false; i++;
		}
		else if (strncmp(argv[i], "-disp", 5) == 0) {
			if ((i+1) >= argc) return false;
			disp_scale = atof(argv[i+1]); i++;
			b_display = true;
		}
		else if (strncmp(argv[i], "-imu", 4) == 0) {
			if ((i+1) >= argc) return false;
			t_offset = atoi(argv[i+1]); i++;
			b_imu = true;
		}
		else if (strncmp(argv[i], "-n", 2) == 0) {
			if ((i+1) >= argc) return false;
			max_feature = min_feature = atof(argv[i+1]); i++;
		}
		else if (strncmp(argv[i], "-t", 4) == 0) {
			if ((i+1) >= argc) return false;
			templ_size = 2*atoi(argv[i+1]) + 1; i++;
		}
		else {
			printf("Invalid switch: %s\n\n");
			printf("usage: %s [-f config] [-log] [-gpu] [-stop] [-imu time_ms] [-n num] [-t size] [-disp scale]\n\n", argv[0]);
			return false;
		}
	}

	printf("\n[Setting]\n");
	printf(" GPU          : %s\n", b_gpu      ? "ON" : "OFF");
	printf(" IMU          : %s\n", b_imu      ? "ON" : "OFF");
	printf(" Log          : %s\n", b_log      ? "ON" : "OFF");
	printf(" Stop         : %s\n", b_stop     ? "ON" : "OFF");
	printf(" Display      : %s\n", b_display  ? "ON" : "OFF");
	printf(" Time_offset  : %d\n", t_offset);
	printf(" Disp_scale   : %.1f\n", disp_scale);
	printf(" Templ_size   : %d\n", templ_size);
	printf(" Mask_size    : %d\n", templ_size);
	printf(" Feature_num  : %d\n", max_feature);
	printf(" Image_width  : %d\n", img_width);
	printf(" Image_heigh  : %d\n", img_height);
	printf(" Image_data   : %s\n", img_list_name);
	printf(" IMU_data     : %s\n", imu_list_name);
	printf(" IMU_shape    : %s\n", imu_shape_name);
	printf(" Camera_param : %s\n", camera_param_name);
	printf(" Camera_table : %s\n", camera_table_name);
	printf(" Log_filename : %s\n", track_log_name);
	printf("\n");

	if (argc == 1) {
		cout << "No command arguments are provided." << endl;
		return false;
	}

	return true;	
}

bool read_config_file(const char* fname)
{
	ifstream fin(fname);
	if (fin.fail()) {
		cout << "Error, unable to open " << fname << endl;
		return false;
	}
	
	string line, str;
	size_t p, q;
	while (!fin.eof()) {
		getline(fin, line);
		p = line.find_first_of('"'); 
		q = line.find_first_of('"', p+1);
		if (line[0] == '%' || line[0] == '#' || line[1] == '/') continue;

		istringstream ss(line);
		ss >> str; cout << str << "|" << endl ;

		if (str == "GPU") {
			ss >> str;  b_gpu = (str.find("ON") != string::npos);
		}
		else if (str == "IMU") {
			ss >> str;  b_imu = (str.find("ON") != string::npos);
		}
		else if (str == "Log") {
			ss >> str;  b_log = (str.find("ON") != string::npos);
		}
		else if (str == "Stop") {
			ss >> str;  b_stop = (str.find("ON") != string::npos);
		}
		else if (str == "Display") {
			ss >> str;  b_display = (str.find("ON") != string::npos);
		}
		else if (str == "Time_offset") {
			ss >> str;  t_offset = atof(str.c_str());
		}
		else if (str == "Disp_scale") {
			ss >> str;  disp_scale = atof(str.c_str());
		}
		else if (str == "Feature_num") {
			ss >> str;  min_feature = max_feature = atoi(str.c_str());
		}
		else if (str == "Templ_size") {
			ss >> str;  templ_size = atoi(str.c_str());
		}
		else if (str == "Mask_size") {
			ss >> str;  mask_size = atoi(str.c_str());
		}
		else if (str == "Pyramid_level") {
			ss >> str;  pyramid_level = atoi(str.c_str());
		}
		else if (str == "Image_width") {
			ss >> str;  img_width = atoi(str.c_str());
		}
		else if (str == "Image_height") {
			ss >> str;  img_height = atoi(str.c_str());
		}
		else if (str == "Image_data") {
			if (p == string::npos || q == string::npos) {
				cout << "Error in setting Image_data in configuration file" << endl;
				return false;
			}
			line.copy(img_list_name, q-p-1, p+1);
		}
		else if (str == "IMU_data") {
			if (p == string::npos || q == string::npos) {
				cout << "Error in setting IMU_data in configuration file" << endl;
				return false;
			}
			line.copy(imu_list_name, q-p-1, p+1);
		}
		else if (str == "IMU_shape_param") {
			if (p == string::npos || q == string::npos) {
				cout << "Error in setting IMU_shape_param in configuration file" << endl;
				return false;
			}
			line.copy(imu_shape_name, q-p-1, p+1);
		}
		else if (str == "Camera_param") {
			if (p == string::npos || q == string::npos) {
				cout << "Error in setting Camera_param in configuration file" << endl;
				return false;
			}
			line.copy(camera_param_name, q-p-1, p+1);
		}
		else if (str == "Camera_table") {
			if (p == string::npos || q == string::npos) {
				cout << "Error in setting Camera_table in configuration file" << endl;
				return false;
			}
			line.copy(camera_table_name, q-p-1, p+1);
		}
		else if (str == "Log_filename") {
			if (p == string::npos || q == string::npos) {
				cout << "Error in setting Log_filename in configuration file" << endl;
				return false;
			}
			line.copy(track_log_name, q-p-1, p+1);
		}
	}

	return true;
}
