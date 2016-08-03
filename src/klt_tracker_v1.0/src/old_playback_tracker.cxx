/**
* CMU MSUAV Project (Micro and Small Unmanned Aerial Vehicle)
*          
* \file  playback.cxx
* \brief load logged image and IMU data in a timely manner
* \date  01-Nov-2009
*
* \author Myung Hwangbo Myung Hwangbo (myung@cs.cmu.edu)
*
* Copyright (c) 2009 Myung Hwangbo
* Robotics Institute, Carnegie Mellon University
*
*/

#include <ipp.h>
#include <highgui.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "playback_tracker.h"

using namespace std;

typedef struct {
    unsigned int time_msec;
    double acc[3], rate[3];
} IMU_DATA;

double* quat_to_Rmat(const double* q, double* R);
double* transpose_mat_3x3(const double* A, double *B);
double* mul_mat_3x3(const double* A, const double* B, const double* C, double *D);
void print_mat(const double* A, int m, int n, const char* str);
void print_vec(const double* v, int m, const char* str);

int playback(const char* fname, int t_offset, IplImage* gimgf)
{
	static bool b_open = false;
	static int count = 0;
	static string dir(fname);
	static ifstream fin(fname);
	
	int time_msec = 0;
	string str, name;
	size_t p, q;

	// Open file only once
	if (!b_open) {
		if (fin.fail()) {
			cout << "Error in opening a file " << fname << endl;
			return false;
		}

		p = dir.rfind("/");
		if (p != string::npos) dir.erase(p+1, dir.size()-p-1);
		getline(fin, str);
		dir += str + '/';

		b_open = true;
	}

	// Read image and timestamp
	if (getline(fin, str))
	{
		name = dir + str;

		IplImage* img = cvLoadImage(name.c_str());
		if (img == NULL) {
			printf("Error in loading %s\n", name.c_str());
			return -1;
		}

		if (img->width != gimgf->width || img->height != gimgf->height) {
			printf("Error: Wrong image size, %d x %d\n", img->width, img->height);
			return -1;
		}

		p = name.rfind("_");
		q = name.rfind(".");
		if (p == string::npos || q == string::npos) {
			cout << "Error in reading timestamp from image file name " << name << endl;
			return -1;
		}
		string str = name.substr(p+1, q-p-1);
		time_msec = atoi(str.c_str()) + t_offset;

		//printf("%s, %d\n", str.c_str(), time_msec);

		smooth_image_float(img, gimgf, 3);
		cvReleaseImage(&img);
	}
	else
		return -1;

	return time_msec;	
}

void smooth_image_float(const IplImage* img, IplImage* gimgf, int w)
{
	CvSize size = cvSize(img->width, img->height);
	IplImage* gimg = cvCreateImage(size, IPL_DEPTH_8U, 1);

	if (img->nChannels == 3) {
		cvCvtColor(img, gimg, CV_BGR2GRAY);
	}
	else if (img->nChannels == 1) {
		cvCopy(img, gimg);
	}

	int len = img->width * img->height;
	if (w > 0) {
		IplImage* temp = cvCreateImage(size, IPL_DEPTH_32F, 1);
		ippsConvert_8u32f((Ipp8u *)gimg->imageData, (Ipp32f *)temp->imageData, len);

		// Smoothing
		int s = ((w % 2) == 0) ? w + 1 : w;
		cvSmooth(temp, gimgf, CV_GAUSSIAN, s, s);
		cvReleaseImage(&temp);
	}
	else {
		ippsConvert_8u32f((Ipp8u *)gimg->imageData, (Ipp32f *)gimgf->imageData, len);
	}

	cvReleaseImage(&gimg);
}

bool compute_homography_IMU(int ptime, int ctime, const char* fname, const char* shape_fname, 
							const double* KK, const double* Ki, double H[9])
{
	static bool b_open = false;
	static int index = 0;
	static vector<IMU_DATA> vdata;
	
	ifstream fin(fname);
	ifstream fin_sh(shape_fname);

	// Open file only once
	if (!b_open) {
		if (fin_sh.fail()) {
			cout << "Error in opening IMU shape file " << shape_fname << endl;
			return false;
		}

		double bias_acc[3], bias_gyro[3];
		double Ca[3][3], Cw[3][3];

		for (int i=0; i < 3; i++) fin_sh >> bias_acc[i];
		for (int k=0; k < 3; k++)
			for (int i=0; i < 3; i++) fin_sh >> Ca[k][i];

		for (int i=0; i < 3; i++) fin_sh >> bias_gyro[i];
		for (int k=0; k < 3; k++)
			for (int i=0; i < 3; i++) fin_sh >> Cw[k][i];

		/*
		printf("bias_acc = %7.1f %7.1f %7.1f\n", bias_acc[0], bias_acc[1], bias_acc[2]);
		printf("Ca = %7.4f %7.4f %7.4f\n", Ca[0][0], Ca[0][1], Ca[0][2]);
		printf("     %7.4f %7.4f %7.4f\n", Ca[1][0], Ca[1][1], Ca[1][2]);
		printf("     %7.4f %7.4f %7.4f\n", Ca[2][0], Ca[2][1], Ca[2][2]);
		printf("bias_acc = %7.1f %7.1f %7.1f\n", bias_gyro[0], bias_gyro[1], bias_gyro[2]);
		printf("Ca = %7.4f %7.4f %7.4f\n", Cw[0][0], Cw[0][1], Cw[0][2]);
		printf("     %7.4f %7.4f %7.4f\n", Cw[1][0], Cw[1][1], Cw[1][2]);
		printf("     %7.4f %7.4f %7.4f\n", Cw[2][0], Cw[2][1], Cw[2][2]);
		*/

		if (fin.fail()) {
			cout << "Error in opening IMU file " << fname << endl;
			return false;
		}

		int t1, t2;
		double a[3], w[3];
		IMU_DATA m;
		while(!fin.eof()) {
			fin >> t1 >> t2 >> a[0] >> a[1] >> a[2] >> w[0] >> w[1] >> w[2];
			for (int i=0; i < 3; i++) a[i] = a[i] - bias_acc[i];
			for (int i=0; i < 3; i++) w[i] = w[i] - bias_gyro[i];

			m.time_msec = t1;
			m.acc[0]  = Ca[0][0]*a[0] + Ca[0][1]*a[1] + Ca[0][2]*a[2];
			m.acc[1]  = Ca[1][0]*a[0] + Ca[1][1]*a[1] + Ca[1][2]*a[2];
			m.acc[2]  = Ca[2][0]*a[0] + Ca[2][1]*a[1] + Ca[2][2]*a[2];
			m.rate[0] = Cw[0][0]*w[0] + Cw[0][1]*w[1] + Cw[0][2]*w[2];
			m.rate[1] = Cw[1][0]*w[0] + Cw[1][1]*w[1] + Cw[1][2]*w[2];
			m.rate[2] = Cw[2][0]*w[0] + Cw[2][1]*w[1] + Cw[2][2]*w[2];

			vdata.push_back(m);

			//printf("%6d %6.1f %6.1f %6.1f\n", t1, m.rate[0], m.rate[1], m.rate[2]);
		}

		printf("IMU: %d samples are loaded\n", (int)vdata.size());

		b_open = true;
	}

	double q[4] = {1, 0, 0, 0};
	int deltaT = 2;
	int tt, n = (int)vdata.size();

	if (index >= n-1) {
		H[0] = H[4] = H[8] = 1.0;
		H[1] = H[2] = H[3] = H[5] = H[6] = H[7] = 0.0;
		return false;
	}

	for (tt = ptime; tt < ctime; ) 
	{
		double a = 1.0;
		int ti = (int)vdata[index].time_msec;
		int tj = (int)vdata[index+1].time_msec;
		
		if (tt < ti) { 
			a = ((ti - tt) < 30) ? 1.0 : 0.0;
		}
		else if (tt >= ti && tt < tj) {
			a = (double)(tj - tt)/(double)(tj - ti);
		}
		else if (tt >= tj) {
			if (index++ > n-1) break;
			else continue;
		}
		else break;
		
		// first-order approximation
		double w[3];
		for(int i=0; i < 3;i++) w[i] = a*vdata[index].rate[i] + (1-a)*vdata[index+1].rate[i];
		//printf("INS: %d/%d %8d %6.2f %8.4f %8.4f %8.4f\n", index, vdata[index].time_msec, tt, 1-a, w[0], w[1], w[2]);

		// Quaternion integration: dq = (0.5*Qw*q)*dt;
		double Qw[4][4] = {{   0,   -w[0],  -w[1], -w[2]},
						   { w[0],     0,    w[2], -w[1]},
						   { w[1],  -w[2],     0,   w[0]},
						   { w[2],   w[1],  -w[0],    0 }};

		double dq[4];
		for (int i=0; i < 4; i++) {
			dq[i] = 0.5f*(Qw[i][0]*q[0] + Qw[i][1]*q[1] + Qw[i][2]*q[2] + Qw[i][3]*q[3])*(0.001*deltaT);
			q[i] += dq[i];
		}

		// Normalize the quaternion
		double norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
		for (int i=0; i < 4; i++) q[i] = q[i]/norm;

		tt += deltaT;
	}

	// H = KK*Ri*Ki
	double R[9], Ri[9];
	transpose_mat_3x3(quat_to_Rmat(q, R), Ri);
	mul_mat_3x3(KK, Ri, Ki, H);

	//printf("R_imu: %d --> %d\n", ptime, ctime);
	//print_mat(R, 3, 3, "R = ");
	//print_mat(H, 3, 3, "H = ");
	
	return true;
}

double* transpose_mat_3x3(const double* A, double *B)
{
	for (int i=0; i < 3; i++) 
		for (int k=0; k < 3; k++) 
			B[i + 3*k] = (i == k) ? A[i + 3*k] : A[k + 3*i];

	return B;
}

double* mul_mat_3x3(const double* A, const double* B, const double* C, double *D)
{
	double AB[9];

	for (int i=0; i < 3; i++) 
		for (int k=0; k < 3; k++)
			AB[i + 3*k] = A[i]*B[3*k] + A[i+3]*B[3*k+1] + A[i+6]*B[3*k+2];

	for (int i=0; i < 3; i++) 
		for (int k=0; k < 3; k++)
			D[i + 3*k] = AB[i]*C[3*k] + AB[i+3]*C[3*k+1] + AB[i+6]*C[3*k+2];

	return D; // D = A*B*C
}

void print_mat(const double* A, int m, int n, const char* str)
{
	if (str != NULL) printf("%s [ ... \n", str);
	for (int i=0; i < m; i++) {
		for (int k=0; k < n; k++) printf("%14.10f ", A[i + m*k]); 
		if (i == m-1) printf("];\n\n");
		else		  printf(";\n");
	}
}

void print_vec(const double* v, int m, const char* str)
{
	if (str != NULL) printf("%s [ ... \n", str);
	for (int i=0; i < m; i++) {
		printf("%11.7f ", v[i]);
		if ((i % 10) == 9) printf(" ...\n");
		if (i == m-1) printf("];\n\n");
	}
}

double* quat_to_Rmat(const double* q, double* R)
{
	double w = q[0];
	double x = q[1];
	double y = q[2];
	double z = q[3];
	double n = sqrt(w*w + x*x + y*y + z*z);
	
	w = w/n;
	x = x/n;
	y = y/n;
	z = z/n;

	double xx = x*x, xy = x*y, xz = x*z, xw = x*w;
	double yy = y*y, yz = y*z, yw = y*w;
	double zz = z*z, zw = z*w;
	double ww = w*w;

	R[0] = 1 - 2*(yy + zz); R[3] =     2*(xy - zw); R[6] =     2*(xz + yw);
	R[1] =     2*(xy + zw);	R[4] = 1 - 2*(xx + zz);	R[7] =     2*(yz - xw);
	R[2] =     2*(xz - yw);	R[5] =     2*(yz + xw);	R[8] = 1 - 2*(xx + yy);

	return R;
}
