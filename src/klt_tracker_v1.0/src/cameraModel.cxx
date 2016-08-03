/**
* CMU MSUAV Project (Micro and Small Unmanned Aerial Vehicle)
*          
* \file  cameraModel.h
* \brief implementation for a pin-hole camera model class (CCameraModel)
* \date  12-Jun-2006
*
* \author Myung Hwangbo (myung@cs.cmu.edu) and Qifa Ke
*
* Copyright (c) 2006-2009 Myung Hwangbo and Qifa Ke
* Robotics Institute, Carnegie Mellon University
*
*/

#include <iostream>
#include <iomanip>
#include <cmath>
#include "cameraModel.h"

CCameraModel::CCameraModel(const CAMERA_INTRINSIC_PARAM& p) : m_cam(p)
{
	b_normalTable = false;
	m_pNormalTable = NULL;

	ComputeCameraMatrix(m_cam);
}

CCameraModel::CCameraModel(int w, int h, float vfov, const char *lut_fname)
{
	b_normalTable = false;
	m_pNormalTable = NULL;

	m_cam.width  = w;
	m_cam.height = h;

	m_cam.cc[0] = w/2;
	m_cam.cc[1] = h/2;
	m_cam.skew  = 0;
	for (int i=0; i < 5; i++) m_cam.kc[i] = 0;

	//
	// FOV to focal length
	//
	//           +
	//          /|
	// vfov/2  / | h/2
	//        /  |
	//       +---+
	//         f 
	//
	//  tan(vfov/2) = (h/2)/f --> f = (h/2)/tan(vfov/2)
	//
	m_cam.fc[1] = (h/2)/tan(vfov/2);
	m_cam.fc[0] = m_cam.fc[1];

	m_index_max = m_cam.width*m_cam.height*2;

//	if (!LoadNormalizeTable(fname)) {
		ComputeCameraMatrix(m_cam);
		ComputeNormalizeTable(lut_fname);
//	}
}

CCameraModel::CCameraModel(int w, int h, float fc0, float fc1, float cc0, float cc1, float s, 
						   float kc0, float kc1, float kc2, float kc3, float kc4, const char *lut_fname)
{
	b_normalTable = false;
	m_pNormalTable = NULL;

	m_cam.width  = w;
	m_cam.height = h;
	
	m_cam.fc[0] = fc0;
	m_cam.fc[1] = fc1;
	m_cam.cc[0] = cc0;
	m_cam.cc[1] = cc1;
	m_cam.skew  = s;

	m_cam.kc[0] = kc0;
	m_cam.kc[1] = kc1;
	m_cam.kc[2] = kc2;
	m_cam.kc[3] = kc3;
	m_cam.kc[4] = kc4;

	m_index_max = m_cam.width*m_cam.height*2;

//	if (!LoadNormalizeTable(fname)) {
		ComputeCameraMatrix(m_cam);
		ComputeNormalizeTable(lut_fname);
//	}
}

CCameraModel::CCameraModel(const char* param_fname, const char *lut_fname)
{
	b_normalTable = false;
	m_pNormalTable = NULL;

	ReadCameraParameter(param_fname, m_cam);

	ComputeCameraMatrix(m_cam);
	ComputeNormalizeTable(lut_fname);

	/*
	b_normalTable = false;

	if (LoadNormalizeTable(fname)) {
		b_normalTable = true;
	}
	*/
}

CCameraModel::~CCameraModel()
{
	if (m_pNormalTable != NULL) 
		delete [] m_pNormalTable;
}

bool CCameraModel::ReadCameraParameter(const char* fname, CAMERA_INTRINSIC_PARAM& p)
{
	ifstream fin(fname);
	if (fin.fail()) {
		cout << "Error in opening camera param file, " << fname << endl;
		return false;
	}

	fin >> p.width >> p.height;
	fin >> p.fc[0] >> p.fc[1];
	fin >> p.cc[0] >> p.cc[1];
	fin >> p.skew;
	fin >> p.kc[0] >> p.kc[1] >> p.kc[2] >> p.kc[3] >> p.kc[4];

	return true;
}

bool CCameraModel::IsValid(float px, float py)
{
	int xi = (int)(px+0.5);
	int yi = (int)(py+0.5);

	if (xi >= m_cam.width  || xi < 0) return false;
	if (yi >= m_cam.height || yi < 0) return false;

	return true;
}

bool CCameraModel::NormalizeFromTable(float px, float py, float& nx, float& ny)
{
	int x[2] = {(int)(px), (int)(px) + 1};
	int y[2] = {(int)(py), (int)(py) + 1};
	
	if ((x[0] < 0) || (x[1] >= m_cam.width) || (y[0] < 0) || (y[1] >= m_cam.height)) {
		nx = ny = 0;
		return false;
	}

	float dx, dy;
	dx = px - x[0];
	dy = py - y[0];
	/*
	if (x[0] < 0) {
		x[0] = x[1] = 0; dx = 0;
	}
	if (y[0] < 0) {
		y[0] = y[1] = 0; dy = 0;
	}
	if (x[1] > m_cam.width-1) {
		x[0] = x[1] = m_cam.width-1; dx = 0;
	}
	if (y[1] > m_cam.height-1) {
		y[0] = y[1] = m_cam.height-1; dy = 0;
	}
	*/

	// Bilinear interpolation
	//
	//   +---------------------- x
	//   |      dx
	//   |    00--+-------10
	//   | dy |   |       |
	//   |    +---+-------+
	//   |    |   |       |
	//   |    |   |       | 
	//   |    10--+-------11
	//  y|
	//  

	float qx[2][2], qy[2][2];
	for (int i=0; i < 2; i++) {
		for (int k=0; k < 2; k++) {
			int index = m_cam.height * x[i] + y[k];
			qx[i][k] = m_pNormalTable[2*index + 0];
			qy[i][k] = m_pNormalTable[2*index + 1];
		}
	}

	nx = qx[0][0]*(1-dx)*(1-dy) + qx[1][0]*(dx)*(1-dy) + qx[0][1]*(1-dx)*dy + qx[1][1]*dx*dy;
	ny = qy[0][0]*(1-dx)*(1-dy) + qy[1][0]*(dx)*(1-dy) + qy[0][1]*(1-dx)*dy + qy[1][1]*dx*dy;

	return true;
}

bool CCameraModel::NormalizeFromTable(const vector<float>& px, const vector<float>& py, vector<float>& nx, vector<float>& ny)
{
	int n = (int)px.size();
	nx.resize(n);
	ny.resize(n);

	for (int i=0; i < n; i++) {
		NormalizeFromTable(px[i], py[i], nx[i], ny[i]);
	}
	return true;
}

bool CCameraModel::NormalizeFromTable(int num, float *px, float *py, float *nx, float *ny)
{
	for (int i=0; i < num; i++) {
		NormalizeFromTable(px[i], py[i], nx[i], ny[i]);
	}
	return true;	
}

bool CCameraModel::NormalizeFromTable(int num, float *pxy, float *nxy)
{
	for (int i=0; i < num; i++) {
		NormalizeFromTable(pxy[i], pxy[num + i], nxy[i], nxy[num + i]);
	}
	return true;	
}

void CCameraModel::ComputeCameraMatrix(CAMERA_INTRINSIC_PARAM& p)
{
	p.KK[0][0] = p.fc[0];	p.KK[0][1] = p.skew*p.fc[0];	p.KK[0][2] = p.cc[0];
	p.KK[1][0] = 0;			p.KK[1][1] = p.fc[1];			p.KK[1][2] = p.cc[1];
	p.KK[2][0] = 0;			p.KK[2][1] = 0;					p.KK[2][2] = 1;

	p.Ki[0][0] = 1/p.fc[0];	p.Ki[0][1] = -p.skew/p.fc[1];	p.Ki[0][2] = -p.cc[0]/p.fc[0] + p.skew*(p.cc[1]/p.fc[1]);
	p.Ki[1][0] = 0;			p.Ki[1][1] = 1/p.fc[1];			p.Ki[1][2] = -p.cc[1]/p.fc[1];
	p.Ki[2][0] = 0;			p.Ki[2][1] = 0;					p.Ki[2][2] =  1;

	bool verbose = true;
	if (verbose) {
		cout << "CameraModel::ComputeCameraMatrix()" << endl;
		cout << " Camera size: " << p.width << " x " << p.height << endl;
	
		cout << " Camera params: " << endl;
		for (int i=0; i < 2; i++) cout << fixed << setw(10) << setprecision(2) << p.fc[i];
		for (int i=0; i < 2; i++) cout << fixed << setw(10) << setprecision(2) << p.cc[i];
		cout << endl;
		cout << fixed << setw(10) << setprecision(5) << p.skew << endl;
		for (int i=0; i < 5; i++) cout << fixed << setw(10) << setprecision(2) << p.kc[i];
		cout << endl;

		cout << " Camera matrix: " << endl;
		for (int i=0; i < 3; i++) {
			for (int k=0; k < 3; k++) {
				cout << fixed << setw(10) << setprecision(2) << p.KK[i][k];
			}
			cout << endl;
		}
		
		cout << " Camera matrix inverse: " << endl;
		for (int i=0; i < 3; i++) {
			for (int k=0; k < 3; k++) {
				cout << fixed << setw(10) << setprecision(6) << p.Ki[i][k];
			}
			cout << endl;
		}
	}
}

void CCameraModel::ComputeNormalizeTable(const char* fname)
{
	if (m_pNormalTable != NULL) 
		delete [] m_pNormalTable;

	int n = m_cam.width*m_cam.height*2;

	m_pNormalTable = new float [n];
	
	float nx, ny;

	for (int px=0; px < m_cam.width; px++) {
		for (int py=0; py < m_cam.height; py++) {
			
			Normalize(px, py, nx, ny);
			
			int i = m_cam.height * px + py;
			m_pNormalTable[2*i + 0] = nx;
			m_pNormalTable[2*i + 1] = ny;
		}
	}

	//cout << "CCameraModel::ComputeNormalizeTable(), done" << endl;

	if (fname != NULL)
		SaveNormalizeTable(fname);


	float nx0, ny0, nx1, ny1, fov, vfov;
	
	NormalizeFromTable(0,             m_cam.height/2, nx0, ny0);
	NormalizeFromTable(m_cam.width-1, m_cam.height/2, nx1, ny1);
	fov = fabs(atan(nx0)) + fabs(atan(nx1));

	bool b_check = false;
	if (b_check) {
		cout << "n0 = " << nx0 << " " << ny0 << endl;
		cout << "n1 = " << nx1 << " " << ny1 << endl;
		cout << "fov = " << fov << " (" << fov*180/acos(-1.0) << " deg)" << endl;
	}

	NormalizeFromTable(m_cam.width/2, 0,              nx0, ny0);
	NormalizeFromTable(m_cam.width/2, m_cam.height-1, nx1, ny1);
	vfov = fabs(atan(ny0)) + fabs(atan(ny1));

	if (b_check) {
		cout << "n0 = " << nx0 << " " << ny0 << endl;
		cout << "n1 = " << nx1 << " " << ny1 << endl;
		cout << "vfov = " << vfov << " (" << vfov*180/acos(-1.0) << " deg)" << endl;
	}
}

void CCameraModel::Normalize(const int px, const int py, float& nx, float& ny)
{
	// Camera matrix
	float dy = (py - m_cam.cc[1])/m_cam.fc[1];
	float dx = (px - m_cam.cc[0])/m_cam.fc[0] - m_cam.skew*dy;
	
	// Antidistortion
	Antidistortion(dx, dy, nx, ny);
}

void CCameraModel::Unnormalize(float nx, float ny, int& px, int& py)
{
	px = (int)(m_cam.fc[0]*(nx + m_cam.skew*ny) + m_cam.cc[0]);
	py = (int)(m_cam.fc[1]*ny + m_cam.cc[1]);
}

void CCameraModel::Antidistortion(float dx, float dy, float& nx, float& ny)
{
	float k1 = m_cam.kc[0];
	float k2 = m_cam.kc[1];
	float k3 = m_cam.kc[2];
	float p1 = m_cam.kc[3];
	float p2 = m_cam.kc[4];

	// Initial starting point
	nx = dx;
	ny = dy;

	// 20 iterations are usually enough to make converged.
	for (int i=0; i < 20; i++) 
	{
		float r2 = nx*nx + ny*ny;
		float k_radial = 1 + k1 * r2 + k2 * r2*r2 + k3 * r2*r2*r2;
		
		float delta_x = 2*p1*nx*ny + p2 * (r2 + 2*nx*nx);
		float delta_y = p1*(r2 + 2*ny*ny) + 2*p2*nx*ny;

		nx = (dx - delta_x) / k_radial;
		ny = (dy - delta_y) / k_radial;
	}
}

void CCameraModel::Distortion(float nx, float ny, float& dx, float& dy)
{
	float k1 = m_cam.kc[0];
	float k2 = m_cam.kc[1];
	float k3 = m_cam.kc[2];
	float p1 = m_cam.kc[3];
	float p2 = m_cam.kc[4];

	float r2 = nx*nx + ny*ny;
	float k_radial = 1 + k1 * r2 + k2 * r2*r2 + k3 * r2*r2*r2;
	float delta_x = 2*p1*nx*ny + p2 * (r2 + 2*nx*nx);
	float delta_y = p1*(r2 + 2*ny*ny) + 2*p2*nx*ny;

	dx = nx*k_radial + delta_x;
	dy = ny*k_radial + delta_y;
}

bool CCameraModel::SaveNormalizeTable(const char* fname)
{
	ofstream fout;

	//fout.open(fname, ofstream::out | ofstream::binary);
	fout.open(fname, ofstream::out);
	if (fout.fail()) {
		cerr << "Error in CCameraModel::SaveNormalizeTable(), file open" << endl;
		return false;
	}

	int n = m_cam.width*m_cam.height*2;
	if (m_pNormalTable == NULL) {
		cout << "Warning in CCameraModel::SaveNormalizeTable(), m_pNormalTable is NULL" << endl;
		return false;
	}

	fout.write((char*)&m_cam, sizeof(m_cam));
	fout.write((char*)m_pNormalTable, n*sizeof(float));
	fout.close();

	//cout << "CCameraModel::SaveNormalizeTable(), done to " << fname << endl;

	return true;
}

bool CCameraModel::LoadNormalizeTable(const char* fname)
{
	ifstream fin;
	fin.open(fname, ifstream::binary);
	if (fin.fail()) {
		cerr << "Error in CCameraModel::LoadNormalizeTable(), file open" << endl;
		return false;
	}

	int n = m_cam.width*m_cam.height*2;
	if (m_pNormalTable == NULL) m_pNormalTable = new float [n];

	fin.read((char*)&m_cam, sizeof(m_cam));
	fin.read((char*)m_pNormalTable, n*sizeof(float));
	fin.close();

	//cout << "CCameraModel::LoadNormalizeTable(), done from " << fname << endl;

	return true;

#if 0
	ofstream fout("test.txt");

	fout << m_cam.width << " " << m_cam.height << endl;
	fout << m_cam.fc[0] << " " << m_cam.fc[1] << " " << m_cam.cc[0] << " " << m_cam.cc[1] << " " << m_cam.skew << endl;
	fout << m_cam.kc[0] << " " << m_cam.kc[1] << " " << m_cam.kc[2] << " " << m_cam.kc[3] << " " << m_cam.kc[4] << endl;
	fout << m_cam.KK[0][0] << " " << m_cam.KK[0][1] << " " << m_cam.KK[0][2] << endl;
	fout << m_cam.KK[1][0] << " " << m_cam.KK[1][1] << " " << m_cam.KK[1][2] << endl;
	fout << m_cam.KK[2][0] << " " << m_cam.KK[2][1] << " " << m_cam.KK[2][2] << endl;

	if (m_pNormalTable == NULL) {
		cout << "Warning in CCameraModel::SaveNormalizeTable(), m_pNormalTable is NULL" << endl;
		return false;
	}

	for (int i=0; i < m_cam.width*m_cam.height; i++) {
		fout << m_pNormalTable[2*i] << " " << m_pNormalTable[2*i + 1] << endl;
	}
	fout.close();

	cout << "CCameraModel::LoadNormalizeTable(), test done." << endl;
#endif
}

double* CCameraModel::GetKK(double KK[9])
{
	for (int i=0; i < 3; i++) 
		for (int k=0; k < 3; k++) KK[i + 3*k] = (double)m_cam.KK[i][k];
	return KK;
}

double* CCameraModel::GetKi(double Ki[9])
{
	for (int i=0; i < 3; i++) 
		for (int k=0; k < 3; k++) Ki[i + 3*k] = (double)m_cam.Ki[i][k];
	return Ki;
}