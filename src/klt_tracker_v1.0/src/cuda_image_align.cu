/**
* CMU MSUAV Project (Micro and Small Unmanned Aerial Vehicle)
*
* \file  cuda_select.cxx
* \brief CUDA implementation of an affine-photometric tracker
* \date  10-Dec-2008
*
* \author Jun-Sik Kim (kimjs@cs.cmu.edu)
*
* Copyright (c) 2008-2009 Jun-Sik Kim
* Robotics Institute, Carnegie Mellon University
*/

#define MAX_NUM_FEATURES	1024

// Copy from "cutil.h" in CUDA SDK
#define CUDA_SAFE_CALL(call) {                                         \
	cudaError err = call;                                              \
	if (cudaSuccess != err) {                                          \
		fprintf(stderr, "Cuda error in file '%s' in line %i : %s.\n",  \
			__FILE__, __LINE__, cudaGetErrorString( err) );            \
		exit(EXIT_FAILURE);                                            \
    }}
    
#define CUT_CHECK_ERROR(errorMessage) {                                  \
	cudaError_t err = cudaGetLastError();                                \
	if (cudaSuccess != err) {                                            \
		fprintf(stderr, "Cuda error: %s in file '%s' in line %i : %s.\n",\
			errorMessage, __FILE__, __LINE__, cudaGetErrorString( err) );\
		exit(EXIT_FAILURE);                                              \
    }}    

#include <stdio.h>
#include <stdlib.h>
#include <cuda_runtime.h>
#include <ipp.h>

#include "cuda_convolution_separable_kernel.cu"

// Only for inverse operation on CPU
// currently NOT USED (Dec, 2008)
//#pragma comment(lib, "libI77.lib")
//#pragma comment(lib, "libF77.lib")
//#pragma comment(lib, "clapack.lib")
//#pragma comment(lib, "blas.lib")

////////////////////////////////////////////////////////////////////////////////
// Common host and device functions
////////////////////////////////////////////////////////////////////////////////
//Round a / b to nearest higher integer value
int iDivUp(int a, int b){
    return (a % b != 0) ? (a / b + 1) : (a / b);
}

//Round a / b to nearest lower integer value
int iDivDown(int a, int b){
    return a / b;
}

//Align a to nearest higher multiple of b
int iAlignUp(int a, int b){
    return (a % b != 0) ?  (a - a % b + b) : a;
}

//Align a to nearest lower multiple of b
int iAlignDown(int a, int b){
    return a - a % b;
}

//////////////////////////////////////////////////////////////////////////
// 
// Definitions of Constants
//
//////////////////////////////////////////////////////////////////////////

#define	_INVERSE

#define MAX_PYR_LEVEL		5
#define	BIG_COST			100000000.0f
#define	VALIDITY_CHECK_COST	800.0f
#define	PHOTO_SCALE			(1.0f/16.0f)
#define	PHOTO_SCALE_SQ		(PHOTO_SCALE*PHOTO_SCALE)


//////////////////////////////////////////////////////////////////////////
// 
// LAPACK functions for inverting Hessians on a cpu
//
//////////////////////////////////////////////////////////////////////////
extern "C" int sgetrf_(int *m, int *n, float *a, int *lda, int *ipiv, int *info);
extern "C" int sgetri_(int *n, float *a, int *lda, int *ipiv, float *work, int *lwork, int *info);
extern int templ_size;

//////////////////////////////////////////////////////////////////////////
// 
// Declarations of CUDA Variables
//
//////////////////////////////////////////////////////////////////////////

// Images, image textures, CUDA arrays for textures
cudaArray *Image1[MAX_PYR_LEVEL], *Image2[MAX_PYR_LEVEL];

// Texture variables and corresponding 2D cudaArray
static texture<float, 2, cudaReadModeElementType> I_Texture;
static texture<float, 2, cudaReadModeElementType> Gx_Texture;
static texture<float, 2, cudaReadModeElementType> Gy_Texture;
static texture<float, 2, cudaReadModeElementType> W_Texture;

cudaArray *TArr[MAX_PYR_LEVEL];
cudaArray *TempArr[MAX_PYR_LEVEL];
cudaArray *GxArr[MAX_PYR_LEVEL];
cudaArray *GyArr[MAX_PYR_LEVEL];
cudaArray *WArr[MAX_PYR_LEVEL];

// CUDA global variables
float *new_featureX[MAX_PYR_LEVEL], *new_featureY[MAX_PYR_LEVEL];
float *templ_H_device[MAX_PYR_LEVEL], *templ_H_host[MAX_PYR_LEVEL];
float *eig[MAX_PYR_LEVEL], *tempmax[MAX_PYR_LEVEL];
int *new_id_map;

float *h_Kernel;
float *result[MAX_PYR_LEVEL];
float *Gx[MAX_PYR_LEVEL], *Gy[MAX_PYR_LEVEL], *W[MAX_PYR_LEVEL];
float *templ_Hessian[MAX_PYR_LEVEL], *featureX[MAX_PYR_LEVEL], *featureY[MAX_PYR_LEVEL];
float *vectorB[MAX_PYR_LEVEL];
float *templ_Gx[MAX_PYR_LEVEL], *templ_Gy[MAX_PYR_LEVEL];
float *templ_W[MAX_PYR_LEVEL],  *templ_T[MAX_PYR_LEVEL];
float *motionParams[MAX_PYR_LEVEL], *delta_motionParams[MAX_PYR_LEVEL];
float *mincost[MAX_PYR_LEVEL], *bestscale[MAX_PYR_LEVEL], *scale[MAX_PYR_LEVEL];

// Constant variable for IMU integration
__constant__ float Hmg[9];

// Tracker parameters
int max_iteration;				// number of iterations in Tracking
int max_linesearch_iteration;	// number of iterations for line search in Tracking
int pyramidLevel;				// pyramid level

int templSize[MAX_PYR_LEVEL];	// template size for each pyr level
int imgWidth[MAX_PYR_LEVEL] ;	// image width for each pyr level
int imgHeight[MAX_PYR_LEVEL];	// image height for each pyr level
int pyrThreads[MAX_PYR_LEVEL];	// number of threads for each pyr level


//////////////////////////////////////////////////////////////////////////
//
// int InitCUDA(int imageWidth, int imageHeight, int pyrLevel, int templateSize, int maxIteration, int maxLinesearch)
//
//		CUDA initialization
//
//			Device initialization
//			Memory allocations
//
//////////////////////////////////////////////////////////////////////////
extern "C" 
int InitCUDA(int imageWidth, int imageHeight, int pyrLevel, int templateSize, int maxIteration, int maxLinesearch)
{
	int i = 0, count = 0;

	////////////////////////////////////////////////////////////////////////////////////
	// CUDA device initialization
	////////////////////////////////////////////////////////////////////////////////////

	cudaGetDeviceCount(&count);
	if (count == 0) {
		fprintf(stderr, "There is no device.\n");
		return 0;
	}

	for (i = 0; i < count; i++) {
		cudaDeviceProp prop;
		if(cudaGetDeviceProperties(&prop, i) == cudaSuccess) {
			if(prop.major >= 1) break;
		}
	}

	if (i == count) {
		fprintf(stderr, "There is no device supporting CUDA.\n");
		return 0;
	}
	cudaSetDevice(i);

	////////////////////////////////////////////////////////////////////////////////////
	// smoothing kernel setting
	// d_Kernel defined in "cuda_convolutionSeparable_kernel.cu" (constant memory)
	////////////////////////////////////////////////////////////////////////////////////

	h_Kernel = (float *)malloc(KERNEL_W * sizeof(float));

	float kernelSum = 0;
	for(i = 0; i < KERNEL_W; i++){
		float dist = (float)(i - KERNEL_RADIUS) / (float)KERNEL_RADIUS;
		h_Kernel[i] = exp(-dist*dist/2);
		kernelSum += h_Kernel[i];
	}
	for(i = 0; i < KERNEL_W; i++) h_Kernel[i] /= kernelSum;

	CUDA_SAFE_CALL( cudaMemcpyToSymbol(d_Kernel, h_Kernel, KERNEL_W * sizeof(float)) );

	////////////////////////////////////////////////////////////////////////////////////
	// Tracker setting
	////////////////////////////////////////////////////////////////////////////////////

	if (pyrLevel > MAX_PYR_LEVEL) 
	{
		printf("pyrLevel should be less than MAX_PYR_LEVEL.\n");
		return 0;
	}

	max_iteration = maxIteration;
	max_linesearch_iteration = maxLinesearch;
	pyramidLevel = pyrLevel;

	for (i=0; i < pyramidLevel; i++)
	{
	    templSize[i]  = templateSize;
		if (i==0)
		{
			imgWidth[i] = imageWidth;
			imgHeight[i] = imageHeight;
		}
		else 
		{
			imgWidth[i] = (int)(imgWidth[i-1]/2);
			imgHeight[i] = (int)(imgHeight[i-1]/2);
		}
		
		pyrThreads[i] = min(iAlignDown(imgWidth[i], 32), imageWidth/4);
		printf("%d, (width,height) = (%d,%d) pyrThreads[i] = %d\n",i,imgWidth[i],imgHeight[i],pyrThreads[i]);
		if (pyrThreads[i]<1) 
		{
			pyramidLevel = i-1;
			printf("The max level of pyramid is set to %d.\n",pyramidLevel);
			break;
		}
	}


	////////////////////////////////////////////////////////////////////////////////////
	// memory allocation
	////////////////////////////////////////////////////////////////////////////////////

	// 1. for id map communication (GPU <--> CPU)
	CUDA_SAFE_CALL( cudaMalloc((void**)&new_id_map, MAX_NUM_FEATURES*sizeof(int)));

	// 2. Texture references
	I_Texture.normalized = false;
	I_Texture.filterMode = cudaFilterModeLinear;;
	I_Texture.addressMode[0] = cudaAddressModeClamp;
	I_Texture.addressMode[1] = cudaAddressModeClamp;

	Gx_Texture.normalized = false;
	Gx_Texture.filterMode = cudaFilterModeLinear;;
	Gx_Texture.addressMode[0] = cudaAddressModeClamp;
	Gx_Texture.addressMode[1] = cudaAddressModeClamp;

	Gy_Texture.normalized = false;
	Gy_Texture.filterMode = cudaFilterModeLinear;;
	Gy_Texture.addressMode[0] = cudaAddressModeClamp;
	Gy_Texture.addressMode[1] = cudaAddressModeClamp;

	W_Texture.normalized = false;
	W_Texture.filterMode = cudaFilterModeLinear;;
	W_Texture.addressMode[0] = cudaAddressModeClamp;
	W_Texture.addressMode[1] = cudaAddressModeClamp;

	// Global memory
	for (i=0;i < pyramidLevel;i++) // for each pyr level
	{
		templSize[i] = templ_size;
	
		CUDA_SAFE_CALL( cudaMalloc((void**) &result[i], imgWidth[i]*imgHeight[i]*sizeof(float)));
		CUDA_SAFE_CALL( cudaMalloc((void**) &Gx[i],     imgWidth[i]*imgHeight[i]*sizeof(float)));
		CUDA_SAFE_CALL( cudaMalloc((void**) &Gy[i],     imgWidth[i]*imgHeight[i]*sizeof(float)));
		CUDA_SAFE_CALL( cudaMalloc((void**) &W[i],      imgWidth[i]*imgHeight[i]*sizeof(float)));

		CUDA_SAFE_CALL( cudaMalloc((void**) &templ_Gx[i], MAX_NUM_FEATURES*templSize[i]*templSize[i]*sizeof(float)));
		CUDA_SAFE_CALL( cudaMalloc((void**) &templ_Gy[i], MAX_NUM_FEATURES*templSize[i]*templSize[i]*sizeof(float)));
		CUDA_SAFE_CALL( cudaMalloc((void**) &templ_W[i],  MAX_NUM_FEATURES*templSize[i]*templSize[i]*sizeof(float)));
		CUDA_SAFE_CALL( cudaMalloc((void**) &templ_T[i],  MAX_NUM_FEATURES*templSize[i]*templSize[i]*sizeof(float)));

		CUDA_SAFE_CALL( cudaMalloc((void**) &eig[i],     imgWidth[i]*imgHeight[i]*sizeof(float)));
		CUDA_SAFE_CALL( cudaMalloc((void**) &tempmax[i], imgWidth[i]*imgHeight[i]*sizeof(float)));

		CUDA_SAFE_CALL( cudaMalloc((void**) &templ_Hessian[i],  64*MAX_NUM_FEATURES*sizeof(float)));
		CUDA_SAFE_CALL( cudaMalloc((void**) &templ_H_device[i], 64*MAX_NUM_FEATURES*sizeof(float)));
		CUDA_SAFE_CALL( cudaMalloc((void**) &featureX[i],          MAX_NUM_FEATURES*sizeof(float)));
		CUDA_SAFE_CALL( cudaMalloc((void**) &featureY[i],          MAX_NUM_FEATURES*sizeof(float)));
		CUDA_SAFE_CALL( cudaMalloc((void**) &new_featureX[i],      MAX_NUM_FEATURES*sizeof(float)));
		CUDA_SAFE_CALL( cudaMalloc((void**) &new_featureY[i],      MAX_NUM_FEATURES*sizeof(float)));

		CUDA_SAFE_CALL( cudaMalloc((void**) &vectorB[i],            9*MAX_NUM_FEATURES*sizeof(float)));
		CUDA_SAFE_CALL( cudaMalloc((void**) &motionParams[i],       8*MAX_NUM_FEATURES*sizeof(float)));
		CUDA_SAFE_CALL( cudaMalloc((void**) &delta_motionParams[i], 8*MAX_NUM_FEATURES*sizeof(float)));
		CUDA_SAFE_CALL( cudaMalloc((void**) &mincost[i],             MAX_NUM_FEATURES*sizeof(float)));
		CUDA_SAFE_CALL( cudaMalloc((void**) &bestscale[i],           MAX_NUM_FEATURES*sizeof(float)));
		CUDA_SAFE_CALL( cudaMalloc((void**) &scale[i],               MAX_NUM_FEATURES*sizeof(float)));

		// setting cudaArray for texture binding
		cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc<float>();
		CUDA_SAFE_CALL( cudaMallocArray( &Image1[i], &channelDesc, imgWidth[i]*sizeof(float), imgHeight[i])); 
		CUDA_SAFE_CALL( cudaMallocArray( &Image2[i], &channelDesc, imgWidth[i]*sizeof(float), imgHeight[i])); 
		CUDA_SAFE_CALL( cudaMallocArray( &GxArr[i],  &channelDesc, imgWidth[i]*sizeof(float), imgHeight[i])); 
		CUDA_SAFE_CALL( cudaMallocArray( &GyArr[i],  &channelDesc, imgWidth[i]*sizeof(float), imgHeight[i])); 
		CUDA_SAFE_CALL( cudaMallocArray( &WArr[i],   &channelDesc, imgWidth[i]*sizeof(float), imgHeight[i])); 
		CUDA_SAFE_CALL( cudaMallocArray( &TArr[i],   &channelDesc, imgWidth[i]*sizeof(float), imgHeight[i])); 
		CUDA_SAFE_CALL( cudaMallocArray( &TempArr[i],   &channelDesc, imgWidth[i]*sizeof(float), imgHeight[i])); 

		templ_H_host[i] = new float[64*MAX_NUM_FEATURES]; // used for inverting matrices on a cpu
	}

	return pyramidLevel;
}

//////////////////////////////////////////////////////////////////////////
// 
// bool ExitCUDA(void)
// 
//		CUDA destruction
//
//			Free Memory
//
//////////////////////////////////////////////////////////////////////////
extern "C" 
bool ExitCUDA(void)
{
	////////////////////////////////////////////////////////////////////////////////////
	// Free memory
	////////////////////////////////////////////////////////////////////////////////////

	CUDA_SAFE_CALL(cudaFree(new_id_map));

	free(h_Kernel);

	for (int i=0; i < pyramidLevel; i++)
	{
		CUDA_SAFE_CALL( cudaFree(result[i]));
		CUDA_SAFE_CALL( cudaFree(Gx[i]));
		CUDA_SAFE_CALL( cudaFree(Gy[i]));
		CUDA_SAFE_CALL( cudaFree(W[i]));

		CUDA_SAFE_CALL( cudaFree(eig[i]	));
		CUDA_SAFE_CALL( cudaFree(tempmax[i]));
		CUDA_SAFE_CALL( cudaFree(templ_Hessian[i]));
		CUDA_SAFE_CALL( cudaFree(templ_H_device[i]));
		CUDA_SAFE_CALL( cudaFree(featureX[i]));
		CUDA_SAFE_CALL( cudaFree(featureY[i]));
		CUDA_SAFE_CALL( cudaFree(new_featureX[i]));
		CUDA_SAFE_CALL( cudaFree(new_featureY[i]));
		CUDA_SAFE_CALL( cudaFree(motionParams[i]));
		CUDA_SAFE_CALL( cudaFree(vectorB[i]));

		CUDA_SAFE_CALL( cudaFreeArray(Image1[i])); 
		CUDA_SAFE_CALL( cudaFreeArray(Image2[i])); 
		CUDA_SAFE_CALL( cudaFreeArray(GxArr[i])); 
		CUDA_SAFE_CALL( cudaFreeArray(GyArr[i])); 
		CUDA_SAFE_CALL( cudaFreeArray(WArr[i])); 
		CUDA_SAFE_CALL( cudaFreeArray(TArr[i])); 
		CUDA_SAFE_CALL( cudaFreeArray(TempArr[i])); 
		CUDA_SAFE_CALL( cudaFree(templ_Gx[i]));
		CUDA_SAFE_CALL( cudaFree(templ_Gy[i]));
		CUDA_SAFE_CALL( cudaFree(templ_W[i]));
		CUDA_SAFE_CALL( cudaFree(templ_T[i]));

		CUDA_SAFE_CALL( cudaFree(delta_motionParams[i]));
		CUDA_SAFE_CALL( cudaFree(mincost[i]));
		CUDA_SAFE_CALL( cudaFree(bestscale[i]));
		CUDA_SAFE_CALL( cudaFree(scale[i]));

		delete templ_H_host[i];
	}

	return true;
}

///////////////////////////////////////////////////////////////////////////////
// 
// __device__ void InvertMatrix(float elem[size][size], float res[size][size]) 
//  
//		n x n matrix inversion on a gpu
//
///////////////////////////////////////////////////////////////////////////////
template <int size>
__device__ void InvertMatrix(float elem[size][size], float res[size][size]) 
{  
	int indx[size];
	float b[size];
	float vv[size];
	for (int i=0;i<size;i++)
		indx[i] = 0;
	int imax = 0;
	float d = 1.0;
	for (int i=0;i<size;i++) { // find biggest element for each row
		float big = 0.0;
		for (int j=0;j<size;j++) {
			float temp = fabs(elem[i][j]); 
			if (temp>big) 
				big = temp;
		}
		if (big>0.0)
			vv[i] = 1.0/big;
		else
			vv[i] = 1e16;
	}
	for (int j=0;j<size;j++) { 
		for (int i=0;i<j;i++) { // i<j
			float sum = elem[i][j]; // i<j (lower left)
			for (int k=0;k<i;k++) // k<i<j
				sum -= elem[i][k]*elem[k][j]; // i>k (upper right), k<j (lower left)
			elem[i][j] = sum; // i<j (lower left)
		}
		float big = 0.0;
		for (int i=j;i<size;i++) { // i>=j
			float sum = elem[i][j]; // i>=j (upper right)
			for (int k=0;k<j;k++) // k<j<=i
				sum -= elem[i][k]*elem[k][j]; // i>k (upper right), k<j (lower left)
			elem[i][j] = sum; // i>=j (upper right)
			float dum = vv[i]*fabs(sum);
			if (dum>=big) {
				big = dum;
				imax = i;  
			}
		}
		if (j!=imax) { // imax>j
			for (int k=0;k<size;k++) {
				float dum = elem[imax][k]; // upper right and lower left
				elem[imax][k] = elem[j][k];
				elem[j][k] = dum;
			}
			d = -d;
			vv[imax] = vv[j];
		}
		indx[j] = imax;
		if (elem[j][j]==0.0)  // j==j (upper right)
			elem[j][j] = 1e-16;
		if (j!=(size-1)) {
			float dum = 1.0/elem[j][j];
			for (int i=j+1;i<size;i++) // i>j
				elem[i][j] *= dum; // i>j (upper right)
		}
	}
	for (int j=0;j<size;j++) {
		for (int k=0;k<size;k++) 
			b[k] = 0.0;  
		b[j] = 1.0;
		int ii = -1;
		for (int i=0;i<size;i++) {
			int ip = indx[i];
			float sum = b[ip];
			b[ip] = b[i];
			if (ii!=-1)
				for (int j=ii;j<i;j++) 
					sum -= elem[i][j]*b[j]; // i>j (upper right)
			else if (sum!=0.0)
				ii = i;
			b[i] = sum;
		}
		for (int i=size-1;i>=0;i--) {
			float sum = b[i];
			for (int j=i+1;j<size;j++) 
				sum -= elem[i][j]*b[j]; // i<j (lower left)
			b[i] = sum/elem[i][i]; // i==i (upper right)
		}
		for (int i=0;i<size;i++)
			res[i][j] = b[i];
	}
}

////////////////////////////////////////////////////////////////////////////////////
//
// spatialDeriv(float2 addr, float2 &deriv, unsigned int width, unsigned int height)
//
//		Spatial Derivative device function -- called by kernel on a GPU
//	
//		Using Texture memory
//
/////////////////////////////////////////////////////////////////////////////////////
__device__ void
spatialDeriv(float2 addr, float2 &deriv,
			 unsigned int width, unsigned int height)
{ 
	/*
	float kk = tex2D(I_Texture, addr.x -1, addr.y );
	float jj = tex2D(I_Texture, addr.x +1, addr.y );
	deriv.x = 0.5f*tex2D(I_Texture, addr.x -1, addr.y ) - 0.5f* tex2D(I_Texture, addr.x + 1, addr.y );
	deriv.y = 0.5f*tex2D(I_Texture, addr.x , addr.y -1) - 0.5f* tex2D(I_Texture, addr.x , addr.y +1 );
	*/
	deriv.x = 0.5f*tex2D(I_Texture, addr.x +1, addr.y ) - 0.5f* tex2D(I_Texture, addr.x -1, addr.y );
	deriv.y = 0.5f*tex2D(I_Texture, addr.x , addr.y +1) - 0.5f* tex2D(I_Texture, addr.x , addr.y -1);
}

//////////////////////////////////////////////////////////////////////////
//
// __global__ void
//	find_max_CRCW_normalize_kernel(float *data, unsigned char* mask, float ratio, float* result,
//								   unsigned int width, unsigned int height)
//
//  Find maximum values of a 640 x n matrix in parallel
//
//      1. find the maximum of n vector by scanning once
//		2. n log n tournament method - n/2 comparisons simultaneously
//		  (NOT CRCW algorithm - can not be used due to limitation of the thread number)
//         NOTE the positinos of __syncthreads()
//
//		CAN BE FASTER by using a parallel algorithm in the "1" process
//
//////////////////////////////////////////////////////////////////////////
__global__ void
find_max_CRCW_normalize_kernel(float *data, float ratio, float* result,
							   unsigned int width, unsigned int height)
{
	__shared__ float maxx[1024];

	unsigned int x;

	x = blockIdx.x * blockDim.x + threadIdx.x;

	if (threadIdx.x < width/2) {
		maxx[threadIdx.x] = 0;
		maxx[threadIdx.x+width/2] = 0;
	}

	__syncthreads();

	for (int y=1; y < height-1; y++) {
		if ((x < width) && (x > 0))
			maxx[threadIdx.x] = max(maxx[threadIdx.x],data[y*width + x]); //*(float)(mask[y*width+x])/255.0f);
		__syncthreads();
	}

	// not CRCW method - n log n tournament method
	int halflength = width/2;
	while (halflength > 1) {
		if (threadIdx.x < halflength) {
			maxx[threadIdx.x] = max(maxx[threadIdx.x], maxx[threadIdx.x + halflength]);
		}
		__syncthreads();
		halflength /= 2;
	}
	__syncthreads();

	if (threadIdx.x == 0) maxx[0] = max(maxx[0],maxx[1]);

	__syncthreads();

	result[threadIdx.x] = maxx[0];
	__syncthreads();
}

//////////////////////////////////////////////////////////////////////////
//
//__global__ void
//	min_max_normalize_kernel(float *eig, float* temp, float ratio, float* result,
//       					 unsigned int width, unsigned int height)
//
//  Normalize the pixel values using a given maximum
//
//////////////////////////////////////////////////////////////////////////
__global__ void
min_max_normalize_kernel(float *eig, float* temp, float ratio, float* result,
						 unsigned int width, unsigned int height)
{
	unsigned int x, y;

	x = blockIdx.x * blockDim.x + threadIdx.x;
	y = blockIdx.y * blockDim.y + threadIdx.y;

	float mxxx = temp[threadIdx.x];
	result[y*width + x] = (eig[y*width + x] > mxxx*ratio) ? (eig[y*width + x]) : 0;
	__syncthreads();
}

//////////////////////////////////////////////////////////////////////////
// 
//__global__ void
//	spatialDerivKernel(float *Gx, float *Gy, float *W, size_t pitch,
//					   unsigned int width, unsigned int height)
//
//		Kernel for spatial derivation 
//
//		NOTE the range check and __syncthreads();
//
//////////////////////////////////////////////////////////////////////////
__global__ void
spatialDerivKernel(float *Gx, float *Gy, float *W, size_t pitch,
				   unsigned int width, unsigned int height)
{
	float2 deriv, pixel;
	

	pixel.x = blockIdx.x * blockDim.x + threadIdx.x;
	pixel.y = blockIdx.y * blockDim.y + threadIdx.y;
	unsigned int offset = pixel.y * width + pixel.x;

	deriv.x = 0;
	deriv.y = 0;
	if ((pixel.x >= 1) && (pixel.y >= 1) && (pixel.x < width-2) && (pixel.y < height-2)) {
		spatialDeriv(pixel, deriv, width, height);
	}
		
	__syncthreads();

	Gx[offset] = deriv.x;
	Gy[offset] = deriv.y;
	W [offset] = 0.25f*(fabs(deriv.x)+fabs(deriv.y));
}

//////////////////////////////////////////////////////////////////////////
//
//extern "C" void 
//	Caller_eig_show_mag(float* target, float ratio, unsigned int width, unsigned int height)
//
//		Kernel for an eigen image -- USED in Feature selection
//
//////////////////////////////////////////////////////////////////////////
extern "C" void 
Caller_eig_show_mag(float* target,  float ratio, unsigned int width, unsigned int height)
{
	dim3 threads(width/2,1);
	dim3 grid = dim3(width/threads.x, height/threads.y);

	dim3 grid1 = dim3(2, 1);
	find_max_CRCW_normalize_kernel<<< grid1 , threads >>>(eig[0], ratio, tempmax[0], width, height);
	min_max_normalize_kernel<<< grid , threads >>>(eig[0], tempmax[0], ratio, result[0], width, height);
	cudaMemcpy(target,result[0],width*height*sizeof(float),cudaMemcpyDeviceToHost);

	CUDA_SAFE_CALL( cudaThreadSynchronize());
}

//////////////////////////////////////////////////////////////////////////
//
// __global__ void
//		eigen_covariance_Kernel(const float *Gx, const float *Gy, float* eig,
//								unsigned int width, unsigned int height)
//
//  Kernel for compute harris measures - OLD version with noncoalesced accesses
//
//////////////////////////////////////////////////////////////////////////
__global__ void
eigen_covariance_Kernel(const float *Gx, const float *Gy, float* eig,
						unsigned int width, unsigned int height)
{
	unsigned int x, y;
	float gx2=0,gy2=0,gxy=0;
	x = blockIdx.x * blockDim.x + threadIdx.x;
	y = blockIdx.y * blockDim.y + threadIdx.y;

	if ((x >= 1) && (y >= 1) && (x < width-2) && (y < height-2))
	{
		gx2 = 
			Gx[x-1 + width*(y-1)]*Gx[x-1 + width*(y-1)] + Gx[x + width*(y-1)]*Gx[x + width*(y-1)] + Gx[x+1 + width*(y-1)]*Gx[x+1 + width*(y-1)] +
			Gx[x-1 + width*(y-0)]*Gx[x-1 + width*(y-0)] + Gx[x + width*(y-0)]*Gx[x + width*(y-0)] + Gx[x+1 + width*(y-0)]*Gx[x+1 + width*(y-0)] +
			Gx[x-1 + width*(y+1)]*Gx[x-1 + width*(y+1)] + Gx[x + width*(y+1)]*Gx[x + width*(y+1)] + Gx[x+1 + width*(y+1)]*Gx[x+1 + width*(y+1)];

		gy2 = 
			Gy[x-1 + width*(y-1)]*Gy[x-1 + width*(y-1)] + Gy[x + width*(y-1)]*Gy[x + width*(y-1)] + Gy[x+1 + width*(y-1)]*Gy[x+1 + width*(y-1)] +
			Gy[x-1 + width*(y-0)]*Gy[x-1 + width*(y-0)] + Gy[x + width*(y-0)]*Gy[x + width*(y-0)] + Gy[x+1 + width*(y-0)]*Gy[x+1 + width*(y-0)] +
			Gy[x-1 + width*(y+1)]*Gy[x-1 + width*(y+1)] + Gy[x + width*(y+1)]*Gy[x + width*(y+1)] + Gy[x+1 + width*(y+1)]*Gy[x+1 + width*(y+1)];

		gxy = 
			Gx[x-1 + width*(y-1)]*Gy[x-1 + width*(y-1)] + Gx[x + width*(y-1)]*Gy[x + width*(y-1)] + Gx[x+1 + width*(y-1)]*Gy[x+1 + width*(y-1)] +
			Gx[x-1 + width*(y-0)]*Gy[x-1 + width*(y-0)] + Gx[x + width*(y-0)]*Gy[x + width*(y-0)] + Gx[x+1 + width*(y-0)]*Gy[x+1 + width*(y-0)] +
			Gx[x-1 + width*(y+1)]*Gy[x-1 + width*(y+1)] + Gx[x + width*(y+1)]*Gy[x + width*(y+1)] + Gx[x+1 + width*(y+1)]*Gy[x+1 + width*(y+1)];

	}

	__syncthreads();

	eig[y*width+x] = (0.5f*gx2+0.5f*gy2) - 0.5f*sqrtf((gx2-gy2)*(gx2-gy2) + gxy*gxy);
}

//////////////////////////////////////////////////////////////////////////
// 
// __global__ void
//	eigen_covariance_Kernel_texturing(float* eig, unsigned int width, unsigned int height)
//
//		Kernel for compute harris measures - NEW version using texturing
//
//		NOTE the difference in the Caller Function (texture binding)
//
//////////////////////////////////////////////////////////////////////////
__global__ void
eigen_covariance_Kernel_texturing(float* eig, unsigned int width, unsigned int height)
{
	unsigned int x, y;
	float gx2=0,gy2=0,gxy=0;
	x = blockIdx.x * blockDim.x + threadIdx.x;
	y = blockIdx.y * blockDim.y + threadIdx.y;
	float gx[3][3],gy[3][3];

	if ((x >= 1) && (y >= 1) && (x < width-2) && (y < height-2))
	{
		gx[0][0] = tex2D(Gx_Texture,x-1,y-1); gx[0][1] = tex2D(Gx_Texture,x-1,y-0); gx[0][2] = tex2D(Gx_Texture,x-1,y+1);
		gx[1][0] = tex2D(Gx_Texture,x-0,y-1); gx[1][1] = tex2D(Gx_Texture,x-0,y-0); gx[1][2] = tex2D(Gx_Texture,x-0,y+1);
		gx[2][0] = tex2D(Gx_Texture,x+1,y-1); gx[2][1] = tex2D(Gx_Texture,x+1,y-0); gx[2][2] = tex2D(Gx_Texture,x+1,y+1);
		gy[0][0] = tex2D(Gy_Texture,x-1,y-1); gy[0][1] = tex2D(Gy_Texture,x-1,y-0); gy[0][2] = tex2D(Gy_Texture,x-1,y+1);
		gy[1][0] = tex2D(Gy_Texture,x-0,y-1); gy[1][1] = tex2D(Gy_Texture,x-0,y-0); gy[1][2] = tex2D(Gy_Texture,x-0,y+1);
		gy[2][0] = tex2D(Gy_Texture,x+1,y-1); gy[2][1] = tex2D(Gy_Texture,x+1,y-0); gy[2][2] = tex2D(Gy_Texture,x+1,y+1);

		gx2 = 
			gx[0][0]*gx[0][0] + gx[0][1]*gx[0][1] + gx[0][2]*gx[0][2] +
			gx[1][0]*gx[1][0] + gx[1][1]*gx[1][1] + gx[1][2]*gx[1][2] +
			gx[2][0]*gx[2][0] + gx[2][1]*gx[2][1] + gx[2][2]*gx[2][2] ;

		gy2 = 
			gy[0][0]*gy[0][0] + gy[0][1]*gy[0][1] + gy[0][2]*gy[0][2] +
			gy[1][0]*gy[1][0] + gy[1][1]*gy[1][1] + gy[1][2]*gy[1][2] +
			gy[2][0]*gy[2][0] + gy[2][1]*gy[2][1] + gy[2][2]*gy[2][2] ;

		gxy = 
			gx[0][0]*gy[0][0] + gx[0][1]*gy[0][1] + gx[0][2]*gy[0][2] +
			gx[1][0]*gy[1][0] + gx[1][1]*gy[1][1] + gx[1][2]*gy[1][2] +
			gx[2][0]*gy[2][0] + gx[2][1]*gy[2][1] + gx[2][2]*gy[2][2] ;

	}

	__syncthreads();

	eig[y*width+x] = 0.5f*((gx2+gy2) -sqrtf((gx2-gy2)*(gx2-gy2) + gxy*gxy));
}

//////////////////////////////////////////////////////////////////////////
//
//extern "C"  void
//	Caller_corner(cudaArray *Image, size_t pitch, unsigned int width, unsigned int height)
//
//		CUDA caller function for compute cornerness
//
//		This version is based on texturing (eigen_covariance_Kernel_texturing).
//
//////////////////////////////////////////////////////////////////////////
extern "C"  void
Caller_corner(cudaArray *Image, size_t pitch, unsigned int width, unsigned int height)
{
	dim3 threads(width/4,1); // Better number for threads?
	dim3 grid = dim3(iDivUp(width,threads.x), height/threads.y);

	CUDA_SAFE_CALL( cudaBindTextureToArray(I_Texture, Image) );
	spatialDerivKernel<<< grid , threads >>> (Gx[0], Gy[0], W[0],  width,width, height);
	// eigen_covariance_Kernel<<< grid , threads >>> (Gx[0], Gy[0], eig[0], width, height);

	CUDA_SAFE_CALL( cudaMemcpy2DToArray(GxArr[0], 0, 0, Gx[0], imgWidth[0]*sizeof(float), imgWidth[0]*sizeof(float), imgHeight[0], cudaMemcpyDeviceToDevice) );
	CUDA_SAFE_CALL( cudaMemcpy2DToArray(GyArr[0], 0, 0, Gy[0], imgWidth[0]*sizeof(float), imgWidth[0]*sizeof(float), imgHeight[0], cudaMemcpyDeviceToDevice) );
	CUDA_SAFE_CALL( cudaMemcpy2DToArray(WArr[0],  0, 0, W[0],  imgWidth[0]*sizeof(float), imgWidth[0]*sizeof(float), imgHeight[0], cudaMemcpyDeviceToDevice) );
	CUDA_SAFE_CALL( cudaBindTextureToArray(Gx_Texture, GxArr[0]) );
	CUDA_SAFE_CALL( cudaBindTextureToArray(Gy_Texture, GyArr[0]) );
	CUDA_SAFE_CALL( cudaBindTextureToArray(W_Texture,  WArr[0] ) );

	eigen_covariance_Kernel_texturing<<< grid , threads >>> (eig[0], width, height);

	CUT_CHECK_ERROR("Kernel execution failed");
	CUDA_SAFE_CALL( cudaThreadSynchronize());
}

//////////////////////////////////////////////////////////////////////////
// 
//	__global__ void
//	scaleDownFeatures(float *fx, float *fy)
//
//		Kernel for scaling down features (Track operation)
//
//////////////////////////////////////////////////////////////////////////
__global__ void
scaleDownFeatures(float *fx, float *fy)
{
	int findex = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;
	fx[findex] /= 2.0f;
	fy[findex] /= 2.0f;
}

//////////////////////////////////////////////////////////////////////////
// 
//	__global__ void
//	ComputeHessian (float *fx, float *fy, int numfeature, int *idmap, float *Hg,  
//					int t_size, unsigned int width, unsigned int height)
//
//		Kernel for Hessian computation (Track thread)
//			Inverse operation on a cpu
//
//////////////////////////////////////////////////////////////////////////
__global__ void
ComputeHessian(float *fx, float *fy, int numfeature, int *idmap, float *Hg,  int t_size, unsigned int width, unsigned int height)
{
	float gx, gy, w, gwx, gwy, gwx2, gwy2, gwxy;
	float tw, tgwx, tgwy, t, dx, dy, dx2, dy2, dxy;
	float H[8][8];
	
	float2 pos, addr;
	int findex = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;

	if (findex < numfeature) 
	{
		int globalindex = idmap[findex];

		pos.x = fx[globalindex];
		pos.y = fy[globalindex];

		for (int i=0;i < 8; i++)
			for (int j=0; j < 8; j++) H[i][j] = 0;

		for (dx = -t_size; dx <= t_size; dx++) {
			for (dy = -t_size; dy <= t_size; dy++)
			{
				addr.x = pos.x + dx;
				addr.y = pos.y + dy;
				if ((addr.x >= 1) && (addr.y >= 1) && (addr.x < width-2) && (addr.y < height-2)) {

					gx = tex2D(Gx_Texture, addr.x, addr.y);
					gy = tex2D(Gy_Texture, addr.x, addr.y);
					w  = tex2D(W_Texture,  addr.x, addr.y);
					t  = tex2D(I_Texture,  addr.x, addr.y);

					dx2	 = dx*dx;	dy2	 = dy*dy;	dxy	 = dx*dy;
					gwx	 = w*gx; 	gwy	 = w*gy;	gwx2 = gwx*gx;	gwy2 = gwy*gy;
					gwxy = gx*gwy;	tgwx = gwx*t;	tgwy = gwy*t;	tw	 = w*t;

					H[0][0] += gwx2;		H[0][1] += gwxy;		H[0][2] += dx*gwx2;		
					H[0][3] +=  dx*gwxy;	H[0][4] += dy*gwx2;		H[0][5] += dy*gwxy;
					H[0][6] += tgwx;		H[0][7] += gwx;

					H[1][1] += gwy2;		H[1][2] += dx*gwxy;		H[1][3] += dx*gwy2;
					H[1][4] += dy*gwxy;		H[1][5] += dy*gwy2;		H[1][6] += tgwy;
					H[1][7] += gwy;

					H[2][2] += dx2*gwx2;	H[2][3] += dx2*gwxy;	H[2][4] += dxy*gwx2;
					H[2][5] += dxy*gwxy;	H[2][6] += dx*tgwx;		H[2][7] += dx*gwx;

					H[3][3] += dx2*gwy2;	H[3][4] += dxy*gwxy;	H[3][5] += dxy*gwy2;
					H[3][6] += dx*tgwy;		H[3][7] += dx*gwy;		
					
					H[4][4] += dy2*gwx2;	H[4][5] += dy2*gwxy;	H[4][6] += dy*tgwx;
					H[4][7] += dy*gwx;

					H[5][5] += dy2*gwy2;	H[5][6] += dy*tgwy;		H[5][7] += dy*gwy;

					H[6][6] += t*tw;		H[6][7] += tw;	

					H[7][7] += w;
				}
			}

			// Transfer to global memory
			int index = __mul24(findex, 64);
			for (int i=0; i < 8; i++) {
				for (int j=i; j < 8; j++) {
					if (j != 7) {
						Hg[(i*8 + j)*MAX_NUM_FEATURES+findex] = PHOTO_SCALE_SQ*H[i][j];
						Hg[(j*8 + i)*MAX_NUM_FEATURES+findex] = PHOTO_SCALE_SQ*H[i][j]; // symmetric
					}
					else {
						Hg[(i*8 + j)*MAX_NUM_FEATURES+findex] = PHOTO_SCALE*H[i][j];
						Hg[(j*8 + i)*MAX_NUM_FEATURES+findex] = PHOTO_SCALE*H[i][j]; // symmetric
					}
				}
			}
			Hg[(7*8 + 7)*MAX_NUM_FEATURES+findex] = H[7][7];
		}
	}

	__syncthreads();
}

//////////////////////////////////////////////////////////////////////////
// 
//	__global__ void
//	ComputeHessian (float *fx, float *fy, int numfeature, int *idmap, float *Hg,  
//					int t_size, unsigned int width, unsigned int height)
//
//		Kernel for Hessian computation (Track thread)
//			Inverse on a gpu
//
//////////////////////////////////////////////////////////////////////////
__global__ void
ComputeHessian_invert(float *fx, float *fy, int numfeature, int *idmap, float *Hg,  int t_size, unsigned int width, unsigned int height)
{
	float gx, gy, w, gwx, gwy, gwx2, gwy2, gwxy;
	float tw, tgwx, tgwy, t, dx, dy, dx2, dy2, dxy;
	float H[8][8],H_inv[8][8];
	
	float2 pos, addr;
	int findex = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;

	if (findex < numfeature) 
	{
		int globalindex = idmap[findex];

		pos.x = fx[globalindex];
		pos.y = fy[globalindex];

		for (int i=0;i < 8; i++)
			for (int j=0; j < 8; j++) H[i][j] = 0;

		for (dx = -t_size; dx <= t_size; dx++) {
			for (dy = -t_size; dy <= t_size; dy++)
			{
				addr.x = pos.x + dx;
				addr.y = pos.y + dy;
				if ((addr.x >= 1) && (addr.y >= 1) && (addr.x < width-2) && (addr.y < height-2)) {

					gx = tex2D(Gx_Texture, addr.x, addr.y);
					gy = tex2D(Gy_Texture, addr.x, addr.y);
					w  = tex2D(W_Texture,  addr.x, addr.y);
					t  = tex2D(I_Texture,  addr.x, addr.y);

					dx2	 = dx*dx;	dy2	 = dy*dy;	dxy	 = dx*dy;
					gwx	 = w*gx; 	gwy	 = w*gy;	gwx2 = gwx*gx;	gwy2 = gwy*gy;
					gwxy = gx*gwy;	tgwx = gwx*t;	tgwy = gwy*t;	tw	 = w*t;

					H[0][0] += gwx2;		H[0][1] += gwxy;		H[0][2] += dx*gwx2;		
					H[0][3] +=  dx*gwxy;	H[0][4] += dy*gwx2;		H[0][5] += dy*gwxy;
					H[0][6] += tgwx;		H[0][7] += gwx;

					H[1][1] += gwy2;		H[1][2] += dx*gwxy;		H[1][3] += dx*gwy2;
					H[1][4] += dy*gwxy;		H[1][5] += dy*gwy2;		H[1][6] += tgwy;
					H[1][7] += gwy;

					H[2][2] += dx2*gwx2;	H[2][3] += dx2*gwxy;	H[2][4] += dxy*gwx2;
					H[2][5] += dxy*gwxy;	H[2][6] += dx*tgwx;		H[2][7] += dx*gwx;

					H[3][3] += dx2*gwy2;	H[3][4] += dxy*gwxy;	H[3][5] += dxy*gwy2;
					H[3][6] += dx*tgwy;		H[3][7] += dx*gwy;		
					
					H[4][4] += dy2*gwx2;	H[4][5] += dy2*gwxy;	H[4][6] += dy*tgwx;
					H[4][7] += dy*gwx;

					H[5][5] += dy2*gwy2;	H[5][6] += dy*tgwy;		H[5][7] += dy*gwy;

					H[6][6] += t*tw;		H[6][7] += tw;	

					H[7][7] += w;
				}
			}
		}
		int index = __mul24(findex, 64);
		float temp = H[7][7];
		for (int i=0; i < 8; i++) {
			for (int j=i; j < 8; j++) {
				if (j != 7) {
					H[i][j] = PHOTO_SCALE_SQ*H[i][j];
					H[j][i] = H[i][j]; // symmetric
				}
				else {
					H[i][j] = PHOTO_SCALE*H[i][j];
					H[j][i] = H[i][j]; // symmetric
				}
			}
		}
		H[7][7] = temp; //H[7][7];

		InvertMatrix<8>(H, H_inv);

		// Transfer to global memory
		for (int i=0; i < 8; i++) {
			for (int j=0; j < 8; j++) {
				Hg[(i*8 + j)*MAX_NUM_FEATURES+findex] = H_inv[i][j];		
			}
		}

	}

	__syncthreads();
}


//////////////////////////////////////////////////////////////////////////
// 
//	__global__ void
//	RegisterFeatures(float *fx, float *fy, int *idmap, float *templ_H_device,   
//					 float *Gx, float *Gy, float *W, float *T, float *templ_Hessian, float *motion,
//					 int t_size, unsigned int width, unsigned int height, int numfeatures)
//
//		Kernel for Feature registation (Track thread)
//
//////////////////////////////////////////////////////////////////////////
__global__ void
RegisterFeatures(float *fx, float *fy, int *idmap, float *templ_H_device, 
				 float *Gx, float *Gy, float *W, float *T, float *templ_Hessian, float *motion, 
				 int t_size, unsigned int width, unsigned int height, int numfeatures)
{
	int findex, globalindex;
	int dx, dy;
	float2 pos, pixel;
	float gx, gy, w, t;
		
	findex = blockIdx.x * blockDim.x + threadIdx.x;
	
	if (findex < numfeatures)
	{
		globalindex = idmap[findex];

		pos.x = fx[globalindex];
		pos.y = fy[globalindex];
		
		int id=0;
		for (dx = -t_size; dx <= t_size; dx++)
			for (dy = -t_size; dy <= t_size; dy++)
			{
				pixel.x = pos.x + dx;
				pixel.y = pos.y + dy;
				
				if((pixel.x >= 2) && (pixel.x < width-2) && (pixel.y >= 2) && (pixel.y < height-2))
				{
					gx = tex2D(Gx_Texture, pixel.x, pixel.y);
					gy = tex2D(Gy_Texture, pixel.x, pixel.y);
					w  = tex2D(W_Texture,  pixel.x, pixel.y);
					t  = tex2D(I_Texture,  pixel.x, pixel.y);
				}
				else {
					gx = 0;
					gy = 0;
					w  = 0;
					t  = 0;
				}

				int index = MAX_NUM_FEATURES*id + globalindex;

				Gx[index] = gx;
				Gy[index] = gy;
				W [index] = w;
				T [index] = t;

				id++;
			}

			// Transfer to global memory
			for (int i=0; i < 8; i++) {
				for (int j=0; j < 8; j++) {
					templ_Hessian[(i*8 + j)*MAX_NUM_FEATURES + globalindex] = templ_H_device[(i*8 + j)*MAX_NUM_FEATURES + findex];
				}
			}

			motion[0*MAX_NUM_FEATURES + globalindex] = 0.0f;
			motion[1*MAX_NUM_FEATURES + globalindex] = 0.0f;
			motion[2*MAX_NUM_FEATURES + globalindex] = 1.0f;
			motion[3*MAX_NUM_FEATURES + globalindex] = 0.0f;
			motion[4*MAX_NUM_FEATURES + globalindex] = 0.0f;
			motion[5*MAX_NUM_FEATURES + globalindex] = 1.0f;
			motion[6*MAX_NUM_FEATURES + globalindex] = 1.0f;
			motion[7*MAX_NUM_FEATURES + globalindex] = 0.0f;
	}

	__syncthreads();
}

//////////////////////////////////////////////////////////////////////////
// 
//	__global__ void
//	Make_Pyramid(int pyramidlevel, float *pyrImage, unsigned int width, unsigned int height)
//
//		Kernel for building pyramid images (pixel thread)
//
//////////////////////////////////////////////////////////////////////////
__global__ void
Make_Pyramid(int pyramidlevel, float *pyrImage, unsigned int width, unsigned int height)

{
	float2 pixel;
	int x = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;
	int y = __mul24(blockIdx.y, blockDim.y) + threadIdx.y;
	if((x<width)&&(y<height))
	{
		int index = __mul24(y, width) + x;

		if (pyramidlevel > 0)	{
			pixel.x = 2*x;
			pixel.y = 2*y; // half size resampling
		}
		else {
			pixel.x = x;
			pixel.y = y;
		}

		float t = tex2D(I_Texture, pixel.x, pixel.y);
		pyrImage[index] = t;
	}
}

//////////////////////////////////////////////////////////////////////////
// 
//	__global__ void 
//	SmoothingCUDA(float *input,  float *temp, int width, int height)
//
//		Kernel for smoothing images (pixel thread)
//
//		NOT USED 
//		NOTE that this is not efficient, because of noncoalesced access
//
//////////////////////////////////////////////////////////////////////////
__global__ void 
SmoothingCUDA(float *input,  float *temp, int width, int height)
{
	float filter[5] = {0.0545, 0.2442, 0.4026, 0.2442, 0.0545};

	int xx = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;
	int yy = __mul24(blockIdx.y, blockDim.y) + threadIdx.y;

	if ((xx > 2) && (xx < width-2)) {
		float filterx=0;
		for (int ii=-2; ii < 2; ii++)
			filterx += filter[ii+2]*input[xx + ii + yy*width];
		temp[xx+width*yy] = filterx;
	}
	__syncthreads();
}

//////////////////////////////////////////////////////////////////////////
// 
//	void 
//	SmoothingCUDA_Gaussian5x5(float *input, float* temp, int width, int height)
//
//		Kernel for smoothing images (pixel thread)
//
//////////////////////////////////////////////////////////////////////////
void 
SmoothingCUDA_Gaussian5x5(float *input, float* temp, int width, int height)
{
	dim3 blockGridRows(iDivUp(width, ROW_TILE_W), height);
    dim3 blockGridColumns(iDivUp(width, COLUMN_TILE_W), iDivUp(height, COLUMN_TILE_H));
    dim3 threadBlockRows(KERNEL_RADIUS_ALIGNED + ROW_TILE_W + 2);
    dim3 threadBlockColumns(COLUMN_TILE_W, 8);

      convolutionRowGPU<<<blockGridRows, threadBlockRows>>>(
            temp,
            input,
            width,
            height
        );
      convolutionColumnGPU<<<blockGridColumns, threadBlockColumns>>>(
           input,
            temp,
            width,
            height,
            COLUMN_TILE_W * threadBlockColumns.y,
            width * threadBlockColumns.y
        );
	
}
//////////////////////////////////////////////////////////////////////////
// 
//	extern "C" void Caller_Tracker_Registration(float* fx, float *fy, int numfeatures, int *indexmap)
//
//		Caller function for tracker registration
//
//		NOTE that there are two kinds of operations: pixel and track
//				  by making different numbers of threads
//		Inverting Hessian matrices is on the CPU by transferring data from the GPU.
//		Current implementation is for accessing hessians in coalesced manner.
//
//////////////////////////////////////////////////////////////////////////
extern "C" void Caller_Tracker_Registration(float* fx, float *fy, int numfeatures, int *indexmap)
{
	int pyr;
	
	// registration to templ_Gx, templ_Gy, templ_W, templ_T
	cudaMemset(new_id_map, 0, MAX_NUM_FEATURES*sizeof(int));
	cudaMemcpy(new_id_map, indexmap, numfeatures*sizeof(int), cudaMemcpyHostToDevice);
	
	for (pyr=0; pyr < pyramidLevel; pyr++)
	{
		dim3 threads(pyrThreads[pyr],1);
		dim3 grid = dim3(iDivUp(imgWidth[pyr],threads.x), imgHeight[pyr]/threads.y);
		dim3 thread_feature(64,1);
		dim3 grid_feature = dim3(16, 1);

		if (pyr == 0) {
			cudaMemcpy(new_featureX[pyr], fx, MAX_NUM_FEATURES*sizeof(float), cudaMemcpyHostToDevice);
			cudaMemcpy(new_featureY[pyr], fy, MAX_NUM_FEATURES*sizeof(float), cudaMemcpyHostToDevice);
		}
		else {
			cudaMemcpy(new_featureX[pyr], new_featureX[pyr-1], MAX_NUM_FEATURES*sizeof(float), cudaMemcpyDeviceToDevice);
			cudaMemcpy(new_featureY[pyr], new_featureY[pyr-1], MAX_NUM_FEATURES*sizeof(float), cudaMemcpyDeviceToDevice);
			scaleDownFeatures<<< grid_feature , thread_feature >>> (new_featureX[pyr], new_featureY[pyr]);
		}
		if(pyr>0)
		{
			SmoothingCUDA_Gaussian5x5(result[pyr-1],tempmax[pyr-1], imgWidth[pyr-1],imgHeight[pyr-1]);
			CUDA_SAFE_CALL( cudaMemcpy2DToArray(TempArr[pyr-1], 0, 0, result[pyr-1], imgWidth[pyr-1]*sizeof(float), imgWidth[pyr-1]*sizeof(float), imgHeight[pyr-1], cudaMemcpyDeviceToDevice) );
			CUDA_SAFE_CALL( cudaBindTextureToArray(I_Texture ,TempArr[pyr-1]) );
		}


		//copy to cuda array
		Make_Pyramid<<< grid , threads >>> (pyr, result[pyr], imgWidth[pyr],imgHeight[pyr]);

		CUDA_SAFE_CALL( cudaMemcpy2DToArray(TArr[pyr], 0, 0, result[pyr], imgWidth[pyr]*sizeof(float), imgWidth[pyr]*sizeof(float), imgHeight[pyr], cudaMemcpyDeviceToDevice) );
		CUDA_SAFE_CALL( cudaBindTextureToArray(I_Texture ,TArr[pyr]) );
		CUDA_SAFE_CALL( cudaThreadSynchronize());

		spatialDerivKernel<<< grid , threads >>> (Gx[pyr], Gy[pyr], W[pyr],  imgWidth[pyr], imgWidth[pyr], imgHeight[pyr]);

		CUDA_SAFE_CALL( cudaMemcpy2DToArray(GxArr[pyr], 0, 0, Gx[pyr], imgWidth[pyr]*sizeof(float), imgWidth[pyr]*sizeof(float), imgHeight[pyr], cudaMemcpyDeviceToDevice) );
		CUDA_SAFE_CALL( cudaMemcpy2DToArray(GyArr[pyr], 0, 0, Gy[pyr], imgWidth[pyr]*sizeof(float), imgWidth[pyr]*sizeof(float), imgHeight[pyr], cudaMemcpyDeviceToDevice) );
		CUDA_SAFE_CALL( cudaMemcpy2DToArray(WArr[pyr],  0, 0, W[pyr],  imgWidth[pyr]*sizeof(float), imgWidth[pyr]*sizeof(float), imgHeight[pyr], cudaMemcpyDeviceToDevice) );
		CUDA_SAFE_CALL( cudaBindTextureToArray(Gx_Texture, GxArr[pyr]) );
		CUDA_SAFE_CALL( cudaBindTextureToArray(Gy_Texture, GyArr[pyr]) );
		CUDA_SAFE_CALL( cudaBindTextureToArray(W_Texture,  WArr[pyr] ) );

		CUDA_SAFE_CALL( cudaThreadSynchronize());
		ComputeHessian_invert<<< grid_feature , thread_feature >>> (new_featureX[pyr], new_featureY[pyr], numfeatures, new_id_map,
			templ_H_device[pyr], (templSize[pyr]-1)/2, imgWidth[pyr], imgHeight[pyr]);
		CUDA_SAFE_CALL( cudaThreadSynchronize());
		
/*		/// comments == using cpu for matrix inversion
		//
		 ComputeHessian<<< grid_feature , thread_feature >>> (new_featureX[pyr], new_featureY[pyr], numfeatures, new_id_map,
			templ_H_device[pyr], (templSize[pyr]-1)/2, imgWidth[pyr], imgHeight[pyr]);

	//	 inverse hessian using LAPACK - on cpu
	//	 column-wise? -- symmetric 

		cudaMemcpy(templ_H_host[pyr], templ_H_device[pyr], 64*MAX_NUM_FEATURES*sizeof(float), cudaMemcpyDeviceToHost);

		int m = 8, n = 8, info, lwork = 8;
		float *tempH = new float[64];

		for (int i=0; i < numfeatures; i++) {
			// reassgin the hessian matrix for using LAPACK    - additional computation
			for (int elem =0; elem<64; elem++)
				tempH[elem] = templ_H_host[pyr][elem*MAX_NUM_FEATURES+i];

			sgetrf_(&m, &n, tempH, &m, ipiv, &info);
			sgetri_(&n, tempH, &m, ipiv, work, &lwork, &info);

			// reassgin the hessian matrix for coalesced access - additional computation
			for (int elem =0; elem<64; elem++)
				templ_H_host[pyr][elem*MAX_NUM_FEATURES+i] = tempH[elem];
		}
		cudaMemcpy(templ_H_device[pyr], templ_H_host[pyr], 64*MAX_NUM_FEATURES*sizeof(float), cudaMemcpyHostToDevice);
*/

		cudaMemcpy(featureX[pyr], new_featureX[pyr], MAX_NUM_FEATURES*sizeof(float), cudaMemcpyDeviceToDevice);
		cudaMemcpy(featureY[pyr], new_featureY[pyr], MAX_NUM_FEATURES*sizeof(float), cudaMemcpyDeviceToDevice);

		RegisterFeatures<<< grid_feature , thread_feature >>>(new_featureX[pyr], new_featureY[pyr], new_id_map, templ_H_device[pyr],
				templ_Gx[pyr], templ_Gy[pyr], templ_W[pyr], templ_T[pyr], templ_Hessian[pyr], motionParams[pyr],
				(templSize[pyr]-1)/2, imgWidth[pyr], imgHeight[pyr], numfeatures);
				
		CUDA_SAFE_CALL( cudaThreadSynchronize());				
	}

	CUDA_SAFE_CALL( cudaThreadSynchronize());
}

//////////////////////////////////////////////////////////////////////////
// 
//	__global__ void
//	ComputeB(float *fx, float *fy, float* Gx, float* Gy, float* W, float* T, float *H, float *motion, float *B, 
//			 int t_size, unsigned int width, unsigned int height)
//
//		Kernel for computing B vectors (Track thread)
//
//		NOTE the usage of the shared memory and __syncthreads()
//
//////////////////////////////////////////////////////////////////////////
#define NUM_B_THREADS	64
__global__ void
ComputeB(float *fx, float *fy, float* Gx, float* Gy, float* W, float* T, float *H, float *motion, float *B, int t_size, unsigned int width, unsigned int height)
{
	int findex = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;

	float2 pixel;
	float t, t2, w, dx, dy;
	float e, ew, ewgx, ewgy;

	__shared__ float b[9*NUM_B_THREADS];
	for (int i=0; i < 9; i++) b[i*NUM_B_THREADS + threadIdx.x] = 0;

	float m[8];
	for (int i=0; i < 8; i++) m[i] = motion[i*MAX_NUM_FEATURES + findex];

	float px = fx[findex] + m[0];
	float py = fy[findex] + m[1];

	__syncthreads(); // end of initialization of data

	// Affine Warping
	int index = findex - MAX_NUM_FEATURES;
	for (dx = -t_size; dx <= t_size; dx++)
	{
		float m2dx = m[2]*dx + px;
		float m3dx = m[3]*dx + py;
		for (dy = -t_size; dy <= t_size; dy++)
		{
			index += MAX_NUM_FEATURES;
			w = W[index];
			t = T[index];

			pixel.x = (float)(m2dx + m[4]*dy);
			pixel.y = (float)(m3dx + m[5]*dy);

			if ((pixel.x >= 1) && (pixel.x < width-2) && (pixel.y >= 1) && (pixel.y < height-2))
			{
				//
				// e = 1/alpha*(I_w - beta) - beta
				//   = 1/alpha*(t2 - beta) - t
				//
				
				t2 = tex2D(I_Texture, pixel.x, pixel.y);
				e = (t2-m[7])/m[6] - t;

				ew =  w*e;
				ewgx = ew*Gx[index];
				ewgy = ew*Gy[index];

				b[0*NUM_B_THREADS + threadIdx.x] += ewgx;
				b[1*NUM_B_THREADS + threadIdx.x] += ewgy;
				b[2*NUM_B_THREADS + threadIdx.x] += (dx*ewgx);
				b[3*NUM_B_THREADS + threadIdx.x] += (dx*ewgy);
				b[4*NUM_B_THREADS + threadIdx.x] += (dy*ewgx);
				b[5*NUM_B_THREADS + threadIdx.x] += (dy*ewgy);
				b[6*NUM_B_THREADS + threadIdx.x] += (ew*t);
				b[7*NUM_B_THREADS + threadIdx.x] += ew;
				b[8*NUM_B_THREADS + threadIdx.x] += e*e;  // residual
			}
		}
	}

	__syncthreads(); // end of process, now copy the shared memory to the global memory

	for (int i=0; i < 7; i++) {
		B[i*MAX_NUM_FEATURES+findex] = b[i*NUM_B_THREADS + threadIdx.x]*PHOTO_SCALE_SQ;
	}
	B[7*MAX_NUM_FEATURES+findex] = b[7*NUM_B_THREADS + threadIdx.x]*PHOTO_SCALE;
	B[8*MAX_NUM_FEATURES+findex] = b[8*NUM_B_THREADS + threadIdx.x];			    
} 

//////////////////////////////////////////////////////////////////////////
// 
//	__global__ void
//	ScaleUpMotionParameters(float *ml, float *mh) 
//
//		Kernel for scaling the motion parameters up (Track thread)
//
//////////////////////////////////////////////////////////////////////////
__global__ void
ScaleUpMotionParameters(float *ml, float *mh)
{
	int findex = __mul24(blockIdx.x,blockDim.x) + threadIdx.x;
	
	for (int i=2; i < 8; i++) {
		mh[i*MAX_NUM_FEATURES + findex] = ml[i*MAX_NUM_FEATURES + findex];	
	}

	mh[0*MAX_NUM_FEATURES + findex] = 2*ml[0*MAX_NUM_FEATURES + findex];	
	mh[1*MAX_NUM_FEATURES + findex] = 2*ml[1*MAX_NUM_FEATURES + findex];
}

//////////////////////////////////////////////////////////////////////////
// 
//	__global__ void
//  ScaleDownMotionParameters(float *mh, float *ml, float* fxh, float *fxl, float *fyh, float* fyl)
//
//		Kernel for scaling the motion parameters and features down (Track thread)
//
//////////////////////////////////////////////////////////////////////////
__global__ void
ScaleDownMotionParameters(float *mh, float *ml, float* fxh, float *fxl, float *fyh, float* fyl)
{
	int findex = __mul24(blockIdx.x,blockDim.x) + threadIdx.x;

	for (int i=2; i < 8; i++) {
		ml[i*MAX_NUM_FEATURES+findex] = mh[i*MAX_NUM_FEATURES+findex];	
	}

	ml[0*MAX_NUM_FEATURES + findex] = mh[0*MAX_NUM_FEATURES + findex]/2.0f;	
	ml[1*MAX_NUM_FEATURES + findex] = mh[1*MAX_NUM_FEATURES + findex]/2.0f;

	fxl[findex] = fxh[findex]/2.0f;
	fyl[findex] = fyh[findex]/2.0f; 
}

//////////////////////////////////////////////////////////////////////////
// 
//	__global__ void
//  RefineMotionFromIMU(float *motion, float *fx, float *fy)
//
//		Kernel for Motion refinement using data from an IMU (Track thread)
//
//////////////////////////////////////////////////////////////////////////
__global__ void
RefineMotionFromIMU(float *motion, float *fx, float *fy)
{
	int findex = __mul24(blockIdx.x,blockDim.x) + threadIdx.x;
	
	float m[8], mc[8], tt, px, py;
	for (int i=0; i < 8; i++) m[i] = motion[i*MAX_NUM_FEATURES + findex];

	px = fx[findex];
	py = fy[findex];

	//
	//       [h0  h3  h6]   [m2  m4  m0+px]  
	//  m' = [h1  h4  h7] * [m3  m5  m1+py]
	//       [h2  h5  h8]   [ 0   0  1    ]  
	//
	mc[0] = Hmg[0]*(m[0] + px) + Hmg[3]*(m[1] + py) + Hmg[6];
	mc[1] = Hmg[1]*(m[0] + px) + Hmg[4]*(m[1] + py) + Hmg[7];
	tt	  = Hmg[2]*(m[0] + px) + Hmg[5]*(m[1] + py) + Hmg[8];

	mc[2] = Hmg[0]*m[2] + Hmg[3]*m[3];
	mc[3] = Hmg[1]*m[2] + Hmg[4]*m[3];
	mc[4] = Hmg[0]*m[4] + Hmg[3]*m[5];
	mc[5] = Hmg[1]*m[4] + Hmg[4]*m[5];

	motion[0*MAX_NUM_FEATURES + findex] = mc[0]/tt - px;
	motion[1*MAX_NUM_FEATURES + findex] = mc[1]/tt - py;
	motion[2*MAX_NUM_FEATURES + findex] = mc[2]/tt;
	motion[3*MAX_NUM_FEATURES + findex] = mc[3]/tt;
	motion[4*MAX_NUM_FEATURES + findex] = mc[4]/tt;
	motion[5*MAX_NUM_FEATURES + findex] = mc[5]/tt;
}

//////////////////////////////////////////////////////////////////////////
// 
//	__global__ void
//  ComputeMotionUpdateDirection(float *H, float *b, float *dir)
//
//		Kernel for computing H^(-1) b (Track thread)
//
//////////////////////////////////////////////////////////////////////////
#define CLAMP(x, y) ((x > 0) ? min(x, y) : max(x, -y));

__global__ void
ComputeMotionUpdateDirection(float *H, float *b, float *dir)
{
	int findex = __mul24(blockIdx.x,blockDim.x) + threadIdx.x;

	// dir = H^(-1) b
	for (int i=0; i < 8; i++) {
		float d = 0;
		for (int j=0; j < 8; j++) {
			d += H[((8*i) + j)*MAX_NUM_FEATURES + findex] * b[j*MAX_NUM_FEATURES + findex]; 
		}
		dir[i*MAX_NUM_FEATURES + findex] = d;
	}

	__syncthreads();
}

//////////////////////////////////////////////////////////////////////////
// 
//	__global__ void
//  ComputeInitialCost(float *fx, float *fy, float* W, float* T, float *motion,
//					   float *mincost, float *bestscale, float *scale,
//					   int t_size, unsigned int width, unsigned int height)
//
//		Kernel for computing initial cost with scale = 0 (Track thread)
//
//		THINK about how to update the scales in each iteration!
//
//////////////////////////////////////////////////////////////////////////
__global__ void
ComputeInitialCost(float *fx, float *fy, float* W, float* T, float *motion, 
				   float *mincost, float *bestscale, float *scale,
				   int t_size, unsigned int width, unsigned int height)
{
	float2 pixel;
	float t, t2, w, dx, dy;
	int findex = __mul24(blockIdx.x,blockDim.x) + threadIdx.x;

	float mc[8];
	for (int i=0; i < 8; i++) mc[i] = motion[i*MAX_NUM_FEATURES + findex];

	float px = fx[findex] + mc[0];
	float py = fy[findex] + mc[1];

	__syncthreads();

	// Affine Warping
	float e, cost = 0, wsum = 0;
	int index = findex - MAX_NUM_FEATURES; 
	for (dx = -t_size; dx <= t_size; dx++)
	{
		float m2dx = mc[2]*dx + px;
		float m3dx = mc[3]*dx + py;
		for (dy = -t_size; dy <= t_size; dy++)
		{
			index += MAX_NUM_FEATURES;

			w = W[index];
			t = T[index];
			pixel.x = (float)(m2dx + mc[4]*dy);
			pixel.y = (float)(m3dx + mc[5]*dy);

			if ((pixel.x >= 0) && (pixel.x < width) && (pixel.y >= 0) && (pixel.y < height))
			{
				t2 = tex2D(I_Texture, pixel.x, pixel.y);
				e = (t2-mc[7])/mc[6] - t;

				cost += w*e*e;
				wsum += w;
			}
		}
	}

	if (wsum > 20) cost /= wsum;
	else cost = BIG_COST;
	// compute shear amount
	float sheararea = fabs(mc[2]*mc[5]-mc[4]*mc[3]);
	float factor1 = fabs((mc[2]+mc[5])+sqrt((mc[2]-mc[5])*(mc[2]-mc[5])+4*mc[3]*mc[4]));
	float factor2 = fabs((mc[2]+mc[5])-sqrt((mc[2]-mc[5])*(mc[2]-mc[5])+4*mc[3]*mc[4]));
	float maxx = max(factor1,factor2);
	float minx = min(factor1,factor2);
	
	if (minx/maxx<0.5) 
		cost += BIG_COST;

	if ((sheararea<0.6)||(sheararea>2.2))
		cost = BIG_COST;
		
	__syncthreads();

	mincost[findex] = cost;
	bestscale[findex] = 0.0f;	// initial scale
	scale[findex] = 1.0f;		// next scale
	
	__syncthreads();
}

//////////////////////////////////////////////////////////////////////////
// 
//	__global__ void
//  ComputeScale(float *fx, float *fy, float* W, float* T, float *motion, float *dir, 
//				  float *mincost, float *bestscale, float *scale,
//				  int t_size, unsigned int width, unsigned int height)
//
//		Kernel for computing cost with a specific scale and comparing it with the best one (Track thread)
//
//		THINK about how to update the scales in each iteration!
//
//////////////////////////////////////////////////////////////////////////
__global__ void
ComputeScale(float *fx, float *fy, float* W, float* T, float *motion, float *dir, 
			  float *mincost, float *bestscale, float *scale,
			  int t_size, unsigned int width, unsigned int height)
{
	float2 pixel;
	float t, t2, w, dx, dy;

	int findex = __mul24(blockIdx.x,blockDim.x) + threadIdx.x;
	float s = scale[findex];

	float c[6], d[8], m[8], mc[8];
	for (int i=0; i < 8; i++) d[i] =  CLAMP(s*dir[i*MAX_NUM_FEATURES + findex], 1e3f); //s*dir[i*MAX_NUM_FEATURES + findex];
	for (int i=0; i < 8; i++) m[i] = motion[i*MAX_NUM_FEATURES + findex];
	for (int i=0; i < 8; i++) mc[i] = m[i];

	/*
	syms d0 d1 d2 d3 d4 d5;

	A = [1+d2    d4  d0;
		   d3  1+d5  d1;
			0     0   1];
	    
	dt = (1+d2)*(1+d5) - d3*d4;
	    
	B = 1/dt*[1+d5   -d4     d1*d4-d0*d5-d0;
			   -d3  1+d2     d0*d3-d1*d2-d1;
				 0     0     dt           ];
	         
	c2 = (1+d5)/dt; c4 =   -d4 /dt; c0 = -(c2*d0 + c4*d1);
	c3 =   -d3 /dt; c5 = (1+d2)/dt; c1 = -(c3*d0 + c5*d1);

	C = [c2  c4 c0;
		 c3  c5 c1;
		  0   0  1];
	  
	simplify(A*C)
	*/
	//
	//       [m2  m4  m0]   [1+d2    d4  d0]^-1
	//  m' = [m3  m5  m1] * [  d3  1+d5  d1]
	//       [ 0   0   1]   [   0     0   1]   
	//
	//
	//  det = (1+d2)*(1+d5) - d3*d4
	//
	//       [m2  m4  m0]            [1+d5   -d4     d1*d4-d0*d5-d0 ]
	//  m' = [m3  m5  m1] * (1/det) *[ -d3  1+d2     d0*d3-d1*d2-d1 ]
	//       [ 0   0   1]            [   0     0     det            ]
	//
	//       [m2  m4  m0]   [c2  c4  c0]
	//  m' = [m3  m5  m1] * [c3  c5  c1]
	//       [ 0   0   1]   [ 0   0   1]
	//

	float det = (1+d[2])*(1+d[5]) - d[3]*d[4];

	c[2] =  (1+d[5])/det;
	c[3] =    -d[3] /det;
	c[4] =    -d[4] /det;
	c[5] =  (1+d[2])/det;
	c[0] = -(c[2]*d[0] + c[4]*d[1]);
	c[1] = -(c[3]*d[0] + c[5]*d[1]);

	mc[0] = m[2]*c[0] + m[4]*c[1] + m[0];
	mc[1] = m[3]*c[0] + m[5]*c[1] + m[1];
	
	mc[2] = m[2]*c[2] + m[4]*c[3];
	mc[3] = m[3]*c[2] + m[5]*c[3];
	mc[4] = m[2]*c[4] + m[4]*c[5];
	mc[5] = m[3]*c[4] + m[5]*c[5];
	
	mc[6] = m[6]*(1.0f + d[6]);
	mc[7] = m[7] + m[6]*d[7]/PHOTO_SCALE;


	float px = fx[findex] + mc[0];
	float py = fy[findex] + mc[1];

	__syncthreads();

	// Affine Warping
	float e, cost = 0, wsum = 0;
	int index = findex - MAX_NUM_FEATURES; //+__mul24(id,MAX_NUM_FEATURES);
	for (dx=-t_size; dx<=t_size; dx++)
	{
		float m2dx = mc[2]*dx+px;
		float m3dx = mc[3]*dx+py;
		for (dy = -t_size; dy <= t_size; dy++)
		{
			index += MAX_NUM_FEATURES;

			w = W[index];
			t = T[index];
			pixel.x = (float)(m2dx + mc[4]*dy);
			pixel.y = (float)(m3dx + mc[5]*dy);

			if ((pixel.x >= 0) && (pixel.x < width) && (pixel.y >= 0) && (pixel.y < height))
			{
				t2 = tex2D(I_Texture, pixel.x, pixel.y);
				e = (t2-mc[7])/mc[6] - t;
				cost += w*e*e;
				wsum += w;
			}
		}
	}

	if (wsum != 0) cost /= wsum;
	else cost = BIG_COST;
	
	// compute shear amount
	float sheararea = fabs(mc[2]*mc[5]-mc[4]*mc[3]);
	float factor1 = fabs((mc[2]+mc[5])+sqrt((mc[2]-mc[5])*(mc[2]-mc[5])+4*mc[3]*mc[4]));
	float factor2 = fabs((mc[2]+mc[5])-sqrt((mc[2]-mc[5])*(mc[2]-mc[5])+4*mc[3]*mc[4]));
	float maxx = max(factor1,factor2);
	float minx = min(factor1,factor2);
	
	if (minx/maxx<0.5) 
		cost += BIG_COST;
	if ((sheararea<0.6)||(sheararea>2.2))
		cost = BIG_COST;
	
	__syncthreads();



	scale[findex] = s/2.0f; // s - 0.1f;
	if (cost < mincost[findex])	{
		mincost[findex] = cost;
		bestscale[findex] = s;
	}
	if (cost > VALIDITY_CHECK_COST)
	{
		mincost[findex] = BIG_COST;
		bestscale[findex] = 0.0f;
	}

	__syncthreads();
}

//////////////////////////////////////////////////////////////////////////
// 
//	__global__ void
//  UpdateMotion_BestScale(float *motion, float *dir, float *bestscale) 
//
//		Kernel for updating the motion vectors using the best scale (Track thread)
//
//////////////////////////////////////////////////////////////////////////
__global__ void
UpdateMotion_BestScale(float *motion, float *dir, float *bestscale)
{
	int findex = __mul24(blockIdx.x,blockDim.x) + threadIdx.x;
	float s = bestscale[findex];

	float d[8], m[8], c[6];
	for (int i=0; i < 8; i++) d[i] =  CLAMP(s*dir[i*MAX_NUM_FEATURES + findex], 1e3f); //s*dir[i*MAX_NUM_FEATURES + findex];
	for (int i=0; i < 8; i++) m[i] = motion[i*MAX_NUM_FEATURES + findex];
	
	float det = (1+d[2])*(1+d[5]) - d[3]*d[4];

	c[2] =  (1+d[5])/det;
	c[3] =    -d[3] /det;
	c[4] =    -d[4] /det;
	c[5] =  (1+d[2])/det;
	c[0] = -(c[2]*d[0] + c[4]*d[1]);
	c[1] = -(c[3]*d[0] + c[5]*d[1]);

	motion[0*MAX_NUM_FEATURES + findex] = m[2]*c[0] + m[4]*c[1] + m[0];
	motion[1*MAX_NUM_FEATURES + findex] = m[3]*c[0] + m[5]*c[1] + m[1];
	
	motion[2*MAX_NUM_FEATURES + findex] = m[2]*c[2] + m[4]*c[3];
	motion[3*MAX_NUM_FEATURES + findex] = m[3]*c[2] + m[5]*c[3];
	motion[4*MAX_NUM_FEATURES + findex] = m[2]*c[4] + m[4]*c[5];
	motion[5*MAX_NUM_FEATURES + findex] = m[3]*c[4] + m[5]*c[5];
	
	motion[6*MAX_NUM_FEATURES + findex] = m[6]*(1.0f + d[6]);
	motion[7*MAX_NUM_FEATURES + findex] = m[7] + m[6]*d[7]/PHOTO_SCALE;
	
	__syncthreads();
}

//////////////////////////////////////////////////////////////////////////
// 
//	extern "C" void 
//		Caller_Align(cudaArray *Image2, float* motion, float *residual, float *vec_invH)
//
//		Caller function for Track threads
//		outputs: motion and residual
//
//////////////////////////////////////////////////////////////////////////
extern "C" void 
Caller_Align(cudaArray *Image2, float* motion, float *residual, float *vec_invH)
{
	dim3 thread_feature(NUM_B_THREADS, 1);
	dim3 grid_feature  = dim3(MAX_NUM_FEATURES/NUM_B_THREADS, 1);
	
	CUDA_SAFE_CALL( cudaBindTextureToArray(I_Texture, Image2) );
	CUDA_SAFE_CALL( cudaMemcpyToSymbol(Hmg, vec_invH, sizeof(float)*9));
	
	// 0. Refine motion using IMU
	RefineMotionFromIMU<<< grid_feature , thread_feature >>> (motionParams[0], featureX[0], featureY[0]);
	CUDA_SAFE_CALL( cudaThreadSynchronize());

	// 1. Make pyramid image
	for (int pyr=0; pyr < pyramidLevel; pyr++)
	{ 
		dim3 threads(pyrThreads[pyr],1);
		dim3 grid = dim3(iDivUp(imgWidth[pyr],threads.x), imgHeight[pyr]/threads.y);

		if(pyr>0)
		{
			SmoothingCUDA_Gaussian5x5(result[pyr-1],tempmax[pyr-1], imgWidth[pyr-1],imgHeight[pyr-1]);
			CUDA_SAFE_CALL( cudaMemcpy2DToArray(TempArr[pyr-1], 0, 0, result[pyr-1], imgWidth[pyr-1]*sizeof(float), imgWidth[pyr-1]*sizeof(float), imgHeight[pyr-1], cudaMemcpyDeviceToDevice) );
			CUDA_SAFE_CALL( cudaBindTextureToArray(I_Texture ,TempArr[pyr-1]) );
		}

		Make_Pyramid<<< grid , threads >>> (pyr, result[pyr], imgWidth[pyr], imgHeight[pyr]);
		
		if (pyr < pyramidLevel-1) {
			ScaleDownMotionParameters<<< grid_feature , thread_feature >>> (motionParams[pyr], motionParams[pyr+1],
				featureX[pyr], featureX[pyr+1], featureY[pyr], featureY[pyr+1]);
		}
		
		CUDA_SAFE_CALL( cudaMemcpy2DToArray(TArr[pyr], 0, 0, result[pyr],
											imgWidth[pyr]*sizeof(float), imgWidth[pyr]*sizeof(float), 
											imgHeight[pyr], cudaMemcpyDeviceToDevice) );
	}

	// 3. Align from the coarsest level
	for (int pyr = pyramidLevel-1; pyr > -1; pyr--)
	{
		CUDA_SAFE_CALL( cudaBindTextureToArray(I_Texture ,TArr[pyr]));
		CUDA_SAFE_CALL( cudaThreadSynchronize());

		for (int iteration=0; iteration<max_iteration; iteration++)
		{
			// 4. Compute the error
			ComputeB<<< grid_feature , thread_feature >>> 
				(featureX[pyr], featureY[pyr], 
				templ_Gx[pyr], templ_Gy[pyr], templ_W[pyr], templ_T[pyr], templ_Hessian[pyr], motionParams[pyr], 
				vectorB[pyr], (templSize[pyr]-1)/2, imgWidth[pyr], imgHeight[pyr]);

			// 5. Compute the direction of motion update from the Hessian and the error
			ComputeMotionUpdateDirection<<< grid_feature , thread_feature >>> 
				(templ_Hessian[pyr], vectorB[pyr], delta_motionParams[pyr]);
	
			ComputeInitialCost<<< grid_feature , thread_feature >>> 
				(featureX[pyr], featureY[pyr], templ_W[pyr], templ_T[pyr], motionParams[pyr], 
				 mincost[pyr], bestscale[pyr], scale[pyr], (templSize[pyr]-1)/2, imgWidth[pyr], imgHeight[pyr]);

			CUDA_SAFE_CALL( cudaThreadSynchronize());

			// 6. Bisectional line search for the scale
			for (int i=0; i < max_linesearch_iteration; i++)
			{
				ComputeScale<<< grid_feature , thread_feature >>> 
					(featureX[pyr], featureY[pyr], templ_W[pyr], templ_T[pyr], motionParams[pyr], delta_motionParams[pyr],
					 mincost[pyr], bestscale[pyr], scale[pyr],(templSize[pyr]-1)/2, imgWidth[pyr], imgHeight[pyr]);

				CUDA_SAFE_CALL( cudaThreadSynchronize());
			}

			// 7. Motion update
			UpdateMotion_BestScale<<< grid_feature , thread_feature >>> 
				(motionParams[pyr], delta_motionParams[pyr], bestscale[pyr]);

			CUDA_SAFE_CALL( cudaThreadSynchronize());
		}

		// 8. Scale up and move to the next level
		if (pyr != 0) {
			ScaleUpMotionParameters<<< grid_feature , thread_feature >>> (motionParams[pyr], motionParams[pyr-1]);
		}
	}

	CUDA_SAFE_CALL( cudaThreadSynchronize());
	cudaMemcpy(motion,   motionParams[0],  8*MAX_NUM_FEATURES*sizeof(float), cudaMemcpyDeviceToHost);
	cudaMemcpy(residual, mincost[0],		 MAX_NUM_FEATURES*sizeof(float), cudaMemcpyDeviceToHost);
}
