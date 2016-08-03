/**
* CMU MSUAV Project (Micro and Small Unmanned Aerial Vehicle)
*
* \file  cuda_select.cxx
* \brief implementation of OpenCV cvGoodFeaturesToTrack() in CUDA
* \date  15-Sept-2008
*
* \author Jun-Sik Kim (kimjs@cs.cmu.edu)
*
*/

#include <cuda_runtime.h>
#include <cv.h>
#include <ipp.h>

extern cudaArray *Image1[5], *Image2[5];

extern "C" void Caller_eig_show_mag(float* target, float ratio, unsigned int width, unsigned int height);
extern "C" void Caller_corner(cudaArray *Image, size_t pitch, unsigned int width, unsigned int height);

#define CV_IMPLEMENT_QSORT_EX( func_name, T, LT, user_data_type )                   \
void func_name( T *array, size_t total, user_data_type aux )                        \
{                                                                                   \
    int isort_thresh = 7;                                                           \
    T t;                                                                            \
    int sp = 0;                                                                     \
                                                                                    \
    struct                                                                          \
    {                                                                               \
        T *lb;                                                                      \
        T *ub;                                                                      \
    }                                                                               \
    stack[48];                                                                      \
                                                                                    \
    aux = aux;                                                                      \
                                                                                    \
    if( total <= 1 )                                                                \
        return;                                                                     \
                                                                                    \
    stack[0].lb = array;                                                            \
    stack[0].ub = array + (total - 1);                                              \
                                                                                    \
    while( sp >= 0 )                                                                \
    {                                                                               \
        T* left = stack[sp].lb;                                                     \
        T* right = stack[sp--].ub;                                                  \
                                                                                    \
        for(;;)                                                                     \
        {                                                                           \
            int i, n = (int)(right - left) + 1, m;                                  \
            T* ptr;                                                                 \
            T* ptr2;                                                                \
                                                                                    \
            if( n <= isort_thresh )                                                 \
            {                                                                       \
            insert_sort:                                                            \
                for( ptr = left + 1; ptr <= right; ptr++ )                          \
                {                                                                   \
                    for( ptr2 = ptr; ptr2 > left && LT(ptr2[0],ptr2[-1]); ptr2--)   \
                        CV_SWAP( ptr2[0], ptr2[-1], t );                            \
                }                                                                   \
                break;                                                              \
            }                                                                       \
            else                                                                    \
            {                                                                       \
                T* left0;                                                           \
                T* left1;                                                           \
                T* right0;                                                          \
                T* right1;                                                          \
                T* pivot;                                                           \
                T* a;                                                               \
                T* b;                                                               \
                T* c;                                                               \
                int swap_cnt = 0;                                                   \
                                                                                    \
                left0 = left;                                                       \
                right0 = right;                                                     \
                pivot = left + (n/2);                                               \
                                                                                    \
                if( n > 40 )                                                        \
                {                                                                   \
                    int d = n / 8;                                                  \
                    a = left, b = left + d, c = left + 2*d;                         \
                    left = LT(*a, *b) ? (LT(*b, *c) ? b : (LT(*a, *c) ? c : a))     \
                                      : (LT(*c, *b) ? b : (LT(*a, *c) ? a : c));    \
                                                                                    \
                    a = pivot - d, b = pivot, c = pivot + d;                        \
                    pivot = LT(*a, *b) ? (LT(*b, *c) ? b : (LT(*a, *c) ? c : a))    \
                                      : (LT(*c, *b) ? b : (LT(*a, *c) ? a : c));    \
                                                                                    \
                    a = right - 2*d, b = right - d, c = right;                      \
                    right = LT(*a, *b) ? (LT(*b, *c) ? b : (LT(*a, *c) ? c : a))    \
                                      : (LT(*c, *b) ? b : (LT(*a, *c) ? a : c));    \
                }                                                                   \
                                                                                    \
                a = left, b = pivot, c = right;                                     \
                pivot = LT(*a, *b) ? (LT(*b, *c) ? b : (LT(*a, *c) ? c : a))        \
                                   : (LT(*c, *b) ? b : (LT(*a, *c) ? a : c));       \
                if( pivot != left0 )                                                \
                {                                                                   \
                    CV_SWAP( *pivot, *left0, t );                                   \
                    pivot = left0;                                                  \
                }                                                                   \
                left = left1 = left0 + 1;                                           \
                right = right1 = right0;                                            \
                                                                                    \
                for(;;)                                                             \
                {                                                                   \
                    while( left <= right && !LT(*pivot, *left) )                    \
                    {                                                               \
                        if( !LT(*left, *pivot) )                                    \
                        {                                                           \
                            if( left > left1 )                                      \
                                CV_SWAP( *left1, *left, t );                        \
                            swap_cnt = 1;                                           \
                            left1++;                                                \
                        }                                                           \
                        left++;                                                     \
                    }                                                               \
                                                                                    \
                    while( left <= right && !LT(*right, *pivot) )                   \
                    {                                                               \
                        if( !LT(*pivot, *right) )                                   \
                        {                                                           \
                            if( right < right1 )                                    \
                                CV_SWAP( *right1, *right, t );                      \
                            swap_cnt = 1;                                           \
                            right1--;                                               \
                        }                                                           \
                        right--;                                                    \
                    }                                                               \
                                                                                    \
                    if( left > right )                                              \
                        break;                                                      \
                    CV_SWAP( *left, *right, t );                                    \
                    swap_cnt = 1;                                                   \
                    left++;                                                         \
                    right--;                                                        \
                }                                                                   \
                                                                                    \
                if( swap_cnt == 0 )                                                 \
                {                                                                   \
                    left = left0, right = right0;                                   \
                    goto insert_sort;                                               \
                }                                                                   \
                                                                                    \
                n = MIN( (int)(left1 - left0), (int)(left - left1) );               \
                for( i = 0; i < n; i++ )                                            \
                    CV_SWAP( left0[i], left[i-n], t );                              \
                                                                                    \
                n = MIN( (int)(right0 - right1), (int)(right1 - right) );           \
                for( i = 0; i < n; i++ )                                            \
                    CV_SWAP( left[i], right0[i-n+1], t );                           \
                n = (int)(left - left1);                                            \
                m = (int)(right1 - right);                                          \
                if( n > 1 )                                                         \
                {                                                                   \
                    if( m > 1 )                                                     \
                    {                                                               \
                        if( n > m )                                                 \
                        {                                                           \
                            stack[++sp].lb = left0;                                 \
                            stack[sp].ub = left0 + n - 1;                           \
                            left = right0 - m + 1, right = right0;                  \
                        }                                                           \
                        else                                                        \
                        {                                                           \
                            stack[++sp].lb = right0 - m + 1;                        \
                            stack[sp].ub = right0;                                  \
                            left = left0, right = left0 + n - 1;                    \
                        }                                                           \
                    }                                                               \
                    else                                                            \
                        left = left0, right = left0 + n - 1;                        \
                }                                                                   \
                else if( m > 1 )                                                    \
                    left = right0 - m + 1, right = right0;                          \
                else                                                                \
                    break;                                                          \
            }                                                                       \
        }                                                                           \
    }                                                                               \
}
#define CV_IMPLEMENT_QSORT( func_name, T, cmp )  \
    CV_IMPLEMENT_QSORT_EX( func_name, T, cmp, int )
#define  cmp_features( f1, f2 )  (*(f1) > *(f2))

static CV_IMPLEMENT_QSORT( icvSortFeatures, int *, cmp_features )

//
// cvGoodFeaturesToTrack_GPU()
//
// faster implementation of cv's good features to track function
// GPU assists the harris measure computation
// currently, the method to compute is only "harris".
// NOTE: NO mem allocs in the function to save the mem-alloc/disalloc time
//		  by using some global variables.
//	      See the declerations about CUDA support
//
void cvGoodFeaturesToTrack_GPU(IplImage* gimgf, void* eigImage, void* tempImage,
			CvPoint2D32f* corners, int *corner_count, double quality_level, double min_distance,
			const void* maskImage, int block_size, int use_harris, double harris_k)
{
	float min_dist = (float)(cvRound( min_distance * min_distance ));
	int count = 0, mask_step = 0;
	int *eig_data = 0, *tmp_data = 0, **ptr_data = 0;
	uchar *mask_data = 0;

	CvPoint *ptr = (CvPoint *) corners;
	CvMat stub, *img = (CvMat*)gimgf;
	CvMat eig_stub, *eig = (CvMat*)eigImage;
	CvMat tmp_stub, *tmp = (CvMat*)tempImage;
	CvMat mask_stub, *mask = (CvMat*)maskImage;
	int coi1 = 0, coi2 = 0, coi3 = 0;
	eig = cvGetMat(eig, &eig_stub, &coi2);	
	tmp = cvGetMat(tmp, &tmp_stub, &coi3);

	CvSize size = cvSize(gimgf->width, gimgf->height);
	ptr_data = (int**)(tmp->data.ptr);
	eig_data = (int*)(eig->data.ptr);
	tmp_data = (int*)(tmp->data.ptr);
	int eig_step, tmp_step;
	int max_count = 200;

	if (corner_count) {
		max_count = *corner_count;
		*corner_count = 0;
	}

	img = cvGetMat(img, &stub, &coi1);
	if (eig) eig = cvGetMat( eig, &eig_stub, &coi2 );
	else return;	

	if (tmp) tmp = cvGetMat( tmp, &tmp_stub, &coi3 );
	else return;

	if (mask) {
		mask = cvGetMat(mask, &mask_stub);
		if (!CV_IS_MASK_ARR(mask)) return;
	}

	if (coi1 != 0 || coi2 != 0 || coi3 != 0) return;
	CvSize imgSize = cvSize(gimgf->width, gimgf->height);

	//
	// Run CUDA kernels
	//
	Caller_corner(Image1[0], gimgf->width, gimgf->width, gimgf->height);
	Caller_eig_show_mag((float*)(eig->data.ptr), (float)quality_level, gimgf->width, gimgf->height);

	// hereafter, the CPU handles the process
	// start of copying from cv function
	cvDilate(eig, tmp);

	min_dist = (float)cvRound( min_distance * min_distance );

	ptr_data = (int**)(tmp->data.ptr);
	eig_data = (int*)(eig->data.ptr);
	tmp_data = (int*)(tmp->data.ptr);
	if (mask) {
		mask_data = (uchar*)(mask->data.ptr);
		mask_step = mask->step;
	}

	eig_step = eig->step / sizeof(eig_data[0]);
	tmp_step = tmp->step / sizeof(tmp_data[0]);
	int x, y, i, k = 0;

	// collect list of pointers to features - put them into temporary image
	for (y = 1, k = 0; y < size.height - 1; y++) {
		eig_data += eig_step;
		tmp_data += tmp_step;
		mask_data += mask_step;
		for (x = 1; x < size.width - 1; x++) {
			int val = eig_data[x];
			if (val != 0 && val == tmp_data[x] && (!mask || mask_data[x]))
				ptr_data[k++] = eig_data + x;
		}
	}

	icvSortFeatures( ptr_data, k, 0 );

	// select the strongest features 
	for (i = 0; i < k; i++)	{
		int j = count, ofs = (int)((uchar*)(ptr_data[i]) - eig->data.ptr);
		y = ofs / eig->step;
		x = (ofs - y * eig->step)/sizeof(float);

		if (min_dist != 0) {
			for (j = 0; j < count; j++) {
				int dx = x - ptr[j].x;
				int dy = y - ptr[j].y;
				int dist = dx * dx + dy * dy;
				if(dist < min_dist) break;
			}
		}

		if (j == count) {
			ptr[count].x = x;
			ptr[count].y = y;
			if(++count >= max_count) break;
		}
	}

	// convert points to floating-point format
	for (i = 0; i < count; i++) {
		assert((unsigned)ptr[i].x < (unsigned)size.width && (unsigned)ptr[i].y < (unsigned)size.height);
		corners[i].x = (float)ptr[i].x;
		corners[i].y = (float)ptr[i].y;
	}

	*corner_count = count;
}