/*
/*	Description: Implements the training/recogniton with embedded 
/*		hidden markov model suing A. Nefian's algorithm
*/

#ifndef _EHMMObjRecognition_H_
#define _EHMMObjRecognition_H_

//Forward declarations
class EHMMObj;
class ImgObj;
class EHMMObjDatabase;
class ImgObjDatabase;

//Disable OpenCv type cast warning
#pragma warning( disable : 4312 )

#include <vector>
#include <cxcore.h>
#include <cv.h>
#include <cvaux.h>

#pragma warning( default : 4312 )

class EHMMObjRecognition
{
public:

	EHMMObjRecognition( );

	virtual ~EHMMObjRecognition( );

	void Create( int imgWidth, int imgHeight, int obsWidth, int obsHeight, int noDCTCoeffX, 
		int noDCTCoeffY, int stepX, int stepY, bool suppressIntensity = false );

	void Release( );

	void Train( ImgObj &imgObj, EHMMObj &ehmmObj );

	void Train( ImgObjDatabase &imgObjDb, EHMMObjDatabase &ehmmObjDb );

	float ComputeLikelihood( IplImage &img, EHMMObj &ehmmObj );

	size_t ComputeLikelihood( IplImage &img, EHMMObjDatabase &ehmmObjDb, 
		std::vector< float > &likelihood );

private:

	void CountObs( IplROI &roi, CvSize &winSize, CvSize &stepSize, 
			  CvSize &noObs );

	void ExtractDCT( float* src, float* dst, int num_vec, int dst_len );

	//Controls the size of the observation vectors (the DCT window size)
	CvSize _dctSize;

	//Step of the DCT through the input image
	CvSize _stepSize;

	//Number of DCT coefficients to use	
	CvSize _noDCTCoeff;

	//Controls how the image is resizes
	int _imgWidth;
	int _imgHeight;

	//Keep or not the first DCT coefficient
	bool _suppressIntensity;
};

#endif //_EHMMObjRecognition_H_
