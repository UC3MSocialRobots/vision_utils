/*
/*	Description: Implements the training of the embedded 
/*		hidden markov model for face recognition
*/

#ifndef _Utils_H_
#define _Utils_H_

//Disable OpenCv type cast warning
#pragma warning( disable : 4312 )

#include <cxcore.h>
#include <cv.h>

#pragma warning( default : 4312 )

void CountObs( IplROI &roi, int winWidth, int winHeight, int stepX, int stepY, 
			  int &obsX, int &obsY );

void ExtractDCT( float* src, float* dst, int num_vec, int dst_len );

#endif //_Utils_H_
