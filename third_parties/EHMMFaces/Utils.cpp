#include "Utils.h"
#include <memory.h>

//*****************************************************************************

void CountObs( IplROI &roi, int winWidth, int winHeight, int stepX, int stepY, 
			  int &obsX, int &obsY  )                                       
{                                                                                    
   obsX = ( roi.width - winWidth + stepX ) / stepX;  
   obsY = ( roi.height - winHeight + stepY ) / stepY;
}

//*****************************************************************************

/*
*	Used to suppress the first DCT coefficient
*/
void ExtractDCT( float* src, float* dst, int numVec, int dstLen )
{
    float *tmpSrc = src + 1;
    float *tmpDst = dst;
	int i;

    for( i = 0; i < numVec; i++ )
    {
        memcpy( tmpDst, tmpSrc, dstLen * sizeof( float ) );
        tmpSrc += dstLen + 1;
        tmpDst += dstLen;
    }
} 

//*****************************************************************************
