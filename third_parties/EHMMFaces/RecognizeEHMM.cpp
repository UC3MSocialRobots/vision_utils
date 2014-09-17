#include "EHMMObj.h"
#include "EHMMObjDatabase.h"
#include "Utils.h"
#include "RecognizeEHMM.h"
#include <vector>
#include <iostream>
#include <float.h>

using std::cerr;
using std::endl;
using std::vector;

//*****************************************************************************

/*
*	Recognize with ehmm using A. Nefian's algorithm (See OpenCV documentation)
*	This function returns the match likelihood
*/
float RecognizeEHMM( IplImage &img, EHMMObj &ehmmObj, int obsWidth, int obsHeight,
					int noDCTCoeffX, int noDCTCoeffY, int stepX, int stepY, 
					bool suppressIntensity )
{  
	CvSize noObs;
	CvSize dctSize = cvSize( obsWidth, obsHeight );
	CvSize stepSize = cvSize( stepX, stepY );
	CvSize obsSize = cvSize( noDCTCoeffX, noDCTCoeffY );
	CvEHMM *tmplEhmm = ehmmObj.GetEHMM( ).GetCvEHMM( );
	int obsVecLen = noDCTCoeffX * noDCTCoeffY;
    CvImgObsInfo* info;
    
    if( suppressIntensity )
    {
        obsVecLen--;
    }

	//Get how many DCT transforms we compute
	CountObs( *img.roi, dctSize.width, dctSize.height, stepSize.width, stepSize.height, 
		noObs.width, noObs.height ); 

	info = cvCreateObsInfo( noObs, obsVecLen );
	assert( info != 0 );
    
    if( suppressIntensity )
    {
        float *observations = new float[ noObs.height * noObs.width * ( obsVecLen + 1 ) ];
        
		cvImgToObs_DCT( &img, observations, dctSize, obsSize, stepSize );
        
		ExtractDCT( observations, info->obs, noObs.height * noObs.width, obsVecLen );
        
		if ( observations )
		{
			delete( observations);
		}
    }
    else
    {      
        cvImgToObs_DCT( &img, info->obs, dctSize, obsSize, stepSize );
    }

	cvEstimateObsProb( info, tmplEhmm );

	return cvEViterbi( info, tmplEhmm );
}

//*****************************************************************************

/*
*	Apply the recognition algorithm on a database of ehmms
*/
size_t RecognizeEHMM( IplImage &img, EHMMObjDatabase &ehmmObjDb, int obsWidth, 
					 int obsHeight, int noDCTCoeffX, int noDCTCoeffY, int stepX, 
					 int stepY, bool suppressIntensity )
{
	size_t i;
	EHMMObj *tmpEHMMObj;
	vector< float > likelihood;
	size_t maxLikelihoodPos = -1;
	float maxLikelihood = -FLT_MAX;

	//JUST FOR DEBUG
	cerr << "Probability scores are" << endl;

	//Train for every object in imgObjDb
	for ( i = 0; i < ehmmObjDb.GetNoObjs( ); i++ )
	{
	  //		try
		{
			tmpEHMMObj = &ehmmObjDb.GetObj( i );

			if ( tmpEHMMObj->GetEHMM( ).GetTrained( ) )
			{
				likelihood.push_back( RecognizeEHMM( img, *tmpEHMMObj, obsWidth, obsHeight, 
					noDCTCoeffX, noDCTCoeffY, stepX, stepY, suppressIntensity ) );

				if ( likelihood[ likelihood.size( ) - 1 ] > maxLikelihood )
				{
					maxLikelihood = likelihood[ likelihood.size( ) - 1 ];
					maxLikelihoodPos = i;
				}

				//JUST FOR DEBUG
				cerr << likelihood[ i ] << "  ";
			}
		}
		/*
		catch( exception &e )
		{
			cerr << e.what( ) << endl;
		}
		catch( ... )
		{
			abort( );
		}
		*/
	}

	//JUST FOR DEBUG
	cerr << endl;

	return maxLikelihoodPos;
}

//*****************************************************************************
