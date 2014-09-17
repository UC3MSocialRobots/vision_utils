#include "EHMMObjRecognition.h"
#include "EHMMObj.h"
#include "ImgObj.h"
#include "ImgObjDatabase.h"
#include "EHMMObjDatabase.h"
#include "Utils.h"
#include <iostream>
#include <cv.h>
#include <vector>
#include <math.h>
#include <float.h>

using std::cerr;
using std::endl;
using std::vector;

//*****************************************************************************

EHMMObjRecognition::EHMMObjRecognition( )
{
	Release( );
}

//*****************************************************************************

EHMMObjRecognition::~EHMMObjRecognition( )
{
}

//*****************************************************************************

void EHMMObjRecognition::Create( int imgWidth, int imgHeight, int obsWidth, 
								int obsHeight, int noDCTCoeffX, int noDCTCoeffY, 
								int stepX, int stepY, bool suppressIntensity )
{
	assert( _imgWidth == -1 );
	assert( _imgHeight == -1 );

	_dctSize = cvSize( obsWidth, obsHeight );

	_stepSize = cvSize( stepX, stepY );

	_noDCTCoeff = cvSize( noDCTCoeffX, noDCTCoeffY );

	//They are used only for training
	_imgWidth = imgWidth;	
	_imgHeight = imgHeight;

	_suppressIntensity = suppressIntensity;
}

//*****************************************************************************

void EHMMObjRecognition::Release( )
{
	assert( _imgWidth != -1 );
	assert( _imgHeight != -1 );

	_imgWidth = _imgHeight = -1;
}

//*****************************************************************************

/*
*	Train the CvEHMM using A. Nefian's algorithm (See OpenCV documentation)
*/
void EHMMObjRecognition::Train( ImgObj &imgObj, EHMMObj &ehmmObj )
{
	const int MAX_ITER = 80;    
	const double STOP_STEP_ITER = 0.01;
	
	CvImgObsInfo **obsInfoVec;
	IplImage *iplImg;
	int obsVecLen = _noDCTCoeff.width * _noDCTCoeff.height;	
	CvSize _noObs;
	CvEHMM *tmplEhmm = ehmmObj.GetEHMM( ).GetCvEHMM( );
	int numImages = (int)imgObj.GetNoImages( );
    bool trained = false;
    float oldLikelihood = 0; 
    int counter = 0;
	int i;

	assert( _imgWidth != -1 );
	assert( _imgHeight != -1 );

    if( _suppressIntensity )
    {
		//Suppress first DCT coefficient
		obsVecLen--;
    }

	//Create the obsinfo array
    obsInfoVec = new CvImgObsInfo*[ numImages ];
	assert( obsInfoVec != 0 );

	for( i = 0; i < numImages; i++ )
    {	
		iplImg = imgObj.GetGrayScaleImage( i, _imgWidth, _imgHeight );   
 
		//Get how many DCT transforms we compute
		CountObs( *iplImg->roi, _dctSize, _stepSize, _noObs ); 
            
		//Create the observation for each of the transforms
        obsInfoVec[ i ] = cvCreateObsInfo( _noObs, obsVecLen );

        if( _suppressIntensity )
        {
            float *observations = new float[ _noObs.height * _noObs.width * ( obsVecLen + 1 ) ];
            
			cvImgToObs_DCT( iplImg, observations, _dctSize, _noDCTCoeff, _stepSize );
            
			ExtractDCT( observations, obsInfoVec[ i ]->obs, _noObs.height * _noObs.width, obsVecLen );
            
			if ( observations )
			{
				delete( observations);
			}
        }
        else
        {
            cvImgToObs_DCT( iplImg, obsInfoVec[ i ]->obs, _dctSize, _noDCTCoeff, _stepSize );
        }
              
		cvUniformImgSegm( obsInfoVec[ i ], tmplEhmm );

		cvReleaseImage( &iplImg );
    }                                              

    cvInitMixSegm( obsInfoVec, numImages, tmplEhmm );

	//Start the iterative training procedure
	while( ( !trained ) && ( counter < MAX_ITER ) )
    { 
		int j;    
		float likelihood = 0;     

		counter++;
        
        cvEstimateHMMStateParams( obsInfoVec, numImages, tmplEhmm );

        cvEstimateTransProb( obsInfoVec, numImages, tmplEhmm); 
                 
        for( j = 0; j < numImages; j++ )
        {           
            cvEstimateObsProb( obsInfoVec[ j ], tmplEhmm );

            likelihood += cvEViterbi( obsInfoVec[ j ], tmplEhmm );
        }

        likelihood /= numImages * obsInfoVec[ 0 ]->obs_size;

        cvMixSegmL2( &obsInfoVec[ 0 ], numImages, tmplEhmm );

        trained = ( fabs( likelihood - oldLikelihood ) < STOP_STEP_ITER ); 
	
        oldLikelihood = likelihood;                   
   }

	//Clear the observations
	for( i = 0; i < numImages; i++ )
	{
		cvReleaseObsInfo( &(obsInfoVec[ i ]) );
	}
	delete []obsInfoVec;
 
	ehmmObj.GetEHMM( ).SetTrained( trained );
}

//*****************************************************************************

/*
*	Apply the training algorithm on two databases of images and ehmms.
*	Train only for the users that appear in the ImgObjDatabase
*/
void EHMMObjRecognition::Train( ImgObjDatabase &imgObjDb, EHMMObjDatabase &ehmmObjDb )
{
	size_t i;
	EHMMObj *tmpEHMMObj;
	ImgObj *tmpImgObj;

	//Train for every object in imgObjDb
	for ( i = 0; i < imgObjDb.GetNoObjs( ); i++ )
	{
	  //		try
		{
			tmpImgObj = &imgObjDb.GetObj( i );

			tmpEHMMObj = &ehmmObjDb.GetObj( tmpImgObj->GetName( ) );

			Train( *tmpImgObj, *tmpEHMMObj );
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

	//Check that all users are trained
	for ( i = 0; i < ehmmObjDb.GetNoObjs( ); i++ )
	{
		if ( !ehmmObjDb.GetObj( i ).GetEHMM( ).GetTrained( ) )
		{
			cerr << "Warning: Obj " << ehmmObjDb.GetObj( i ).GetName( ) 
				<< " is not trained." << endl;
		}
	}
}

//*****************************************************************************

/*
*	Recognize with ehmm using A. Nefian's algorithm (See OpenCV documentation)
*	This function returns the match likelihood
*/
float EHMMObjRecognition::ComputeLikelihood( IplImage &img, EHMMObj &ehmmObj )
{  
	CvSize _noObs;

	CvEHMM *tmplEhmm = ehmmObj.GetEHMM( ).GetCvEHMM( );
	int obsVecLen = _noDCTCoeff.width * _noDCTCoeff.height;
    CvImgObsInfo* info;
    
    if( _suppressIntensity )
    {
        obsVecLen--;
    }

	//Get how many DCT transforms we compute
	CountObs( *img.roi, _dctSize, _stepSize, _noObs ); 

	info = cvCreateObsInfo( _noObs, obsVecLen );
	assert( info != 0 );
    
    if( _suppressIntensity )
    {
        float *observations = new float[ _noObs.height * _noObs.width * ( obsVecLen + 1 ) ];
        
		cvImgToObs_DCT( &img, observations, _dctSize, _noDCTCoeff, _stepSize );
        
		ExtractDCT( observations, info->obs, _noObs.height * _noObs.width, obsVecLen );
        
		if ( observations )
		{
			delete( observations);
		}
    }
    else
    {      
        cvImgToObs_DCT( &img, info->obs, _dctSize, _noDCTCoeff, _stepSize );
    }

	cvEstimateObsProb( info, tmplEhmm );

	return cvEViterbi( info, tmplEhmm );
}

//*****************************************************************************

/*
*	Apply the recognition algorithm on a database of ehmms. Returns all the 
*	likelihoods for all the ehmms in teh database and the index of the best match
*/
size_t EHMMObjRecognition::ComputeLikelihood( IplImage &img, EHMMObjDatabase &ehmmObjDb,
											 vector< float > &likelihood )
{
	size_t i;
	EHMMObj *tmpEHMMObj;
	size_t maxLikelihoodPos = -1;
	float maxLikelihood = -FLT_MAX;

	//Train for every object in imgObjDb
	for ( i = 0; i < ehmmObjDb.GetNoObjs( ); i++ )
	{
	  //		try
		{
			tmpEHMMObj = &ehmmObjDb.GetObj( i );

			if ( tmpEHMMObj->GetEHMM( ).GetTrained( ) )
			{
				likelihood.push_back( ComputeLikelihood( img, *tmpEHMMObj ) );

				if ( likelihood[ likelihood.size( ) - 1 ] > maxLikelihood )
				{
					maxLikelihood = likelihood[ likelihood.size( ) - 1 ];
					maxLikelihoodPos = i;
				}
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

	return maxLikelihoodPos;
}

//*****************************************************************************

void EHMMObjRecognition::CountObs( IplROI &roi, CvSize &winSize, CvSize &_stepSize, 
							  CvSize &_noObs  )                                       
{                                                                                    
	_noObs.width = ( roi.width - winSize.width + _stepSize.width ) / _stepSize.width;  
	_noObs.height = ( roi.height - winSize.height + _stepSize.height ) / _stepSize.height;
}

//*****************************************************************************

/*
*	Used to suppress the first DCT coefficient
*/
void EHMMObjRecognition::ExtractDCT( float* src, float* dst, int numVec, int dstLen )
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


