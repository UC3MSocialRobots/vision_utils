/*
/*	Description: Implements an embedded hidden markov model
*/

#ifndef _EHMM_H_
#define _EHMM_H_

//Disable OpenCv type cast warning
#pragma warning( disable : 4312 )

#include <string>
#include <cxcore.h>
#include <cv.h>
#include <cvaux.h>

#pragma warning( default : 4312 )

class EHMM
{
public:

	EHMM( );

	virtual ~EHMM( );

	void Create( int *noStates, int *noMix, int vecSize );

	void Load( const std::string &fileName );

	void Save( const std::string &fileName );

	void Release( );

	bool GetTrained( );

	void SetTrained( bool trained );

	void GetNoStates( int *noStates ) const;

	void GetNoMix( int *noMix ) const;

	int GetVecSize( ) const;

	CvEHMM* GetCvEHMM( );

private:

	//Hidden markov model
	CvEHMM* _ehmm;

	//Size of the vector that is used as input
	int _vecSize;

	//If the ehmm is trained or not
	bool _trained;
};
#endif //_EHMM_H_
