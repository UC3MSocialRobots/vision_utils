/*
/*	Description: Implements a simple description 
/*		of an EHMMObj that is detected using EHMM
*/

#ifndef _EHMMObj_H_
#define _EHMMObj_H_

#include "Obj.h"
#include "EHMM.h"

class EHMMObj : public Obj
{
public:

	EHMMObj( );

	virtual ~EHMMObj( );

	void Create( const std::string &userName, int *noStates, int *noMix, int vecSize );

	void Load( const std::string &userName, const std::string &path );

	void Save( const std::string &path );

	EHMM& GetEHMM( );

private:

	//Hidden markov model associated to the object
	EHMM _ehmm;
};
#endif //_EHMMObj_H_
