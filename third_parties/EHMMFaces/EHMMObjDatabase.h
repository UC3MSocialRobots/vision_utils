/*
/*	Description: Implements a simple database used 
/*		for the ehmm face detection algorithm
*/

#ifndef _EHMMObjDatabase_H_
#define _EHMMObjDatabase_H_

#include "EHMMObj.h"
#include <vector>
#include <iostream>

//Exception specification warning
#pragma warning( disable : 4290 )

class EHMMObjDatabase
{
private:

	enum{ MAX_STATES = 128 };

public:

	EHMMObjDatabase( );

	virtual ~EHMMObjDatabase( );

	void Create( const std::string &databaseName, int *noStates, 
		int *noMix, int vecSize );

	void Load( const std::string &databaseName, const std::string &path );

	void Save( const std::string &path );

	void Release( );

	void LoadObj( const std::string &userName, const std::string &path );

	void AddObj( const std::string &userName );

	void RemoveObj( size_t index );
	
	void RemoveObj( const std::string &userName );

	EHMMObj& GetObj( size_t index );

	EHMMObj& GetObj( const std::string &userName );// throw( exception );

	size_t GetNoObjs( ) const;

	friend std::ostream& operator<<( std::ostream &os, EHMMObjDatabase &obj );

private:

	void CheckEHMMConsistence( EHMMObj &ehmmObj );

	//Stores all the users with their 
	//associated properties
	std::vector< EHMMObj* > _objs;

	//Total number of states/substates in the ehmm
	int _numStates[ MAX_STATES ];

	//Number of gaussian mixtures per state
	int _numMix[ MAX_STATES - 1 ];

	//Size of the input vector
	int _vecSize;

	//Name of the database file
	std::string _databaseName;
};

std::ostream& operator<<( std::ostream &os, EHMMObjDatabase &obj );

#endif //_EHMMObjDatabase_H_
