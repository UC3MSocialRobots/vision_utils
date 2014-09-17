#include "EHMMObjDatabase.h"
#include <fstream>
#include <iostream>
#include <assert.h>

using std::vector;
using std::ifstream;
using std::ofstream;
using std::iostream;
using std::ostream;
using std::string;
using std::cout;
using std::endl;
using std::cerr;

//*****************************************************************************

EHMMObjDatabase::EHMMObjDatabase( )
{
	_databaseName = "";
	memset( _numStates, 0, sizeof( _numStates ) );
	memset( _numMix, 0, sizeof( _numMix ) );
	_vecSize = 0;
}

//*****************************************************************************

EHMMObjDatabase::~EHMMObjDatabase( )
{
	Release( );
}

//*****************************************************************************

/*
*	Create an empty data base
*/
void EHMMObjDatabase::Create( const string &databaseName, int *noStates, 
							  int *noMix, int vecSize )
{
	int i;
	int totalNoInternalStates;

	assert( noStates != 0 );
	assert( noMix != 0 );
	assert( vecSize != 0 );
	assert( databaseName != "" );

	//Save the name of the database
	_databaseName = databaseName;

	//Save the ehmm creation parameters

	memcpy( _numStates, noStates, sizeof( int ) * ( noStates[ 0 ] + 1 ) );

	//Count the number of internal states
	for ( totalNoInternalStates = 0, i = 1; i < noStates[ 0 ] + 1; i++ )
	{
		totalNoInternalStates += noStates[ i ];
	}

	memcpy( _numMix, noMix, sizeof( int ) * totalNoInternalStates );

	_vecSize = vecSize;
}

//*****************************************************************************

/*
*	Load a database from disk
*/
void EHMMObjDatabase::Load( const string &databaseName, 
						   const string &path )
{
	const string DATABASE_PATH = path + databaseName;
	const int MAX_LINE_WIDTH = 255;
	const int INITIAL_SIZE = 1000;

	ifstream file( DATABASE_PATH.c_str( ) );
	vector< EHMMObj* >::iterator objsIt;
	char line[ MAX_LINE_WIDTH ];

	assert( file.is_open( ) );

	//Save the name of the database
	_databaseName = databaseName;

	_objs.reserve( INITIAL_SIZE );

	while ( file.getline( line, sizeof( line ) ) )
	{
		_objs.push_back( new EHMMObj( ) );

		try
		{		
			_objs[ _objs.size( ) - 1 ]->Load( line, path );
		}
		catch( ... )
		{
			if ( _objs[ _objs.size( ) - 1 ] )
			{
				delete _objs[ _objs.size( ) - 1 ];
				_objs[ _objs.size( ) - 1 ] = 0;
			}

			_objs.pop_back( );

			cerr << "Exception occured when creating " << line << "." << endl;
		}
	}	

	//Save the ehmm creation parameters	
	if ( _objs.size( ) == 0 )
	{
		cerr << "Database is emtpy. Please use 'Create' before "
			"adding new users" << endl;
	}
	else
	{
		_objs[ 0 ]->GetEHMM( ).GetNoStates( _numStates );
		_objs[ 0 ]->GetEHMM( ).GetNoMix( _numMix );
		_vecSize = _objs[ 0 ]->GetEHMM( ).GetVecSize( );
	}

	//Check to see if the db is consistent
	for ( objsIt = _objs.begin( ); objsIt < _objs.end( ); objsIt++ )
	{
		CheckEHMMConsistence( *(*objsIt) );
	}
}

//*****************************************************************************

/*
*	Save the database to disk. It will overwrite the previous data
*/
void EHMMObjDatabase::Save( const string &path )
{
	const string DATABASE_PATH = path + _databaseName;
	
	vector< EHMMObj* >::iterator objsIt;
	ofstream file( DATABASE_PATH.c_str( ) );

	assert( _databaseName != "" );
	assert( file.is_open( ) );

	for ( objsIt = _objs.begin( ); objsIt < _objs.end( ); objsIt++ )
	{
		file << (*objsIt)->GetName( ) << endl;
		(*objsIt)->Save( path );
	}
}

//*****************************************************************************

void EHMMObjDatabase::Release( )
{
	vector< EHMMObj* >::iterator objsIt;

	for ( objsIt = _objs.begin( ); objsIt < _objs.end( ); objsIt++ )
	{
		if ( *objsIt )
		{
			delete (*objsIt);
			*objsIt = 0;
		}
	}

	_databaseName = "";
}

//*****************************************************************************

/*
*	Load the object from an already trained ehmm	
*/
void EHMMObjDatabase::LoadObj( const string &userName, 
							  const string &path )
{
	_objs.push_back( new EHMMObj( ) );

	//	try
	{		
		_objs[ _objs.size( ) - 1 ]->Load( userName, path );

		//Check to see if the object is a correct one
		CheckEHMMConsistence(  *_objs[ _objs.size( ) - 1 ] );
	}
	/*
	catch( ... )
	{
		if ( _objs[ _objs.size( ) - 1 ] )
		{
			delete _objs[ _objs.size( ) - 1 ];
			_objs[ _objs.size( ) - 1 ] = 0;
		}

		_objs.pop_back( );

		cerr << "Exception occured when creating " << userName << "." << endl;
	}
	*/
}

//*****************************************************************************

/*
*	Add a new with not yet trained ehmm object
*/
void EHMMObjDatabase::AddObj( const string &userName )
{
	assert( _vecSize != 0 );

	_objs.push_back( new EHMMObj( ) );
	_objs[ _objs.size( ) - 1 ]->Create( userName, _numStates, _numMix, _vecSize );
}

//*****************************************************************************

/*
*	Remove an object from the database by index
*/
void EHMMObjDatabase::RemoveObj( size_t index )
{
	assert( _databaseName != "" );
	assert( index < _objs.size( ) );

	_objs.erase( _objs.begin( ) + index );
}

//*****************************************************************************

/*
*	Remove the first occurance of an object from the database by name
*/
void EHMMObjDatabase::RemoveObj( const string &userName )
{
	const string EXCEPTION_STR = "EHMMObjDatabase::RemoveObj: " + userName + " not found.";

	vector< EHMMObj* >::iterator objsIt;

	assert( _databaseName != "" );
	
	for ( objsIt = _objs.begin( ); objsIt < _objs.end( ); objsIt++ )
	{
		if ( (*objsIt)->GetName( ) == userName )
		{
			_objs.erase( objsIt );

			return;
		}
	}

	//throw exception( EXCEPTION_STR.c_str( ) );
}

//*****************************************************************************

EHMMObj& EHMMObjDatabase::GetObj( size_t index )
{
	assert( _databaseName != "" );
	assert( index < _objs.size( ) );

	return *_objs[ index ];
}

//*****************************************************************************

/*
*	If more than one object has the same name the first object is returned
*/
EHMMObj& EHMMObjDatabase::GetObj( const string &userName )
{
	const string EXCEPTION_STR = "EHMMObjDatabase::GetObj: " + userName + " not found.";

	vector< EHMMObj* >::iterator objsIt;

	assert( _databaseName != "" );
	
	for ( objsIt = _objs.begin( ); objsIt < _objs.end( ); objsIt++ )
	{
		if ( (*objsIt)->GetName( ) == userName )
		{
			return *(*objsIt);
		}
	}

	//	throw exception( EXCEPTION_STR.c_str( ) );
}

//*****************************************************************************

size_t EHMMObjDatabase::GetNoObjs( ) const
{
	assert( _databaseName != "" );

	return _objs.size( );
}

//*****************************************************************************

/*
*	Check that the new loaded object has the same parameters 
*	for his ehmm as the database
*/
void EHMMObjDatabase::CheckEHMMConsistence( EHMMObj &ehmmObj )
{
	const string ERROR = "Database '" + _databaseName + "' is not consistent. Terminating application.";
	int noStates[ MAX_STATES ];
	int noMix[ MAX_STATES - 1 ];
	int totalNoInternalStates;
	int i;

	assert( _databaseName != "" );

	ehmmObj.GetEHMM( ).GetNoStates( noStates );
	ehmmObj.GetEHMM( ).GetNoMix( noMix );	

	//Count the number of internal states
	for ( totalNoInternalStates = 0, i = 1; i < noStates[ 0 ] + 1; i++ )
	{
		totalNoInternalStates += noStates[ i ];
	}

	//Check vector size, states and mixtures
	if ( ehmmObj.GetEHMM( ).GetVecSize( ) != _vecSize
		|| memcmp( noStates, _numStates, _numStates[ 0 ] + 1 )
		|| memcmp( noMix, _numMix, totalNoInternalStates ) )
	{
		cerr << ERROR;
		abort( );
	}
}

//*****************************************************************************

ostream& operator<<( ostream &os, EHMMObjDatabase &obj )
{
	vector< EHMMObj* >::iterator objsIt;
	int c;

	assert( obj._databaseName != "" );

	os << "Database of ehmms '" << obj._databaseName << "' contains..." << endl;

	for ( c = 1, objsIt = obj._objs.begin( ); objsIt < obj._objs.end( ); 
		objsIt++, c++ )
	{
		os << c << ")" << " " << (*objsIt)->GetName( ) << endl;
	}

	return os;
}

//*****************************************************************************
