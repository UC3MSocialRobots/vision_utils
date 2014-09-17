#include "ImgObjDatabase.h"
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

ImgObjDatabase::ImgObjDatabase( )
{
	_databaseName = "";
}

//*****************************************************************************

ImgObjDatabase::~ImgObjDatabase( )
{
	Release( );
}

//*****************************************************************************

void ImgObjDatabase::Create( const string &databaseName )
{
	assert( _databaseName == "" );

	_databaseName = databaseName;
}

//*****************************************************************************

/*
*	Load a database from disk
*/
void ImgObjDatabase::Load( const string &databaseName, const string &path )
{
	const string DATABASE_PATH = path + databaseName;
	const int MAX_LINE_WIDTH = 255;
	const int INITIAL_SIZE = 1000;

	ifstream file( DATABASE_PATH.c_str( ) );
	char line[ MAX_LINE_WIDTH ];

	assert( file.is_open( ) );

	_objs.reserve( INITIAL_SIZE );

	while ( file.getline( line, sizeof( line ) ) )
	{
		//Create a new object
		_objs.push_back( new ImgObj( ) );

		//try
		{
			//Create the corresponding object and load ehmm
			_objs[ _objs.size( ) - 1 ]->Load( line, path );
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

			cerr << "Exception occured when creating " << line << "." << endl;
		}
		*/
	}	

	//Save the name of the database
	_databaseName = databaseName;
}

//*****************************************************************************

/*
*	Save the database to disk. It will overwrite the previous data
*/
void ImgObjDatabase::Save( const string &path )
{
	const string DATABASE_PATH = path + _databaseName;

	vector< ImgObj* >::iterator usersIt;
	ofstream file( DATABASE_PATH.c_str( ) );

	assert( _databaseName != "" );
	assert( file.is_open( ) );

	for ( usersIt = _objs.begin( ); usersIt < _objs.end( ); usersIt++ )
	{
		file << (*usersIt)->GetName( ) << endl;
		(*usersIt)->Save( path );
	}
}

//*****************************************************************************

void ImgObjDatabase::Release( )
{
	vector< ImgObj* >::iterator usersIt;

	assert( _databaseName != "" );

	for ( usersIt = _objs.begin( ); usersIt < _objs.end( ); usersIt++ )
	{
		delete (*usersIt);
		*usersIt = 0;
	}

	_databaseName = "";
}

//*****************************************************************************

/*
*	Load the object	
*/
void ImgObjDatabase::LoadObj( const string &userName, const string &path )
{
	_objs.push_back( new ImgObj( ) );
	_objs[ _objs.size( ) - 1 ]->Load( userName, path );
}

//*****************************************************************************

/*
*	Add a new object to the database
*/
void ImgObjDatabase::AddObj( const string &userName )
{
	_objs.push_back( new ImgObj( ) );
	_objs[ _objs.size( ) - 1 ]->Create( userName );
}

//*****************************************************************************

/*
*	Remove an object from the database by index
*/
void ImgObjDatabase::RemoveObj( size_t index )
{
	assert( _databaseName != "" );
	assert( index < _objs.size( ) );

	_objs.erase( _objs.begin( ) + index );
}

//*****************************************************************************

/*
*	Remove the first occurance of an object from the database by name
*/
void ImgObjDatabase::RemoveObj( const string &userName )
{
	const string EXCEPTION_STR = "ImgObjDatabase::RemoveObj: " + userName + " not found.";

	vector< ImgObj* >::iterator usersIt;

	assert( _databaseName != "" );
	
	for ( usersIt = _objs.begin( ); usersIt < _objs.end( ); usersIt++ )
	{
		if ( (*usersIt)->GetName( ) == userName )
		{
			_objs.erase( usersIt );

			return;
		}
	}

	//throw exception( EXCEPTION_STR.c_str( ) );
}

//*****************************************************************************

ImgObj& ImgObjDatabase::GetObj( size_t index )
{
	assert( _databaseName != "" );
	assert( index < _objs.size( ) );

	return *_objs[ index ];
}

//*****************************************************************************

/*
*	If more than one object has the same name the first object is returned
*/
ImgObj& ImgObjDatabase::GetObj( const string &userName )
{
	const string EXCEPTION_STR = "ImgObjDatabase::GetObj: " + userName + " not found.";

	vector< ImgObj* >::iterator usersIt;

	assert( _databaseName != "" );
	
	for ( usersIt = _objs.begin( ); usersIt < _objs.end( ); usersIt++ )
	{
		if ( (*usersIt)->GetName( ) == userName )
		{
			return *(*usersIt);
		}
	}

	//throw exception( EXCEPTION_STR.c_str( ) );
}

//*****************************************************************************

size_t ImgObjDatabase::GetNoObjs( ) const
{
	assert( _databaseName != "" );

	return _objs.size( );
}

//*****************************************************************************

ostream& operator<<( ostream &os, ImgObjDatabase &obj )
{
	vector< ImgObj* >::iterator usersIt;
	int c;

	assert( obj._databaseName != "" );

	os << "Database of images '" << obj._databaseName << "' contains..." << endl;

	for ( c = 1, usersIt = obj._objs.begin( ); usersIt < obj._objs.end( ); 
		usersIt++, c++ )
	{
		os << c << ")" << " " << (*usersIt)->GetName( ) << endl;
	}

	return os;
}

//*****************************************************************************
