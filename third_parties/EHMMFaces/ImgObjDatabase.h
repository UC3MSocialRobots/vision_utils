/*
/*	Description: Implements a simple database of imgusers 
*/

#ifndef _ImgObjDatabase_H_
#define _ImgObjDatabase_H_

#include "ImgObj.h"
#include <vector>
#include <iostream>

//Exception specification warning
#pragma warning( disable : 4290 )

class ImgObjDatabase
{
public:

	ImgObjDatabase( );

	virtual ~ImgObjDatabase( );

	void Create( const std::string &databaseName );

	void Load( const std::string &databaseName, const std::string &path );

	void Save( const std::string &path );

	void Release( );

	void LoadObj( const std::string &userName, const std::string &path );

	void AddObj( const std::string &userName );

	void RemoveObj( size_t index );

	void RemoveObj( const std::string &userName );

	ImgObj& GetObj( size_t index );

	ImgObj& GetObj( const std::string &userName ); //throw( exception );

	size_t GetNoObjs( ) const;

	friend std::ostream& operator<<( std::ostream &os, ImgObjDatabase &obj );

private:

	//Stores all the users with their 
	//associated properties
	std::vector< ImgObj* > _objs;

	//Name of the database file
	std::string _databaseName;
};

std::ostream& operator<<( std::ostream &os, ImgObjDatabase &obj );

#endif //_ImgObjDatabase_H_
