/*
/*	Description: Implements a simple description of an object
*/

#ifndef _Obj_H_
#define _Obj_H_

#include <string>
#include <vector>

class Obj
{
public:

	Obj( );

	virtual ~Obj( );

	void Create( const std::string &userName );

	const std::string& GetName( ) const;

protected:

	//Stores the name of the object
	std::string _name;
};
#endif //_Obj_H_
