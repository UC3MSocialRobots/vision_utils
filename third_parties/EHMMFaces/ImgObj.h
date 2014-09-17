/*
/*	Description: Implements a simple description of an ImgObj 
/*		that stores all the available images for that object
*/

#ifndef _ImgObj_H_
#define _ImgObj_H_

#include "Obj.h"
#include "EHMM.h"
#include <vector>
#include <string>
#include <ostream>
#include <cxcore.h>
#include <cv.h>

class ImgObj : public Obj
{
public:

	ImgObj( );

	virtual ~ImgObj( );

	void Create( const std::string &userName );

	void Load( const std::string &userName, const std::string &path );

	void Save( const std::string &path );

	void Release( );

	void AddImage( const std::string &imagePath );

	IplImage* GetGrayScaleImage( size_t index, int imgWidth = 0, int imgHeight = 0, 
		bool showImage = false );

	size_t GetNoImages( );

	friend std::ostream& operator<<( std::ostream &os, ImgObj &obj );

private:

	//Stores the paths to the images
	std::vector< std::string > _imagePaths;
};

std::ostream& operator<<( std::ostream &os, ImgObj &obj );

#endif //_ImgObj_H_ 
