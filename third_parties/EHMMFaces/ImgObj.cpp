#include "ImgObj.h"
#include <fstream>
#include <iostream>
#include <assert.h>
#include <highgui.h>

using std::string;
using std::vector;
using std::ifstream;
using std::ofstream;
using std::endl;
using std::ostream;

using std::cout;

//Standard extension to the user file
static const string EXTENSION = ".imobj";

//*****************************************************************************

ImgObj::ImgObj( )
{
}

//*****************************************************************************

ImgObj::~ImgObj( )
{
}

//*****************************************************************************

void ImgObj::Create( const string &userName )
{
	assert( _name == "" );

	Obj::Create( userName );
}

//*****************************************************************************

void ImgObj::Load( const string &userName, const string &path )
{
	const int INITIAL_SIZE = 1000;
	const int MAX_LINE_WIDTH =1000;// _MAX_PATH;
	const string IMGUSER_FILE = path + userName + EXTENSION;

	char line[ MAX_LINE_WIDTH ];
	ifstream file( IMGUSER_FILE.c_str( ) );

	assert( _name == "" );
	assert( file.is_open( ) );

	Obj::Create( userName );

	_imagePaths.reserve( INITIAL_SIZE );

	while ( file.getline( line, sizeof( line ) ) )
	{
		_imagePaths.push_back( line );
	}
}

//*****************************************************************************

void ImgObj::Save( const string &path )
{
	const string IMGUSER_FILE = path + _name + EXTENSION;

	vector< string >::iterator imIt;
	ofstream file( IMGUSER_FILE.c_str( ) );

	assert( _name != "" );
	assert( file.is_open( ) );

	for ( imIt = _imagePaths.begin( ); imIt < _imagePaths.end( ); imIt++ )
	{
		file << (*imIt) << endl;
	}
}

//*****************************************************************************

void ImgObj::Release( )
{
	vector< string >::iterator imIt;

	assert( _name != "" );

	for ( imIt = _imagePaths.begin( ); imIt < _imagePaths.end( ); imIt++ )
	{
		(*imIt) = "";
	}

	_name = "";
}

//*****************************************************************************

void ImgObj::AddImage( const string &imagePath )
{
	assert( _name != "" );

	_imagePaths.push_back( imagePath );
}

//*****************************************************************************

/*
*	Load the image from the disk. Convert it to gray scale 
*	because it is always loaded as a color image.
*	The IplImage will be returned by GetImage with 
*	this dimensions no matter the dimensions 
*	of image stored on disk. If one of the dimensions
*	is 0 then only the other dimension is used. If both
*	are 0 the image will not be rescaled
*/
IplImage* ImgObj::GetGrayScaleImage( size_t index, int imgWidth, int imgHeight, 
									bool showImage )
{
	IplImage *imgDb;
	IplImage *imgHdd;

	assert( _name != "" );

	imgHdd = cvLoadImage( _imagePaths[ index ].c_str( ), 0 );
	cout<<_imagePaths[index]<<endl;
	assert( imgHdd != 0 );

	//Create the output image at the requested size
	if ( imgWidth && imgHeight )
	{
		imgDb = cvCreateImage( cvSize( imgWidth, imgHeight ), IPL_DEPTH_8U, 1 );
	}
	else if ( imgHeight )
	{
		imgDb = cvCreateImage( cvSize( imgHdd->width * imgHeight / imgHdd->height, imgHeight ), 
			IPL_DEPTH_8U, 1 );
	}
	else if ( imgWidth )
	{
		imgDb = cvCreateImage( cvSize( imgWidth, imgHdd->height * imgWidth / imgHdd->width ), 
			IPL_DEPTH_8U, 1 );
	}
	else
	{
		imgDb = cvCreateImage( cvSize( imgHdd->width, imgHdd->height ), IPL_DEPTH_8U, 1 );
	}
	assert( imgDb != 0 );

	//Resize the output to the requested size
	cvResize( imgHdd, imgDb );

	//Set a roi to the full image
	cvSetImageROI( imgDb, cvRect( 0, 0, imgDb->width, imgDb->height ) );

	if ( showImage )
	{
		cvNamedWindow( _imagePaths[ index ].c_str( ), CV_WINDOW_AUTOSIZE );
		cvShowImage( _imagePaths[ index ].c_str( ), imgDb );
		cvWaitKey( 0 );
		cvDestroyWindow( _imagePaths[ index ].c_str( ) );
	}

	//Delete all the temp images
	cvReleaseImage( &imgHdd );

	return imgDb;
}

//*****************************************************************************

size_t ImgObj::GetNoImages( )
{
	assert( _name != "" );

	return _imagePaths.size( );
}

//*****************************************************************************

ostream& operator<<( ostream &os, ImgObj &obj )
{
	vector< string >::iterator imIt;	
	int c;

	assert( obj._name != "" );

	os << "Obj '" << obj._name << "' contains..." << endl << endl;

	for ( c = 1, imIt = obj._imagePaths.begin( ); imIt < obj._imagePaths.end( ); 
		imIt++, c++ )
	{
		os << c << ")" << " " << (*imIt) << endl;
	}

	return os;
}

//*****************************************************************************
