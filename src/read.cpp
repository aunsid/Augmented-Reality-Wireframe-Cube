#include <glob.h>   // number of files
#include <iostream>
#include <string>
#include <vector> 
#include <sstream> // reading data from files or file path
#include <fstream> // for files
//#include <Eigen/Dense>
#include <Eigen/Core>
#include "read.h"




std::vector<std::vector<double>> GetPoses(const std::string & file_name){
	/*
	 This function gets the poses file name and then returns 
	 vector of
 vectors of poses.
	*/
	
	// initialize vectors of vectors
	std::vector<std::vector<double>> poses;

	// file object
	std::ifstream infile(file_name);
	// reads the line
	std::string entry;
	// to store value in the string

	double val;

	// read line and store in entry
	while(getline(infile,entry)){
		// get individual poses
		std::vector<double> pose;
		// convert entry to string stream type
		std::stringstream streamed_entry(entry);

		// push values to val from the stream
		while(streamed_entry >> val){
			// poses in a single line
			pose.push_back(val);
		}
		// accumulate poses
		poses.push_back(pose);
	}

	return poses;

}

void ReadDistortionParameters(const std::string &file, double &k1, double &k2){
	/*
	 Reads the radial distortion parameters from the file
	*/

	std::ifstream infile(file);
	std::string entry;
	while(std::getline(infile, entry)){
		std::stringstream streamed_entry(entry);
		streamed_entry>>k1>>k2;
	}


}


void ReadCameraMatrix(const std::string &file, Eigen::MatrixXd &K){
	/*
	Given a path writes the camera matrix to K.
	*/
	std::ifstream infile(file);
	std::string entry;
	int row_counter = 0;
	while(std::getline(infile, entry)) {
		
		std::stringstream streamed_entry(entry);
		
		streamed_entry>> K(row_counter,0)>>K(row_counter,1)>>K(row_counter,2);

		row_counter++;

	}

}


int GetNumberofImages(const std::string & path, const std::string & type){
	/* 
	Get the path of the image and type of files to search

	Arguments:
	path - path in which to search for files
		   const string ref, as the file does not need to be changed.
	type -  extension of the file to look for  
		   const string  ref, does not need to be changed 


	Returns:
	number_images -  number of images found in the folder with "type" extension
			 int 
	*/

	glob_t gl;

 	size_t number_images = 0;
 	if(glob((path+"*"+type).c_str(), GLOB_NOSORT, NULL, &gl) == 0)
	  		number_images = gl.gl_pathc;

	return (int)number_images;

}
