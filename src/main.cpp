#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <glob.h>   // number of files
#include <iostream>
#include <vector> 
#include <sstream> // reading data from files or file path
#include <fstream> // for files
#include <math.h>


using namespace cv;
using namespace std;
using namespace Eigen;


vector<vector<double>> GetPoses(const string & file_name){
	/*
	 This function gets the poses file name and then returns 
	 vector of vectors of poses.
	*/
	
	// initialize vectors of vectors
	vector<vector<double>> poses;

	// file object
	ifstream infile(file_name);
	// reads the line
	string entry;
	// to store value in the string

	double val;

	// read line and store in entry
	while(getline(infile,entry)){
		// get individual poses
		vector<double> pose;
		// convert entry to string stream type
		stringstream streamed_entry(entry);

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

void ReadDistortionParameters(const string &file, double &k1, double &k2){
	/*
	 Reads the radial distortion parameters from the file
	*/

	ifstream infile(file);
	string entry;
	while(getline(infile, entry)){
		stringstream streamed_entry(entry);
		streamed_entry>>k1>>k2;
	}


}


void ReadCameraMatrix(const string &file, MatrixXd &K){
	/*
	Given a path writes the camera matrix to K.
	*/
	ifstream infile(file);
	string entry;
	int row_counter = 0;
	while(getline(infile, entry)) {
		
		stringstream streamed_entry(entry);
		
		streamed_entry>> K(row_counter,0)>>K(row_counter,1)>>K(row_counter,2);

		row_counter++;

	}

}


int GetNumberofImages(const string & path, const string & type){
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


void InitializeGrid(MatrixXd &mat_x, MatrixXd &mat_y){
	/*

	The two matrices mat_x and mat_y  will give the coods of the matrix
	independently in the x and y directions. With the top corner being (0,0)
	as (u,v) in Figure 3 of the exercise.

	Arguments:
	References to two matrices(Eigen)that store the location of the intersections 
	of the checkerboards.

	*/

	for  (int row = 0;  row < mat_x.rows(); row ++){
 		for(int col =0; col < mat_x.cols(); col++){
 			mat_x(row, col) = 0.04*(row+1);
 			mat_y(row, col) = 0.04*(col+1);
 		}
 	}

}


MatrixXd InitializeBoardCoods3d(){
	/*
	Takes a reference to a matrix and returns the 3d coods of the intersections
	on the checkerboard.
	*/

	MatrixXd board(3,54);

	double counter_x = 0.0;
	double counter_y = 0.0;
	double spacing = 0.04; // in mts
	for(int col = 0; col<54; col++){
			if (col % 6 ==0 &&  col!= 0 )
				counter_x = counter_x +1.0;
			board(0,col) = spacing*counter_x;
			if (counter_y == 6)
				counter_y = 0.0;
			board(1,col) = spacing*counter_y;
			counter_y = counter_y+ 1;
			board(2,col) = 0.0;

	}

	return board;
}

MatrixXd InitializeCubeCoods3d(){
	/*
	Takes a reference to a matrix and returns the 3d coods of the intersections
	on the checkerboard.
	*/

	 MatrixXd cube(3,8);
	 cube << 0.0,2.0,0.0,2.0, 0.0,2.0,0.0,2.0,
	 		 0.0,0.0,2.0,2.0, 0.0,0.0,2.0,2.0,
	 		 0.0,0.0,0.0,0.0, -2.0,-2.0,-2.0,-2.0;
		     
	

	cube =0.04*cube;


	
	std::cout << cube << std::endl;

	return cube;



}

MatrixXd GetRotationFromPose(const vector<double> &pose){
	/*Converts pose to rotation R using Rodrigues' rotation formula
	
	Arguments:
	pose - pose of the current image

	Returns:
	R - 3x3 rotation matrix.
	 */

	VectorXf w(3);
	w<<pose[0],pose[1],pose[2];
	MatrixXd t(3,1);
	t<<pose[3],pose[4],pose[5];

	double theta = w.norm();

	VectorXf k(3);

	k = w/theta;

	// identity matrix
	Matrix3d I = Matrix3d::Identity();


	Matrix3d skew_k;
	skew_k <<   0   , -k(2) ,  k(1),
			  k(2)  ,    0  , -k(0),
			 -k(1)	,  k(0) ,    0 ;

	MatrixXd R = I + (sin(theta))*skew_k +(1-cos(theta))*skew_k*skew_k;

	MatrixXd homo_matrix(R.rows(),R.cols()+t.cols());
	
	homo_matrix<<R,t;

	MatrixXd copy = homo_matrix;

	// Check if rotation, translation and homogeneuos matrix the same.
	// cout<<"t: "<<t<<endl;
	// cout<<"r:"<<R<<endl;
	// cout<<"h:"<<homo_matrix<<endl;
	

	return copy;

}

vector<cv::Point> GetIntersections(const MatrixXd & points){
	/*
	This function takes i 
	*/

	vector<cv::Point> intersections;

	for (int c = 0;c<=points.cols(); c++){
		
		cv::Point temp;
		temp.x = points.coeff(0,c)/points.coeff(2,c);
		temp.y = points.coeff(1,c)/points.coeff(2,c);

		intersections.push_back(temp);
	
	// cv::Point temp;
	// temp.x	=0;
	// temp.y = 0;
	// intersections.push_back(temp);
	}

	return intersections;

}

MatrixXd DistortPoints(const MatrixXd & points, const double k1, const double k2 ){
	/*
	Uses lens distortion to distort the points on the checkerboard

	Arguments: 
	points - Matrix of points that are to be distorted
	k1 & k2 -  lens distortion parameters

	Returns:
	distorted_points - distorted points of the same size as that of Matrix points.
	 */


	MatrixXd distorted_points = MatrixXd::Constant(points.rows()+1,points.cols(),0.0);

	// cout<< distorted_points.rows()<<"x"<<distorted_points.cols()<<endl;
	// cout<<distorted_points<<endl;


	for( int c =0; c<points.cols(); c++){
		double r2 = points.coeff(0,c)*points.coeff(0,c) + points.coeff(1,c)*points.coeff(1,c);
		double x  = points.coeff(0,c)*(1+k1*r2+k2*r2*r2);
		double y  = points.coeff(1,c)*(1+k1*r2+k2*r2*r2);

		distorted_points(0,c) = x;
		distorted_points(1,c) = y;
		distorted_points(2,c) = 1;	
		// distorted_points.col(c)<<x,y,1;
		// cout<<c<<" x  "<<x<<"y   "<<y<<endl;
	}

	// cout<<distorted_points<<endl;
	
	return distorted_points; 
}




int main()
{
	// paths to different data and data types that are required
	string images_path = "/home/aun/Documents/visionalgorithms/exercise1/augmented_reality/data/images/";
 	string image_type  = ".jpg";
 	string poses_path  = "/home/aun/Documents/visionalgorithms/exercise1/augmented_reality/data/poses.txt";
 	string matrix_path = "/home/aun/Documents/visionalgorithms/exercise1/augmented_reality/data/K.txt";
 	string distort_path = "/home/aun/Documents/visionalgorithms/exercise1/augmented_reality/data/D.txt";

 	// Check if correct number of images are returned
 	// cout<<"Number of Images: "<<GetNumberofImages(images_path, image_type)<<endl;

 	// get the number of images in the folder
	int number_images = GetNumberofImages(images_path, image_type);


	// image of the checkerboard
 	Mat image;

 	// // initialize a 3x54 matrix for the points of the intersections
 	// MatrixXd p_w_3d(3,54);

 	// // initialize coods of points in 3d world 
 	// 
 	MatrixXd p_w_3d = InitializeCubeCoods3d();

 	// check if init properly
 	// cout<<positions_world_3d<<endl;
	

 	// store the poses of images 
	vector<vector<double>> poses = GetPoses("/home/aun/Documents/visionalgorithms/exercise1/augmented_reality/data/poses.txt"); 

 	
	// get intrinsic camera params
	MatrixXd k(3,3);
	ReadCameraMatrix(matrix_path, k);


	double k1 = 0;
	double k2 = 0;
	ReadDistortionParameters(distort_path, k1, k2);

	cout<<k1<<"      "<<k2<<endl;

	// check if K matrix has been read
	// cout<<K<<endl;


 	for (int image_index  = 1; image_index <= number_images; image_index++){


 		// stringstream to images name 
 		stringstream current_image_path;
 		current_image_path<<images_path<<"img_"<<setfill('0')<<setw(4)<<image_index<<image_type;
 		

 		//check image path is correct
 		// cout<<image_index<<"   "<<current_image_path.str()<<endl;


 		// read image
 		image = imread(current_image_path.str().c_str());
 		Mat original = image.clone();

 		// check the pose vector is the same as that of the image.
 		// cout<<poses[image_index][0]<<endl;


 		// get the extrinsic camera params
 		MatrixXd r_t(3,4);
 		r_t = GetRotationFromPose(poses[image_index]);


 		// points in homogeneous  coods system.
 		MatrixXd points_camera(4,8);
 		MatrixXd ones = MatrixXd::Constant(1,8,1.0);

 		points_camera<<p_w_3d, ones;

 		MatrixXd points_image = r_t*points_camera;
 		// cout<<points_image.rows()<<"x"<<points_image.cols()<<endl;
 		

 		// normalize points_image
 		MatrixXd normalized_points_image = points_image.colwise().hnormalized();
 		// normalized_points_image<<temp, ones;

 		// cout<<"Normalized Points:  "<<endl;
 		// cout<<normalized_points_image<<endl;
 		// cout<<normalized_points_image.rows()<<"x"<<normalized_points_image.cols()<<endl;

 		
 		/*
 		Other method for normalization.

 		// 	int last_row = r.rows() - 1;
		//  r.array().rowwise() /= r.row(last_row).array();
 		// 	cout<<normalized_points_image.rows()<<"x"<<normalized_points_image.cols()<<endl;

		*/

 		// apply lens distortion
 		MatrixXd distorted_image_points = DistortPoints(normalized_points_image,k1,k2);

 		// cout<<"Distorted Points:  "<<endl;
 		// cout<<distorted_image_points.rows()<<"x"<<distorted_image_points.cols()<<endl;

 		MatrixXd pixel_coods = k*distorted_image_points;


 	

 		vector<cv::Point> intersections = GetIntersections(pixel_coods);

 	
 		// for (auto x : intersections){

 		// 	cv::circle(image,
 		// 			   x,
 		// 			   4,
   //    				   Scalar( 0, 0, 255),
   //    				   FILLED);
 		
 		// }

 		// lower square
 		cv::line(image, intersections[0], intersections[1],Scalar( 0, 0, 255),3);
 		cv::line(image, intersections[1], intersections[3],Scalar( 0, 0, 255),3);
 		cv::line(image, intersections[3], intersections[2],Scalar( 0, 0, 255),3);
 		cv::line(image, intersections[2], intersections[0],Scalar( 0, 0, 255),3);

 		// upper square
 		cv::line(image, intersections[4], intersections[5],Scalar( 0, 0, 255),3);
 		cv::line(image, intersections[5], intersections[7],Scalar( 0, 0, 255),3);
 		cv::line(image, intersections[7], intersections[6],Scalar( 0, 0, 255),3);
 		cv::line(image, intersections[6], intersections[4],Scalar( 0, 0, 255),3);

 		// inbetween
 		cv::line(image, intersections[0], intersections[4],Scalar( 0, 0, 255),3);
 		cv::line(image, intersections[1], intersections[5],Scalar( 0, 0, 255),3);
 		cv::line(image, intersections[2], intersections[6],Scalar( 0, 0, 255),3);
 		cv::line(image, intersections[3], intersections[7],Scalar( 0, 0, 255),3);






 		imshow("images", image);
 		imshow("orignal", original);
 		waitKey(0);

 		
 	}

 	



  return 0;

}