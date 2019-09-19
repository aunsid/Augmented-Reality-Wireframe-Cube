#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <glob.h>   // number of files
#include <iostream>
#include <vector> 
#include <sstream> // reading data from files or file path
#include <fstream> // for files
#include <math.h>
#include "read.h"
#include "init.h"


using namespace cv;
using namespace std;
using namespace Eigen;





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