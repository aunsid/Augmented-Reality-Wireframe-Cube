#include <iostream>
#include "init.h"

using namespace Eigen;


void InitializeGrid(Eigen::MatrixXd &mat_x, Eigen::MatrixXd &mat_y){
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


Eigen::MatrixXd InitializeBoardCoods3d(){
	/*
	Takes a reference to a matrix and returns the 3d coods of the intersections
	on the checkerboard.
	*/

	Eigen::MatrixXd board(3,54);

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

Eigen::MatrixXd InitializeCubeCoods3d(){
	/*
	Takes a reference to a matrix and returns the 3d coods of the intersections
	on the checkerboard.
	*/

	 Eigen::MatrixXd cube(3,8);
	 cube << 0.0,2.0,0.0,2.0, 0.0,2.0,0.0,2.0,
	 		 0.0,0.0,2.0,2.0, 0.0,0.0,2.0,2.0,
	 		 0.0,0.0,0.0,0.0, -2.0,-2.0,-2.0,-2.0;
		     
	

	cube =0.04*cube;


	
	// std::cout << cube << std::endl;

	return cube;



}