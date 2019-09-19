#ifndef INIT_H
#define INIT_H

#include <Eigen/Core>
#include <Eigen/Dense>

void InitializeGrid(Eigen::MatrixXd &mat_x, Eigen::MatrixXd &mat_y);
Eigen::MatrixXd InitializeBoardCoods3d();
Eigen::MatrixXd InitializeCubeCoods3d();
 
#endif