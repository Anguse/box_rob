/*
 * kalman.cpp
 *
 *  Created on: 14 aug. 2019
 *      Author: harald
 */
#include <iostream>
#include <eigen3/Eigen/QR>
#include "kalman.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*Function that calculates new position and certainty, takes 3 inputs. uncertainty is uncert. matrix. old_var is co-variance matrix from last kalman. d
	pos_est is position estimate.
*/
MatrixXd kalman_filter(MatrixXd cox_var, MatrixXd odometry_var, VectorXd cox_pos, VectorXd odometry_pos) {

	VectorXd new_pos(3);
	MatrixXd updated_var(3,3), inv_cox_var(3,3), inv_odometry_var(3,3), pinv(3,3);

	pinv = (cox_var + odometry_var).completeOrthogonalDecomposition().pseudoInverse();

	/*To compute new certainty*/
	inv_cox_var = cox_var.completeOrthogonalDecomposition().pseudoInverse();
	inv_odometry_var = odometry_var.completeOrthogonalDecomposition().pseudoInverse();

	/* Update new position and certainty */
	new_pos = ((cox_var* pinv *odometry_pos) + (odometry_var * pinv *cox_pos));
	updated_var = (inv_cox_var + inv_odometry_var).completeOrthogonalDecomposition().pseudoInverse();
	updated_var.conservativeResize(4, 3);
	/*Extend matrix so that col 4 will contain the positions */
	updated_var.row(updated_var.rows()-1) = new_pos.transpose();

	return updated_var;
}



