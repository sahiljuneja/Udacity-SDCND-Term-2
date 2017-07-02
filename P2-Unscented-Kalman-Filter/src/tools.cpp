#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	// check the validity of the following inputs:
	//  the estimation vector size should not be zero
	//  the estimation vector size should equal ground truth vector size
	if (estimations.size() == 0 || ground_truth.size() == 0 || (estimations.size() != ground_truth.size())) 
	{
		std::cout << "Invalid estimation or ground truth data";
		return rmse;
	}

	//accumulate squared residuals
	for (int i = 0; i < estimations.size(); ++i) 
	{
		VectorXd residuals(4);
		residuals = estimations[i] - ground_truth[i];
		residuals = residuals.array() * residuals.array();
		rmse += residuals;
	}

	//calculate the mean
	rmse = rmse / estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}