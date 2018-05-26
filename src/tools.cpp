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
	* Calculate the RMSE here.
  */
 	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
  
}

VectorXd Tools::MapPolarToCart(const VectorXd& x_state) {
	/*
	This method maps the polar coordinates // [pos1 pos2] in SI units and rad
	*/
	float px = 0;
	float py = 0;

	float rho=x_state(0);
	float phi=x_state(1);

	px = rho * cos(phi);
    py = rho * sin(phi);

	VectorXd out = VectorXd(1, 2);

	out(0)=px;
	out(1)=py;

	return out;
}

double Tools::NormalizeAngle(double angle) {
	/*
	This method normalizes angles to be between -pi and pi
	*/
    double a = fmod(angle + M_PI, 2 * M_PI);
    return a >= 0 ? (a - M_PI) : (a + M_PI);
}