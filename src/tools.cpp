#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

const float ESPILON = 0.0001;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

  	VectorXd rmse(4);
  	rmse << 0., 0., 0., 0.;

  	if (0 == estimations.size())  //Zero-size estimation vector
  	{
		cout << "Error: The estimation vector size should not be zero.\n";
        return rmse;
  	}

    if (estimations.size() != ground_truth.size())  //Estimation and ground truth vectors have 
    												//different sizes
    {
        cout << "Error: The estimation and ground truth vectors should have the same size.\n";
        return rmse;
    }

    
    //Accumulate squared residuals
    for (unsigned int t = 0; t < estimations.size(); t++)
    {
    	VectorXd residual = estimations[t] - ground_truth[t];
    	residual = residual.array() * residual.array();
    	rmse += residual;
    }

    //Calculate the mean
    rmse /= estimations.size();

    //Take the square root
    rmse = rmse.array().sqrt();

    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) 
{
	//Declare the Jacobian matrix
	MatrixXd Hj(3,4);

	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//pre-compute some values
	float c1 = pow(x_state(0), 2) + pow(x_state(1), 2);
	float c2 = sqrt(c1);
	float c3 = c1 * c2;


	//check division by zero
	if(fabs(c1) < ESPILON)
	{
		//TODO: When div by zero, place an indicator (eg. Hj(0, 3) = -1) 
		//to allow ignoring this measurement update.

		cout << "Error in the Jacobian: Division by zero.\n";
	    
	} else
	{
	    //compute the Jacobian matrix
	    Hj(0, 0) = px / c2;
	    Hj(0, 1) = py / c2;
	    Hj(0, 2) = 0.;
	    Hj(0, 3) = 0.;
	    
	    Hj(1, 0) = -py / c1;
	    Hj(1, 1) = px / c1;
	    Hj(1, 2) = 0.;
	    Hj(1, 3) = 0.;
	    
	    Hj(2, 0) = py * (vx * py - vy * px) / c3;
	    Hj(2, 1) = px * (vy * px - vx * py) / c3;
	    Hj(2, 2) = px / c2;
	    Hj(2, 3) = py / c2;
	}

	return Hj;
}
