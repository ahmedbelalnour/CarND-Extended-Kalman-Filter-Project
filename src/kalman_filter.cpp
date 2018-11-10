#include <iostream>
#include "kalman_filter.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
    * predict the state, section 13, lesson 10
  */
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

bool KalmanFilter::ValidateBeforeUpdate(const Eigen::VectorXd &x_new)
{
	bool retVal = false;
	float x_diff = x_new(0) - x_(0);
	float y_diff = x_new(1) - x_(1);
	cout << "y_diff" << y_diff << endl;

	if (false == isInit)
	{
		retVal = true;
	}
	if (abs(y_diff) < 0.3)
	{
		retVal = true;
	}
	cout << "isValid: " << retVal << endl;
	return retVal;
}
void KalmanFilter::Update(const VectorXd &z) {
  /**
    * update the state by using Kalman Filter equations, section 13, lesson 10
  */
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;
	bool isValid;

	//new estimate
	x_new = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	
	isValid = ValidateBeforeUpdate(x_new);
	if (true == isValid)
	{
		x_ = x_new;
		P_ = (I - K * H_) * P_;
	}
	isInit = true;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) 
{
  /**
    * update the state by using Extended Kalman Filter equations, section 15, lesson 10
  */
	float x = x_(0);
	float y = x_(1);
	float vx = x_(2);
	float vy = x_(3);
	static int counter = 0;
	bool isValid = false;
	counter++;
	float rho = sqrt(x*x + y*y);
	float theta = atan2(y, x);
	float rho_dot = (x*vx + y*vy) / rho;



	VectorXd z_pred = VectorXd(3);
	z_pred << rho, theta, rho_dot;

	VectorXd y_vector = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_new = x_ + (K * y_vector);

	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);

	isValid = ValidateBeforeUpdate(x_new);
	if (true == isValid)
	{
		x_ = x_new;
		P_ = (I - K * H_) * P_;
	}
	isInit = true;

}
