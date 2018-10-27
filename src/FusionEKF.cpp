#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF()
{
	is_initialized_ = false;

	previous_timestamp_ = 0;

	// initializing matrices
	R_laser_ = MatrixXd(2, 2);	// this values are provided by the laser manufacturer
	R_radar_ = MatrixXd(3, 3);	// this values are provided by the laser manufacturer
	H_laser_ = MatrixXd(2, 4);	// section 11 lesson 5
	Hj_ = MatrixXd(3, 4);

	//measurement covariance matrix - laser
	R_laser_ << 0.0225, 0,
		0, 0.0225;

	//measurement covariance matrix - radar
	R_radar_ << 0.09, 0, 0,
		0, 0.0009, 0,
		0, 0, 0.09;

	/**
	  * Finish initializing the FusionEKF.
	  * Set the process and measurement noises
	*/
	// from section 11 in lesson 5
	H_laser_ << 1, 0, 0, 0,
		0, 1, 0, 0;

	ekf_.F_ = MatrixXd(4, 4);	//4x4 matrix state transition
	ekf_.P_ = MatrixXd(4, 4);	//4x4 matrix

	//set the acceleration noise components
	noise_ax = 9;
	noise_ay = 9;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{


	/*****************************************************************************
	 *  Initialization
	 ****************************************************************************/
	if (!is_initialized_) {
		/**
		  * Initialize the state ekf_.x_ with the first measurement.
		  * Create the covariance matrix.
		  * Remember: you'll need to convert radar from polar to cartesian coordinates.
		*/
		// first measurement
		cout << "EKF: " << endl;
		ekf_.x_ = VectorXd(4);
		ekf_.x_ << 1, 1, 1, 1;	//this value is important for the RMSE

		if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
		{
			// extracting ro, theta and ro_dot values
			float ro = measurement_pack.raw_measurements_(0);
			float theta = measurement_pack.raw_measurements_(1);
			float ro_dot = measurement_pack.raw_measurements_(2);

			/**
			Convert radar from polar to cartesian coordinates and initialize state.
			*/
			ekf_.x_(0) = ro * cos(theta);
			ekf_.x_(1) = ro * sin(theta);
		}
		else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
		{
			/**
			Initialize state.
			*/
			// extracting px and py values
			float px = measurement_pack.raw_measurements_(0);
			float py = measurement_pack.raw_measurements_(1);
			ekf_.x_(0) = px;
			ekf_.x_(1) = py;
		}

		// state transition matrix, one diagonal matrix, section 9, lesson 10
		ekf_.F_ << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;

		previous_timestamp_ = measurement_pack.timestamp_;

		// done initializing, no need to predict or update
		is_initialized_ = true;
		return;
	}

	/*****************************************************************************
	 *  Prediction
	 ****************************************************************************/

	 /**
		* Update the state transition matrix F according to the new elapsed time.
		 - Time is measured in seconds.
		* Update the process noise covariance matrix.
		* Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
	  */
	  // dt expressed in seconds
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
	previous_timestamp_ = measurement_pack.timestamp_;

	// integrate the dt in the state transition matrix, section 9, lesson 10
	ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt;

	// set the process covariance matrix Q, section 10, lesson 10
	float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;

	float term0_0 = (dt_4 * this->noise_ax) / 4;
	float term0_2 = (dt_3 * this->noise_ax) / 2;
	float term1_1 = (dt_4 * this->noise_ay) / 4;
	float term1_3 = (dt_3 * this->noise_ay) / 2;
	float term2_0 = term0_2;
	float term2_2 = dt_2 * this->noise_ax;
	float term3_1 = term1_3;
	float term3_3 = dt_2 * this->noise_ay;
	ekf_.Q_ = MatrixXd(4, 4);
	ekf_.Q_ << term0_0, 0, term0_2, 0
		, 0, term1_1, 0, term1_3
		, term2_0, 0, term2_2, 0
		, 0, term3_1, 0, term3_3;

	ekf_.Predict();

	/*****************************************************************************
	 *  Update
	 ****************************************************************************/

	 /**
		* Use the sensor type to perform the update step.
		* Update the state and covariance matrices.
	  */

	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
	{
		// Radar updates
		Hj_ = tools.CalculateJacobian(ekf_.x_);
		ekf_.H_ = Hj_;
		ekf_.R_ = R_radar_;	// laser uncertinity
		ekf_.UpdateEKF(measurement_pack.raw_measurements_);
	}
	else
	{
		// Laser updates
		ekf_.H_ = H_laser_;
		ekf_.R_ = R_laser_;	// laser uncertinity 
		ekf_.Update(measurement_pack.raw_measurements_);
	}

	// print the output
	cout << "x_ = " << ekf_.x_ << endl;
	cout << "P_ = " << ekf_.P_ << endl;
}
