#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
   H_laser_ << 1,0,0,0,
               0,1,0,0;

   Hj_ << 0,0,0,0,
          0,0,0,0,
          0,0,0,0;

  // Process noises
  ekf_.P_ = MatrixXd(4,4);
  ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

  //
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    // Process noise
    ekf_.Q_ = MatrixXd(4,4);
    ekf_.Q_ << 0,0,0,0,
               0,0,0,0,
               0,0,0,0,
               0,0,0,0;


    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates
      //         and initialize state.
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rho_dot = measurement_pack.raw_measurements_[2];
      ekf_.x_ << rho*cos(phi),
                  rho*sin(phi),
                  rho_dot*cos(phi),
                  rho_dot*sin(phi);

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      ekf_.x_ << measurement_pack.raw_measurements_[0],
                  measurement_pack.raw_measurements_[1],
                  0,
                  0;
    }
    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // Updating State transition matrix F
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  // Updating Process noise Q
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  float noise_ax = 9; // sigma_ax^2
  float noise_ay = 9; // sigma_ay^2

  ekf_.Q_ <<  noise_ax*dt_4/4,0,noise_ax*dt_3/2,0,
                  0,noise_ay*dt_4/4,0,noise_ay*dt_3/2,
              noise_ax*dt_3/2,0,noise_ax*dt_2,0,
                  0,noise_ay*dt_3/2,0,noise_ay*dt_2;

  ekf_.Predict();
  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates

    ekf_.R_ = MatrixXd(3,3);
    ekf_.R_ = R_radar_;

    Tools t;
    ekf_.H_ = MatrixXd(3,4);
    ekf_.H_ = t.CalculateJacobian(ekf_.x_);

    VectorXd z = VectorXd(3);
    z << measurement_pack.raw_measurements_[0],
        measurement_pack.raw_measurements_[1],
        measurement_pack.raw_measurements_[2];

    ekf_.UpdateEKF(z);

  }
  else {
    // TODO: Laser updates

    ekf_.R_ = MatrixXd(2,2);
    ekf_.R_ = R_laser_;

    ekf_.H_ = MatrixXd(2,4);
    ekf_.H_ = H_laser_;

    VectorXd z = VectorXd(2);
    z << measurement_pack.raw_measurements_[0],
        measurement_pack.raw_measurements_[1];
    ekf_.Update(z);

  }


  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;


  return;
}
