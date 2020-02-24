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

  //Initializing measurement  matrix for laser
  H_laser_<<1,0,0,0,
            0,1,0,0;
  //Measurement matrix for laser 
  noise_ax=9;
  noise_ay=9;

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

    if(measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
      //Converting radial measurements to cartesian coordinates 

      VectorXd first_measure = Tools::convert_radial_to_cartesian(measurement_pack.raw_measurements_);

      //Initial values for matrix is small 
      MatrixXd P_init = MatrixXd(4,4);
      P_init << 1,0,0,0,
                   0,1,0,0,
                   0,0,1,0,
                   0,0,0,1;

      //Initial transition matrix with dt =0
      MatrixXd F_init = Tools::compute_transition_mat(0);


      //Initial Jacobian matrix is assigned to first_measure

      MatrixXd H_init = Tools::CalculateJacobian(first_measure);

      //Initial Covariance matrix is equal to the first measurement of radar

      MatrixXd R_init = R_radar_;

      //Initial process covariance matrix is calculated based on noise and dt =0
      MatrixXd Q_init = Tools::compute_covariance_mat(0,noise_ax,noise_ay);

      cout<< " CAlling ekf_init after radar init"<<endl;
      ekf_.Init(first_measure,P_init,F_init,H_init,R_init,Q_init);

      }

      else if (measurement_pack.sensor_type_==MeasurementPackage::LASER)
      {
          float px = measurement_pack.raw_measurements_[0];
          float py = measurement_pack.raw_measurements_[1];
          //For Lidar initial velocity is assumed to be 0
          float vx=0;
          float vy =0;

          VectorXd first_measure = VectorXd(4);
          first_measure << px,py,vx,vy;


          //Initial values of P_init in Lidar has high uncertainity as there is no measurement. Hence based on KF exercise
          // the values are assigned to 1000
          MatrixXd P_init = MatrixXd(4,4);
          P_init << 1,0,0,0,
                       0,1,0,0,
                       0,0,1000,0,
                       0,0,0,1000;


          //Initial transition matrix with dt =0
          MatrixXd F_init = Tools::compute_transition_mat(0);


          //Initial Jacobian matrix is assigned to first_measure
          MatrixXd H_init = Tools::CalculateJacobian(first_measure);

          //Initial Covariance matrix is equal to the first measurement of lidar

          MatrixXd R_init = R_laser_;

          //Initial process covariance matrix is calculated based on noise and dt =0
          MatrixXd Q_init = Tools::compute_covariance_mat(0,noise_ax,noise_ay);
		
          ekf_.Init(first_measure,P_init,F_init,H_init,R_init,Q_init);


      }
    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
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
  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;

  ekf_.F_= Tools::compute_transition_mat(dt);

  ekf_.Q_=Tools::compute_covariance_mat(dt,noise_ax,noise_ay);

  //Making prediction only when the time difference is small 

  if (dt>0.001)
  {
  previous_timestamp_ = measurement_pack.timestamp_;
  ekf_.Predict();
  }
  /**
   * Update
   */

  /**
   * - Using the sensor type to perform the update step.
   * - Updating the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.R_=R_radar_;
    ekf_.H_=Tools::CalculateJacobian(ekf_.x_);


    VectorXd z = measurement_pack.raw_measurements_;
    ekf_.UpdateEKF(z);

  } else if ( measurement_pack.sensor_type_==MeasurementPackage::LASER)

  {
  ekf_.R_= R_radar_;
  ekf_.H_= H_laser_;
  VectorXd z = measurement_pack.raw_measurements_;
  ekf_.Update(z);
  }

  

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
