#include "kalman_filter.h"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  cout<< " Init started";
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;

}

void KalmanFilter::Predict() {
  cout<< "Predict started ";
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = (F_ * P_ * Ft) + Q_;
  cout<<"Predict completed";
  
}

void KalmanFilter::Update(const VectorXd &z) {
   
   VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  cout<<"Update started\n"<<endl;
  MatrixXd S = H_ * P_ * Ht + R_;
  cout<<"Update after Matrix S assignment"<<endl;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  cout<<"Update after Matrix K assignment"<<endl;
  

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  cout<< "Update  completed";
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  cout<<"Update EKF started";
  MatrixXd Ht = H_.transpose();
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  
  // Calculations  
  VectorXd y = z - Tools::convert_cartesian_to_radial(x_); //try atan() function
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();

  // New Estimates
  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_;
  cout<< "Update EKF completed";
}
