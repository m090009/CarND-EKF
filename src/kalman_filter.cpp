#include "kalman_filter.h"

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
  TODO:
    * predict the state
  */
  // cout << "Predict" << endl;
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}


void KalmanFilter::UpdateWithVector(const VectorXd &v){
  // Completing the standard Kalman Filter equations using a 
  // vector v , y is passed.
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
  // New estimate 
  x_ = x_ + (K * v);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  // 
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  // cout << "Update" << endl;
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  // Call the UpdateWithVector method to continue the 
  // remaining equations
  UpdateWithVector(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  // cout << "EKFUpdate" << endl;

  //get state parameters
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  

  // Convert to polar
  double rho = sqrt(pow(px, 2) + pow(py, 2));

  // Check zero
  if (rho < 0.0001) {
    px = 0.001;
    py = 0.001;
    rho = sqrt(pow(px, 2) + pow(py, 2));
  }

  double phi = atan2(py, px);
  double rho_dot = (px * vx + py * vy) / rho;
  
  VectorXd h = VectorXd(3);
  h << rho, phi, rho_dot;
  
  VectorXd y = z - h;
  
  // Normalizing the angle
  while(y(1) > M_PI || y(1) < -M_PI){
    if (y(1) > M_PI) {
      y(1) -= M_PI;
    } else {
      y(1) += M_PI;
    }
  }
  // Call the UpdateWithVector method to continue the 
  // remaining equations
  UpdateWithVector(y);
}
