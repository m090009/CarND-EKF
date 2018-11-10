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
  // Init Variables 
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  
  // Check Estimation size
  // Estimation Vector check
  if (estimations.size() == 0) {
    cout << "CalculateRMSE: size Zero" << endl; 
    return rmse;
  }
  if (estimations.size() != ground_truth.size()) {
    cout << "CalculateRMSE: size mismatch" << endl; 
    return rmse;
  }
//   if (estimations.size() > 0 && estimations.size() == ground_truth.size()) {
  // Sum all squared residuals
  for(unsigned int i=0; i < estimations.size(); ++i){
    // ... your code here
    VectorXd residual = estimations[i] - ground_truth[i];
    // Squared Residual
    VectorXd residual_squared = pow(residual.array(), 2);
    // Adding it to the total rmse
    rmse += residual_squared;
  }   

  //calculate the mean 1/n
  rmse = rmse / estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();                                                             
  //return the result
  return rmse;
  
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  // Init variables
  MatrixXd Hj(3,4);
  
  if (x_state.size() != 4) {
    cout << "CalculateJacobian: size mismatch" << endl; 
    return Hj;
  }
  
  //Get x_state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  // Helpers
  float c1 = pow(px, 2) + pow(py, 2);
  float c2 = sqrt(c1);
  float c3 = (c1 * c2);
  //check division by zero
  if (fabs(c1) < 0.0001){
    cout << "CalculateJacobian: Zero" << endl; 
    return Hj;
  }
  //compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
  -(py/c1), (px/c1), 0, 0,
  (py * (vx * py - vy * px))/c3, (px * (vy * px - vx * py))/c3, px/c2, py/c2; 
      
  return Hj;
}
