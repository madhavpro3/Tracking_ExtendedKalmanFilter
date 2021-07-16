#include "tools.h"
#include <iostream>
#include <cmath>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */

   VectorXd rmse(4);
   rmse << 0,0,0,0;

   if(estimations.size()==0 || estimations.size()!=ground_truth.size())
     return rmse;

   for (int t=0; t < estimations.size(); ++t) {
     VectorXd diff = estimations[t] - ground_truth[t];
     VectorXd diff_2 = diff.array()*diff.array();
     rmse = rmse + diff_2;
   }

     rmse = rmse/estimations.size();
     rmse = rmse.array().sqrt();

   return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */

     MatrixXd Hj(3,4);
     Hj << 0,0,0,0,
           0,0,0,0,
           0,0,0,0;
     // recover state parameters
     float px = x_state(0);
     float py = x_state(1);
     float vx = x_state(2);
     float vy = x_state(3);

     float denom = sqrt(pow(px,2)+pow(py,2));

     // check division by zero
     if(denom <= 0.00001){
         cout << "CalculateJacobian() - Error - Division by Zero" << endl;
         return Hj;
     }

     // compute the Jacobian matrix
     Hj(0,0) = px/denom;
     Hj(0,1) = py/denom;
     Hj(1,0) = -py/pow(denom,2);
     Hj(1,1) = px/pow(denom,2);

     Hj(2,0) = py*(vx*py - vy*px)/pow(denom,3);
     Hj(2,1) = px*(vy*px - vx*py)/pow(denom,3);

     Hj(2,2) = Hj(0,0);
     Hj(2,3) = Hj(0,1);

     return Hj;
}
