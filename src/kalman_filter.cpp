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
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */

   x_ = F_*x_;
   P_ = F_*P_*F_.transpose() + Q_;
   return;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */

    VectorXd y = VectorXd(2);
    MatrixXd S = MatrixXd(2,2);
    MatrixXd K = MatrixXd(4,2);

    y = z - H_*x_;
    S = H_*P_*H_.transpose() + R_;

    K = P_*H_.transpose()*S.inverse();
    x_ = x_ + (K*y);

    MatrixXd I = MatrixXd(4,4);
    I << 1,0,0,0,
         0,1,0,0,
         0,0,1,0,
         0,0,0,1;

    P_ = (I - K*H_)*P_;

    return;
}

void KalmanFilter::UpdateEKF(const VectorXd& z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
   // x_ = px,py,vx,vy
   // h = sqrt(px^2+py^2), atan(py/px), (px*vx + py*vy)/sqrt(px^2+py^2)

   float px = x_(0);
   float py = x_(1);
   float vx = x_(2);
   float vy = x_(3);

   float denom = sqrt(pow(px,2)+pow(py,2));
   if(denom<=0.000001)
    return;

   float phi = atan2(py,px);

   VectorXd h = VectorXd(3);
   h << denom, phi,
        (px*vx + py*vy)/denom;

   MatrixXd I = MatrixXd(4,4);
   I << 1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1;

   VectorXd y = VectorXd(3);
   MatrixXd S = MatrixXd(3,3);
   MatrixXd K = MatrixXd(4,3);

   y = z - h;

   while(y(1)>M_PI){ y(1) -= 2.*M_PI;}
   while(y(1)<-M_PI){ y(1) += 2.*M_PI;}

   S = H_*P_*H_.transpose() + R_;

   K = P_*H_.transpose()*S.inverse();

   x_ = x_ + K*y;
   P_ = (I - K*H_)*P_;

   return;
}
