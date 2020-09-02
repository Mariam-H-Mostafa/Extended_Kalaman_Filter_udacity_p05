#include "kalman_filter.h"
#include <iostream>
#include "tools.h"



using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
Tools tools;

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
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  z_pred = H_ * x_;
  y = z - z_pred;
  Ht = H_.transpose();
  S = H_ * P_ * Ht + R_;
  Si = S.inverse();
  K =  P_ * Ht * Si;

    //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

    hx = VectorXd(3);
    
    double px = x_(0);
    double py = x_(1);
    double px2 = pow(x_(0), 2);
    double py2 = pow(x_(1), 2);
    double theta = atan2(py,px);

    hx << sqrt(px2 + py2), theta, ((x_(0) * x_(2)) + (x_(1) * x_(3))) / sqrt(px2 + py2);
    
    y = z - hx;

    while(y(1)>M_PI || y(1) <-1*M_PI)
    {
      if (y(1)>M_PI)
      {
        y(1)-=M_PI;
      }
      if (y(1)<-M_PI)
      {
        y(1)+=M_PI;
      }
    }
  Ht = H_.transpose();
  S = H_ * P_ * Ht + R_;
  Si = S.inverse();
  K =  P_ * Ht * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}
