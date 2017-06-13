#include "kalman_filter.h"
#include "tools.h"
#define PI 3.14159265359

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
    * predict the state
    */
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    /**
    * update the state by using Kalman Filter equations
    */
    VectorXd z_pred = H_ * x_;
    VectorXd err = z - z_pred;
    correction(err);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    /**
    * update the state by using Extended Kalman Filter equations
    */

    double px = x_(0), py=x_(1), vx=x_(2), vy=x_(3);
    double rho = sqrt(px*px+py*py);
    double theta = atan2(py,px);
    VectorXd z_pred(3);
    z_pred<< rho, theta, (px*vx+py*vy)/rho;

    VectorXd err = z - z_pred;
    //bound -PI <= err_theta <= PI
    if (err(1) > PI)    err(1) = err(1) - 2*PI;
    if (err(1) < -PI)   err(1) = err(1) + 2*PI;

    correction(err);
}

void KalmanFilter::correction(const Eigen::VectorXd &err)
{
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * err);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}
