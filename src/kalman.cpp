/**
* Implementation of KalmanFilter class.
*
* @author: Hayk Martirosyan
* @date: 2014.11.15
*/

#include <iostream>
#include <stdexcept>

#include "kalman.hpp"

/**
 * Our state(x): x, y, theta, x_dot, y_dot
 * Our control input (u): cmd_vel (v, omega)
 * Our measurement (y): odom (v, omega)
 */
KalmanFilter::KalmanFilter(
    double dt,
    const cv::Mat& Q,
    const cv::Mat& R,
    const cv::Mat& P)
  : Q(Q), R(R), P0(P),
    m(4), n(9), dt(dt), initialized(false),
    I(cv::Mat::eye(n, n, CV_32F)), x_hat(cv::Mat(n, 1, CV_32F)), x_hat_new(cv::Mat(n, 1, CV_32F))
{
  C = cv::Mat::zeros(m, n, CV_32F);
  C.at<float>(0, 3) = 1;
  C.at<float>(1, 4) = 1;
}

KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(double t0, const cv::Mat& x0) {
  x_hat = x0.clone();
  P = P0.clone();
  this->t0 = t0;
  t = t0;
  initialized = true;
}

void KalmanFilter::init() {
  x_hat = cv::Mat::zeros(n, n, CV_32F);
  P = P0.clone();
  t0 = 0;
  t = t0;
  initialized = true;
}

void KalmanFilter::update(const cv::Mat& y, const cv::Mat& u) {
  
  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");

  float cosTheta = cos(x_hat.at<float>(2, 0)); 
  float sinTheta = sin(x_hat.at<float>(2, 0));
  float x_r =  x_hat.at<float>(0, 0);
  float y_r =  x_hat.at<float>(1, 0);
  float x_p =  x_hat.at<float>(5, 0);
  float y_p =  x_hat.at<float>(6, 0);

  // x_hat_newx_hat_new.at<float>(7, 0) =  = A * x_hat;
  /*
   * Our state propagation model is nonlinear :(, cannot do matrix multiplication directly
   */
  x_hat_new.at<float>(0, 0) = x_hat.at<float>(0, 0) + x_hat.at<float>(3, 0) * cos(x_hat.at<float>(2, 0)) * dt;
  x_hat_new.at<float>(1, 0) = x_hat.at<float>(1, 0) + x_hat.at<float>(3, 0) * sin(x_hat.at<float>(2, 0)) * dt;
  x_hat_new.at<float>(2, 0) = x_hat.at<float>(2, 0) + x_hat.at<float>(4, 0) * dt;
  // no velocity smoothing prior
  // x_hat_new.at<float>(3, 0) = u.at<float>(0, 0);
  // x_hat_new.at<float>(4, 0) = u.at<float>(1, 0);
  x_hat_new.at<float>(3, 0) = (x_hat.at<float>(3, 0) + u.at<float>(0, 0))/2.0;
  x_hat_new.at<float>(4, 0) = (x_hat.at<float>(4, 0) + u.at<float>(1, 0))/2.0;
  // assume static prior on other robot position :(
  // x_hat_new.at<float>(5, 0) = x_hat.at<float>(5, 0);
  // x_hat_new.at<float>(6, 0) = x_hat.at<float>(6, 0);
  x_hat_new.at<float>(5, 0) = (x_hat.at<float>(5, 0) + x_hat.at<float>(7, 0)*dt);
  x_hat_new.at<float>(6, 0) = (x_hat.at<float>(6, 0) + x_hat.at<float>(8, 0)*dt);
  x_hat_new.at<float>(7, 0) = x_hat.at<float>(7, 0);
  x_hat_new.at<float>(8, 0) = x_hat.at<float>(8, 0);

  std::cout << "Prior: " << x_hat_new.t() << std::endl;

  // reduce the states
  // cv::Mat x_red = x_hat_new.rowRange(0, 7);
  // cv::Mat A_red = A.rowRange(0, 7).colRange(0, 7);
  // cv::Mat P_red = P.rowRange(0, 7).colRange(0, 7);
  // cv::Mat C_red = C.colRange(0, 7);
  // cv::Mat Q_red = Q.rowRange(0, 7).colRange(0, 7);

  // P_red = A_red*P_red*A_red.t() + Q_red;
  // K = P_red*C_red.t()*(C_red*P_red*C_red.t() + R).inv();

  P = A*P*A.t() + Q;
  K = P*C.t()*(C*P*C.t() + R).inv();
  
  cv::Mat h(m, 1, CV_32F);
  h.at<float>(0, 0) = x_hat_new.at<float>(3, 0);
  h.at<float>(1, 0) = x_hat_new.at<float>(4, 0);
  // when our measurement is relative pose
  // h.at<float>(2, 0) = cosTheta*x_p + sinTheta*y_p - (cosTheta*x_r + sinTheta * y_r) * x_p;
  // h.at<float>(3, 0) = -sinTheta*x_p + cosTheta*y_p + (sinTheta* x_r - cosTheta*y_r) * y_p;
  // when our measurement is absolute pose
  h.at<float>(2, 0) = x_p;
  h.at<float>(3, 0) = y_p;

  // std::cout << "y: " << y.t() << " h: " << h.t() << std::endl;
  // x_red += K * (y - h);
  // P_red = (I.rowRange(0, 7).colRange(0, 7) - K*C_red)*P_red;
  // x_red.copyTo(x_hat.rowRange(0, 7));
  // P_red.copyTo(P.rowRange(0, 7).colRange(0, 7));
  
  x_hat_new += K * (y - h);
  P = (I - K*C)*P;
  x_hat = x_hat_new;

  std::cout << "Posteriori: " << x_hat.t() << std::endl;

  t += dt;
}

void KalmanFilter::update(const cv::Mat& y, double dt, const cv::Mat& u) {

  float cosTheta = cos(x_hat.at<float>(2, 0)); 
  float sinTheta = sin(x_hat.at<float>(2, 0));
  float x_r =  x_hat.at<float>(0, 0);
  float y_r =  x_hat.at<float>(1, 0);
  float x_p =  x_hat.at<float>(5, 0);
  float y_p =  x_hat.at<float>(6, 0);
  
  this->A = cv::Mat::zeros(n, n, CV_32F);
  A.at<float>(0, 0) = A.at<float>(1, 1) = A.at<float>(2, 2) = 1;
  
  // no velocity smoothing prior
  // A.at<float>(3, 3) = A.at<float>(4, 4) = 0;
  A.at<float>(3, 3) = A.at<float>(4, 4) = 0.5;

  A.at<float>(0, 2) = -x_hat.at<float>(3, 0) * sinTheta * dt;
  A.at<float>(0, 3) = cosTheta* dt;

  A.at<float>(1, 2) = x_hat.at<float>(3, 0) * cosTheta * dt;
  A.at<float>(1, 3) = sinTheta * dt;

  A.at<float>(2, 4) = dt;

  A.at<float>(5, 5) = A.at<float>(6, 6) = A.at<float>(7, 7) = A.at<float>(8, 8) = 1;

  A.at<float>(5, 7) = A.at<float>(6, 8) = dt;

  C.at<float>(0, 3) = 1;
  C.at<float>(1, 4) = 1;
  C.at<float>(2, 5) = 1;
  C.at<float>(3, 6) = 1;
  // C.at<float>(2, 0) = -cosTheta*x_p;
  // C.at<float>(2, 1) = -sinTheta*x_p;
  // C.at<float>(2, 2) = -sinTheta*x_p + cosTheta*y_p - (-sinTheta*x_r+cosTheta*y_r) * x_p;
  // C.at<float>(2, 5) = cosTheta - (cosTheta*x_r + sinTheta*y_r);
  // C.at<float>(2, 6) = sinTheta;

  // C.at<float>(3, 0) = sinTheta*y_p;
  // C.at<float>(3, 1) = -cosTheta*y_p;
  // C.at<float>(3, 2) = -cosTheta*x_p - sinTheta*y_p + (cosTheta*x_r + sinTheta*y_r)*y_p;
  // C.at<float>(3, 5) = -sinTheta;
  // C.at<float>(3, 6) = cosTheta + sinTheta*x_r - cosTheta*y_r;
  
  this->dt = dt;
  update(y, u);
}
