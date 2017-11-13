#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

const float EPSILON = 0.0001;

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

  x_ = F_ * x_;  //state prediction
  P_ = F_ * P_ * F_.transpose() + Q_;  //covariance matrix update

  cout << "F = " << F_ << endl;
  cout << "x' = " << x_ << endl;
  cout << "P = " << P_ << endl;
}

void KalmanFilter::Update(const VectorXd &z) {

  //equations for LIDAR measurements
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_ * PHt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;

  cout << "y = " << y << endl;
  cout << "H = " << H_ << endl;
  cout << "Ht = " << Ht << endl;
  cout << "PHt = " << PHt << endl;
  cout << "S = " << S << endl;
  cout << "Si = " << Si << endl;
  cout << "K = " << K << endl;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

  cout << "New x = " << x_ << endl;
  cout << "x_size = " << x_size << endl;
  cout << "I = " << I << endl;
  cout << "P = " << P_ << endl;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  
  //equations for RADAR measurements
  
  Tools tools;

  //define variables for easier reading and saving cycles
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  float rho = sqrt(px * px + py * py);
  float rho_dot;

  cout << "px= " << px << endl;
  cout << "py = " << py << endl;
  cout << "vx = " << vx << endl;
  cout << "vy = " << vy << endl;
  cout << "rho = " << rho << endl;

  //measurement function
  VectorXd h_of_x(3);

  //measurement error
  VectorXd y(3);
  
  if (fabs(px) < EPSILON)
  {
    cout << "Px close to zero\n";
    px = EPSILON;
  }

  if (rho < EPSILON) 
  { //if division by zero
    cout << "Division by zero in UpdateEFK()\n";
    rho = EPSILON;
    rho_dot = 0;

  } else
  { 
    rho_dot = (px * vx + py * vy) / rho;
  }

  h_of_x << rho,
            atan2(py, px),
            rho_dot;
  
  y = z - h_of_x;  
  
  cout << "h(x') = " << h_of_x << endl;
  cout << "y = z - h(x') = " << y << endl;

  //normalize the angle element of y
  y(1) = atan2(sin(y(1)), cos(y(1)));

  cout << "After normalization, y = " << y << endl;
  
  MatrixXd Hjt = H_.transpose();
  MatrixXd PHjt = P_ * Hjt;
  MatrixXd S = H_ * PHjt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHjt * Si;

  cout << "Hjt=" << Hjt << endl;
  cout << "PHjt= " << PHjt << endl;
  cout << "S = " << S << endl;
  cout << "Si = " << Si << endl;
  cout << "K = " << K << endl;

  //new estimate
  x_ = x_ + K * y;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

  cout << "New x = " << x_ << endl;
  cout << "x_size = " << x_size << endl;
  cout << "I = " << I << endl;
  cout << "P = " << P_ << endl; 
}
