#include <iostream>
#include <vector>
#include "kalman_filter.hpp"
#include <math.h>
#include <Eigen/Dense>
//#include "matplotlibcpp.h"

const double dt = 0.4; // the time of simulation in seconds
double a_x = 2, a_y = 3; //acceleration on x and y axis
double dx = 6, dy = 6, dv_x = 15, dv_y = 15; /*displacement for X, for Y, 
change of velocity for X, for Y
*/
double pred_x = 20, pred_y = 15, pred_v_x = 5, pred_v_y = 3; /*predicted X value, 
predicted Y value, predicted velocity value for X, 
predicted velocity value for Y
*/
double sigma_squared = 0.3;
const int n = 4,m = 2,l = 4;

int main(){
    KalmanFilter kf;
    
    Eigen::Matrix<double,n,n> mtx_state;
    mtx_state << 1.0, 0.0, dt, 0.0,
                 0.0, 1.0, 0.0, dt,
                 0.0, 0.0, 1.0, 0.0,
                 0.0, 0.0, 0.0, 1.0;
    kf.setStateMatrix(mtx_state);
    Eigen::Matrix<double,n,m> mtx_input;
	mtx_input << 1/2*dt*dt, 0.0,
                 0.0, 1/2*dt*dt,
                 dt, 0.0,
                 0.0, dt;
    kf.setCntrlInputMatrix(mtx_input);
    Eigen::Matrix<double,n,n> mtx_proc_cov;
	mtx_proc_cov << pred_x * pred_x, pred_x * pred_y, pred_x * pred_v_x, pred_x * pred_v_y,
                   pred_y * pred_x, pred_y * pred_y, pred_y * pred_v_x, pred_y * pred_v_y,
                   pred_v_x * pred_x, pred_v_x * pred_y, pred_v_x * pred_v_x, pred_v_x * pred_v_y,
                   pred_v_y * pred_x, pred_v_y * pred_y, pred_v_y * pred_v_x, pred_v_y * pred_v_y;
    kf.setProcessCovMatrix(mtx_proc_cov);
    Eigen::Matrix<double,n,n> mtx_proc_err_cov;
	mtx_proc_err_cov << 1/36 * pow(dt,6), 1/12 * pow(dt,5), 1/6 * pow(dt,4), 1/6 * pow(dt,3),
                      1/12 * pow(dt,5), 1/4 * pow(dt,4), 0.5 * pow(dt,3), dt * dt,
                      1/6 * pow(dt,4), 0.5 * pow(dt,3), dt * dt, dt,
                      1/6 * pow(dt,3), 0.5 * dt * dt, dt, 1;
    mtx_proc_err_cov *= sigma_squared;
    kf.setProcessNoiseCovMatrix(mtx_proc_err_cov);
    Eigen::Matrix<double,l,l> mtx_meas_noise_cov;
    mtx_meas_noise_cov << dx * dx, 0.0, 0.0, 0.0,
                          0.0, dy * dy, 0.0, 0.0,
                          0.0, 0.0, dv_x * dv_x, 0.0,
                          0.0, 0.0, 0.0, dv_y * dv_y;
    kf.setMeasurementNoiseCovMatrix(mtx_meas_noise_cov);
    Eigen::VectorXd vec_input(m);
    vec_input << a_x, a_y;
    kf.setCntrlInputVector(vec_input);
    Eigen::Matrix<double,n,n> mtx_identity;
    mtx_identity << Eigen::MatrixXd::Identity(n,n);
    kf.setIdentityMatrix(mtx_identity);
    Eigen::Matrix<double, l,n> H;
    H << Eigen::MatrixXd::Identity(l,n);
    kf.setObservationMatrix(H);
    Eigen::VectorXd vec_state(n);
    vec_state << 4000.0, 3000.0, 280.0, 120.0;
    kf.setStateVector(vec_state);
    Eigen::VectorXd vec_meas(n);
    vec_meas << 4260.0, 3100.0, 282.0, 121.0;
    kf.setMeasuredVector(vec_meas);
    Eigen::VectorXd vec_ctrl(m);
    vec_ctrl << 2.0, 2.0;
    kf.setCntrlInputVector(vec_ctrl);
    kf.summarized();
    Eigen::VectorXd M = kf.getStateVector();
    std::cout << M;

}
