#include <iostream>
#include <random>
#include <cmath>
#include <complex>
#include <string>
#include <eigen3/Eigen/Dense>

#include "cnpy.h"
#include "unscented_kalman_filter.h"
#include "matplot/matplot.h"


UKF::UKF() {

    /* ------------ read in files for demo --------------*/

    UKF::process_sensor_data();

    /* ------------ set parameters for Kalman filters --------------*/

    dim_ = 1; // dimension of a state vector

    x_ = Eigen::VectorXd(dim_); // state vector [altitude z]

    std_z_ = 20; // process noise [m]

    std_gps_z_ = 2; // standard deviation for GPS [m]

    std_sensor_z_ = 1; // standard deviation for proximity sensor [m]

    F_ = Eigen::MatrixXd(dim_, dim_); // state transition matrix

    P_ = Eigen::MatrixXd(dim_, dim_); // state covariance matrix

    Q_ = Eigen::MatrixXd(dim_, dim_); // process covariance matrix
    
    R_gps_ = Eigen::MatrixXd(dim_, dim_); // noise covariance matrix for gps

    R_prox_ = Eigen::MatrixXd(dim_, dim_); // noise covariance matrix for proximity sensor

}

void UKF::run_simulation() {
    /*  runs the unscented Kalman filter with dummy data and plots the output */
    
    // initialise the simulation time step
    size_t current_time_step(0);

    // set the initial state to be the average of two values
    x_(current_time_step) = (get_gps_measurement(current_time_step) + get_prox_sensor_measurement(current_time_step)) / 2; 

    // initialise a state covariance matrix with a std of 10m
    P_(0) = 100; 

    // construct the process covariance matrix
    Q_(0) = std::pow(std_z_, 2);

    while (current_time_step < simulation_end_time_) {
        double z_gps = get_gps_measurement(current_time_step);
        double z_prox = get_prox_sensor_measurement(current_time_step);

        predict(); 
        correct(z_gps, z_prox); 

        results_(current_time_step) = x_(0);
        current_time_step++;
    }

    // plot the final result
    plot();

    
}

double UKF::get_gps_measurement(size_t time_step) {
    // normally sensor read from the actual running hardware
    return gps_data_[time_step];
}

double UKF::get_prox_sensor_measurement(size_t time_step) {
    // normally sensor read from the actual running hardware
    return sensor_data_[time_step];
}

        
void UKF::process_sensor_data() {
    // read 2 sensor data files - GPS and proximity sensor and store them as class variables

    std::string file_path_gps = std::string(DATA_DIR)+ "cootha_gps_noise.npy";
    std::string file_path_sensor = std::string(DATA_DIR)+ "cootha_sensor_noise.npy";

    cnpy::NpyArray gps_arr = cnpy::npy_load(file_path_gps);
    cnpy::NpyArray sensor_arr = cnpy::npy_load(file_path_sensor);

    double* gps_data_ptr = gps_arr.data<double>();
    double* sensor_data_ptr = sensor_arr.data<double>();

    // assume the same sized array for both sensors
    size_t size = gps_arr.shape[0];
    
    UKF::gps_data_.resize(size);
    UKF::sensor_data_.resize(size);

    results_.resize(size);

    gps_data_ = Eigen::Map<Eigen::VectorXd>(gps_data_ptr, size);
    sensor_data_ = Eigen::Map<Eigen::VectorXd>(sensor_data_ptr, size);

    simulation_end_time_ = static_cast<int>(size);
}


void UKF::predict() {
    /* Prediction stage of the Kalman filter, updates x_ and P_ */

    
    // construct the process covariance matrix
    Eigen::MatrixXd P_new_(dim_, dim_);

    // constrct the transition matrix - using a static model 
    F_ << 1; 

    // new state update using a static model
    x_ = F_*x_;

    // Propagate the process noise
    P_new_ << (F_*P_*F_.transpose() + Q_);

    // update the state covariance
    P_ = P_new_;

}

void UKF::correct(double gps_data, double prox_data) {
    /* Prediction stage of the Kalman filter, updates x_ and P_ */

    Eigen::MatrixXd S_gps = P_ + R_gps_; // S = P + R_gps
    Eigen::MatrixXd K_gps = P_ * S_gps.inverse(); // K_gps = P * S^(-1)

    Eigen::MatrixXd S_prox = P_ + R_prox_; // S = P + R_prox
    Eigen::MatrixXd K_prox = P_ * S_prox.inverse(); // K_prox = P * S^(-1)

    Eigen::VectorXd innovation_gps = Eigen::VectorXd::Constant(dim_, gps_data) - x_; // Subtract x_ from gps_data
    Eigen::VectorXd innovation_prox = Eigen::VectorXd::Constant(dim_, prox_data) - x_; // Subtract x_ from prox_data

    Eigen::MatrixXd x_gps_ = K_gps * innovation_gps;
    Eigen::MatrixXd x_prox_ = K_prox * innovation_prox;

    x_ += ((x_gps_ + x_prox_) / 2);

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(P_.rows(), P_.cols());
    Eigen::MatrixXd P_new_ = (I - K_gps) * P_; 

    P_ = P_new_;
   
}


void UKF::plot() {
    std::cout << results_.transpose() << std::endl;
    std::vector<double> results(results_.data(), results_.data() + results_.size());
    std::vector<double> gps(gps_data_.data(), gps_data_.data() + gps_data_.size());
    std::vector<double> prox(sensor_data_.data(), sensor_data_.data() + sensor_data_.size());

    matplot::figure();
    matplot::hold(matplot::on);
    
    // Plot each dataset and set colors
    auto line_results = matplot::plot(results);
    line_results->color("blue");

    auto line_gps = matplot::plot(gps);
    line_gps->color("red");


    auto line_prox = matplot::plot(prox);
    line_prox->color("green");


    // Add legend
    matplot::legend({"Kalman Filter","GPS","Proximity Sensor"});

    // Show the plot
    matplot::title("Altitude State Estimation Using a Kalman Filter");
    matplot::xlabel("Timestep");
    matplot::ylabel("Altitude [m]");
    matplot::show();

}


int main() {
    UKF unscented_KF;
    unscented_KF.run_simulation();
}