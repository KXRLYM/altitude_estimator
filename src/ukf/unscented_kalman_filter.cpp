#include <iostream>
#include <random>
#include <cmath>
#include <complex>
#include <string>
#include <eigen3/Eigen/Dense>

#include "cnpy.h"
#include "unscented_kalman_filter.h"


UKF::UKF() {

    /* ------------ read in files for demo --------------*/
    UKF::process_sensor_data();

    /* ------------ set parameters --------------*/

    x_ = Eigen::VectorXd(1); // state vector [altitude]

    std_z_ = 2; // process noise  [m]

    P_ = std::pow(std_z_, 2); // (co)variance (matrix) for process noise

    std_gps_z_ = 0.5; // standard deviation for GPS [m]

    std_sensor_z_ = 0.5; // standard deviation for proximity sensor [m]
    

}

void UKF::run_simulation() {
    // run the unscented Kalman filter with dummy data
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

        gps_data_ = Eigen::Map<Eigen::VectorXd>(gps_data_ptr, size);
        sensor_data_ = Eigen::Map<Eigen::VectorXd>(sensor_data_ptr, size);
}


void UKF::prediction() {

}

void UKF::plot() {

}


int main() {
    UKF unscented_KF;
    unscented_KF.run_simulation();
}