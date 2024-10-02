#ifndef UFK_H
#define UKF_H

#include <eigen3/Eigen/Dense>

class UKF {
    public:
        UKF();

        int dim_; // state vector dimension
        
        Eigen::VectorXd x_; // state vector [altitude]

        double std_z_; // standard deviation for process noise [m]

        double std_gps_z_; // standard deviation for gps sensor [m]

        double std_sensor_z_; // standard deviation for proximity sensor noise [m]

        Eigen::MatrixXd F_; // state transition matrix

        Eigen::MatrixXd Q_; // process covariance matrix

        Eigen::MatrixXd P_; // state covariance matrix

        Eigen::MatrixXd R_gps_; // measurement noise matrix for gps

        Eigen::MatrixXd R_prox_; // measurement noise matrix for proximity sensor


        void run_simulation();
   
    private: 
        Eigen::VectorXd sensor_data_;
        Eigen::VectorXd gps_data_;
        Eigen::VectorXd averaged_data_;
        size_t simulation_end_time_;
        Eigen::VectorXd results_;

        void process_sensor_data();

        double get_gps_measurement(size_t time_step);

        double get_prox_sensor_measurement(size_t time_step);

        void predict();

        void correct(double gps_data, double prox_data);

        void plot();
};

#endif /* UFK_H */