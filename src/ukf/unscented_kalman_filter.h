#ifndef UFK_H
#define UKF_H

#include <eigen3/Eigen/Dense>

class UKF {
    public:
        UKF();
        
        Eigen::VectorXd x_; // state vector [altitude]

        double std_z_; // standard deviation for process noise [m]

        double P_; // (co)variance (matrix) for process noise 

        double std_gps_z_; // standard deviation for gps sensor [m]

        double std_sensor_z_; // standard deviation for proximity sensor noise [m]


        void run_simulation();
   
    private: 
        Eigen::VectorXd sensor_data_;
        Eigen::VectorXd gps_data_;

        void process_sensor_data();

        void prediction();

        void plot();
};

#endif /* UFK_H */