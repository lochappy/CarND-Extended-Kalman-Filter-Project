#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "FusionEKF.h"
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


int main(int argc, char* argv[])
{

    if (argc < 3){
        std::cout << "USESAGE: " << argv[0] << " [INPUT] [OUTPUT] \n";
        return -1;
    }
    /*******************************************************************************
     *  Set Measurements															 *
     *******************************************************************************/
    vector<MeasurementPackage> measurement_pack_list;
    vector<VectorXd> measurement_xy;

    Tools tools;
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;

    // hardcoded input file with laser and radar measurements
    string in_file_name_(argv[1]);//"../../data/obj_pose-laser-radar-synthetic-input.txt";
    ifstream in_file(in_file_name_.c_str(),std::ifstream::in);
    string out_file_name_(argv[2]);// = "../../data/obj_pose-laser-radar-synthetic-output.txt";
    ofstream out_file(out_file_name_.c_str(),std::ofstream::out);

    if (!in_file.is_open()) {
        cout << "Cannot open input file: " << in_file_name_ << endl;
    }

    string line;
    // set i to get only first 3 measurments
    int i = 0;
    while(getline(in_file, line) /*&& (i<=273)*/){

        MeasurementPackage meas_package;

        istringstream iss(line);
        string sensor_type;
        iss >> sensor_type;	//reads first element from the current line
        long timestamp;
        if(sensor_type.compare("L") == 0){	//laser measurement
            //read measurements
            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = VectorXd(2);
            float x;
            float y;
            iss >> x;
            iss >> y;
            meas_package.raw_measurements_ << x,y;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
            VectorXd pose(2);
            pose << x,y;
            measurement_xy.push_back(pose);

        }else if(sensor_type.compare("R") == 0){
            //Skip Radar measurements
            //continue;
            meas_package.sensor_type_ = MeasurementPackage::RADAR;
            meas_package.raw_measurements_ = VectorXd(3);
            float ro;
            float theta;
            float ro_dot;
            iss >> ro;
            iss >> theta;
            iss >> ro_dot;
            meas_package.raw_measurements_ << ro, theta, ro_dot;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;

            VectorXd pose(2);
            float x = ro*cos(theta),
                  y = ro*sin(theta);
            pose << ro*cos(theta),ro*sin(theta);
            measurement_xy.push_back(pose);

        }

        measurement_pack_list.push_back(meas_package);

        float x_gt;
        float y_gt;
        float vx_gt;
        float vy_gt;
        iss >> x_gt;
        iss >> y_gt;
        iss >> vx_gt;
        iss >> vy_gt;
        VectorXd gt_values(4);
        gt_values(0) = x_gt;
        gt_values(1) = y_gt;
        gt_values(2) = vx_gt;
        gt_values(3) = vy_gt;
        ground_truth.push_back(gt_values);

        i++;

    }

    // Create a Kalman Filter instance
    FusionEKF fusionEKF;
    for(int i=0; i<measurement_pack_list.size(); i++){
        fusionEKF.ProcessMeasurement(measurement_pack_list[i]);

        //Push the current estimated x,y positon from the Kalman filter's state vector
        VectorXd estimate(4);

        double p_x = fusionEKF.ekf_.x_(0);
        double p_y = fusionEKF.ekf_.x_(1);
        double v1  = fusionEKF.ekf_.x_(2);
        double v2 = fusionEKF.ekf_.x_(3);

        estimate(0) = p_x;
        estimate(1) = p_y;
        estimate(2) = v1;
        estimate(3) = v2;

        estimations.push_back(estimate);
        out_file << p_x << "\t"
                 << p_y << "\t"
                 << v1 << "\t"
                 << v2 << "\t"
                 << measurement_xy[i](0) << "\t"
                 << measurement_xy[i](1)  << "\t"
                 << ground_truth[i](0) << "\t"
                 << ground_truth[i](1) << "\t"
                 << ground_truth[i](2) << "\t"
                 << ground_truth[i](3) << "\n";
    }

    VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);
    std::cout << "RMSE = [" << RMSE(0) << ", "
                            << RMSE(1) << ", "
                            << RMSE(2) << ", "
                            << RMSE(3) << "]"<<std::endl;

    return 0;
}









































































