#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    /**
    TODO:
      * Calculate the RMSE here.
    */
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if (estimations.size() != ground_truth.size()
        || estimations.size() == 0) {
        cout << "Invalid estimation or ground_truth data" << endl;
        return rmse;
    }

    //accumulate squared residuals
    for (unsigned int i = 0; i < estimations.size(); ++i) {

        VectorXd residual = estimations[i] - ground_truth[i];

        //coefficient-wise multiplication
        residual = residual.array() * residual.array();
        rmse += residual;
    }

    //calculate the mean
    rmse = rmse / estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
    /**
    TODO:
      * Calculate a Jacobian here.
    */
    MatrixXd Hj(3, 4);
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    //TODO: YOUR CODE HERE

    //check division by zero
    if (px == 0 || py == 0 || vx == 0 || vy == 0) return Hj;

    //compute the Jacobian matrix
    float sqsum = px * px + py * py;

    float drhopx = px / sqrt(sqsum);
    float drhopy = py / sqrt(sqsum);

    float dphipx = -(py / sqsum);
    float dphipy = px / sqsum;

    double drhodotpx = px * (vx * py - vy * px) / pow(sqsum, 3 / 2.0);
    double drhodotpy = py * (vy * px - vx * py) / pow(sqsum, 3 / 2.0);

    Hj << drhopx, drhopy, 0, 0,
            dphipx, dphipy, 0, 0,
            drhodotpx, drhodotpy, drhopx, drhopy;
    return Hj;
}
