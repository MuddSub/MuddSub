#pragma once

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <math.h>
#include <ros/ros.h>

namespace MuddSub::Controls {

    /**
     * This class implements the thruster allocation as described in Optimization of Thrust Allocation in the Propulsion System of an Underwater Vehicle
     * by Jerzy Garus (http://matwbn.icm.edu.pl/ksiazki/amc/amc14/amc1443.pdf). 
     * 
     * The purpose of this class is to take in a force vector computed by the control algorithm of the craft, then
     * compute how much force should be output by each individual thruster. This is output in a thrust vector with
     * a row per thruster.
     * 
     * 
     * TODO:
     *  1) Setup ROS functionality to subscribe for the force vector from controls
     *  2) Setup ROS functionality to publish the thrust vector
     *  3) Fix resetConfiguration to actually get values from a parameter server
     *  4) Current implementation fixes the dimensions of the thrust configuration matrix, if any other thrusters are added/removed, this will need to be changed
     *      - Generalize it
     *  5) The paper outlines an algorithm variation for dynamic P-Matrix (i.e. some non-operational thrusters)
     */ 
template <size_t thrustDim, size_t numOfThrusters>
class ThrusterManager
{
    public:
        // This will be the T matrix in most thrust alloc papers
        using T_Config_Matrix_t = Eigen::Matrix<double, thrustDim, numOfThrusters>;

        // This will be the thruster availability square matrix
        using PMatrix_t = Eigen::Matrix<size_t, numOfThrusters, numOfThrusters>;

        // This is the input for the thrust allocation function
        using ForceVector_t = Eigen::Matrix<double, thrustDim, 1>;

        // This is the output for the thrust allocation function
        using ThrustVector_t = Eigen::Matrix<double, numOfThrusters, 1>;


        // The default constructor 
        // This should also initialize the configuration matrix
        // As well as initialize the P matrix
        ThrusterManager();


        // Driving function that converts a given force vector in 6 DOF to 
        // a thrust vector of numOfThrusters 
        ThrustVector_t calculateThrust(const ForceVector_t& force);

        void resetConfiguration();

    private:

        typedef struct ThrustersInfo_st 
        {
            std::string name_;
            /* For visual example, see http://matwbn.icm.edu.pl/ksiazki/amc/amc14/amc1443.pdf
            * page 2
            * d - distance from thruster to center of gravity
            * alpha - angle of thruster from longitudinal axis in the direction
            *         the thrust faces
            * phi - angle of thruster from longitudinal axis in the opposite direction
            *          the thrust faces
            * gamma - alpha_i - phi_i
            * Note: map keys correspond to l/v (longitudinal/vertical), f/b (front/back), l/r (left/right)
            */
            std::map<std::string, double> d_, alpha_, phi_, gamma_;
        } ThrusterInfo;

        ThrusterInfo thrustersInfo_;

        PMatrix_t pMatrix_;

        T_Config_Matrix_t tMatrix_{T_Config_Matrix_t::Zero()};

        ros::NodeHandle nh_;
};
} 
#include "controls/ThrusterManager.inl"