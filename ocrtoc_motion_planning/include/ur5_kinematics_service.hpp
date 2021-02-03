#pragma once
#include <ros/ros.h>
#include <string>
#include <cstring>
#include <math.h>
#include <stdio.h>
#include "motion_planning/UR5IKService.h"
namespace ur5_kinematics
{
const double ZERO_THRESH = 0.00000001;
const double PI = M_PI;
const double d1 = 0.1625;
const double a2 = -0.425;
const double a3 = -0.3922;
const double d4 = 0.1333;
const double d5 = 0.0997;
const double d6 = 0.0996 + 0.145;
int SIGN(double x) {
return (x > 0) - (x < 0);
}

void forward(const double* q, double* T);
const std::vector<std::string> JOINT_NAMES {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
int inverse(const double* T, double* q_sols, double q6_des = 0.0);


/**
 * @class UR5Kinematics
 * @brief Provides an interface for UR5 kinematics solvers inherited from KinematicsBase
 */

bool ur5_ik_service(motion_planning::UR5IKService::Request &req,
                    motion_planning::UR5IKService::Response &res);
}
int main(int argc, char **argv);
