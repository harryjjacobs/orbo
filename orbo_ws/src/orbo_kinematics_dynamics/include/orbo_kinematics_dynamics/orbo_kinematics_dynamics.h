/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   orbo_kinematics_dynamics.h
 * Author: mario
 *
 * Created on 13 November 2019, 12:57
 */

#ifndef ORBO_KINEMATICS_DYNAMICS_H
#define ORBO_KINEMATICS_DYNAMICS_H

#include <vector>
#include <eigen3/Eigen/Eigen>

#include "orbo_kinematics_dynamics_define.h"
#include "link_data.h"

enum TreeSelect
{
//  Manipulation,
  Walking,
  WholeBody
};

class OrboKinematicsDynamics
{

 public:
     
  OrboKinematicsDynamics();
  ~OrboKinematicsDynamics();
  OrboKinematicsDynamics(TreeSelect tree);

  std::vector<int> findRoute(int to);
  std::vector<int> findRoute(int from, int to);

  double calcTotalMass(int joint_id);
  
  Eigen::MatrixXd calcMassCenter(int joint_id);
  Eigen::MatrixXd calcCenterOfMass(Eigen::MatrixXd mc);

  void calcForwardKinematics(int joint_ID);

  Eigen::MatrixXd calcJacobian(std::vector<int> idx);
  Eigen::MatrixXd calcJacobianCenterOfMass(std::vector<int> idx);
  
  Eigen::MatrixXd calcVWerr(Eigen::MatrixXd tar_position, Eigen::MatrixXd curr_position,
                            Eigen::MatrixXd tar_orientation, Eigen::MatrixXd curr_orientation);

  bool calcInverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter,
                             double ik_err);
  bool calcInverseKinematics(int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation,
                             int max_iter, double ik_err);

  // with weight
  bool calcInverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter,
                             double ik_err, Eigen::MatrixXd weight);
  bool calcInverseKinematics(int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation,
                             int max_iter, double ik_err, Eigen::MatrixXd weight);

  
  bool calcInverseKinematicsForLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw);
  
  bool calcInverseKinematicsForRightLeg(double *out, double x, double y, double z, double roll, double pitch,
                                        double yaw);
  
  bool calcInverseKinematicsForLeftLeg(double *out, double x, double y, double z, double roll, double pitch,
                                       double yaw);

  LinkData *orbo_link_data_[ ALL_JOINT_ID + 1];

  LinkData *getLinkData(const std::string link_name);
  
  LinkData *getLinkData(const int link_id);
  
  Eigen::MatrixXd getJointAxis(const std::string link_name);
  
  double getJointDirection(const std::string link_name);
  double getJointDirection(const int link_id);

  Eigen::MatrixXd calcPreviewParam(double preview_time, double control_cycle,
                                   double lipm_height,
                                   Eigen::MatrixXd K, Eigen::MatrixXd P);

  double thigh_length_m_;
  double calf_length_m_;
  double ankle_length_m_;
  double leg_side_offset_m_;
  
};


#endif /* ORBO_KINEMATICS_DYNAMICS_H */

