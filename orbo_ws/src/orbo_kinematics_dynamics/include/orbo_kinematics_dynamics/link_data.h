/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   link_data.h
 * Author: mario
 *
 * Created on 13 November 2019, 12:23
 */

#ifndef LINK_DATA_H
#define LINK_DATA_H

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT

#include <eigen3/Eigen/Eigen>
#include "robotis_math/robotis_math.h"

class LinkData
{
public:

  LinkData();
  ~LinkData();

  std::string name_;

  int parent_;
  int sibling_;
  int child_;

  double mass_;

  Eigen::MatrixXd relative_position_;
  Eigen::MatrixXd joint_axis_;
  Eigen::MatrixXd center_of_mass_;
  Eigen::MatrixXd inertia_;
  Eigen::MatrixXd joint_center_of_mass_;

  double joint_limit_max_;
  double joint_limit_min_;

  double joint_angle_;
  double joint_velocity_;
  double joint_acceleration_;

  Eigen::MatrixXd position_;
  Eigen::MatrixXd orientation_;
  Eigen::MatrixXd transformation_;

};

#endif /* LINK_DATA_H */

