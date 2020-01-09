/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   orbo_kinematics_dynamics_define.h
 * Author: mario
 *
 * Created on 13 November 2019, 12:37
 */

#ifndef ORBO_KINEMATICS_DYNAMICS_DEFINE_H
#define ORBO_KINEMATICS_DYNAMICS_DEFINE_H

#define MAX_JOINT_ID    (18)
#define ALL_JOINT_ID    (15)

// #define MAX_ARM_ID      (3)
#define MAX_LEG_ID      (6)
#define MAX_ITER        (5)

// #define ID_HEAD_END     (20)
#define ID_COB          (15)
#define ID_TORSO        (15)

// #define ID_R_ARM_START  (1)
// #define ID_L_ARM_START  (2)
// #define ID_R_ARM_END    (21)
// #define ID_L_ARM_END    (22)

#define ID_R_LEG_START  (1)
#define ID_L_LEG_START  (2)
#define ID_R_LEG_END    (13)
#define ID_L_LEG_END    (14)

#define GRAVITY_ACCELERATION (9.8)


#endif /* ORBO_KINEMATICS_DYNAMICS_DEFINE_H */

