/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#ifndef OPEN_MANIPULATOR_TELEOP_KEYBOARD_H_
#define OPEN_MANIPULATOR_TELEOP_KEYBOARD_H_

#include <termios.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"
#include "open_manipulator_msgs/setxyz.h" //kudos

#include "std_msgs/String.h" //kudospub
#include "std_msgs/Int8.h"//kudospub

#include "geometry_msgs/Vector3.h" //deltasub
/*/kudos
#include "open_manipulator_msgs/OpenManipulatorState.h"
#include "open_manipulator_msgs/SetActuatorState.h"
//#include <QThread>
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
//kudos*/



#define NUM_OF_JOINT 4
//#define DELTA 0.01
#define DELTA 0.002
#define JOINT_DELTA 0.05
#define PATH_TIME 0.5


open_manipulator_msgs::KinematicsPose kinematics_pose_; //kudos
  ros::ServiceClient goal_task_space_path_position_only_client_;//kudos


class OpenManipulatorTeleop
{
 public:
  OpenManipulatorTeleop();
  ~OpenManipulatorTeleop();

  // update
  void printText();
  void setGoal(char ch);

 private:
  /*****************************************************************************
  ** ROS NodeHandle
  *****************************************************************************/
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  /*****************************************************************************
  ** Variables
  *****************************************************************************/
  std::vector<double> present_joint_angle_;
  std::vector<double> present_kinematic_position_;
  //open_manipulator_msgs::KinematicsPose kinematics_pose_; //kudos


  /*****************************************************************************
  ** Init Functions
  *****************************************************************************/
  void initSubscriber();
  void initClient();

  /*****************************************************************************
  ** ROS Subscribers, Callback Functions and Relevant Functions
  *****************************************************************************/
  ros::Subscriber joint_states_sub_;
  ros::Subscriber kinematics_pose_sub_;
  ros::Subscriber open_topic_sub_woo;
  ros::Subscriber open_delta_woo;

  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg);
  void opensubCallback(const std_msgs::Int8::ConstPtr opensub_msg);//kudossub
  void opendeltaCallback(const geometry_msgs::Vector3::ConstPtr opendelta_msg); //delta
  /*****************************************************************************
  ** ROS Clients and Callback Functions
  *****************************************************************************/
//  ros::ServiceClient goal_task_space_path_position_only_client_;//kudos
  ros::ServiceClient goal_joint_space_path_client_;
  ros::ServiceClient goal_joint_space_path_from_present_client_;
  ros::ServiceClient goal_task_space_path_from_present_position_only_client_;
  ros::ServiceClient goal_tool_control_client_;

  bool setTaskSpacePath(std::vector<double> kinematics_pose, double path_time); //kudos
  bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setJointSpacePathFromPresent(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setTaskSpacePathFromPresentPositionOnly(std::vector<double> kinematics_pose, double path_time);
  bool setToolControl(std::vector<double> joint_angle);

  /*****************************************************************************
  ** Others
  *****************************************************************************/
  struct termios oldt_;

  void disableWaitingForEnter(void);
  void restoreTerminalSettings(void);
  std::vector<double> getPresentJointAngle();
  std::vector<double> getPresentKinematicsPose();
};

#endif //OPEN_MANIPULATOR_TELEOP_KEYBOARD_H_
