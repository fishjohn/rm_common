//
// Created by huakang on 2021/3/21.
//
#include "rm_common/chassis_base.h"
#include <rm_common/ros_utilities.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <rm_common/ori_tool.h>
#include <rm_msgs/ChassisCmd.h>

namespace rm_chassis_base {
void ChassisBase::update(const ros::Time &time, const ros::Duration &period) {
  cmd_chassis_ = *chassis_rt_buffer_.readFromRT();
  ramp_x->setAcc(cmd_chassis_.accel.linear.x);
  ramp_y->setAcc(cmd_chassis_.accel.linear.y);
  ramp_w->setAcc(cmd_chassis_.accel.angular.z);

  geometry_msgs::Twist vel_cmd;
  vel_cmd = *vel_rt_buffer_.readFromRT();
  vel_cmd_.vector.x = vel_cmd.linear.x;
  vel_cmd_.vector.y = vel_cmd.linear.y;
  vel_cmd_.vector.z = vel_cmd.angular.z;

  updateOdom(time, period);

  if (state_ != cmd_chassis_.mode) {
    state_ = StandardState(cmd_chassis_.mode);
    state_changed_ = true;
  }

  if (state_ == PASSIVE)
    passive();
  else {
    if (state_ == RAW)
      raw();
    else if (state_ == GYRO)
      gyro(time);
    else if (state_ == FOLLOW)
      follow(time, period);
    else if (state_ == TWIST)
      twist(time, period);
    moveJoint(period);
  }
}

}
