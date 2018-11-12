/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, PickNik Consulting
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Consulting nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Object for wrapping remote control functionality
*/

#ifndef RVIZ_VISUAL_TOOLS_GRIPPER_H
#define RVIZ_VISUAL_TOOLS_GRIPPER_H

#include <std_msgs/Bool.h>
#include <ros/ros.h>

namespace rviz_visual_tools
{
class Gripper
{
public:
  Gripper()
  {
    sim_gripper_publisher_ = nh_.advertise<std_msgs::Bool>("/sim_gripper", 1);
    real_gripper_publisher_ = nh_.advertise<std_msgs::Bool>("/gripper_joint", 1);
  }

  void simclose()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "Closing sim gripper...");
    std_msgs::Bool msg;
    msg.data = false;
    sim_gripper_publisher_.publish(msg);
  }

  void simopen()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "Opening sim gripper...");
    std_msgs::Bool msg;
    msg.data = true;
    sim_gripper_publisher_.publish(msg);
  }

  void realclose()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "Closing real gripper...");
    std_msgs::Bool msg;
    msg.data = false;
    real_gripper_publisher_.publish(msg);
  }

  void realopen()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "Opening real gripper...");
    std_msgs::Bool msg;
    msg.data = true;
    real_gripper_publisher_.publish(msg);
  }

protected:
  // The ROS publishers
  ros::Publisher sim_gripper_publisher_;
  ros::Publisher real_gripper_publisher_;

  // The ROS node handle.
  ros::NodeHandle nh_;
};

}  // end namespace rviz_visual_tools

#endif  // RVIZ_VISUAL_TOOLS_REMOTE_RECIEVER_H
