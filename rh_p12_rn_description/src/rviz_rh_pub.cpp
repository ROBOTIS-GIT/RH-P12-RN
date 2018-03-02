/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

ros::Publisher g_present_joint_states_pub;
ros::Publisher g_goal_joint_states_pub;

void presentJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  sensor_msgs::JointState present_msg;

  for (int index = 0; index < msg->name.size(); index++)
  {
    present_msg.name.push_back(msg->name[index]);
    present_msg.position.push_back(msg->position[index]);

    if (present_msg.name[index] == "rh_p12_rn")
    {
      present_msg.name.push_back("rh_r2");
      present_msg.position.push_back(present_msg.position[index] * (1.0 / 1.1));
      present_msg.name.push_back("rh_l1");
      present_msg.position.push_back(present_msg.position[index]);
      present_msg.name.push_back("rh_l2");
      present_msg.position.push_back(present_msg.position[index] * (1.0 / 1.1));
    }
  }
  g_present_joint_states_pub.publish(present_msg);
}

void goalJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  sensor_msgs::JointState goal_msg;

  for (int index = 0; index < msg->name.size(); index++)
  {
    goal_msg.name.push_back(msg->name[index]);
    goal_msg.position.push_back(msg->position[index]);

    if (goal_msg.name[index] == "rh_p12_rn")
    {
      goal_msg.name.push_back("rh_r2");
      goal_msg.position.push_back(goal_msg.position[index] * (1.0 / 1.1));
      goal_msg.name.push_back("rh_l1");
      goal_msg.position.push_back(goal_msg.position[index]);
      goal_msg.name.push_back("rh_l2");
      goal_msg.position.push_back(goal_msg.position[index] * (1.0 / 1.1));
    }
  }
  g_goal_joint_states_pub.publish(goal_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rviz_rh_p12_rn_publisher");
  ros::NodeHandle nh("~");

  g_present_joint_states_pub = nh.advertise<sensor_msgs::JointState>("/robotis/rh_p12_rn/present_joint_states", 0);
  g_goal_joint_states_pub = nh.advertise<sensor_msgs::JointState>("/robotis/rh_p12_rn/goal_joint_states", 0);

  ros::Subscriber present_joint_states_sub = nh.subscribe("/robotis/present_joint_states", 5, presentJointStatesCallback);
  ros::Subscriber goal_joint_states_sub = nh.subscribe("/robotis/goal_joint_states", 5, goalJointStatesCallback);

  ros::spin();

  return 0;
}
