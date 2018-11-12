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
   Desc:   Rviz display panel for controlling and debugging ROS applications
*/

#include <cstdio>

#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>

#include "rviz_visual_tools_gui2.h"

#include <std_msgs/Bool.h>
#include <ros/ros.h>


namespace rviz_visual_tools
{
RvizVisualToolsGui::RvizVisualToolsGui2(QWidget* parent) : rviz::Panel(parent)
{
  // Create a push button
  btn_simclose_ = new QPushButton(this);
  btn_simclose_->setText("Close Sim Gripper");
  connect(btn_simclose_, SIGNAL(clicked()), this, SLOT(simclose()));

  // Create a push button
  btn_simopen_ = new QPushButton(this);
  btn_simopen_->setText("Open Sim Gripper");
  connect(btn_simopen_, SIGNAL(clicked()), this, SLOT(simopen()));

  // Create a push button
  btn_realclose_ = new QPushButton(this);
  btn_realclose_->setText("Close Real Gripper");
  connect(btn_realclose_, SIGNAL(clicked()), this, SLOT(realclose()));

  // Create a push button
  btn_realopen_ = new QPushButton(this);
  btn_realopen_->setText("Open Real Gripper");
  connect(btn_realopen_, SIGNAL(clicked()), this, SLOT(realopen()));

  // Horizontal Layout
  auto* hlayout1 = new QHBoxLayout;
  hlayout1->addWidget(btn_simclose_);
  hlayout1->addWidget(btn_simopen_);
  hlayout1->addWidget(btn_realclose_);
  hlayout1->addWidget(btn_realopen_);

  // Verticle layout
  auto* layout = new QVBoxLayout;
  layout->addLayout(hlayout1);
  setLayout(layout);

  btn_simclose_->setEnabled(true);
  btn_simopen_->setEnabled(true);
  btn_realclose_->setEnabled(true);
  btn_realopen_->setEnabled(true);

}

void RvizVisualToolsGui2::simclose()
{
  gripper_.simclose();
}

void RvizVisualToolsGui2::simopen()
{
  gripper_.simopen();
}

void RvizVisualToolsGui2::realclose()
{
  gripper_.realclose();
}

void RvizVisualToolsGui2::realopen()
{
  gripper_.realopen();
}

// void RvizVisualToolsGui::save(rviz::Config config) const
// {
//   rviz::Panel::save(config);
// }

// void RvizVisualToolsGui::load(const rviz::Config& config)
// {
//   rviz::Panel::load(config);
// }
}  // end namespace rviz_visual_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_visual_tools::RvizVisualToolsGui2, rviz::Panel)
