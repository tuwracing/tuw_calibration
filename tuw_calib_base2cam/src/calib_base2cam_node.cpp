/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2017 by Florian Beck <florian.beck@tuwien.ac.at>        *
 *                                                                         *
 *   Redistribution and use in source and binary forms, with or without    *
 *   modification, are permitted provided that the following conditions    *
 *   are met:                                                              *
 *                                                                         *
 *   1. Redistributions of source code must retain the above copyright     *
 *      notice, this list of conditions and the following disclaimer.      *
 *   2. Redistributions in binary form must reproduce the above copyright  *
 *      notice, this list of conditions and the following disclaimer in    *
 *      the documentation and/or other materials provided with the         *
 *      distribution.                                                      *
 *   3. Neither the name of the copyright holder nor the names of its      *
 *      contributors may be used to endorse or promote products derived    *
 *      from this software without specific prior written permission.      *
 *                                                                         *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   *
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     *
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     *
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        *
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  *
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  *
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      *
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY *
 *   WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           *
 *   POSSIBILITY OF SUCH DAMAGE.                                           *
 ***************************************************************************/

#include <math.h>
#include "calib_base2cam_node.h"

Base2CamNode::Base2CamNode() : nh_private_("~")
{
  tf_listener_ = std::make_shared< tf::TransformListener >();
  tf_broadcaster_ = std::make_shared< tf::TransformBroadcaster >();
  nh_private_.param("camera_link", camera_link_, std::string("/camera_link"));
  nh_private_.param("base_link", base_link_, std::string("/base_link"));
  nh_private_.param("checkerboard_frame", checkerboard_frame_, std::string("/checkerboard"));
  nh_private_.param("corner_frame", corner_frame_, std::string("/corner"));
  nh_private_.param("laser_height", laser_height_, 0.3);
  nh_private_.param("checker_height", checker_height_, 1.295);
  nh_private_.param("checker_y", checker_y_, 0.475);
  checker_z_ = checker_height_ - laser_height_;
  nh_private_.param("rotate_camera_image_180", rotate_180_, false);
  nh_private_.param("publish_all_tf", publish_all_tf_, true);
  calibHistory_.clear();
}

static void avgOf(std::deque< tf::Transform > tfs, tf::Transform& averaged)
{
  double x = 0.0, y = 0.0, z = 0.0;
  for (tf::Transform& tf : tfs)
  {
    x += tf.getOrigin().getX();
    y += tf.getOrigin().getY();
    z += tf.getOrigin().getZ();
  }
  tf::Vector3 origin(x / tfs.size(), y / tfs.size(), z / tfs.size());
  averaged.setOrigin(origin);
  averaged.setRotation(tfs.front().getRotation());
}

void Base2CamNode::getBase2CamTf()
{
  // create corner to checkerboard tf
  tf::Transform corner2checker;
  corner2checker.setOrigin(tf::Vector3(0, checker_y_, checker_z_));
  tf::Quaternion q;

  // align to checkerboard coordinates
  q.setRPY(0, -M_PI / 2, M_PI);

  corner2checker.setRotation(q);

  if (publish_all_tf_)
    // clang-format off
    tf_broadcaster_->sendTransform(tf::StampedTransform(
      corner2checker,
      ros::Time::now(),
      corner_frame_,
      "checkerboardVIS"));
  // clang-format on

  tf::StampedTransform checker2camlink;
  try
  {
    tf_listener_->lookupTransform(checkerboard_frame_, camera_link_, ros::Time(0), checker2camlink);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  if (publish_all_tf_)
    // clang-format off
    tf_broadcaster_->sendTransform(tf::StampedTransform(
      checker2camlink,
      ros::Time::now(),
      "checkerboardVIS",
      "camlinkVIS"));
  // clang-format on

  tf::StampedTransform base2corner;

  try
  {
    tf_listener_->lookupTransform(base_link_, corner_frame_, ros::Time(0), base2corner);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  if (publish_all_tf_)
    tf_broadcaster_->sendTransform(tf::StampedTransform(base2corner, ros::Time::now(), base_link_, "cornerVIS"));

  tf::Transform checker2cam;
  tf::Quaternion qRealCamera;

  // rotate resulting frame towards REAL camera frame (actual zed mounting frame) if cb is aligned with camera frame
  qRealCamera.setRPY(-M_PI / 2, -M_PI / 2, 0);
  checker2cam.setOrigin(checker2camlink.getOrigin());
  checker2cam.setRotation(checker2camlink.getRotation() * qRealCamera);

  // transform from the camera position in world coordinates to the view frame coordinates
  tf::Transform cam2frame;
  cam2frame.setOrigin(tf::Vector3(0, 0, 0));
  tf::Quaternion qFrame;
  qFrame.setRPY(M_PI / 2, M_PI, M_PI / 2);
  cam2frame.setRotation(qFrame);

  tf::Transform base2cam = base2corner * corner2checker * checker2cam;

  tf::Vector3 origin = base2cam.getOrigin();
  tf::Quaternion rotation = base2cam.getRotation();

  if (rotate_180_)
  {
    tf::Quaternion flip;
    flip.setRPY(M_PI, 0, 0);
    rotation = rotation * flip;
    base2cam.setRotation(rotation);
  }

  if (publish_all_tf_)
  {
    tf_broadcaster_->sendTransform(tf::StampedTransform(base2cam, ros::Time::now(), base_link_, "camVIS"));
  }

  // average a history of calibration values
  if (calibHistory_.size() >= 100)
  {
    calibHistory_.pop_front();
  }
  calibHistory_.push_back(base2cam);
  tf::Transform avgCalib;
  avgOf(calibHistory_, avgCalib);
  tf_broadcaster_->sendTransform(tf::StampedTransform(avgCalib, ros::Time::now(), base_link_, "edge/zed/left_cam"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tuw_calib_base2cam");

  Base2CamNode base2cam_node;
  ros::Rate rate(10.0);
  while (base2cam_node.nh_.ok())
  {
    base2cam_node.getBase2CamTf();
  }

  return 0;
}
