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

#include "laser2corner_node.h"

#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

using std::vector;
using std::string;

Laser2CornerNode::Laser2CornerNode() : nh_private_("~")
{
  tf_broadcaster_ = std::make_shared< tf::TransformBroadcaster >();
  sub_segments_ = nh_.subscribe("line_segments", 1000, &Laser2CornerNode::callbackSegments, this);
  pub_marker_ = nh_.advertise< visualization_msgs::Marker >("line_segments_marker", 10);

  nh_private_.param("corner_point_tolerance", corner_point_tolerance_, 0.005);
  nh_private_.param("corner_point_x", corner_point_x_, 1.0);
  nh_private_.param("corner_point_y", corner_point_y_, -1.0);
}

void Laser2CornerNode::callbackSegments(const tuw_geometry_msgs::LineSegments& _segments_msg)
{
  // assume corner lies near (corner_point_x_, corner_point_y_, 0) wrt laser_base
  tuw::Point2D pc(corner_point_x_, corner_point_y_);

  linesegments_.resize(_segments_msg.segments.size());

  if (linesegments_.size() < 2)
  {
    ROS_WARN("need at least two lines for corner");
    return;
  }

  for (int i = 0; i < linesegments_.size(); i++)
  {
    linesegments_[i].set(_segments_msg.segments[i].p0.x,
                         _segments_msg.segments[i].p0.y,
                         _segments_msg.segments[i].p1.x,
                         _segments_msg.segments[i].p1.y);
  }

  // search 2 lines nearest to this point and copy lines to member variable
  double closest_dist_1 = linesegments_[0].distanceTo(pc);
  double closest_dist_2 = linesegments_[1].distanceTo(pc);
  int closest_idx_1 = 0;
  int closest_idx_2 = 1;

  if (closest_dist_1 > closest_dist_2)
  {
    closest_dist_1 = linesegments_[1].distanceTo(pc);
    closest_dist_2 = linesegments_[0].distanceTo(pc);
    closest_idx_1 = 1;
    closest_idx_2 = 0;
  }

  double tmp_dist;
  for (int i = 2; i < linesegments_.size(); i++)
  {
    tmp_dist = linesegments_[i].distanceTo(pc);
    if (!std::isfinite(closest_dist_1) || tmp_dist < closest_dist_1)
    {
      // std::cout << tmp_dist << " vs " << closest_dist_1 << std::endl;
      closest_dist_1 = tmp_dist;
      closest_idx_1 = i;
    }
    else if (!std::isfinite(closest_dist_2) || tmp_dist < closest_dist_2)
    {
      // std::cout << tmp_dist << " vs " << closest_dist_2 << std::endl;
      closest_dist_2 = tmp_dist;
      closest_idx_2 = i;
    }
  }

  // find corner point (where linesegments meet)
  tuw::Point2D corner_point;
  bool cornerFound = false;

  tuw::Point2D p10 = linesegments_[closest_idx_1].p0();
  tuw::Point2D p11 = linesegments_[closest_idx_1].p1();
  tuw::Point2D p20 = linesegments_[closest_idx_2].p0();
  tuw::Point2D p21 = linesegments_[closest_idx_2].p1();

  std::cout << "p10 " << p10 << std::endl;
  std::cout << "p11 " << p11 << std::endl;
  std::cout << "p20 " << p20 << std::endl;
  std::cout << "p21 " << p21 << std::endl;

  if (p10.equal(p20, corner_point_tolerance_) || p10.equal(p21, corner_point_tolerance_))
  {
    std::cout << "cornercase 1" << std::endl;
    cornerFound = true;
    corner_point = p10;
  }
  else if (p11.equal(p20, corner_point_tolerance_) || p11.equal(p21, corner_point_tolerance_))
  {
    std::cout << "cornercase 2" << std::endl;
    cornerFound = true;
    corner_point = p11;
  }

  double corner_yaw;  // rotation around z axis

  // use middle of line segments to check which is upfront and which on the side
  tuw::Point2D middle_1 = linesegments_[closest_idx_1].pc();
  tuw::Point2D middle_2 = linesegments_[closest_idx_2].pc();

  // publish center of selected line segments as marker
  visualization_msgs::Marker centers;
  centers.header.frame_id = _segments_msg.header.frame_id;
  centers.header.stamp = ros::Time::now();
  centers.id = 0;
  centers.ns = "linesegment_centers";
  centers.type = visualization_msgs::Marker::POINTS;
  centers.action = visualization_msgs::Marker::ADD;
  centers.scale.x = 0.1;
  centers.scale.y = 0.1;
  centers.color.g = 1.0;
  centers.color.a = 1.0;
  geometry_msgs::Point p;
  p.x = middle_1.x();
  p.y = middle_1.y();
  centers.points.push_back(p);
  p.x = middle_2.x();
  p.y = middle_2.y();
  centers.points.push_back(p);

  pub_marker_.publish(centers);

  // use angle from line on the side
  if (middle_1.x() > middle_2.x())
  {
    corner_yaw = linesegments_[closest_idx_2].angle();
  }
  else
  {
    corner_yaw = linesegments_[closest_idx_1].angle();
  }

  tf::Transform laser_to_corner;

  if (!cornerFound ||
      !(std::isfinite(corner_point.x()) && std::isfinite(corner_point.y()) && std::isfinite(corner_yaw)))
  {
    // likely to happen if line segment detection fails temporarily
    // TODO use avg or history of tf instead
    laser_to_corner.setOrigin(lastTransform_.getOrigin());
    laser_to_corner.setRotation(lastTransform_.getRotation());
  }
  else
  {
    tf::Vector3 T = tf::Vector3(corner_point.x(), corner_point.y(), 0);
    tf::Quaternion Q;
    Q.setRPY(0, 0, corner_yaw);

    laser_to_corner.setOrigin(T);
    laser_to_corner.setRotation(Q);
  }

  tf_broadcaster_->sendTransform(
      tf::StampedTransform(laser_to_corner, ros::Time::now(), _segments_msg.header.frame_id, "corner"));
  lastTransform_ = laser_to_corner;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tuw_laser2corner");

  Laser2CornerNode laser2corner_node;

  ros::spin();

  return 0;
}
