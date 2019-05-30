/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <rclcpp/rclcpp.hpp>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_broadcaster.h>

#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <math.h>

#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

// %Tag(vars)%
rclcpp::Node::SharedPtr node = nullptr;
std::shared_ptr<interactive_markers::InteractiveMarkerServer> server = nullptr;
std::shared_ptr<interactive_markers::MenuHandler> menu_handler = nullptr;
std::shared_ptr<tf2_ros::TransformBroadcaster> br = nullptr;
// %EndTag(vars)%


// %Tag(Box)%
visualization_msgs::msg::Marker makeBox( visualization_msgs::msg::InteractiveMarker &msg )
{
  visualization_msgs::msg::Marker marker;

  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

visualization_msgs::msg::InteractiveMarkerControl& makeBoxControl( visualization_msgs::msg::InteractiveMarker &msg )
{
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}
// %EndTag(Box)%

// %Tag(frameCallback)%
void frameCallback()
{
  static uint32_t counter = 0;

  tf2::Stamped<tf2::Transform> t;
  geometry_msgs::msg::TransformStamped t_msg;

  t_msg.header.frame_id = "base_link";
  t_msg.header.stamp = node->now();

  t.setOrigin(tf2::Vector3(0.0, 0.0, sin(float(counter)/140.0) * 2.0));
  t.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  t_msg.transform = tf2::toMsg(t);
  t_msg.child_frame_id = "moving_frame";
  br->sendTransform(t_msg);

  t.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  tf2::Quaternion q;
  q.setRPY(0.0, float(counter)/140.0, 0.0);
  t.setRotation(q);
  t_msg.transform = tf2::toMsg(t);
  t_msg.child_frame_id = "rotating_frame";

  br->sendTransform(t_msg);

  counter++;
}
// %EndTag(frameCallback)%

// %Tag(processFeedback)%
void processFeedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback )
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  switch ( feedback->event_type )
  {
    case visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK:
      RCLCPP_INFO(node->get_logger(), "%s: button click%s.", s.str().c_str(), mouse_point_ss.str().c_str() );
      break;

    case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
      RCLCPP_INFO(node->get_logger(), "%s: menu item %u clicked%s.", s.str().c_str(), feedback->menu_entry_id, mouse_point_ss.str());
      break;

    case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
    {
      std::ostringstream debug_str;
      debug_str << s.str() << ": pose changed"
          << "\nposition = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << "\norientation = "
          << feedback->pose.orientation.w
          << ", " << feedback->pose.orientation.x
          << ", " << feedback->pose.orientation.y
          << ", " << feedback->pose.orientation.z
          << "\nframe: " << feedback->header.frame_id
          << " time: " << feedback->header.stamp.sec << "sec, "
          << feedback->header.stamp.nanosec << " nsec";
        RCLCPP_INFO(node->get_logger(), debug_str.str().c_str());
      break;
    }
    case visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_DOWN:
      RCLCPP_INFO(node->get_logger(), "%s: mouse down%s.", s.str().c_str(), mouse_point_ss.str().c_str() );
      break;

    case visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP:
      RCLCPP_INFO(node->get_logger(), "%s: mouse up%s.", s.str().c_str(), mouse_point_ss.str().c_str() );
      break;
  }

  server->applyChanges();
}
// %EndTag(processFeedback)%

// %Tag(alignMarker)%
void alignMarker( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback )
{
  geometry_msgs::msg::Pose pose = feedback->pose;

  pose.position.x = round(pose.position.x-0.5)+0.5;
  pose.position.y = round(pose.position.y-0.5)+0.5;

  std::ostringstream debug_str;
  debug_str << feedback->marker_name << ":"
      << " aligning position = "
      << feedback->pose.position.x
      << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z
      << " to "
      << pose.position.x
      << ", " << pose.position.y
      << ", " << pose.position.z;
  
  RCLCPP_INFO(node->get_logger(), debug_str.str().c_str());

  server->setPose( feedback->marker_name, pose );
  server->applyChanges();
}
// %EndTag(alignMarker)%

double rand( double min, double max )
{
  double t = (double)rand() / (double)RAND_MAX;
  return min + t*(max-min);
}

////////////////////////////////////////////////////////////////////////////////////
void pointTFToMsg(const tf2::Vector3& point, geometry_msgs::msg::Point& msg) {
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
}

void quaternionTFToMsg(const tf2::Quaternion& q, geometry_msgs::msg::Quaternion& msg) {
  msg.w = q.w();
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
}

// %Tag(6DOF)%
void make6DofMarker( bool fixed, unsigned int interaction_mode, const tf2::Vector3& position, bool show_6dof )
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";

  pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "simple_6dof";
  int_marker.description = "Simple 6-DOF Control";

  // insert a box
  makeBoxControl(int_marker);
  int_marker.controls[0].interaction_mode = interaction_mode;

  visualization_msgs::msg::InteractiveMarkerControl control;

  if ( fixed )
  {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  }

  if (interaction_mode != visualization_msgs::msg::InteractiveMarkerControl::NONE)
  {
      std::string mode_text;
      if( interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
      if( interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
      if( interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
      int_marker.name += "_" + mode_text;
      int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
  }

  if(show_6dof)
  {
    tf2::Quaternion orien(1.0, 0.0, 0.0, 1.0);
    orien.normalize();
    quaternionTFToMsg(orien, control.orientation);
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    orien = tf2::Quaternion(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    quaternionTFToMsg(orien, control.orientation);
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    orien = tf2::Quaternion(0.0, 0.0, 1.0, 1.0);
    orien.normalize();
    quaternionTFToMsg(orien, control.orientation);
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  if (interaction_mode != visualization_msgs::msg::InteractiveMarkerControl::NONE)
    menu_handler->apply( *server, int_marker.name );
}
// %EndTag(6DOF)%

// %Tag(RandomDof)%
void makeRandomDofMarker( const tf2::Vector3& position )
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "6dof_random_axes";
  int_marker.description = "6-DOF\n(Arbitrary Axes)";

  makeBoxControl(int_marker);

  visualization_msgs::msg::InteractiveMarkerControl control;

  for ( int i=0; i<3; i++ )
  {
    tf2::Quaternion orien(rand(-1,1), rand(-1,1), rand(-1,1), rand(-1,1));
    orien.normalize();
    quaternionTFToMsg(orien, control.orientation);
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(RandomDof)%


// %Tag(ViewFacing)%
void makeViewFacingMarker( const tf2::Vector3& position )
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "view_facing";
  int_marker.description = "View Facing 6-DOF";

  visualization_msgs::msg::InteractiveMarkerControl control;

  // make a control that rotates around the view axis
  control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation.w = 1;
  control.name = "rotate";

  int_marker.controls.push_back(control);

  // create a box in the center which should not be view facing,
  // but move in the camera plane.
  control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
  control.independent_marker_orientation = true;
  control.name = "move";

  control.markers.push_back( makeBox(int_marker) );
  control.always_visible = true;

  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(ViewFacing)%


// %Tag(Quadrocopter)%
void makeQuadrocopterMarker( const tf2::Vector3& position )
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "quadrocopter";
  int_marker.description = "Quadrocopter";

  makeBoxControl(int_marker);

  visualization_msgs::msg::InteractiveMarkerControl control;

  tf2::Quaternion orien(0.0, 1.0, 0.0, 1.0);
  orien.normalize();
  quaternionTFToMsg(orien, control.orientation);
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE;
  int_marker.controls.push_back(control);
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(Quadrocopter)%

// %Tag(ChessPiece)%
void makeChessPieceMarker( const tf2::Vector3& position )
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "chess_piece";
  int_marker.description = "Chess Piece\n(2D Move + Alignment)";

  visualization_msgs::msg::InteractiveMarkerControl control;

  tf2::Quaternion orien(0.0, 1.0, 0.0, 1.0);
  orien.normalize();
  quaternionTFToMsg(orien, control.orientation);
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
  int_marker.controls.push_back(control);

  // make a box which also moves in the plane
  control.markers.push_back( makeBox(int_marker) );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  // we want to use our special callback function
  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);

  // set different callback for POSE_UPDATE feedback
  server->setCallback(int_marker.name, &alignMarker, visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE );
}
// %EndTag(ChessPiece)%

// %Tag(PanTilt)%
void makePanTiltMarker( const tf2::Vector3& position )
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "pan_tilt";
  int_marker.description = "Pan / Tilt";

  makeBoxControl(int_marker);

  visualization_msgs::msg::InteractiveMarkerControl control;

  tf2::Quaternion orien(0.0, 1.0, 0.0, 1.0);
  orien.normalize();
  quaternionTFToMsg(orien, control.orientation);
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  int_marker.controls.push_back(control);

  orien = tf2::Quaternion(0.0, 0.0, 1.0, 1.0);
  orien.normalize();
  quaternionTFToMsg(orien, control.orientation);
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::INHERIT;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(PanTilt)%

// %Tag(Menu)%
void makeMenuMarker( const tf2::Vector3& position )
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "context_menu";
  int_marker.description = "Context Menu\n(Right Click)";

  visualization_msgs::msg::InteractiveMarkerControl control;

  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MENU;
  control.name = "menu_only_control";

  visualization_msgs::msg::Marker marker = makeBox( int_marker );
  control.markers.push_back( marker );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  menu_handler->apply( *server, int_marker.name );
}
// %EndTag(Menu)%

// %Tag(Button)%
void makeButtonMarker( const tf2::Vector3& position )
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "button";
  int_marker.description = "Button\n(Left Click)";

  visualization_msgs::msg::InteractiveMarkerControl control;

  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  control.name = "button_control";

  visualization_msgs::msg::Marker marker = makeBox( int_marker );
  control.markers.push_back( marker );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(Button)%

// %Tag(Moving)%
void makeMovingMarker( const tf2::Vector3& position )
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "moving_frame";
  pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "moving";
  int_marker.description = "visualization_msgs::msg::Marker Attached to a\nMoving Frame";

  visualization_msgs::msg::InteractiveMarkerControl control;

  tf2::Quaternion orien(1.0, 0.0, 0.0, 1.0);
  orien.normalize();
  quaternionTFToMsg(orien, control.orientation);
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
  control.always_visible = true;
  control.markers.push_back( makeBox(int_marker) );
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(Moving)%

// %Tag(main)%
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>("basic_controls", "");
  br = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  menu_handler = std::make_shared<interactive_markers::MenuHandler>(node);

  // create a timer to update the published transforms
  auto frame_timer = node->create_wall_timer(std::chrono::milliseconds(10), frameCallback);

  server.reset( new interactive_markers::InteractiveMarkerServer(node, "basic_controls","",false) );

  rclcpp::sleep_for(std::chrono::milliseconds(500));

  menu_handler->insert( "First Entry", &processFeedback );
  menu_handler->insert( "Second Entry", &processFeedback );
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler->insert( "Submenu" );
  menu_handler->insert( sub_menu_handle, "First Entry", &processFeedback );
  menu_handler->insert( sub_menu_handle, "Second Entry", &processFeedback );

  tf2::Vector3 position;
  position = tf2::Vector3(-3, 3, 0);
  make6DofMarker( false, visualization_msgs::msg::InteractiveMarkerControl::NONE, position, true );
  position = tf2::Vector3( 0, 3, 0);
  make6DofMarker( true, visualization_msgs::msg::InteractiveMarkerControl::NONE, position, true );
  position = tf2::Vector3( 3, 3, 0);
  makeRandomDofMarker( position );
  position = tf2::Vector3(-3, 0, 0);
  make6DofMarker( false, visualization_msgs::msg::InteractiveMarkerControl::ROTATE_3D, position, false );
  position = tf2::Vector3( 0, 0, 0);
  make6DofMarker( false, visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D, position, true );
  position = tf2::Vector3( 3, 0, 0);
  make6DofMarker( false, visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D, position, false );
  position = tf2::Vector3(-3,-3, 0);
  makeViewFacingMarker( position );
  position = tf2::Vector3( 0,-3, 0);
  makeQuadrocopterMarker( position );
  position = tf2::Vector3( 3,-3, 0);
  makeChessPieceMarker( position );
  position = tf2::Vector3(-3,-6, 0);
  makePanTiltMarker( position );
  position = tf2::Vector3( 0,-6, 0);
  makeMovingMarker( position );
  position = tf2::Vector3( 3,-6, 0);
  makeMenuMarker( position );
  position = tf2::Vector3( 0,-9, 0);
  makeButtonMarker( position );

  server->applyChanges();

  rclcpp::spin(node);

  server.reset();
}
// %EndTag(main)%
