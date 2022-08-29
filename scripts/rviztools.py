#!/usr/bin/env python

"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster

from random import random
from math import sin

server = None
menu_handler = MenuHandler()
br = None
counter = 0


def frameCallback(msg):
    global counter, br
    time = rospy.Time.now()
    br.sendTransform((0, 0, sin(counter / 140.0) * 1.0), (0, 0, 0, 1.0), time, "base_link", "moving_frame")
    # br.sendTransform((0, 0, 0, 0), (0, 0, 0, 1), "map", time, "base_link")
    counter += 1


def processFeedback(feedback):
    s = "Feedback from marker '" + feedback.marker_name
    s += "' / control '" + feedback.control_name + "'"

    mp = ""
    if feedback.mouse_point_valid:
        mp = " at " + str(feedback.mouse_point.x)
        mp += ", " + str(feedback.mouse_point.y)
        mp += ", " + str(feedback.mouse_point.z)
        mp += " in frame " + feedback.header.frame_id

    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo(s + ": button click" + mp + ".")
    elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        rospy.loginfo(s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + ".")
    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        rospy.loginfo(s + ": pose changed")
    # TODO
    #          << "\nposition = "
    #          << feedback.pose.position.x
    #          << ", " << feedback.pose.position.y
    #          << ", " << feedback.pose.position.z
    #          << "\norientation = "
    #          << feedback.pose.orientation.w
    #          << ", " << feedback.pose.orientation.x
    #          << ", " << feedback.pose.orientation.y
    #          << ", " << feedback.pose.orientation.z
    #          << "\nframe: " << feedback.header.frame_id
    #          << " time: " << feedback.header.stamp.sec << "sec, "
    #          << feedback.header.stamp.nsec << " nsec" )
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
        rospy.loginfo(s + ": mouse down" + mp + ".")
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        rospy.loginfo(s + ": mouse up" + mp + ".")
    server.applyChanges()


def alignMarker(feedback):
    pose = feedback.pose

    # pose.position.x = pose.position.x
    # pose.position.y = pose.position.y

    rospy.loginfo(feedback.marker_name + ": aligning position = " + str(feedback.pose.position.x) + "," + str(
        feedback.pose.position.y) + "," + str(feedback.pose.position.z) + " to " +
                  str(pose.position.x) + "," + str(pose.position.y) + "," + str(pose.position.z))

    server.setPose(feedback.marker_name, pose)
    server.applyChanges()


def rand(min_, max_):
    return min_ + random() * (max_ - min_)


def makeBox(msg):
    marker = Marker()

    marker.type = Marker.ARROW
    marker.scale.x = msg.scale * 1
    marker.scale.y = msg.scale * 0.1
    marker.scale.z = msg.scale * 0.1

    # time = rospy.Time.now()
    marker.pose.position.z = 1

    marker.pose.orientation.x = 2000
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = -2000
    marker.pose.orientation.w = 1

    marker.color.r = 1
    marker.color.g = 0
    marker.color.b = 0
    marker.color.a = 1.0

    return marker


def makeBoxControl(msg):
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(makeBox(msg))
    msg.controls.append(control)
    return control


def saveMarker(int_marker):
    server.insert(int_marker, processFeedback)


#####################################################################
# Marker Creation

def normalizeQuaternion(quaternion_msg):
    norm = quaternion_msg.x ** 2 + quaternion_msg.y ** 2 + quaternion_msg.z ** 2 + quaternion_msg.w ** 2
    s = norm ** (-0.5)
    quaternion_msg.x *= s
    quaternion_msg.y *= s
    quaternion_msg.z *= s
    quaternion_msg.w *= s


def makeChessPieceMarker(position):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.pose.position = position
    int_marker.scale = 1

    int_marker.name = "landmark"
    int_marker.description = "landmark ID\nlandmark pose"

    move = InteractiveMarkerControl()
    move.orientation.w = 1
    move.orientation.x = 0
    move.orientation.y = 1
    move.orientation.z = 0
    normalizeQuaternion(move.orientation)
    move.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    move.always_visible = False
    int_marker.controls.append(copy.deepcopy(move))

    # make a box which also moves in the plane
    move.markers.append(makeBox(int_marker))
    int_marker.controls.append(move)

    menu = InteractiveMarkerControl()
    menu.interaction_mode = InteractiveMarkerControl.MENU
    int_marker.controls.append(copy.deepcopy(menu))

    marker = makeBox(int_marker)
    menu.markers.append(marker)
    menu.always_visible = True
    int_marker.controls.append(menu)

    server.insert(int_marker, processFeedback)
    menu_handler.apply(server, int_marker.name)
    # we want to use our special callback function
    # set different callback for POSE_UPDATE feedback
    server.setCallback(int_marker.name, alignMarker, InteractiveMarkerFeedback.POSE_UPDATE)


if __name__ == "__main__":
    rospy.init_node("basic_controls")
    br = TransformBroadcaster()
    menu_handler.insert("First Entry", callback=processFeedback)
    menu_handler.insert("Second Entry", callback=processFeedback)
    sub_menu_handle = menu_handler.insert("Submenu")
    menu_handler.insert("First Entry", parent=sub_menu_handle, callback=processFeedback)
    menu_handler.insert("Second Entry", parent=sub_menu_handle, callback=processFeedback)

    # create a timer to update the published transforms
    rospy.Timer(rospy.Duration(0.01), frameCallback)

    server = InteractiveMarkerServer("basic_controls")

    position = Point(3, -3, 0)
    makeChessPieceMarker(position)

    server.applyChanges()

    rospy.spin()
