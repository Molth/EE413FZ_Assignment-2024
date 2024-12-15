#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import math

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

# Specification
color = (0.635, 0, 0)
# Global Configuration
num_points = 100
# Initialization
rospy.init_node("draw_logo_node", anonymous = True)
robot = moveit_commander.RobotCommander()
# Moveit Initialization
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("ur5_arm")
# Marker Publisher Initialization
marker_publisher = rospy.Publisher("/visualization_marker", Marker, queue_size = 10)

def create_pose(point):
    pose = group.get_current_pose().pose
    pose.position.x = point[0]
    pose.position.y = point[1]
    pose.position.z = point[2]
    return pose

def execute_trajectory(waypoints):
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)
    if fraction == 1.0:
        print("Full trajectory computed!")
        group.execute(plan, wait=True)
    else:
        print("Could not compute full trajectory. Only " + str(fraction * 100) + "%" + "achieved.")

def circle_trajectory(center, radius):
    waypoints = []
    for i in range(num_points + 1):
        theta = 2 * math.pi * i / num_points
        x = center[0] + radius * math.cos(theta)
        y = center[1] + radius * math.sin(theta)
        z = center[2]
        pose = group.get_current_pose().pose
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        waypoints.append(pose)
        publish_marker(pose.position)
    return waypoints

def square_trajectory(center, length):
    waypoints = []
    step_width = length / num_points
    for i in range(num_points):
        x = center[0] - length / 2 + i * step_width
        y = center[1] + length / 2
        z = center[2]
        pose = group.get_current_pose().pose
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        waypoints.append(pose)
        publish_marker(pose.position)
    for i in range(num_points):
        x = center[0] + length / 2
        y = center[1] - length / 2 + i * step_width
        z = center[2]
        pose = group.get_current_pose().pose
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        waypoints.append(pose)
        publish_marker(pose.position)
    for i in range(num_points):
        x = center[0] + length / 2 - i * step_width
        y = center[1] - length / 2
        z = center[2]
        pose = group.get_current_pose().pose
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        waypoints.append(pose)
        publish_marker(pose.position)
    for i in range(num_points):
        x = center[0] - length / 2
        y = center[1] + length / 2 - i * step_width
        z = center[2]
        pose = group.get_current_pose().pose
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        waypoints.append(pose)
        publish_marker(pose.position)
    return waypoints

def rectangle_trajectory(center, width, height):
    waypoints = []
    step_width = width / num_points
    step_height = height / num_points
    for i in range(num_points):
        x = center[0] - width / 2 + i * step_width
        y = center[1] + height / 2
        z = center[2]
        pose = group.get_current_pose().pose
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        waypoints.append(pose)
        publish_marker(pose.position)
    for i in range(num_points):
        x = center[0] + width / 2
        y = center[1] - height / 2 + i * step_height
        z = center[2]
        pose = group.get_current_pose().pose
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        waypoints.append(pose)
        publish_marker(pose.position)
    for i in range(num_points):
        x = center[0] + width / 2 - i * step_width
        y = center[1] - height / 2
        z = center[2]
        pose = group.get_current_pose().pose
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        waypoints.append(pose)
        publish_marker(pose.position)
    for i in range(num_points):
        x = center[0] - width / 2
        y = center[1] + height / 2 - i * step_height
        z = center[2]
        pose = group.get_current_pose().pose
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        waypoints.append(pose)
        publish_marker(pose.position)
    return waypoints

def diagonal_rectangle_trajectory(center, width, height, angle):
    waypoints = []
    step_width = width / num_points
    step_height = height / num_points
    cos_theta = math.cos(math.radians(angle))
    sin_theta = math.sin(math.radians(angle))
    for i in range(num_points):
        x = center[0] - width / 2 + i * step_width
        y = center[1] + height / 2
        x_rot = cos_theta * x - sin_theta * y
        y_rot = sin_theta * x + cos_theta * y
        z = center[2]
        pose = group.get_current_pose().pose
        pose.position.x = x_rot
        pose.position.y = y_rot
        pose.position.z = z
        waypoints.append(pose)
        publish_marker(pose.position)
    for i in range(num_points):
        x = center[0] + width / 2
        y = center[1] - height / 2 + i * step_height
        x_rot = cos_theta * x - sin_theta * y
        y_rot = sin_theta * x + cos_theta * y
        z = center[2]
        pose = group.get_current_pose().pose
        pose.position.x = x_rot
        pose.position.y = y_rot
        pose.position.z = z
        waypoints.append(pose)
        publish_marker(pose.position)
    for i in range(num_points):
        x = center[0] + width / 2 - i * step_width
        y = center[1] - height / 2
        x_rot = cos_theta * x - sin_theta * y
        y_rot = sin_theta * x + cos_theta * y
        z = center[2]
        pose = group.get_current_pose().pose
        pose.position.x = x_rot
        pose.position.y = y_rot
        pose.position.z = z
        waypoints.append(pose)
        publish_marker(pose.position)
    for i in range(num_points):
        x = center[0] - width / 2
        y = center[1] + height / 2 - i * step_height
        x_rot = cos_theta * x - sin_theta * y
        y_rot = sin_theta * x + cos_theta * y
        z = center[2]
        pose = group.get_current_pose().pose
        pose.position.x = x_rot
        pose.position.y = y_rot
        pose.position.z = z
        waypoints.append(pose)
        publish_marker(pose.position)
    return waypoints

def line(point1, point2):
    waypoints = []
    x_step = (point2[0] - point1[0]) / num_points
    y_step = (point2[1] - point1[1]) / num_points
    for i in range(num_points + 1):
        x = point1[0] + i * x_step
        y = point1[1] + i * y_step
        z = 0
        pose = group.get_current_pose().pose
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        waypoints.append(pose)
        publish_marker(pose.position)
    return waypoints

def semicircle_line(point1, point2, direction):
    center_x = (point1[0] + point2[0]) / 2
    center_y = (point1[1] + point2[1]) / 2
    radius = math.sqrt((point1[0] - center_x) ** 2 + (point1[1] - center_y) ** 2)
    waypoints = []
    for i in range(num_points):
        theta = math.pi * i / (num_points - 1) * direction
        x = center_x + radius * math.cos(theta)
        y = center_y + radius * math.sin(theta)
        z = 0
        pose = group.get_current_pose().pose
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        waypoints.append(pose)
        publish_marker(pose.position)
    return waypoints

def publish_marker(point, marker_id):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.025
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 1.0
    marker.id = marker_id
    marker.pose.orientation.w = 1.0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.points.append(point)
    marker_publisher.publish(marker)

def draw_fu():
    execute_trajectory(circle_trajectory([0, 0, 0], 1200 / 1000.0))
    execute_trajectory(rectangle_trajectory([200 / 1000.0, 275 / 1000.0, 0], 220 / 1000.0, 50 / 1000.0))
    execute_trajectory(rectangle_trajectory([200 / 1000.0, 100 / 1000.0, 0], 220 / 1000.0, 220 / 1000.0))
    execute_trajectory(rectangle_trajectory([200 / 1000.0, -200 / 1000.0, 0], 280 / 1000.0, 280 / 1000.0))
    execute_trajectory(rectangle_trajectory([130 / 1000.0, -130 / 1000.0, 0], 140 / 1000.0, 140 / 1000.0))
    execute_trajectory(rectangle_trajectory([270 / 1000.0, -270 / 1000.0, 0], 140 / 1000.0, 140 / 1000.0))
    execute_trajectory(rectangle_trajectory([-200 / 1000.0, 270 / 1000.0, 0], 100 / 1000.0, 70 / 1000.0))
    execute_trajectory(rectangle_trajectory([-200 / 1000.0, 160 / 1000.0, 0], 300 / 1000.0, 50 / 1000.0))
    execute_trajectory(rectangle_trajectory([-200 / 1000.0, -105 / 1000.0, 0], 100 / 1000.0, 480 / 1000.0))
    execute_trajectory(rectangle_trajectory([-320 / 1000.0, -30 / 1000.0, 0], 50 / 1000.0, 200 / 1000.0))
    execute_trajectory(rectangle_trajectory([-80 / 1000.0, -30 / 1000.0, 0], 50 / 1000.0, 200 / 1000.0))

def draw_f():
    execute_trajectory(line([-330 / 1000.0, 190 / 1000.0, 0], [-150 / 1000.0, 190 / 1000.0, 0]))
    execute_trajectory(line([-330 / 1000.0, 70 / 1000.0, 0], [-150 / 1000.0, 70 / 1000.0, 0]))
    execute_trajectory(line([-330 / 1000.0, 190 / 1000.0, 0], [-330 / 1000.0, -220 / 1000.0, 0]))

def draw_z():
    execute_trajectory(line([-70 / 1000.0, 190 / 1000.0, 0], [130 / 1000.0, 190 / 1000.0, 0]))
    execute_trajectory(line([-70 / 1000.0,  -220 / 1000.0, 0], [130 / 1000.0,  -220 / 1000.0, 0]))
    execute_trajectory(line([-70 / 1000.0,  -220 / 1000.0, 0], [130 / 1000.0, 190 / 1000.0, 0]))

def draw_u():
    execute_trajectory(line([205 / 1000.0, 200 / 1000.0, 0], [205 / 1000.0, -130 / 1000.0, 0]))
    execute_trajectory(line([350 / 1000.0, 200 / 1000.0, 0], [350 / 1000.0, -130 / 1000.0, 0]))
    execute_trajectory(semicircle_line([205 / 1000.0, -130 / 1000.0, 0], [350 / 1000.0, -130 / 1000.0, 0], -1))

def draw_logo():
    execute_trajectory(circle_trajectory([0 - 400 / 1000.0, 0, 0], 400 / 1000.0), 0)
    execute_trajectory(rectangle_trajectory([200 / 1000 * 0.75 - 400 / 1000.0, 275 / 1000 * 0.75, 0], 220 / 1000 * 0.75, 50 / 1000 * 0.75))
    execute_trajectory(rectangle_trajectory([200 / 1000 * 0.75 - 400 / 1000.0, 100 / 1000 * 0.75, 0], 220 / 1000 * 0.75, 220 / 1000 * 0.75))
    execute_trajectory(rectangle_trajectory([200 / 1000 * 0.75 - 400 / 1000.0, -200 / 1000 * 0.75, 0], 280 / 1000 * 0.75, 280 / 1000 * 0.75))
    execute_trajectory(rectangle_trajectory([130 / 1000 * 0.75 - 400 / 1000.0, -130 / 1000 * 0.75, 0], 140 / 1000 * 0.75, 140 / 1000 * 0.75))
    execute_trajectory(rectangle_trajectory([270 / 1000 * 0.75 - 400 / 1000.0, -270 / 1000 * 0.75, 0], 140 / 1000 * 0.75, 140 / 1000 * 0.75))
    execute_trajectory(rectangle_trajectory([-200 / 1000 * 0.75 - 400 / 1000.0, 270 / 1000 * 0.75, 0], 100 / 1000 * 0.75, 70 / 1000 * 0.75))
    execute_trajectory(rectangle_trajectory([-200 / 1000 * 0.75 - 400 / 1000.0, 160 / 1000 * 0.75, 0], 300 / 1000 * 0.75, 50 / 1000 * 0.75))
    execute_trajectory(rectangle_trajectory([-200 / 1000 * 0.75 - 400 / 1000.0, -105 / 1000 * 0.75, 0], 100 / 1000 * 0.75, 480 / 1000 * 0.75))
    execute_trajectory(rectangle_trajectory([-320 / 1000 * 0.75 - 400 / 1000.0, -30 / 1000 * 0.75, 0], 50 / 1000 * 0.75, 200 / 1000 * 0.75))
    execute_trajectory(rectangle_trajectory([-80 / 1000 * 0.75 - 400 / 1000.0, -30 / 1000 * 0.75, 0], 50 / 1000 * 0.75, 200 / 1000 * 0.75))

    execute_trajectory(line([-330 / 1000 * 0.75 + 400 / 1000.0, 190 / 1000 * 0.75, 0], [-150 / 1000 * 0.75 + 400 / 1000.0, 190 / 1000 * 0.75, 0]))
    execute_trajectory(line([-330 / 1000 * 0.75 + 400 / 1000.0, 70 / 1000 * 0.75, 0], [-150 / 1000 * 0.75 + 400 / 1000.0, 70 / 1000 * 0.75, 0]))
    execute_trajectory(line([-330 / 1000 * 0.75 + 400 / 1000.0, 190 / 1000 * 0.75, 0], [-330 / 1000 * 0.75 + 400 / 1000.0, -220 / 1000 * 0.75, 0]))

    execute_trajectory(line([-70 / 1000 * 0.75 + 400 / 1000.0, 190 / 1000 * 0.75, 0], [130 / 1000 * 0.75 + 400 / 1000.0, 190 / 1000 * 0.75, 0]))
    execute_trajectory(line([-70 / 1000 * 0.75 + 400 / 1000.0,  -220 / 1000 * 0.75, 0], [130 / 1000 * 0.75 + 400 / 1000.0,  -220 / 1000 * 0.75, 0]))
    execute_trajectory(line([-70 / 1000 * 0.75 + 400 / 1000.0,  -220 / 1000 * 0.75, 0], [130 / 1000 * 0.75 + 400 / 1000.0, 190 / 1000 * 0.75, 0]))

    execute_trajectory(line([205 / 1000 * 0.75 + 400 / 1000.0, 200 / 1000 * 0.75, 0], [205 / 1000 * 0.75 + 400 / 1000.0, -130 / 1000 * 0.75, 0]))
    execute_trajectory(line([350 / 1000 * 0.75 + 400 / 1000.0, 200 / 1000 * 0.75, 0], [350 / 1000 * 0.75 + 400 / 1000.0, -130 / 1000 * 0.75, 0]))
    execute_trajectory(semicircle_line([205 / 1000 * 0.75 + 400 / 1000.0, -130 / 1000 * 0.75, 0], [350 / 1000 * 0.75 + 400 / 1000.0, -130 / 1000 * 0.75, 0], -1))

if __name__ == "__main__":
    current_pose = group.get_current_pose().pose.position
    while not rospy.is_shutdown():
        draw_logo()
        rospy.sleep(100) # Add a long delay to observe the drawing if needed

