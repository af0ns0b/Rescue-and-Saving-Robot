#!/usr/bin/env python

import rospy # type: ignore
import math

from sensor_msgs.msg import NavSatFix # latitude, longitude, altitude, position covariance, position_covariance_type
from geometry_msgs.msg import Pose2D # x, y, theta
from std_msgs.msg import Float32, String # data 
from pyproj import Proj

global waypoint_x, waypoint_y, robot_x, robot_y
waypoint_x = 0.0
waypoint_y = 0.0
robot_x = 0.0
robot_y = 0.0
  
projection = Proj(proj="utm", zone="29", ellps='WGS84')

def receive_waypoint(waypoint_msg):
  # Extract the waypoint information from the message
  waypoint_info = waypoint_msg.data.split(',')
  name, lat, lon, alt = waypoint_info[1], waypoint_info[2], waypoint_info[3], waypoint_info[4]
  # Process the waypoint and control the robot
  print(f'Received waypoint: {name} at ({lat}, {lon}, {alt})')

# Callback functions
def robotGPS_CB(gpspose):
  global robot_x, robot_y
  robot_x, robot_y = projection(gpspose.longitude, gpspose.latitude)

def main():
  # Init ROS node
  print("Begin")
  rospy.init_node('waypoint_convert')

  # Publishers
  pose_pub = rospy.Publisher('robot_pose', Pose2D, queue_size = 10)
  robot_gps_pub = rospy.Publisher('robot_gps_pose', NavSatFix, queue_size = 10)
  robot_heading_pub = rospy.Publisher('robot_heading', Float32, queue_size = 10)

  # Subscribers
  robot_gps_sub = rospy.Subscriber('mavros/global_position/global', NavSatFix, robotGPS_CB)
  waypoint_sub = rospy.Subscriber('robot_waypoints', String, receive_waypoint)

  rate = rospy.Rate(25)
  
  while not rospy.is_shutdown():
    # Publish robot position in projection coordinate
    print("Begin 2")
    pose = Pose2D() #x, y, theta
    gpspose = NavSatFix() # latitude, longitude, altitude, position covariance, position_covariance_type
    pose.x = robot_x
    pose.y = robot_y
    pose.theta = math.atan2( (waypoint_y-robot_y), (waypoint_x-robot_x) ) + math.pi / 2.0
    if pose.theta > math.pi:
      pose.theta = pose.theta - 2.0 * math.pi
    pose_pub.publish(pose)
    
    # Publish robot position in GPS
    gpspose.longitude,gpspose.latitude = projection(pose.x,pose.y,inverse=True)
    robot_gps_pub.publish(gpspose)
    
    # Publish robot heading, 0-359 degrees, North: 0, East: 90
    heading_rad =-math.atan2( (waypoint_y-robot_y), (waypoint_x-robot_x) )
    if heading_rad < 0:
      heading_rad = heading_rad + 2.0 * math.pi
    heading = math.fabs(heading_rad) / math.pi * 180.0
    robot_heading_pub.publish(heading)

    rate.sleep()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
