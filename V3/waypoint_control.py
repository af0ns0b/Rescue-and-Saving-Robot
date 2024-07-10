#!/usr/bin/env python

import rospy # type: ignore
import math

from geometry_msgs.msg import Point, Pose2D, Twist # x,y,z / x,y,theta / linear,angular

# Variables
global goal_set, distance, angle, goal, vel

goal_set = False
distance = 0.0
angle = math.pi
vel = Twist()
vel.linear = 0.5
vel.angular = 0.5


# Callback functions
def goalCB(g):
  global goal_set, goal, substate
  goal = g
  goal_set = True


def poseCB(pose):
  global goal_set, distance, angle, goal
  if goal_set == True:
    dx = (goal.x-pose.x)*math.cos(pose.theta)+(goal.y-pose.y)*math.sin(pose.theta)
    dy = -(goal.x-pose.x)*math.sin(pose.theta)+(goal.y-pose.y)*math.cos(pose.theta)
    distance = math.sqrt( (dx)**2 + (dy)**2 )
    angle = math.atan2(dy,dx)


def main():
  # Init ROS node
  rospy.init_node('waypoint_control')

  # Publishers
  vel_pub = rospy.Publisher('/mavros/actuator_control', Twist, queue_size = 10)

  # Subscribers
  pose_sub = rospy.Subscriber('robot_pose', Pose2D, poseCB)
  goal_sub = rospy.Subscriber('robot_waypoints', Point, goalCB)

  rate = rospy.Rate(25)

  print ("Begin")

  while not rospy.is_shutdown():
    print ("Begin 2")
    if goal_set == True:
      if distance > 0.2:
        if angle > 0.2:
          vel.linear = 0.8
          vel.angular = 0.8
        if angle < -0.2:  
          vel.linear = 0.8
          vel.angular = 0.2
        if math.abs(angle) < 0.2:
          vel.linear = 0.8
          vel.angular = 0.5
      if distance < -0.2:
        if angle > 0.2:
          vel.linear = 0.2
          vel.angular = 0.8
        if angle < -0.2:
          vel.linear = 0.2
          vel.angular = 0.2
        if math.abs(angle) < 0.2:
          vel.linear = 0.2
          vel.angular = 0.5
      if math.abs(distance) < 0.2:
        vel.linear = 0.5
        vel.angular = 0.5

      vel_pub.publish(vel)

  # 0.5 - 1 ->velocidade positiva
  # 0 - 0.5 ->velocidade negativa
        
    rate.sleep()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
