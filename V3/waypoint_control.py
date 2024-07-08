#!/usr/bin/env python

import rospy # type: ignore
import math

from geometry_msgs.msg import Point, Pose2D, Twist # x,y,z / x,y,theta / linear,angular
from std_msgs.msg import String, Float32 # data

# Variables
global k1, k2, state, substate, prestate, goal_set, distance, angle, goal, vel, vel_maxlin, vel_maxang
k1 = 0.3
k2 = 0.8
state = "STOP"
substate = "STOP"
prestate = "STOP" 
goal_set = False
distance = 0.0
angle = math.pi
way = Point()
vel = Twist()
vel.linear = 0
vel.angular = 0
vel_maxlin = 1
vel_maxang = 2

# Callback functions
def goalCB(g):
  global goal_set, goal, substate
  goal = g
  goal_set = True
  substate = "STOP"

def paraCB(p):
  global k1, k2
  if len(p.data) > 0:
    parts = p.data.split(',')
    if len(parts) == 2:
      k1 = float(parts[0])
      k2 = float(parts[1])
      print ("Parameter updated:")
      print ("k1: " + str(k1))
      print ("k2: " + str(k2))
    else:
      print ("2 parameter needed, only " + str(len(parts)) + " sent")

def stateCB(s):
  global state
  state = s.data
  print ("Waypoint control state updated: " + state)

def poseCB(p):
  global goal_set, distance, angle, goal
  if goal_set:
    dx = (goal.x-p.x)*math.cos(p.theta)+(goal.y-p.y)*math.sin(p.theta)
    dy = -(goal.x-p.x)*math.sin(p.theta)+(goal.y-p.y)*math.cos(p.theta)
    distance = math.sqrt( (dx)**2 + (dy)**2 )
    angle = math.atan2(dy,dx)

def linCB(l):
  global vel_maxlin
  vel_maxlin = l.data
  print ("Max linear speed is set to: " + str(vel_maxlin))

def angCB(a):
  global vel_maxang
  vel_maxang = a.data
  print ("Max angular speed is set to: " + str(vel_maxang))

def main():
  global substate
  # Init ROS node
  rospy.init_node('waypoint_control')

  # Publishers
  vel_pub = rospy.Publisher('/mavros/actuator_control', Twist, queue_size = 10)
  state_pub = rospy.Publisher('waypoint/robot_state', String, queue_size = 10)

  # Subscribers
  state_sub = rospy.Subscriber('waypoint/state', String, stateCB)
  pose_sub = rospy.Subscriber('robot_pose', Pose2D, poseCB)
  goal_sub = rospy.Subscriber('waypoint/goal', Point, goalCB)
  para_sub = rospy.Subscriber('waypoint/control_parameters', String, paraCB)
  maxlin_sub = rospy.Subscriber('waypoint/max_linear_speed', Float32, linCB)
  maxang_sub = rospy.Subscriber('waypoint/max_angular_speed', Float32, angCB)

  rate = rospy.Rate(25)

  print ("Begin")

  while not rospy.is_shutdown():
    print ("Begin 2")
    if goal_set:
      if state == "RUNNING":
        if substate != "FORWARDING":
          substate = "TURNING"
        if substate == "TURNING":
          vel.linear = 0
          vel.angular = k2 * angle
          if math.abs(angle) < 0.2:
            substate = "FORWARDING"
        elif substate == "FORWARDING":
          if (angle > math.pi/2):
            vel.linear = -k1 * distance
            vel.angular = k2 * (angle-math.pi)
          elif (angle < -math.pi/2):
            vel.linear = -k1 * distance
            vel.angular = k2 * (angle+math.pi)
          else:
            vel.linear = k1 * distance
            vel.angular = k2 * angle

        if vel.linear > vel_maxlin:
          vel.linear = vel_maxlin
        if vel.angular > vel_maxang:
          vel.angular = vel_maxang

        vel_pub.publish(vel)

  
  
    else:
      substate = "STOP"
      vel.linear.x = 0
      vel.angular.z = 0
      vel_pub.publish(vel)

      
    state_pub.publish(substate)
    rate.sleep()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass



#publicar velocidades em /mavros/actuator_control
