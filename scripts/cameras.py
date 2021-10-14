#!/usr/bin/env python
import math

#An helper class containing all the parameters regarding cameras mounted on the robot
#Attention: data must match those in the robot datasheet
#hardcoded
class CamerasParams:

  # def __init__(self):
  w = 3840 #image width
  h = 2160 #image height
  hfov = float(60*math.pi/180) # must be transformed in rads
  vfov = 2*math.atan(math.tan(hfov/2)*h/w) #not available on ROS
  base_height = 0.195 #from pal model
  robot_height = 1.8 
  top_camera_height = robot_height - base_height
  middle_camera_height = 2*robot_height/3 - 2*base_height
  bottom_camera_height = robot_height/3 - 2*base_height