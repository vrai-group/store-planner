#!/usr/bin/env python

import os.path
import rospy
import rospkg
import cv2
import math 
import numpy as np
import yaml
from shelf import Shelf
from utils import Utils

class Map:

    def __init__(self, path, ratio=0.0):

        self.map = Utils.read_image(path + '/map.pgm')

        #TODO: from filepath open map.pgm and yaml to store values
        # self.map = static_map
        self.img_height = self.map.shape[0]
        self.img_width = self.map.shape[1]
        self.map_origin, self.meters_each_px = self.parse_yaml(path + '/map.yaml')
        self.pxs_each_meter = int(float(1)/self.meters_each_px)

        self.store_height = self.img_height * self.meters_each_px

    def parse_yaml(self,filename):
        file = open(filename)
        parsed_yaml_file = yaml.load(file)
        return tuple(parsed_yaml_file["origin"]), float(parsed_yaml_file["resolution"])

    def get_map_origin(self):
        return self.map_origin

    def get_meters_each_px(self):
        return meters_each_px

    def get_pxs_each_meter(self):
        return self.pxs_each_meter

    def get_static_map(self):
        return self.map

    def set_m_px_ratio(self, ratio):
        self.m_px_ratio = ratio

    ##################################
    ## COORDINATES UTILS
    ##################################

    def pixels2meters(self,x_px,y_px):
        x_m = x_px * self.meters_each_px
        y_m = y_px * self.meters_each_px

        return x_m,y_m

    def meters2pixels(self,x_m,y_m):
        x_px = int(x_m / self.meters_each_px)
        y_px = int(y_m / self.meters_each_px)

        return x_px,y_px

    #changes coordinates of a point in meter from image to map coordinates
    def image2map(self,p_x,p_y):
        x_map = p_x * self.meters_each_px
        y_map = self.store_height - p_y * self.meters_each_px

        return x_map,y_map
    #changes coordinates of a point in pixels from map to image coordinates
    def map2image(self,p_x,p_y):
        x_img = int(p_x / self.meters_each_px)
        y_img = self.img_height - int(p_y / self.meters_each_px)

        return x_img,y_img

    #change reference (invert y axis) of a point in pixels
    def map2image_pixels(self,p_x,p_y):
        x = p_x
        y = self.img_height - p_y

        return x,y
    #change reference (invert y axis) of a point in meters
    def map2image_meters(self,p_x,p_y):
        x = p_x
        y = self.store_height - p_y
        
        return x,y
