#!/usr/bin/env python

import json
import sys
import os
import shutil
import rospy
import cv2
import math 
import numpy as np
from shelf import Shelf

class Utils:

    @staticmethod
    def print_usage(exit_code=1):
        if exit_code==1:
            print '''Usage: rosrun store-planner store_planner_node.py <param_filename>  

            - <param_filename> the param file you want to use. 
                            It should be under config folder

                
            Example: rosrun store-planner store_planner_node.py params.yaml
            '''

            sys.exit(exit_code)
        else:
            print '''Usage: python POIs_generator.py -f export_heatmap_25072021_store28 -m 0  

            - <param_filename> the param file you want to use. 
                            It should be under config folder

                
            Example: python POIs_generator.py -f export_heatmap_25072021_store28 -m 0
            '''
            


    ##################################
    ## OPENCV UTILS
    ##################################
    @staticmethod
    def show_img_and_wait_key(window_name,img, img2=None):
        cv2.namedWindow(window_name,cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, int(1602/2), int(1210/2))
        cv2.imshow(window_name, img)
        if img2 is not None:
            cv2.namedWindow("Auxiliary image",cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Auxiliary image", int(1602/2), int(1210/2))
            cv2.imshow("Auxiliary image", img2)
            
        cv2.waitKey(0)
        cv2.destroyWindow(window_name)
        if img2 is not None:
            cv2.destroyWindow("Auxiliary image")
        
    @staticmethod
    def read_image(filename,mode=1): #color
        return cv2.imread(filename, mode)
    @staticmethod
    def save_image(filename,image):
        cv2.imwrite(filename,image)
    @staticmethod
    def gray2bgr(image):
        return cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)
    @staticmethod
    def bgr2gray(image):
        return cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

    ##################################
    ## MISCELLANEOUS UTILS
    ################################## 
    @staticmethod
    def parse_param_file(filepath):
        if os.path.exists(filepath):
            with open(filepath,'rb') as json_file:
                data = json.load(json_file)
        else:
            rospy.signal_shutdown('Could not find parameter file.\nCheck file path.')

        planner_mode = int(data["planner_mode"])
        debug_mode = bool(data["debug_mode"])
        map_resolution = float(data["map_resolution"])

        #docker position is used also as initial pose for the robot
        
        if data["docker"]["unit"] == "px":
            x_px = int(data["docker"]["position"][0])
            y_px = int(data["docker"]["position"][1])

        else:
            x = float(data["docker"]["position"][0])
            y = float(data["docker"]["position"][1])

            x_px = int(x / map_resolution)
            y_px = int(y / map_resolution)
        
        if data["robot"]["unit"] == "deg":
            yaw = Utils.deg2rad(float(data["robot"]["orientation"]))
        else:
            yaw = float(data["robot"]["orientation"])

        #always returns position in pixels and angle in radiants
        #watch out: pose in mete
        return x_px, y_px, yaw, planner_mode, debug_mode

    ##################################
    ## DRAWING UTILS
    ################################## 
    @staticmethod
    def draw_point(img,pt,color=(0,0,255),thickness=3):
        return cv2.circle(img,pt,int(thickness),color,-1)
    @staticmethod
    def draw_PoI(img,poi,color=(0,0,255),thickness=3):
        return cv2.circle(img,(poi.x,poi.y),int(thickness),color,-1)
    @staticmethod
    def draw_robot(img,pt,radius,color=(0,255,0)):
        return cv2.circle(img,pt,int(radius),color,2)
    @staticmethod
    def draw_arrow(img, pt1, pt2, color=(0,0,255),thickness=1):
        return cv2.arrowedLine(img, pt1, pt2, color, thickness)
    @staticmethod
    def draw_roi(img,roi,color=(0,255,0),thickness=2):
        return cv2.rectangle(img,(roi[0],roi[1]),(roi[0]+roi[2],roi[1]+roi[3]),color,thickness)
    @staticmethod
    def draw_shelf(img,shelf,color=(0,255,0),thickness=1):
        cv2.putText(img,str(shelf.id),(shelf.x+int(shelf.w/2)-15,shelf.y+int(shelf.h/2)+8),cv2.FONT_HERSHEY_SIMPLEX,0.7,color,1)
        return cv2.rectangle(img,(shelf.x,shelf.y),(shelf.x+shelf.w,shelf.y+shelf.h),color,thickness)
    @staticmethod
    def draw_forbidden(img,shelf,color=(0,255,0),thickness=1):
        cv2.putText(img,str(shelf.id),(shelf.x+int(shelf.w/2)-5,shelf.y+int(shelf.h/2)+5),cv2.FONT_HERSHEY_SIMPLEX,0.7,color,1)
        return cv2.rectangle(img,(shelf.x,shelf.y),(shelf.x+shelf.w,shelf.y+shelf.h),color,thickness)


    @staticmethod
    def img_contains_color(img, up_color, lw_color):
        w, h, ch = img.shape
        for i in range(w):
            for j in range(h):
                if img[i,j][0]<=up_color[2] and img[i,j][1]<=up_color[1] and img[i,j][2]<=up_color[0] and img[i,j][0]>=lw_color[2] and img[i,j][1]>=lw_color[1] and img[i,j][2]>=lw_color[0]:
                    print(img[i,j])
                    

    @staticmethod
    def hex2rgb(hex_color):
        h = str(hex_color).lstrip('#')
        return np.uint8([[[int(h[i:i+2], 16) for i in (0, 2, 4)]]])

    @staticmethod
    def hex2bgr(hex_color):
        h = str(hex_color).lstrip('#')
        return np.uint8([[[int(h[i:i+2], 16) for i in (4, 2, 0)]]])

    @staticmethod
    def rgb2bgr(RGB):
        return cv2.cvtColor(RGB,cv2.COLOR_RGB2BGR)

    @staticmethod
    def bgr2rbg(BGR):
        return cv2.cvtColor(BGR,cv2.COLOR_BGR2RGB)

    @staticmethod
    def rgb2hsv(RGB):
        return cv2.cvtColor(RGB,cv2.COLOR_RGB2HSV)

    @staticmethod
    def bgr2hsv(BGR):
        return cv2.cvtColor(BGR,cv2.COLOR_BGR2HSV)
    
    @staticmethod
    def bgr2gray(BGR):
        return cv2.cvtColor(BGR,cv2.COLOR_BGR2GRAY)


    @staticmethod
    def dir_exists(path):
        if os.path.exists(path):
            return True
        else:
            return Utils.create_dir(path)

    @staticmethod
    def create_dir(path):
        if not os.path.exists(path):
            os.makedirs(path)

    @staticmethod
    def file_exists(filepath):
        return os.path.isfile(filepath)

    @staticmethod
    def remove_file(filepath):
        os.remove(filepath)

    @staticmethod
    def remove_dir(directory):
        os.rmdir(directory)

    @staticmethod
    def rm_dir_and_contents(directory):
        if Utils.dir_exists(directory):
            shutil.rmtree(directory)

    @staticmethod
    def deg2rad(angle_deg):
        return math.radians(angle_deg)

    @staticmethod
    def rad2deg(angle_rad):
        return math.degrees(angle_rad)

    @staticmethod
    def wrap_deg_yaw(theta): 
        return theta - 360 if theta > 0 else theta + 360

    @staticmethod
    def wrap_rad_yaw(theta): 
        return theta - 2*Utils.pi()  if theta > 0 else theta + 2*Utils.pi()

    @staticmethod
    def calc_eucl_dist(p1,p2):
       return math.sqrt((p2[0] - p1[0])**2 + (p1[1] - p2[1])**2)

    @staticmethod
    def dist_point_to_line(p1,p2,p3): #p1-p2 define the line
        return np.cross(p2-p1,p3-p1)/norm(p2-p1)

    @staticmethod
    def dot(v,w):
        x,y = v
        X,Y = w
        return x*X + y*Y

    @staticmethod
    def length(v):
        x,y = v
        return math.sqrt(x*x + y*y)

    @staticmethod
    def vector(b,e):
        x,y = b
        X,Y = e
        return (X-x, Y-y)

    @staticmethod
    def unit(v):
        x,y = v
        mag = Utils.length(v)
        return (x/mag, y/mag)

    @staticmethod
    def distance(p0,p1):
        return Utils.length(Utils.vector(p0,p1))

    @staticmethod
    def scale(v,sc):
        x,y = v
        return (x * sc, y * sc)

    @staticmethod
    def add(v,w):
        x,y = v
        X,Y = w
        return (x+X, y+Y)

    @staticmethod
    def dist_point_to_segment(p, start, end):
        line_vec = Utils.vector(start, end)
        p_vec = Utils.vector(start, p)
        line_len = Utils.length(line_vec)
        line_unitvec = Utils.unit(line_vec)
        p_vec_scaled = Utils.scale(p_vec, 1.0/line_len)
        t = Utils.dot(line_unitvec, p_vec_scaled)    
        if t < 0.0:
            t = 0.0
        elif t > 1.0:
            t = 1.0
        nearest = Utils.scale(line_vec, t)
        dist = Utils.distance(nearest, p_vec)
        nearest = Utils.add(nearest, start)
        return dist, nearest

    @staticmethod
    def pi():
        return math.pi

    @staticmethod
    def write_dict_to_file(path, dictionary):
        with open(path + '.txt', "w") as f:
            for k, v in dictionary.items():
                f.write(str(k) + ' -> '+ str(v) + '\n')

    @staticmethod
    def get_patch_average(patch):
        return np.average(patch)
    @staticmethod
    def get_patch_variance(patch):
        return np.var(patch)

    @staticmethod
    def parse_shelves(filepath):
        shelves = list()
        with open(filepath,'rb') as json_file:
            data = json.load(json_file)
        
        for shelf in data['shelves']:
            shelf_obj = Shelf(shelf['id'],shelf['x'],shelf['y'],shelf['z'],shelf['w'],shelf['h'])       
            shelves.append(shelf_obj)

        return shelves
    @staticmethod
    def select_shelves(img,winname="Shelves selector"):
    
        cv2.namedWindow(winname,cv2.WINDOW_NORMAL)
        cv2.resizeWindow(winname, img.shape[1], img.shape[0])
        shelves = cv2.selectROIs(winname,img)
        cv2.destroyWindow(winname)
     
        return shelves

    @staticmethod
    def save_shelves(filename, shelves):

        shelf_cnt = 0
        data = {}
        data['shelves'] = []

        for shelf in shelves:
            shelf_cnt = shelf_cnt + 1
            (x, y, w, h) = shelf
            data['shelves'].append({
                'id': shelf_cnt,
                'x': int(x),
                'y': int(y),
                'z': 2.0, #TODO: hardcoded
                'w': int(w),
                'h': int(h)
            })
        with open(filename, 'w') as outfile:
            json.dump(data, outfile)

    
    @staticmethod
    def save_pose_metadata(path,time,x,y,yaw,shelf_id):
        with open(path,'w') as f:
            f.write('# pose.yaml file')
            f.write('\n\n')
            f.write('shelf_id: ' + str(shelf_id) + '\n')
            f.write('capture_time: ' + str(time.secs) + '.' + str(time.nsecs) + '\n')
            f.write('x_robot: ' + str(x) + '\n')
            f.write('y_robot: ' + str(y) + '\n')
            f.write('yaw_robot: ' + str(yaw) )

    @staticmethod
    def save_camera_metadata(path,time,cam_info):
        with open(path,'w') as f:
            f.write('# cameras.yaml file')
            f.write('\n\n')
            f.write('capture_time: ' + str(time.secs) + '.' + str(time.nsecs) + '\n')
            f.write('height: ' + str(cam_info.height) + '\n')
            f.write('width: ' + str(cam_info.width) + '\n')
            f.write('distorton coefficients: ' + str(cam_info.D) + '\n')
            f.write('camera matrix: ' + str(cam_info.K) + '\n')
            f.write('rectification matrix: ' + str(cam_info.R) + '\n') #only for stereo, so not used


