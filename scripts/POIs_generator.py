#!/usr/bin/env python

import argparse
import rospkg
import cv2
import numpy as np
from utils import Utils
import math
import sys
import os
from cameras import CamerasParams
from map import Map
from shelf import Shelf
from tsp import Tsp


#a Point of interest is a point in map with a shelf inside the store to which it refers and eventually an angle
class PoI:
    def __init__(self, x, y, shelf_id=0, theta=0):
        self.x = x
        self.y = y
        self.shelf_id = shelf_id
        self.theta = theta #deg

    def __setitem__(self, index, data):
        if index == 0:
            self.x = data
        if index == 1:
            self.y = data
        if index == 2:
            self.shelf_id = data
        if index == 3:
            self.theta = data

    def __getitem__(self, index):
        if index == 0:
            return self.x 
        if index == 1:
            return self.y
        if index == 2:
            return self.shelf_id

        return self.theta

class PointsOfInterestGenerator:
    def __init__(self, args):
        rospack = rospkg.RosPack()
        self.path = rospack.get_path('store-planner') + '/heatmaps/' + str(args["folder"]) + '/'

        self.base_path = rospack.get_path('store-planner') + '/'
        param_file = self.base_path + 'config/params.json'

        x, y, yaw, self.planner_mode, self.debug_mode = Utils.parse_param_file(param_file)
        
        self.folder = str(args["folder"])

        self.docker_position = PoI(x,y)
        
        self.robot_size = 0.35 #0.5 meters length exactly and it is square -> 0.35 radious

        self.pois = list()

        if self.planner_mode == 0:
            print('Customers behaviour following!')
            self.heatmap = Utils.read_image(self.path + 'hotmap.png') 
        else:
            print('Customers avoidance behaviour!')
            self.heatmap = Utils.read_image(self.path + 'coldmap.png')

        self.img_height = self.heatmap.shape[0]
        self.img_width = self.heatmap.shape[1]

        self.map_ = Map(self.base_path + 'maps/' + str(args["folder"]))
        self.static_map = self.map_.get_static_map()

        check_img = self.heatmap.copy()
        
        #if not stored already, selectROIs will be opened
        self.shelves = self.calc_shelves_position()

        if len(self.shelves) == 0:
            print('Shelves list cannot be empty! Please select shelves regions.')
            Utils.remove_file(self.path + 'shelves.json')
            sys.exit(-1)

        #if not stored already, selectROIs will be opened
        self.forbidden_areas = self.calc_forbidden_areas() #stored as shelves as well
        if len(self.forbidden_areas) == 0:
            print('Forbidden areas list cannot be empty! Please select at least one region.')
            Utils.remove_file(self.path + 'forbidden.json')
            sys.exit(-2)

        self.rep_areas, self.d_min = self.calc_repulsive_areas()

        #capture pois have a shelf id associates
        #captures orientation is done considering robot cameras on the right side (hardcoded)
        self.capture_pois = self.calc_capture_pois()

        #navigation pois instead are "free" from shelf id
        self.nav_pois = list()
        self.final_pois = list()


        #draw them
        for i in range(0,len(self.forbidden_areas)):
            Utils.draw_forbidden(check_img,self.forbidden_areas[i],(0,0,255))

        for i in range(0,len(self.rep_areas)):
            Utils.draw_shelf(check_img,self.rep_areas[i],(255,0,0))

        for i in range(0,len(self.shelves)):
            Utils.draw_shelf(check_img,self.shelves[i])


        for i in range(0,len(self.capture_pois)):
            Utils.draw_point(check_img,(self.capture_pois[i].x,self.capture_pois[i].y),(255,255,0))
            

    def run(self):

        print('Generating Points Of Interest!')
       
        self.pois = self.quadtree2POIs()

        filtered_pois = self.forbidden_pois_removal()

        #navigation pois have a shelf id associates
        self.nav_pois = self.calc_navigation_pois(filtered_pois)

        self.final_pois = self.calc_final_pois()

        #Add the docker position as the first node (or PoI) to consider next as the depot for the TSP
        self.final_pois.insert(0,self.docker_position)

        #we are considering cameras to be on the right side only of the robot.
        #if this is not the case the code should be changed accordingly
        

        img = self.heatmap.copy()
        final_img = img.copy()

        for i in range(0,len(self.nav_pois)):
            Utils.draw_point(img,(self.nav_pois[i].x,self.nav_pois[i].y)) #just for drawing need to invert coords
            cv2.putText(img,str(self.nav_pois[i].shelf_id),(self.nav_pois[i].x-10,self.nav_pois[i].y+10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),1)


        for i in range(0,len(self.capture_pois)):
            Utils.draw_point(img,(self.capture_pois[i].x,self.capture_pois[i].y),(255,0,0)) #just for drawing need to invert coords
            # cv2.putText(img,str(self.capture_pois[i].shelf_id),(self.capture_pois[i].x-10,self.capture_pois[i].y+10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1)
        
        for i in range(0,len(self.shelves)):
            Utils.draw_shelf(final_img,self.shelves[i])

        # for i in range(0,len(self.final_pois)):
        #     Utils.draw_point(final_img,(self.final_pois[i].x,self.final_pois[i].y),(255,0,0))
        #     cv2.putText(final_img,str(self.final_pois[i].shelf_id),(self.final_pois[i].x-10,self.final_pois[i].y+10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),1)
        
        for i in range(0,len(self.nav_pois)):
            Utils.draw_point(final_img,(self.nav_pois[i].x,self.nav_pois[i].y)) #just for drawing need to invert coords
            cv2.putText(final_img,str(self.nav_pois[i].shelf_id),(self.nav_pois[i].x-10,self.nav_pois[i].y+10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),1)
        
        # Utils.show_img_and_wait_key('FInal pois', img,final_img)
        # Utils.save_image('/home/majo/Desktop/final_img.png', final_img)

        ## OPTIMIZATION (Traveling Salesman Problem) SETUP
        tsp = Tsp(self.heatmap, self.final_pois, self.shelves, self.base_path + 'results/' + self.folder)
        tsp.write_final_path()
     


    def quadtree2POIs(self):
        pois = list()
        #block_img only for visual check (can be canceled)
        heatmap = self.heatmap.copy()
        blocks_img = heatmap.copy()

        if self.planner_mode == 0:
            blocks_path = self.base_path + 'config/' + self.folder + '/quadtree_blocks/hotmap/'
        else:
            blocks_path = self.base_path + 'config/' + self.folder + '/quadtree_blocks/coldmap/'
        
        for file in os.listdir(blocks_path):
            if file.endswith(".txt"):
                block_size = int(os.path.splitext(file)[0])
                with open(os.path.join(blocks_path, file), 'r') as f:
                    coords = f.read().splitlines()
                    if block_size >= 8:#and block_size <= 128:
                        for coord in coords:
                            x = int(coord.split()[0])-1 #-1 necessary since comes from matlab
                            y = int(coord.split()[1])-1
                            roi = (x,y,block_size,block_size)
                            
                            patch = heatmap[y:y+block_size, x:x+block_size]

                            if Utils.get_patch_average(patch) > 200:# and Utils.get_patch_variance(patch) < 100:
                                
                                shelf_id = self.find_query_shelf_id((x,y)) #currently not used, just for the future
                                
                                pois.append(PoI(int(x+block_size/2),int(y+block_size/2),shelf_id, -1))
                                Utils.draw_roi(blocks_img,roi,(0,0,255),1)
                                Utils.draw_point(blocks_img,(int(x+block_size/2),int(y+block_size/2)),(0,255,0),1)
                            else:
                                Utils.draw_roi(blocks_img,roi,(255,0,0),1)

        Utils.show_img_and_wait_key('QuadTree Decomp',blocks_img)

        return pois

    def calc_shelves_position(self):
        shelves = list()
        if Utils.file_exists(self.path + 'shelves.json'):
            return Utils.parse_shelves(self.path + 'shelves.json')
        else:
            shelves = Utils.select_shelves(self.static_map)
            Utils.save_shelves(self.path + 'shelves.json',shelves)
            return Utils.parse_shelves(self.path + 'shelves.json')
            

    def calc_forbidden_areas(self):
        forbidden = list()
        if Utils.file_exists(self.path + 'forbidden.json'):
            return Utils.parse_shelves(self.path + 'forbidden.json')
        else:
            forbidden = Utils.select_shelves(self.static_map,'Forbidden Areas Selector')
            Utils.save_shelves(self.path + 'forbidden.json',forbidden)
            return Utils.parse_shelves(self.path + 'forbidden.json')

    def calc_repulsive_areas(self):

        rep_areas = list()
        d_min = 0
    
        for shelf in self.shelves:
            
            if shelf.z > CamerasParams.robot_height: #make sure we take the shelf top part
                d_min = (shelf.z - CamerasParams.top_camera_height)/math.tan(CamerasParams.vfov/2)
            else: #make sure we take the shelf bottom part
                d_min = (CamerasParams.bottom_camera_height)/math.tan(CamerasParams.vfov/2)
            
            d_min = d_min

            d_min_px = self.map_.map2image(d_min,0)[0]
             
            eps = 0
            d = d_min_px + eps #to have some margin eventually
   
            repulsive_area = Shelf(shelf.id,shelf.x-d,shelf.y-d,shelf.z,shelf.w+2*d,shelf.h+2*d) #basicallly enhancing the shelves
            rep_areas.append(repulsive_area)

        return rep_areas, d_min

    def calc_capture_pois(self):
        capture_pois = list()
        pois_d_min = self.calc_px_hfov(self.d_min) - 2 #to have 10 cm overlap
        count = 0

        for rep_area in self.rep_areas:
            count + count + 1
                  
            x = rep_area.x
            y = rep_area.y
            h = rep_area.h
            w = rep_area.w
            id_ = rep_area.id #same as the shelf for construction

            if id_ == 0:
                print('ECCOLO!!!!')

            for shelf in self.shelves:

                if shelf.id == 0:
                    print('ECCOLO22!!!!')
                if shelf.id == id_:
                    s_x = shelf.x
                    s_y = shelf.y
                    s_h = shelf.h
                    s_w = shelf.w
                    break
            
            #TODO:refactor this part
            # if self.is_capture_point_good(y,x+int(w/2)):
            #     capture_pois.append(PoI(x+int(w/2),y,id_,-90))
            # if self.is_capture_point_good(y+h,x+int(w/2)):
            #     capture_pois.append(PoI(x+int(w/2),y+h,id_,90))
            # if self.is_capture_point_good(y+int(h/2),x):
            #     capture_pois.append(PoI(x,y+int(h/2),id_,0))
            # if self.is_capture_point_good(y+int(h/2), x+w):
            #     capture_pois.append(PoI(x+w,y+int(h/2),id_,180))

            if self.is_capture_point_good(y,x+int(w/2)):
                capture_pois.append(PoI(x+int(w/2),y,id_,0))
            if self.is_capture_point_good(y+h,x+int(w/2)):
                capture_pois.append(PoI(x+int(w/2),y+h,id_,180))
            if self.is_capture_point_good(y+int(h/2),x):
                capture_pois.append(PoI(x,y+int(h/2),id_,90))
            if self.is_capture_point_good(y+int(h/2), x+w):
                capture_pois.append(PoI(x+w,y+int(h/2),id_,-90))

            num_h_points = 0
            num_v_points = 0

            while (num_h_points*pois_d_min<=int(w/2)): #TODO ABSOLUTELY WITH w and h of SHELVES AND NOT REP AREAS
                num_h_points =  num_h_points + 1

            while (num_v_points*pois_d_min<=int(h/2)):
                num_v_points =  num_v_points + 1
        
            #TODO: eventually check also in the neighbourhood if it's white
            for i in range(1,num_h_points):
                if self.is_capture_point_good(y,x+int(w/2) + i*pois_d_min):
                    capture_pois.append(PoI(x+int(w/2) + i*pois_d_min,y,id_,-90))
                if self.is_capture_point_good(y,x+int(w/2) - i*pois_d_min):
                    capture_pois.append(PoI(x+int(w/2) - i*pois_d_min,y,id_,-90))
                if self.is_capture_point_good(y+h, x+int(w/2) + i*pois_d_min):
                    capture_pois.append(PoI(x+int(w/2) + i*pois_d_min,y+h,id_,90))
                if self.is_capture_point_good(y+h, x+int(w/2) - i*pois_d_min):
                    capture_pois.append(PoI(x+int(w/2) - i*pois_d_min,y+h,id_,90))

            for i in range(1,num_v_points):
                if self.is_capture_point_good(y+int(h/2) + i*pois_d_min,x):
                    capture_pois.append(PoI(x, y+int(h/2) + i*pois_d_min,id_,0))
                if self.is_capture_point_good(y+int(h/2) - i*pois_d_min, x):
                    capture_pois.append(PoI(x, y+int(h/2) - i*pois_d_min,id_,0))
                if self.is_capture_point_good(y+int(h/2) + i*pois_d_min, x+w):     
                    capture_pois.append(PoI(x+w, y+int(h/2) + i*pois_d_min,id_,180))
                if self.is_capture_point_good(y+int(h/2) - i*pois_d_min, x+w):
                    capture_pois.append(PoI(x+w, y+int(h/2) - i*pois_d_min,id_,180)) 

            # for i in range(1,num_h_points):
            #     if self.is_capture_point_good(y,x+int(w/2) + i*pois_d_min) and self.is_shelf_visible(x+int(w/2) + i*pois_d_min,s_x,s_x+s_w):
            #         capture_pois.append(PoI(x+int(w/2) + i*pois_d_min,y,id_,0))
            #     if self.is_capture_point_good(y,x+int(w/2) - i*pois_d_min) and self.is_shelf_visible(x+int(w/2) - i*pois_d_min,s_x,s_x+s_w):
            #         capture_pois.append(PoI(x+int(w/2) - i*pois_d_min,y,id_,0))
            #     if self.is_capture_point_good(y+h, x+int(w/2) + i*pois_d_min) and self.is_shelf_visible(x+int(w/2) + i*pois_d_min,s_x,s_x+s_w):
            #         capture_pois.append(PoI(x+int(w/2) + i*pois_d_min,y+h,id_,180))
            #     if self.is_capture_point_good(y+h, x+int(w/2) - i*pois_d_min) and self.is_shelf_visible(x+int(w/2) - i*pois_d_min,s_x,s_x+s_w):
            #         capture_pois.append(PoI(x+int(w/2) - i*pois_d_min,y+h,id_,180))

            # for i in range(1,num_v_points):
            #     if self.is_capture_point_good(y+int(h/2) + i*pois_d_min,x) and self.is_shelf_visible(y+int(h/2) + i*pois_d_min,s_y,s_y+s_h):
            #         capture_pois.append(PoI(x, y+int(h/2) + i*pois_d_min,id_,90))
            #     if self.is_capture_point_good(y+int(h/2) - i*pois_d_min, x) and self.is_shelf_visible(y+int(h/2) - i*pois_d_min,s_y,s_y+s_h):     
            #         capture_pois.append(PoI(x, y+int(h/2) - i*pois_d_min,id_,90))
            #     if self.is_capture_point_good(y+int(h/2) + i*pois_d_min, x+w) and self.is_shelf_visible(y+int(h/2) + i*pois_d_min,s_y,s_y+s_h):     
            #         capture_pois.append(PoI(x+w, y+int(h/2) + i*pois_d_min,id_,-90))
            #     if self.is_capture_point_good(y+int(h/2) - i*pois_d_min, x+w) and self.is_shelf_visible(y+int(h/2) - i*pois_d_min,s_y,s_y+s_h):
            #         capture_pois.append(PoI(x+w, y+int(h/2) - i*pois_d_min,id_,-90)) 

        return capture_pois

    def is_capture_point_good(self,y,x):
        if 0 <= x <= self.img_width and 0 <= y <= self.img_height and (self.heatmap[y,x]== 255).all() and not self.is_in_forbidden_area(PoI(x,y)):
            return True
        return False

    def is_shelf_visible(self, coord, lb, ub):
        return True if lb <= coord <= ub else False

    def calc_navigation_pois(self, filtered_pois):
        pois_d_min = self.calc_px_hfov(self.d_min) - 2 #to have 10 cm overlap
        nav_pois = list()

        for i in range(0,len(filtered_pois)-1):
            dists = list()
            for j in range(i+1,len(filtered_pois)):
                if i != j:
                    dists.append(Utils.calc_eucl_dist(filtered_pois[i],filtered_pois[j]))
                # print(Utils.calc_eucl_dist(filtered_pois[i],filtered_pois[j]))
            if min(dists) > pois_d_min -10:
                nav_pois.append(filtered_pois[i])

        return nav_pois

    def calc_final_pois(self):
        pois_d_min = self.calc_px_hfov(self.d_min) - 2 #to have 10 cm overlap
        final_pois = self.capture_pois

        img2 = self.heatmap.copy()
        for i in range(0,len(final_pois)):
            Utils.draw_point(img2,(final_pois[i].x,final_pois[i].y),(255,0,0)) #just for drawing need to invert coords
            cv2.putText(img2,str(final_pois[i].shelf_id),(final_pois[i].x-10,final_pois[i].y+10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1)

        Utils.show_img_and_wait_key('afasfas POIS', img2)
        Utils.save_image('/home/majo/Desktop/asfa.png',img2)

        for i in range(0,len(self.nav_pois)):
            dists = list()
            for j in range(0,len(final_pois)):
                dists.append(Utils.calc_eucl_dist(self.nav_pois[i],final_pois[j]))
                # print(Utils.calc_eucl_dist(filtered_pois[i],filtered_pois[j]))
            if min(dists) > pois_d_min -10:
                final_pois.append(self.nav_pois[i])

        return final_pois


    def calc_px_hfov(self,d_min):
        horiz_d_min = 2*d_min*math.tan(CamerasParams.hfov/2)
        return self.map_.map2image(horiz_d_min,0)[0]

    #since shelves and repulsive areas share the same center and id, we get the query shelf by using the repulsive areas
    def find_query_shelf_id(self,poi):
        dist_to_shelf = 99999
        query_shelf_id = 0

        for shelf in self.shelves:
            #top
            d1 = Utils.dist_point_to_segment(poi,(shelf.x,shelf.y),(shelf.x+shelf.w,shelf.y))[0]
            #bottom
            d2 = Utils.dist_point_to_segment(poi,(shelf.x,shelf.y+shelf.h),(shelf.x+shelf.w,shelf.y+shelf.h))[0]
            #left
            d3 = Utils.dist_point_to_segment(poi,(shelf.x,shelf.y),(shelf.x,shelf.y+shelf.h))[0]
            #right
            d4 = Utils.dist_point_to_segment(poi,(shelf.x+shelf.w,shelf.y),(shelf.x+shelf.w,shelf.y + shelf.h))[0]

            min_dist = min(d1,d2,d3,d4)

            if   min_dist < dist_to_shelf:
                query_shelf_id = shelf.id
                dist_to_shelf_center = min_dist

        if dist_to_shelf > 50: #the generated wpoint is too far/wrong, thus not scanning (TODO in meters)
            query_shelf_id = 0

        return query_shelf_id

    def is_in_forbidden_area(self,poi):
        for i in range(0,len(self.forbidden_areas)):
            x = self.forbidden_areas[i].x
            y = self.forbidden_areas[i].y
            w = self.forbidden_areas[i].w
            h = self.forbidden_areas[i].h

            if ( x <= poi.x <= x + w ) and ( y <= poi.y <= y + h ):
                return True
        return False

    def forbidden_pois_removal(self):
        filtered_pois = list()
        for poi in self.pois:
            if not self.is_in_forbidden_area(poi):
                filtered_pois.append(poi)

        return filtered_pois


if __name__ == '__main__':

    ap = argparse.ArgumentParser()
    ap.add_argument("-f", "--folder", required=True, help="Heatmaps subfolder name")
    # ap.add_argument("-m", "--mode", required=True, help="Planner mode: 0 for customers behaviour, 1 for customers avoidance")

    args = vars(ap.parse_args())
    POIs_gen = PointsOfInterestGenerator(args)
    POIs_gen.run()
    