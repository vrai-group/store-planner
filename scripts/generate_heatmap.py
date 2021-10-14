#!/usr/bin/env python

import csv
import argparse
import rospkg
import cv2
import numpy as np
from utils import Utils
import yaml
# from matplotlib import pyplot as plt
from scipy.ndimage.filters import generic_filter
from scipy.stats import mode
import sys

class heatmapGenerator:
    def __init__(self, args):

        rospack = rospkg.RosPack()
        self.path = rospack.get_path('store-planner') + '/'
        self.filename = str(args["csv_file"])
        self.k_sz = int(args["resize_factor"])

        self.filter_type = str(args["filter_type"])

        self.steps = list()
        self.x = list()
        self.y = list()
        self.details = list()

        self.intensities = list()
        self.original_width = 0
        self.original_heigth = 0

        self.bgr_heatmap = 0

        self.hotmap = 0
        self.coldmap = 0
        self.filtered_img = 0
        self.filter_k_size = 0


    def run(self):
        print('Generating Heatmap!')
        self.parse_store_csv()

        heatmap, self.static_map = self.create_heatmap()

        eq_heatmap = cv2.equalizeHist(heatmap)

        #to keep coherency (TODO: optimize)
        self.eq_heatmap = 255 * np.ones([eq_heatmap.shape[0], eq_heatmap.shape[1],1],dtype=np.uint8) 
        self.eq_heatmap[:,:,0] = eq_heatmap
        shoothed_heatmap = self.smooth_heatmap()
        self.hotmap, self.coldmap = self.binarize_heatmap(shoothed_heatmap)

        Utils.create_dir(self.path + 'maps/'+ str(self.filename) + '/')
        Utils.create_dir(self.path + 'heatmaps/'+ str(self.filename) + '/')

        Utils.save_image(self.path + 'maps/'+ str(self.filename) + '/map.pgm', self.static_map)
        Utils.save_image(self.path + 'maps/'+ str(self.filename) + '/submap_0.pgm', self.static_map) #for ROS
        Utils.save_image(self.path + 'heatmaps/'+ str(self.filename) + '/heatmap.png', self.eq_heatmap)

        Utils.save_image(self.path + 'heatmaps/'+ str(self.filename) + '/hotmap.png', self.hotmap)
        Utils.save_image(self.path + 'heatmaps/'+ str(self.filename) + '/coldmap.png', self.coldmap)

        self.write_map_yaml() #hardcoded TODO: parametrize it

        
    def parse_store_csv(self):

        with open(self.path + 'store_csv/' + str(self.filename) + '.csv', 'r') as file:
            reader = csv.reader(file)
            cnt = 0
            for row in reader:
                if cnt > 0:
                    if row[0] == '':
                        self.steps.append(0)
                    else:
                        self.steps.append(int(row[0]))
                    self.x.append(int(row[1]))
                    self.y.append(int(row[2]))
                    self.details.append(str(row[3]))

                cnt = cnt + 1

        self.original_width = max(self.x)
        self.original_heigth = max(self.y)
  

    def create_heatmap(self):

        max_steps = max(self.steps)
        #resized image dimensions
        width = self.k_sz * self.original_width
        heigth = self.k_sz * self.original_heigth

        heatmap = 255 * np.ones([heigth, width,1],dtype=np.uint8) #grayscale
        static_map = 255 * np.ones([heigth, width,1],dtype=np.uint8) #grayscale
    
        for i in range(0,len(self.steps)):
            if self.details[i] == 'Wall' or self.details[i] == 'Shelf':
                heatmap[self.k_sz*(self.y[i]-1):self.k_sz*self.y[i], self.k_sz*(self.x[i]-1):self.k_sz*self.x[i]] = [0]
                static_map[self.k_sz*(self.y[i]-1):self.k_sz*self.y[i], self.k_sz*(self.x[i]-1):self.k_sz*self.x[i]] = [0]
            else: #steps
                intensity = self.calculate_intensity(self.steps[i], max_steps)
                if intensity not in self.intensities : self.intensities.append(intensity)
                heatmap[self.k_sz*(self.y[i]-1):self.k_sz*self.y[i], self.k_sz*(self.x[i]-1):self.k_sz*self.x[i]] = [intensity]


        return heatmap, static_map



    def calculate_intensity(self, num_steps, max_steps):

        min_steps = 0
        min_intensity = 10 #to distinguish with black walls and shelves
        max_intensity = 255

        return self.step2intensity(num_steps,min_steps,max_steps,min_intensity,max_intensity)


    def step2intensity(self, s,s_min,s_max,i_min,i_max):
        return i_min+int((float((i_max-i_min))/float((s_max-s_min)))*float((s-s_min)))


    def equalize_intensities(self, heatmap):

        eq_heatmap = heatmap.copy()
        int_step = (255.0- 10.0)/len(self.intensities)
        self.intensities.sort()

        eq_intensities = list()
        
        step = 0
        for k in self.intensities:
            mask = heatmap == k
            eq_heatmap[mask] = [10 + int(step*int_step)]
            eq_intensities.append(10 + int(step*int_step))
            step = step + 1  

        bgr_heatmap = cv2.cvtColor(eq_heatmap,cv2.COLOR_GRAY2BGR)
        bgr_heatmap[:,:,0] = 0
        bgr_heatmap[:,:,1] = 0

        return eq_heatmap, eq_intensities, bgr_heatmap


    def make_bgr_heatmap(self,heatmap):
        bgr_heatmap = cv2.cvtColor(heatmap,cv2.COLOR_GRAY2BGR)
        bgr_heatmap[:,:,0] = 0
        bgr_heatmap[:,:,1] = 0

        return bgr_heatmap


    def hotmap_creator(self,val):
        level = cv2.getTrackbarPos('Grayscale level', 'HotMap')
        self.hotmap = np.where(np.all((self.eq_heatmap >= (level)) , axis=-1, keepdims=True), (255), (0)).astype(np.uint8)
        self.coldmap = np.bitwise_not(self.hotmap) & self.static_map
        cv2.imshow('HotMap', self.hotmap)
        cv2.imshow('ColdMap', self.coldmap)

    def smooth_heatmap(self):     
        print('Press s to save the current image as result')
        cv2.namedWindow('Filter')
        cv2.createTrackbar('Kernel size', 'Filter', 3, 21, self.apply_filter)
        self.apply_filter(0)
    
        k = cv2.waitKey(0)
        if k == ord('s'):
            #opencv filters return 2darray
            smoothed_heatmap = 255 * np.ones([self.filtered_img.shape[0], self.filtered_img.shape[1],1],dtype=np.uint8)
            if len(self.filtered_img.shape) == 2:
                smoothed_heatmap[:,:,0] = self.filtered_img
                smoothed_heatmap[self.static_map == 0] = [0]
            else:
                smoothed_heatmap = self.filtered_img
                smoothed_heatmap[self.static_map == 0] = [0]
        
            # # Utils.save_image(self.path + 'heatmaps/'+ str(self.filename) + '/' + str(self.filter_type) + '_' + str(self.filter_k_size) + '.png', self.filtered_img - np.bitwise_not(self.static_map))
            Utils.save_image(self.path + 'heatmaps/'+ str(self.filename) + '/' + str(self.filter_type) + '_' + str(self.filter_k_size) + '.png', smoothed_heatmap)
           
        return smoothed_heatmap

    def apply_filter(self,val):
        self.filter_k_size = cv2.getTrackbarPos('Kernel size', 'Filter')

        if self.filter_k_size%2 == 0:
            self.filter_k_size = self.filter_k_size + 1

        if self.filter_type == 'gauss':
            self.filtered_img = cv2.GaussianBlur(self.eq_heatmap + np.bitwise_not(self.static_map),(self.filter_k_size,self.filter_k_size),0)
        if self.filter_type == 'median':
            self.filtered_img = cv2.medianBlur(self.eq_heatmap + np.bitwise_not(self.static_map),self.filter_k_size)
        if self.filter_type == 'bilateral':
            self.filtered_img = cv2.bilateralFilter(self.eq_heatmap + np.bitwise_not(self.static_map),self.filter_k_size,75,75)
        if self.filter_type == 'majority':
            self.filtered_img = self.majority_filter(self.eq_heatmap + np.bitwise_not(self.static_map),self.filter_k_size)
        if self.filter_type == 'max':
            self.filtered_img = self.max_filter(self.eq_heatmap + np.bitwise_not(self.static_map),self.filter_k_size)
        if self.filter_type == 'min':
            self.filtered_img = self.min_filter(self.eq_heatmap + np.bitwise_not(self.static_map),self.filter_k_size)

        cv2.imshow('Filter', self.filtered_img)


    def majority_filter(self, img, k_size):
        height= img.shape[0]
        width  = img.shape[1]
        pad = k_size// 2
        out_img = img.copy()

        #excluding boundaries
        for y in range(pad, height-pad):
            for x in range(pad, width-pad):
                out_img[y,x] = np.bincount(img[y-pad:y+pad, x-pad:x+pad].flatten()).argmax()

        return out_img

    def max_filter(self, img, k_size):
        height= img.shape[0]
        width  = img.shape[1]
        pad = k_size // 2
        out_img = img.copy()
   
        #excluding boundaries
        for y in range(pad, height-pad):
            for x in range(pad, width-pad):
                out_img[y,x] = max((img[y-pad:y+pad, x-pad:x+pad].flatten()))
                
        return out_img

    def min_filter(self, img, k_size):
        height= img.shape[0]
        width  = img.shape[1]
        pad = k_size // 2
        out_img = img.copy()
   
        #excluding boundaries
        for y in range(pad, height-pad):
            for x in range(pad, width-pad):
                out_img[y,x] = min((img[y-pad:y+pad, x-pad:x+pad].flatten()))
                
        return out_img
    
    def write_map_yaml(self):

        map_data = dict()
        # ALL HARDCODED: watch out
        map_data["image"] = 'submap_0.pgm'
        map_data["resolution"] = 0.200000000 / self.k_sz
        map_data["origin"] = [0.000000, 0.000000, 0.000000]
        map_data["negate"] = 0
        map_data["occupied_thresh"] = 0.65
        map_data["free_thresh"] = 0.196

        with open(self.path + 'maps/'+ str(self.filename) + '/map.yaml', 'w') as file:
            yaml.dump(map_data, file)

        line = "{numberOfSubMaps: 1}"
        with open(self.path + 'maps/'+ str(self.filename) + '/mmap.yaml', 'w') as file:
            file.write(line)
       

    def binarize_heatmap(self, img):
        r, hotmap = cv2.threshold(img,128,255,cv2.THRESH_BINARY)
        r, coldmap = cv2.threshold(img,128,255,cv2.THRESH_BINARY_INV)

        coldmap = coldmap - np.bitwise_not(self.static_map[:,:,0])

        Utils.show_img_and_wait_key('Hotmap', hotmap)
        Utils.show_img_and_wait_key('Coldmap', coldmap)

        return hotmap, coldmap


if __name__ == '__main__':

    ap = argparse.ArgumentParser()
    ap.add_argument("-csv", "--csv_file", required=True, help="csv filename (without file extension)")
    ap.add_argument("-r", "--resize_factor",default=4, help="Resize factor (int) for the output heatmap, default: 4")
    ap.add_argument("-f", "--filter_type",default='majority', help="Filter type for heatmap smoothing, default: majority. Options: gauss, median, bilateral, majority, max, min.")


    args = vars(ap.parse_args())
    heatmap_gen = heatmapGenerator(args)
    heatmap_gen.run()
    