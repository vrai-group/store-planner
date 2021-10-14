#!/usr/bin/env python

class Shelf:

    def __init__(self, id=0, x=0.0, y=0.0, z=0.0, w=0.0, h=0.0):
        self.id = id
        self.x = x
        self.y = y
        self.z = z #in meters
        self.w = w
        self.h = h
    
        self.center = (self.x + int(self.w/2), self.y + int(self.h/2))

        self.dir = 'h' if w > h else 'v'
        
        if w == h:
            self.dir = 's' #square
