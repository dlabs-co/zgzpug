#!/usr/bin/env python
"""
Demo for Kinect Talk
Mouse movement with Kinect Sensor using freenect library
David Pellicer Martín 
davidpellicermartin@gmail.com
"""
from freenect import sync_get_depth as get_depth, sync_get_video as get_video
import cv,cv2  
import numpy as np
from Xlib import X, display
import Xlib.XK
import Xlib.error
import Xlib.ext.xtest


"""
Attempt to show image, but it has bad colour RGB
"""
def show1(img):
    cv.ShowImage('rgb', cv.fromarray(img.astype(np.uint8)))
  
"""
OpenCV uses BGR format so we have to change image
"""
def show2(img):
    image = img[:, :, ::-1] #RGB -> BGR We reverse the colour array
    cv.ShowImage('rgb', cv.fromarray(image.astype(np.uint8)))

"""
We draw a circle in the closest point detected by depth camera
"""
def show3(img, depth):
    image = img[:, :, ::-1] #RGB -> BGR We reverse the colour array
    point = np.unravel_index(depth.argmin(), depth.shape) # Get closest point to camera
    image = cv.fromarray(image.astype(np.uint8))
    cv.Circle(image, point[::-1], 20, cv.Scalar(255,0,0), -1) # Add a circle in pointer
    cv.ShowImage('rgb', image)

""" 
Calculates euclidean distance between two points
"""
def dist(x,y):   
    return np.sqrt(np.sum((np.array(x)-np.array(y))**2))

def move_mouse(point):
    d = display.Display()
    s = d.screen()
    root = s.root
    mouse_data = root.query_pointer()._data
    mx = mouse_data['root_x'] # Actual mouse position
    my = mouse_data['root_y']
    x, y = point # New movement coordinates
    cx, cy = cached_point # Previous movement 
    dx = s.root.get_geometry().width # Display resolution
    dy = s.root.get_geometry().height
    ny = my + ((y - cy)*dy)/480 # Adjust movement to real resolution
    nx = mx - ((x - cx)*dx)/640 # We have flip the img, so + becomes -
    root.warp_pointer(nx,ny) # Move pointer
    d.sync()


"""
It uses a cache to remove noise and flip image 
"""
def show4(img, depth):
    global cached_point
    image = img[:, :, ::-1] #RGB -> BGR We reverse the colour array
    np.clip(depth, 0, 1024, depth)
    point = np.unravel_index(depth.argmin(), depth.shape)[::-1]# Get closest point to camera
    distance = dist(cached_point, point)
    if distance > 20 and distance < 100:
        cached_point = point
    image = cv.fromarray(image.astype(np.uint8))
    cv.Circle(image, cached_point, 20, cv.Scalar(255,0,0), -1) # Add a circle in pointer
    cv.Flip(image, image, 1) # We flip image to get mirror video
    cv.ShowImage('rgb', image)

def show_and_move(img, depth):
    global cached_point
    image = img[:, :, ::-1] #RGB -> BGR We reverse the colour array
    np.clip(depth, 0, 1024, depth)
    point = np.unravel_index(depth.argmin(), depth.shape)[::-1]# Get closest point to camera
    distance = dist(cached_point, point)
    if distance > 20 and distance < 100:
        move_mouse(point)
        cached_point = point
    image = cv.fromarray(image.astype(np.uint8))
    cv.Circle(image, cached_point, 20, cv.Scalar(255,0,0), -1) # Add a circle in pointer
    cv.Flip(image, image, 1) # We flip image to get mirror video
    cv.ShowImage('rgb', image)


def show_depth(depth):
    cv.ShowImage('depth', cv.fromarray(depth.astype(np.uint8)))

def show_depth2(depth):
    depth=depth.astype(np.float32)
    front = cv2.threshold(depth, 900, 450, cv2.THRESH_BINARY_INV)[1]
    cv.ShowImage('window', cv.fromarray(front))


def do_loop():
    while True:
        # Get a fresh frame
        (depth, _), (rgb, _) = get_depth(), get_video()

        # Show it
        #show_and_move(rgb, depth)
        show_and_move(rgb, depth)
        show_depth(depth)
        cv.WaitKey(5)

cached_point = (0,0)
do_loop()
