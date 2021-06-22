#!/usr/bin/env python
## @file knowledgRep.py
# This python file contain the class Rooms() over which the knowledg is build, in fact is inside this script where the color of the balls,
# the rooms and their positions are releated. It also implement some useful function for handling room detection. 

# Python Libraries
import random
# Ros Libraries
import rospy 
# Action Server
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

## Class which contain the knowledg representation, for each room name we have a releated color, position and a boolean which notify
# if the rooms has been detected or not.
class Rooms():

    def __init__(self):
        self.ROOMS = [ 
        {'name':"Entrance",'color': "blue", "x":0, "y":0, 'detected':False},
        {'name':"Closet",'color': "red", "x":0, "y":0, 'detected':False},
        {'name':"LivingRoom",'color': "green", "x":0, "y":0, 'detected':False},
        {'name':"Kitchen",'color': "yellow", "x":0, "y":0, 'detected':False},
        {'name':"BathRoom",'color': "magenta", "x":0, "y":0, 'detected':False},
        {'name':"BedRoom",'color':"black","x":0,"y":0, 'detected':False},
        {'name':"Home",'color':"","x":-5,"y":7, 'detected':True}		           
        ]
        self.prevXpos = 0
        self.prevYpos = 0
        self.locationKnown = [[-5,7]] #initialize with only the home position known a-priori
         
    ## Function that check if the input color (associated to a particular rooms) was already detected.
    def room_check(self, color):
        for room in self.ROOMS:
            if color == room['color']:
                if room['detected'] == True:
                    return True
        return False
    ## Function that returns the position of the given input room.
    def room_position(self, target_room):
        for room in self.ROOMS:
            if target_room == room['name']:
                if room['detected'] == True:
                    return [room["x"], room["y"]]
        return False
    ## Function that check and inform the user if a new room is detected, now the room has a position and the robot know it.
    def new_room(self, color, x, y):
        for room in self.ROOMS:
            if color == room['color']:
                room['detected'] = True
                room['x'] = int(x)
                room['y'] = int(y)
                name = str(room['name'])
                print("[knowledgRep]: *****DISCOVERED ROOM:"+name+"*****")

    ## Function that return the room name given an input position.
    def room_name(self, x, y):
        for room in self.ROOMS:
            if (x == room['x'] and y == room['y']):
                return room['name']
        return False    
    ## Function that return the room color given of an input room name
    def room_color(self, name):
        for room in self.ROOMS:
            if name == room['name']:
                return room['color']
        return False
    ## Function that generate random positions (inside our house)
    def random_pos(self):
        while True:
            tmpX = random.randint(-6,6)
            tmpY = random.randint(-8,5)
            if tmpX != self.prevXpos and tmpY != self.prevYpos:
                if not (3 > tmpX > 0 and tmpY > 0) or not(tmpX < 0 and tmpY < -5):
                    self.prevXpos = tmpX
                    self.prevYpos = tmpY
                    return [tmpX, tmpY]
    
    ## This function receive as input a coordinate and a scalar "l", returns an array of "l-neighborhood" of the given coordinate. 
    # @param a number. Coordinate of a position
    # @param l number. Dimension of the neighborhood we want.
    def room_range(self, a, l):
        least_area = a - l
        neighborhood = []
        area = l*2+1
        for i in range(0,area):
            neighborhood.append(least_area + i)
        return neighborhood

    ## This function implement the exploration for the state FIND. Returns random positions NOT NEAR already discoverd rooms, so simply apply a condition, using "room_range", to the random position generated which are discarde if inside the neighborhood of discovered rooms.
    def room_explore(self):
        while True:
            bool = True
            position = self.random_pos()
            for i in self.locationKnown:
                if (position[0] in self.room_range(i[0],1)) and (position[1] in self.room_range(i[1],1)):
                    bool = False
                    break
            if bool:
                for room in self.ROOMS:
                    if room['detected'] == True:
                        range_x = self.room_range(room['x'], 2)
                        range_y = self.room_range(room['y'], 2)
                        if (position[0] in range_x and position[1] in range_y):
                            bool = False
                            break
            if bool:
                self.locationKnown.append(position)
                return position
