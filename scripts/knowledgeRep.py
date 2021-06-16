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
        {'name':"LeavingRoom",'color': "green", "x":0, "y":0, 'detected':False},
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
    ## Function that return the room color given an input room name
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
                if not (3 > tmpX > 0 and tmpY > 0) or not(tmpX < 0 and tempY < -5):
                    self.prevXpos = tmpX
                    self.prevYpos = tmpY
                    return [tmpX, tmpY]
    
    ## Returns a array which contains a neighborhood of a given number. For instance if a = 1 and l = 2 it'll return [-1, 0, 1, 2, 3]. 
    # This method guaranties, that given a point (x,y), to return a 2lX2l area around the point.
    # @param a number that correspond to a coordinate of a point.
    # @param l is half of the neighborhood that will be generated.
    def mrange(self, a, l):
        minA = a - l
        r = []
        for i in range(0,l*2+1):
            r.append(minA + i)
        return r

    ## Explore function that returns a random position away from the rooms already visited and the position reached during the FIND mode. 
    # Basically it generate a random position and check if it belongs 
    # in the neighborhood of each detected room (or position), if so it will reach such location. See the README for more details. 
    def explore(self):
        while True:
            ok = True
            pos = self.random_pos()
            for i in self.locationKnown:
                if (pos[0] in self.mrange(i[0],1)) and (pos[1] in self.mrange(i[1],1)):
                    ok = False
                    break
            if ok:
                for room in self.ROOMS:
                    if room['detected'] == True:
                        rx = self.mrange(room['x'], 2)
                        ry = self.mrange(room['y'], 2)
                        if (pos[0] in rx and pos[1] in ry):
                            ok = False
                            break
            if ok:
                self.locationKnown.append(pos)
                return pos
