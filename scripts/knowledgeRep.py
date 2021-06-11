#!/usr/bin/env python
import actionlib
import rospy 
import random

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Rooms():

    def __init__(self):
        self.ROOMS = [ 
        {'name':"Entrance",'color': "blue", "x":0, "y":0, 'detected':False},
        {'name':"Closet",'color': "red", "x":0, "y":0, 'detected':False},
        {'name':"LeavingRoom",'color': "green", "x":0, "y":0, 'detected':False},
        {'name':"Kitchen",'color': "yellow", "x":0, "y":0, 'detected':False},
        {'name':"BathRoom",'color': "orange", "x":0, "y":0, 'detected':False},
        {'name':"BedRoom",'color':"black","x":0,"y":0, 'detected':False},
        {'name':"Home",'color':"","x":-5,"y":7, 'detected':True}		           
        ]
        self.prevXpos = 0
        self.prevYpos = 0
        self.locationKnown = [[-5,7]]
    
# check if the room contained in msg is already visited, if so then the robot will move to that position 
    def room_check(self, color):
        for room in self.ROOMS:
            if color == room['color']:
                if room['detected'] == True:
                    return True
        return False
    
    def get_room_position(self, target_room):
        for room in self.ROOMS:
            if target_room == room['name']:
                if room['detected'] == True:
                    return [room["x"], room["y"]]
        return False

    def visited(seld):
        v_rooms = []
        for room in self.ROOMS:
            if room['detected'] == True:
                v_rooms.append(room['name'])
        return v_rooms

    def add_new_room(self, color, x, y):
        for room in self.ROOMS:
            if color == room['color']:
                room['detected'] = True
                room['x'] = int(x)
                room['y'] = int(y)
                name = str(room['name'])
		print("[ROOMS] discovered room:"+name)
 
    def get_name_position(self, x, y):
        for room in self.ROOMS:
            if (x == room['x'] and y == room['y']):
                return room['name']
        return False    

    def get_color_room(self, name):
        for room in self.ROOMS:
            if name == room['name']:
                return room['color']
        return False

    def random_pos(self):
        while True:
            tmpX = random.randint(-6,6)
            tmpY = random.randint(-8,5)
            if tmpX != self.prevXpos and tmpY != self.prevYpos:
                if not (3 > tmpX > 0 and tmpY > 0) or not(tmpX < 0 and tempY < -5):
                    self.prevXpos = tmpX
                    self.prevYpos = tmpY
                    return [tmpX, tmpY]
                
    def mrange(self, a, l):
        minA = a - l
        r = []
        for i in range(0,l*2+1):
            r.append(minA + i)
        return r

    def cancel_room(self):
        self.locationKnown.pop()

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
