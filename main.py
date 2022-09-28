from math import dist
from random import randint
import numpy as np
from tqdm import trange
import cv2
WIDTH=720
HEIGHT=720
FPS=60
TIMESTRETCH=0.05
LENGTH=10
FRAMES=FPS*LENGTH
SUBSTEPS=2
OBJECTNUMBER=2
class Vec2:
    def __init__(self,x,y) -> None:
        self.x=x
        self.y=y
    def __sub__(self,other):
        if type(other)==type(self):
            return Vec2(self.x-other.x,self.y-other.y)
        else:
            return Vec2(self.x-other,self.y-other)
    def __add__(self,other):
        if type(other)==type(self):
            return Vec2(self.x+other.x,self.y+other.y)
        else:
            return Vec2(self.x+other,self.y+other)
    def __div__(self,other):
        if type(other)==type(self):
            return Vec2(self.x/other.x,self.y/other.y)
        else:
            return Vec2(self.x/other,self.y/other)
    def __mul__(self,other):
        if type(other)==type(self):
            return Vec2(self.x*other.x,self.y*other.y)
        else:    
            return Vec2(self.x*other,self.y*other)
    def __rsub__(self,other):
        if type(other)==type(self):
            return Vec2(self.x-other.x,self.y-other.y)
        else:
            return Vec2(self.x-other,self.y-other)
    def __radd__(self,other):
        if type(other)==type(self):
            return Vec2(self.x+other.x,self.y+other.y)
        else:
            return Vec2(self.x+other,self.y+other)
    def __rdiv__(self,other):
        if type(other)==type(self):
            return Vec2(self.x/other.x,self.y/other.y)
        else:
            return Vec2(self.x/other,self.y/other)
    def __rmul__(self,other):
        if type(other)==type(self):
            return Vec2(self.x*other.x,self.y*other.y)
        else:
            return Vec2(self.x*other,self.y*other)
    def tt(self):
        return (self.x,self.y)
    def __str__(self):
        return "Vec2("+str(self.x)+","+str(self.y)+")"
constraintPosition=Vec2(WIDTH/2,HEIGHT/2)
constraintRadius=200
constraintMap=np.zeros((WIDTH,HEIGHT,3),np.uint8)
for x in range(WIDTH):
    for y in range(HEIGHT):
        if dist(constraintPosition.tt(),(x,y))>constraintRadius:
            constraintMap[x][y]=(25, 25, 25)
class Object:
    position_current:Vec2
    position_old:Vec2
    acceleration:Vec2
    radius:float
    color:tuple
    def updatePosition(self,dt):
        velocity=self.position_current-self.position_old
        # fd=0.025*velocity*velocity*self.radius

        #save current position
        self.position_old=self.position_current
        #perform verlet integration
        self.position_current=self.position_current+velocity+self.acceleration*dt*dt
        #reset acceleration
        self.acceleration=0
    
    def accelerate(self,acc):
        self.acceleration+=acc

    
def length(v:Vec2):
    return (v.x**2+v.y**2)**0.5
class Solver:
    gravity=Vec2(0,1000.0)
    # objects:Object
    def update(self,dt,objects):
        self.applyGravity(objects)
        for n in range(SUBSTEPS):
            self.updatePositions(dt/SUBSTEPS,objects)
        self.applyConstraint(objects)
    def updatePositions(self,dt,objects):
        for n in range(len(objects)):
            objects[n].updatePosition(dt)
    def applyGravity(self,objects):
        for n in range(len(objects)):
            objects[n].accelerate(self.gravity)
    def applyConstraint(self,objects):

        for n in range(len(objects)):
            obj=objects[n]
            toObj=obj.position_current-constraintPosition
            distance=length(toObj)
            if distance>(constraintRadius-obj.radius):
                n=Vec2(toObj.x/distance,toObj.y/distance)
                obj.position_current=constraintPosition+n*(distance-obj.radius*0.5)

def render(objects):
    pixels=np.copy(constraintMap)
            
    for n in objects:
        x=round(n.position_current.x)
        y=round(n.position_current.y)
        r=round(n.radius)
        for x1 in range(x-r,x+r):
            for y1 in range(y-r,y+r):
                if dist((x,y),(x1,y1))<r:
                    if x1<WIDTH and x1>=0 and y1<HEIGHT and y1>=0:
                        pixels[y1][x1]=n.color
    return pixels
def main():
    o=[]
    for n in range(OBJECTNUMBER):
        o.append(Object())
        o[n].position_old=Vec2(WIDTH/2,HEIGHT/2)
        
        o[n].position_current=Vec2(WIDTH/2,HEIGHT/2)+Vec2(randint(-40,40),randint(-40,40))
        o[n].acceleration=Vec2(0,0)
        o[n].radius=20
        o[n].color=(255,255,255)
    fr=[]
    s=Solver()
    
    video=cv2.VideoWriter("video.mp4",cv2.VideoWriter_fourcc(*"mp4v"),FPS,(WIDTH,HEIGHT))
    for n in trange(FRAMES):
        s.update(TIMESTRETCH/FPS,o)
        rr=render(o)
        
        video.write(rr)
    video.release()
if __name__=="__main__":
    main()