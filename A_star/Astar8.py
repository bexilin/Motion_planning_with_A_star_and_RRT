from Queue import PriorityQueue
from math import *
import openravepy
import numpy as np

#defines a basic node class
class Node:
    def __init__(self,x_in,y_in,theta_in):
        self.x = x_in
        self.y = y_in
        self.theta = theta_in
        self.parent=[]
        self.g=0
        self.h=0
        self.f=0
    def printme(self):
        print "\tNode",":", "x =", self.x, "y =",self.y, "theta =", self.theta ,"g=",self.g, "h",self.h,"f",self.f


def c(n,m):
    return sqrt(pow(n.x-m.x,2)+pow(n.y-m.y,2)+pow(min(abs(n.theta-m.theta),2*pi-abs(n.theta-m.theta)),2))

def h(n):
    g=[2.6,-1.3,-pi/2]
    return sqrt(pow(n.x-g[0],2)+pow(n.y-g[1],2)+pow(min(abs(n.theta-g[2]),2*pi-abs(n.theta-g[2])),2))

def Astar8(start,goal,env,robot):
    start_node=Node(start[0],start[1],start[2])
    #start_node.g = start_node.h = start_node.f = 0
    start_node.g=0
    start_node.h=0
    start_node.f=0
    
    #initialize frontier, explored and collision list 
    frontier=PriorityQueue()
    explored=[]
    collision=[]
    
    frontier.put((start_node.f,start_node))

    x=[]
    
    while not frontier.empty():
          
          #get current node from frontier
          current_node=frontier.get()[1]

          #add to explored
          check=0
          for i in range(len(explored)):
              if abs(current_node.x-explored[i].x)<0.001 and abs(current_node.y-explored[i].y)<0.001 and abs(current_node.theta-explored[i].theta)<0.001 and current_node.g>=explored[i].g:
                 check=1
                 break
          
          if check == 1:
             continue
          explored.append(current_node)
             
          #If found the goal, return the path, explored and collision 
          if abs(current_node.x-goal[0])<0.001 and abs(current_node.y-goal[1])<0.001 and abs(current_node.theta-goal[2])<0.001:
             
             #generate pathset and pathcost
             pathset=[]
             pathcost=0
             pathset.append((current_node.x,current_node.y,current_node.theta))
             while current_node is not start_node:
                   for i in explored:
                       if current_node.parent==i:
                          for j in explored:
                              if current_node.parent==j and j.g<i.g:
                                 current_node.parent=j
                          pathset.append((current_node.parent.x,current_node.parent.y,current_node.parent.theta))
                          current_node=current_node.parent
                          continue
             pathset.reverse()
             for i in range(len(pathset)-1):
                 pathcost=pathcost+sqrt(pow(pathset[i][0]-pathset[i+1][0],2)+pow(pathset[i][1]-pathset[i+1][1],2)+pow(min(abs(pathset[i][2]-pathset[i+1][2]),2*pi-abs(pathset[i][2]-pathset[i+1][2])),2))
             
             #generate noncollisionset
             noncollisionset=[]
             for i in explored:
                 noncollisionset.append((i.x,i.y,i.theta))

             #generate collisionset
             collisionset=[]
             for i in collision:
                 collisionset.append((i.x,i.y,i.theta))
             
             return pathset, pathcost, noncollisionset, collisionset

          #Generate children
          children=[]
          #for i in [(-0.2,0.,0.),(0.2,0.,0.),(0.,-0.1,0.),(0.,0.1,0.),(0.2,0.1,0.),(-0.2,0.1,0.),(0.2,-0.1,0.),(-0.2,-0.1,0.),(0.,0.,-pi/4),(0.,0.,pi/4)]:
          for i in [(-0.2,0.,0.),(0.2,0.,0.),(0.,-0.1,0.),(0.,0.1,0.),(0.,0.,-pi/2),(0.,0.,pi/2),(0.2,0.1,0.),(-0.2,0.1,0.),(0.2,-0.1,0.),(-0.2,-0.1,0.),(0.2,0.,pi/2),(0.2,0.,-pi/2),(-0.2,0.,pi/2),(-0.2,0.,-pi/2),(0.,0.1,pi/2),(0.,0.1,-pi/2),(0.,-0.1,pi/2),(0.,-0.1,-pi/2),(0.2,0.1,pi/2),(0.2,-0.1,pi/2),(0.2,0.1,-pi/2),(0.2,-0.1,-pi/2),(-0.2,0.1,pi/2),(-0.2,0.1,-pi/2),(-0.2,-0.1,pi/2),(-0.2,-0.1,-pi/2)]:
              
              #Get new node position and id, renew increment
              new_node=Node(current_node.x+i[0],current_node.y+i[1],current_node.theta+i[2])
              
              #Make position sure within range
              if new_node.x<-3.4 or new_node.x>3.4:
                 continue
              if new_node.y<-1.4 or new_node.y>1.4:
                 continue
              if new_node.theta<-pi:
                 current_node.theta=current_node.theta+2*pi
              elif new_node.theta>5*pi/6:
                 current_node.theta=current_node.theta-2*pi
       
              #insert to children
              children.append(new_node)
         

          #loop through children
          for child in children:
              
              #create the f, g and h values
              child.g=current_node.g+c(current_node,child)
              child.h=h(child)
              child.f=child.g+child.h
              child.parent=current_node

              #check if child is already in frontier
              check=0
              for open_node in frontier.queue:
                  if abs(child.x-open_node[1].x)<0.001 and abs(child.y-open_node[1].y)<0.001 and abs(child.theta-open_node[1].theta)<0.001 and child.g>=open_node[1].g:
                     check=1
                     break
              if check == 1:
                 continue
              
              #check if child collides
              with env:
                   robot.SetTransform([[ cos(child.theta)  ,  -sin(child.theta)  ,  0.  ,  child.x ],
                            [sin(child.theta)  ,  cos(child.theta)  ,  0.  ,  child.y ],
                            [ 0.  ,  0.  ,  1.  ,   0.05 ],
                            [ 0.  ,  0.  ,  0.  ,   1.  ]])
                   if env.CheckCollision(robot):
                      collision.append(child)
                      x.append(env.plot3(points=np.array((child.x,child.y,0.05)),pointsize=10.0,colors=np.array((1,0,0))))
                      continue
              
              #add the child to frontier
              frontier.put((child.f,child))
              
              #plot new frontier point
              x.append(env.plot3(points=np.array((child.x,child.y,0.05)),pointsize=10.0,colors=np.array((0,0,1))))

    return 'failure'
