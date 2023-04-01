#!/usr/bin/env python3
import rospy
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import matplotlib.pyplot as plt

#A-star algo
import numpy as np

class Node:
  
  def __init__(self, parent = None, position = None):
    self.parent = parent
    self.position = position
    
    self.g = 0
    self.h = 0
    self.f = self.g + self.h
  
  def get_neighbours(self, current_Node):
    adjacent = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    neighbours = []
    for i in adjacent:
      pos = current_Node.position
      new_node = Node(current_Node, (pos[0] + i[0], pos[1] + i[1]))
      
      new_g = current_Node.g
      new_node.g = new_g  
      neighbours.append(new_node)
    return neighbours
  
  def get_min_cost_node(self, neighbours):
    min_cost_node = neighbours[0]
    min = min_cost_node.f
  
    for node in neighbours:
      if(node.f < min):
        min_cost_node = node
        min = node.f
        min_cost_node.parent = node.parent
      
    return min_cost_node
    
  def distance(self,NodeA, NodeB):
    posA = NodeA.position
    posB = NodeB.position
    
    if(posA[0] != posB[0] and posA[1] != posB[1]):
      dist = 14
    else:
      dist = 10
  
    return dist
    
  def heuristic(self, node, goal):
    pos = node.position
    dist = abs(goal[0] - pos[0]) + abs(goal[1] - pos[1])
    return dist

  def path(self, end_node, start):
    path = []
    current_node = end_node
    while (current_node.position[0] != start[0] or current_node.position[1] != start[1]):
        path.append(current_node.position)
        current_node = current_node.parent
    return path[::-1]
      
  def A_star(self, maze, start, end):
    open_list = []
    closed_list = []
    
    start_node = Node(None, start)
    start_node.f = start_node.g = start_node.h = 0
    start_node.parent = start_node
    
    open_list.append(start_node)
      
    while len(open_list) > 0:
      
      #Find the node in the open_list with lowest F_score
      current_node = Node(None, None)
      current_node = self.get_min_cost_node(open_list)
      
      #If we have reached the goal, return the path and reconstruct the path
      if(current_node.position[0] == end[0] and current_node.position[1] == end[1]):
        new_path = self.path(current_node, start)
        return new_path
        
      open_list.remove(current_node)
      closed_list.append(current_node)
      
      for neighbour in self.get_neighbours(current_node):
        
        #check if the neighbour is out of the grid
        if(neighbour.position[0] < 0 or neighbour.position[0] > len(maze)-1 or neighbour.position[1] < 0 or neighbour.position[1] > len(maze[0])-1):
          continue
        #First check if the neighbour exists or not
        grid_pos = neighbour.position
        if(maze[grid_pos[0]][grid_pos[1]] == 100):
          continue
        #check if the neighbour is in the closed_list
        if(neighbour.position in[node.position for node in closed_list]):
          continue
        
        #Calculate the cost of remaining neighbours
        tentative_g = neighbour.g + self.distance(neighbour, current_node)
        
        count = 0
        for i in open_list:
          if(neighbour.position[0] == i.position[0] and neighbour.position[1] == i.position[1]):
            count = count + 1
        
        if(count == 0):
            open_list.append(neighbour)
        elif(tentative_g > neighbour.g):
            neighbour.parent = current_node.parent
            continue
         
        #updating the gscore, fscore and parent of the neighbor
        neighbour.g = tentative_g
        neighbour.f = neighbour.g + self.heuristic(neighbour, end)
        neighbour.parent = current_node

#Publishing path on topic
def main():
    rospy.init_node('Astar_path')
    rospy.wait_for_service('/static_map')
    service = rospy.ServiceProxy("/static_map", GetMap)

    rospy.wait_for_service('static_map')
    get_map = rospy.ServiceProxy('static_map', GetMap)
    map_data = get_map().map
    map_array = np.array(map_data.data).reshape((map_data.info.height, map_data.info.width))

    start = (0,0)
    end = (49, 49)

    node = Node()
    p = node.A_star(map_array, start, end)
    print(p)
    
    path_pub = rospy.Publisher('path_topic', Path, queue_size=10)
    path = Path()
    path.header.frame_id = 'map'

    for(r, c) in p:
        pose_i = PoseStamped()
        pose_i.pose.position.x = c
        pose_i.pose.position.y = r
        path.poses.append(pose_i)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        path_pub.publish(path)
        rate.sleep()

if __name__ == '__main__':
    main()