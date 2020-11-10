#!/usr/bin/env python
import rospy
import roslib
import sys
import numpy as np
import random
import math
from mpl_toolkits import mplot3d 
from geometry_msgs.msg import Point
# from pahap.srv import displayData, displayDataResponse
from pahap.srv import rrt, rrtResponse

from numpy import expand_dims
import sys
from sklearn import mixture
import matplotlib.pyplot as plt
import numpy.linalg as linalg
import copy
# from chinesepostman import eularian, network

show_animation = True

class rrt_server:
  
  class Node: 
    """ RRT Node """
    def __init__(self, x, y, phi):
      self.x = x
      self.y = y
      self.phi = phi
      self.path_x = []
      self.path_y = []
      self.path_phi = []
      self.parent = None

  def __init__(self,
                rand_area,
                expand_dis=0.16,
                path_resolution=0.02,
                angle_resolution=0.085,
                goal_sample_rate=5,
                max_iter=500
                ):
    """
    Setting Parameter: 
    start:Start Position [x,y]
    goal:Goal Position [x,y]
    randArea:Random Sampling Area [min,max]  """

    # self.start = [] #self.Node(start[0], start[1], start[2])
    # self.end = [] #self.Node(goal[0], goal[1], goal[2])
    self.min_x = rand_area[0]
    self.max_x = rand_area[1]
    self.min_y = rand_area[2]
    self.max_y = rand_area[3]
    self.min_phi = rand_area[4]
    self.max_phi = rand_area[5]
    self.expand_dis = expand_dis
    self.path_resolution = path_resolution
    self.angle_resolution = angle_resolution
    self.goal_sample_rate = goal_sample_rate
    self.max_iter = max_iter
    self.node_list = []    
    rospy.init_node('rrt_motionPlanning_node')    # initialize a node
    

  def callSpawnServiceTopic(self):
    rospy.Service('rrt_motionPlanning', rrt, self.rrt_response)   # advertise a service
    r = rospy.Rate(50)
    while not rospy.is_shutdown():
      r.sleep()

  def rrt_response(self, data):
    if data.cmd:  
      # print('check the si asdfaosdioasijdolasjdlasjdl;jfl;sajf')
      clusterSet = []
      centers = []
      X = []
      Y = []
      for j in range(len(data.clusterSet)):  
        D = np.array([[0,0]])
        x = []
        y = []
        for i in range(len(data.clusterSet[j].cluster)):
          insert1 = np.array([[data.clusterSet[j].cluster[i].x, data.clusterSet[j].cluster[i].y]])
          # print('check the data:', data.clusterSet[j].cluster[i].x, data.clusterSet[j].cluster[i].y)  
          x.append(data.clusterSet[j].cluster[i].x)
          y.append(data.clusterSet[j].cluster[i].y)
          if i == 0:
            D[0,0] = insert1[0,0]
            D[0,1] = insert1[0,1]
          else: 
            D = np.append(D, insert1, axis=0)
        X.append(x)
        Y.append(y)
        center = np.mean(D, axis=0)
        if len(centers) ==0:
          centers = np.array([center])
        else: 
          centers = np.append(centers, [center], axis=0)
        clusterSet.append(D)


      # start = self.Node(data.vocpp_path[0].x, data.vocpp_path[0].y, data.vocpp_path[0].z)
      start = self.Node(-0.8, 0.03, 0.0)
      # for i in range(20):
      #   start = self.Node(random.uniform(-0.55, -2.0), random.uniform(-0.65, 0.2), 0.0)
      #   # test the PIBC algorithm
      #   print("The position of checking points:", start.x, start.y, start.phi, '\n')
        
      #   # draw the boundaries

      #   check, insect, closePoint = self.ara_check_collision(start, clusterSet)
      #   if check:
      #     print("the checking point lies inside the boundaries")
      #   else:
      #     print('Lie Outside----------')
        
      #   for i in range(len(X)):
      #     color = ['b','r', 'y', 'm', 'g', 'c', 'k', 'w']
      #     for j in range(len(X[i])):
      #       # print('check the value of x, y: ', X[i][j], Y[i][j])
      #       plt.scatter(X[i][j], Y[i][j], c=color[i])
             
      #   # draw the starting point and target
      #   plt.scatter(start.x, start.y, c='k')
      #   plt.scatter(centers[0][0], centers[0][1], c='k')
      #   plt.scatter(centers[1][0], centers[1][1], c='k')
      #   plt.scatter(centers[2][0], centers[2][1], c='k')
      #   plt.scatter(insect[0], insect[1], c='c')
      #   plt.scatter(insect[2], insect[3], c='c')
      #   plt.scatter(closePoint[0], closePoint[1], c='g')


      #   plt.axis("equal")
      #   # plt.axis([-2, 15, -2, 15])
      #   plt.axis([-0.5, -2.1, -0.8, 0.3])
      #   # plt.grid(True)
      #   plt.show()



      # target = self.Node(data.vocpp_path[-1][0], data.vocpp_path[-1][1], data.vocpp_path[-1][2])
      # target = self.Node(data.vocpp_path[2].x, data.vocpp_path[2].y, data.vocpp_path[2].z)
      target = self.Node(-1.88, -0.45, 0.0)
      path = self.planning(clusterSet, start, target, animation=show_animation)
     


      # for k in range(len(neighNumber)):
      return rrtResponse(True, None, None, None)      


  def line_though2Points(self, p1, p2):
    a = (p1[1] - p2[1])/(p1[0] - p2[0])
    b = (p1[0]*p2[1] - p1[1]*p2[0])/(p1[0] - p2[0])
    return np.array([a,b])

  def closePoint_twoPoints(self, node, p1, p2):  # return the closer point
    dis1 = np.sqrt(np.power((node[0]-p1[0]),2)+np.power((node[1] - p1[1]),2))
    dis2 = np.sqrt(np.power((node[0]-p2[0]),2)+np.power((node[1] - p2[1]),2))
    if dis1 >= dis2:
      return p2
    else: 
      return p1

  def lineBound_intersect(self, lineCo, bound): # intersection between one line and one boudnary
    distance =0.0
    min_distance =0.0
    check_distance=0.0
    interPoints = []
    point1 = []
    point2 = []
    # convert a list to an array
    for i in range(len(bound)):
      distance = np.absolute(bound[i,1]-lineCo[0]*bound[i,0]-lineCo[1])/np.sqrt(np.power(lineCo[0],2)+np.power(lineCo[1],2))
      if (i==0) :
        min_distance = distance;
        point1 = bound[i]
      
      if (min_distance > distance): 
        min_distance = distance
        point1 = np.array([bound[i][0],bound[i][1]])
      
    min_distance = 2.0
    # find the second intersection
    for i in range(len(bound)):
      # check with the first point - should be far way from it
      check_distance = np.sqrt(np.power((bound[i,0] -point1[0]),2) + np.power((bound[i,1] -point1[1]),2))
      if (check_distance > 0.1):
        distance = np.absolute(bound[i,1] - lineCo[0]*bound[i,0]-lineCo[1])/np.sqrt(np.power(lineCo[0],2) +np.power(lineCo[1],2))
        if (min_distance > distance):
          min_distance = distance
          point2 = np.array([bound[i][0],bound[i][1]])     
      
    interPoints = np.array([point1[0], point1[1], point2[0], point2[1]])
    return interPoints

  def planning(self, boudnaries, start, target, animation=True):
    """ rrt path planning
    animation: flag for animation on or off  """
    self.node_list = [start]
    print('the target location:', target.x, target.y, target.phi)
    for i in range(self.max_iter):
      # print('to check the planning', i)

      rnd_node = self.get_random_node(target)
      # return the index of the nearest node
      nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
      nearest_node = self.node_list[nearest_ind]

      new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

      # if self.check_collision(new_node, self.obstacle_list):
      notClo, insect, closP = self.ara_check_collision(new_node, boudnaries)
      if notClo:
        self.node_list.append(new_node)

      # if animation and i % 3 == 0:
      self.draw_graph(start, target, boudnaries, rnd_node)

      if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y,
                                self.node_list[-1].phi, target) <= self.expand_dis:
        
        final_node = self.steer(self.node_list[-1], target, self.expand_dis)
        if self.ara_check_collision(final_node, boudnaries):
            print('get the destination')
            return self.generate_final_course(len(self.node_list) - 1, target)

      # if animation and i % 3:
      self.draw_graph(start, target, boudnaries, rnd_node)

    return None  # cannot find path
 
 
  def steer(self, from_node, to_node, extend_length=float("inf")):

    new_node = self.Node(from_node.x, from_node.y, from_node.phi)
    d, theta, d_phi = self.calc_distance_and_angle(new_node, to_node)

    new_node.path_x = [new_node.x]
    new_node.path_y = [new_node.y]
    new_node.path_phi = [new_node.phi]

    if extend_length > d:
        extend_length = d
    # find the number of step to extend by rounding down ...
    n_expand = int(math.floor(extend_length / self.path_resolution))

    for i in range(n_expand):
        new_node.x += self.path_resolution * math.cos(theta)
        new_node.y += self.path_resolution * math.sin(theta)
        new_node.phi += d_phi
        new_node.path_x.append(new_node.x)
        new_node.path_y.append(new_node.y)
        new_node.path_phi.append(new_node.phi)


    d, e, f = self.calc_distance_and_angle(new_node, to_node)
    
    if d <= self.path_resolution:
        new_node.path_x.append(to_node.x)
        new_node.path_y.append(to_node.y)
        new_node.path_phi.append(to_node.phi)
        new_node.x = to_node.x
        new_node.y = to_node.y
        new_node.phi = to_node.phi

    new_node.parent = from_node

    return new_node

  def generate_final_course(self, goal_ind, target):
      # path = [[self.end.x, self.end.y, self.end.phi]]
      path = [[target.x, target.y, target.phi]]
      node = self.node_list[goal_ind]
      while node.parent is not None:
          path.append([node.x, node.y, node.phi])
          node = node.parent
      path.append([node.x, node.y, node.phi])

      return path

  def calc_dist_to_goal(self, x, y, phi, goal):
      dx = x - goal.x
      dy = y - goal.y
      dphi = phi - goal.phi
      dis = np.sqrt(np.power(dx,2) + np.power(dy,2) + np.power(dphi,2))
      return dis

  def get_random_node(self, target):
      if random.randint(0, 100) > self.goal_sample_rate:
          rnd = self.Node(
              random.uniform(self.min_x, self.max_x),
              random.uniform(self.min_y, self.max_y),
              random.uniform(self.min_phi, self.max_phi))
      else:  # goal point sampling
          rnd = self.Node(target.x, target.y, target.phi)
      return rnd

  def draw_graph(self, start, end, boundaries, rnd=None):
    plt.clf()
    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect(
        'key_release_event',
        lambda event: [exit(0) if event.key == 'escape' else None])
    
    # syntax for 3-D projection 
    # ax = plt.axes(projection ='3d') 
    
    # draw the boundaries
    X =[]
    Y =[]
    for j in range(len(boundaries)):
      x = []
      y = []
      for i in range(len(boundaries[j])):
        x.append(boundaries[j][i][0])
        y.append(boundaries[j][i][1])
      X.append(x)
      Y.append(y)
    
    for i in range(len(X)):
      color = ['b','r', 'y', 'm', 'g', 'c', 'k', 'w']
      for j in range(len(X[i])):
        # print('check the value of x, y: ', X[i][j], Y[i][j])
        plt.scatter(X[i][j], Y[i][j], c=color[i])
    
    if rnd is not None:
        plt.plot(rnd.x, rnd.y, "^k")
        # ax.scatter(rnd.x, rnd.y, rnd.phi, c='r')
    for node in self.node_list:
        if node.parent:
            plt.plot(node.path_x, node.path_y, "-g")
            # ax.plot3D(node.x, node.y, node.phi, c='g')
    
    # draw the starting point and target
    plt.scatter(-0.8, 0.03, c='k')
    plt.scatter(-1.88, -0.45, c='k')


    # ax.plot3D(start.x, start.y, "xr")
    # ax.plot3D(end.x, end.y, "xr")
    plt.axis("equal")
    # plt.axis([-2, 15, -2, 15])
    plt.axis([-0.5, -2.5, -0.8, 0.3])
    plt.grid(True)
    # plt.show()
    plt.pause(0.02)
    # ax.pause(0.05)

  @staticmethod
  def get_nearest_node_index(node_list, rnd_node):
      dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2 + (node.phi - rnd_node.phi)**2
               for node in node_list]
      minind = dlist.index(min(dlist))

      return minind

  # @staticmethod
  def ara_check_collision(self, node, data):
    if node is None:
        return False
    # calculate all the other conner of the robot
    # for h in range(len(node)):
    # roPos = self.robot_positions(node)
    # roPos = np.array([[node.x, node.y]])
    # calculate the center point of the clusters
    cenPointSet = []

    for i in range(len(data)):
      # print('check the data: ', data[i])
      cen = np.mean(data[i], axis=0)
      # print('check the cen:', cen)
      if i == 0:
          cenPointSet = np.array([cen])
      else:
          cenPointSet = np.append(cenPointSet, [cen], axis=0)

    closetPoints = []
    
    # check in the boudnary for each robot point
    for pos in range(len(node.path_x)):
      # select 3 closest clusters around the robot point by their center
      clus_index = np.array([0,0,0])
      clus_dis = np.array([1.5,1.5,1.5])
      d_max = 1.5
      dmax_index = 0
      # find potential cluster around the robot point
      if len(cenPointSet) < 3:
          clus_index[2] = -1
          clus_disp[2] = 0.0
      for i in range(len(cenPointSet)):
        dis = self.distance([node.path_x[pos],node.path_y[pos]], cenPointSet[i])
        if dis < d_max:
          d_max = dis
          clus_dis[dmax_index] = dis
          clus_index[dmax_index] = i
          for k in range(len(clus_dis)):
            if d_max < clus_dis[k]:
              d_max = clus_dis[k]
              dmax_index = k

      # check the robot point whether it belongs to one of the clusters
      inside1Clus = False
      for j in range(len(clus_index)):
        if not (clus_index[j] == -1):
          data2 = data[clus_index[j]]
          # check_inside, closetPoints = self.check_inside(roPos[pos], data2)
          check_inside, inS, clP = self.check_inside([node.path_x[pos],node.path_y[pos]], data2)

          if check_inside:
            inside1Clus = inside1Clus or check_inside                      
            # return False, inS, clP   # Collision
      if not inside1Clus:
        return False, inS, clP   # Collision
    
    return True, inS, clP  # Safe



  def distance(self, pos1, pos2):
      return np.sqrt(np.power((pos1[0] - pos2[0]),2) + np.power((pos1[1]-pos2[1]),2))


  def robot_positions(self, node): 
      x = node.x
      y = node.y
      phi = node.phi
      d = 0.12 #0.4 # distance between two feet
      l = 0.05 #0.18 # length of the foot
      w = 0.07 #0.22 # width of the foot
      dc = np.sqrt(np.power((l/2),2) + np.power((w/2),2))
      alpha = np.arctan(w/d)
      p1_x = x + dc*np.cos(-alpha + phi)
      p1_y = y + dc*np.sin(-alpha + phi)
      PoRo = np.array([[p1_x, p1_y]])

      p2_x = x + dc*np.cos(alpha + phi)
      p2_y = y + dc*np.sin(alpha + phi)
      PoRo = np.append(PoRo, [[p2_x, p2_y]], axis=0)

      p3_x = x + dc*np.cos(np.pi - alpha + phi)
      p3_y = y + dc*np.sin(np.pi - alpha + phi)
      PoRo = np.append(PoRo, [[p3_x, p3_y]], axis=0)

      p4_x = x + dc*np.cos(-np.pi + alpha + phi)
      p4_y = y + dc*np.sin(-np.pi + alpha + phi)
      PoRo = np.append(PoRo, [[p4_x, p4_y]], axis=0)

      p5_x = -d*np.cos(phi) + x + dc*np.cos(-alpha + phi)
      p5_y = -d*np.sin(phi) + y + dc*np.sin(-alpha + phi)
      PoRo = np.append(PoRo, [[p5_x, p5_y]], axis=0)

      p6_x = -d*np.cos(phi) + x + dc*np.cos(alpha + phi)
      p6_y = -d*np.sin(phi) + y + dc*np.sin(alpha + phi)
      PoRo = np.append(PoRo, [[p6_x, p6_y]], axis=0)

      p7_x = -d*np.cos(phi) + x + dc*np.cos(np.pi - alpha + phi)
      p7_y = -d*np.sin(phi) + y + dc*np.sin(np.pi - alpha + phi)
      PoRo = np.append(PoRo, [[p7_x, p7_y]], axis=0)

      p8_x = -d*np.cos(phi) + x + dc*np.cos(-np.pi + alpha + phi)
      p8_y = -d*np.sin(phi) + y + dc*np.sin(-np.pi + alpha + phi)
      PoRo = np.append(PoRo, [[p8_x, p8_y]], axis=0)

      return PoRo
  
  @staticmethod
  def calc_distance_and_angle(from_node, to_node):
    dx = to_node.x - from_node.x
    dy = to_node.y - from_node.y
    d = math.hypot(dx, dy)
    delta_phi = to_node.phi - from_node.phi
    d_d=0.0
    if (d < 0.02) and (delta_phi > 0.088):
      return False, 0.0, 0.0, 0.0
    elif (d>=0.02):
      d_d = delta_phi*self.path_resolution/d
      if d_d > 0.088:
        return return False, 0.0, 0.0, 0.0
    else: 
      theta = math.atan2(dy, dx)
    return d, theta, phi_res

 
  def check_inside(self, node, data): # data = boundary. node = point
    cen = np.mean(data, axis=0)
    cen_x = cen[0]
    cen_y = cen[1]
    # distance from center point to node
    dcn = self.distance(node, [cen_x, cen_y])
    # find four closest points to the node - point
    # neigh_index = np.array([0,0,0,0])
    # dist4neighbors = [] #np.array([1.0,1.0,1.0,1.0])
    # max_distance = 0.0
    # point_index = 0
    # closetPoints = []
    
    # find 4 closest point around the robot point in each cluster
    # for k in range(len(data)):
    #   dis = self.distance(node, data[k])
    #   if k < 3:
    #     dist4neighbors.append(dis)
    #     if k ==0:
    #       closetPoints = np.array([data[k]])
    #     else:
    #       closetPoints = np.append(closetPoints, [data[k]], axis=0)
    #     if dis > max_distance:
    #       max_distance = dis
    #       point_index = k   
    #   elif dis < max_distance:
    #     max_distance = dis
    #     dist4neighbors[point_index] = dis
    #     closetPoints[point_index][0] = data[k][0]
    #     closetPoints[point_index][1] = data[k][1]
    #     for m in range(len(dist4neighbors)):
    #       if max_distance < dist4neighbors[m]:
    #         max_distance = dist4neighbors[m]
    #         point_index = m

    # d_nc = [] 
    # for g in range(len(dist4neighbors)):
    #   d_cn = self.distance(closetPoints[g], [cen_x, cen_y])
    #   d_nc.append(d_cn)
    
    # # boolean check set
    # # bcs = [False, False, False, False]
    # bcs = []
    # inside = True
    # noise_check = False
    # for h in range(len(d_nc)):
    #   if (dcn - d_nc[h]) < 0.008:
    #     inside = True
    #   else:
    #     inside = False
    #     noise_check = True
    #   if noise_check:
    #     dis_aver = np.mean(d_nc)
    #     if (dcn - dis_aver) < 0.004:
    #       inside = True
    #   bcs.append(inside)
    # check = True
    # for u in range(len(bcs)):
    #     check = check and bcs[u]
    # if check:
    #     return True, closetPoints # Inside 
    
    # the second method to determine the inside boundary
    line = self.line_though2Points(node, cen)
    insect = self.lineBound_intersect(line, data)
    closePoint = self.closePoint_twoPoints(node, [insect[0],insect[1]], [insect[2], insect[3]])
    if self.distance(closePoint, cen) > self.distance(node, cen):
      print('check inside the check-inside function')
      return True, insect, closePoint
    else:
      print('Inside the check-inside function - NOT INSIDE')
      return False, insect, closePoint


    # return False, closetPoints  # outside



def main(args):
  
  rrtMotionPlanner = rrt_server(rand_area=[-2.5, -0.5, -0.8, 0.8, -1.75, 0.52])
  rrtMotionPlanner.callSpawnServiceTopic()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")




if __name__ == '__main__':

    main(sys.argv)
