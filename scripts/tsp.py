#!/usr/bin/env python

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from itertools import combinations

from utils import Utils
import numpy as np
import cv2
from skimage.draw import line

class PoI:
    def __init__(self, x, y, shelf_id=0, theta=0.0):
        self.x = x
        self.y = y
        self.shelf_id = shelf_id
        self.theta = theta #deg

class Tsp:
  def __init__(self,img,pois,shelves, path, debug_mode=False):

    self.debug_mode = debug_mode
    self.path = path

    #TODO: cancel paths that cross shelves from the distance matrix: do a feasible_pois and then calculates D,O and C on them
    #pros: no forbidden paths and less computational cost
    
    self.max_range = 1000
    self.min_range = 10

    self.pois = pois
    self.shelves = shelves

    self.gray_img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    self.shelves = shelves
    # distance weights calculation
    D, D_max,D_min = self.calc_distance_matrix(pois)
    D_norm,D_norm_max,D_norm_min = self.normalize_distance_matrix(D,D_max,D_min)

    # turning weights calculation (returns already normalized matrix)
    O = self.calc_orientation_weights(pois)
  
    #path brightness weigths calculation
    C, C_max, C_min = self.calc_heatmap_weights(pois)
    C_norm, C_norm_max, C_norm_min = self.normalize_heatmap_matrix(C,C_max,C_min)

    if self.debug_mode == True:
      for i in range(1,6):
        for j in range(1,6):
          for k in range(1,6):
            new_img = img.copy()
            self.k1 = i
            self.k2 = j
            self.k3 = k
            new_D = i*D_norm
            new_O = j*O
            new_C = k*C_norm

            sequence = run_tsp(np.add(np.add(new_D,new_O),new_C),new_img,pois)
            self.draw_solution(new_img,sequence,pois)
    else:
      sequence = run_tsp(np.add(np.add(D_norm,O),C_norm),img,pois)
      # sequence = run_tsp(D_norm,img,pois)
      self.draw_solution(img,sequence,pois)

    self.index_sequence = sequence

    self.final_path = self.order_pois()

  def draw_solution(self,img,sequence,pois):

    for poi in pois:
      Utils.draw_PoI(img,poi)
    pt1 = (pois[sequence[0]].x, pois[sequence[0]].y)
    pt2 = (pois[sequence[1]].x, pois[sequence[1]].y)
    Utils.draw_arrow(img,pt1,pt2,(0,0,255))
    
    for i in range(1,len(sequence)-1):
      pt1 = (pois[sequence[i]].x, pois[sequence[i]].y)
      pt2 = (pois[sequence[i+1]].x, pois[sequence[i+1]].y)
      Utils.draw_arrow(img,pt1,pt2,(0,255,0))

    pt1 = (pois[sequence[-1]].x, pois[sequence[-1]].y)
    pt2 = (pois[sequence[0]].x, pois[sequence[0]].y)
    Utils.draw_arrow(img,pt1,pt2,(0,0,255))

    Utils.show_img_and_wait_key('Final Route', img)
    # Utils.create_dir('/home/majo/catkin_ws/src/store-planner/scripts/results/' + str(self.k1) + '_'+ str(self.k2) + '_' + str(self.k3))
    if self.debug_mode == True:
      Utils.save_image(self.path + '/' + str(self.k1) + '_'+ str(self.k2) + '_' + str(self.k3) + '.png', img)
    else:
      Utils.save_image(self.path + '/final_path.png', img)


  def get_index_sequence(self):
    return self.index_sequence

  def get_final_path(self):
    return self.final_path

  def write_final_path(self):
    Utils.create_dir(self.path)
    with open(self.path + '/final_path.txt', 'w') as f:
      for pt in self.final_path:
        f.write(str(pt.x))
        f.write(',')
        f.write(str(pt.y))
        f.write(',')
        f.write(str(pt.theta))
        f.write(',')
        f.write(str(pt.shelf_id))
        f.write('\n')

  def order_pois(self):
    ordered_pois = list()

    for index in self.index_sequence:
      ordered_pois.append(self.pois[index])

    return ordered_pois

  def calc_distance_matrix(self, pois):
    k1 = 1
    D = np.zeros((len(pois),len(pois)),dtype=int)
    D_max = 0
    D_min = 9999999999
    
    for i in range(0,len(pois)):
      for j in range(i+1,len(pois)):
        D[i,j] = int(k1*Utils.calc_eucl_dist((pois[i].x,pois[i].y),(pois[j].x,pois[j].y)))
        D[j,i] = D[i,j]
      
        for shelf in self.shelves:

          tl = PoI(shelf.x,shelf.y)
          tr = PoI(shelf.x+shelf.w,shelf.y)
          bl = PoI(shelf.x,shelf.y+shelf.h)
          br = PoI(shelf.x+shelf.w,shelf.y+shelf.h)
          
          if self.path_cross_shelf(pois[i],pois[j],tl,tr) or self.path_cross_shelf(pois[i],pois[j],tl,bl) or self.path_cross_shelf(pois[i],pois[j],tr,br) or self.path_cross_shelf(pois[i],pois[j],bl,br):
            D[i,j] = 9999999999
            D[j,i] = 9999999999
            break

        if D[i,j] > D_max and D[i,j] != 9999999999:
          D_max = D[i,j]
          
        if D[i,j] < D_min and D[i,j] != 0 :
          D_min = D[i,j]

    return D, D_max, D_min


  def normalize_distance_matrix(self,D,D_max,D_min):
    D_norm = np.zeros((len(D),len(D)),dtype=int)

    D_norm_max = 0 
    D_norm_min = 9999999999

    old_range = (D_max - D_min) 
    new_range = (self.max_range - self.min_range)  

    for i in range(0,len(D_norm)):
      for j in range(i+1,len(D_norm)):
        if D[i,j] != 9999999999 and D[i,j] != 0 :
          D_norm[i,j] =  int((((D[i,j] - D_min) * new_range) / old_range) + self.min_range)
          D_norm[j,i] = D_norm[i,j]
          if D_norm[i,j] < D_norm_min:
            D_norm_min = D_norm[i,j]
            
          if D_norm[i,j] > D_norm_max:
            D_norm_max = D_norm[i,j]
            
        else:
          D_norm[i,j] = 9999999999 if D[i,j] == 9999999999 else 0
          D_norm[j,i] = 9999999999 if D[i,j] == 9999999999 else 0

    return D_norm, D_norm_max, D_norm_min

  def normalize_heatmap_matrix(self,C,C_max,C_min):

    C_norm = np.zeros((len(C),len(C)),dtype=int)

    C_norm_max = 0 
    C_norm_min = 9999999999

    old_range = (C_max - C_min) 
    new_range = (self.max_range - self.min_range)  

    for i in range(0,len(C_norm)):
      for j in range(i+1,len(C_norm)):
        C_norm[i,j] =  int((((C[i,j] - C_min) * new_range) / old_range) + self.min_range)
        C_norm[j,i] = C_norm[i,j]
        if C_norm[i,j] < C_norm_min:
          C_norm_min = C_norm[i,j]
          
        if C_norm[i,j] > C_norm_max:
          C_norm_max = C_norm[i,j]
          
    return C_norm, C_norm_max, C_norm_min


  def calc_orientation_weights(self,pois):
    k2 = 1
    O = np.zeros((len(pois),len(pois)),dtype=int)
    ratio = int((self.max_range - self.min_range) / 3)
    for i in range(0,len(pois)):
      for j in range(i+1,len(pois)):
        if i != j:
          if pois[i].theta == -1 or pois[j].theta == -1:
            O[i,j] = 0
            O[j,i] = O[i,j]
          
          else:
            theta = int(abs(pois[i].theta - pois[j].theta)) #we can have only 3 values: 90, 180, 270
            
            # print('(i,j) theta and difference: ',i,j, pois[i].theta, pois[j].theta,pois[i].theta - pois[j].theta )
            if theta == 90:     
              O[i,j] = int(k2*ratio)
              O[j,i] = O[i,j]
            if theta == 180:     
              O[i,j] = int(2*k2*ratio)
              O[j,i] = O[i,j]
            if theta == 270:     
              O[i,j] = int(3*k2*ratio)
              O[j,i] = O[i,j]
            if theta != 0 and theta != 90 and theta != 180 and theta != 270:
              print('wrong value theta!') #just for check

    return O

  def calc_heatmap_weights(self, pois):
    k3 = 1
    C = np.zeros((len(pois),len(pois)))

    C_max = 0 
    C_min = 9999999999

    for i in range(0,len(pois)):
      for j in range(i+1,len(pois)):
        C[i,j] = k3*(self.path_brightness(pois[i],pois[j])) 
        C[j,i] = C[i,j]

        if C[i,j] > C_max:
          C_max = C[i,j]
          
        if C[i,j] < C_min:
          C_min = C[i,j]

    return C, C_max, C_min

  def onSegment(self, p, q, r):
    if ( (q.x <= max(p.x, r.x)) and (q.x >= min(p.x, r.x)) and
           (q.y <= max(p.y, r.y)) and (q.y >= min(p.y, r.y))):
        return True
    return False
 
  def orientation(self,p, q, r):

    val = (float(q.y - p.y) * (r.x - q.x)) - (float(q.x - p.x) * (r.y - q.y))
    if (val > 0):  
        # Clockwise orientation
        return 1
    elif (val < 0):
        # Counterclockwise orientation
        return 2
    else:  
        # Colinear orientation
        return 0

  def path_cross_shelf(self,p1,q1,p2,q2):

    o1 = self.orientation(p1, q1, p2)
    o2 = self.orientation(p1, q1, q2)
    o3 = self.orientation(p2, q2, p1)
    o4 = self.orientation(p2, q2, q1)

    # General case
    if ((o1 != o2) and (o3 != o4)):
        return True

    # Special Cases

    # p1 , q1 and p2 are colinear and p2 lies on segment p1q1
    if ((o1 == 0) and self.onSegment(p1, p2, q1)):
        return True

    # p1 , q1 and q2 are colinear and q2 lies on segment p1q1
    if ((o2 == 0) and self.onSegment(p1, q2, q1)):
        return True

    # p2 , q2 and p1 are colinear and p1 lies on segment p2q2
    if ((o3 == 0) and self.onSegment(p2, p1, q2)):
        return True

    # p2 , q2 and q1 are colinear and q1 lies on segment p2q2
    if ((o4 == 0) and self.onSegment(p2, q1, q2)):
        return True
 
    # If none of the cases
    return False

  def path_brightness(self, pt1, pt2):
    rr, cc = line(pt1.y,pt1.x,pt2.y,pt2.x)
    result = float(np.sum(self.gray_img[rr,cc] == 0)) / float(len(rr))
    return result

  ##ORTOOL SOLVER

def create_data_model(D):
    """Stores the data for the problem."""
    data = {}
    data['distance_matrix'] = D
    # yapf: disable

    data['num_vehicles'] = 1
    data['depot'] = 0
    return data


def print_solution(manager, routing, solution, img, pois):
    """Prints solution on console."""
    print('Objective: {} miles'.format(solution.ObjectiveValue()))
    index = routing.Start(0)
    plan_output = 'Route for vehicle 0:\n'
    route_distance = 0
    sequence = list()
    while not routing.IsEnd(index):
        plan_output += ' {} ->'.format(manager.IndexToNode(index))
        sequence.append(index)
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += ' {}\n'.format(manager.IndexToNode(index))
    print(plan_output)
    plan_output += 'Route distance: {}miles\n'.format(route_distance)

    return sequence


def run_tsp(D,img,pois):
  """Entry point of the program."""
  # Instantiate the data problem.
  data = create_data_model(D)

  # Create the routing index manager.
  manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                          data['num_vehicles'], data['depot'])

  # Create Routing Model.
  routing = pywrapcp.RoutingModel(manager)


  def distance_callback(from_index, to_index):
      """Returns the distance between the two nodes."""
      # Convert from routing variable Index to distance matrix NodeIndex.
      from_node = manager.IndexToNode(from_index)
      to_node = manager.IndexToNode(to_index)
      return data['distance_matrix'][from_node][to_node]

  transit_callback_index = routing.RegisterTransitCallback(distance_callback)

  # Define cost of each arc.
  routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

  # Setting first solution heuristic.
  search_parameters = pywrapcp.DefaultRoutingSearchParameters()
  search_parameters.first_solution_strategy = (
      routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

  # Solve the problem.
  solution = routing.SolveWithParameters(search_parameters)

  # Print solution on console.
  if solution:
      sequence = print_solution(manager, routing, solution, img, pois)
      return sequence

  return 0
      