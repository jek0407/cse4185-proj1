###### Write Your Library Here ###########
from collections import *
from math import *
from heapq import *
import itertools

#########################################


def search(maze, func):
    return {
        "bfs": bfs,
        "astar": astar,
        "astar_four_circles": astar_four_circles,
        "astar_many_circles": astar_many_circles
    }.get(func)(maze)


# -------------------- Stage 01: One circle - BFS Algorithm ------------------------ #

def bfs(maze):
    """
    [문제 01] 제시된 stage1의 맵 세가지를 BFS Algorithm을 통해 최단 경로를 return하시오.(20점)
    """
    start_point=maze.startPoint()

    path=[]

    ####################### Write Your Code Here ################################
    end_point = None
    visited = set()         
    frontier = deque()
    solution = {}
    frontier.append(start_point)
    solution[start_point] = start_point

    while len(frontier) > 0 :
        x, y = frontier.popleft()

        for (row, col) in maze.neighborPoints(x, y):
            if (row, col) not in visited:
                solution[row, col] = x, y
                frontier.append((row, col))
                visited.add((row,col))
            if maze.isObjective(row, col):
                end_point = (row, col)
                break
            
    x, y = end_point
    while (x, y) != start_point:   
        path.append((x, y))
        x, y = solution[x, y]
       # print(f"debug {x, y}")
    path.append(start_point)

    path.reverse()
    return path
    ############################################################################

class Node:
    def __init__(self,parent,location):
        self.parent=parent
        self.location=location #현재 노드

        self.obj=[] #

        # F = G+H
        self.f=0
        self.g=0
        self.h=0

    def __eq__(self, other):
        return self.location==other.location and str(self.obj)==str(other.obj)

    def __le__(self, other):
        return self.g+self.h<=other.g+other.h

    def __lt__(self, other):
        return self.g+self.h<other.g+other.h

    def __gt__(self, other):
        return self.g+self.h>other.g+other.h

    def __ge__(self, other):
        return self.g+self.h>=other.g+other.h


# -------------------- Stage 01: One circle - A* Algorithm ------------------------ #

def manhatten_dist(p1,p2):
    return abs(p1[0]-p2[0])+abs(p1[1]-p2[1])

def astar(maze, heuristic_func=manhatten_dist):

    """
    [문제 02] 제시된 stage1의 맵 세가지를 A* Algorithm을 통해 최단경로를 return하시오.(20점)
    (Heuristic Function은 위에서 정의한 manhatten_dist function을 사용할 것.)
    """

    start_point=maze.startPoint()

    end_point=maze.circlePoints()[0]
    path=[]

    ####################### Write Your Code Here ################################
    iteration = 0
    max_iteration = len(maze.mazeRaw)**2
    start_node = Node(None, start_point)
    start_node.g = 0 ##
    end_node = Node(None, end_point)
    end_node.g = end_node.h = 0 ##
    start_node.h = heuristic_func(start_node.location, end_node.location)

    open_list = []
    closed_list = []

    open_list.append(start_node)
 
    while len(open_list) > 0:
        iteration += 1
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item < current_node:
                current_node = item
                current_index = index
        if iteration > max_iteration: 
            print("failure")
            return path

        open_list.pop(current_index)
        closed_list.append(current_node)

        # Goal test
        if current_node == end_node: 
            current = current_node
            while current is not None:
                path.append(current.location)
                current = current.parent
            break

        children = []
        x, y = current_node.location
        for (row, col) in maze.neighborPoints(x, y):
            node_location = (row, col)
            new_node = Node(current_node, node_location)
            children.append(new_node)

        for child in children:
     
            if len([closed_child for closed_child in closed_list if closed_child == child]) > 0:
                 continue
            # for closed_child in closed_list:
            #     if child == closed_child:
            #         continue

            child.g = current_node.g + 1
            child.h = heuristic_func(child.location, end_node.location)
           
            # if len([i for i in open_list if child == i and child.g > i.g]) > 0:
            #     continue
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            open_list.append(child)

    path.reverse()
    return path
    ############################################################################


# -------------------- Stage 02: Four circles - A* Algorithm  ------------------------ #

def stage2_heuristic(p1,p2):  
    return (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2

def astar_four_circles(maze):
    """
    [문제 03] 제시된 stage2의 맵 세가지를 A* Algorithm을 통해 최단 경로를 return하시오.(30점)
    (단 Heurstic Function은 위의 stage2_heuristic function을 직접 정의하여 사용해야 한다.)
    """

    end_points=maze.circlePoints()
    end_points.sort()

    path=[]

    ####################### Write Your Code Here ################################
    start_point=maze.startPoint()
    path24 = [] # 4! 

    #set maximum iteration 
    path_i = 0
    max_iteration = (len(maze.mazeRaw)**2) + 100

    end_nodes = []
    end_nodes.append(Node(None, end_points[0]))
    end_nodes.append(Node(None, end_points[1]))
    end_nodes.append(Node(None, end_points[2]))
    end_nodes.append(Node(None, end_points[3]))
    end_node_sequences = itertools.permutations(end_nodes, 4)

    for end_node_sequence in list(end_node_sequences):
        start_node = Node(None, start_point)
        n_visited = 0  # 4개 endpoint 방문 여부 확인
        for end_node in end_node_sequence:
            n_visited += 1
            #print(f"{path_i} {start_node.location}, {end_node.location}")
            path24.append([])
            iteration = 0
            start_node.g = 0
            end_node.g = end_node.h = 0
            start_node.h = stage2_heuristic(start_node.location, end_node.location)
            open_list = []
            closed_list = []

            open_list.append(start_node)
 
            while len(open_list) > 0:
                iteration += 1
                current_node = open_list[0]
                current_index = 0
                for index, item in enumerate(open_list):
                    if item < current_node:
                        current_node = item
                        current_index = index
                if iteration > max_iteration: 
                    print(f"failure : {iteration} iterations")
                    return path

                open_list.pop(current_index)
                closed_list.append(current_node)

                # Goal test
                if current_node == end_node: 
                    current = current_node
                    start_node = current_node
                    while current is not None:
                        if n_visited == 4:
                            path24[path_i].append(current.location)
                        current = current.parent
                    break

                children = []
                x, y = current_node.location
                for (row, col) in maze.neighborPoints(x, y):
                    node_location = (row, col)
                    new_node = Node(current_node, node_location)
                    children.append(new_node)

                for child in children:
                    if len([closed_child for closed_child in closed_list if closed_child == child]) > 0:
                        continue
          
                    child.g = current_node.g + 1
                    child.h = stage2_heuristic(child.location, end_node.location)
           
                    for open_node in open_list:
                        if child == open_node and child.g > open_node.g:
                            continue

                    #print(f"debug : {path_i}, {len(open_list)}")
                    open_list.append(child)
            
        path_i += 1

    min = max_iteration
    flag = 0
    for end_node_sequence in range(0,24):
        if min > len(path24[end_node_sequence]) :
            min = len(path24[end_node_sequence])
            flag = end_node_sequence

    path = path24[flag]
    #print(f"min : {min}\nflag : {flag}")
    path.reverse()
    return path

    ############################################################################


def mst(objectives, edges): # objectives : location(좌표)
                            # edges : key 두 좌표, value 거리(weight)
    cost_sum=0
    ####################### Write Your Code Here ################################
    mst_prim = []
    #print(objectives)
    discovered = [(0, objectives[0], objectives[0])]
    explored = []

    # print("DEBUG!!!")
    # print(type(edges))
    # print(edges)
    """ check below """
    while discovered:
        cost, _from, _to = heappop(discovered)
        if _to not in explored:
            #print(f"{_from},{_to} : {cost}")
            explored.append(_to)
            cost_sum += cost
            mst_prim.append((_from, _to))

            for i in objectives:
                if i not in explored:
            #     print(f"{len(edges[_to, i])}, {_to}, {i}")
            #     print(f"{type(len(edges[_to, i]))}, {type(_to)}, {type(i)}")
                    if (_to, i) in edges.keys():
                        heappush(discovered,(len(edges[_to, i])-1,_to,i))

    return cost_sum


# -------------------- Stage 03: Many circles - A* Algorithm -------------------- #

def stage3_heuristic(Node, dists):
    edges = {}
    two_points = itertools.permutations(Node.obj, 2)

    for dots in set(two_points):
        edges[dots] = dists[dots]

    return mst(Node.obj, edges)


def astar_many_circles(maze):
    """
    [문제 04] 제시된 stage3의 맵 세가지를 A* Algorithm을 통해 최단 경로를 return하시오.(30점)
    (단 Heurstic Function은 위의 stage3_heuristic function을 직접 정의하여 사용해야 하고, minimum spanning tree
    알고리즘을 활용한 heuristic function이어야 한다.)
    """

    end_points= maze.circlePoints()
    end_points.sort()

    path=[] # 최종적으로 구하고자 하는 pacman path

    ####################### Write Your Code Here ################################

    # initialize nodes : parent & location
    path_tmp = []
    start_point=maze.startPoint()
    start_node = Node(None, start_point)
    point_nodes = [] # 모든 point에 대해서 초기 node를 잡아준다.
    point_nodes.append(start_node)
    for i in range (0,len(end_points)):
        point_nodes.append(Node(None, end_points[i])) 

    """ 우선 dists의 정보를 얻기 위한 과정 """
    """ dists로 heuristic에 사용되는 정보를 얻기 위함"""
    # 모든 점 (startpoint, endpoints) 각각 사이의 최단 dists를 구하기 위함
    two_nodes = itertools.combinations(point_nodes, 2)
    paths = [] # nC2 shortest paths
    dists: dict = {} # dists[(point1, point2)] = shortest path

    path_i = 0 # paths[] 과 dists[]에 사용되는 index
    for start, end in list(two_nodes):
        ########
        start.g = end.g = end.h = 0
        start.h = manhatten_dist(start.location, end.location)

        open_list = []
        closed_list = []

        # A* algorithm (Finding shortest path between start & end)
        open_list.append(start)
        while len(open_list) > 0:
            
            # choosing current_node by smallest f = g + h
            current_node = open_list[0]
            current_index = 0
            for index, item in enumerate(open_list):
                if item < current_node:
                    current_node = item
                    current_index = index

            open_list.pop(current_index)
            closed_list.append(current_node)

            # Goal test
            if current_node == end:
                current = current_node
                # append new path
                paths.append([])
                while current is not None:
                    paths[path_i].append(current.location)
                    current = current.parent
                dists[end.location, start.location] = paths[path_i]
                paths[path_i].reverse()
                dists[start.location,end.location] = paths[path_i]
                path_i += 1 
                break

            children = []
            x, y = current_node.location
            for (row, col) in maze.neighborPoints(x, y):
                node_location = (row, col)
                new_node = Node(current_node, node_location)
                children.append(new_node)

            for child in children:
                if len([closed_child for closed_child in closed_list if closed_child == child]) > 0:
                    continue
                # for closed_child in closed_list:
                #     if child == closed_child:
                #         continue

                child.g = current_node.g + 1
                child.h = manhatten_dist(child.location, end.location)
           
                # if len([i for i in open_list if child == i and child.g > i.g]) > 0:
                #     continue
                for open_node in open_list:
                    if child == open_node and child.g > open_node.g:
                        continue

                open_list.append(child)
        # dist = start와 end 사이의 거리
        ########
        dists[(start, end)] = dist
    """ n개의 점에 대하여 각각의 shortest path를 구하였다. (총 nP2 개) """
    
    """ shortest path들을 통해서 TSP 적용 """
    # initialize start & end nodes : obj[] (해당 노드에서 h 값을 구할때 사용되는 mst의 vertex들)
    for i in range(0,len(point_nodes)):
        for j in range(1,len(point_nodes)):
            if(i != j):
                point_nodes[i].obj.append(point_nodes[j].location)
    # for i in range(0,len(point_nodes)):
    #     print(f"{point_nodes[i].location} : {point_nodes[i].obj}")

    """ stopped """
    # initialize start & end nodes : g, h
    point_nodes[0].g = 0; point_nodes[0].h = stage3_heuristic(point_nodes[0], dists)
    #print(f"{point_nodes[0].location} : {point_nodes[0].h}")
    for i in range(1,len(point_nodes)):
        point_nodes[i].g = len(dists[point_nodes[0].location, point_nodes[i].location])-1
        point_nodes[i].h = stage3_heuristic(point_nodes[i], dists)
        #print(f"{point_nodes[i].location} : {point_nodes[i].g}+ {point_nodes[i].h}")

    # # set iterations
    iteration = 0
    # max_iteration = (len(maze.mazeRaw)**2) * 5

    # set open_list & closed_list
    open_list = []
    closed_list = []
    open_list.append(point_nodes[0])
 
    # finding the path
    while len(open_list) > 0:
        # increase iteration
        print(f"{iteration}")
        iteration += 1
        
        # choosing current_node by smallest f = g + h
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item < current_node:
                current_node = item
                current_index = index
                # if point_nodes[i].location in current_node.obj:
                #     current_node.obj.remove(point_nodes[i].location)
        # break if the iteration is too much (maybe no path)
        # if iteration > max_iteration:
        #     print(f"failure : {iteration} iterations")
        #     return path

        open_list.pop(current_index)
        closed_list.append(current_node)

        # print(f"current obj length : {len(current_node.obj)}")
        """ later """
        # Goal test
        # for i in range(1,len(point_nodes)):       # current_node가 end_points 중 하나인지
        #     #print(f"{current_node.location}")
        #     if current_node.location == point_nodes[i].location: # 만약 그렇다면
        #         if point_nodes[i].location in current_node.obj:
        #             current_node.obj.remove(point_nodes[i].location)
        #         # for j in range(0,4):         # 다른 end_node.obj에서
        #         #     if tmp in end_nodes[j].obj:
        #         #         end_nodes[j].obj.remove(tmp)  #  지워준다
        if len(current_node.obj) == 0:  # 만약 current_node의 obj가 비었다면      
            current = current_node      # 모든 목적지를 다 도착한 것이므로
            while current is not None:  # 최종 path_tmp에 append해준다.
                path_tmp.append(current.location)
                current = current.parent
            path_tmp.reverse()
            print(path_tmp)
            break
        """        """
        
        # Finding candidate nodes to append in open_list
        children = []
       # x, y = current_node.location
        for (row, col) in current_node.obj:
            node_location = (row, col)
            new_node = Node(current_node, node_location)
            new_node.obj = current_node.obj
            # print(new_node.obj)
            # print(current_node.location)
            # print()
            if current_node.location in new_node.obj:
                new_node.obj.remove(current_node.location)
            #print(f"new_node obj length : {len(new_node.obj)}")
            children.append(new_node)

        # if the candidate node is in the closed_list -> continue
        for child in children:
            # if len([closed_child for closed_child in closed_list if closed_child == child]) > 0:
            #      continue
            for closed_child in closed_list:
                if child == closed_child: # location 과 obj 까지 같아야 함
                    continue
            # allocating candidate node's g, h
            if current_node.location == child.location:
                child.g = current_node.g + 0
            else:
                child.g = current_node.g + len(dists[current_node.location, child.location])-1
            child.h = stage3_heuristic(child, dists)
           
            # if candidate's f = g + h is bigger -> continue
            # if len([i for i in open_list if child == i and child.g > i.g]) > 0:
            #     continue
            for open_node in open_list:
                if child == open_node and (child.g > open_node.g or child > open_node):
                    continue

            open_list.append(child)
    for i in range(i,len(path_tmp)-1):
        path.append(dists[path_tmp[i], path_tmp[i+1]])
    
    print(path)
    return path

    ############################################################################
