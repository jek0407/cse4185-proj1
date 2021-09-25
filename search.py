###### Write Your Library Here ###########
import copy
from collections import *
from math import *
from heapq import *
from typing import List, Tuple, Dict

from maze import Maze
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
    start와 end가 하나씩만 있는 maze가 들어왔을 때 주어진 heuristic_func로 astar를 돌려
    path를 반환하는 함수

    :param maze:
    :param heuristic_func:
    :return:
    """

    """
    [문제 02] 제시된 stage1의 맵 세가지를 A* Algorithm을 통해 최단경로를 return하시오.(20점)
    (Heuristic Function은 위에서 정의한 manhatten_dist function을 사용할 것.)
    """

    # start_point=maze.startPoint()
    # end_point=maze.circlePoints()[0]
    start_point=maze.__start
    end_point=maze.__objective[0]
    path=[]

    ####################### Write Your Code Here ################################
    iteration = 0
    max_iteration = len(maze.mazeRaw)**3
    start_node = Node(None, start_point)
    start_node.g = 0 ##
    end_node = Node(None, end_point)
    end_node.g = end_node.h = 0 ##
    start_node.h = heuristic_func(start_node.location, end_node.location)

    open_list = []
    closed_list = []

    open_list.append(start_node)
 
    while len(open_list) > 0:
        # print("Open list:", end="")
        # for node in open_list:
        #     print(node.location, end="")
        # print()
        iteration += 1
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item < current_node:
                current_node = item
                current_index = index
        if iteration > max_iteration: 
            raise Exception("failure")

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
            if child in closed_list:
                continue
            if child in open_list:
                continue
            child.g = current_node.g + 1
            # if len([i for i in open_list if child == i and child.g > i.g]) > 0:
            #     continue

            child.h = heuristic_func(child.location, end_node.location)
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


def mst(objectives, edges): # objectives: 남은 points (시작점 포함)
                            # edges: { key=두 좌표: value=shortest_path }
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

mst_cache = dict()


def reset_mst_cache():
    global mst_cache
    mst_cache = dict()


def stage3_heuristic(node, shortest_paths):
    global mst_cache
    remaining_points = [node.location] + node.obj
    cache_key = tuple(sorted(remaining_points))
    if cache_key not in mst_cache:
        mst_cache[cache_key] = mst(remaining_points, edges=shortest_paths)
    return mst_cache[cache_key]


def astar_many_circles(maze):
    """
    [문제 04] 제시된 stage3의 맵 세가지를 A* Algorithm을 통해 최단 경로를 return하시오.(30점)
    (단 Heurstic Function은 위의 stage3_heuristic function을 직접 정의하여 사용해야 하고, minimum spanning tree
    알고리즘을 활용한 heuristic function이어야 한다.)
    """
    ####################### Write Your Code Here ################################
    # initialize nodes : parent & location
    start_point = maze.startPoint()
    end_points= maze.circlePoints()
    end_points.sort()

    # 모든 점 (startpoint, endpoints) 각각 사이의 최단 dists를 구하기 위함 (총 nP2)
    points = [start_point] + end_points
    point_pairs: Iterable[Tuple] = itertools.combinations(points, 2)
    point_shortest_paths: Dict[List[Tuple]] = dict()
    # Type hint: point_shortest_paths[(point1: Tuple, point2: Tuple)] = shortest_path

    print("Number of points".center(80, "#"))
    print(len(points))

    for start, end in list(point_pairs):
        assert(type(start) == tuple)
        assert(type(end) == tuple)

        maze.__start = start
        maze.__objective = [end]
        shortest_path = astar(maze)
        point_shortest_paths[start, end] = shortest_path  # [start . . . . end]
        point_shortest_paths[end, start] = list(reversed(shortest_path))  # 키 문제 때문에 그냥 양방향 추가

    maze.__start = start
    maze.__objective = end_points

    # Point에 대한 A* 알고리즘 수행하여 point_path 구하기
    reset_mst_cache()
    point_path = []

    iteration = 0
    max_iteration = len(points) ** 3
    start_node = Node(None, start_point)
    start_node.obj = copy.deepcopy(end_points)
    start_node.g = 0
    start_node.h = stage3_heuristic(start_node, point_shortest_paths)

    open_list = []
    closed_list = []

    open_list.append(start_node)

    while len(open_list) > 0:
        # maze.neighborPoints(0, 0)  # search state 늘려주기 위함
        iteration += 1
        if iteration > max_iteration:
            raise Exception("Exceeded max iteration")

        # Select node with minimum f from open_list and pop
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item < current_node:
                current_node = item
                current_index = index
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Goal test
        if not current_node.obj:  # if empty
            current = current_node
            while current is not None:
                point_path.append(current.location)
                current = current.parent
            break

        # Find candidate children
        children = []
        for node_location in current_node.obj:
            new_node = Node(current_node, node_location)
            new_node.obj = copy.deepcopy(current_node.obj)
            assert(new_node.location in new_node.obj)
            new_node.obj.remove(new_node.location)
            children.append(new_node)

        # Select from candidate children
        for child in children:
            key = (current_node.location, child.location)
            child.g = current_node.g + len(point_shortest_paths[key]) - 1

            if child in open_list:
                continue

            child.h = stage3_heuristic(child, point_shortest_paths)
            open_list.append(child)

    point_path.reverse()
    print("Point Path".center(40).center(80, '#'))
    print(point_path)

    # Point path에서 coord path 도출
    path = []  # 최종적으로 구하고자 하는 pacman path
    for i in range(len(point_path) - 1):
        start = point_path[i]
        end = point_path[i + 1]
        intermediate_path = point_shortest_paths[(start, end)][:-1]
        path += intermediate_path
    end = point_path[-1]
    path.append(end)

    return path
    ############################################################################
