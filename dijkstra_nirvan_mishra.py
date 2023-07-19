import numpy as np
import cv2 as cv
import heapq
import time

start_time = time.time()
#-------------------------------Step 1: Map using opencv-------------------------------------------------------------------------------------------
canvas = np.zeros((251, 601, 3), np.uint8)
# canvas.fill(255)

c = int(input('Enter clearence of the obstacles: '))
rect1 = cv.rectangle(canvas,(100, 100), (150, 0),(255,0,0), -1)
rect1_c = cv.rectangle(canvas,(100-c, 100), (150+c, 0),(0,255,0), 5)
rect2 = cv.rectangle(canvas,(100,250), (150,150), (255,0,0),-1)
rect2_c = cv.rectangle(canvas,(100-c,250), (150+c,150), (0,255,0),5)

center_hex = (300, 125)
side = 75
angle_deg = 60
vertices = np.array([[300,200],[235,162],[235,87],[300,50],[364,87],[364,162]])
vertices_c = np.array([[300,200+c],[235-c,162],[235-c,87],[300,50-c],[364+c,87],[364+c,162]])
canvas1 = cv.fillPoly(canvas, [np.array(vertices)], color=(255,0,0))
canvas1 = cv.polylines(canvas, [np.array(vertices_c)],isClosed=True,color= (0,255,0), thickness=5)

tri_vertices = np.array([[[460, 25]], [[460, 225]], [[510, 125]]])
tri_vertices_c = np.array([[[460-c, 25]], [[460-c, 225]], [[510+c, 125]]])
canvas1 = cv.fillPoly(canvas, [tri_vertices], color=(255, 0, 0))
canvas1 = cv.polylines(canvas, [np.array(tri_vertices_c)],isClosed=True,color= (0,255,0), thickness=5)

obstacle_space = []

for y in range(canvas1.shape[0]):
    for x in range(canvas1.shape[1]):
        if canvas1[y,x].any():
            obstacle_space.append((x,y))
# print(obstacle_space)

#------------------------------------------------- Step 2: UI --------------------------------------------------------------------------------------
coordinate = True
while coordinate:
 
    start_x = input('Enter starting node x coordinate: ')
    start_y = input('Enter starting node y coordinate: ')
    goal_x = input('Enter goal node x coordinate: ')
    goal_y = input('Enter goal node y coordinate: ')

    start_node = (int(start_x),int( start_y))
    goal_node = (int(goal_x),int(goal_y))
    cv.circle(canvas1,goal_node, 1, (0,0,255),2)
    if start_node in obstacle_space:
        print("The start coordinates falls into obstacles.")
    elif goal_node in obstacle_space:
        print("The goal coordinates falls into obstacles.")
    elif start_node[0] >= canvas1.shape[1] and start_node[1] >= canvas1.shape[0]:
        print("The coordinates falls outside the map.")
    elif goal_node[0] >= canvas1.shape[1] and goal_node[1] >= canvas1.shape[0]:
        print("The coordinates falls outside the map.")
    else:
        coordinate = False
#--------------------------------Step 3: Defining Action Set----------------------------------------------------------------------------------------

def move_above(current_node):
    x, y = current_node
    if 0<=x<600:
        return(x+1, y)
    
def move_down(current_node):
    x, y = current_node
    if 0<=x<600:
        return(x-1, y)
    
def move_right(current_node):
    x, y = current_node
    if 0<=y<250:
        return(x, y+1)

def move_left(current_node):
    x, y = current_node
    if 0<=y<250:
        return(x, y-1)
    
def move_up_right(current_node):
    x, y = current_node
    if 0<=x<600 and 0<=y<250:
        return(x+1, y+1)

def move_up_left(current_node):
    x, y = current_node
    if 0<=x<600 and 0<=y<250:
        return(x+1, y-1)  
    
def move_down_right(current_node):
    x, y = current_node
    if 0<=x<600 and 0<=y<250:
        return(x-1, y+1)

def move_down_left(current_node):
    x, y = current_node
    if 0<=x<600 and 0<=y<250:
        return(x-1, y-1)

# (x, y) = move_down_left(start_node)
# print(x,y)

#---------------------------------------------------------------------------------------------------------------------------------------------------
def dijkstra_algo(start, goal, action):

    open_list = [(0, start)]   # "OL = [(c2c, start_node)]"
    closed_list = {start:(0, None)}    # "CL = {start_node: (c2c, parent_node)}"

    # actions = [move_above, move_down, move_left, move_right, move_up_left,move_up_right, move_down_left, move_down_right]
    while open_list:

        c2c, current_node = heapq.heappop(open_list)    #Pop the node with smallest c2c
#----------------------Current node is goal node founding path-------------------------------------------------------------------------------------
        if current_node == goal:
            path = []
            print('Path found!!')
            while current_node != start_node:
                path.append(current_node)
                current_node = closed_list[current_node][1]    #Parent node of the current node.
            
            path.append(start)
            path.reverse()
            for point in path:
                cv.circle(canvas1, point, 3,(255,0,255),1)
                flip = cv.flip(canvas1, 0)
                cv.imshow('Map', flip)
                cv.waitKey(50)

            return path, closed_list[goal_node][0]
#-----------------------------------------If current node isn't goal node--------------------------------------------------------------------------

        for moves in action:
            next_node = moves(current_node) #Generating new nodes by applying the actions.
            # print(next_node)
            if next_node in obstacle_space:
                continue
            else:
                if moves in [move_above, move_down, move_left, move_right]:
                    cost = 1
                else:
                    cost = 1.4
            total_cost = c2c + cost
            if next_node not in closed_list:    #The nodes that are not connected directly, their c2c = infinity
                closed_list[next_node] = (float('inf'), None)
            #_____________________Performing relaxation___________________________________#

            if total_cost < closed_list[next_node][0] and next_node is not None:
                closed_list[next_node] = (total_cost, current_node)
                heapq.heappush(open_list, (total_cost, next_node))
                cv.circle(canvas1, closed_list[next_node][1], 3,(255,255,0),1)
                flip = cv.flip(canvas1, 0)
                cv.imshow('Map',flip)
                cv.waitKey(1)

    return None, None
    
action_set = [move_above, move_down, move_left, move_right, move_up_right, move_up_left, move_down_left, move_down_right]
path, cost = dijkstra_algo(start_node,goal_node, action_set)
end_time = time.time()
print("Path taken:\n ", path)
print()
print("Total Cost: \n", cost)
print()
print("Total time taken: ", (end_time-start_time), "secs")
