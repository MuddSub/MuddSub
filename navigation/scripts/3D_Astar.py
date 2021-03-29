from matplotlib import pyplot as plt 
from queue import PriorityQueue
import random
from matplotlib import colors
import numpy as np
import time

def decision(probability):
    return random.random() < probability

infinity = float('inf')


class Node(object):

    def __init__(self, x, y, z, end):
        self.isOpen = False 
        self.isClosed = False
        self.isObstacle = False
        self.x = x
        self.y = y
        self.z = z
        self.f = infinity
        self.h = 0
        self.string = "."
        if(end != None):
            self.h = ((end.x - self.x)**2 + (end.y - self.y)**2 + (end.z - self.z)**2)**0.5
        self.g = infinity
        self.parent = None 
    
    def __lt__(self, other):
        return self.f < other.f
    def __gt__(self, other):
        return self.f > other.f
    def __str__(self):
        
        
        return self.string
    
    def setObstacle(self, isObstacle):
        self.isObstacle = isObstacle
        if(self.isObstacle):
            if(self.isObstacle):  
                self.string = "1"
            else:
                self.string = "0"
               

def makeGrid(width, height, depth, prob, start_x, start_y, start_z, end_x, end_y, end_z):
    endNode = Node(end_x,end_y, end_z, None)
    grid = []
    for i in range(width):
        arr = []
        for j in range(height):
            arr2 = []
            for k in range(depth):
                node = Node(i,j,k,endNode)
                if(decision(prob) and (i != start_x or j != start_y or k != start_z)):
                    node.setObstacle(True)
                arr2.append(node)
            arr.append(arr2)
        grid.append(arr)
    grid[end_x][end_y][end_z] = endNode
    
    return grid


def solveGrid(grid, start_x, start_y, start_z, end_x, end_y, end_z):
    q = PriorityQueue()
    grid[start_x][start_y][start_z].isOpen = True 
    grid[start_x][start_y][start_z].g = 0
    grid[start_x][start_y][start_z].f = grid[start_x][start_y][start_z].h + grid[start_x][start_y][start_z].g
    #q_finished = PriorityQueue()
    #put starting node node in the queue
    q.put(grid[start_x][start_y][start_z])
    while(not q.empty()):
        #take the node from the priority list 
        nownode = q.get()
        nownode.isClosed = True
        #nownode.isOpen = False
        nownode.string = "2"
        #q_finished.put(nownode)
        if(nownode.x == end_x and nownode.y == end_y and nownode.z == end_z):
            #if we are at the end, then we are done
            return
        
        #check the neighbors
        dy = [1,1,1,0,0,-1,-1,-1, 1,1,1,0,0,-1,-1,-1,0, 1,1,1,0,0,-1,-1,-1,0]
        dx = [-1,0,1,-1,1,-1,0,1, -1,0,1,-1,1,-1,0,1,0, -1,0,1,-1,1,-1,0,1,0]
        dz = [0,0,0,0,0,0,0,0, 1,1,1,1,1,1,1,1,1, -1,-1,-1,-1,-1,-1,-1,-1,-1]
        for i in range(26):
            #check if index out of bounds 
            if (0 <= nownode.x + dx[i] < len(grid) and 0 <= nownode.y + dy[i] < len(grid[0]) and 0 <= nownode.z + dz[i] < len(grid[0][0])):
                neighbor = grid[nownode.x + dx[i]][nownode.y + dy[i]][nownode.z + dz[i]]
                if(neighbor.isObstacle):
                    continue
                if(neighbor.isClosed):
                    continue
                distance = ((nownode.x - neighbor.x)**2 + (nownode.y - neighbor.y)**2 + (nownode.z - neighbor.z)**2)**0.5
                if(nownode.g + distance < neighbor.g or not neighbor.isOpen):
                    #update f_cost
                    neighbor.g = nownode.g + distance 
                    neighbor.f = neighbor.g + neighbor.h
                    #make parent
                    neighbor.parent = nownode
                    if(not neighbor.isOpen):
                        neighbor.isOpen = True
                        q.put(neighbor)
                    



def makeParent(grid, node):
    grid[node.x][node.y][node.z].string = "3"
    if(node.parent == None): return 
    makeParent(grid, node.parent)

def printGrid(grid):
    """Create a graphical representation of the grid in matplotlib"""
    data = []
    for i in range(len(grid)):
        newarray = []
        for j in range(len(grid[0])):
            if(grid[i][j].string == "."):
                newarray += [0]
            elif(grid[i][j].string == "1"):
                newarray += [1]
            elif(grid[i][j].string == "2"):
                newarray += [2]
            else:
                newarray += [3]
        data += [newarray]
    #print(data)
    height = len(grid)
    width = len(grid[0])

    # create discrete colormap
    cmap = colors.ListedColormap(['lightblue', 'red', 'yellowgreen', 'green'])
    bounds = [0,1,2,3,4]
    norm = colors.BoundaryNorm(bounds, cmap.N)

    fig, ax = plt.subplots()
    ax.imshow(data, cmap=cmap, norm=norm)

    # draw gridlines
    ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=1)
    ax.set_xticks(np.arange(-0.5, width, 1))
    ax.set_yticks(np.arange(-0.5, height, 1))
    # ax.set_xticklabels([])
    # ax.set_yticklabels([])

    plt.show()


def solveGridDFS(grid, start_x, start_y, start_z, end_x, end_y, end_z):
    distance =[]
    for i in range(len(grid)):
        newarr = []
        for j in range(len(grid[0])):
            newarr2 = []
            for k in range(len(grid[0][0])):
                newarr2.append(infinity)
            newarr.append(newarr2)
        distance.append(newarr)
    
    distance[end_x][end_y][end_z] = 0
    DFS(grid, distance, end_x, end_y, end_z)

    return distance[start_x][start_y][start_z]


def DFS(grid, distance, x, y, z):
    dy = [1,1,1,0,0,-1,-1,-1, 1,1,1,0,0,-1,-1,-1,0, 1,1,1,0,0,-1,-1,-1,0]
    dx = [-1,0,1,-1,1,-1,0,1, -1,0,1,-1,1,-1,0,1,0, -1,0,1,-1,1,-1,0,1,0]
    dz = [0,0,0,0,0,0,0,0, 1,1,1,1,1,1,1,1,1, -1,-1,-1,-1,-1,-1,-1,-1,-1]
    for i in range(26):
        #check if index out of bounds 
        if (0 <= x + dx[i] < len(grid) and 0 <= y + dy[i] < len(grid[0]) and 0 <= z + dz[i] < len(grid[0][0])):
            neighbor = grid[x + dx[i]][y + dy[i]][z + dz[i]]
            if(neighbor.isObstacle):
                    continue
            d = ((x - neighbor.x)**2 + (y - neighbor.y)**2 + (z - neighbor.z)**2)**0.5

            if(distance[neighbor.x][neighbor.y][neighbor.z] > distance[x][y][z] + d):
                distance[neighbor.x][neighbor.y][neighbor.z] = distance[x][y][z] + d
                DFS(grid, distance, neighbor.x, neighbor.y, neighbor.z)
    

def main():

    start_time = time.time()
    #".": cell (light blue)
    #"1": obstacle (red)
    #"2": visited (yellow green)
    #"3": actual path (green)
    width = 3
    length = 10
    depth = 10
    grid = makeGrid(width,length,depth,0.4,0,0,0, width-1,length-1, depth-1)
    solveGrid(grid, 0, 0, 0, width-1, length-1, depth-1)
    makeParent(grid, grid[width-1][length-1][depth-1])

    for i in range(len(grid)):
        for j in range(len(grid[0])):
            for k in range(len(grid[0][0])):
                print(grid[i][j][k], end = " ")
            print("\n")
        print("\n")
        print("\n")

    print("Process finished --- %s seconds ---" % (time.time() - start_time))

    actualDistance = solveGridDFS(grid, 0, 0, 0, width-1,length-1, depth-1)
    astarDistance = grid[width-1][length-1][depth-1].f

    print("We are printing the DFS shortest path versus the A*. If they are both the same, then we can conclude that A* has found the shortest path")

    print(actualDistance, astarDistance)
    
    for i in range(len(grid)):
        printGrid(grid[i])


if __name__ == "__main__":
    main()
