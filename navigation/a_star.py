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

    def __init__(self, x, y, end):
        self.isOpen = False 
        self.isClosed = False
        self.isObstacle = False
        self.x = x
        self.y = y
        self.f = infinity
        self.h = 0
        self.string = "."
        if(end != None):
            self.h = ((end.x - self.x)**2 + (end.y - self.y)**2)**0.5
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
            
#https://www.dcode.fr/maze-generator           
def makeGridfromMaze(width, height,text):
    end_x = width - 1
    end_y = height - 1
    start_x = 0
    start_y = 0
    endNode = Node(end_x, end_y, None)

    text = text.strip()
    print(len(text))
    grid = []
    counter = 0
    for i in range(width):
        arr = []
        for j in range(height):
            node = Node(i,j,endNode)
            character = text[counter]
            if(character == '1' and (i != start_x or j != start_y)):
                node.setObstacle(True)
            arr.append(node)
            counter += 1
        grid.append(arr)
    print(len(grid), len(grid[0]))
    grid[end_x][end_y] = endNode
    
    return grid
    

    

def makeGrid(width, height, prob, start_x, start_y, end_x, end_y):
    endNode = Node(end_x,end_y, None)
    grid = []
    for i in range(width):
        arr = []
        for j in range(height):
            node = Node(i,j,endNode)
            if(decision(prob) and (i != start_x or j != start_y)):
                node.setObstacle(True)
            arr.append(node)
        grid.append(arr)
    grid[end_x][end_y] = endNode
    
    return grid


def solveGrid(grid, start_x, start_y, end_x, end_y):
    q = PriorityQueue()
    grid[start_x][start_y].isOpen = True 
    grid[start_x][start_y].g = 0
    grid[start_x][start_y].f = grid[start_x][start_y].h + grid[start_x][start_y].g
    #q_finished = PriorityQueue()
    #put starting node node in the queue
    q.put(grid[start_x][start_y])
    while(not q.empty()):

        #animateGrid(grid)

        #take the node from the priority list 
        nownode = q.get()
        nownode.isClosed = True
        #nownode.isOpen = False
        nownode.string = "2"
        #q_finished.put(nownode)
        if(nownode.x == end_x and nownode.y == end_y):
            #if we are at the end, then we are done
            return
        
        #check the neighbors
        dy = [1,1,1,0,0,-1,-1,-1]
        dx = [-1,0,1,-1,1,-1,0,1]
        for i in range(8):
            #check if index out of bounds 
            if (0 <= nownode.x + dx[i] < len(grid) and 0 <= nownode.y + dy[i] < len(grid[0])):
                neighbor = grid[nownode.x + dx[i]][nownode.y + dy[i]]
                if(neighbor.isObstacle):
                    continue
                if(neighbor.isClosed):
                    continue
                distance = ((nownode.x - neighbor.x)**2 + (nownode.y - neighbor.y)**2)**0.5
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
    grid[node.x][node.y].string = "3"
    if(node.parent == None): return 
    makeParent(grid, node.parent)

def animateGrid(grid):
    """Create a graphical representation of the grid in matplotlib"""

    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if(grid[i][j].string == "1"):
                plt.plot(i, j, "sr", ms = 25)
            elif(grid[i][j].string == "2"):
                plt.plot(i, j, "sy", ms = 25)
            else:
                plt.plot(i, j, "sg", ms = 25)

    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if(grid[i][j].string == "."):
                plt.plot(i, j, "sb", ms = 25)
    plt.axis([-.5, len(grid)-.5, -.5, len(grid[0])-.5])
    plt.grid(True)
    plt.pause(0.01)

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


def solveGridDFS(grid, start_x, start_y, end_x, end_y):
    distance =[]
    for i in range(len(grid)):
        newarr = []
        for j in range(len(grid[0])):
            newarr.append(infinity)
        distance.append(newarr)
    
    distance[end_x][end_y] = 0
    DFS(grid, distance, end_x, end_y)

    return distance[start_x][start_y]


def DFS(grid, distance, x, y):
    dy = [1,1,1,0,0,-1,-1,-1]
    dx = [-1,0,1,-1,1,-1,0,1]
    for i in range(8):
        #check if index out of bounds 
        if (0 <= x + dx[i] < len(grid) and 0 <= y + dy[i] < len(grid[0])):
            neighbor = grid[x + dx[i]][y + dy[i]]
            if(neighbor.isObstacle):
                    continue
            d = ((x - neighbor.x)**2 + (y - neighbor.y)**2)**0.5

            if(distance[neighbor.x][neighbor.y] > distance[x][y] + d):
                distance[neighbor.x][neighbor.y] = distance[x][y] + d
                DFS(grid, distance, neighbor.x, neighbor.y)
    

def main():

    print("hello")
    start_time = time.time()
    #".": cell (light blue)
    #"1": obstacle (red)
    #"2": visited (yellow green)
    #"3": actual path (green)
    width = 100
    length = 100
    text = """
0001111111111111111111111111111
0000000001000000000000001001001
1111001001111111001001111001001
1000001000001001001000001000001
1001001111111001111001111001111
1001001000000000000000000000001
1001001001111111111001001111001
1001000000000001000001000001001
1111111111001111111111001111001
1000001000000000001001001000001
1001111001111001001001111001111
1000000000001001000001000000001
1001111111001001111111001111111
1001000001001000001000000001001
1001001111001001111111111001001
1000001001001000000001000001001
1111001001001001111001111001001
1000001000001001000001001000001
1001111111001111001111001111001
1000001000001000001000000000001
11111111111111111111111111110000
"""
    print(len(text))
    #grid = makeGridfromMaze(21, 32, text)
    grid = makeGrid(width,length,0.3,0,0,width-1,length-1)
    solveGrid(grid, 0, 0, width-1, length-1)
    makeParent(grid, grid[width-1][length-1])
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            print(grid[i][j], end = " ")
        print("\n")
    print("Process finished --- %s seconds ---" % (time.time() - start_time))

    #animateGrid(grid)
    #plt.grid(True)
    #plt.show()
            

    actualDistance = solveGridDFS(grid, 0, 0, width-1, length-1)
    astarDistance = grid[width-1][length-1].f

    print("We are printing the DFS shortest path versus the A*. If they are both the same, then we can conclude that A* has found the shortest path")

    print(actualDistance, astarDistance)

    printGrid(grid)  
    

if __name__ == "__main__":
    main()
