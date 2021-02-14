from queue import PriorityQueue
import random

def decision(probability):
    return random.random() < probability

infinity = 10000
class Node(object):

    def __init__(self, i, j, end):
        self.isOpen = False 
        self.isClosed = False
        self.isObstacle = False
        self.i = i
        self.j = j
        self.f = infinity
        self.h = 0
        self.string = "0"
        if(end != None):
            self.h = ((end.i - self.i)**2 + (end.j - self.j)**2)**0.5
        self.g = infinity
        self.parent = None 
    
    def __lt__(self, other):
        return self.f < other.f
    def __gt__(self, other):
        return self.f > other.f
    def __str__(self):
        
        if(self.isObstacle):  return "1"
        return self.string
        
    

def makeGrid(width, height, prob, start_i, start_j, end_i, end_j):
    endNode = Node(end_i,end_j, None)
    grid = []
    for i in range(width):
        arr = []
        for j in range(height):
            node = Node(i,j,endNode)
            if(decision(prob) and (i != start_i or j != start_j)):
                node.isObstacle = True
            arr.append(node)
        grid.append(arr)
    grid[end_i][end_j] = endNode
    
    return grid


def solveGrid(grid, start_i, start_j, end_i, end_j):
    q = PriorityQueue()
    grid[start_i][start_j].isOpen = True 
    grid[start_i][start_j].g = 0
    grid[start_i][start_j].f = grid[start_i][start_j].h + grid[start_i][start_j].g
    #q_finished = PriorityQueue()
    #put starting node node in the queue
    q.put(grid[start_i][start_j])
    while(not q.empty()):
        
        #take the node from the priority list 
        nownode = q.get()
        nownode.isFinished = True
        #nownode.isOpen = False
        nownode.string = "2"
        #q_finished.put(nownode)
        if(nownode.i == end_i and nownode.j == end_j):
            #if we are at the end, then we are done
            return
        
        #check the neighbors
        dy = [1,1,1,0,0,-1,-1,-1]
        dx = [-1,0,1,-1,-1,-1,0,1]
        for i in range(8):
            #check if in range
            if (0 <= nownode.i + dx[i] < len(grid) and 0 <= nownode.j + dy[i] < len(grid[0])):
                neighbor = grid[nownode.i + dx[i]][nownode.j + dy[i]]
                if(neighbor.isObstacle):
                    continue
                if(neighbor.isClosed):
                    continue
                distance = ((nownode.i - neighbor.i)**2 + (nownode.j - neighbor.j)**2)**0.5
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
    grid[node.i][node.j].string = "3"
    if(node.parent == None): return 
    makeParent(grid, node.parent)



def main():

    width = 10
    length = 10
    grid = makeGrid(width,length,0.2,0,0,width-1,length-1)
    solveGrid(grid, 0, 0, width-1, length-1)
    makeParent(grid, grid[width-1][length-1])
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            print(grid[i][j], end = " ")
        print("\n")
    

if __name__ == "__main__":
    main()
    