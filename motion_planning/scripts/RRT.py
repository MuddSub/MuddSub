import math
import random

class RRT:

    def __init__(self, start, goal, obstacle_list, max_length):
        # start: starting location
        # goal: goal location
        # obstacle_list: list of obstacles
        # max_length: length of the nodes

        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[0])
        self.obstacle_list = obstacle_list
        self.max_length = max_length
    
    def path_planning(self):
        # setting the list of nodes to the starting location
        self.nodes = [self.start]

        while True:

            # I've been looking into RRT* and I think it deals with
            # random sampling and thinking of changing the getNearestIndex
            # to fit that (hopefully)

            # finding node index in relation to the goal
            new_ind = self.getNearestIndex(self.nodes, self.goal) 

            # place the index into a new node to generate the new node
            newNode = self.nodeList[new_ind]
            theta = math.atan2(goal[1] - newNode.y, goal[0] - newNode.x)
            newNode.x += self.max_length * math.cos(theta)
            newNode.y += self.max_length * math.sin(theta)
            newNode.parent = new_ind

            # check if the node is in an obstacle
            # if not, add to list of nodes
            if not self.checkCollisions(newNode, self.obstacle_list):
                continue
            self.nodes.append(newNode)

            # check if the node hit the goal
            if self.checkGoal(newNode, self.goal):
                break
            
        # creating the final path that hit the goal

        # starting from the goal node and working backwards
        path = [[self.goal.x, self.goal.y]]
        last_ind = len(self.nodes) - 1

        # loops through each parent and appends to list of path
        while self.nodes[last_ind].parent is not None:
            node = self.nodes[lastIndex]
            path.append([node.x, node.y])
            last_ind = node.parent
        
        # including the starting location
        path.append([self.start.x, self.start.y])

        return path


    def checkCollisions(self, node, obstacle_list):
        # not completely sure how the obstacle_list will work but I'm imaginging
        # something like a square with position and sizing
        for (x, y, w, h) in obstacle_list:
            dx = x - node.x
            dy = y - node.y
            d = math.sqrt(dx**2 + dy**2)
            if dist <= (w*h):
                return False
            return True
    
    def checkGoal(self, node, goal):
        dx = node.x - goal.x
        dy = node.y - goal.y
        d = math.sqrt(dx**2 + dy**2)
        if d <= self.max_length:
            return True

    def getNearestIndex(self, nodes, goal):
        distance = [math.sqrt((node.x - goal[0]) ** 2 + (node.y - goal[1])** 2) for node in nodeList]
        min_index = distance.index(min(distance))
        return min_index

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


def main():
    # set the start, goal, obstacles
    # everything needed to graph (still working on it)
    pass

if __name__ == '__main__':
    main()
