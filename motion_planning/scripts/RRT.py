import math
import random
import copy
import matplotlib.pyplot as plt

show_animation = True

class RRT:

    def __init__(self, start, goal, obstacle_list, max_length, random_area):
        # start: starting location
        # goal: goal location
        # obstacle_list: list of obstacles
        # max_length: length of the nodes

        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[0])
        self.obstacle_list = obstacle_list
        self.max_length = max_length        
        self.min_rand = random_area[0]
        self.max_rand = random_area[1]
    
    def path_planning(self, animation = True):
        # setting the list of nodes to the starting location
        self.nodes = [self.start]

        while True:

            # Random Sampling within the area of the grid
            # 80% of a nearby area on anywhere on the grid
            if random.randint(0, 100) > 20:
                rnd = [random.uniform(self.min_rand, self.max_rand), random.uniform(
                    self.min_rand, self.max_rand)]
            else:
                rnd = [self.goal.x, self.goal.y]

            # finding node index in relation to the goal
            new_ind = self.getNearestIndex(self.nodes, rnd) 
    
            # place the index into a new node to generate the new node
            nearestNode = self.nodes[new_ind]
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)

            newNode = copy.deepcopy(nearestNode)
            newNode.x += self.max_length * math.cos(theta)
            newNode.y += self.max_length * math.sin(theta)
            newNode.parent = new_ind

            # check if the node is in an obstacle
            # if not, add to list of nodes
            if not self.checkCollisions(newNode, self.obstacle_list):
                continue
            self.nodes.append(newNode)
            print("Number of Nodes so far: ", len(self.nodes))

            if animation:
                self.drawGraph()

            # check if the node hit the goal
            if self.checkGoal(newNode, self.goal):
                self.nodes.append(self.goal)
                if animation:
                    self.drawGraph()
                print("We hit the goal!")
                break
        
        # creating the final path that hit the goal

        # starting from the goal node and working backwards
        path = [[self.goal.x, self.goal.y]]
        last_ind = len(self.nodes) - 1

        # loops through each parent and appends to list of path
        while self.nodes[last_ind].parent is not None:
            node = self.nodes[last_ind]
            path.append([node.x, node.y])
            last_ind = node.parent
        
        # including the starting location
        path.append([self.start.x, self.start.y])

        return path


    def checkCollisions(self, node, obstacle_list):
        # not completely sure how the obstacle_list will work but I'm imaginging
        # something like a square with position and sizing
        for (x, y, size) in obstacle_list:
            dx = x - node.x
            dy = y - node.y
            d = math.sqrt(dx**2 + dy**2)
            if d <= (size):
                return False
            return True
    
    def checkGoal(self, node, goal):
        dx = node.x - goal.x
        dy = node.y - goal.y
        d = math.sqrt(dx**2 + dy**2)
        if d <= self.max_length:
            return True

    def getNearestIndex(self, nodes, rnd):
        distance = [(math.sqrt((node.x - rnd[0]) ** 2 + (node.y - rnd[1])** 2)) for node in nodes]
        min_index = distance.index(min(distance))
        return min_index

    def drawGraph(self):
        for node in self.nodes:
            if node.parent is not None:
                plt.plot([node.x, self.nodes[node.parent].x], [node.y, self.nodes[node.parent].y], "-g")
        
        for (x, y, size) in self.obstacle_list:
            plt.plot(x, y, "sk", ms = size*20)
        
        plt.plot(self.start.x, self.start.y, "or")
        plt.plot(self.goal.x, self.goal.y, "or")
        plt.axis([0, 20, 0, 20])
        plt.grid(True)
        plt.pause(0.01)

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


def main():
    # set the start, goal, obstacles
    # everything needed to graph (still working on it)
    print("RRT")

    obstacleList = [(5,5,2)]

    rrt = RRT(start = [0, 0], goal = [10, 10], obstacle_list = obstacleList, max_length = 1, random_area=[0, 20])
    grid = rrt.path_planning(animation=show_animation)

    if show_animation:
        rrt.drawGraph()
        plt.plot([x for (x, y) in grid], [y for (x, y) in grid], '-r')
        plt.grid(True)
        plt.show()

if __name__ == '__main__':
    main()
