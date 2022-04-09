from audioop import mul
from Sine import getmultpaths
from Sine import parabola
from Sine import printPath
import a_star


def use_astar(width, length):
    grid = a_star.makeGrid(width,length,0.3,0,0,width-1,length-1, "e")
    a_star.solveGrid(grid, 0, 0, width-1, length-1)
    path = a_star.makeParent(grid, grid[width-1][length-1])
    print("path", path)

    for i in range(len(grid)):
        for j in range(len(grid[0])):
            print(grid[i][j], end = "")
        print()
    
    a_star.printGrid(grid)

    return grid, path

def sine_visualization(width = 20, length = 20, amp = 2, freq = 3):
    grid, path = use_astar(width, length)

    mult_paths = getmultpaths(path, amp, freq)

    
    
    for path in mult_paths:
        print("\n\n")
        path = path[::-1]
        print(path)
        path_i = parabola(path, amp, freq)
        print(path_i)




def main():
    sine_visualization()

if __name__ == "__main__":
    main()



