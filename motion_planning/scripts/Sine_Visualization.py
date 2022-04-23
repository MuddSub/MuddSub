from Sine import getmultpaths
from Sine import parabola
from Sine import printPath
import numpy as np
import a_star
from matplotlib import colors
from matplotlib import pyplot as plt 



def use_astar(width, length):
    grid = a_star.makeGrid(width,length,0.3,0,0,width-1,length-1, "e")
    a_star.solveGrid(grid, 0, 0, width-1, length-1)
    path = a_star.makeParent(grid, grid[width-1][length-1])
    print("path", path)

    for i in range(len(grid)):
        for j in range(len(grid[0])):
            print(grid[i][j], end = "")
        print()
    
    data = a_star.printGrid(grid)

    return data, grid, path

def sine_visualization(width = 20, length = 20, amp = 4, freq = 10, conversion = 3):

    
    data, grid, path = use_astar(width, length)



    mult_paths = getmultpaths(path, amp, freq)

    paths = []
    
    for path in mult_paths:
        print("\n\n")
        path = path[::-1]
        print(path)
        path_i = parabola(path, amp, freq)
        print(path_i)
        paths += [path_i]
    
    get_new_data(mult_paths, data, paths, width, length, conversion)



def get_new_data(mult_paths, data, paths, width, length, conversion):
    new_width = (int) ((width * conversion) * 1.0)
    new_length = (int)((length * conversion) * 1.0)
    arr = np.zeros((new_width, new_length))

    for i in range(width):
        for j in range(length):
            x = (int)(i * conversion)
            y = (int)(j * conversion)

            if(x > new_width or y > new_length):
                continue
            else:
                arr[x][y] = data[i][j]
    index = 4
    for path in paths:
        for i in range(len(path)):
            
            x = (int)(path[i][0] * conversion) 
            y = (int)(path[i][1] * conversion) 

            print(x,y)

            if(x >= new_width or y >= new_length):
                continue
            else:
                if(arr[x][y] != 3):
                    arr[x][y] = index
        if(index == 4):
            index = 5
        else:
            index = 4
    

    
    visualize(arr, new_width, new_length)

def visualize(data, width, height):
    cmap = colors.ListedColormap(['lightblue', 'red', 'yellowgreen', 'green', 'purple' , 'fuchsia'])
    bounds = [0,1,2,3,4,5,6 ]
    norm = colors.BoundaryNorm(bounds, cmap.N)

    fig, ax = plt.subplots()
    ax.imshow(data, cmap=cmap, norm=norm)

    # draw gridlines
    #ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=1)
    #ax.set_xticks(np.arange(-0.5, width, 1))
    #ax.set_yticks(np.arange(-0.5, height, 1))
    # ax.set_xticklabels([])
    # ax.set_yticklabels([])

    plt.show()

    plt.savefig("Sine_Visualization.png")
    




example_paths = [[[18, 19.0], [18.33, 20.73], [18.67, 20.73], [19.0, 19.0]], [[14.0, 14.0], [13.01, 15.46], [13.25, 15.7], [14.71, 14.71], [16.17, 13.72], [16.4, 13.95], [15.41, 15.41], [14.43, 16.87], [14.66, 17.11], [16.12, 16.12], [17.58, 15.13], [17.82, 15.37]], [[-13.0, 14], [-11.27, 14.33], [-11.27, 14.67], [-13.0, 15.0]], [[12.0, 12.0], [11.01, 13.46], [11.25, 13.7], [12.71, 12.71]], [[-11.0, 10], [-12.73, 10.33], [-12.73, 10.67], [-11.0, 11.0]], [[9, 9.0], [9.33, 10.73], [9.67, 10.73], [10.0, 9.0]], [[6.0, 6.0], [5.01, 7.46], [5.25, 7.7], [6.71, 6.71], [8.17, 5.72], [8.4, 5.95], [7.41, 7.41], [6.43, 8.87]], [[1.0, 1.0], [0.01, 2.46], [0.25, 2.7], [1.71, 1.71], [3.17, 0.72], [3.4, 0.95], [2.41, 2.41], [1.43, 3.87], [1.66, 4.11], [3.12, 3.12], [4.58, 2.13], [4.82, 2.37], [3.83, 3.83], [2.84, 5.29], [3.08, 5.52], [4.54, 4.54]]]

example_data = [[3, 2, 2, 1, 1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0], [2, 3, 1, 2, 1, 2, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0], [2, 2, 3, 2, 2, 2, 2, 2, 2, 1, 1, 
0, 1, 0, 0, 0, 1, 0, 0, 0], [2, 1, 2, 3, 2, 2, 1, 2, 2, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0], [2, 2, 2, 2, 3, 2, 2, 2, 2, 2, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0], [1, 2, 
2, 2, 2, 3, 3, 2, 2, 2, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1], [2, 2, 2, 2, 1, 1, 1, 3, 2, 2, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0], [0, 2, 2, 2, 2, 1, 1, 2, 3, 2, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0], [0, 0, 2, 1, 1, 2, 2, 2, 1, 3, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 2, 1, 2, 2, 2, 3, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1], [0, 1, 0, 0, 1, 2, 2, 2, 2, 3, 1, 2, 2, 0, 0, 0, 0, 0, 1, 1], [1, 0, 0, 0, 1, 0, 1, 2, 2, 2, 3, 3, 2, 2, 2, 0, 0, 1, 0, 1], [1, 0, 0, 0, 0, 0, 0, 2, 2, 2, 1, 1, 3, 2, 1, 0, 0, 0, 1, 1], [1, 0, 0, 0, 1, 1, 1, 0, 2, 1, 1, 2, 1, 3, 3, 3, 2, 0, 0, 0], [1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 2, 1, 1, 1, 3, 2, 1, 1], [0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 2, 1, 0, 2, 3, 2, 0], [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 2, 3, 1], [0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 2, 1, 
3], [1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 3], [0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 3]]






def main():
    sine_visualization()

if __name__ == "__main__":
    main()



