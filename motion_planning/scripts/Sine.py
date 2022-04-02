import math

def plot_sine(path, period, amp):
    newL = [(path[0][0], path[0][1], path[0][2])]
    side = -amp
    count = 1
    for i in path[int(period/4)::int(period/4)]:
        if path[0][0] - path[1][0] == 0:
            newL += [(side+i[0], i[1], i[2])]
        elif path[0][1] - path[1][1] == 0:
            newL += [(i[0], side+i[1], i[2])]
        elif abs(path[0][0] - path[1][0]) == 1:
            newL += [(i[0]+side, i[1]-side, i[2])]

        count += 1
        if count == 1:
            side = -amp
        elif count == 2:
            side = 0
        elif count == 3:
            side = amp
        else:
            count = 0
            side = 0

    return newL

def line (slope, intercept, startx, endx, timestart):
    x = startx
    points = []
    t = timestart
    for i in range(startx, endx):
        y = slope * i + intercept
        points += [(i, y,t)]
        t+=1
    return points

def coordinates(path):
    coord = []
    for i in range(len(path)):
        coord += [(path[i][0], path[i][1])]
    return coord


def parabola(path, period, amp, freq):
    h = path[0][0]
    k = path[0][1]
    m = amp
    parab_path = []
    x = path[0][0]
    y = path[0][1]

    # y-direction: x=msin(fy-h)+k
    if path[0][0] - path[1][0] == 0: 
        mult = 1
        if(path[0][1] - path[0][0] < 0):
            mult = -1
        f = (1/(path[0][1] - path[1][1]) * math.pi) * mult
        endy = path[len(path)-1][1]
        interval = (path[1][1] - path[0][1])/freq 
    
        index = 0
        while(abs(y) <= (abs(endy) + 0.0001)):            
            x = (m*math.sin((f*(y-k))))+h
            parab_path += [[-x,y,path[index][2]]]
            y += interval
            if (y > mult * path[index][1]):
                index += 1
                index = min(index, len(path)-1)
                
    # x-direction: y=msin(fx-h)+k
    elif path[0][1] - path[1][1] == 0:
        mult = 1
        if(path[1][0] - path[0][0] < 0):
            mult = -1
        f = (1/(path[1][0] - path[0][0]) * math.pi) * mult
        interval = (path[1][0] - path[0][0])/freq
        lastx = path[len(path)-1][0]
        index = 0 #first z value is path[index][2]
        while(abs(x) <= (abs(lastx) + 0.0001)):
            y = (m*math.sin((f*(x-h))))+k
            parab_path += [[x,y,path[index][2]]]
            x = x + interval
            if (x > path[index][0]):
                index += 1
                index = min(index, len(path)-1)
            
                
    # diagonal-direction: 
    elif abs(path[0][0] - path[1][0]) == 1:
        multx = 1
        multy = 1
        if(path[1][0] - path[0][0] < 0):
            multx = -1
        if(path[1][1] - path[0][1] < 0):
            multy = -1

        xlength = pow((path[len(path)-1][0] - path[0][0]),2)
        ylength = pow((path[len(path)-1][1] - path[0][1]),2)
        distance = int(pow((xlength+ylength), 0.5)) * multx

        f = (1/(path[1][0] - path[0][0]) * math.pi) * multx
        a = (math.pi/4)
        interval = (path[1][0] - path[0][0])/freq
        index = 0
        for dx in range(abs(path[0][0]), (abs(distance))):
            for i in range(freq+1):
                y = (m*math.sin((f*(x-h))))+k
                xcoord = ((x-h)*math.cos(-a) + (y-k)*math.sin(-a) + h) 
                ycoord = (-(x-h)*math.sin(-a) + (y-k)*math.cos(-a) + h) * multy
                parab_path += [[xcoord, ycoord, path[index][2]]]
                x = x + interval
                if (abs(x) > abs(path[index][0])):
                    index += 1
                    index = min(index, len(path)-1)

    
    for point in parab_path:
        for i in range(len(point)):
            point[i] = round(point[i], 2)
    
    return parab_path


def parabola1(path, period, amp):
    if path[0][0] - path[1][0] == 0:
        p_E = sizing*(pow((y-horiz),2)) + amp
    elif path[0][1] - path[1][1] == 0:
        p_E = sizing*(pow((x-horiz),2)) + amp
    elif abs(path[0][0] - path[1][0]) == 1:
        newL += [[i[0]+side, i[1]-side, i[2]]]
    sizing = int(1/(path[1][0] - path[0][0]))
    horiz = path[1][0]
    p_E = sizing*(pow((x-horiz),2)) + amp

def split(path):
    n_path = []
    directions = []
    count = 0

    for i in range(0, len(path)-1):
        if path[i][0] - path[i+1][0] == 0:
            directions += [1]
        elif path[i][1] - path[i+1][1] == 0:
            directions += [2]
        elif abs(path[i][0] - path[i+1][0]) == 1:
            directions += [3]

    for i in range(1, len(directions)):
        if count == 0:
            n_path += [[path[i-1], path[i]]]
        count = 0

        if directions[i-1] == directions[i]:
            n_path[-1] += [path[i+1]]
            count += 1        

    if count == 0:
        n_path += [[path[-2], path[-1]]]

    return n_path

def time_stamp(path, velocity):
    for i in range(len(path)):
        path[i] += [i/velocity]            


def printPath(twodarray):
    for point in twodarray:
        print("(", point[0], ", ", point[1], ", ", point[2], "),", end = "")
    

def main():
    path1 = [[0, 1, 1], [0, 2, 2], [0, 3, 3], [0, 4, 4]] # Straight vertical line
    path2 = [[1, 0, 1], [2, 0, 2], [3, 0, 3], [4, 0, 4]] # Straight horizontal line
    path3 = [[0, 1, 1], [0, 3, 2], [0, 5, 3], [0, 6, 4]] # Vertical line (points not all same distance)
    path4 = [[1, 1, 1], [2, 2, 2], [3, 3, 3], [4, 4, 4]] # Diagonal line (up left)

    path5 = [(0, -1, 1), (0, -2, 2), (0, -3, 3), (0, -4, 4)] # Straight vertical line
    path6 = [(-1, 0, 1), (-2, 0, 2), (-3, 0, 3), (-4, 0, 4)] # Straight horizontal line
    path7 = [(0, -1, 1), (0, -3, 2), (0, -5, 3), (0, -6, 4)] # Vertical line (points not all same distance)
    path8 = [(-1, -1, 1), (-2, -2, 2), (-3, -3, 3), (-4, -4, 4)] # Diagonal line (down left)
    path9 = [(1, -1, 1), (2, -2, 2), (3, -3, 3), (4, -4, 4)] # Diagonal line (down right)
    path10 = [(-1, 1, 1), (-2, 2, 2), (-3, 3, 3), (-4, 4, 4)] # Diagonal line (up left)
    
    printPath(parabola(path4, 2, 2, 1))

if __name__=="__main__":
    main()