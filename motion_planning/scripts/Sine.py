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
        f = (1/(path[0][1] - path[1][1]) * math.pi)
        interval = abs(path[0][1] - path[1][1])/freq
        for dy in range(path[0][1], (path[len(path)-1][1])+1):
            for i in range(freq+1):
                y = y + interval
                x = (m*math.sin((f*y)-h))+k
                parab_path += [[-x,y,path[0][2]]]
    # x-direction: y=msin(fx-h)+k
    elif path[0][1] - path[1][1] == 0:
        f = (1/(path[1][0] - path[0][0]) * math.pi)
        interval = abs(path[0][0] - path[1][0])/freq
        for dx in range(path[0][0], (path[len(path)-1][0])+1):
            for i in range(freq+1):
                x = x + interval
                y = (m*math.sin((f*x)-h))+k
                parab_path += [[x,y,path[0][2]]]
    
    # diagonal-direction: 
    elif abs(path[0][0] - path[1][0]) == 1:
        xlength = pow((path[len(path)-1][0] - path[0][0]),2)
        ylength = pow((path[len(path)-1][1] - path[0][1]),2)
        distance = int(pow((xlength+ylength), 0.5))

        f = (1/(path[1][0] - path[0][0]) * math.pi)
        a = (math.pi/4)
        interval = abs(path[0][0] - path[1][0])/freq
        for dx in range(path[0][0], distance):
            for i in range(freq+1):
                x = x + interval
                y = (m*math.sin((f*x)-h))+k
                xcoord = ((x-h)*math.cos(-a) + (y-k)*math.sin(-a) + h)
                ycoord = (-(x-h)*math.sin(-a) + (y-k)*math.cos(-a) + h)
                parab_path += [[xcoord, ycoord, path[0][2]]]

    
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

