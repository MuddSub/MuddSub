import math

def plot_sine(path, period, amp):
    newL = [path[0]]
    side = -amp
    count = 1
    for i in path[int(period/4)::int(period/4)]:
        if path[0][0] - path[1][0] == 0:
            newL += [[side+i[0], i[1], i[2]]]
        elif path[0][1] - path[1][1] == 0:
            newL += [[i[0], side+i[1], i[2]]]
        elif abs(path[0][0] - path[1][0]) == 1:
            newL += [[i[0]+side, i[1]-side, i[2]]]

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

def parabola(path, period, amp):
    p_L = path[]
    

