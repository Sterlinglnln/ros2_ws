import sys
import numpy as np
from matplotlib import pyplot as plt

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = sys.maxsize
        self.basecost = sys.maxsize
    
    def __eq__(self, __o):
        return self.x == __o.x and self.y == __o.y
    
    def dist(self, ptb):
        dx = ptb.x - self.x
        dy = ptb.y - self.y
        c = (dx**2 + dy**2)**0.5
        rad = np.arctan2(dy, dx)
        return c, rad

class AStar:
    def __init__(self, mp):
        self.map = mp
        self.size = mp.shape
        self.path = []
        self.open_set = []
        self.close_set = []
    
    def IsObstacle(self, x, y):
        return self.map[y, x] > 0
    
    def BaseCost(self, s, p):
        if s == p:
            return 0
        x_dis = p.x - p.parent.x
        y_dis = p.y - p.parent.y
        return (x_dis**2 + y_dis**2)**0.5 + p.parent.basecost
    
    def HeuristicCost(self, p, g):
        x_dis = g.x - p.x
        y_dis = g.y - p.y
        return (x_dis**2 + y_dis**2)**0.5
    
    def TotalCost(self, s, p, g):
        bc = self.BaseCost(s, p)
        p.basecost = bc
        return bc + self.HeuristicCost(p, g)
    
    def IsValidPoint(self, x, y):
        if x < 0 or y < 0:
            return False
        if x >= self.size[1] or y >= self.size[0]:
            return False
        return not self.IsObstacle(x, y)
    
    def IsInPointList(self, p, point_list):
        for idx, point in enumerate(point_list):
            if point == p:
                return idx
        return -1
    
    def IsInOpenList(self, p):
        return self.IsInPointList(p, self.open_set)
    
    def IsInCloseList(self, p):
        return self.IsInPointList(p, self.close_set)
    
    def ProcessPoint(self, x, y, parent):
        if not self.IsValidPoint(x, y):
            return
        curp = Point(x, y)
        if self.IsInCloseList(curp) > -1:
            return
        index = self.IsInOpenList(curp)
        if index > -1:
            p = self.open_set[index]
            if p.cost > (parent.basecost + p.dist(parent)[0] + self.HeuristicCost(p, self.g)):
                p.parent = parent
                p.cost = self.TotalCost(self.s, p, self.g)
                self.open_set[index] = p
        else:
            curp.parent = parent
            curp.cost = self.TotalCost(self.s, curp, self.g)
            self.open_set.append(curp)
    
    def SelectPointInOpenList(self):
        selected_index = -1
        min_cost = sys.maxsize
        for index, p in enumerate(self.open_set):
            if p.cost == sys.maxsize:
                cost = self.TotalCost(self.s, p, self.g)
                p.cost = cost
            else:
                cost = p.cost
            if cost < min_cost:
                min_cost = cost
                selected_index = index
        return selected_index
    
    def BuildPath(self, p):
        path = []
        while p is not None:
            path.insert(0, p)
            p = p.parent
        self.path = path
        return path
    
    def Run(self, s, g):
        self.path = []
        self.open_set = []
        self.close_set = []
        self.s = s
        self.g = g
        s.cost = 0
        s.basecost = 0
        self.open_set.append(s)
        while True:
            index = self.SelectPointInOpenList()
            if index < 0:
                print('No path found, algorithm failed!!!')
                return
            p = self.open_set[index]
            if p == g:
                return self.BuildPath(p)
            del self.open_set[index]
            self.close_set.append(p)
            x = p.x
            y = p.y
            self.ProcessPoint(x-1, y, p)
            self.ProcessPoint(x+1, y, p)
            self.ProcessPoint(x, y-1, p)
            self.ProcessPoint(x, y+1, p)
            self.ProcessPoint(x-1, y-1, p)
            self.ProcessPoint(x+1, y-1, p)
            self.ProcessPoint(x-1, y+1, p)
            self.ProcessPoint(x+1, y+1, p)
    
    def show(self):
        if self.path:
            z = np.copy(self.map)
            for p in self.path:
                z[p.y, p.x] = 150
            plt.imshow(z)
            plt.show()
    
    def update(self, newst):
        idx = None
        maxstep = 20
        if len(self.path) > maxstep:
            midp = self.path[maxstep]
            idx = maxstep
        elif len(self.path) > 0:
            midp = self.path[-1]
            idx = -1
        else:
            return self.path
        p1 = Point(newst.x, newst.y)
        path = AStar(self.map).Run(p1, midp)
        if idx >= maxstep:
            self.path = path + self.path[maxstep:]
        else:
            self.path = path
        return self.path
    
    def getspeed(self):
        if len(self.path) > 20:
            pt = self.path[20]
        else:
            pt = self.path[-1]
        st = self.path[0]
        return st.dist(pt)[0]

def main():
    print('Astar')

if __name__ == '__main__':
    mp = (np.random.random((120, 160)) > 0.95) * 100
    stx,sty = 10,10
    edx,edy = 150,110
    mp[sty,stx] = 0
    mp[edy,edx] = 0
    astar = AStar(mp)
    points = astar.Run(s = Point(stx,sty), g = Point(edx,edy))
    print(points)
    points(astar.getspeed())
    astar.show()
    while len(astar.path) > 20:
        cp = astar.path[5]
        astar.update(cp)
        print(astar.getspeed())
        astar.show()
