import matplotlib.pyplot as plt
import numpy as np

# the coordinate boundries
XMIN = -75.86602778
XMAX = -75.88267778
YMIN = 39.0746
YMAX = 39.08871111

# represents a 1d obstacle
class line:
    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2
    
    def plot(self):
        plt.plot([self.p1[0], self.p2[0]], [self.p1[1], self.p2[1]], color = 'black')

    def intersect(self, A, B):
        return (ccw(A,self.p1,self.p2) != ccw(B,self.p1,self.p2)) and (ccw(A,B,self.p1) != ccw(A,B,self.p2)) and not on_line(A, self) and not on_line(B, self)

# represents a 2d obstacle
class quad:
    def __init__(self, p1, p2, p3, p4):
        self.l1 = line(p1, p3)
        self.l2 = line(p2, p4)

    def getl1(self):
        return [(self.l1.p1[0], self.l1.p1[1]), (self.l1.p2[0], self.l1.p2[1])]

    def getl2(self):
        return [(self.l2.getx1(), self.l2.gety1()), (self.l2.getx2(), self.l2.gety2())]
    
    def plot(self):
        self.l1.plot()
        self.l2.plot()

    def intersect(self, A, B):
        return self.l1.intersect(A,B) or self.l2.intersect(A,B)

# determines if the three points follow a clockwise or counterclockwise direction
def ccw(A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

# determines if a point is on a given line
def on_line(point, line):
    if point[1] == line.p1[1]:
        return point[0] == line.p1[0]

    if point[1] == line.p2[1]:
        return point[0] == line.p2[0]

    return ((point[0]-line.p1[0]) / (point[1]-line.p1[1]) - (line.p2[0]-point[0]) / (line.p2[1]-point[1])) == 0

# Initial obstacles
obstacles = []
obstacles.append(line((-75.87439444, 39.079425), (-75.87331111, 39.08091111)))
obstacles.append(line((-75.87320833, 39.07993889), (-75.871675, 39.07527778)))
#obstacles.append(quad((-75.87740556, 39.08141944), (-75.87347222, 39.08063056), (-75.87551389, 39.07835278), (-75.877625, 39.07926667)))
obstacles.append(line((-75.87404444, 39.08616667), (-75.87263056, 39.08143889)))

# Optimized obstacles
obstacles.append(line((-75.87740556, 39.08141944), (-75.87596, 39.07918)))


for obs in obstacles:
    obs.plot()


# target points
target_points = [(-75.87514167, 39.08471667), (-75.87820833, 39.077525), (-75.87034444, 39.08389722)]

for point in target_points:
    x = point[0]
    y = point[1]
    
    plt.plot(x, y, marker = 'x', color='blue')

invalids = []

# checks for numerous different x and y values
for x in np.linspace(XMIN, XMAX, 50):
    for y in np.linspace(YMIN, YMAX, 50):
        has_valid = False

        # determines if there is at least one valid target point
        for point in target_points:
            valid = True

            # if it intersects any obstacle, it's an invalid target point
            for obs in obstacles:
                if obs.intersect((x,y), point):
                    valid = False
                    break

            if valid:
                has_valid = True
                break
        
        if not has_valid:
            plt.plot(x, y, marker = 'o', color='red')
            invalids.append((x,y))
        

if len(invalids) > 0:
    print("The target points are insufficient. These starting locations have no valid target points: " , invalids)

else:
    print("WOOOOOOOOOOHHHOOOOOOOOOO")

plt.xlim(XMAX, XMIN)
plt.ylim(YMIN,YMAX)
plt.show()
