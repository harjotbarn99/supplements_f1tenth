import numpy as np

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


RED = ColorRGBA(1,0,0,1)
GREEN = ColorRGBA(0,1,0,1)
BLUE = ColorRGBA(0,0,1,1)
NO_COLOR= ColorRGBA(0,0,0,0)


class ColorGrid:
    wall = BLUE
    robot = NO_COLOR
    padding = BLUE


class TypeGrid:
    wall = 1
    robot = -1
    padding = 2


class Grid:
    null_grid = np.zeros(1)
    angle_min = -1*np.pi
    angle_increment = 2*np.pi/1080
    rot_map_to_grid = np.pi/2
    rot_grid_to_map = -np.pi/2
    robot = Point(0,0,0)

    factor = 20 # 100 for 1 cm - 1 unit

# untis in meters 
    def __init__(self,width=5, length=8) -> None:
        self.null_grid = np.zeros([length*self.factor,width*self.factor],np.int16)
        self.robot = Point(self.factor*width//2,self.factor*length//6,0)
        self.null_grid[self.robot.y][self.robot.x] = TypeGrid.robot
        self.grid = self.null_grid.copy()
        self.build = self.null_grid.copy()

# called and given list of ranges
    def build_grid(self,ranges):
        self.build = self.null_grid.copy()
        angle = self.angle_min + self.rot_map_to_grid
        for r in ranges :
            x = r*np.cos(angle)
            y = r*np.sin(angle)
            self.mark(x,y,TypeGrid.wall)
            angle += self.angle_increment
        self.grid = self.build
        return

# units given in meters
# return units according to grid
    def to_grid(self,x,y):
        return (int(x*self.factor)+self.robot.x, int(y*self.factor)+self.robot.y)

    def mark(self,x_b,y_b,mark_type):
        x, y = self.to_grid(x_b,y_b)
        if x > self.grid.shape[1] or x < 0 or y > self.grid.shape[0] or y < 0:
            #TODO: handle gap
            return
        self.build[y-1][x-1] = mark_type
        #TODO: add padding
        return

    def to_map(self,x,y):
        x, y =  ((x-self.robot.x)/self.factor, (y-self.robot.y)/self.factor)
        xr = np.cos(self.rot_grid_to_map)*x - np.sin(self.rot_grid_to_map)*y
        yr = np.sin(self.rot_grid_to_map)*x + np.cos(self.rot_grid_to_map)*y
        return (xr,yr)


class EX():

    grid: Grid = None

    def __init__(self):
        scan_topic = "/scan"
        self.grid = Grid()
        rospy.Subscriber(scan_topic, LaserScan, self.scan_callback)
        self.visualizer = Visualizer()


    def scan_callback(self, scan_msg):
        self.grid.build_grid(scan_msg.ranges)
        self.visualizer.publish_grid(self.grid)
        # print_grid(self.grid)
        # print("\n\n---------------------------\n\n")
        return
        

def get_grid_color(grid_type):
    # TODO: after adding padding add viz for padding
    # TODO: add viz for gap
    if TypeGrid.wall == grid_type:
        return ColorGrid.wall
    else :
        return NO_COLOR

class Visualizer:

    def __init__(self) -> None:
        self.viz_pub = rospy.Publisher("/rrt_viz",Marker,queue_size=10)
        self.grid_markers = self.init_marker(0,"grid_points",Marker.POINTS, RED,0.025)


    def init_marker(self, id, name_space,type_marker,color:ColorRGBA,duration = 0.05,action=Marker.ADD)->Marker:
        marker = Marker()
        marker.header.frame_id = "laser"
        marker.ns = name_space
        marker.type = type_marker

        marker.lifetime = rospy.Duration(duration)
        marker.action = Marker.ADD
        marker.pose.position.z = 0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color = color
        marker.id = id
        return marker

    count_grid = 0

    def publish_grid(self,grid:Grid):
        self.grid_markers.points=[]
        self.grid_markers.colors=[]
        w , l = grid.grid.shape
        y = 0
        for i in grid.grid: 
            x = 0
            for j in i :
                #  plot
                x_m, y_m = grid.to_map(x,y)
                self.grid_markers.points.append(Point(x_m,y_m,0))
                self.grid_markers.colors.append(get_grid_color(j))
                x +=1
            y+=1
        # self.count_grid +=1
        # print("sent ",self.count_grid)
        self.viz_pub.publish(self.grid_markers)


 
def print_grid(grid:Grid, b = False):
    x = 0
    gr = ""
    g = grid.grid
    if b == True:
        g = grid.null_grid
    for ar in g:
        y = 0
        s = ""
        for j in ar:
            s+=str(int(j))
        gr = s+"\n"+gr
    print(gr)
    return


def main():
    rospy.init_node('example')
    ex = EX()
    rospy.spin()

if __name__ == '__main__':
    main()