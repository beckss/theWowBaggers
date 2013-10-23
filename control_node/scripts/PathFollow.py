#!/usr/bin/env python
import roslib; roslib.load_manifest('control_node')
import rospy
from geometry_msgs.msg import Twist, Vector3, PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path#Odometry
from std_msgs.msg import Header
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import sys
import numpy as np

def new_header():
    header = Header()
    header.frame_id = "world"
    header.stamp = rospy.Time.now()
    return header

def distance2(a, b):
    return (b[0]-a[0])*(b[0]-a[0]) + (b[1]-a[1])*(b[1]-a[1])

def add(a, b):
    return [a[0]+b[0], a[1]+b[1]]
    
def neg(a):
    return [-a[0], -a[1]]
    
def normalize(a):
    length = math.sqrt(a[0]*a[0]+a[1]*a[1])
    return [a[0]/length, a[1]/length]
    
def scale(a, s):
    return[a[0]*s, a[1]*s]
    
def getProjection(point, linea, dirVect):
    if dirVect[0] == 0:
        distance = (point[0] - linea[0]) * dirVect[1]
    elif dirVect[1] == 0:
        distance = (-point[1] + linea[1]) * dirVect[0]
    else:
        distance = ((point[0] - linea[0])/dirVect[0] - (point[1] - linea[1])/dirVect[1])/2
    return add(point, scale([-dirVect[1], dirVect[0]], distance))

class talker(object):
    global posprint
    def __init__(self):
        #self.path  = [[0, 0], [2, 0], [2, -1], [1, -1], [1, 0], [2, 0]]
        self.path = [[-0.76, -3.5], [-3, -3.56], [-2.93, -2.3], [-2.47, -1.23], [-2.45, -0.07], [-2.42, 0.51], [-2.4, 1.25], [-1.06, 1.24], [0.94, 1.26], [1.5, 1.26]]
        self.followDistance = 0.5
        self.P = 0.5
        self.speed = 0.1
        self.flagAdjust = False
        self.halt = True
        self.debugPrint = False
        self.command_map = {
            's'     : (self.cmd_stop,   'Stop!'),
            'start' : (self.cmd_start,  'Start!'),
            'stat' :  (self.cmd_status,  'print status'),
            'deb' :  (self.cmd_togglePrint,  'enable/disable debug print'),
            'adjust': (self.cmd_adjust, 'Adjust path start to current position')}
        
    def isPassed(self, currentPos, point, pointBack, pointFront):
        vectorBack = normalize(add(pointBack, neg(point)))
        vectorFront = normalize(add(pointFront, neg(point)))
        distFront = distance2(add(point, vectorBack), currentPos)
        distBack = distance2(add(point, vectorFront), currentPos)
        if(distBack > distFront):
            return False
        else:
            return True
            
    def adjustPath(self, robotPose):
        robotCoord = [robotPose.position.x, robotPose.position.y]
        print('robot coord: '+str(robotCoord))
        diff = add(robotCoord, neg(self.path[0]))
        for i in range(len(self.path)):
            self.path[i] = add(self.path[i], diff)
        print('path adjusted:')
        print(self.path)
        
    def control_angle(self, robotPose):
        robotCoord = [robotPose.position.x, robotPose.position.y]
        quaternionArray = np.array([robotPose.orientation.x, robotPose.orientation.y,
                           robotPose.orientation.z, robotPose.orientation.w])
        (roll, pitch, robotYaw) = euler_from_quaternion(quaternionArray)
        #remove passed points
        while (len(self.path) > 2) and self.isPassed(robotCoord, self.path[1], self.path[0], self.path[2]):
            self.path.pop(0)
            print('waypoint removed. Remaining: '+str(len(self.path)))
        #now the first point in path is the one behind us, the second is the one ahead
        dirVect = normalize(add(self.path[1], neg(self.path[0])))
        projection = getProjection(robotCoord, self.path[0], dirVect)
        followPoint = add(projection, scale(dirVect, self.followDistance))
        if self.debugPrint:
            print('preliminary followpoint: '+str(followPoint))
        #!TODO not checking if follow point is in the next segment
        for i in range(len(self.path)-2):
            segLen2 = distance2(self.path[i], self.path[i+1])
            followLen2 = distance2(self.path[i], followPoint)
            if followLen2 > segLen2:
                #should place it to the next segment
                extraLen = math.sqrt(followLen2) - math.sqrt(segLen2)
                dirVect = normalize(add(self.path[i+2], neg(self.path[i+1])))
                followPoint = add(self.path[i+1], scale(dirVect, extraLen))
                if self.debugPrint:
                    print('followpoint changed to: '+str(followPoint))
                    print('extra length: '+str(extraLen)+' towards '+str(dirVect))
            else:
                break
        self.send_debug_msg(followPoint, math.atan2(dirVect[1], dirVect[0]))
        positionError = add(neg(robotCoord), followPoint)
        targetYaw = math.atan2(positionError[1], positionError[0])
        angleError = targetYaw - robotYaw
        while angleError > math.pi:
            angleError -= 2*math.pi
        while angleError < -math.pi:
            angleError += 2*math.pi
        return angleError * self.P
        
    def handle_pose_msg(self, robotPose):
        posePart = robotPose.pose
        if self.flagAdjust:
            self.flagAdjust = False
            self.adjustPath(posePart)
        elif self.halt:
            self.send_cmd_msg(0, 0)
        else:
            self.control(posePart)
            
    def handle_path_msg(self, path):
        self.path = []
        hal = self.halt
        self.halt = True
        for pos in path.poses:
            path.append([pos.pose.position.x, pos.pose.position.y])
        self.halt = hal
            
    def control(self, robotPose):
        if len(self.path) >= 3:
            z = self.control_angle(robotPose)
            x = self.speed
        else:
            z = 0
            x = 0
        self.send_cmd_msg(x, z)
        #print('omega: '+str(z))
        
    def send_cmd_msg(self, speed, omega):
        #print('receieved odometry')
        msg = Twist()
        msg.linear = Vector3()
        msg.angular = Vector3()
        msg.linear.x = speed
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = omega
        self.publisher.publish(msg)
        
    def send_debug_msg(self, pos, angle):
        #print('receieved odometry')
        msg = PoseStamped()
        msg.pose = Pose()
        msg.header = new_header()
        msg.pose.position = Point()
        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.orientation = Quaternion()
        (x, y, z, w) = quaternion_from_euler(0, 0, angle)
        msg.pose.orientation.x = x
        msg.pose.orientation.y = y
        msg.pose.orientation.z = z
        msg.pose.orientation.w = w
        self.publishDebug.publish(msg)
        
    def cmd_stop(self, args):
        print('stop command')
        self.halt = True
        
    def cmd_start(self, args):
        print('start command')
        self.halt = False
        
    def cmd_adjust(self, args):
        self.flagAdjust = True
        
    def cmd_togglePrint(self, args):
        self.debugPrint = not self.debugPrint
        print('debug print: '+str(self.debugPrint))
        
    def cmd_status(self, args):
        print(self.path)
                 
    def process_stdin(self, line):
        '''handle commands from user'''
        if line is None:
            sys.exit(0)
        line = line.strip()
    
        if not line:
            return
    
        args = line.split()
        cmd = args[0]
            
        if cmd in ['help', 'h']:
            k = self.command_map.keys()
            k.sort()
            for cmd in k:
                (fn, help) = self.command_map[cmd]
                print("%-15s : %s" % (cmd, help))
            return
        if not cmd in self.command_map:
            print("Unknown command '%s'" % line)
            return
        (fn, help) = self.command_map[cmd]
        try:
            fn(args)
        except Exception as e:
            print("ERROR in command: %s" % str(e))
        
    def run(self):
        self.publisher = rospy.Publisher('cmd_vel', Twist)
        self.publishDebug = rospy.Publisher('control_debug', PoseStamped)
        rospy.init_node('ControlNode')
        #self.poseTopic = rospy.Subscriber('odom', Odometry, self.send_cmd_msg)
        self.poseTopic = rospy.Subscriber('robot/0/pose', PoseStamped, self.handle_pose_msg)
        self.poseTopic = rospy.Subscriber('robot_path', Path, self.handle_path_msg)
        while(True):
            input_command = raw_input('>')
            cmds = input_command.split(';')
            for c in cmds:
                self.process_stdin(c)


if __name__ == '__main__':
    talkR = talker()
    try:
        talkR.run()
    except rospy.ROSInterruptException:
        pass
       
