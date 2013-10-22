#!/usr/bin/env python
import roslib; roslib.load_manifest('ControlNode')
import rospy
from geomerty_msgs.msg import Twist, Vector3

class talker(object):
    global posprint
    def __init__(self):
        self.x  = 0
        self.omega = 0
        self.command_map = {
            'x'     : (self.cmd_x,   'set x velocity [m/s]'),
            'stop'     : (self.cmd_stop,   'Stop!'),
            'omega' : (self.cmd_theta,    'set angular velocity [rad/s]')}
        self.publisher = rospy.Publisher('cmd_vel', Twist)
        rospy.init_node('ControlNode')
        
    def send_cmd_msg(self):
        msg = Twist()
        msg.linear = Vector3()
        msg.angular = Vector3()
        msg.linear.x = self.x
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = self.omega
        self.publisher.publish(msg)        

    def cmd_x(self, args):
        usage = "usage: x [speed m/s]"
        if len(args) < 2:
            print(usage)
        self.x = float(args[1])
        self.send_cmd_msg()
        
    def cmd_stop(self, args):
        self.x = 0
        self.omega = 0
        self.send_cmd_msg()
        
    def cmd_omega(self, args):
        usage = "usage: omega [angular speed rad/s]"
        if len(args) < 2:
            print(usage)
        self.omega = float(args[1])
        self.send_cmd_msg()
                 
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
        rospy.init_node('ControlNode')
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
       
