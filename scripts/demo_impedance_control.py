#!/usr/bin/env python3
import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import Wrench, TwistStamped

K = 45.  # stiffness
D = 2.5  # velocity damping
G = 0.001 # acceleration damping

class Node:

    hz = 100
    dt = 1.0/float(hz)

    def __init__(self):
        rospy.init_node('demo_node')

        Kx = rospy.get_param('~Kx', K)
        Ky = rospy.get_param('~Ky', K)
        Kz = rospy.get_param('~Kz', K)
        self.K = np.diag([Kx, Ky, Kz])

        Dx = rospy.get_param('~Dx', D)
        Dy = rospy.get_param('~Dy', D)
        Dz = rospy.get_param('~Dz', D)
        self.D = np.diag([Dx, Dy, Dz])

        Gx = rospy.get_param('~Gx', G)
        Gy = rospy.get_param('~Gy', G)
        Gz = rospy.get_param('~Gz', G)
        self.G = np.diag([Gx, Gy, Gz])

        self.tf_buffer = tf2_ros.Buffer()
        self.pub = rospy.Publisher('cmd_force', Wrench, queue_size=1)
        self.ee_vel = np.zeros(3)
        self.ee_acc = np.zeros(3)
        self.dt_vel = 1.0/200.  # twist messages are published at 200Hz
        rospy.Subscriber('twist', TwistStamped, self.twist_callback)
        tf2_ros.TransformListener(self.tf_buffer)
        self.ee_pos_goal = np.zeros(3)
        self.ee_vel_goal = np.zeros(3)
        self.ee_acc_goal = np.zeros(3)
        self.fig8 = rospy.get_param('~fig8', False)
        self.start_time = rospy.Time.now().to_sec()
        rospy.Timer(rospy.Duration(self.dt), self.main_loop)

    def twist_callback(self, msg):
        ee_vel = np.array([
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
        ])
        self.ee_acc = (ee_vel - self.ee_vel) / self.dt_vel
        self.ee_vel = ee_vel

    def get_ee_pos(self):
        try:
            tf = self.tf_buffer.lookup_transform('touch_x_base', 'touch_x_ee', rospy.Time())
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            tf = None
        return tf

    def main_loop(self, event):

        # Get ee position
        ee = self.get_ee_pos()
        if ee is None: return
        ee_pos = np.array([
            ee.transform.translation.x,
            ee.transform.translation.y,
            ee.transform.translation.z,
        ])

        # Update goal position
        if self.fig8:
            t = rospy.Time.now().to_sec() - self.start_time
            self.ee_pos_goal[0] = 0.04*np.sin(t*np.pi*0.5)
            self.ee_pos_goal[1] = 0.04*np.sin(t*np.pi)

        # Impedance controller
        f = self.K@(self.ee_pos_goal - ee_pos) + self.D@(self.ee_vel_goal - self.ee_vel) + self.G@(self.ee_acc_goal - self.ee_acc)

        # Publish wrench
        msg = Wrench()
        msg.force.x = f[0]
        msg.force.y = f[1]
        msg.force.z = f[2]
        self.pub.publish(msg)

    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__ == '__main__':
    main()
