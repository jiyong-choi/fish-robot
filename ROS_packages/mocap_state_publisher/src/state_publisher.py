### Python 2 with ROS melodic ###

#!/usr/bin/env python
import rospy
import tf.transformations as tf
import math
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray

class Motioncapture:
    def __init__(self):
        rospy.loginfo("Initializing Motioncapture node...")

        self.rate = rospy.Rate(30) # publication Hz
        self.mocap_sub = rospy.Subscriber('/mocap_node/Robot_1/pose', PoseStamped, self.mocap_callback, queue_size=100)
        self.angle_pub = rospy.Publisher('/Fish_state', Float64MultiArray, queue_size=100)

        self.x = self.y = self.z = 0.0  # unit: meters
        self.quat_x = self.quat_y = self.quat_z = self.quat_w = 0.0
        self.roll = self.pitch = self.yaw = 0.0  # unit: degrees

    def mocap_callback(self, msg):
        position = msg.pose.position
        orientation = msg.pose.orientation

        self.x, self.y, self.z = position.x, position.y, position.z
        self.quat_x = orientation.x
        self.quat_y = orientation.y
        self.quat_z = orientation.z
        self.quat_w = orientation.w

    def run(self):
        while not rospy.is_shutdown():
            quat = [self.quat_x, self.quat_y, self.quat_z, self.quat_w]
            R = tf.quaternion_matrix(quat)
            euler_angles = tf.euler_from_matrix(R, axes='sxyz')
            self.roll, self.pitch, self.yaw = [math.degrees(angle) for angle in euler_angles]

            state_data = [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]
            state_msg = Float64MultiArray()
            state_msg.data = state_data

            self.angle_pub.publish(state_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('Robot_state', anonymous=True)
        mocap = Motioncapture()
        rospy.loginfo("Starting position & pose publication")
        mocap.run()
    except rospy.ROSInterruptException:
        pass
