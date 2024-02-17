#! usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

print("Starting ...")
class odometry_cm:
    import math
    r = 0.0
    d = 0.0
    theta = 0.0

    def __init__(self) -> None:
        pass

    def callback_f(self, data):
        odom = data
        front_right_wheel_vel = odom.velocity[0]
        front_left_wheel_vel = odom.velocity[1]
        back_right_wheel_vel = odom.velocity[2]
        back_left_wheel_vel = odom.velocity[3]
        position = odom.position
        print(f"Position : {position}")
        print(f"Velocity :\nFront Right wheel :{front_right_wheel_vel}    Front Left wheel :{front_left_wheel_vel}\nBack Right wheel :{back_right_wheel_vel}    Back Left wheel :{back_left_wheel_vel}")
        

    def odometry(self):
        rospy.init_node("odometry", anonymous=False)
        rospy.Subscriber("/prometheus_final/joint_states", JointState, self.callback_f)
        self.data = rospy.wait_for_message("/joint_states", JointState)
        rospy.spin()


#rate = rospy.Rate(1)
if __name__ == '__main__':
    robot = odometry_cm()
    robot.odometry()
    #rate.sleep()