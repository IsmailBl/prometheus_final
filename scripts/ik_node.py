import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf import transformations
import numpy as np
import math

#initialization of global parameters of the robot
vel = Twist()
yaw = 0
max_vel_x = 4
max_vel_y = 4
max_angular_vel_z = 4
base_diag = 0.31112698372 * 2
wheel_radius = 0.127

# Set desired position
x_desired = 1.0  # meters
y_desired = 2.0  # meters

def vel_callback(c_vel):
    """
    Takes the Twist msg and sets the global velocity variable.
    """
    global vel
    vel = c_vel

def clbk_odom(msg):
    """
    Takes the Odometry msg and computes the orientation of the robot.
    """
    global yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]

def main():
    global vel, yaw, x_desired, y_desired

    print("####################################################")
    print("-----------INVERSE KINEMATICS INITIALIZED----------")
    print("####################################################")
    
    # Initialize ROS node and publishers/subscribers
    rospy.init_node("inverse_kinematics", anonymous=False)
    sub = rospy.Subscriber("/odom", Odometry, clbk_odom)
    pub1 = rospy.Publisher("/prometheus_assembly/rf_joint_velocity_controller/command", Float64, queue_size=10)
    pub2 = rospy.Publisher("/prometheus_assembly/lf_joint_velocity_controller/command", Float64, queue_size=10)
    pub3 = rospy.Publisher("/prometheus_assembly/lb_joint_velocity_controller/command", Float64, queue_size=10)
    pub4 = rospy.Publisher("/prometheus_assembly/rb_joint_velocity_controller/command", Float64, queue_size=10)
    
    # Rate at which to run the loop
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        # Compute distance and angle to desired point
        x_current = 3
        y_current = 5
        dx = x_desired - x_current
        dy = y_desired - y_current
        angle_to_point = math.atan2(dy, dx)
        
        # Compute the linear and angular velocity to drive the robot to the desired point
        x_dot = max_vel_x * math.cos(angle_to_point)
        y_dot = max_vel_y * math.sin(angle_to_point)
        theta_dot = max_angular_vel_z * (angle_to_point - yaw)
        
        # Normalize velocities so they don't exceed maximums
        if (x_dot < -max_vel_x):
            x_dot = -max_vel_x
        elif abs(x_dot) < max_vel_x:
            x_dot = x_dot
        else:
            x_dot = max_vel_x
            
        if (y_dot < -max_vel_y):
            y_dot = -max_vel_y
        elif abs(y_dot) < max_vel_y:
            y_dot = y_dot
        else:
            y_dot = max_vel_y
            
        if (theta_dot > max_angular_vel_z):
            theta_dot = max_angular_vel_z
        elif (vel.angular.z < -max_angular_vel_z):
            theta_dot = -max_angular_vel_z
        else:
            theta_dot = vel.angular.z

            	#float angular velocities for each wheel 
        omega1 = Float64()
        omega2 = Float64()
        omega3 = Float64()
        omega4 = Float64()

		#all the equations bellow are results of inverse kinematics mathematical computation , for deeper info check the readme file
        omega1.data = (-np.sin(yaw+1.75*math.pi)*x_dot + np.cos(yaw+1.75*math.pi)*y_dot + base_diag*theta_dot)/wheel_radius
        omega2.data = (-np.sin(yaw+0.25*math.pi)*x_dot + np.cos(yaw+0.25*math.pi)*y_dot + base_diag*theta_dot)/wheel_radius
        omega3.data = (-np.sin(yaw+1.25*math.pi)*x_dot + np.cos(yaw+1.25*math.pi)*y_dot + base_diag*theta_dot)/wheel_radius
        omega4.data = (-np.sin(yaw+0.75*math.pi)*x_dot + np.cos(yaw+0.75*math.pi)*y_dot + base_diag*theta_dot)/wheel_radius
		
#publishing the angular velocity to each wheel (conversion from robot motion to wheel velocity)
#
        pub1.publish(omega1)                                          
        pub2.publish(omega2)                                          
        pub3.publish(omega3)                                         
        pub4.publish(omega4)
        rate.sleep()


if __name__ == '__main__':
	main()



