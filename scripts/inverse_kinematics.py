
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf import transformations
import numpy as np
import math

#initialization of global parmetres of the robot
vel=0
yaw=0
max_vel_x=4
max_vel_y=4
max_angular_vel_z=4
base_diag=0.31112698372*2
wheel_radius=0.127



def vel_callback(c_vel):
	"""
	takes the twist msg return nothing
	"""
	global vel 
	vel=c_vel
	
def clbk_odom(msg):
	"""
	takes the odom msg returns nothing
	computes the orientation of the robot
	"""
	global yaw
	quaternion = (
    	msg.pose.pose.orientation.x,                                     #inorder to extract the orinetation info from the robot we need first to extract the quaternion
    	msg.pose.pose.orientation.y,					#then apply an euler transformation that result a list of 3 parameters  roll pitch and yaw									
    	msg.pose.pose.orientation.z,
    	msg.pose.pose.orientation.w)
	euler = transformations.euler_from_quaternion(quaternion)
	yaw= euler[2]
	#so pitch = euler[0] and roll = euler[1]
	
def main():
	global x_dot
	global y_dot
	global theta_dot
	global vel,yaw

	print("####################################################")
	print("-----------INVERSE KINEMATICS INITIALIZED----------")
	print("####################################################")
	#velocities initialization
	x_dot = 1
	y_dot = 1
	theta_dot = 0
	vel=Twist()
	#float angular velocities for each wheel 
	omega1 = Float64()
	omega2 = Float64()
	omega3 = Float64()
	omega4 = Float64()

	sub=rospy.Subscriber("/cmd_vel",Twist,vel_callback)
	# all this publishers are generated using a yaml controller file situated in final_control/config  
	pub1=rospy.Publisher("/prometheus_assembly/rf_joint_velocity_controller/command",Float64,queue_size=10) 
	pub2=rospy.Publisher("/prometheus_assembly/lf_joint_velocity_controller/command",Float64,queue_size=10)
	pub3=rospy.Publisher("/prometheus_assembly/lb_joint_velocity_controller/command",Float64,queue_size=10)
	pub4=rospy.Publisher("/prometheus_assembly/rb_joint_velocity_controller/command",Float64,queue_size=10)
	

	while not rospy.is_shutdown():

		rospy.init_node("inverse_kinematics",anonymous=False)
		rate = rospy.Rate(10)
		#velocities normalization so it will not surpass a certain lvl
		if (vel.linear.x < -max_vel_x):
			x_dot = -max_vel_x
		elif  (abs(vel.linear.x) < max_vel_x):
			x_dot = vel.linear.x
		else:
			x_dot = max_vel_x

		if (vel.linear.y < -max_vel_y):
			y_dot = -max_vel_y
		elif (abs(vel.linear.y) < max_vel_y):
			y_dot = vel.linear.y
		else:
			y_dot = max_vel_y
		if (vel.angular.z > max_angular_vel_z):
			theta_dot = max_angular_vel_z
			
		elif (vel.angular.z < -max_angular_vel_z):
			theta_dot = -max_angular_vel_z
			
		else:
			theta_dot = vel.angular.z

		#all the equations bellow are results of inverse kinematics mathematical computation , for deeper info check the readme file	
		omega1.data = (-np.sin(yaw+1.75*math.pi)*x_dot + np.cos(yaw+1.75*math.pi)*y_dot + base_diag*theta_dot)/wheel_radius
		omega2.data = (-np.sin(yaw+0.25*math.pi)*x_dot + np.cos(yaw+0.25*math.pi)*y_dot + base_diag*theta_dot)/wheel_radius
		omega3.data = (-np.sin(yaw+1.25*math.pi)*x_dot + np.cos(yaw+1.25*math.pi)*y_dot + base_diag*theta_dot)/wheel_radius
		omega4.data = (-np.sin(yaw+0.75*math.pi)*x_dot + np.cos(yaw+0.75*math.pi)*y_dot + base_diag*theta_dot)/wheel_radius
		
#publishing the angular velocity to each wheel (conversion from robot motion to wheel velocity)
		pub1.publish(omega1)                                          
		pub2.publish(omega2)                                          
		pub3.publish(omega3)                                         
		pub4.publish(omega4)
		rate.sleep()


if __name__ == '__main__':
	main()



