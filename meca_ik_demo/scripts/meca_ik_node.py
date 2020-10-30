#!/usr/bin/env python2

''' 
This ROS node solves the inverse kinematics of the Mecademic Meca500 r3. 
It listens to the /desired_pose topic, solves the ik, then publishes to the /joint_pos_desired topic

Author: Garrison Johnston
'''

# Imports
import rospy
import PyKDL as kdl
import kdl_parser_py.urdf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

class MecaIkNode:

	## Initializes the member variables
	def __init__(self):

		'''
		Members needed to solve the Inverse kinematics of the meca500.
		'''
		# Load the urdf from the parameter server and get a pyKDL tree.
		(self.parse_success,self.tree) = kdl_parser_py.urdf.treeFromParam('/robot_description')

		# If the parsing was successfull, initialize the solvers 
		if self.parse_success:

			# PyKDL kinematic chain object. The inputs are the base and ee link names in the URDF.
			self.chain = self.tree.getChain("meca_base_link", "meca_axis_6_link")

			# PyKDL inverse kinematics solver for position using Levenberg-Marquardt
			self.ik_solver_position = kdl.ChainIkSolverPos_LMA(self.chain)

			# Array for storing joint angle solutions
			self.joint_positions = kdl.JntArray(self.chain.getNrOfJoints())

			# Array for initial condition joint angles
			self.init_joint = kdl.JntArray(self.chain.getNrOfJoints())

			# Set initial conditions joint angles at zero
			for i in range(self.init_joint.rows()):
				self.init_joint[i] = 0 

			'''
			Members needed to Publish and subscribe to rostopics.
			'''
			# ROS publisher object for /joint_pos_desired rostopic
			self.joint_pos_pub = rospy.Publisher('/joint_pos_desired', JointState, queue_size=10)

			# create joint state ros message to send to RVIZ
			self.joint_msg = JointState()

			# Set names of the joints
			self.joint_msg.name = ['meca_axis_1_joint', 'meca_axis_2_joint', 'meca_axis_3_joint', 'meca_axis_4_joint', 'meca_axis_5_joint', 'meca_axis_6_joint']

			# Set positions to zero 
			self.joint_msg.position = [0]*6

			# ROS subscriber object for /desired_pose rostopic
			self.desired_pose_sub = rospy.Subscriber('/desired_pose', Pose, self._desired_pose_callback)

		# If the parsing was NOT successfull, print out a warning message
		else:
			rospy.logwarn("PARSING URDF FAILED")

	## Callback for the /desired_pose rostopic. Gets called automatically when there is an update on the topic.
	def _desired_pose_callback(self, msg):

		# Translate desired position from rostopic into a PyKDL Vector object
		vec = kdl.Vector(msg.position.x, msg.position.y, msg.position.z)

		# Translate desired orientation (quaternion) from rostopic into a PyKDL Rotation object
		quat = kdl.Rotation.Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)

		# Combine the desired translation and rotation into a pyKDL frame object
		frame = kdl.Frame(quat, vec)

		# Solve IK
		self.ik_solver_position.CartToJnt(self.init_joint, frame, self.joint_positions)

		# Copy positions from pyKDL joint array object to the ros message object
		for i in range(self.joint_positions.rows()):
			self.joint_msg.position[i] = self.joint_positions[i] 

		# Publish joint angles
		self.joint_pos_pub.publish(self.joint_msg)

if __name__=='__main__':

	# Initialize node
	rospy.init_node('meca_ik_node', anonymous = True)

	# Create instance of the class
	meca_node = MecaIkNode()

	# This line loops indefinitely and updates rostopics
	rospy.spin()