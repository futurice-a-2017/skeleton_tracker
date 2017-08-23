#!/usr/bin/env python
import rospy
import roslib
import tf
import time
import threading
import numpy as np
import math
import config
import sys
from pyrr import quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from userListenerThread import userListenerThread
from collections import deque

# Main program function
def main():
	### Initialize
	# Rolling average filter
	filter_length = 0
	filter_max_length = 40

	# ROS
	rospy.init_node('skeleton_tracker')
	tfListener = tf.TransformListener()

	# Thread for listening kinect users
	userListener = userListenerThread()
	userListener.start()

	# Publish TF between kinect TF and robot TF
	publishTF()

	# Create FIFO queues
	config.angles_right_elbow = deque([])
	config.angles_left_elbow = deque([])
	config.angles_right_shoulder_parallel = deque([])
	config.angles_right_shoulder_perpendicular = deque([])
	config.angles_left_shoulder_parallel = deque([])
	config.angles_left_shoulder_perpendicular = deque([])
	config.angles_right_bicep = deque([])
	config.angles_left_bicep = deque([])
	config.angles_head = deque([])
	config.angles_waist = deque([])

	# Create strings for each TF topic
	tf_strings = []

	tf_strings.append('/brix_2/user_{}/right_hip'.format(config.user)) 		# 0
	tf_strings.append('/brix_2/user_{}/right_shoulder'.format(config.user)) # 1
	tf_strings.append('/brix_2/user_{}/right_elbow'.format(config.user))	# 2
	tf_strings.append('/brix_2/user_{}/right_hand'.format(config.user))		# 3

	tf_strings.append('/brix_2/user_{}/left_hip'.format(config.user))		# 4
	tf_strings.append('/brix_2/user_{}/left_shoulder'.format(config.user))	# 5
	tf_strings.append('/brix_2/user_{}/left_elbow'.format(config.user))		# 6
	tf_strings.append('/brix_2/user_{}/left_hand'.format(config.user))		# 7

	tf_strings.append('/brix_2/user_{}/neck'.format(config.user))			# 8
	tf_strings.append('/brix_2/user_{}/head'.format(config.user))			# 9

	# Create joint connection chains
	connections = [	[0, 8, 1, 2, 3, 8, 9],
					[4, 8, 5, 6, 7]]


	while(True):
		transform = 0
		rotation = 0
		transforms = []
		rotations = []

		# Get fresh transforms
		for i in range(len(connections)): 				# Iterate over chains of connections
			chain_transforms = []
			chain_rotations = []
			for j in range(len(connections[i]) - 1):	# Iterate over a chain

				tfListener.waitForTransform(tf_strings[connections[i][j]], tf_strings[connections[i][j+1]], rospy.Time(), rospy.Duration(30))
				(transform, rotation) = tfListener.lookupTransform(tf_strings[connections[i][j]], tf_strings[connections[i][j+1]], rospy.Time(0))
				chain_transforms.append(transform)
				chain_rotations.append(rotation)

			transforms.append(chain_transforms)
			rotations.append(chain_rotations)

		# Compute joint angles
		angles = compute_angles(transforms, rotations)

		# Filter angle value
		if filter_length < filter_max_length:
			angles = filter_angles(angles, 0)
			filter_length += 1
		else:
			angles = filter_angles(angles, 1)

		# Publish as ROS message
		publish_angles()

		# Publish a transformation connecting kinect TF to robot TF eliminating separate TF trees
		publishTF()


# Compute needed angles, separate calculations for each type to match InMoov structure
def compute_angles(transforms, rotations):
	angles = []

	#########################################
	### Elbows - simple angle calculation ###
	#########################################
	elbow_angles = []
	for i in range(2):			# Iterate over joint chains - get right and left

		# Create vectors from transforms
		vector_shoulder_elbow = np.array([	transforms[i][2][0],
											transforms[i][2][1],
											transforms[i][2][2]])
		vector_elbow_wrist = np.array([	transforms[i][3][0],
										transforms[i][3][1],
										transforms[i][3][2]])

		# Rotate vector_elbow_wrist to match vector_shoulder_elbow coordinate system
		inverse_quaternion = quaternion.inverse(rotations[i][2])
		vector_elbow_wrist = quaternion.apply_to_vector(inverse_quaternion, vector_elbow_wrist)

		# Angle between 2 vectors
		angle = np.arccos(np.dot(vector_shoulder_elbow, vector_elbow_wrist) / (np.linalg.norm(vector_shoulder_elbow) * np.linalg.norm(vector_elbow_wrist)))

		# Limit angle
		if angle > 3 * math.pi / 4:
			angle = 3 * math.pi / 4
		elif angle < 0:
			angle = 0

		elbow_angles.append(angle)

	###########################################################################################
	### Shoulders, 2 angles per shoulder - projecting upper arm on two perpendicular planes ###
	###########################################################################################
	shoulder_angles = []
	for i in range(2):			# Iterate over joint chains - get right and left

		# Create vectors from transformations
		vector_hip_neck = np.array([transforms[i][0][0],
									transforms[i][0][1],
									transforms[i][0][2]])
		vector_neck_shoulder = np.array([	transforms[i][1][0],
											transforms[i][1][1],
											transforms[i][1][2]])
		vector_shoulder_elbow = np.array([	transforms[i][2][0],
											transforms[i][2][1],
											transforms[i][2][2]])

		# Rotate vector_shoulder_elbow to match vector_neck_shoulder coordinate system
		inverse_quaternion = quaternion.inverse(rotations[i][1])
		vector_shoulder_elbow = quaternion.apply_to_vector(inverse_quaternion, vector_shoulder_elbow)

		# Rotate vector_hip_neck to match vector_neck_shoulder coordinate system
		vector_hip_neck = quaternion.apply_to_vector(rotations[i][0], vector_hip_neck)

		# Calculate normal vector for plane set by vector_hip_neck and vector_neck_shoulder
		vector_normal_forward = np.cross(vector_hip_neck, vector_neck_shoulder)

		# Initialize second normal vector
		vector_normal_side = vector_neck_shoulder

		# Project vector_shoulder_elbow to plane set by vector_normal_forward
		vector_shoulder_elbow_parallel = vector_shoulder_elbow - ((np.dot(vector_shoulder_elbow, vector_normal_forward) / math.pow(np.linalg.norm(vector_normal_forward), 2)) * vector_normal_forward)

		# Project vector_shoulder_elbow to plane set by vector_normal_side
		vector_shoulder_elbow_perpendicular = vector_shoulder_elbow - ((np.dot(vector_shoulder_elbow, vector_normal_side) / math.pow(np.linalg.norm(vector_normal_side), 2)) * vector_normal_side)
		if i == 1:
			vector_shoulder_elbow_perpendicular = -vector_shoulder_elbow_perpendicular

		# Angles between 2 vectors
		angle_parallel = np.arccos(np.dot(vector_normal_side, vector_shoulder_elbow_parallel) / (np.linalg.norm(vector_normal_side) * np.linalg.norm(vector_shoulder_elbow_parallel)))
		angle_perpendicular = np.arccos(np.dot(vector_normal_forward, vector_shoulder_elbow_perpendicular) / (np.linalg.norm(vector_normal_forward) * np.linalg.norm(vector_shoulder_elbow_perpendicular)))

		# Fix ambiquity of results below and above zero-level
		if vector_shoulder_elbow[1] > 0:
			angle_parallel += math.pi / 2
			angle_perpendicular = math.fabs(angle_perpendicular + (math.pi / 2))
		# Set angles to match the simulation
		else:
			angle_parallel = math.fabs(angle_parallel - (math.pi / 2))
			angle_perpendicular = math.fabs(angle_perpendicular - (math.pi / 2))

		# In simulation, right goes down from zero while left goes up from zero
		if i == 0:
			angle_parallel = -angle_parallel

		# Set angle_parallel to zero if hand is above parallel
		if vector_shoulder_elbow[1] > 0:
			if vector_shoulder_elbow[0] > 0 and i == 0:
				angle_parallel = 0
			elif vector_shoulder_elbow[0] < 0 and i == 1:
				angle_parallel = 0

		# Limit parallel angle
		if angle_parallel > 0.75:
			angle_parallel = 0.75
		elif angle_parallel < -0.75:
			angle_parallel = -0.75

		# Limit perpendicular angle
		if angle_perpendicular > math.pi + math.pi / 4:
			angle_perpendicular = 0
		elif angle_perpendicular < -math.pi / 4:
			angle_perpendicular = 0
		elif angle_perpendicular > math.pi:
			angle_perpendicular = math.pi
		elif angle_perpendicular < 0:
			angle_perpendicular = 0

		shoulder_angles.append(angle_parallel)
		shoulder_angles.append(angle_perpendicular)

	######################
	### Bicep rotation ###
	######################
	bicep_angles = []
	for i in range(2):			# Iterate over joint chains - get right and left

		# Create vectors from transformations
		vector_shoulder_elbow = np.array([	transforms[i][2][0],
											transforms[i][2][1],
											transforms[i][2][2]])

		vector_elbow_hand = np.array([	transforms[i][3][0],
										transforms[i][3][1],
										transforms[i][3][2]])

		# Rotate vector_shoulder_elbow to match vector_neck_shoulder coordinate system
		inverse_quaternion = quaternion.inverse(rotations[i][1])
		vector_shoulder_elbow = quaternion.apply_to_vector(inverse_quaternion, vector_shoulder_elbow)

		# Calculate a unit vector from vector_elbow_hand
		vector_elbow_hand_lenght = np.linalg.norm(vector_elbow_hand)
		vector_elbow_hand_unit = vector_elbow_hand / vector_elbow_hand_lenght

		# Calculate bicep rotation angle
		angle = 2 * math.sin(vector_elbow_hand_unit[1])

		if vector_shoulder_elbow[1] > 0 and i == 1:
			angle =  angle - math.pi
		elif vector_shoulder_elbow[1] > 0 and i == 0:
			angle =  -angle + math.pi
		elif vector_shoulder_elbow[1] < 0 and i == 0:
			angle = -angle

		bicep_angles.append(angle)

	#################
	### Head tilt ###
	#################
	head_angles = []
	# Create vectors from transformations
	vector_neck_head = np.array([	transforms[0][5][0],
									transforms[0][5][1],
									transforms[0][5][2]])

	# Calculate a unit vector from vector_neck_head
	vector_neck_head_lenght = np.linalg.norm(vector_neck_head)
	vector_neck_head_unit = vector_neck_head / vector_neck_head_lenght

	# Calculate head rotation angle
	angle = math.sin(vector_neck_head_unit[0])

	# Limit angle
	if angle > 0.3:
		angle = 0.3
	elif angle < -0.3:
		angle = -0.3

	head_angles.append(angle)

	##################
	### Waist lean ###
	##################s
	waist_angles = []
	# Create vector up from shoulder line
	vector_normal_up = np.cross(vector_normal_side, vector_normal_forward)

	# Calculate a unit vector from vector_normal_up
	vector_normal_up_lenght = np.linalg.norm(vector_normal_up)
	vector_normal_up_unit = vector_normal_up / vector_normal_up_lenght

	# Calculate waist lean angle
	angle = math.sin(vector_neck_head_unit[0])

	# Limit angle
	if angle > 0.3:
		angle = 0.3
	elif angle < -0.3:
		angle = -0.3

	waist_angles.append(angle)


	# Test print
	outputStr = '{}\n'.format(vector_normal_up)		
	rospy.loginfo(outputStr)


	angles.append(elbow_angles)
	angles.append(shoulder_angles)
	angles.append(bicep_angles)
	angles.append(head_angles)
	angles.append(waist_angles)

	return angles

# Publish angles as ROS JointState message
def publish_angles():
	pub = rospy.Publisher('joint_states', JointState, queue_size=10)
	# Create new joint state message
	new_states = JointState()
	# Set header and timestamp
	new_states.header = Header()
	new_states.header.stamp = rospy.Time.now()
	# Set joint names
	new_states.name = [ 'waist_rotate', 'waist_lean', 'head_tilt', 'head_updown', 
						'head_leftright', 'jaw', 'eyes_updown', 'eye_leftright', 
						'left_eye_leftright', 'right_shoulder_up', 'right_bicep_rotate', 'right_bicep', 
						'right_shoulder_side', 'right_thumb1', 'right_thumb', 'right_thumb3', 
						'right_index1', 'right_index', 'right_index3', 'right_middle1', 
						'right_middle', 'right_middle3', 'right_ring1', 'right_ring', 
						'right_ring3', 'right_ring4', 'right_pinky1', 'right_pinky', 
						'right_pinky3', 'right_pinky4', 'right_hand', 'left_shoulder_up', 
						'left_bicep_rotate', 'left_bicep', 'left_shoulder_side', 'left_thumb1', 
						'left_thumb', 'left_thumb3', 'left_index1', 'left_index', 
						'left_index3', 'left_middle1', 'left_middle', 'left_middle3', 
						'left_ring1', 'left_ring', 'left_ring3', 'left_ring4', 
						'left_pinky1', 'left_pinky', 'left_pinky3', 'left_pinky4', 
						'left_hand']

	# Initialize joint positions
	new_states.position = np.zeros(53)

	# Set joint positions
	new_states.position[11] = config.angle_right_elbow
	new_states.position[33] = config.angle_left_elbow
	new_states.position[12] = config.angle_right_shoulder_parallel
	new_states.position[9] = config.angle_right_shoulder_perpendicular
	new_states.position[34] = config.angle_left_shoulder_parallel
	new_states.position[31] = config.angle_left_shoulder_perpendicular
	new_states.position[10] = config.angle_right_bicep
	new_states.position[32] = config.angle_left_bicep
	new_states.position[2] = config.angle_head
	new_states.position[1] = config.angle_waist

	# Set velocity and effort
	new_states.velocity = []
	new_states.effort = []

	# Publish the message
	pub.publish(new_states)

# Publish TF between kinect TF and robot TF
def publishTF():
	br = tf.TransformBroadcaster()
	br.sendTransform(	(10, 10, 0),
						tf.transformations.quaternion_from_euler(0, 0, 0),
						rospy.Time.now(),
						'global_space',
						'base_link')

# Rolling average filter
def filter_angles(angles, fifo_status):

	if fifo_status == 0:  	# If FIFO queue is not at maximum size: append new angle
		config.angles_right_elbow.append(angles[0][0])

		config.angles_left_elbow.append(angles[0][1])

		config.angles_right_shoulder_parallel.append(angles[1][0])
		config.angles_right_shoulder_perpendicular.append(angles[1][1])

		config.angles_left_shoulder_parallel.append(angles[1][2])
		config.angles_left_shoulder_perpendicular.append(angles[1][3])

		config.angles_right_bicep.append(angles[2][0])
		config.angles_left_bicep.append(angles[2][1])

		config.angles_head.append(angles[3][0])

		config.angles_waist.append(angles[4][0])

	else:					# If FIFO queue is at maximum size: pop oldest angle and append new angle
		config.angles_right_elbow.popleft()
		config.angles_right_elbow.append(angles[0][0])

		config.angles_left_elbow.popleft()
		config.angles_left_elbow.append(angles[0][1])

		config.angles_right_shoulder_parallel.popleft()
		config.angles_right_shoulder_perpendicular.popleft()
		config.angles_right_shoulder_parallel.append(angles[1][0])
		config.angles_right_shoulder_perpendicular.append(angles[1][1])

		config.angles_left_shoulder_parallel.popleft()
		config.angles_left_shoulder_perpendicular.popleft()
		config.angles_left_shoulder_parallel.append(angles[1][2])
		config.angles_left_shoulder_perpendicular.append(angles[1][3])

		config.angles_right_bicep.popleft()
		config.angles_right_bicep.append(angles[2][0])

		config.angles_left_bicep.popleft()
		config.angles_left_bicep.append(angles[2][1])

		config.angles_head.popleft()
		config.angles_head.append(angles[3][0])

		config.angles_waist.popleft()
		config.angles_waist.append(angles[4][0])
	
	# Calculate averages
	config.angle_right_elbow = np.mean(config.angles_right_elbow)
	config.angle_left_elbow = np.mean(config.angles_left_elbow)
	config.angle_right_shoulder_parallel = np.mean(config.angles_right_shoulder_parallel)
	config.angle_right_shoulder_perpendicular = np.mean(config.angles_right_shoulder_perpendicular)
	config.angle_left_shoulder_parallel = np.mean(config.angles_left_shoulder_parallel)
	config.angle_left_shoulder_perpendicular = np.mean(config.angles_left_shoulder_perpendicular)
	config.angle_right_bicep = np.mean(config.angles_right_bicep)
	config.angle_left_bicep = np.mean(config.angles_left_bicep)
	config.angle_head = np.mean(config.angles_head)
	config.angle_waist = np.mean(config.angles_waist)

if __name__ == '__main__':
	main()
