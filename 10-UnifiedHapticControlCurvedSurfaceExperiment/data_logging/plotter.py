#!/usr/bin/env python3

# read text files and plot them

import matplotlib.pyplot as plt
import numpy as np
import sys
import math

# data file to read given as argument
if len(sys.argv) < 2:
	print("Give the name of the file to read as an argument\n")
	exit()

file = np.loadtxt(sys.argv[1] ,skiprows=1)

time = file[1::, 0]

robot_q = file[1::, 1:8]
robot_dq = file[1::, 8::15]
robot_command_torques = file[1::, 15:22]
robot_desired_position = file[1::, 22:25]
robot_desired_orientation = file[1::, 25:34]
robot_desired_force = file[1::, 34:37]
robot_desired_moment = file[1::, 37:40]
robot_current_position = file[1::, 40:43]
robot_current_velocity = file[1::, 43:46]
robot_current_orientation = file[1::, 46:55]
robot_current_angular_velocity = file[1::, 55:58]
robot_task_force = file[1::, 58:64]

haptic_position = file[1::, 64:67]
haptic_velocity = file[1::, 67:70]
haptic_orientation = file[1::, 70:79]
haptic_angular_velocity = file[1::, 79:82]
haptic_command_force = file[1::, 82:85]
haptic_command_torques = file[1::, 85:88]
haptic_command_force_plus_passivity = file[1::, 88:91]
haptic_command_torque_plus_passivity = file[1::, 91:94]

R_robot_sensor = file[1::, 94:103]
R_haptic_robot = file[1::, 103:112]

sensed_force_robot_frame = file[1::, 112:115]
sensed_moment_robot_frame = file[1::, 115:118]

bilateral_passivity_alpha_force = file[1::,118]
bilateral_passivity_alpha_moment = file[1::,119]
passivity_Rc_force = file[1::,120]
passivity_Rc_moment = file[1::,121]

f_virtual = file[1::,122:125]
m_virtual = file[1::,125:128]

pos_error_z = file[1::,128]
angular_error_orient = file[1::,129:132]

angular_error_orient_norm = np.sqrt(np.sum(angular_error_orient**2,axis=1))


time -= time[0]

plt.figure(1)
plt.plot(sensed_force_robot_frame)
plt.plot(-haptic_command_force, '--')
plt.title("sensed force and haptic force")

plt.figure(2)
plt.plot(sensed_moment_robot_frame)
plt.plot(-haptic_command_torques)
plt.title("sensed moment and haptic moment")

# plt.figure(3)
# plt.plot(angular_error_orient)
# plt.title("normal error")

plt.figure(4)
plt.plot(angular_error_orient_norm)
plt.title("normal error norm square")

plt.figure(10)
plt.plot(robot_current_position[:,1],robot_current_position[:,2])
# plt.plot(robot_current_position[:,2])
plt.title("trajectory")

# plt.figure(2)
# plt.plot(robot_current_position)
# plt.plot(robot_desired_position, '--')
# plt.title("robot desired position and current position")

# plt.figure(10)
# plt.plot(haptic_position)
# plt.title("haptic position")

# plt.figure(11)
# plt.plot(haptic_orientation)
# plt.title("haptic orientation")

# plt.figure(3)
# plt.plot(bilateral_passivity_alpha_force)
# plt.title("passivity_bilateral_damping")

# plt.figure(4)
# plt.plot(passivity_Rc_force)
# plt.title("passivity Rc")


plt.show()


