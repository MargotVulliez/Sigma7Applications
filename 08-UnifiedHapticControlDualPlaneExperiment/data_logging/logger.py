#!/usr/bin/env python3

# read redis keys and dump them to a file
import redis, time, signal, sys
import numpy as np
import os
import json

runloop = True
counter = 0

# handle ctrl-C and close the files
def signal_handler(signal, frame):
    global runloop
    runloop = False
    print(' ... Exiting data logger')

signal.signal(signal.SIGINT, signal_handler)

# data files
folder = 'simulation/'

if not os.path.exists(folder):
    os.makedirs(folder)

# date and time
timestamp = time.strftime("%x").replace('/','-') + '_' + time.strftime("%X").replace(':','-')

# file names
# name = "impedance_controller"
name = "unified_controller"

# open files
file = open(folder + '/' + name + '_' + timestamp,'w')

file.write('time\trobot_q[7]\trobot_dq[7]\trobot_command_torques[7]\tdesired_robot_position[3]\tdesired_robot_orientation[3][3]' +\
  'desired_robot_force[3]\tdesired_robot_moment[3]\tcurrent_robot_position[3]\tcurrent_robot_velocity\t' +\
  'current_robot_orientation[3][3]\tcurrent_robot_angular_velocity[3]\trobot_task_force[6]\t' +\
  'haptic_position[3]\thaptic_velocity[3]\thaptic_orientation[3][3]\thaptic_angular_velocity[3]\t' +\
  'haptic_command_force[3]\thaptic_command_torque[3]\thaptic_command_force_plus_passivity[3]\thaptic_command_torque_plus_passivity[3]\t' +\
  'R_robot_fsensor[3][3]\tR_haptic_robot[3][3]\tsensed_force_robot_frame[3]\tsensed_moment_robot_frame[3]\n')

# open redis server
r_server = redis.StrictRedis(host='localhost', port=6379, db=0)
pipe = r_server.pipeline()

# redis keys used in SAI2
LOGGING_TIME_KEY = "sai2::Sigma7Applications::logging::time";

LOGGING_ROBOT_JOINT_ANGLES = "sai2::Sigma7Applications::logging::q";
LOGGING_ROBOT_JOINT_VELOCITIES = "sai2::Sigma7Applications::logging::dq";
LOGGING_ROBOT_COMMAND_TORQUES = "sai2::Sigma7Applications::logging::fgc";
LOGGING_ROBOT_DESIRED_POSITION = "sai2::Sigma7Applications::logging::x_desired";
LOGGING_ROBOT_DESIRED_ORIENTATION = "sai2::Sigma7Applications::logging::R_desired";
LOGGING_ROBOT_DESIRED_FORCE = "sai2::Sigma7Applications::logging::f_desired";
LOGGING_ROBOT_DESIRED_MOMENT = "sai2::Sigma7Applications::logging::m_desired";
LOGGING_ROBOT_CURRENT_POSITION = "sai2::Sigma7Applications::logging::x_current";
LOGGING_ROBOT_CURRENT_VELOCITY = "sai2::Sigma7Applications::logging::dx_current";
LOGGING_ROBOT_CURRENT_ORIENTATION = "sai2::Sigma7Applications::logging::R_current";
LOGGING_ROBOT_CURRENT_ANGVEL = "sai2::Sigma7Applications::logging::w_current";
LOGGING_ROBOT_TASK_FORCE = "sai2::Sigma7Applications::logging::posori_task_force";

LOGGING_HAPTIC_POSITION = "sai2:Sigma7Applications::logging::haptic_position";
LOGGING_HAPTIC_VELOCITY = "sai2:Sigma7Applications::logging::haptic_velocity";
LOGGING_HAPTIC_ORIENTATION = "sai2:Sigma7Applications::logging::haptic_orientation";
LOGGING_HAPTIC_ANGVEL = "sai2:Sigma7Applications::logging::haptic_angular_velocity";
LOGGING_HAPTIC_COMMAND_FORCE = "sai2:Sigma7Applications::logging::haptic_command_force";
LOGGING_HAPTIC_COMMAND_TORQUE = "sai2:Sigma7Applications::logging::haptic_command_torque";
LOGGING_HAPTIC_COMMAND_FORCE_TOTAL = "sai2:Sigma7Applications::logging::haptic_command_force_plus_passivity";
LOGGING_HAPTIC_COMMAND_TORQUE_TOTAL = "sai2:Sigma7Applications::logging::haptic_command_torque_plus_passivity";

LOGGING_R_ROBOT_SENSOR = "sai2::Sigma7Applications::logging::R_robot_sensor";
LOGGING_R_HAPTIC_ROBOT = "sai2::Sigma7Applications::logging::R_robot_haptic";

LOGGING_SENSED_FORCE_ROBOT_FRAME = "sai2::Sigma7Applications::logging::force_sensed_robot_frame";
LOGGING_SENSED_MOMENT_ROBOT_FRAME = "sai2::Sigma7Applications::logging::moment_sensed_robot_frame";

# data logging frequency
logger_frequency = 100.0  # Hz
logger_period = 1.0/logger_frequency
t_init = time.time()
t = t_init

print('Start Logging Data ... \n')

while(runloop):
    t += logger_period

    pipe.get(LOGGING_TIME_KEY)

    pipe.get(LOGGING_ROBOT_JOINT_ANGLES)
    pipe.get(LOGGING_ROBOT_JOINT_VELOCITIES)
    pipe.get(LOGGING_ROBOT_COMMAND_TORQUES)
    pipe.get(LOGGING_ROBOT_DESIRED_POSITION)
    pipe.get(LOGGING_ROBOT_DESIRED_ORIENTATION)
    pipe.get(LOGGING_ROBOT_DESIRED_FORCE)
    pipe.get(LOGGING_ROBOT_DESIRED_MOMENT)
    pipe.get(LOGGING_ROBOT_CURRENT_POSITION)
    pipe.get(LOGGING_ROBOT_CURRENT_VELOCITY)
    pipe.get(LOGGING_ROBOT_CURRENT_ORIENTATION)
    pipe.get(LOGGING_ROBOT_CURRENT_ANGVEL)
    pipe.get(LOGGING_ROBOT_TASK_FORCE)

    pipe.get(LOGGING_HAPTIC_POSITION)
    pipe.get(LOGGING_HAPTIC_VELOCITY)
    pipe.get(LOGGING_HAPTIC_ORIENTATION)
    pipe.get(LOGGING_HAPTIC_ANGVEL)
    pipe.get(LOGGING_HAPTIC_COMMAND_FORCE)
    pipe.get(LOGGING_HAPTIC_COMMAND_TORQUE)
    pipe.get(LOGGING_HAPTIC_COMMAND_FORCE_TOTAL)
    pipe.get(LOGGING_HAPTIC_COMMAND_TORQUE_TOTAL)

    pipe.get(LOGGING_R_ROBOT_SENSOR)
    pipe.get(LOGGING_R_HAPTIC_ROBOT)

    pipe.get(LOGGING_SENSED_FORCE_ROBOT_FRAME)
    pipe.get(LOGGING_SENSED_MOMENT_ROBOT_FRAME)

    responses = pipe.execute()

    line = "";

    for response in responses:
        r_array = np.array(json.loads(response.decode("utf-8")))

        if(len(np.shape(r_array)) == 0):
            line += str(r_array) + '\t'
        elif(len(np.shape(r_array)) == 1):
            line += " ".join([str(x) for x in r_array]) + '\t'
        elif(len(np.shape(r_array)) == 2):
            r_array_row = np.resize(r_array, (1, np.shape(r_array)[0]*np.shape(r_array)[1]))[0]
            line += " ".join([str(x) for x in r_array_row]) + '\t'
    line += '\n'

    file.write(line)

    counter = counter + 1

    time.sleep(max(0.0,t-time.time()))

elapsed_time = time.time() - t_init
print("Elapsed time : ", elapsed_time, " seconds")
print("Loop cycles  : ", counter)
print("Frequency    : ", counter/elapsed_time, " Hz")

file.close()
