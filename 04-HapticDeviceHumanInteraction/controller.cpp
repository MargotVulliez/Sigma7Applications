// This example tests the haptic device driver and the open-loop bilateral teleoperation controller.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "chai3d.h" //To manage haptic device control, inputs, outputs

// Include task primitives from 'sai2-primitives'
#include "tasks/JointTask.h"
#include "tasks/PosOriTask.h"
#include "haptic_tasks/OpenLoopTeleop.h"

#include <iostream>
#include <string>
#include <sstream>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;
using namespace chai3d;

//// Define robots ////
 const string robot_file = "../resources/04-HapticDeviceHumanInteraction/kuka_iiwa.urdf";
 const string robot_name = "Kuka-IIWA";

//////////////////////////////////////////////////////////////////////
// //Definition of the state machine for the robot controller
#define GOTO_INITIAL_CONFIG               0
#define HAPTIC_CONTROL                    1
#define MAINTAIN_POSITION				  2


#define DEBUG	6//
// int state_dev = DEBUG;

int state = GOTO_INITIAL_CONFIG;

// Logger/Plotter keys
const string DEVICE_FORCE_KEY = "sai2::Sigma7Applications::commands::device_force";
const string DEVICE_TORQUE_KEY = "sai2::Sigma7Applications::commands::device_torque";
const string DRIFT_FORCE_KEY = "sai2::Sigma7Applications::commands::drift_force";
const string DRIFT_TORQUE_KEY = "sai2::Sigma7Applications::commands::drift_torque";
const string DRIFT_VEL_TRANS_KEY = "sai2::Sigma7Applications::commands::drift_trans_vel";
const string DRIFT_VEL_ROT_KEY = "sai2::Sigma7Applications::commands::drift_rot_vel";
const string SIM_TIMESTAMP_KEY = "sai2::Sigma7Applications::simulation::timestamp";
//...
//...






unsigned long long controller_counter = 0;

const bool inertia_regularization = true;

int remote_enabled = 1;
int restart_cycle = 0;

// Create redis keys
const string REMOTE_ENABLED_KEY = "sai2::Sigma7Applications::sensors::remote_enabled";
const string RESTART_CYCLE_KEY = "sai2::Sigma7Applications::sensors::restart_cycle";

const string JOINT_ANGLES_KEY  = "sai2::Sigma7Applications::sensors::q";
const string JOINT_VELOCITIES_KEY = "sai2::Sigma7Applications::sensors::dq";
const string FORCE_SENSED_KEY = "sai2::Sigma7Applications::sensors::force_task_sensed";
const string JOINT_TORQUES_COMMANDED_KEY  = "sai2::Sigma7Applications::actuators::torque_joint_robot";



//// Declaration of internal functions for Chai-Eigen transformations////
cVector3d convertEigenToChaiVector( Eigen::Vector3d a_vec )
{
    double x = a_vec(0);
    double y = a_vec(1);
    double z = a_vec(2);
    return cVector3d(x,y,z);
}

Eigen::Vector3d convertChaiToEigenVector( cVector3d a_vec )
{
    double x = a_vec.x();
    double y = a_vec.y();
    double z = a_vec.z();
    return Eigen::Vector3d(x,y,z);
}

cMatrix3d convertEigenToChaiRotation( Eigen::Matrix3d a_mat )
{
    return cMatrix3d( a_mat(0,0), a_mat(0,1), a_mat(0,2), a_mat(1,0), a_mat(1,1), a_mat(1,2), a_mat(2,0), a_mat(2,1), a_mat(2,2) );
}

Eigen::Matrix3d convertChaiToEigenMatrix( cMatrix3d a_mat )
{
    Eigen::Matrix3d asdf;
    asdf << a_mat(0,0), a_mat(0,1), a_mat(0,2),
            a_mat(1,0), a_mat(1,1), a_mat(1,2),
            a_mat(2,0), a_mat(2,1), a_mat(2,2);

    return asdf;
}

/////////////////////////////////////////////////////////////////////////////////

int main() {

	/////////////////////////////// init ////////////////////////////////////////
 	
	remote_enabled=1;

	// position of robot in world
	Affine3d robot_pose_in_world = Affine3d::Identity();
	robot_pose_in_world.translation() = Vector3d(0, 0.0, 0.0);
	robot_pose_in_world.linear() = AngleAxisd(0.0, Vector3d::UnitZ()).toRotationMatrix();

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	redis_client.set(REMOTE_ENABLED_KEY, to_string(remote_enabled));
	redis_client.set(RESTART_CYCLE_KEY, to_string(restart_cycle));

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
	robot->updateModel();

	//// Joint Task controller (manage the iiwa robot nullspace - posture) ////
	auto joint_task = new Sai2Primitives::JointTask(robot);
	MatrixXd N_prec = MatrixXd::Identity(robot->dof(), robot->dof());
	VectorXd joint_task_torques = VectorXd::Zero(robot->dof());
	joint_task->_kp = 10.0;
	joint_task->_kv = 5.0;
	VectorXd command_torques = VectorXd::Zero(robot->dof());

	// Define goal position according to the desired posture ///////////////////////////////////////////////////////////////////////////////////////////////// 
	VectorXd goal_posture(robot->dof());
	goal_posture << 0.0, -30.0, 0.0, 60.0, 0.0, -90.0, 0.0; /////////////////////////// 
	goal_posture *= M_PI/180.0;
	joint_task->_desired_position = goal_posture;

	//// PosOriTask controller (manage the control in position of the iiwa robot) ////
	const string link_name = "link6";
	const Vector3d pos_in_link = Vector3d(0.0,0.0,0.0); /////////////////////////////////Define center of tool in end-effector frame
	auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	VectorXd posori_task_torques = VectorXd::Zero(robot->dof());

	posori_task->_kp_pos = 50.0;
	posori_task->_kv_pos = 14.0;
	posori_task->_kp_ori = 50.0;
	posori_task->_kv_ori = 14.0;

	// Define position and orientation of the task center ////////////////////////////////////////////////////////////////////////////////
	Vector3d centerPos_rob;
	centerPos_rob << -0.5, 0.0, 0.6;
	Matrix3d centerRot_rob;
	//centerRot_rob.setIdentity();
	centerRot_rob << -1.0, 0.0, 0.0,
					  0.0, 1.0, 0.0,
            		  0.0, 0.0, -1.0;

	posori_task->_desired_position = centerPos_rob;
	posori_task->_desired_orientation = centerRot_rob;

	//// Open loop teleoperation haptic controller ////
	// create a handler
	auto handler = new cHapticDeviceHandler();

	Matrix3d transformDev_Rob = robot_pose_in_world.linear();
	int device_number=0;
	auto teleop_task = new Sai2Primitives::OpenLoopTeleop(handler, device_number, centerPos_rob, centerRot_rob, transformDev_Rob);
	//Task scaling factors
	double Ks=2.0;
	double KsR=1.0;
	teleop_task->setScalingFactors(Ks, KsR);
	 // Center of the haptic device workspace
	Vector3d HomePos_op;
	HomePos_op << 0.0, 0.01, 0.0;
	Matrix3d HomeRot_op;
	HomeRot_op.setIdentity();
	teleop_task->setDeviceCenter(HomePos_op, HomeRot_op);
	// Force feedback stiffness proxy parameters
	double k_pos = 200.0;
	double d_pos = 20.0;
	double k_ori = 15.0;
	double d_ori = 5.0;
	Matrix3d Red_factor_rot = Matrix3d::Identity();
	Matrix3d Red_factor_trans = Matrix3d::Identity();
	Red_factor_rot << 1/20.0, 0.0, 0.0,
						  0.0, 1/20.0, 0.0,
						  0.0, 0.0, 1/20.0;

	Red_factor_trans << 1/2.0, 0.0, 0.0,
						  0.0, 1/2.0, 0.0,
						  0.0, 0.0, 1/2.0;
	teleop_task->setForceFeedbackCtrlGains (k_pos, d_pos, k_ori, d_ori, Red_factor_rot, Red_factor_trans);

	Vector3d pos_rob = Vector3d::Zero(); // Set position and orientation in robot frame
	Matrix3d rot_rob = Matrix3d::Identity();
	Vector3d pos_rob_model = Vector3d::Zero(); // Model position and orientation in robot frame
	Matrix3d rot_rob_model = Matrix3d::Identity();
	Vector3d vel_rot_rob_model = Vector3d::Zero(); // Model linear and angular velocities in robot frame
	Vector3d vel_trans_rob_model = Vector3d::Zero();
	VectorXd f_task_sensed = VectorXd::Zero(6);
	// write task force in redis
	redis_client.setEigenMatrixJSON(FORCE_SENSED_KEY,f_task_sensed);

	teleop_task->_haptic_feedback_from_proxy = false;
	teleop_task->_filter_on = false;

	// Initialize haptic device if Sigma.7
	teleop_task->initializeSigmaDevice();

	//Enable user switch
	teleop_task->EnableGripperUserSwitch();
	bool gripper_state=false;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); //Compiler en mode release
	double current_time = 0;
	double prev_time = 0;
	double dt = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs

	/////////////////////////////// cyclic ////////////////////////////////////////
	while (runloop) {
	// wait for next scheduled loop
	timer.waitForNextLoop();
	double time = timer.elapsedTime() - start_time;
	dt = current_time - prev_time;

	//// Read robots data ////
	// read iiwa robot info from redis
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
	
	robot->updateModel();

	// read force sensor data
	f_task_sensed = redis_client.getEigenMatrixJSON(FORCE_SENSED_KEY);
	// read renabling/disabling teleoperation brush
	remote_enabled = stoi(redis_client.get(REMOTE_ENABLED_KEY));
	// read restar cycle key
	restart_cycle = stoi(redis_client.get(RESTART_CYCLE_KEY));


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//// Run Robot/Haptic Device state machine ////
	if(state == GOTO_INITIAL_CONFIG)
	{
		// update tasks model and priority 
		N_prec.setIdentity();
		posori_task->updateTaskModel(N_prec);
		N_prec = posori_task->_N;
		joint_task->updateTaskModel(N_prec);
		// compute torques
		posori_task->computeTorques(posori_task_torques);
		joint_task->computeTorques(joint_task_torques);
		command_torques = posori_task_torques + joint_task_torques;

		// compute homing haptic device
		teleop_task->HomingTask();

		// Read new gripper state
        gripper_state = teleop_task->ReadGripperUserSwitch();
        

		if( remote_enabled==1 && teleop_task->device_homed && gripper_state && (posori_task->_desired_position - posori_task->_current_position).norm() < 0.001 && posori_task->_orientation_error.norm()<0.001)
		{
			
			// Reinitialize controllers
			joint_task->reInitializeTask();
			joint_task->_desired_position = goal_posture;
			posori_task->reInitializeTask();

			HomePos_op = teleop_task->_current_position_device ;
			teleop_task->setDeviceCenter(HomePos_op, HomeRot_op);

			centerPos_rob = posori_task->_current_position;
			centerRot_rob = posori_task->_current_orientation;
			teleop_task->setRobotCenter(centerPos_rob, centerRot_rob);
		
			state = HAPTIC_CONTROL;
		}
	}

	else if(state == HAPTIC_CONTROL)
	{

		// Update position and orientation of the robot from the model
		robot->position(pos_rob_model, link_name, pos_in_link);
		robot->linearVelocity(vel_trans_rob_model, link_name, pos_in_link);
		robot->rotation(rot_rob_model, link_name);
		robot->angularVelocity(vel_rot_rob_model, link_name);
		//Compute haptic commands
		teleop_task->_haptic_feedback_from_proxy = false; // If set to true, the force feedback is computed from a stiffness/damping proxy.
	    teleop_task->_filter_on = true;
		double fc_force=0.02;
		double fc_moment=0.02;
		teleop_task->setFilterCutOffFreq(fc_force, fc_moment);

		teleop_task->updateSensedForce(f_task_sensed);
		//teleop_task->updateSensedRobotPositionVelocity(pos_rob_model, vel_trans_rob_model, rot_rob_model, vel_rot_rob_model);

		// if (f_task_sensed.norm()>=0.001)
		// {
		// 	teleop_task->_send_haptic_feedback = true;	
		// }
		// else
		// {
		// 	teleop_task->_send_haptic_feedback = false;
		// }
		teleop_task->_send_haptic_feedback = true;
		
		// Constrain haptic device motion in the (YZ) plan
		teleop_task->_haptic_guidance_plan = true;
		teleop_task->_guidancePlan_XYZ=1; //set nomal to guidance plan to be X


		teleop_task->computeHapticCommands3d(pos_rob);

		//// Impedance controller of the robot ////
		// set goal position
		posori_task->_desired_position = pos_rob;
		posori_task->_desired_orientation = centerRot_rob;
		
		// update model and priority
		N_prec.setIdentity();
		posori_task->updateTaskModel(N_prec);
		N_prec = posori_task->_N;
		joint_task->updateTaskModel(N_prec);
		// compute torques
		posori_task->computeTorques(posori_task_torques);
		joint_task->computeTorques(joint_task_torques);

		command_torques = posori_task_torques + joint_task_torques;

		if(remote_enabled == 0)
		{
			// joint controller to maintin robot in current position
			joint_task->reInitializeTask();
			joint_task->_desired_position = robot->_q;

			// set current haptic device position
			teleop_task->setDeviceCenter(teleop_task->_current_position_device, teleop_task->_current_rotation_device);


			state = MAINTAIN_POSITION;

		}
	}

	else if(state == MAINTAIN_POSITION)
	{
		//Maintain the robot in current position
		// update tasks model
		N_prec.setIdentity();
		joint_task->updateTaskModel(N_prec);
		// compute torques
		joint_task->computeTorques(joint_task_torques);
		command_torques = joint_task_torques;

		// Maintin haptic device position
		teleop_task->HomingTask();

		// read gripper state
		gripper_state = teleop_task->ReadGripperUserSwitch();

		if (remote_enabled==1 && gripper_state)
		{
			posori_task->reInitializeTask();
			centerPos_rob = posori_task->_current_position;
			centerRot_rob = posori_task->_current_orientation;
			teleop_task->setRobotCenter(centerPos_rob, centerRot_rob);

			HomePos_op = teleop_task->_current_position_device ;
			teleop_task->setDeviceCenter(HomePos_op, HomeRot_op);
	
			state = HAPTIC_CONTROL;
		}
		else if (restart_cycle == 1)
		{
			// reset joint controller to robot home position
			joint_task->reInitializeTask();
			joint_task->_desired_position = goal_posture;

			centerPos_rob << -0.5, 0.0, 0.6;
			centerRot_rob << -1.0, 0.0, 0.0,
							  0.0, 1.0, 0.0,
            				  0.0, 0.0, -1.0;
			posori_task->_desired_position = centerPos_rob;
			posori_task->_desired_orientation = centerRot_rob;

			// reset haptic device home position
			HomePos_op << 0.0, 0.01, 0.0;
			HomeRot_op.setIdentity();
			teleop_task->setDeviceCenter(HomePos_op, HomeRot_op);

			state = GOTO_INITIAL_CONFIG;
		}
	}
	else
	{
			command_torques.setZero(robot->dof());
			teleop_task->GravityCompTask();
	}

	//// Send to commands redis ////
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	prev_time = current_time;
	controller_counter++;
	}

	command_torques.setZero(robot->dof());
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	delete teleop_task;
	teleop_task = NULL;

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}
