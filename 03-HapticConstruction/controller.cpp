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
 const string robot_file = "../resources/03-HapticConstruction/kuka_iiwa.urdf";
 const string robot_name = "Kuka-IIWA";

//////////////////////////////////////////////////////////////////////
//Definition of the state machine for the robot controller
#define MOVE_TO_INITIAL      0 // Move the iiwa robot to the center of the task workspace
#define WAIT_DEVICE_READY	 1 // Waiting for haptic device to be in home position and for first user input
#define REMOTE_CONTROL       2 // Remote control of the iiwa robot from the haptic device
#define END_REMOTE_TASK		 3 // End of the remote task, waiting for user input to redo a new task

//Definition of the state machine for the haptic device controller
#define MOVE_TO_HOME      	 0 // Move the haptic device to the home position
#define WAIT_ROBOT_READY	 1 // Waiting for the controlled robot to be in the center of task environment and for first user input
#define WAIT_USER_INPUT		 4 //

#define DEBUG	6//
// int state_dev = DEBUG;

int state = MOVE_TO_INITIAL;
int state_dev = MOVE_TO_HOME;

// redis keys:
// - read:
string JOINT_ANGLES_KEY;
string JOINT_VELOCITIES_KEY;
string FORCE_SENSED_KEY;
string REMOTE_ENABLED_KEY;

// - write
string JOINT_TORQUES_COMMANDED_KEY;

// - model (for implementation with real robot)
string MASSMATRIX_KEY;
string CORIOLIS_KEY;
string ROBOT_GRAVITY_KEY;

// gains position controllers
const string KP_JOINT_KEY = "sai2::Sigma7Applications::controller::kp_joint";
const string KV_JOINT_KEY = "sai2::Sigma7Applications::controller::kv_joint";
const string KP_POS_KEY = "sai2::Sigma7Applications::controller::kp_pos";
const string KV_POS_KEY = "sai2::Sigma7Applications::controller::kv_pos";
const string KP_ORI_KEY = "sai2::Sigma7Applications::controller::kp_ori";
const string KV_ORI_KEY = "sai2::Sigma7Applications::controller::kv_ori";

// Logger/Plotter keys
const string DEVICE_FORCE_KEY = "sai2::Sigma7Applications::commands::device_force";
const string DEVICE_TORQUE_KEY = "sai2::Sigma7Applications::commands::device_torque";
const string DRIFT_FORCE_KEY = "sai2::Sigma7Applications::commands::drift_force";
const string DRIFT_TORQUE_KEY = "sai2::Sigma7Applications::commands::drift_torque";
const string DRIFT_VEL_TRANS_KEY = "sai2::Sigma7Applications::commands::drift_trans_vel";
const string DRIFT_VEL_ROT_KEY = "sai2::Sigma7Applications::commands::drift_rot_vel";
const string SIM_TIMESTAMP_KEY = "sai2::Sigma7Applications::simulation::timestamp";

unsigned long long controller_counter = 0;

//const bool flag_simulation = false; // To use while controlling real iiwa robot
const bool flag_simulation = true;

const bool inertia_regularization = true;

bool robot_ready = false;
bool device_ready = false;


//// Declaration of internal functions ////
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
 	
	int remote_enabled=0;

	// Create redis keys
	REMOTE_ENABLED_KEY = "sai2::Sigma7Applications::sensors::remote_enabled";

	if(flag_simulation)
	{
		JOINT_ANGLES_KEY  = "sai2::Sigma7Applications::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::Sigma7Applications::sensors::dq";
		FORCE_SENSED_KEY = "sai2::Sigma7Applications::sensors::force_task_sensed";
		JOINT_TORQUES_COMMANDED_KEY  = "sai2::Sigma7Applications::actuators::torque_joint_robot";
		
	}
	else
	{

//		JOINT_ANGLES_KEY  = "sai2::Kuka-IIWA::sensors::q";
//		JOINT_VELOCITIES_KEY = "sai2::Kuka-IIWA::sensors::dq";
//		FORCE_SENSED_KEY = "sai2::Kuka-IIWA::sensors::force";
//		JOINT_TORQUES_COMMANDED_KEY  = "sai2::Kuka-IIWA::actuators::fgc";
//		MASSMATRIX_KEY = "sai2::Kuka-IIWA::sensors::model::massmatrix";
//		CORIOLIS_KEY = "sai2::Kuka-IIWA::sensors::model::coriolis";
//		ROBOT_GRAVITY_KEY = "sai2::Kuka-IIWA::sensors::model::robot_gravity";		

	}

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

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
	redis_client.set(KP_JOINT_KEY, to_string(joint_task->_kp));
	redis_client.set(KV_JOINT_KEY, to_string(joint_task->_kv));
	VectorXd command_torques = VectorXd::Zero(robot->dof());

	// Define goal position according to the desired posture ///////////////////////////////////////////////////////////////////////////////////////////////// 
	VectorXd goal_posture(robot->dof());
	goal_posture << 0.0, -20.0, 0.0, 30.0, 0.0, -45.0, 0.0; /////////////////////////// 
	goal_posture *= M_PI/180.0;
	joint_task->_desired_position = goal_posture;

	//// PosOriTask controller (manage the control in position of the iiwa robot) ////
	const string link_name = "link6";
	const Vector3d pos_in_link = Vector3d(0.0,0.0,0.015); /////////////////////////////////Define center of tool in end-effector frame
	auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	VectorXd posori_task_torques = VectorXd::Zero(robot->dof());

	posori_task->_kp_pos = 50.0;
	posori_task->_kv_pos = 14.0;
	posori_task->_kp_ori = 50.0;
	posori_task->_kv_ori = 14.0;
	redis_client.set(KP_POS_KEY, to_string(posori_task->_kp_pos));
	redis_client.set(KV_POS_KEY, to_string(posori_task->_kv_pos));
	redis_client.set(KP_ORI_KEY, to_string(posori_task->_kp_ori));
	redis_client.set(KV_ORI_KEY, to_string(posori_task->_kv_ori));

	// Define position and orientation of the task center ////////////////////////////////////////////////////////////////////////////////
	Vector3d centerPos_rob;
	centerPos_rob << -0.5, 0.0, 0.9;

	Matrix3d centerRot_rob;
	 //centerRot_rob.setIdentity();
	 centerRot_rob << 0.0, 0.0, -1.0,
	 				  0.0, 1.0, 0.0,
	            		  1.0, 0.0, 0.0;

	posori_task->_desired_position = centerPos_rob;
	posori_task->_desired_orientation = centerRot_rob;


	//// Open loop teleoperation haptic controller ////
	// create a handler
	auto handler = new cHapticDeviceHandler();

	Matrix3d transformDev_Rob = Eigen::Matrix3d::Identity();
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
	double k_pos = 400.0;
	double d_pos = 20.0;
	double k_ori = 10.0;
	double d_ori = 0.5;
	Matrix3d Red_factor_rot = Matrix3d::Identity();
	Matrix3d Red_factor_trans = Matrix3d::Identity();
	Red_factor_rot << 0.0, 0.0, 0.0,
						  0.0, 0.0, 0.0,
						  0.0, 0.0, 0.0;

	Red_factor_trans << 1.0, 0.0, 0.0,
						  0.0, 1.0, 0.0,
						  0.0, 0.0, 1.0;
	teleop_task->setForceFeedbackCtrlGains (k_pos, d_pos, k_ori, d_ori, Red_factor_rot, Red_factor_trans);

	//Enable user switch
	teleop_task->EnableGripperUserSwitch();
	bool gripper_state=false;

	Vector3d pos_rob = Vector3d::Zero(); // Set position and orientation in robot frame
	Matrix3d rot_rob = Matrix3d::Identity();
	Vector3d pos_rob_model = Vector3d::Zero(); // Model position and orientation in robot frame
	Matrix3d rot_rob_model = Matrix3d::Identity();
	Vector3d vel_rot_rob_model = Vector3d::Zero(); // Model linear and angular velocities in robot frame
	Vector3d vel_trans_rob_model = Vector3d::Zero();
	VectorXd f_task_sensed = VectorXd::Zero(6);


	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); //Compiler en mode release
	double _t_diff=1.0/1000; //////////////////////////////////////////////////////////////////////////////////
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	/////////////////////////////// cyclic ////////////////////////////////////////
	while (runloop) {
	// wait for next scheduled loop
	timer.waitForNextLoop();
	double time = timer.elapsedTime() - start_time;

	//// Read robots data ////
	// read iiwa robot info from redis
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
	f_task_sensed = redis_client.getEigenMatrixJSON(FORCE_SENSED_KEY);
	remote_enabled = stoi(redis_client.get(REMOTE_ENABLED_KEY));

	//// update robot model ////
	if(flag_simulation)
	{
		robot->updateModel();
	}
	else
	{
		// joint_task->_kp = stod(redis_client.get(KP_JOINT_KEY));
		// joint_task->_kv = stod(redis_client.get(KV_JOINT_KEY));
		// posori_task->_kp_pos = stod(redis_client.get(KP_POS_KEY));
		// posori_task->_kv_pos = stod(redis_client.get(KV_POS_KEY));
		// posori_task->_kp_ori = stod(redis_client.get(KP_ORI_KEY));
		// posori_task->_kv_ori = stod(redis_client.get(KV_ORI_KEY));
		// robot->updateKinematics();
		// robot->_M = redis_client.getEigenMatrixJSON(MASSMATRIX_KEY);
		// if(inertia_regularization)
		// {
		// 	robot->_M(4,4) += 0.07;
		// 	robot->_M(5,5) += 0.07;
		// 	robot->_M(6,6) += 0.07;
		// }
		// // cout << robot->_M << endl;
		// // robot->_M = Eigen::MatrixXd::Identity(robot->dof(), robot->dof());
		// robot->_M_inv = robot->_M.inverse();
	}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//// Run Robot controller state machine ////
	if(state == MOVE_TO_INITIAL)
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

		if( (posori_task->_desired_position - posori_task->_current_position).norm() < 0.001 && posori_task->_orientation_error.norm()<0.001)
		{
			cout << "Waiting for haptice device" << endl;
			robot_ready = true;
			state = WAIT_DEVICE_READY;
		}

	}

	else if(state == WAIT_DEVICE_READY)
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

		 if(device_ready == true && remote_enabled == 1)
		{
			cout << "Remote control of the robot" << endl;
			// Reinitialize controllers
			joint_task->reInitializeTask();
			joint_task->_desired_position = goal_posture;
			posori_task->reInitializeTask();
			teleop_task->setRobotCenter(posori_task->_current_position, posori_task->_current_orientation);
			state = REMOTE_CONTROL;
		}
	}


	else if(state == REMOTE_CONTROL)
	{	
		// // Update position and orientation of the robot from the model
		robot->position(pos_rob_model, link_name, pos_in_link);
		robot->linearVelocity(vel_trans_rob_model, link_name, pos_in_link);
		robot->rotation(rot_rob_model, link_name);
		robot->angularVelocity(vel_rot_rob_model, link_name);
		// //Compute haptic commands
		teleop_task->_haptic_feedback_from_proxy = false; // If set to true, the force feedback is computed from a stiffness/damping proxy.
	    teleop_task->_filter_on = false;
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
		
		//teleop_task->computeHapticCommandsWorkspaceExtension6d(pos_rob, rot_rob);
		teleop_task->computeHapticCommands6d(pos_rob, rot_rob);
		//teleop_task->computeHapticCommands3d(pos_rob);


		//// Impedance controller of the robot ////
		// set goal position
		posori_task->_desired_position = pos_rob;
		posori_task->_desired_orientation = rot_rob;
		
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
			robot_ready = false;
			state = END_REMOTE_TASK;

		}
	}

	else if(state == END_REMOTE_TASK)
	{
		//Maintain the robot in current position
		// update tasks model
		N_prec.setIdentity();
		joint_task->updateTaskModel(N_prec);
		// compute torques
		joint_task->computeTorques(joint_task_torques);
		command_torques = joint_task_torques;

		if(remote_enabled == 1) 
		{
			// Reinitialize controllers
			joint_task->reInitializeTask();
			joint_task->_desired_position = goal_posture;

			posori_task->reInitializeTask();
			posori_task->_desired_position = centerPos_rob;
			posori_task->_desired_orientation = centerRot_rob;
			
			state = MOVE_TO_INITIAL;
		}

	}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//// Run Haptic Device controller state machine ////
	
	if(state_dev == MOVE_TO_HOME)
	{
		teleop_task->HomingTask();

		if(teleop_task->device_homed)
		{
			
			cout << "Waiting for robot ready" << endl;
			state_dev = WAIT_ROBOT_READY;
		}
	}


	else if(state_dev == WAIT_ROBOT_READY)
	{
		teleop_task->HomingTask();

        if(robot_ready == true)
		{
			cout << "Waiting for user input" << endl;
			state_dev = WAIT_USER_INPUT;
		}
	}

	else if(state_dev == WAIT_USER_INPUT)
	{
		teleop_task->HomingTask();
        // Read new gripper state
        gripper_state = teleop_task->ReadGripperUserSwitch();
        
		if(gripper_state && remote_enabled == 1)
		{
			teleop_task->GravityCompTask();
			device_ready = true;
			state_dev = REMOTE_CONTROL;
		}
	}

	else if(state_dev == REMOTE_CONTROL)
	{	
		
		//Haptic feedback sent through robot state machine

		if(remote_enabled == 0)
		{
			device_ready = false;
			teleop_task->GravityCompTask();
			state_dev = END_REMOTE_TASK;
		}

	}

	else if(state_dev == END_REMOTE_TASK)
	{
		teleop_task->GravityCompTask();

		if(remote_enabled == 1)
		{
			// Reinitialize controllers
			teleop_task->GravityCompTask();

			state_dev = MOVE_TO_HOME;
		}
		
	}


	//// Send to commands redis ////
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	redis_client.setEigenMatrixJSON(DEVICE_FORCE_KEY, teleop_task->_device_force);
	redis_client.setEigenMatrixJSON(DEVICE_TORQUE_KEY, teleop_task->_device_torque);
	redis_client.setEigenMatrixJSON(DRIFT_FORCE_KEY, teleop_task->_drift_force);
	redis_client.setEigenMatrixJSON(DRIFT_TORQUE_KEY, teleop_task->_drift_torque);
	redis_client.setEigenMatrixJSON(DRIFT_VEL_TRANS_KEY, teleop_task->_drift_trans_velocity);
	redis_client.setEigenMatrixJSON(DRIFT_VEL_ROT_KEY, teleop_task->_drift_trans_velocity);
	redis_client.set(SIM_TIMESTAMP_KEY, to_string(time));

	controller_counter++;
	}

	delete teleop_task;
	teleop_task = NULL;

	command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}
