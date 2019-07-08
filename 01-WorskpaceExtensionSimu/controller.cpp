// This example simulates the haptic interaction of a grinder with a part during a 
// polishing operation and implements the workspace extension algorithm.
// This example application loads a URDF world file with a part to polish. A kuka iiwa robot
// with physics and contact with the virtual world is simulated. The graphics of the task 
// interaction is rendered by using Chai3D.

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
const string robot_file = "../resources/01-WorskpaceExtensionSimu/kuka_iiwa.urdf";
const string robot_name = "Kuka-IIWA";
//Introduce a massless/stiff virtual avatar
// const string proxy_file = "../resources/01-WorskpaceExtensionSimu/proxy.urdf";
// const string proxy_name = "proxy";
const string proxy_file = "../resources/01-WorskpaceExtensionSimu/proxy6d.urdf";
const string proxy_name = "proxy6d";

// a haptic device handler
cHapticDeviceHandler* handler;
// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;
bool device_started = false;

//////////////////////////////////////////////////////////////////////
// //Definition of the state machine for the robot controller
#define GOTO_INITIAL_CONFIG               0
#define HAPTIC_CONTROL                    1
#define MAINTAIN_POSITION				  2


#define DEBUG	6//
// int state = DEBUG;

int state = GOTO_INITIAL_CONFIG;

unsigned long long controller_counter = 0;

const bool inertia_regularization = true;

//const bool flag_simulation = false; // To use while controlling real iiwa robot
const bool flag_simulation = true;


int remote_enabled = 1;
int restart_cycle = 0;
// int proxy_computation = 0;

// Create redis keys
const string REMOTE_ENABLED_KEY = "sai2::Sigma7Applications::sensors::remote_enabled";
const string RESTART_CYCLE_KEY = "sai2::Sigma7Applications::sensors::restart_cycle";

const string JOINT_ANGLES_KEY  = "sai2::Sigma7Applications::sensors::q";
const string JOINT_VELOCITIES_KEY = "sai2::Sigma7Applications::sensors::dq";
const string FORCE_SENSED_KEY = "sai2::Sigma7Applications::sensors::force_task_sensed";
const string JOINT_TORQUES_COMMANDED_KEY  = "sai2::Sigma7Applications::actuators::commanded_torque_robot";
const string JOINT_TORQUES_SENSED_KEY = "sai2::Sigma7Applications::sensors::sensed_torque_robot";

// Drift factor HapticWorkspaceExtension controller
const string DRIFT_PERC_FORCE_KEY = "sai2::Sigma7Applications::controller:Fdrift_perc";

//Proxy related keys (with massless/stiff virtual avatar)
const string JOINT_TORQUES_PROXY_KEY = "sai2::Sigma7Applications::actuators::commanded_torque_proxy";
const string JOINT_ANGLES_PROXY_KEY  = "sai2::Sigma7Applications::sensors::q_proxy";
const string JOINT_VELOCITIES_PROXY_KEY  = "sai2::Sigma7Applications::sensors::dq_proxy";

//Proxy related keys (with Finger proxy algorithm)
// const string TRANS_VELOCITY_KEY =  "sai2::Sigma7Applications::sensors::commanded_velocity";
// const string POSITION_COMMANDED_KEY = "sai2::Sigma7Applications::sensors::commanded_position";
// const string PROXY_FORCE_KEY = "sai2::Sigma7Applications::controller:Fproxy_simu";
// const string PROXY_COMPUTATION_KEY = "sai2::Sigma7Applications::sensors::proxy_computation";


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
	// position of robot in world
	Affine3d robot_pose_in_world = Affine3d::Identity();
	robot_pose_in_world.translation() = Vector3d(0, 0.0, 0.0);
	robot_pose_in_world.linear() = AngleAxisd(0.0, Vector3d::UnitZ()).toRotationMatrix();

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	redis_client.set(REMOTE_ENABLED_KEY, to_string(remote_enabled));
	redis_client.set(RESTART_CYCLE_KEY, to_string(restart_cycle));
	// redis_client.set(PROXY_COMPUTATION_KEY, to_string(proxy_computation));

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
	robot->updateModel();

	auto avatar = new Sai2Model::Sai2Model(proxy_file, false);
	avatar->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_PROXY_KEY);
	avatar->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_PROXY_KEY);
	avatar->updateModel();

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
	const Vector3d pos_in_link = Vector3d(0.0,0.0,0.04); /////////////////////////////////Define center of tool in end-effector frame
	auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	VectorXd posori_task_torques = VectorXd::Zero(robot->dof());

	posori_task->_kp_pos = 200.0;
	posori_task->_kv_pos = 15.0;
	posori_task->_kp_ori = 200.0;
	posori_task->_kv_ori = 20.0;

	// Define position and orientation of the task center ////////////////////////////////////////////////////////////////////////////////
	Vector3d centerPos_rob;
	centerPos_rob << -0.45, 0.0, 0.6;
	Matrix3d centerRot_rob;
	//centerRot_rob.setIdentity();
	centerRot_rob << -1.0, 0.0, 0.0,
					  0.0, 1.0, 0.0,
            		  0.0, 0.0, -1.0;
    MatrixXd _Offset_Lambda = MatrixXd::Zero(6,6); //Add constant value to the roational effective inertia of the equivalent unit mass system
    _Offset_Lambda << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    				0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    				0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    				0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
    				0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
    				0.0, 0.0, 0.0, 0.0, 0.0, 0.1;

	posori_task->_desired_position = centerPos_rob;
	posori_task->_desired_orientation = centerRot_rob;

	posori_task->_use_interpolation_flag=false;

	//// PosOriTask controller for proxy (manage the control in position of the avatar) ////
	const string proxy_link = "link5";//proxy end-effector
	const Vector3d poxy_pos_in_link = Vector3d(0.0,0.0,0.0); //Position of the control point in the end-effector frame
	auto posori_task_proxy = new Sai2Primitives::PosOriTask(avatar, proxy_link, poxy_pos_in_link);
	VectorXd posori_proxy_torques = VectorXd::Zero(avatar->dof());
	MatrixXd N_prec_proxy = MatrixXd::Identity(avatar->dof(), avatar->dof());

	posori_task_proxy->_kp_pos = 2000.0;
	posori_task_proxy->_kv_pos = 10.0;
	posori_task_proxy->_kp_ori = 1000.0;
	posori_task_proxy->_kv_ori = 8.0;

	posori_task_proxy->_desired_position = centerPos_rob;
	posori_task_proxy->_desired_orientation = centerRot_rob;

	posori_task_proxy->_use_interpolation_flag=false;

	//// Open loop teleoperation haptic controller ////
	// create a handler
	auto handler = new cHapticDeviceHandler();

	Matrix3d transformDev_Rob = robot_pose_in_world.linear();
	int device_number=0;
	auto teleop_task = new Sai2Primitives::OpenLoopTeleop(handler, device_number, centerPos_rob, centerRot_rob, transformDev_Rob);
	//Task scaling factors
	double Ks=1.0;
	double KsR=1.0;
	teleop_task->setScalingFactors(Ks, KsR);

	//// Set workspace extension parameters
	double Rmax_dev = 0.025; // in [mm]
	double Rmax_env = 0.2;
	double Thetamax_dev = 20*M_PI/180.0; // in [rad]
	double Thetamax_env = 40*M_PI/180.0; ///////////////////////////////////////////////////
	double Fdrift_perc = 200.0/100.0; ///////////////////////////////////////////////////////
	redis_client.set(DRIFT_PERC_FORCE_KEY, to_string(Fdrift_perc));

	teleop_task->setWorkspaceSize(Rmax_dev, Rmax_env, Thetamax_dev, Thetamax_env);
	teleop_task->setForceNoticeableDiff(Fdrift_perc);

	 // Center of the haptic device workspace
	Vector3d HomePos_op;
	HomePos_op << 0.0, 0.01, 0.0;
	Matrix3d HomeRot_op;
	HomeRot_op.setIdentity();
	teleop_task->setDeviceCenter(HomePos_op, HomeRot_op);

	// Force feedback stiffness proxy parameters
	double proxy_position_impedance = 3000.0;
	double proxy_position_damping = 5.0;
	double proxy_orientation_impedance = 20.0;
	double proxy_orientation_damping = 0.1;
	teleop_task->setVirtualProxyGains (proxy_position_impedance, proxy_position_damping,
									   proxy_orientation_impedance, proxy_orientation_damping);
	// Set haptic controllers parameters
	Matrix3d Red_factor_rot = Matrix3d::Identity();
	Matrix3d Red_factor_trans = Matrix3d::Identity();
	Red_factor_rot << 1/20.0, 0.0, 0.0,
						  0.0, 1/20.0, 0.0,
						  0.0, 0.0, 1/20.0;
	Red_factor_trans << 1/2.0, 0.0, 0.0,
						  0.0, 1/2.0, 0.0,
						  0.0, 0.0, 1/2.0;
	double kp_robot_trans_velocity = 10.0;
	double ki_robot_trans_velocity = 0.0;
	double kp_robot_rot_velocity =10.0;
	double ki_robot_rot_velocity =0.0;
	double robot_trans_admittance =1/50.0;
	double robot_rot_admittance =1/1.5;
	teleop_task->setForceFeedbackCtrlGains (kp_robot_trans_velocity, ki_robot_trans_velocity,
									kp_robot_rot_velocity, ki_robot_rot_velocity,
									robot_trans_admittance, robot_rot_admittance,
									Red_factor_trans, Red_factor_rot);

	Vector3d pos_rob = Vector3d::Zero(); // Set position and orientation in robot frame

	Matrix3d rot_rob = Matrix3d::Identity();
	Vector3d pos_rob_model = Vector3d::Zero(); // Model position and orientation in robot frame
	Matrix3d rot_rob_model = Matrix3d::Identity();
	Vector3d vel_rot_rob_model = Vector3d::Zero(); // Model linear and angular velocities in robot frame
	Vector3d vel_trans_rob_model = Vector3d::Zero();
	VectorXd f_task_sensed = VectorXd::Zero(6);
	VectorXd torques_sensed = VectorXd::Zero(robot->dof());

	Vector3d pos_proxy_model = Vector3d::Zero(); // Model position and orientation in robot frame
	Matrix3d rot_proxy_model = Matrix3d::Identity();
	Vector3d vel_rot_proxy_model = Vector3d::Zero(); // Model linear and angular velocities in robot frame
	Vector3d vel_trans_proxy_model = Vector3d::Zero();

	// Vector3d f_proxy_simulation = Vector3d::Zero();

	// Vector3d command_trans_velocity;
	// Vector3d command_position;
	// command_trans_velocity.setZero();
	// command_position = centerPos_rob;

	// write initial task force in redis
	redis_client.setEigenMatrixJSON(FORCE_SENSED_KEY,f_task_sensed);

	// redis_client.setEigenMatrixJSON(PROXY_FORCE_KEY,f_proxy_simulation);
	// write initial sensed torques in redis
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_SENSED_KEY,torques_sensed);

	teleop_task->_haptic_feedback_from_proxy = false;
	teleop_task->_filter_on = false;
	double fc_force=0.02;
	double fc_moment=0.02;
	teleop_task->setFilterCutOffFreq(fc_force, fc_moment);

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
	// read proxy info from redis
	avatar->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_PROXY_KEY);
	avatar->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_PROXY_KEY);

	// f_proxy_simulation = redis_client.getEigenMatrixJSON(PROXY_FORCE_KEY);
	
	//// update robot model ////
	robot->updateModel();
	avatar->updateModel();

	// Update position and orientation of the robot from the model
	robot->position(pos_rob_model, link_name, pos_in_link);
	robot->linearVelocity(vel_trans_rob_model, link_name, pos_in_link);
	robot->rotation(rot_rob_model, link_name);
	robot->angularVelocity(vel_rot_rob_model, link_name);

	// Update position and orientation of the avatar from the model
	avatar->position(pos_proxy_model, proxy_link, poxy_pos_in_link);
	avatar->linearVelocity(vel_trans_proxy_model, proxy_link, poxy_pos_in_link);
	avatar->rotation(rot_proxy_model, proxy_link);
	avatar->angularVelocity(vel_rot_proxy_model, proxy_link);

	// read force sensor data
	f_task_sensed = redis_client.getEigenMatrixJSON(FORCE_SENSED_KEY);

	//read sensed_torque from robot
	torques_sensed = redis_client.getEigenMatrixJSON(JOINT_TORQUES_SENSED_KEY);


	// read renabling/disabling teleoperation
	remote_enabled = stoi(redis_client.get(REMOTE_ENABLED_KEY));
	// read restar cycle key
	restart_cycle = stoi(redis_client.get(RESTART_CYCLE_KEY));


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//// Run Robot/Haptic Device state machine ////
	if(state == GOTO_INITIAL_CONFIG)
	{
		// update proxy position task
		N_prec_proxy.setIdentity();
		posori_task_proxy->updateTaskModel(N_prec_proxy);
		posori_task_proxy->computeTorques(posori_proxy_torques);


		// update tasks model and priority 
		N_prec.setIdentity();
		posori_task->updateTaskModel(N_prec);
		N_prec = posori_task->_N;
		// Adjust effective inertia of the equivalent unit mass system to ensure a sufficient roational stiffness
		posori_task->_Lambda = posori_task->_Lambda + _Offset_Lambda;

		joint_task->updateTaskModel(N_prec);
		// compute torques
		posori_task->computeTorques(posori_task_torques);
		joint_task->computeTorques(joint_task_torques);
		command_torques = posori_task_torques + joint_task_torques;

		// compute homing haptic device
		teleop_task->HomingTask();

		// Read new gripper state
        gripper_state = teleop_task->ReadGripperUserSwitch();

		if( remote_enabled==1 && teleop_task->device_homed && gripper_state && (posori_task->_desired_position - posori_task->_current_position).norm() < 0.02 && posori_task->_orientation_error.norm()<0.01)
		{
			
			// Reinitialize controllers
			joint_task->reInitializeTask();
			joint_task->_desired_position = goal_posture;
			posori_task->reInitializeTask();

			posori_task_proxy->reInitializeTask();


			HomePos_op = teleop_task->_current_position_device ;
			teleop_task->setDeviceCenter(HomePos_op, HomeRot_op);

			centerPos_rob = posori_task->_current_position;
			centerRot_rob = posori_task->_current_orientation;
			teleop_task->setRobotCenter(centerPos_rob, centerRot_rob);

			// command_trans_velocity.setZero();
		 //    command_position = centerPos_rob;

		    // proxy_computation = 1;
		
			state = HAPTIC_CONTROL;
		}
	}


	else if(state == HAPTIC_CONTROL)
	{


		//Compute haptic commands
		teleop_task->_haptic_feedback_from_proxy = false; // If set to true, the force feedback is computed from a stiffness/damping proxy.
	    teleop_task->_filter_on = false;

		// Force feedback from force sensor
		// teleop_task->updateSensedForce(f_proxy_simulation); // BUG... - Computed through chai3D Finger Proxy algorithm
		teleop_task->updateSensedForce(f_task_sensed);
		teleop_task->_send_haptic_feedback = true;
		
		// Force feedback from proxy
		// teleop_task->updateVirtualProxyPositionVelocity(pos_proxy_model, vel_trans_proxy_model, rot_proxy_model, vel_rot_proxy_model);
		// teleop_task->_send_haptic_feedback = true;

		// if (f_task_sensed.norm()>=0.000001)
		// {
		// 	teleop_task->_send_haptic_feedback = true;	
		//  }
		//  else
		//  {
		//  	teleop_task->_send_haptic_feedback = false;
		//  }

		// Force feedback from velocity PI in admittance-type scheme
		// teleop_task->updateSensedRobotPositionVelocity(pos_rob_model, vel_trans_rob_model, rot_rob_model, vel_rot_rob_model);


	    // teleop_task->computeHapticCommands3d(pos_rob);
		teleop_task->computeHapticCommands6d(pos_rob, rot_rob);
		//teleop_task->computeHapticCommandsWorkspaceExtension6d(pos_rob, rot_rob);
		// teleop_task->computeHapticCommandsWorkspaceExtension3d(pos_rob);
		
		//Send set position and velocity from haptic device to simulation for finger proxy calculation of chai3D
		// command_trans_velocity = teleop_task->_vel_trans_rob;
		// command_position = pos_rob;

		//// Impedance controller of the robot ////
		// set goal position
		posori_task->_desired_position = pos_rob;
		posori_task->_desired_orientation = rot_rob;
		// posori_task->_desired_orientation = centerRot_rob; // If only 3D feedback/control

		// Set new commanded proxy position
		posori_task_proxy->_desired_position = pos_rob;
		posori_task_proxy->_desired_orientation = rot_rob; 
		// posori_task_proxy->_desired_orientation = centerRot_rob; // If only 3D feedback/control

		// update proxy position task
		N_prec_proxy.setIdentity();
		posori_task_proxy->updateTaskModel(N_prec_proxy);
		posori_task_proxy->computeTorques(posori_proxy_torques);
		
		// update model and priority
		N_prec.setIdentity();
		posori_task->updateTaskModel(N_prec);
		N_prec = posori_task->_N;
		// Adjust effective inertia of the equivalent unit mass system to ensure a sufficient roational stiffness
		posori_task->_Lambda = posori_task->_Lambda + _Offset_Lambda;

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

			// reinitialize proxy
			posori_task_proxy->reInitializeTask();

			// set current haptic device position
			teleop_task->setDeviceCenter(teleop_task->_current_position_device, teleop_task->_current_rotation_device);

			// command_trans_velocity.setZero();
			// command_position = centerPos_rob;

			// proxy_computation = 0;

			state = MAINTAIN_POSITION;

		}
	}

	else if(state == MAINTAIN_POSITION)
	{
		
		// update proxy position task
		N_prec_proxy.setIdentity();
		posori_task_proxy->updateTaskModel(N_prec_proxy);
		posori_task_proxy->computeTorques(posori_proxy_torques);

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
			posori_task_proxy->reInitializeTask();
			posori_task->reInitializeTask();
			centerPos_rob = posori_task->_current_position;
			centerRot_rob = posori_task->_current_orientation;
			teleop_task->setRobotCenter(centerPos_rob, centerRot_rob);

			HomePos_op = teleop_task->_current_position_device ;
			teleop_task->setDeviceCenter(HomePos_op, HomeRot_op);

			// command_trans_velocity.setZero();
		 //    command_position = centerPos_rob;

		    // proxy_computation = 1;

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

			posori_task_proxy->_desired_position = centerPos_rob;
			posori_task_proxy->_desired_orientation = centerRot_rob;

			// reset haptic device home position
			HomePos_op << 0.0, 0.01, 0.0;
			HomeRot_op.setIdentity();
			teleop_task->setDeviceCenter(HomePos_op, HomeRot_op);

			// command_trans_velocity.setZero();
			// command_position = centerPos_rob;

			// proxy_computation = 0;

			state = GOTO_INITIAL_CONFIG;
		}
	}
	else
	{
			command_torques.setZero(robot->dof());
			posori_proxy_torques.setZero(avatar->dof());
			// command_trans_velocity.setZero();
			// command_position = centerPos_rob;
			teleop_task->GravityCompTask();
			// proxy_computation = 0;
	}

	//// Send to commands redis ////
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	// redis_client.setEigenMatrixJSON(TRANS_VELOCITY_KEY, command_trans_velocity);
	// redis_client.setEigenMatrixJSON(POSITION_COMMANDED_KEY, command_position);

	// redis_client.set(PROXY_COMPUTATION_KEY, to_string(proxy_computation));

	redis_client.setEigenMatrixJSON(JOINT_TORQUES_PROXY_KEY, posori_proxy_torques);
	


	prev_time = current_time;
	controller_counter++;
	}

	command_torques.setZero(robot->dof());
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	// command_trans_velocity.setZero();
	// redis_client.setEigenMatrixJSON(TRANS_VELOCITY_KEY, command_trans_velocity);

	// command_position = centerPos_rob;
	// redis_client.setEigenMatrixJSON(POSITION_COMMANDED_KEY, command_position);

	posori_proxy_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_PROXY_KEY, posori_proxy_torques);

	// proxy_computation = 0;

	delete teleop_task;
	teleop_task = NULL;

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}

