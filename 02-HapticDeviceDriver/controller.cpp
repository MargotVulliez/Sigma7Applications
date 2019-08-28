// The application simulates the remote control of a Kuka iiwa interacting with a rigid box.
// This controller manages the application state machine and executes the haptic device's and
// the robot's controllers from the sai2-primitive tasks. It implements the HapticController
// class to test the different bilateral teleoperation scheme with force feedback.
// The device driver transmits the commands from and toward the haptic device. The robot behavior
// can be simulated by running simviz.cpp or the real robot can be controlled with its own driver.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

// Include task primitives from 'sai2-primitives'
#include "tasks/JointTask.h"
#include "tasks/PosOriTask.h"
#include "haptic_tasks/HapticController.h"

#include <iostream>
#include <string>
#include <sstream>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

//// Define robots ////
const string robot_file = "../resources/02-HapticDeviceDriver/kuka_iiwa.urdf";
const string robot_name = "Kuka-IIWA";
//Introduce a massless/stiff virtual avatar
// const string proxy_file = "../resources/02-HapticDeviceDriver/proxy.urdf";
// const string proxy_name = "proxy";
const string proxy_file = "../resources/02-HapticDeviceDriver/proxy6d.urdf";
const string proxy_name = "proxy6d";

bool admittance_control = false;

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

//// Create redis keys ////
// Application state related keys
const string REMOTE_ENABLED_KEY = "sai2::Sigma7Applications::sensors::remote_enabled";
const string RESTART_CYCLE_KEY = "sai2::Sigma7Applications::sensors::restart_cycle";

// Robot related keys
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

//// Haptic device related keys ////
// Maximum stiffness, damping and force specifications
vector<string> DEVICE_MAX_STIFFNESS_KEYS = {
	"sai2::ChaiHapticDevice::device0::specifications::max_stiffness",
	"sai2::ChaiHapticDevice::device1::specifications::max_stiffness",
	};
vector<string> DEVICE_MAX_DAMPING_KEYS = {
	"sai2::ChaiHapticDevice::device0::specifications::max_damping",
	"sai2::ChaiHapticDevice::device1::specifications::max_damping",
	};
vector<string> DEVICE_MAX_FORCE_KEYS = {
	"sai2::ChaiHapticDevice::device0::specifications::max_force",
	"sai2::ChaiHapticDevice::device1::specifications::max_force",
	};
// Set force and torque feedback of the haptic device
vector<string> DEVICE_COMMANDED_FORCE_KEYS = {
	"sai2::ChaiHapticDevice::device0::actuators::commanded_force",
	"sai2::ChaiHapticDevice::device1::actuators::commanded_force",
	};
vector<string> DEVICE_COMMANDED_TORQUE_KEYS = {
	"sai2::ChaiHapticDevice::device0::actuators::commanded_torque",
	"sai2::ChaiHapticDevice::device1::actuators::commanded_torque",
	};
vector<string> DEVICE_COMMANDED_GRIPPER_FORCE_KEYS = {
	"sai2::ChaiHapticDevice::device0::actuators::commanded_force_gripper",
	"sai2::ChaiHapticDevice::device1::actuators::commanded_force_gripper",
	};
// Haptic device current position and rotation
vector<string> DEVICE_POSITION_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_position",
	"sai2::ChaiHapticDevice::device1::sensors::current_position",
	};
vector<string> DEVICE_ROTATION_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_rotation",
	"sai2::ChaiHapticDevice::device1::sensors::current_rotation",
	};
vector<string> DEVICE_GRIPPER_POSITION_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_position_gripper",
	"sai2::ChaiHapticDevice::device1::sensors::current_position_gripper",
	};
// Haptic device current velocity
vector<string> DEVICE_TRANS_VELOCITY_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_trans_velocity",
	"sai2::ChaiHapticDevice::device1::sensors::current_trans_velocity",
	};
vector<string> DEVICE_ROT_VELOCITY_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_rot_velocity",
	"sai2::ChaiHapticDevice::device1::sensors::current_rot_velocity",
	};
vector<string> DEVICE_GRIPPER_VELOCITY_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_gripper_velocity",
	"sai2::ChaiHapticDevice::device1::sensors::current_gripper_velocity",
	};
vector<string> DEVICE_SENSED_FORCE_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::sensed_force",
	"sai2::ChaiHapticDevice::device1::sensors::sensed_force",
	};
vector<string> DEVICE_SENSED_TORQUE_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::sensed_torque",
	"sai2::ChaiHapticDevice::device1::sensors::sensed_torque",
	};

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
	joint_task->_kp = 200.0;
	joint_task->_kv = 15.0;
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
	posori_task->_ki_pos = 0.0;
	posori_task->_kp_ori = 200.0;
	posori_task->_kv_ori = 20.0;
	posori_task->_ki_ori = 0.0;

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
	Matrix3d transformDev_Rob = robot_pose_in_world.linear();
	auto teleop_task = new Sai2Primitives::HapticController(centerPos_rob, centerRot_rob, transformDev_Rob);
	//Task scaling factors
	double Ks=1.0;
	double KsR=1.0;
	teleop_task->setScalingFactors(Ks, KsR);

	//// Set workspace extension parameters
	double Rmax_dev = 0.025; // in [mm]
	double Rmax_env = 0.2;
	double Thetamax_dev = 20*M_PI/180.0; // in [rad]
	double Thetamax_env = 40*M_PI/180.0; ///////////////////////////////////////////////////
	double Fdrift_perc = 60.0/100.0; ///////////////////////////////////////////////////////
	double Vdrift_perc = 30.0/100;
	redis_client.set(DRIFT_PERC_FORCE_KEY, to_string(Fdrift_perc));
	teleop_task->setWorkspaceSize(Rmax_dev, Rmax_env, Thetamax_dev, Thetamax_env);
	teleop_task->setNoticeableDiff(Fdrift_perc, Vdrift_perc);

	// Center of the haptic device workspace
	Vector3d HomePos_op;
	HomePos_op << 0.0, 0.01, 0.0;
	Matrix3d HomeRot_op;
	HomeRot_op.setIdentity();
	teleop_task->setDeviceCenter(HomePos_op, HomeRot_op);

	// Force feedback stiffness proxy parameters
	double proxy_position_impedance = 1400.0;
	double proxy_position_damping = 8.0;
	double proxy_orientation_impedance = 20.0;
	double proxy_orientation_damping = 0.5;
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

	// read haptic device specifications
	VectorXd _max_stiffness_device0 = redis_client.getEigenMatrixJSON(DEVICE_MAX_STIFFNESS_KEYS[0]);
	VectorXd _max_damping_device0 = redis_client.getEigenMatrixJSON(DEVICE_MAX_DAMPING_KEYS[0]);
	VectorXd _max_force_device0 = redis_client.getEigenMatrixJSON(DEVICE_MAX_FORCE_KEYS[0]);
	// VectorXd _max_stiffness_device1 = redis_client.getEigenMatrixJSON(DEVICE_MAX_STIFFNESS_KEYS[1]);
	// VectorXd _max_damping_device1 = redis_client.getEigenMatrixJSON(DEVICE_MAX_DAMPING_KEYS[1]);
	// VectorXd _max_force_device1 = redis_client.getEigenMatrixJSON(DEVICE_MAX_FORCE_KEYS[1]);
	
	// set the device specifications to the haptic controller
	teleop_task->_max_linear_stiffness_device = _max_stiffness_device0[0];
	teleop_task->_max_angular_stiffness_device = _max_stiffness_device0[1];
	teleop_task->_max_linear_damping_device = _max_damping_device0[0];
	teleop_task->_max_angular_damping_device = _max_damping_device0[1];
	teleop_task->_max_force_device = _max_force_device0[0];
	teleop_task->_max_torque_device = _max_force_device0[1];

	// Create robot commanded vectors
	Vector3d desired_position_robot = Vector3d::Zero(); // Set position and orientation in robot frame
	Matrix3d desired_rotation_robot = Matrix3d::Identity();
	Vector3d desired_trans_velocity_robot = Vector3d::Zero(); // Set velocities in robot frame from admittance controller
	Vector3d desired_rot_velocity_robot = Vector3d::Zero();

	// Create robot current vectors
	Vector3d pos_rob_model = Vector3d::Zero(); // Model position and orientation in robot frame
	Matrix3d rot_rob_model = Matrix3d::Identity();
	Vector3d vel_rot_rob_model = Vector3d::Zero(); // Model linear and angular velocities in robot frame
	Vector3d vel_trans_rob_model = Vector3d::Zero();
	VectorXd f_task_sensed = VectorXd::Zero(6);
	VectorXd torques_sensed = VectorXd::Zero(robot->dof());
	// create proxy current vectors
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
	double fc_force=0.04;
	double fc_moment=0.04;
	teleop_task->setFilterCutOffFreq(fc_force, fc_moment);
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
	teleop_task->updateSensedRobotPositionVelocity(pos_rob_model, vel_trans_rob_model, rot_rob_model, vel_rot_rob_model);

	// Update position and orientation of the avatar from the model
	avatar->position(pos_proxy_model, proxy_link, poxy_pos_in_link);
	avatar->linearVelocity(vel_trans_proxy_model, proxy_link, poxy_pos_in_link);
	avatar->rotation(rot_proxy_model, proxy_link);
	avatar->angularVelocity(vel_rot_proxy_model, proxy_link);
		
	teleop_task->updateVirtualProxyPositionVelocity(pos_proxy_model, vel_trans_proxy_model, rot_proxy_model, vel_rot_proxy_model);

	// read force sensor data
	f_task_sensed = redis_client.getEigenMatrixJSON(FORCE_SENSED_KEY);

	//read sensed_torque from robot
	torques_sensed = redis_client.getEigenMatrixJSON(JOINT_TORQUES_SENSED_KEY);


	// read renabling/disabling teleoperation
	remote_enabled = stoi(redis_client.get(REMOTE_ENABLED_KEY));
	// read restar cycle key
	restart_cycle = stoi(redis_client.get(RESTART_CYCLE_KEY));

	//// Read haptic device data ////
	// Haptic device and gripper position and velocity
	teleop_task->_current_position_device = redis_client.getEigenMatrixJSON(DEVICE_POSITION_KEYS[0]);
	teleop_task->_current_rotation_device = redis_client.getEigenMatrixJSON(DEVICE_ROTATION_KEYS[0]);
	teleop_task->_current_trans_velocity_device = redis_client.getEigenMatrixJSON(DEVICE_TRANS_VELOCITY_KEYS[0]);
	teleop_task->_current_rot_velocity_device = redis_client.getEigenMatrixJSON(DEVICE_ROT_VELOCITY_KEYS[0]);
	teleop_task->_current_position_gripper_device = stod(redis_client.get(DEVICE_GRIPPER_POSITION_KEYS[0]));
	teleop_task->_current_gripper_velocity_device = stod(redis_client.get(DEVICE_GRIPPER_VELOCITY_KEYS[0]));
	// Sensed force
	teleop_task->_sensed_force_device = redis_client.getEigenMatrixJSON(DEVICE_SENSED_FORCE_KEYS[0]);
	teleop_task->_sensed_torque_device = redis_client.getEigenMatrixJSON(DEVICE_SENSED_TORQUE_KEYS[0]);

	// Use the haptic device gripper as a switch and update the gripper state
	teleop_task->UseGripperAsSwitch();

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
        gripper_state = teleop_task->gripper_state;

		if( remote_enabled==1 && teleop_task->device_homed && gripper_state && (posori_task->_desired_position - posori_task->_current_position).norm() < 0.02 && posori_task->_orientation_error.norm()<0.01)
		{
			
			// Reinitialize controllers
			joint_task->reInitializeTask();
			joint_task->_desired_position = goal_posture;


		if (admittance_control)
		 {
	 		posori_task->_kp_pos = 15.0; //Integral term for the velocity controller
			posori_task->_kv_pos = 30.0; //Proportional gain for the velocity controller
			posori_task->_ki_pos = 0.0;
			posori_task->_kp_ori = 15.0;
			posori_task->_kv_ori = 30.0;
			posori_task->_ki_ori = 0.0;

		 }

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

		if (admittance_control) //// Admittance controller of the robot ////
		{
		// Force feedback from velocity PI in admittance-type scheme

		teleop_task->computeHapticCommandsAdmittance3d(desired_trans_velocity_robot);

		// teleop_task->computeHapticCommandsAdmittance6d(desired_trans_velocity_robot, desired_rot_velocity_robot);

		// cout << "desired trans velocity" << desired_trans_velocity_robot << endl;

		// Compute the desired position by integrating the velocity
		desired_position_robot = desired_trans_velocity_robot*dt + pos_rob_model;
		//desired_rotation_robot = desired_rot_velocity_robot*dt.................
		
		// set goal velocities
		posori_task->_desired_velocity = desired_trans_velocity_robot;
		// posori_task->_desired_angular_velocity = desired_rot_velocity_robot; // only if 6D feedback/control
		}
		else //// Impedance controller of the robot ////
		{
			//Compute haptic commands
		teleop_task->_haptic_feedback_from_proxy = true; // If set to true, the force feedback is computed from a stiffness/damping proxy.
	    teleop_task->_filter_on = true;

		// Force feedback from force sensor
		// teleop_task->updateSensedForce(f_task_sensed);
		// teleop_task->_send_haptic_feedback = true;

		//teleop_task->updateSensedForce(f_proxy_simulation); // BUG... - Computed through chai3D Finger Proxy algorithm
		
		
		// Force feedback from proxy - URDF: Collision should only be between the proxy and the environment
		teleop_task->_send_haptic_feedback = true;

		// cout << "force_des" << teleop_task->_commanded_force_device<< endl;

		// if (f_task_sensed.norm()>=0.000001)
		// {
		// 	teleop_task->_send_haptic_feedback = true;	
		//  }
		//  else
		//  {
		//  	teleop_task->_send_haptic_feedback = false;
		//  }

	    teleop_task->computeHapticCommands3d(desired_position_robot);
		// teleop_task->computeHapticCommands6d(desired_position_robot, desired_rotation_robot);
		// teleop_task->computeHapticCommandsWorkspaceExtension6d(desired_position_robot, desired_rotation_robot);
		// teleop_task->computeHapticCommandsWorkspaceExtension3d(desired_position_robot);
		
		//Send set position and velocity from haptic device to simulation for finger proxy calculation of chai3D
		// command_trans_velocity = teleop_task->_vel_trans_rob;
		// command_position = desired_position_robot;
		}
		
		// set goal position
		posori_task->_desired_position = desired_position_robot;
		// posori_task->_desired_orientation = desired_rotation_robot;// Only if 6D feedback/control

		// Set new commanded proxy position
		posori_task_proxy->_desired_position = desired_position_robot;
		//posori_task_proxy->_desired_orientation = desired_rotation_robot;// Only if 6D feedback/control

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

			posori_task->_kp_pos = 200.0;
			posori_task->_kv_pos = 15.0;
			posori_task->_ki_pos = 0.0;
			posori_task->_kp_ori = 200.0;
			posori_task->_kv_ori = 20.0;
			posori_task->_ki_ori = 0.0;
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
		gripper_state = teleop_task->gripper_state;

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
	
	//// Send the haptic commands to the device driver through Redis keys ////
	redis_client.setEigenMatrixJSON(DEVICE_COMMANDED_FORCE_KEYS[0], teleop_task->_commanded_force_device);
	redis_client.setEigenMatrixJSON(DEVICE_COMMANDED_TORQUE_KEYS[0], teleop_task->_commanded_torque_device);
	redis_client.set(DEVICE_COMMANDED_GRIPPER_FORCE_KEYS[0], to_string(teleop_task->_commanded_gripper_force_device));

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

	//// Send the haptic commands to the device driver through Redis keys ////
	redis_client.setEigenMatrixJSON(DEVICE_COMMANDED_FORCE_KEYS[0], Vector3d::Zero());
	redis_client.setEigenMatrixJSON(DEVICE_COMMANDED_TORQUE_KEYS[0], Vector3d::Zero());
	redis_client.set(DEVICE_COMMANDED_GRIPPER_FORCE_KEYS[0], "0.0");

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

