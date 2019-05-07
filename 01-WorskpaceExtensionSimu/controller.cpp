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
#include "tasks/HapticWorkspaceExtension.h"

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
// a haptic device handler
cHapticDeviceHandler* handler;
// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;
bool device_started = false;

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
string FORCE_TASK_KEY;
string REMOTE_ENABLED_KEY;

// - write
string JOINT_TORQUES_COMMANDED_KEY;
string DES_DEVICE_FORCE_KEY;

// - model (for implementation with real robot)
string MASSMATRIX_KEY;
string CORIOLIS_KEY;
string ROBOT_GRAVITY_KEY;

// gains position controllers
const string KP_JOINT_KEY = "sai2::Sigma7Applications::controller:kp_joint";
const string KV_JOINT_KEY = "sai2::Sigma7Applications::controller:kv_joint";
const string KP_POS_KEY = "sai2::Sigma7Applications::controller:kp_pos";
const string KV_POS_KEY = "sai2::Sigma7Applications::controller:kv_pos";
const string KP_ORI_KEY = "sai2::Sigma7Applications::controller:kp_ori";
const string KV_ORI_KEY = "sai2::Sigma7Applications::controller:kv_ori";

// Drift factor HapticWorkspaceExtension controller
const string DRIFT_PERC_FORCE_KEY = "sai2::Sigma7Applications::controller:Fdrift_perc";


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


	// create a haptic device handler
    auto handler = new cHapticDeviceHandler();

	// get a handle to the first haptic device
	handler->getDevice(hapticDevice, 0);
	if (NULL == hapticDevice) {
		cout << "No haptic device found. " << endl;
		device_started = false;
	} else {
		hapticDevice->open();
		hapticDevice->calibrate();
		device_started = true;
	}
	
	// retrieve information about the current haptic device
	cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();
	// if the device has a gripper, then enable it to behave like a user switch
	hapticDevice->setEnableGripperUserSwitch(true);
	bool gripper_state = false;

	// get properties of haptic device 
	double maxForce_dev = hapticDeviceInfo.m_maxLinearForce;
	double maxTorque_dev = hapticDeviceInfo.m_maxAngularTorque;
	double maxLinDamping_dev = hapticDeviceInfo.m_maxLinearDamping;
	double maxAngDamping_dev = hapticDeviceInfo.m_maxAngularDamping;
	double maxLinStiffness_dev = hapticDeviceInfo.m_maxLinearStiffness;
	double maxAngStiffness_dev = hapticDeviceInfo.m_maxAngularStiffness;


	/////////////////////////////// init ////////////////////////////////////////
 	
	int remote_enabled=0;

	// Create redis keys
	DES_DEVICE_FORCE_KEY = "sai2::Sigma7Applications::actuators::force_op_dev";
	FORCE_TASK_KEY = "sai2::Sigma7Applications::sensors::force_task";
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
	goal_posture << 0.0, -30.0, 0.0, 60.0, 0.0, -90.0, 0.0; /////////////////////////// 
	goal_posture *= M_PI/180.0;
	joint_task->_desired_position = goal_posture;

	//// PosOriTask controller (manage the control in position of the iiwa robot) ////
	const string link_name = "link6";
	const Vector3d pos_in_link = Vector3d(0.0,0.0,0.04); /////////////////////////////////Define center of tool in end-effector frame
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
	centerPos_rob << -0.5, 0.0, 0.6;
	Matrix3d centerRot_rob;
	//centerRot_rob.setIdentity();
	centerRot_rob << -1.0, 0.0, 0.0,
					  0.0, 1.0, 0.0,
            		  0.0, 0.0, -1.0;

	posori_task->_desired_position = centerPos_rob;
	posori_task->_desired_orientation = centerRot_rob;


	//// HapticWorkspaceExtension controller (compute the mapping between the haptic device and the task workspaces) ////
	double Rmax_dev = 0.025; // in [mm]
	double Rmax_env = 0.2;
	double Thetamax_dev = 20*M_PI/180.0; // in [rad]
	double Thetamax_env = 45*M_PI/180.0; ///////////////////////////////////////////////////
	auto workspace_extension = new Sai2Primitives::HapticWorkspaceExtension(Rmax_dev, Rmax_env, Thetamax_dev, Thetamax_env);
	double Fdrift_perc = 0.0/100.0;
	redis_client.set(DRIFT_PERC_FORCE_KEY, to_string(Fdrift_perc));
	workspace_extension->setForceNoticeableDiff(Fdrift_perc); // Percentage of drift force in the haptic feedback
	Vector3d HomePos_op;
	HomePos_op << 0.0, 0.01, 0.0; //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	Matrix3d HomeRot_op;
	HomeRot_op.setIdentity();
	workspace_extension->setDeviceCenter(HomePos_op, HomeRot_op); // Center of the haptic device workspace

	VectorXd f_hapt = VectorXd::Zero(6);
	VectorXd f_set_dev = VectorXd::Zero(6);
	Vector3d pos_rob = Vector3d::Zero(); // Set position and orientation in robot frame
	Matrix3d rot_rob = Matrix3d::Identity();
	Vector3d pos_rob_model = Vector3d::Zero(); // Model position and orientation in robot frame
	Matrix3d rot_rob_model = Matrix3d::Identity();
	Vector3d pos_rob_dev = Vector3d::Zero(); // Set position and orientation in device frame
	Matrix3d rot_rob_dev = Matrix3d::Identity();
	VectorXd f_task = VectorXd::Zero(6); // Force task in the robot frame
	Vector3d f_task_trans = Vector3d::Zero();
	Vector3d f_task_rot = Vector3d::Zero();
	VectorXd f_task_dev = VectorXd::Zero(6); //Force task in the device frame
	VectorXd f_task_sensed = VectorXd::Zero(6); //Force task sensed through the force sensor
	Vector3d pos_dev = Vector3d::Zero(); 
	Matrix3d rot_dev = Matrix3d::Identity();
	VectorXd vel_dev = VectorXd::Zero(6);
	VectorXd vel_dev_trans = Vector3d::Zero();
	VectorXd vel_dev_rot = Vector3d::Zero();
	cVector3d	pos_dev_chai;
	cMatrix3d 	rot_dev_chai;
	cVector3d	vel_dev_trans_chai;
	cVector3d	vel_dev_rot_chai;
	MatrixXd massmatrix_dev_hum = MatrixXd::Identity(6,6); //Lambda haptice device and human user (in cartesian space)
	Vector3d orientation_error = Vector3d::Zero();
	Vector3d integrated_position_error = Vector3d::Zero();
	Vector3d integrated_orientation_error = Vector3d::Zero();
	Vector3d force_dev = Vector3d::Zero();
	Vector3d torque_dev = Vector3d::Zero();
	cVector3d force_chai;
	force_chai.set(0.0,0.0,0.0);
	cVector3d torque_chai;
	torque_chai.set(0.0,0.0,0.0);
	Vector3d orientation = Vector3d::Zero();
	Vector3d force_virtual = Vector3d::Zero();
	Vector3d torque_virtual = Vector3d::Zero();

	// Haptice device position controller gains
	double _kp_pos =0.1 * maxLinStiffness_dev;
	double _kv_pos =0.4 * maxLinDamping_dev;
	double _ki_pos =0.0;
	double _kp_ori =0.3 * maxAngStiffness_dev;
	double _kv_ori =0.1 * maxAngDamping_dev;
	double _ki_ori =0.0;

	// Task force calculation through stiffness proxy
	double k_pos = 200.0;
	double k_ori = 15.0;
	Vector3d orientation_dev = Vector3d::Zero();
	MatrixXd scaling_factor = MatrixXd::Identity(6,6);
	VectorXd f_task_red = VectorXd::Zero(6);


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

	// read haptic device info
	hapticDevice->getPosition(pos_dev_chai);
	hapticDevice->getRotation(rot_dev_chai);
	pos_dev = convertChaiToEigenVector(pos_dev_chai);
	rot_dev = convertChaiToEigenMatrix(rot_dev_chai);
	hapticDevice->getLinearVelocity(vel_dev_trans_chai);
	hapticDevice->getAngularVelocity(vel_dev_rot_chai);
	vel_dev_trans = convertChaiToEigenVector(vel_dev_trans_chai);
	vel_dev_rot = convertChaiToEigenVector(vel_dev_rot_chai);
	vel_dev << vel_dev_trans,vel_dev_rot;


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
			workspace_extension->reInitializeTask();
			robot->position(pos_rob, link_name, pos_in_link);
			robot->rotation(rot_rob, link_name);

			state = REMOTE_CONTROL;
		}
	}


	else if(state == REMOTE_CONTROL)
	{	
		// Update position and orientation of the robot from the model
		robot->position(pos_rob_model, link_name, pos_in_link);
		robot->rotation(rot_rob_model, link_name);

		//Evaluate the task force through stiffness proxy
		f_task_trans = k_pos*(pos_rob_model - pos_rob);

		// Compute the orientation error
		Sai2Model::orientationError(orientation_dev, rot_rob, rot_rob_model);
		// Evaluate task torque
		f_task_rot = k_ori*orientation_dev;
		
		// f_task = f_task_sensed;

		if (f_task_sensed.norm() <= 0.0001)
		{
			f_task = f_task_sensed;
		}
		else
		{
			f_task << f_task_trans, f_task_rot;
		}

		// Reduce sensed task force before sending it to the haptic device
		scaling_factor << 1/2.0, 0.0, 0.0, 0.0, 0.0, 0.0,
						  0.0, 1/2.0, 0.0, 0.0, 0.0, 0.0, 
						  0.0, 0.0, 1/2.0, 0.0, 0.0, 0.0,
						  0.0, 0.0, 0.0, 1/20.0, 0.0, 0.0, 
						  0.0, 0.0, 0.0, 0.0, 1/20.0, 0.0,
						  0.0, 0.0, 0.0, 0.0, 0.0, 1/20.0;
		f_task_red = scaling_factor*f_task;

		//Transfer task force from robot to haptic device global frame
		// f_task_dev << f_task[0], f_task[1], f_task[2], f_task[3], f_task[4], f_task[5];
		f_task_dev = f_task_red;

		//// Evaluation of the device haptic feedback and the set position of the controlled robot after workspace extension ////
		workspace_extension->computeHapticCommands(f_hapt, pos_rob_dev, rot_rob_dev, pos_dev, rot_dev, vel_dev, f_task_dev, massmatrix_dev_hum);

		//Transfer set position and orientation from device to robot global frame
		pos_rob = pos_rob_dev;
		rot_rob = rot_rob_dev;
		// Adjust set position and orientation to the center of the task workspace
		pos_rob = pos_rob + centerPos_rob;
		rot_rob = rot_rob * centerRot_rob;

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
			f_hapt.setZero();
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
			
			workspace_extension->reInitializeTask();
			
			state = MOVE_TO_INITIAL;
		}

	}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//// Run Haptic Device controller state machine ////
	if(state_dev == MOVE_TO_HOME)
	{
		// update integrated error for I term
		integrated_position_error += (pos_dev - HomePos_op) * _t_diff;
		// Evaluate position controller force
		force_dev = -_kp_pos*(pos_dev - HomePos_op) - _kv_pos * vel_dev_trans - _ki_pos * integrated_position_error;
		// Compute the orientation error
		Sai2Model::orientationError(orientation_error, HomeRot_op, rot_dev);
		// update integrated error for I term
		integrated_orientation_error += orientation_error * _t_diff;
		// Evaluate orientation controller force
		torque_dev = -_kp_ori*orientation_error - _kv_ori * vel_dev_rot - _ki_ori*integrated_orientation_error; 

		if( (pos_dev - HomePos_op).norm()<0.002 && orientation_error.norm() < 0.04)
		{
			
			cout << "Waiting for robot ready" << endl;
			state_dev = WAIT_ROBOT_READY;
		}
	}

	// if(state_dev == DEBUG)
	//  {
	//  	force_dev.setZero();
	// 	torque_dev.setZero();

	// 	cout << _kp_pos << endl;
	// 	cout << _kv_pos << endl;
	// 	cout << _kp_ori << endl;
	// 	cout << _kv_ori << endl;

	//  }

	else if(state_dev == WAIT_ROBOT_READY)
	{
		// update integrated error for I term
		integrated_position_error += (pos_dev - HomePos_op) * _t_diff;
		// Evaluate position controller force
		force_dev = -_kp_pos*(pos_dev - HomePos_op) - _kv_pos * vel_dev_trans - _ki_pos * integrated_position_error;
		// Compute the orientation error
		Sai2Model::orientationError(orientation_error, HomeRot_op, rot_dev);
		// update integrated error for I term
		integrated_orientation_error += orientation_error * _t_diff;
		// Evaluate orientation controller force
		torque_dev = -_kp_ori*orientation_error - _kv_ori * vel_dev_rot - _ki_ori*integrated_orientation_error;

        if( robot_ready == true)
		{
			cout << "Waiting for user input" << endl;
			state_dev = WAIT_USER_INPUT;
		}
	}

	else if(state_dev == WAIT_USER_INPUT)
	{
		// update integrated error for I term
		integrated_position_error += (pos_dev - HomePos_op) * _t_diff;
		// Evaluate position controller force
		force_dev = -_kp_pos*(pos_dev - HomePos_op) - _kv_pos * vel_dev_trans - _ki_pos * integrated_position_error;
		// Compute the orientation error
		Sai2Model::orientationError(orientation_error, HomeRot_op, rot_dev);
		// update integrated error for I term
		integrated_orientation_error += orientation_error * _t_diff;
		// Evaluate orientation controller force
		torque_dev = -_kp_ori*orientation_error - _kv_ori * vel_dev_rot - _ki_ori*integrated_orientation_error;
        // Read new gripper state
        hapticDevice->getUserSwitch (0, gripper_state);
        
		if(gripper_state && remote_enabled == 1)
		{
			force_dev.setZero();
			torque_dev.setZero();
			f_hapt.setZero();
			device_ready = true;
			state_dev = REMOTE_CONTROL;
		}
	}

	else if(state_dev == REMOTE_CONTROL)
	{	
		
		// Cartesian set force for haptic feedback and drift
		force_dev = f_hapt.head(3);
		torque_dev = f_hapt.tail(3);
		// force_dev.setZero();
		// torque_dev.setZero();

		cout << f_hapt.transpose() << endl;

		//// Add virtual forces according to the operational space limits ////
		if ((pos_dev - HomePos_op).norm() >= Rmax_dev)
		{
			force_virtual = -(0.8 * maxLinStiffness_dev * ((pos_dev - HomePos_op).norm()-Rmax_dev)/((pos_dev - HomePos_op).norm()))*(pos_dev - HomePos_op);
		}
		force_dev += force_virtual;
		
		Sai2Model::orientationError(orientation, HomeRot_op, rot_dev); // Compute the orientation
		if (orientation.norm() >= Thetamax_dev)
		{
			torque_virtual = -(0.8 * maxAngStiffness_dev *(orientation.norm()-Thetamax_dev)/(orientation.norm()))*orientation;
		}
		torque_dev += torque_virtual;


		if(remote_enabled == 0)
		{
			device_ready = false;
			force_dev.setZero();
			torque_dev.setZero();
			state_dev = END_REMOTE_TASK;
		}

	}

	else if(state_dev == END_REMOTE_TASK)
	{
		force_dev.setZero();
		torque_dev.setZero();

		if(remote_enabled == 1)
		{
			// Reinitialize controllers
			integrated_position_error.setZero();
			integrated_orientation_error.setZero();
			force_dev.setZero();
			torque_dev.setZero();

			state_dev = MOVE_TO_HOME;
		}
		
	}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//// Saturate to Force and Torque limits of the haptic device ////
	if (force_dev.norm() >= maxForce_dev)
	{
		force_dev = maxForce_dev*force_dev/(force_dev.norm());
	}
	if (torque_dev.norm() >= maxTorque_dev)
	{
		torque_dev = maxTorque_dev*torque_dev/(torque_dev.norm());
	}

	//// Send controllers force and torque to haptic device ////
	force_chai = convertEigenToChaiVector(force_dev);
	torque_chai = convertEigenToChaiVector(torque_dev);
    hapticDevice->setForceAndTorque(force_chai,torque_chai);

	//// Send to commands redis ////
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	f_set_dev << force_dev , torque_dev;
	redis_client.setEigenMatrixJSON(DES_DEVICE_FORCE_KEY, f_set_dev);

	redis_client.setEigenMatrixJSON(FORCE_TASK_KEY, f_task);

	controller_counter++;
	}


	force_chai.set(0.0,0.0,0.0);
	torque_chai.set(0.0,0.0,0.0);
	hapticDevice->setForceAndTorque(force_chai,torque_chai);	
	hapticDevice->close();

	command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}
