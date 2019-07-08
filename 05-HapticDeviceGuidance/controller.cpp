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
#include <chrono>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;
using namespace chai3d;

//// Define robots ////
 const string robot_file = "../resources/05-HapticDeviceGuidance/kuka_iiwa.urdf";
 const string robot_name = "Kuka-IIWA";

//////////////////////////////////////////////////////////////////////
// //Definition of the state machine for the robot controller
#define GOTO_INITIAL_CONFIG               0
#define HAPTIC_CONTROL                    1
#define MAINTAIN_POSITION				  2


#define DEBUG	6//
// int state_dev = DEBUG;

int state = GOTO_INITIAL_CONFIG;


unsigned long long controller_counter = 0;

const bool inertia_regularization = true;

int remote_enabled = 1;
int restart_cycle = 1;

// Create redis keys
const string REMOTE_ENABLED_KEY = "sai2::Sigma7Applications::sensors::remote_enabled";
const string RESTART_CYCLE_KEY = "sai2::Sigma7Applications::sensors::restart_cycle";

// const bool flag_simulation = false;
const bool flag_simulation = true;

string JOINT_ANGLES_KEY  = "sai2::Sigma7Applications::sensors::q";
string JOINT_VELOCITIES_KEY = "sai2::Sigma7Applications::sensors::dq";
string JOINT_TORQUES_COMMANDED_KEY  = "sai2::Sigma7Applications::actuators::torque_joint_robot";

string MASSMATRIX_KEY = "sai2::Sigma7Applications::sensors::model::massmatrix";
string CORIOLIS_KEY = "sai2::Sigma7Applications::sensors::model::coriolis";
string ROBOT_GRAVITY_KEY = "sai2::Sigma7Applications::sensors::model::robot_gravity";
string JOINT_TORQUES_SENSED_KEY = "sai2::Sigma7Applications::sensors::torques";	

string FORCE_SENSED_KEY = "sai2::Sigma7Applications::sensors::force_task_sensed";

const string GUIDANCE_PLANE_NORMAL_KEY = "sai2::Sigma7Applications::simulation::guidance_plane_normal_vec";
const string GUIDANCE_PLANE_POINT_KEY = "sai2::Sigma7Applications::simulation::guidance_plane_point";
const string ROBOT_HAPTIC_ROTATION_MATRIX_KEY = "sai2::Sigma7Applications::simulation::robot_haptic_rotation_matrix_guidance";
const string ROBOT_HAPTIC_FRAME_TRANSLATION_KEY = "sai2::Sigma7Applications::simulation::robot_haptic_frame_translation_guidance";
const string GUIDANCE_LINE_FIRST_POINT = "sai2::Sigma7Applications::simulation::line_guidance_first_point";
const string GUIDANCE_LINE_SECOND_POINT = "sai2::Sigma7Applications::simulation::line_guidance_second_point";
const string GUIDANCE_PLANE_ENABLE = "sai2::Sigma7Applications::simulation::_enable_plane_guidance_3D";
const string GUIDANCE_LINE_ENABLE = "sai2::Sigma7Applications::simulation::_enable_line_guidance_3D";
const string GUIDANCE_SCALE_FACTOR = "sai2::Sigma7Applications::simulation::_guidance_scale_factor";


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


	if (!flag_simulation)
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::actuators::fgc";
		JOINT_TORQUES_SENSED_KEY = "sai2::FrankaPanda::sensors::torques";

		JOINT_ANGLES_KEY  = "sai2::FrankaPanda::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::sensors::dq";
		MASSMATRIX_KEY = "sai2::FrankaPanda::sensors::model::massmatrix";
		CORIOLIS_KEY = "sai2::FrankaPanda::sensors::model::coriolis";
		ROBOT_GRAVITY_KEY = "sai2::FrankaPanda::sensors::model::robot_gravity";	
			
		FORCE_SENSED_KEY= "sai2::optoforceSensor::6Dsensor::force";	
	}

	/////////////////////////////// init ////////////////////////////////////////
 	
	remote_enabled=1;


	Eigen::Affine3d robot_pose_in_world = Affine3d::Identity();
	robot_pose_in_world.translation() = Vector3d(0.0, 0.0, 0.0);
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
	const Vector3d pos_in_link = Vector3d(0.0,0.0,0.04); /////////////////////////////////Define center of tool in end-effector frame
	auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	VectorXd posori_task_torques = VectorXd::Zero(robot->dof());

	posori_task->_kp_pos = 50.0;
	posori_task->_kv_pos = 14.0;
	posori_task->_kp_ori = 50.0;
	posori_task->_kv_ori = 14.0;
	
	// position of robot in world
	Eigen::Vector3d centerPos_rob;
	Eigen::Matrix3d centerRot_rob;
	
	centerPos_rob << -0.2, 0.0, 0.5;
	centerRot_rob << -1.0, 0.0, 0.0, 
					  0.0, 1.0, 0.0, 
					  0.0, 0.0, -1.0;

	posori_task->_desired_position = centerPos_rob;
	posori_task->_desired_orientation = centerRot_rob;

	auto handler = new cHapticDeviceHandler();

	Eigen::Matrix3d transformDev_Rob = robot_pose_in_world.linear();
	int device_number=0;
	auto teleop_task = new Sai2Primitives::OpenLoopTeleop(handler, device_number, centerPos_rob, centerRot_rob, transformDev_Rob);

	 //Task scaling factors
	double Ks=2.0;
	double KsR=1.0;
	teleop_task->setScalingFactors(Ks, KsR);
	redis_client.set(GUIDANCE_SCALE_FACTOR, to_string(Ks));

	// Center of the haptic device workspace
	Vector3d HomePos_op;
	HomePos_op << 0.0, 0.01, 0.0;
	Matrix3d HomeRot_op;
	HomeRot_op.setIdentity();
	teleop_task->setDeviceCenter(HomePos_op, HomeRot_op);
	// Force feedback stiffness proxy parameters
	double proxy_position_impedance = 2000.0;
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
	
	// write task force in redis
	redis_client.setEigenMatrixJSON(FORCE_SENSED_KEY,f_task_sensed);


	teleop_task->_haptic_feedback_from_proxy = false;
	teleop_task->_filter_on = false;

	// Initialize haptic device if Sigma.7
	teleop_task->initializeSigmaDevice();

	//Enable user switch
	teleop_task->EnableGripperUserSwitch();
	bool gripper_state=false;
	bool button_change = false;
	double button_debounce_counter = 1;
	auto debounce_start_time = chrono::high_resolution_clock::now();

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); //Compiler en mode release
	double current_time = 0;
	double prev_time = 0;
	double dt = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs


	
	// ************************************* Plane Guidance ************************************* //
	// Set up the plane for guidance using an origin point (world origin) and a normal vector
	teleop_task->_enable_plane_guidance_3D = true;
	Eigen::Vector3d guidance_point;
	Eigen::Vector3d guidance_normal_vec;
	Eigen::Matrix3d robot_haptic_rotation_matrix;
	Eigen::Vector3d robot_haptic_frame_translation;
/////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////





	guidance_point = HomePos_op;
	guidance_normal_vec << 0, 1, 1;
	robot_haptic_rotation_matrix = HomeRot_op;
	robot_haptic_frame_translation = centerPos_rob - HomePos_op;

	teleop_task->setDeviceRobotRotation(robot_haptic_rotation_matrix);
	teleop_task->setPlane(guidance_point, guidance_normal_vec);

	// Gains for haptic guidance
	double guidance_stiffness = 0.7;
	double guidance_damping = 1.0;
	teleop_task->setHapticGuidanceGains(guidance_stiffness, guidance_damping);
	
	redis_client.setEigenMatrixJSON(GUIDANCE_PLANE_POINT_KEY, guidance_point);
	redis_client.setEigenMatrixJSON(GUIDANCE_PLANE_NORMAL_KEY, guidance_normal_vec);
	redis_client.setEigenMatrixJSON(ROBOT_HAPTIC_ROTATION_MATRIX_KEY, robot_haptic_rotation_matrix);
	redis_client.setEigenMatrixJSON(ROBOT_HAPTIC_FRAME_TRANSLATION_KEY, robot_haptic_frame_translation);
	redis_client.set(GUIDANCE_PLANE_ENABLE, to_string(teleop_task->_enable_plane_guidance_3D));

	// ************************************* Plane Guidance ************************************* //


	// ************************************* Line Guidance ************************************* //
	// Set up the plane for guidance using an origin point (world origin) and a normal vector
	teleop_task->_enable_line_guidance_3D = false;
	Eigen::Vector3d _first_point;
	Eigen::Vector3d _second_point;

	_first_point = HomePos_op;
	_second_point << HomePos_op(0), 2 + HomePos_op(1), 5 + HomePos_op(2);
	
	teleop_task->setLine(_first_point, _second_point);
	
	redis_client.setEigenMatrixJSON(GUIDANCE_LINE_FIRST_POINT, _first_point);
	redis_client.setEigenMatrixJSON(GUIDANCE_LINE_SECOND_POINT, _second_point);
	redis_client.set(GUIDANCE_LINE_ENABLE, to_string(teleop_task->_enable_line_guidance_3D));

	// ************************************* Line Guidance ************************************* //

	// ************************************* Switching Guidance *************************************//
	bool isPressed = false;
	// *********************************** End Switching Guidance ***********************************//



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

		// // Update position and orientation of the robot from the model
		// robot->position(pos_rob_model, link_name, pos_in_link);
		// robot->linearVelocity(vel_trans_rob_model, link_name, pos_in_link);
		// robot->rotation(rot_rob_model, link_name);
		// robot->angularVelocity(vel_rot_rob_model, link_name);


		//Compute haptic commands
		teleop_task->_haptic_feedback_from_proxy = false; // If set to true, the force feedback is computed from a stiffness/damping proxy.
	    teleop_task->_filter_on = true;
		double fc_force=0.02;
		double fc_moment=0.02;
		teleop_task->setFilterCutOffFreq(fc_force, fc_moment);

		teleop_task->updateSensedForce(f_task_sensed);
		teleop_task->_send_haptic_feedback = true;
		
		teleop_task->computeHapticCommands3d(pos_rob);

		//// Impedance controller of the robot ////
		// set goal position
		posori_task->_desired_position = pos_rob;
		// don't need to redefine the desired orientation it was already done in the initialization
		//posori_task->_desired_orientation = centerRot_rob;
		
		// update model and priority
		N_prec.setIdentity();
		posori_task->updateTaskModel(N_prec);
		N_prec = posori_task->_N;
		joint_task->updateTaskModel(N_prec);
		// compute torques
		posori_task->computeTorques(posori_task_torques);
		joint_task->computeTorques(joint_task_torques);

		command_torques = posori_task_torques + joint_task_torques;


		// grab the current gripper state
		gripper_state = teleop_task->ReadGripperUserSwitch();
		isPressed = false;

		// button debounce code to change guidance type
		if (gripper_state == true && button_change == false) //if button pushed and no change recorded yet
		{
			// if button pushed and entering this code for the first time
			if (button_debounce_counter == 1)
			{
				// a change is detected
				button_change = true;
			} 

		}

		if (gripper_state == false && button_change == true) 
		{
			isPressed = true;
			button_change = false;
		}

		if(remote_enabled == 0)
		{
			// joint controller to maintin robot in current position
			joint_task->reInitializeTask();
			joint_task->_desired_position = robot->_q;

			// set current haptic device position
			teleop_task->setDeviceCenter(teleop_task->_current_position_device, teleop_task->_current_rotation_device);


			state = MAINTAIN_POSITION;

		// if the gripper state is true, change the guidance type (i.e., if the button was pushed)
		} else if(isPressed) {
			teleop_task->_enable_plane_guidance_3D = !teleop_task->_enable_plane_guidance_3D;
			teleop_task->_enable_line_guidance_3D = !teleop_task->_enable_line_guidance_3D;

			redis_client.set(GUIDANCE_PLANE_ENABLE, to_string(teleop_task->_enable_plane_guidance_3D));
			redis_client.set(GUIDANCE_LINE_ENABLE, to_string(teleop_task->_enable_line_guidance_3D));
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

			centerPos_rob << -0.2, 0.0, 0.5;
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

	// calculate new values for feedback and visualization
	if (isPressed) {
		_first_point = teleop_task->_current_position_device;
		_second_point << _first_point(0), 2 + _first_point(1), 5 + _first_point(2);
		guidance_point = _first_point;

		// set new line and plane values
		teleop_task->setPlane(guidance_point, guidance_normal_vec);
		teleop_task->setLine(_first_point, _second_point);
	}
	//// Send to commands redis ////
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.setEigenMatrixJSON(ROBOT_HAPTIC_ROTATION_MATRIX_KEY, robot_haptic_rotation_matrix);
	redis_client.setEigenMatrixJSON(ROBOT_HAPTIC_FRAME_TRANSLATION_KEY, robot_haptic_frame_translation);
	redis_client.setEigenMatrixJSON(GUIDANCE_LINE_FIRST_POINT, _first_point);
	redis_client.setEigenMatrixJSON(GUIDANCE_LINE_SECOND_POINT, _second_point);
	redis_client.setEigenMatrixJSON(GUIDANCE_PLANE_POINT_KEY, guidance_point);
	redis_client.setEigenMatrixJSON(GUIDANCE_PLANE_NORMAL_KEY, guidance_normal_vec);

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
