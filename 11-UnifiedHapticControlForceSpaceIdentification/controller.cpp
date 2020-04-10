// This application applies the haptic unified controller with a closed-loop surface-Surface alignment done
// autonomously (Zero moment - autonomous_aligment=true) or driven through the haptic torque commands and a
// normal closed-loop force control from the desired haptic force. It simulates the interaction with an
// inclined plane or a sphere (change in world file).

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

// Include task primitives from 'sai2-primitives'
#include "tasks/JointTask.h"
#include "tasks/PosOriTask.h"
#include "haptic_tasks/HapticController.h"
#include "haptic_tasks/BilateralPassivityController.h"

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

//// Define robots ////
 const string robot_file = "../resources/11-UnifiedHapticControlForceSpaceIdentification/panda_arm.urdf";
 const string robot_name = "panda";

//////////////////////////////////////////////////////////////////////
// //Definition of the state machine for the robot controller
#define GOTO_INITIAL_CONFIG           0
#define IMPEDANCE_CONTROL             1
#define UNIFIED_CONTROL					      2
#define MAINTAIN_POSITION				      3

int state = GOTO_INITIAL_CONFIG;

unsigned long long controller_counter = 0;

const bool inertia_regularization = true;

// const bool flag_simulation = true;
const bool flag_simulation = true;

const bool using_keyboard_input = true;

//const bool autonomous_aligment = false;

int remote_enabled = 1;
int restart_cycle = 0;

int switch_state_counter = 500;

// Create redis keys
const string REMOTE_ENABLED_KEY = "sai2::Sigma7Applications::sensors::remote_enabled";
const string RESTART_CYCLE_KEY = "sai2::Sigma7Applications::sensors::restart_cycle";

// Robot related keys
string JOINT_ANGLES_KEY  = "sai2::Sigma7Applications::sensors::q";
string JOINT_VELOCITIES_KEY = "sai2::Sigma7Applications::sensors::dq";
string JOINT_TORQUES_COMMANDED_KEY  = "sai2::Sigma7Applications::actuators::torque_joint_robot";

string MASSMATRIX_KEY = "sai2::Sigma7Applications::sensors::model::massmatrix";
string CORIOLIS_KEY = "sai2::Sigma7Applications::sensors::model::coriolis";
string ROBOT_GRAVITY_KEY = "sai2::Sigma7Applications::sensors::model::robot_gravity";
string JOINT_TORQUES_SENSED_KEY = "sai2::Sigma7Applications::sensors::torques";

string FORCE_SENSED_KEY = "sai2::Sigma7Applications::sensors::force_task_sensed";

string MOUSE_POSITION_KEY = "sai2::Sigma7Applications::sensors::current_mouse_position";
string MOUSE_GRIPPER_KEY = "sai2::Sigma7Applications::sensors::current_mouse_gripper";
int mouse_gripper = 0;
string CONTROLLED_CONTACT_DIRECTION_1 = "sai2::Sigma7Applications::sensors::first_direction_controlled_contact_force";
string CONTROLLED_CONTACT_DIRECTION_2 = "sai2::Sigma7Applications::sensors::second_direction_controlled_contact_force";
string CONTROLLED_CONTACT_DIRECTION_3 = "sai2::Sigma7Applications::sensors::third_direction_controlled_contact_force";


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


// redis keys for logging
// const string LOGGING_TIME_KEY = "sai2::Sigma7Applications::logging::time";
//
// const string LOGGING_ROBOT_JOINT_ANGLES = "sai2::Sigma7Applications::logging::q";
// const string LOGGING_ROBOT_JOINT_VELOCITIES = "sai2::Sigma7Applications::logging::dq";
// const string LOGGING_ROBOT_COMMAND_TORQUES = "sai2::Sigma7Applications::logging::fgc";
// const string LOGGING_ROBOT_DESIRED_POSITION = "sai2::Sigma7Applications::logging::x_desired";
// const string LOGGING_ROBOT_DESIRED_ORIENTATION = "sai2::Sigma7Applications::logging::R_desired";
// const string LOGGING_ROBOT_DESIRED_FORCE = "sai2::Sigma7Applications::logging::f_desired";
// const string LOGGING_ROBOT_DESIRED_MOMENT = "sai2::Sigma7Applications::logging::m_desired";
// const string LOGGING_ROBOT_CURRENT_POSITION = "sai2::Sigma7Applications::logging::x_current";
// const string LOGGING_ROBOT_CURRENT_VELOCITY = "sai2::Sigma7Applications::logging::dx_current";
// const string LOGGING_ROBOT_CURRENT_ORIENTATION = "sai2::Sigma7Applications::logging::R_current";
// const string LOGGING_ROBOT_CURRENT_ANGVEL = "sai2::Sigma7Applications::logging::w_current";
// const string LOGGING_ROBOT_TASK_FORCE = "sai2::Sigma7Applications::logging::posori_task_force";
//
// const string LOGGING_HAPTIC_POSITION = "sai2:Sigma7Applications::logging::haptic_position";
// const string LOGGING_HAPTIC_VELOCITY = "sai2:Sigma7Applications::logging::haptic_velocity";
// const string LOGGING_HAPTIC_ORIENTATION = "sai2:Sigma7Applications::logging::haptic_orientation";
// const string LOGGING_HAPTIC_ANGVEL = "sai2:Sigma7Applications::logging::haptic_angular_velocity";
// const string LOGGING_HAPTIC_COMMAND_FORCE = "sai2:Sigma7Applications::logging::haptic_command_force";
// const string LOGGING_HAPTIC_COMMAND_TORQUE = "sai2:Sigma7Applications::logging::haptic_command_torque";
// const string LOGGING_HAPTIC_COMMAND_FORCE_TOTAL = "sai2:Sigma7Applications::logging::haptic_command_force_plus_passivity";
// const string LOGGING_HAPTIC_COMMAND_TORQUE_TOTAL = "sai2:Sigma7Applications::logging::haptic_command_torque_plus_passivity";
//
// const string LOGGING_R_ROBOT_SENSOR = "sai2::Sigma7Applications::logging::R_robot_sensor";
// const string LOGGING_R_HAPTIC_ROBOT = "sai2::Sigma7Applications::logging::R_robot_haptic";
//
// const string LOGGING_SENSED_FORCE_ROBOT_FRAME = "sai2::Sigma7Applications::logging::force_sensed_robot_frame";
// const string LOGGING_SENSED_MOMENT_ROBOT_FRAME = "sai2::Sigma7Applications::logging::moment_sensed_robot_frame";
//
// const string LOGGING_BILATERAL_PASSIVITY_ALPHA_FORCE = "sai2::Sigma7Applications::logging::bilateral_passivity_alpha_force";
// const string LOGGING_BILATERAL_PASSIVITY_ALPHA_MOMENT = "sai2::Sigma7Applications::logging::bilateral_passivity_alpha_moment";
// const string LOGGING_PASSIVITY_RC_FORCE = "sai2::Sigma7Applications::logging::autonomous_passivity_Rc_force";
// const string LOGGING_PASSIVITY_RC_MOMENT = "sai2::Sigma7Applications::logging::autonomous_passivity_Rc_moment";
//
// const string LOGGING_VIRTUAL_FORCE_ROBOT_FRAME = "sai2::Sigma7Applications::logging::force_virtual_robot_frame";
// const string LOGGING_VIRTUAL_MOMENT_ROBOT_FRAME = "sai2::Sigma7Applications::logging::moment_virtual_robot_frame";


/////////////////////////////////////////////////////////////////////////////////

int main() {

	if (!flag_simulation)
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::Clyde::actuators::fgc";
		JOINT_TORQUES_SENSED_KEY = "sai2::FrankaPanda::Clyde::sensors::torques";

		JOINT_ANGLES_KEY  = "sai2::FrankaPanda::Clyde::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Clyde::sensors::dq";
		MASSMATRIX_KEY = "sai2::FrankaPanda::Clyde::sensors::model::massmatrix";
		CORIOLIS_KEY = "sai2::FrankaPanda::Clyde::sensors::model::coriolis";
		ROBOT_GRAVITY_KEY = "sai2::FrankaPanda::Clyde::sensors::model::robot_gravity";

		FORCE_SENSED_KEY= "sai2::ATIGamma_Sensor::force_torque";
	}

	/////////////////////////////// init ////////////////////////////////////////
	Eigen::Affine3d robot_pose_in_world = Affine3d::Identity();
	// robot_pose_in_world.translation() = Vector3d(0, -0.5, 0.0);
	//robot_pose_in_world.linear() = Matrix3d::Identity ();
	// robot_pose_in_world.linear() = AngleAxisd(0.3010693, Vector3d::UnitZ()).toRotationMatrix();
	Matrix3d R_tool = Matrix3d::Identity();

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

	//Create commanded, coriolis and gravity torque vector for the robot
	VectorXd command_torques = VectorXd::Zero(robot->dof());
	VectorXd coriolis_torques = VectorXd::Zero(robot->dof());
	VectorXd gravity_torques = VectorXd::Zero(robot->dof());
	VectorXd torques_sensed = VectorXd::Zero(robot->dof());

	//// Joint Task controller (manage the iiwa robot nullspace - posture) ////
	auto joint_task = new Sai2Primitives::JointTask(robot);
	MatrixXd N_prec = MatrixXd::Identity(robot->dof(), robot->dof());
	VectorXd joint_task_torques = VectorXd::Zero(robot->dof());
	joint_task->_kp = 250.0;
	joint_task->_kv = 18.0;
	joint_task->_ki = 35.0;

	// Define goal position according to the desired posture /////////////////////////////////////////////////////////////////////////////////////////////////
	VectorXd goal_posture(robot->dof());
	goal_posture << 0.0,-0.618209,0.0,-2.27868,0.0,1.65988,0.0;
	//goal_posture = joint_task->_current_position;
	joint_task->_desired_position = goal_posture;

	// Velocity saturation and/or interpolation for real robot application
	joint_task->_use_velocity_saturation_flag = false;
	joint_task->_use_interpolation_flag = true;
	joint_task->_otg->setMaxVelocity(M_PI/3);

	//// PosOriTask controller (manage the control in position of the iiwa robot) ////
	const string link_name = "link7";
	const Vector3d pos_in_link = Vector3d(0.0,0.0,0.147); /////////////////////////////////Define control point on tool in end-effector frame
	auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	VectorXd posori_task_torques = VectorXd::Zero(robot->dof());

	posori_task->_use_interpolation_flag = false;
	posori_task->_use_velocity_saturation_flag = false;
	// posori_task->_linear_saturation_velocity = 0.5;
	// posori_task->_angular_saturation_velocity = M_PI/1.5;

	posori_task->_kp_pos = 150.0;
	posori_task->_kv_pos = 16.0;
	posori_task->_kp_ori = 500.0;
	posori_task->_kv_ori = 30.0;
	posori_task->_ki_pos = 0.0;
	posori_task->_ki_ori = 0.0;

	posori_task->_kp_force = 0.5;
	posori_task->_kv_force = 5.0;
	posori_task->_ki_force = 0.4;
	posori_task->_kp_moment = 0.5;
	posori_task->_kv_moment = 10.0;
	posori_task->_ki_moment = 1.5;

	// position of robot in world
	Eigen::Vector3d centerPos_rob = posori_task->_current_position;
	Eigen::Matrix3d centerRot_rob = posori_task->_current_orientation;
	// centerPos_rob << -0.2, 0.0, 0.5;
	// centerRot_rob << -1.0, 0.0, 0.0,
	// 				  0.0, 1.0, 0.0,
	// 				  0.0, 0.0, -1.0;
	// posori_task->_desired_position = centerPos_rob;
	// posori_task->_desired_orientation = centerRot_rob;

	////Haptic teleoperation controller ////
	Eigen::Matrix3d transformDev_Rob = robot_pose_in_world.linear();
	auto teleop_task = new Sai2Primitives::HapticController(centerPos_rob, centerRot_rob, transformDev_Rob);

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

	double force_guidance_position_impedance = 1000.0;
	double force_guidance_orientation_impedance = 50.0;
	double force_guidance_position_damping = 5.0;
	double force_guidance_orientation_damping = 0.1;
	teleop_task->setVirtualGuidanceGains (force_guidance_position_impedance, force_guidance_position_damping,
									force_guidance_orientation_impedance, force_guidance_orientation_damping);

  // Force feedback stiffness proxy parameters
  double proxy_position_impedance = force_guidance_position_impedance;
  double proxy_position_damping = force_guidance_position_damping;
  double proxy_orientation_impedance = force_guidance_orientation_impedance;
  double proxy_orientation_damping = force_guidance_orientation_damping;
  teleop_task->setVirtualProxyGains (proxy_position_impedance, proxy_position_damping,
    proxy_orientation_impedance, proxy_orientation_damping);

  // Set haptic controllers parameters
	Matrix3d Red_factor_rot = Matrix3d::Identity();
	Matrix3d Red_factor_trans = Matrix3d::Identity();
	Red_factor_rot << 1/10.0, 0.0, 0.0,
						  0.0, 1/10.0, 0.0,
						  0.0, 0.0, 1/10.0;
	Red_factor_trans << 1.0/1.0, 0.0, 0.0,
						  0.0, 1.0/1.0, 0.0,
						  0.0, 0.0, 1.0/1.0;
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

	// Read haptic device specifications from haptic driver
  if (using_keyboard_input)
  {
    teleop_task->_max_linear_stiffness_device = 5000.0;
    teleop_task->_max_angular_stiffness_device = 0.0;
    teleop_task->_max_linear_damping_device = 30.0;
    teleop_task->_max_angular_damping_device = 0.0;
    teleop_task->_max_force_device = 12.0;
    teleop_task->_max_torque_device = 0.0;
  }
  else
  {
	VectorXd _max_stiffness_device0 = redis_client.getEigenMatrixJSON(DEVICE_MAX_STIFFNESS_KEYS[0]);
	VectorXd _max_damping_device0 = redis_client.getEigenMatrixJSON(DEVICE_MAX_DAMPING_KEYS[0]);
	VectorXd _max_force_device0 = redis_client.getEigenMatrixJSON(DEVICE_MAX_FORCE_KEYS[0]);

	VectorXd _max_stiffness_device1 = redis_client.getEigenMatrixJSON(DEVICE_MAX_STIFFNESS_KEYS[1]);
	VectorXd _max_damping_device1 = redis_client.getEigenMatrixJSON(DEVICE_MAX_DAMPING_KEYS[1]);
	VectorXd _max_force_device1 = redis_client.getEigenMatrixJSON(DEVICE_MAX_FORCE_KEYS[1]);

	//set the device specifications to the haptic controller
	teleop_task->_max_linear_stiffness_device = _max_stiffness_device0[0];
	teleop_task->_max_angular_stiffness_device = _max_stiffness_device0[1];
	teleop_task->_max_linear_damping_device = _max_damping_device0[0];
	teleop_task->_max_angular_damping_device = _max_damping_device0[1];
	teleop_task->_max_force_device = _max_force_device0[0];
	teleop_task->_max_torque_device = _max_force_device0[1];
  }

	// Create robot commanded vectors
	Vector3d desired_position_robot = Vector3d::Zero(); // Set position and orientation in robot frame
	Matrix3d desired_rotation_robot = Matrix3d::Identity();
	Vector3d desired_force_robot = Vector3d::Zero(); // Set force and torque in robot frame
	Vector3d desired_torque_robot = Vector3d::Zero();

	Vector3d pos_rob_model = Vector3d::Zero(); // Model position and orientation in robot frame
  Vector3d pos_rob_model_prev = Vector3d::Zero();
  Vector3d robot_displacement = Vector3d::Zero();
  Vector3d robot_total_displacement = Vector3d::Zero();
	Matrix3d rot_rob_model = Matrix3d::Identity();
	Vector3d vel_rot_rob_model = Vector3d::Zero(); // Model linear and angular velocities in robot frame
	Vector3d vel_trans_rob_model = Vector3d::Zero();
	VectorXd f_task_sensed_sensor_point = VectorXd::Zero(6); // Sensed force in world frame at sensor point
	VectorXd f_task_sensed_control_point = VectorXd::Zero(6); // Sensed force in world frame at control point

	//// passivity observer and controller ////
	auto passivity_controller = new Sai2Primitives::BilateralPassivityController(posori_task, teleop_task);
	Vector3d haptic_damping_force_passivity = Vector3d::Zero();
	Vector3d haptic_damping_torque_passivity = Vector3d::Zero();
	Vector3d command_force_device_plus_damping = Vector3d::Zero();

	double Rc_moment = 0;

	Vector3d haptic_command_torque = Vector3d::Zero();

	//Define end-effector properties
	double tool_mass = 0; /////////////////////////////////////////////////////////////////////////
	Vector3d tool_com = Vector3d::Zero();
	VectorXd force_bias_global = VectorXd::Zero(6);
	if(!flag_simulation)
	{
		force_bias_global << 0.107568,   -0.0632298,      1.42982,  -0.00369506,    0.0110065, -0.000951872;
		tool_mass = 0.05;
		tool_com = Vector3d(0.0, 0.0, 0.0);  //Defined in sensor frame
	}

	// Define sensor frame
	Matrix3d R_sensor = Matrix3d::Identity(); //Sensor frame rotation in world
	Vector3d sensor_pos_in_link = Eigen::Vector3d(0.0,0.0,0.117);
	Eigen::Affine3d sensor_transform_in_link = Affine3d::Identity(); //Sensor frame transformation in end-effector
	// Set sensor frame transform in end-effector frame
	sensor_transform_in_link.translation() = sensor_pos_in_link;
	//sensor_transform_in_link.linear() = Matrix3d::Identity();

	posori_task->setForceSensorFrame(link_name, sensor_transform_in_link);

	// write inital task force and sensed torques in redis
	redis_client.setEigenMatrixJSON(FORCE_SENSED_KEY,f_task_sensed_sensor_point);
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_SENSED_KEY,torques_sensed);

	teleop_task->_haptic_feedback_from_proxy = false;
	teleop_task->_filter_on = false;
	double fc_force=0.06;
	double fc_moment=0.06;
	teleop_task->setFilterCutOffFreq(fc_force, fc_moment);

	//User switch states
	bool gripper_state=false;
	bool gripper_state_prev = false;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); //Compiler en mode release
	double current_time = 0;
	double prev_time = 0;
	// double dt = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs

	// Add device workspace virtual limits
	teleop_task->_add_workspace_virtual_limit=true;
	double device_workspace_angle_limit = 90*M_PI/180.0;
  double device_workspace_radius_limit  = 0.08;
  if (using_keyboard_input)
  {
    device_workspace_radius_limit = 0.2;
  }
	teleop_task->setWorkspaceLimits(device_workspace_radius_limit, device_workspace_angle_limit);

	// Set-up guidance parameters
	// Gains for haptic guidance
	double guidance_stiffness = 0.6;
	double guidance_damping = 1.0;
	teleop_task->setHapticGuidanceGains(guidance_stiffness, guidance_damping);
	teleop_task->_enable_plane_guidance = false;
	teleop_task->_enable_line_guidance = false;

  // Robot displacement
  int disp_counter = 0;
  double robot_displacement_contact = -1;
  double contact_displacement_threshold = 0;
  Matrix3d sigma_displacement = Matrix3d::Identity();
  Vector3d displacement_direction;

  // Contact detection parameters
  double first_contact_threshold = 1.5; //contact force in N
  double maintained_contact_threshold = 3.0;
  int contact_counter = 0;
  int release_counter_1 = 0;
  int release_counter_2 = 0;
  int release_counter_3 = 0;
  int contact_duration = 200; //increments to evaluate contact normal (0.2s)
  double mean_contact_force = 0;
  Vector3d mean_contact_normal = Vector3d::Zero();
  Vector3d contact_force = Vector3d::Zero();
  Vector3d guidance_normal_vec_first = Vector3d::Zero();
  Vector3d guidance_normal_vec_second = Vector3d::Zero();
  Vector3d guidance_normal_vec_third = Vector3d::Zero();
  Vector3d guidance_motion_vec = Vector3d::Zero();
	Vector3d guidance_normal_vec = Vector3d::Zero();
  int nbr_force_direction = 0;
  int nbr_force_direction_prev = 0;
	// Points selection
	bool isPressed = false;

	// setup redis keys to be updated with the callback
	// objects to read from redis
	// redis_client.addEigenToRead(JOINT_ANGLES_KEY, robot->_q);
	// redis_client.addEigenToRead(JOINT_VELOCITIES_KEY, robot->_dq);
	// redis_client.addEigenToRead(JOINT_TORQUES_SENSED_KEY, torques_sensed);
	// redis_client.addEigenToRead(DEVICE_POSITION_KEYS[0], teleop_task->_current_position_device);
	// redis_client.addEigenToRead(DEVICE_ROTATION_KEYS[0], teleop_task->_current_rotation_device);
	// redis_client.addEigenToRead(DEVICE_TRANS_VELOCITY_KEYS[0], teleop_task->_current_trans_velocity_device);
	// redis_client.addEigenToRead(DEVICE_ROT_VELOCITY_KEYS[0], teleop_task->_current_rot_velocity_device);
  //
	// redis_client.addEigenToRead(FORCE_SENSED_KEY, f_task_sensed_sensor_point);
	// redis_client.addEigenToRead(DEVICE_SENSED_FORCE_KEYS[0], teleop_task->_sensed_force_device);
	// redis_client.addEigenToRead(DEVICE_SENSED_TORQUE_KEYS[0], teleop_task->_sensed_torque_device);
  //
	// redis_client.addIntToRead(REMOTE_ENABLED_KEY, remote_enabled);
	// redis_client.addIntToRead(RESTART_CYCLE_KEY, restart_cycle);
  //
	// redis_client.addDoubleToRead(DEVICE_GRIPPER_POSITION_KEYS[0], teleop_task->_current_position_gripper_device);
	// redis_client.addDoubleToRead(DEVICE_GRIPPER_VELOCITY_KEYS[0], teleop_task->_current_gripper_velocity_device);
  //
	// // objects to write to redis
	// redis_client.addEigenToWrite(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	// redis_client.addEigenToWrite(DEVICE_COMMANDED_FORCE_KEYS[0], command_force_device_plus_damping);
	// redis_client.addEigenToWrite(DEVICE_COMMANDED_TORQUE_KEYS[0], teleop_task->_commanded_torque_device);
	// redis_client.addDoubleToWrite(DEVICE_COMMANDED_GRIPPER_FORCE_KEYS[0], teleop_task->_commanded_gripper_force_device);

	// logging to redis
	// redis_client.addDoubleToWrite(LOGGING_TIME_KEY, current_time);
  //
	// redis_client.addEigenToWrite(LOGGING_ROBOT_JOINT_ANGLES, robot->_q);
	// redis_client.addEigenToWrite(LOGGING_ROBOT_JOINT_VELOCITIES, robot->_dq);
	// redis_client.addEigenToWrite(LOGGING_ROBOT_COMMAND_TORQUES, command_torques);
	// redis_client.addEigenToWrite(LOGGING_ROBOT_DESIRED_POSITION, posori_task->_desired_position);
	// redis_client.addEigenToWrite(LOGGING_ROBOT_DESIRED_ORIENTATION, posori_task->_desired_orientation);
	// redis_client.addEigenToWrite(LOGGING_ROBOT_DESIRED_FORCE, posori_task->_desired_force);
	// redis_client.addEigenToWrite(LOGGING_ROBOT_DESIRED_MOMENT, posori_task->_desired_moment);
	// redis_client.addEigenToWrite(LOGGING_ROBOT_CURRENT_POSITION, posori_task->_current_position);
	// redis_client.addEigenToWrite(LOGGING_ROBOT_CURRENT_VELOCITY, posori_task->_current_velocity);
	// redis_client.addEigenToWrite(LOGGING_ROBOT_CURRENT_ORIENTATION, posori_task->_current_orientation);
	// redis_client.addEigenToWrite(LOGGING_ROBOT_CURRENT_ANGVEL, posori_task->_current_angular_velocity);
	// redis_client.addEigenToWrite(LOGGING_ROBOT_TASK_FORCE, posori_task->_task_force);
  //
	// redis_client.addEigenToWrite(LOGGING_HAPTIC_POSITION, teleop_task->_current_position_device);
	// redis_client.addEigenToWrite(LOGGING_HAPTIC_VELOCITY, teleop_task->_current_trans_velocity_device);
	// redis_client.addEigenToWrite(LOGGING_HAPTIC_ORIENTATION, teleop_task->_current_rotation_device);
	// redis_client.addEigenToWrite(LOGGING_HAPTIC_ANGVEL, teleop_task->_current_rot_velocity_device);
	// redis_client.addEigenToWrite(LOGGING_HAPTIC_COMMAND_FORCE, teleop_task->_commanded_force_device);
	// redis_client.addEigenToWrite(LOGGING_HAPTIC_COMMAND_TORQUE, haptic_command_torque);
	// redis_client.addEigenToWrite(LOGGING_HAPTIC_COMMAND_FORCE_TOTAL, command_force_device_plus_damping);
	// redis_client.addEigenToWrite(LOGGING_HAPTIC_COMMAND_TORQUE_TOTAL, haptic_command_torque);
  //
	// redis_client.addEigenToWrite(LOGGING_R_ROBOT_SENSOR, R_sensor);
	// redis_client.addEigenToWrite(LOGGING_R_HAPTIC_ROBOT, teleop_task->_Rotation_Matrix_DeviceToRobot);
  //
	// redis_client.addEigenToWrite(LOGGING_SENSED_FORCE_ROBOT_FRAME, posori_task->_sensed_force);
	// redis_client.addEigenToWrite(LOGGING_SENSED_MOMENT_ROBOT_FRAME, posori_task->_sensed_moment);
  //
	// redis_client.addDoubleToWrite(LOGGING_BILATERAL_PASSIVITY_ALPHA_FORCE, passivity_controller->_alpha_force);
	// redis_client.addDoubleToWrite(LOGGING_BILATERAL_PASSIVITY_ALPHA_MOMENT, passivity_controller->_alpha_moment);
	// redis_client.addDoubleToWrite(LOGGING_PASSIVITY_RC_FORCE, posori_task->_Rc_inv_force);
	// redis_client.addDoubleToWrite(LOGGING_PASSIVITY_RC_MOMENT, Rc_moment);

	Vector3d f_virtual_trans_rob_frame = teleop_task->_Rotation_Matrix_DeviceToRobot.transpose() * teleop_task->_f_virtual_trans;
	Vector3d f_virtual_rot_rob_frame = teleop_task->_Rotation_Matrix_DeviceToRobot.transpose() * teleop_task->_f_virtual_rot;

	// redis_client.addEigenToWrite(LOGGING_VIRTUAL_FORCE_ROBOT_FRAME, f_virtual_trans_rob_frame);
	// redis_client.addEigenToWrite(LOGGING_VIRTUAL_MOMENT_ROBOT_FRAME, f_virtual_rot_rob_frame);


	double t0 = 0;

	VectorXd xy_desired = VectorXd::Zero(2);


	/////////////////////////////// cyclic ////////////////////////////////////////
	while (runloop)
	{
		// wait for next scheduled loop
		timer.waitForNextLoop();
		current_time = timer.elapsedTime() - start_time;
		// dt = current_time - prev_time;

		f_virtual_trans_rob_frame = teleop_task->_Rotation_Matrix_DeviceToRobot.transpose() * teleop_task->_f_virtual_trans;
		f_virtual_rot_rob_frame = teleop_task->_Rotation_Matrix_DeviceToRobot.transpose() * teleop_task->_f_virtual_rot;

		// read all redis keys
		//redis_client.readAllSetupValues();

    // Robot data
    robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
    robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
    torques_sensed = redis_client.getEigenMatrixJSON(JOINT_TORQUES_SENSED_KEY);
    f_task_sensed_sensor_point = redis_client.getEigenMatrixJSON(FORCE_SENSED_KEY);
    // read renabling/disabling teleoperation
  	remote_enabled = stoi(redis_client.get(REMOTE_ENABLED_KEY));
  	// read restar cycle key
  	restart_cycle = stoi(redis_client.get(RESTART_CYCLE_KEY));
    // Haptic device data
    if (using_keyboard_input)
    {
      // keyboard commands if no device
      teleop_task->_current_position_device = redis_client.getEigenMatrixJSON(MOUSE_POSITION_KEY);
      teleop_task->_current_trans_velocity_device.setZero();
      teleop_task->_current_rotation_device.setIdentity();
      teleop_task->_current_rot_velocity_device.setZero();
      teleop_task->_sensed_force_device.setZero();
      teleop_task->_sensed_torque_device.setZero();
      mouse_gripper = stoi(redis_client.get(MOUSE_GRIPPER_KEY));
    }
    else
    {
      teleop_task->_current_position_device = redis_client.getEigenMatrixJSON(DEVICE_POSITION_KEYS[0]);
      teleop_task->_current_rotation_device = redis_client.getEigenMatrixJSON(DEVICE_ROTATION_KEYS[0]);
      teleop_task->_current_trans_velocity_device = redis_client.getEigenMatrixJSON(DEVICE_TRANS_VELOCITY_KEYS[0]);
      teleop_task->_current_rot_velocity_device = redis_client.getEigenMatrixJSON(DEVICE_ROT_VELOCITY_KEYS[0]);
      teleop_task->_sensed_force_device = redis_client.getEigenMatrixJSON(DEVICE_SENSED_FORCE_KEYS[0]);
      teleop_task->_sensed_torque_device = redis_client.getEigenMatrixJSON(DEVICE_SENSED_TORQUE_KEYS[0]);
      teleop_task->_current_position_gripper_device = stod(redis_client.get(DEVICE_GRIPPER_POSITION_KEYS[0]));
      teleop_task->_current_gripper_velocity_device = stod(redis_client.get(DEVICE_GRIPPER_VELOCITY_KEYS[0]));
    }

    if(controller_counter % 100 == 20)
		{
    //cout << "Position device :" << (teleop_task->_current_position_device).transpose() << endl ;
    cout << "Uncontrolled sensed contact force :" << contact_force.transpose() << endl;
    //cout << "Commanded robot force :" << (posori_task->_desired_force).transpose() << endl;
    //cout << "Sensed robot force :" << (f_task_sensed_control_point.head(3)).transpose() << endl;
    //cout << "norm contact force :" << contact_force.norm() << endl;
    //cout << "robot displacement :" << robot_displacement.transpose() << endl;
    cout << "robot displacement in contact direction : " << robot_displacement_contact << endl;

    cout <<  "Contact force state :" << nbr_force_direction << endl;
    cout << "First contact normal :"<< guidance_normal_vec_first.transpose() << endl;
    cout << "Second contact normal :"<< guidance_normal_vec_second.transpose() << endl;
    cout << "Motion direction" << guidance_motion_vec.transpose() << endl;
    }

		// Update robot model
		if(flag_simulation)
		{
			robot->updateModel();
			robot->coriolisForce(coriolis_torques);
			robot->gravityVector(gravity_torques);
		}
		else
		{
			robot->updateKinematics();
			robot->_M = redis_client.getEigenMatrixJSON(MASSMATRIX_KEY);
			robot->_M_inv = robot->_M.inverse();

			coriolis_torques = redis_client.getEigenMatrixJSON(CORIOLIS_KEY);
			gravity_torques = redis_client.getEigenMatrixJSON(ROBOT_GRAVITY_KEY);
		}

		// Update position and orientation of the robot from the model
		robot->position(pos_rob_model, link_name, pos_in_link);
		robot->linearVelocity(vel_trans_rob_model, link_name, pos_in_link);
		robot->rotation(rot_rob_model, link_name);
		robot->angularVelocity(vel_rot_rob_model, link_name);
		teleop_task->updateSensedRobotPositionVelocity(pos_rob_model, vel_trans_rob_model,
														rot_rob_model, vel_rot_rob_model);

    teleop_task->updateVirtualProxyPositionVelocity(pos_rob_model, vel_trans_rob_model,
														rot_rob_model, vel_rot_rob_model);

		//Sensor frame rotation in world frame
		robot->rotation(R_sensor, link_name);
		R_sensor = R_sensor*sensor_transform_in_link.rotation();

		if(!flag_simulation)
		{
			// Remove sensor bias in sensor frame
			f_task_sensed_sensor_point -= force_bias_global;

			// Adjust sensed force with effector mass in sensor frame
			Vector3d P_tool_SensorFrame = tool_mass * R_sensor.transpose() * Vector3d(0,0,-9.81);
			f_task_sensed_sensor_point.head(3) += P_tool_SensorFrame;
			f_task_sensed_sensor_point.tail(3) += tool_com.cross(P_tool_SensorFrame);

			//Transfer sensed task force in world frame
			f_task_sensed_sensor_point.head(3) = R_sensor*f_task_sensed_sensor_point.head(3);
			f_task_sensed_sensor_point.tail(3) = R_sensor*f_task_sensed_sensor_point.tail(3);

			f_task_sensed_sensor_point = -f_task_sensed_sensor_point;
		}

		// Send sensed force to robot controller in sensor frame
		posori_task->updateSensedForceAndMoment(-R_sensor.transpose()*f_task_sensed_sensor_point.head(3), -R_sensor.transpose()*f_task_sensed_sensor_point.tail(3));

		// Send sensed force at control point in world frame to haptic controller
		f_task_sensed_control_point.head(3) = -posori_task->_sensed_force;
		f_task_sensed_control_point.tail(3) = -posori_task->_sensed_moment;
		teleop_task->updateSensedForce(f_task_sensed_control_point);

		// Use the haptic device gripper as a switch and update the gripper state
		teleop_task->UseGripperAsSwitch();

    // Read new gripper state
    if (using_keyboard_input)
    {
      if (mouse_gripper==1)
      {
        gripper_state = true;
      }
      else
      {
        gripper_state = false;
      }
    }
    else
    {
      gripper_state = teleop_task->gripper_state;
    }

		//////////////////////// Run Robot/Haptic Device state machine //////////////////////////////////////////////////
		if(state == GOTO_INITIAL_CONFIG)
		{
			// update tasks model and priority
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// Adjust mass matrix to increase wrist desired stiffness
			for(int i=4 ; i<7 ; i++)
			{
				robot->_M(i,i) += 0.07;
			}
			// Compute torques
			joint_task->computeTorques(joint_task_torques);
			command_torques = joint_task_torques + coriolis_torques;

      if (!using_keyboard_input)
      {
  			// compute homing haptic device
  			teleop_task->HomingTask();
      }
      else
      {
        teleop_task->device_homed = true;
      }

			// if((joint_task->_desired_position - joint_task->_current_position).norm() < 0.2)
			if( remote_enabled==1 && teleop_task->device_homed && gripper_state && (joint_task->_desired_position - joint_task->_current_position).norm() < 0.2)
			{

				// Reinitialize controllers
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				teleop_task->reInitializeTask();

				// cout << posori_task->_desired_orientation << endl;
				posori_task->_desired_orientation << 1, 0, 0, 0, -1, 0, 0, 0, -1;

				joint_task->_kp = 50.0;
				joint_task->_kv = 14.0;
				joint_task->_ki = 0.0;

        HomePos_op = teleop_task->_current_position_device ;
				HomeRot_op = teleop_task->_current_rotation_device;
				teleop_task->setDeviceCenter(HomePos_op, HomeRot_op);

				centerPos_rob = posori_task->_desired_position;
				centerRot_rob = posori_task->_current_orientation;
				teleop_task->setRobotCenter(centerPos_rob, centerRot_rob);

        pos_rob_model_prev = pos_rob_model;

				teleop_task->_haptic_feedback_from_proxy = false; // If set to true, the force feedback is computed from a stiffness/damping proxy.
			  teleop_task->_filter_on = true;
				teleop_task->_send_haptic_feedback = true;

				gripper_state_prev = gripper_state;

				//state = IMPEDANCE_CONTROL;
				state = UNIFIED_CONTROL;
			}
		}

// 		else if(state == IMPEDANCE_CONTROL)
// 		{
// 			//// Haptic impedance controller ////
// 			//Compute haptic commands
// 			teleop_task->computeHapticCommands3d(posori_task->_desired_position);
//
// 			// update model and priority (position and joint tasks)
// 			N_prec.setIdentity();
// 			posori_task->updateTaskModel(N_prec);
// 			N_prec = posori_task->_N;
// 			joint_task->updateTaskModel(N_prec);
//
// 			// Adjust mass matrix to increase wrist desired stiffness
// 			if(!flag_simulation)
// 			{
// 				for(int i=3 ; i<6 ; i++)
// 				{
// 					posori_task->_Lambda(i,i) += 0.1;
// 				}
// 			}
//
// 			// Compute commanded robot torques
// 			posori_task->computeTorques(posori_task_torques);
// 			joint_task->computeTorques(joint_task_torques);
// 			command_torques = posori_task_torques + joint_task_torques + coriolis_torques;
//
// 			if (!using_keyboard_input)
//        {
//         // compute PO
// 			   passivity_controller->computePOPCForce(haptic_damping_force_passivity);
//        }
//
// 			// button code to change guidance type
// 			if(gripper_state && gripper_state != gripper_state_prev) //if button pushed and no change recorded yet
// 			{
// 				// if button pushed
// 				isPressed = true;
// 			} else {
// 				isPressed = false;
// 			}
// 			gripper_state_prev = gripper_state;
//
// 			if(remote_enabled == 0) //Stop haptic teleoperation
// 			{
// 				joint_task->_kp = 250.0;
// 				joint_task->_kv = 18.0;
// 				joint_task->_ki = 0.0;
// 				// joint controller to maintain robot in current position
// 				joint_task->reInitializeTask();
// 				teleop_task->reInitializeTask();
// 				posori_task->reInitializeTask();
//
// 				// set current haptic device position
// 				teleop_task->setDeviceCenter(teleop_task->_current_position_device, teleop_task->_current_rotation_device);
//
// 				state = MAINTAIN_POSITION;
// 			}
// 			else if(isPressed && switch_state_counter == 0) //If the button is pushed switch to unified haptic controller
// 			{
//
//         // set up unified robot controller into pure motion control
//         nbr_force_direction = 0;
// 				posori_task->setFullLinearMotionControl();
//         posori_task->setFullAngularMotionControl();
//
// 				// Update the selection matrices for haptic controller
// 				teleop_task->updateSelectionMatrices(posori_task->_sigma_position, posori_task->_sigma_orientation,
// 								 						posori_task->_sigma_force, posori_task->_sigma_moment);
//
// 				// Switch to new haptic controller
// 				teleop_task->reInitializeTask();
// 				teleop_task->_haptic_feedback_from_proxy = false; // If set to true, the force feedback is computed from a stiffness/damping proxy.
// 			  teleop_task->_filter_on = true;
// 				teleop_task->_send_haptic_feedback = true;
//
//         pos_rob_model_prev = pos_rob_model;
//
// 				passivity_controller->reInitializeTask();
//
// 				switch_state_counter = 500;
// 				t0 = current_time;
// 				state = UNIFIED_CONTROL;
// 			}
//
// 			switch_state_counter--;
// 			if(switch_state_counter < 0)
// 			{
// 				switch_state_counter = 0;
// 			}
// 		}


		else if(state == UNIFIED_CONTROL) {

      // update model and priority
      N_prec.setIdentity();
      posori_task->updateTaskModel(N_prec);
      N_prec = posori_task->_N;
      joint_task->updateTaskModel(N_prec);

      // Evaluate robot relative displacement
      robot_displacement = pos_rob_model - pos_rob_model_prev;
      pos_rob_model_prev = pos_rob_model;

      // Total displacement over 100ms
      if (disp_counter <= 100)
      {
        robot_total_displacement += robot_displacement;
        disp_counter ++;
      }
      else
      {
        // Displacement in uncontrolled force direction
        if ((robot_total_displacement.transpose()*contact_force) >= -0.0001 && (robot_total_displacement.transpose()*contact_force) <= 0.0001)
        {
          robot_displacement_contact = 0.0;
          sigma_displacement.setIdentity();
        }
        else
        {
          robot_displacement_contact = robot_total_displacement.transpose()*contact_force;
          // compute normal space to displacement
          displacement_direction = robot_total_displacement.normalized();
          sigma_displacement = Matrix3d::Identity() - displacement_direction*displacement_direction.transpose();
        }
        robot_total_displacement.setZero();
        disp_counter = 0;
      }

      //// Unified haptic controller with automatic contact detection and control ////
      switch (nbr_force_direction)
            {
               case 0 : // Full motion control
                 posori_task->setFullLinearMotionControl();
                 posori_task->setFullAngularMotionControl();

                 teleop_task->_haptic_feedback_from_proxy = false;

                 // First contact detection
                 if (contact_force.norm() >= first_contact_threshold && robot_displacement_contact >= contact_displacement_threshold)
                 {
                   contact_counter = 0;
                   mean_contact_normal.setZero();
                   mean_contact_force = 0;
                   nbr_force_direction_prev = nbr_force_direction;
                   nbr_force_direction = 10; //switch to contact detection state
                 }
                  break;

               case 1 : // One controlled contact force

               guidance_normal_vec_first = guidance_normal_vec;

               // update force control direction in robot space for unified control
         			posori_task->updateForceAxis(guidance_normal_vec_first);

              teleop_task->_haptic_feedback_from_proxy = false;

              // Adjust force-controlled direction to stay normal to displacement
              guidance_normal_vec_first = sigma_displacement*guidance_normal_vec_first;

              guidance_normal_vec = guidance_normal_vec_first;

              // Contact detection
              if (contact_force.norm() >= maintained_contact_threshold && robot_displacement_contact >= contact_displacement_threshold)
              {
                contact_counter = 0;
                mean_contact_normal.setZero();
                mean_contact_force = 0;
                nbr_force_direction_prev = nbr_force_direction;
                nbr_force_direction = 10; //switch to contact detection state
              }

              // Contact release
              if (guidance_normal_vec_first.transpose() * f_task_sensed_control_point.head(3) <= 1.0)
              {
                release_counter_1 ++;
              }
              else
              {
                release_counter_1 = 0;
              }
              if (release_counter_1 >= contact_duration)
              {
                guidance_normal_vec_first.setZero();
                nbr_force_direction = nbr_force_direction-1;
              }
                  break;

                case 2 : // Two controlled contact forces

                guidance_normal_vec_second = guidance_normal_vec;

                // Compute motion direction
                guidance_motion_vec = guidance_normal_vec_first.cross(guidance_normal_vec_second);

                // update force control direction in robot space for unified control
                posori_task->updateLinearMotionAxis(guidance_motion_vec);

                teleop_task->_haptic_feedback_from_proxy = false;

               // Adjust force-controlled direction to stay normal to displacement
               guidance_normal_vec_second = sigma_displacement*guidance_normal_vec_second;
               guidance_normal_vec_first = sigma_displacement*guidance_normal_vec_first;

               guidance_normal_vec = guidance_normal_vec_second;

               // Contact detection
               if (contact_force.norm() >= maintained_contact_threshold && robot_displacement_contact >= contact_displacement_threshold)
               {
                 contact_counter = 0;
                 mean_contact_normal.setZero();
                 mean_contact_force = 0;
                 nbr_force_direction_prev = nbr_force_direction;
                 nbr_force_direction = 10; //switch to contact detection state
               }

               // Contact release
               if (guidance_normal_vec_second.transpose() * f_task_sensed_control_point.head(3) <= 1.0)
               {
                 release_counter_2 ++;
               }
               else
               {
                 release_counter_2 = 0;
               }
               if (guidance_normal_vec_first.transpose() * f_task_sensed_control_point.head(3) <= 1.0)
               {
                 release_counter_1 ++;
               }
               else
               {
                 release_counter_1 = 0;
               }
               if (release_counter_1 >= contact_duration)
               {
                 guidance_normal_vec = guidance_normal_vec_second;
                 guidance_normal_vec_second.setZero();
                 nbr_force_direction = nbr_force_direction-1;
               }
               if (release_counter_2 >= contact_duration)
               {
                 guidance_normal_vec = guidance_normal_vec_first;
                 guidance_normal_vec_second.setZero();
                 nbr_force_direction = nbr_force_direction-1;
               }
                   break;

                case 3 : // Full force control

                guidance_normal_vec_third = guidance_normal_vec;

                posori_task->setFullForceControl();
                teleop_task->_haptic_feedback_from_proxy = false;

                // Contact release
                if (guidance_normal_vec_third.transpose() * f_task_sensed_control_point.head(3) <= 1.0)
                {
                  release_counter_3 ++;
                }
                else
                {
                  release_counter_3 = 0;
                }
                if (guidance_normal_vec_second.transpose() * f_task_sensed_control_point.head(3) <= 1.0)
                {
                  release_counter_2 ++;
                }
                else
                {
                  release_counter_2 = 0;
                }
                if (guidance_normal_vec_first.transpose() * f_task_sensed_control_point.head(3) <= 1.0)
                {
                  release_counter_1 ++;
                }
                else
                {
                  release_counter_1 = 0;
                }
                if (release_counter_1 >= contact_duration)
                {
                  guidance_normal_vec_first = guidance_normal_vec_second;
                  guidance_normal_vec_second = guidance_normal_vec_third;
                  guidance_normal_vec_third.setZero();
                  nbr_force_direction = nbr_force_direction-1;
                }
                if (release_counter_2 >= contact_duration)
                {
                  guidance_normal_vec_second = guidance_normal_vec_third;
                  guidance_normal_vec_third.setZero();
                  nbr_force_direction = nbr_force_direction-1;
                }
                if (release_counter_3 >= contact_duration)
                {
                  guidance_normal_vec_third.setZero();
                  nbr_force_direction = nbr_force_direction-1;
                }
                  break;

                case 10 : // Contact detection
                  // set force feedback to proxy computation during contact establishment
                  teleop_task->_haptic_feedback_from_proxy = true;

                  if (robot_displacement_contact < contact_displacement_threshold)
                  {
                    nbr_force_direction = nbr_force_direction_prev;
                    contact_counter = 0;
                  }

                  // Evaluate mean contact force and normal direction
                  if (contact_counter <= contact_duration)
                  {
                    mean_contact_force += contact_force.norm();
                    mean_contact_normal += contact_force.normalized();
                    contact_counter ++;
                  }
                  else
                  {
                    contact_counter = 0;
                    if ((mean_contact_force/contact_duration) >= maintained_contact_threshold)
                    {
                      guidance_normal_vec = mean_contact_normal/contact_duration;

                      //posori_task->setClosedLoopForceControl();
                      //posori_task->_passivity_enabled = true;

                      release_counter_1 = 0;
                      release_counter_2 = 0;
                      release_counter_3 = 0;
                      nbr_force_direction = nbr_force_direction_prev + 1;
                    }
                    else
                    {
                      nbr_force_direction = nbr_force_direction_prev;
                    }
                  }
                  break;
            }


			// Update the selection matrices for haptic controller
			teleop_task->updateSelectionMatrices(posori_task->_sigma_position, posori_task->_sigma_orientation,
							 						posori_task->_sigma_force, posori_task->_sigma_moment);

      // compute contact force in motion-controlled space
      contact_force = posori_task->_sigma_position*f_task_sensed_control_point.head(3);

      //Compute haptic commands
			teleop_task->computeHapticCommandsUnifiedControl3d(posori_task->_desired_position, posori_task->_desired_force);

			//posori_task->_desired_force(2) -= 3.0;

			// Adjust mass matrix to increase wrist desired stiffness
			if(!flag_simulation)
			{
				for(int i=3 ; i<6 ; i++)
				{
					posori_task->_Lambda(i,i) += 0.1;
				}

				// double coupling_correction = 0.0;
				// coupling_correction = 3.5;
				// posori_task->_Lambda(0,2) += coupling_correction;
				// posori_task->_Lambda(2,0) += coupling_correction;
			}

			// Compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);
			command_torques = posori_task_torques + joint_task_torques + coriolis_torques;

      if (!using_keyboard_input)
      {
  			// compute PO
  			passivity_controller->computePOPCForce(haptic_damping_force_passivity);
      }




			// button code to change guidance type
			if(gripper_state && gripper_state != gripper_state_prev) //if button pushed and no change recorded yet
			{
				// if button pushed
				isPressed = true;
			} else {
				isPressed = false;
			}
			gripper_state_prev = gripper_state;

			if(remote_enabled == 0)
			{
				joint_task->_kp = 250.0;
				joint_task->_kv = 18.0;
				joint_task->_ki = 0.0;
				// joint controller to maintain robot in current position
				joint_task->reInitializeTask();
				teleop_task->reInitializeTask();
				posori_task->reInitializeTask();

				// set current haptic device position
				teleop_task->setDeviceCenter(teleop_task->_current_position_device, teleop_task->_current_rotation_device);

				state = MAINTAIN_POSITION;
			}
			else if(isPressed && switch_state_counter == 0) //If the button is pushed switch to impedance controller
			{
				switch_state_counter = 500;

				joint_task->_kp = 250.0;
				joint_task->_kv = 18.0;
				joint_task->_ki = 0.0;
				// joint controller to maintain robot in current position
				joint_task->reInitializeTask();
				teleop_task->reInitializeTask();
				posori_task->reInitializeTask();

				// set current haptic device position
				teleop_task->setDeviceCenter(teleop_task->_current_position_device, teleop_task->_current_rotation_device);

				state = MAINTAIN_POSITION;
			}

			switch_state_counter--;
			if(switch_state_counter < 0)
			{
				switch_state_counter = 0;
			}
		}

		else if(state == MAINTAIN_POSITION) // Maintain the robots in current position
		{
			// update tasks model
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// Adjust mass matrix to increase wrist desired stiffness
			for(int i =4 ; i<7 ; i++)
			{
				robot->_M(i,i) += 0.07;
			}
			// Compute torques
			joint_task->computeTorques(joint_task_torques);
			command_torques = joint_task_torques + coriolis_torques;

      // Maintain haptic device position
			teleop_task->HomingTask();

			// button code to change guidance type
			if(gripper_state && gripper_state != gripper_state_prev) //if button pushed and no change recorded yet
			{
				// if button pushed
				isPressed = true;
			} else {
				isPressed = false;
			}
			gripper_state_prev = gripper_state;

			if (remote_enabled==1 && isPressed && switch_state_counter == 0)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				teleop_task->reInitializeTask();

				joint_task->_desired_position = goal_posture;

				centerPos_rob = posori_task->_current_position;
				centerRot_rob = posori_task->_current_orientation;
				teleop_task->setRobotCenter(centerPos_rob, centerRot_rob);

				HomePos_op = teleop_task->_current_position_device ;
				teleop_task->setDeviceCenter(HomePos_op, HomeRot_op);

				state = IMPEDANCE_CONTROL;
			}
			else if (restart_cycle == 1)
			{
				// reset joint controller to robot home position
				joint_task->reInitializeTask();
				teleop_task->reInitializeTask();

				joint_task->_desired_position = goal_posture;

				// reset haptic device home position
				HomePos_op << 0.0, 0.01, 0.0;
				HomeRot_op.setIdentity();
				teleop_task->setDeviceCenter(HomePos_op, HomeRot_op);

				state = GOTO_INITIAL_CONFIG;
			}
			switch_state_counter--;
			if(switch_state_counter < 0)
			{
				switch_state_counter = 0;
			}
		}
		else
		{
				command_torques.setZero(robot->dof());
				teleop_task->GravityCompTask();
		}

		// add damping to passivity controller and write all to redis
		command_force_device_plus_damping = teleop_task->_commanded_force_device + haptic_damping_force_passivity;
		//command_force_device_plus_damping = teleop_task->_commanded_force_device;
		// command_force_device_plus_damping.setZero();


		//redis_client.writeAllSetupValues();

    //// Send robot commands through redis ////
  	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
  	//// Send the haptic commands to the device driver through Redis keys ////
  	redis_client.setEigenMatrixJSON(DEVICE_COMMANDED_FORCE_KEYS[0], command_force_device_plus_damping);
  	redis_client.setEigenMatrixJSON(DEVICE_COMMANDED_TORQUE_KEYS[0], teleop_task->_commanded_torque_device);
  	redis_client.set(DEVICE_COMMANDED_GRIPPER_FORCE_KEYS[0], to_string(teleop_task->_commanded_gripper_force_device));

    redis_client.setEigenMatrixJSON(CONTROLLED_CONTACT_DIRECTION_3,guidance_normal_vec_third);
    redis_client.setEigenMatrixJSON(CONTROLLED_CONTACT_DIRECTION_1,guidance_normal_vec_first);
    redis_client.setEigenMatrixJSON(CONTROLLED_CONTACT_DIRECTION_2,guidance_normal_vec_second);


		prev_time = current_time;
		controller_counter++;
	}

	command_torques.setZero(robot->dof());
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	//// Send the haptic commands to the device driver through Redis keys ////
	redis_client.setEigenMatrixJSON(DEVICE_COMMANDED_FORCE_KEYS[0], Vector3d::Zero());
	redis_client.setEigenMatrixJSON(DEVICE_COMMANDED_TORQUE_KEYS[0], Vector3d::Zero());
	redis_client.set(DEVICE_COMMANDED_GRIPPER_FORCE_KEYS[0], "0.0");

	delete teleop_task;
	teleop_task = NULL;

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}
