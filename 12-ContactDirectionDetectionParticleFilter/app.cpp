// This example tests the haptic device driver and the open-loop bilateral teleoperation controller.

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "tasks/JointTask.h"
#include "tasks/PositionTask.h"
#include "haptic_tasks/HapticController.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

#include "force_sensor/ForceSensorSim.h" // Add force sensor simulation and display classes
#include "force_sensor/ForceSensorDisplay.h"

#include <iostream>
#include <string>
#include <random>

#define INIT            0
#define CONTROL         1

#include <signal.h>
bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

using namespace std;
using namespace Eigen;
using namespace chai3d;

const string world_file = "./resources/world.urdf";
const string robot_file = "./resources/sphere.urdf";
const string robot_name = "sphere";
const string camera_name = "camera";
const string link_name = "end_effector"; //robot end-effector
// Set sensor frame transform in end-effector frame
Affine3d sensor_transform_in_link = Affine3d::Identity();
const Vector3d sensor_pos_in_link = Eigen::Vector3d(0.0,0.0,0.0);
const Vector3d pos_in_link = Vector3d(0.0,0.0,0.0);

VectorXd sensed_force_moment = VectorXd::Zero(6);

// Define parameters for control with keyboard
Vector3d current_mouse_position = Eigen::Vector3d(0.0, 0.01, 0.0);
int mouse_gripper = 0;
// flags for keyboard commands
bool fMouseXp = false;
bool fMouseXn = false;
bool fMouseYp = false;
bool fMouseYn = false;
bool fMouseZp = false;
bool fMouseZn = false;
bool fMouseGripper = false;

// redis keys:
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

RedisClient redis_client;

// simulation function prototype
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim, ForceSensorSim* force_sensor);
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);
void particle_filter();

Vector3d control_pfilter;
Vector3d measured_velocity_pfilter;
Vector3d measured_force_pfilter;

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// flags for scene camera movement
bool fTransXp = false;
bool fTransYp = false;
bool fTransXn = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fshowCameraPose = false;
bool fRotPanTilt = false;
// flag for enabling/disabling remote task
bool fOnOffRemote = false;

// particle filter parameters
const int n_particles = 100;
vector<Vector3d> particles;
vector<cShapeSphere*> particles_graphics;
const double particle_radius = 0.007;
cColorf center_particle_color = cColorf(1.0, 1.0, 0.0, 1.0);
cColorf sphere_particle_color = cColorf(1.0, 0.0, 0.0, 1.0);
const Vector3d center_graphic_representation = Vector3d(0.5,0.5,0.5);
const double graphic_representation_radius = 0.15;

const double percent_chance_contact_appears = 0.15;
const double percent_chance_contact_disapears = 0.85;
const double mean_scatter = 0.0;
const double std_scatter = 0.01;

double sampleNormalDistribution(const double mean, const double std)
{
	// random device class instance, source of 'true' randomness for initializing random seed
    random_device rd; 
    // Mersenne twister PRNG, initialized with seed from random device instance
    mt19937 gen(rd()); 
    // instance of class normal_distribution with specific mean and stddev
    normal_distribution<float> d(mean, std); 
    // get random number with normal distribution using gen as random source
    return d(gen); 
}

double sampleUniformDistribution(const double min, const double max)
{
	double min_internal = min;
	double max_internal = max;
	if(min > max)
	{
		min_internal = max;
		max_internal = min;
	}
	// random device class instance, source of 'true' randomness for initializing random seed
    random_device rd; 
    // Mersenne twister PRNG, initialized with seed from random device instance
    mt19937 gen(rd()); 
    // instance of class uniform_distribution with specific min and max
    uniform_real_distribution<float> d(min_internal, max_internal); 
    // get random number with normal distribution using gen as random source
    return d(gen); 
}

vector<pair<Vector3d, double>> particleFilterMotionUpdate(const vector<Vector3d> particles, 
														const Vector3d unit_mass_control, 
														const Vector3d measured_velocity, 
														const Vector3d measured_force)
{
	int n = particles.size();
	Vector3d control_normalized = Vector3d::Zero();
	Vector3d measure_velocity_normalized = Vector3d::Zero();
	Vector3d measure_force_normalized = Vector3d::Zero();
	if(unit_mass_control.norm() > 1e-3)
	{
		control_normalized = unit_mass_control/unit_mass_control.norm();
	}
	if(measured_velocity.norm() > 1e-3)
	{
		measure_velocity_normalized = measured_velocity/measured_velocity.norm();
	}
	if(measured_force.norm() > 1e-3)
	{
		measure_force_normalized = measured_force/measured_force.norm();
	}

	vector<pair<Vector3d, double>> weighted_particles;
	double cumulative_weight = 0;


	for(int i=0 ; i<n ; i++)
	{
		// control update : compute new particle location
		Vector3d current_particle = particles[i];
		double uniform_rand = sampleUniformDistribution(0, 1);
		if(current_particle.norm() < 1e-3) // non contact
		{
			// percentage chance to go into contact in the direction of motion
			if(uniform_rand < percent_chance_contact_appears)
			{
				current_particle = control_normalized;
			}
		}
		// chance to lose contact high if control is opposite to contact normal
		else if(uniform_rand < - percent_chance_contact_disapears * current_particle.dot(control_normalized))
		{
			current_particle.setZero();
		}
		else
		{
			double normal_rand = sampleNormalDistribution(mean_scatter, std_scatter);
			current_particle += normal_rand * unit_mass_control;
			current_particle.normalize();
		}

		// measurement update : compute weight due to force measurement
		double weight_force = 0;
		if(current_particle.norm() < 1e-3)
		{
			weight_force = 1-tanh(measured_force.norm()/2);
		}
		else
		{
			weight_force = -tanh(current_particle.dot(measured_force));
		}
		if(weight_force < 0)
		{
			weight_force = 0;
		}

		// measurement update : compute weight due to velocity measurement
		double weight_velocity = 1 - abs(measure_velocity_normalized.dot(current_particle));

		// final weight
		double weight = weight_force + weight_velocity;
		cumulative_weight += weight;

		weighted_particles.push_back(make_pair(current_particle, cumulative_weight));
	}

	for(int i=0 ; i<n ; i++)
	{
		weighted_particles[i].second /= cumulative_weight;
	}

	return weighted_particles;

}


vector<Vector3d> particleFilterResampling(const vector<pair<Vector3d, double>> weighted_particles, const int n_particles)
{
	vector<Vector3d> new_particles;
	int n_weighted_particles = weighted_particles.size();

	for(int i=0 ; i<n_particles ; i++)
	{

		int k = 0;
		double rand = sampleUniformDistribution(0,1);
		while(weighted_particles[k].second < rand && k < n_weighted_particles)
		{
			k++;
		}

		new_particles.push_back(weighted_particles[k].first);
	}

	return new_particles;
}



int main() {
	cout << "Loading URDF world model file: " << world_file << endl;



	// for(int i = 0 ; i<50 ; i++)
	// {
	// 	cout << sampleUniformDistribution(0,0.5) << endl;
	// }

	// exit(0);


	// start redis client
	redis_client = RedisClient();
	redis_client.connect();

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
	Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setCollisionRestitution(0);
	sim->setCoeffFrictionStatic(0.2);

	// read joint positions, velocities, update model
	sim->getJointPositions(robot_name, robot->_q);
	sim->getJointVelocities(robot_name, robot->_dq);
	robot->updateModel();

	// Add force sensor to the end-effector
	sensor_transform_in_link.translation() = sensor_pos_in_link;
	auto force_sensor = new ForceSensorSim(robot_name, link_name, sensor_transform_in_link, robot);
	force_sensor->enableFilter(0.005);
	auto fsensor_display = new ForceSensorDisplay(force_sensor, graphics);
	fsensor_display->_force_line_scale = 10.0;

	// prepare particle filter visualization
	for(int i=0 ; i<n_particles ; i++)
	{
		particles.push_back(Vector3d::Zero());
		particles_graphics.push_back(new chai3d::cShapeSphere(particle_radius));
		particles_graphics[i]->m_material->setColor(center_particle_color);
		graphics->_world->addChild(particles_graphics[i]);
	}

	for(int i=0 ; i<n_particles/3 ; i++)
	{
		particles[i] = Vector3d::Random();
		particles[i].normalize();
	}

	/*------- Set up visualization -------*/
	// set up error callback
	glfwSetErrorCallback(glfwError);

	// initialize GLFW
	glfwInit();

	// retrieve resolution of computer display and position window accordingly
	GLFWmonitor* primary = glfwGetPrimaryMonitor();
	const GLFWvidmode* mode = glfwGetVideoMode(primary);

	// information about computer screen and GLUT display window
	int screenW = mode->width;
	int screenH = mode->height;
	int windowW = 0.8 * screenH;
	int windowH = 0.5 * screenH;
	int windowPosY = (screenH - windowH) / 2;
	int windowPosX = windowPosY;

	// create window and make it current
	glfwWindowHint(GLFW_VISIBLE, 0);
	GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - Sigma7Applications", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// cache variables
	double last_cursorx, last_cursory;

	fSimulationRunning = true;
	thread sim_thread(simulation, robot, sim, force_sensor);
	thread control_thread(control, robot, sim);
	thread particle_filter_thread(particle_filter);

	// while window is open:
	while (!glfwWindowShouldClose(window) && fSimulationRunning)
	{
		for(int i=0 ; i<n_particles ; i++)
		{
			particles_graphics[i]->setLocalPos(center_graphic_representation + graphic_representation_radius * particles[i]);
			if(particles[i].norm() < 1e-3)
			{
				particles_graphics[i]->m_material->setColor(center_particle_color);
			}
			else
			{
				particles_graphics[i]->m_material->setColor(sphere_particle_color);
			}
		}

		fsensor_display->update();
		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		graphics->render(camera_name, width, height);

		// swap buffers
		glfwSwapBuffers(window);

		// wait until all GL commands are completed
		glFinish();

		// check for any OpenGL errors
		GLenum err;
		err = glGetError();
		assert(err == GL_NO_ERROR);

		// poll for events
		glfwPollEvents();

		// move scene camera as required
		// graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
		Eigen::Vector3d cam_depth_axis;
		cam_depth_axis = camera_lookat - camera_pos;
		cam_depth_axis.normalize();
		Eigen::Vector3d cam_up_axis;
		// cam_up_axis = camera_vertical;
		// cam_up_axis.normalize();
		cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
		Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
		cam_roll_axis.normalize();
		Eigen::Vector3d cam_lookat_axis = camera_lookat;
		cam_lookat_axis.normalize();
		if (fTransXp) {
			camera_pos = camera_pos + 0.01*cam_roll_axis;
			camera_lookat = camera_lookat + 0.01*cam_roll_axis;
		}
		if (fTransXn) {
			camera_pos = camera_pos - 0.01*cam_roll_axis;
			camera_lookat = camera_lookat - 0.01*cam_roll_axis;
		}
		if (fTransYp) {
			// camera_pos = camera_pos + 0.05*cam_lookat_axis;
			camera_pos = camera_pos + 0.01*cam_up_axis;
			camera_lookat = camera_lookat + 0.01*cam_up_axis;
		}
		if (fTransYn) {
			// camera_pos = camera_pos - 0.05*cam_lookat_axis;
			camera_pos = camera_pos - 0.01*cam_up_axis;
			camera_lookat = camera_lookat - 0.01*cam_up_axis;
		}
		if (fTransZp) {
			camera_pos = camera_pos + 0.05*cam_depth_axis;
			camera_lookat = camera_lookat + 0.05*cam_depth_axis;
		}
		if (fTransZn) {
			camera_pos = camera_pos - 0.05*cam_depth_axis;
			camera_lookat = camera_lookat - 0.05*cam_depth_axis;
		}
		if (fshowCameraPose) {
			cout << endl;
			cout << "camera position : " << camera_pos.transpose() << endl;
			cout << "camera lookat : " << camera_lookat.transpose() << endl;
			cout << endl;
		}
		if (fRotPanTilt) {
			// get current cursor position
			double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			//TODO: might need to re-scale from screen units to physical units
			double compass = 0.006*(cursorx - last_cursorx);
			double azimuth = 0.006*(cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
			Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
			camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
		}
		graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
		glfwGetCursorPos(window, &last_cursorx, &last_cursory);
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();
	control_thread.join();
	particle_filter_thread.join();

	// destroy context
	glfwDestroyWindow(window);

	// terminate
	glfwTerminate();

	return 0;
}

void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim)
{
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	int state = INIT;
	MatrixXd N_prec = MatrixXd::Identity(dof,dof);

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);
	Vector3d x_init = joint_task->_current_position;
	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_use_interpolation_flag = false;
	joint_task->_use_velocity_saturation_flag = false;

	// position task
	auto pos_task = new Sai2Primitives::PositionTask(robot, link_name, pos_in_link);
	VectorXd pos_task_torques = VectorXd::Zero(dof);
	pos_task->_use_interpolation_flag = false;
	pos_task->_use_velocity_saturation_flag = false;

	// haptic task
	////Haptic teleoperation controller ////
	auto teleop_task = new Sai2Primitives::HapticController(x_init, Matrix3d::Zero());
	teleop_task->_send_haptic_feedback = false;

	//User switch states
	teleop_task->UseGripperAsSwitch();
	bool gripper_state = false;
	bool gripper_state_prev = false;

	 //Task scaling factors
	double Ks = 2.0;
	double KsR = 1.0;
	teleop_task->setScalingFactors(Ks, KsR);

	// // Center of the haptic device workspace
	// Vector3d HomePos_op;
	// HomePos_op << 0.0, 0.0, 0.0;
	// Matrix3d HomeRot_op;
	// HomeRot_op.setIdentity();
	// teleop_task->setDeviceCenter(HomePos_op, HomeRot_op);

	// double force_guidance_position_impedance = 1000.0;
	// double force_guidance_orientation_impedance = 50.0;
	// double force_guidance_position_damping = 5.0;
	// double force_guidance_orientation_damping = 0.1;
	// teleop_task->setVirtualGuidanceGains (force_guidance_position_impedance, force_guidance_position_damping,
	// 								force_guidance_orientation_impedance, force_guidance_orientation_damping);

	VectorXd _max_stiffness_device0 = redis_client.getEigenMatrixJSON(DEVICE_MAX_STIFFNESS_KEYS[0]);
	VectorXd _max_damping_device0 = redis_client.getEigenMatrixJSON(DEVICE_MAX_DAMPING_KEYS[0]);
	VectorXd _max_force_device0 = redis_client.getEigenMatrixJSON(DEVICE_MAX_FORCE_KEYS[0]);

	//set the device specifications to the haptic controller
	teleop_task->_max_linear_stiffness_device = _max_stiffness_device0[0];
	teleop_task->_max_angular_stiffness_device = _max_stiffness_device0[1];
	teleop_task->_max_linear_damping_device = _max_damping_device0[0];
	teleop_task->_max_angular_damping_device = _max_damping_device0[1];
	teleop_task->_max_force_device = _max_force_device0[0];
	teleop_task->_max_torque_device = _max_force_device0[1];

	// setup redis keys to be updated with the callback
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	// Objects to read from redis
    redis_client.addEigenToReadCallback(0, DEVICE_POSITION_KEYS[0], teleop_task->_current_position_device);
    redis_client.addEigenToReadCallback(0, DEVICE_ROTATION_KEYS[0], teleop_task->_current_rotation_device);
    redis_client.addEigenToReadCallback(0, DEVICE_TRANS_VELOCITY_KEYS[0], teleop_task->_current_trans_velocity_device);
    redis_client.addEigenToReadCallback(0, DEVICE_ROT_VELOCITY_KEYS[0], teleop_task->_current_rot_velocity_device);
    redis_client.addEigenToReadCallback(0, DEVICE_SENSED_FORCE_KEYS[0], teleop_task->_sensed_force_device);
    redis_client.addEigenToReadCallback(0, DEVICE_SENSED_TORQUE_KEYS[0], teleop_task->_sensed_torque_device);
    redis_client.addDoubleToReadCallback(0, DEVICE_GRIPPER_POSITION_KEYS[0], teleop_task->_current_position_gripper_device);
    redis_client.addDoubleToReadCallback(0, DEVICE_GRIPPER_VELOCITY_KEYS[0], teleop_task->_current_gripper_velocity_device);

	// Objects to write to redis
	//write haptic commands
	redis_client.addEigenToWriteCallback(0, DEVICE_COMMANDED_FORCE_KEYS[0], teleop_task->_commanded_force_device);
	redis_client.addEigenToWriteCallback(0, DEVICE_COMMANDED_TORQUE_KEYS[0], teleop_task->_commanded_torque_device);
	redis_client.addDoubleToWriteCallback(0, DEVICE_COMMANDED_GRIPPER_FORCE_KEYS[0], teleop_task->_commanded_gripper_force_device);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); //Compiler en mode release
	double current_time = 0;
	double prev_time = 0;
	// double dt = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs

	while (fSimulationRunning)
	{
		// wait for next scheduled loop
		timer.waitForNextLoop();
		current_time = timer.elapsedTime() - start_time;

		// read haptic state and robot state
		redis_client.executeReadCallback(0);
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateModel();

		N_prec.setIdentity(dof,dof);
		pos_task->updateTaskModel(N_prec);

		teleop_task->UseGripperAsSwitch();
		gripper_state_prev = gripper_state;
		gripper_state = teleop_task->gripper_state;


		if(state == INIT)
		{

			pos_task->computeTorques(joint_task_torques);
			command_torques = joint_task_torques;

  			// compute homing haptic device
  			teleop_task->HomingTask();

			// if((pos_task->_desired_position - pos_task->_current_position).norm() < 0.2)
			if(teleop_task->device_homed && gripper_state && (pos_task->_desired_position - pos_task->_current_position).norm() < 0.2)
			{
				// Reinitialize controllers
				pos_task->reInitializeTask();
				teleop_task->reInitializeTask();

				pos_task->_kp = 200.0;
				pos_task->_kv = 30.0;

				state = CONTROL;
			}
		}

		else if(state == CONTROL)
		{
			Vector3d desired_position = Vector3d::Zero();
			teleop_task->computeHapticCommands3d(desired_position);

			Vector3d desired_velocity = teleop_task->_current_trans_velocity_device_RobFrame;
			// desired_position = teleop_task->_current_position_device;


			pos_task->_desired_velocity = desired_velocity;
			pos_task->_desired_position = desired_position;
			pos_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques;

		}

		// particle filter
		control_pfilter = pos_task->_unit_mass_control;
		measured_velocity_pfilter = pos_task->_current_velocity;
		measured_force_pfilter = sensed_force_moment.head(3);

		// write control torques
		redis_client.executeWriteCallback(0);
		sim->setJointTorques(robot_name, command_torques);

	}
	//// Send zero force/torque to robot and haptic device through Redis keys ////
	redis_client.setEigenMatrixJSON(DEVICE_COMMANDED_FORCE_KEYS[0], Vector3d::Zero());
	redis_client.setEigenMatrixJSON(DEVICE_COMMANDED_TORQUE_KEYS[0], Vector3d::Zero());
	redis_client.set(DEVICE_COMMANDED_GRIPPER_FORCE_KEYS[0], "0.0");

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


}

void particle_filter()
{
	// create a timer
	double pfilter_freq = 50.0;
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(pfilter_freq); //Compiler en mode release
	double current_time = 0;
	double prev_time = 0;
	// double dt = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs

	while(fSimulationRunning)
	{
		timer.waitForNextLoop();
		vector<pair<Vector3d, double>> weighted_particles = particleFilterMotionUpdate(particles, control_pfilter, measured_velocity_pfilter, measured_force_pfilter);
		particles = particleFilterResampling(weighted_particles, n_particles);

		int n_particles_center = 0;
		for(int i=0 ; i<n_particles ; i++)
		{
			if(particles[i].norm() < 1e-3)
			{
				n_particles_center++;
			}
		}
		cout << "particles at the center : " << n_particles_center << endl;
		cout << "total particles : " << particles.size() << endl;
		cout << endl;

	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Particle Filter Loop run time  : " << end_time << " seconds\n";
	std::cout << "Particle Filter Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Particle Filter Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
}

//------------------------------------------------------------------------------
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim, ForceSensorSim* force_sensor) {

	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);

	// sensed force
	Vector3d sensed_force = Vector3d::Zero();
	Vector3d sensed_moment = Vector3d::Zero();

	// create a timer
	double sim_frequency = 1000.0;
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(sim_frequency);
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	unsigned long long simulation_counter = 0;

	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// sim->setJointTorques(robot_name, command_torques);

		// integrate forward
		// double curr_time = timer.elapsedTime();
		// double loop_dt = curr_time - last_time;
		// sim->integrate(loop_dt);
		sim->integrate(1.0/sim_frequency);

		// read joint positions, velocities, update model
		// sim->getJointPositions(robot_name, robot->_q);
		// sim->getJointVelocities(robot_name, robot->_dq);
		// robot->updateKinematics();

		// read end-effector task forces from the force sensor simulation
		force_sensor->update(sim);
		force_sensor->getForceLocalFrame(sensed_force);
		force_sensor->getMomentLocalFrame(sensed_moment);
		sensed_force_moment << -sensed_force, -sensed_moment;

		// cout << sensed_force_moment.transpose() << endl;

		//update last time
		// last_time = curr_time;

		simulation_counter++;
	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
	std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Simulation Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
	switch(key) {
		case GLFW_KEY_ESCAPE:
			// exit application
			glfwSetWindowShouldClose(window,GL_TRUE);
			break;
		case GLFW_KEY_RIGHT:
			fTransXp = set;
			break;
		case GLFW_KEY_LEFT:
			fTransXn = set;
			break;
		case GLFW_KEY_UP:
			fTransYp = set;
			break;
		case GLFW_KEY_DOWN:
			fTransYn = set;
			break;
		case GLFW_KEY_A:
			fTransZp = set;
			break;
		case GLFW_KEY_Z:
			fTransZn = set;
			break;
		case GLFW_KEY_R:
			fOnOffRemote = set;
	    break;
		case GLFW_KEY_S:
			fshowCameraPose = set;
			break;
	// device input keys
			case GLFW_KEY_U:
				fMouseXp = set;
		    break;
			case GLFW_KEY_J:
				fMouseXn = set;
		    break;
			case GLFW_KEY_I:
				fMouseYp = set;
		    break;
			case GLFW_KEY_K:
				fMouseYn = set;
		    break;
			case GLFW_KEY_O:
				fMouseZp = set;
		    break;
			case GLFW_KEY_L:
				fMouseZn = set;
		    break;
			case GLFW_KEY_X:
				fMouseGripper = set;
		    break;

		default:
			break;

	}
}

//------------------------------------------------------------------------------

void mouseClick(GLFWwindow* window, int button, int action, int mods) {
	bool set = (action != GLFW_RELEASE);
	//TODO: mouse interaction with robot
	switch (button) {
		// left click pans and tilts
		case GLFW_MOUSE_BUTTON_LEFT:
			fRotPanTilt = set;
			// NOTE: the code below is recommended but doesn't work well
			// if (fRotPanTilt) {
			// 	// lock cursor
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
			// } else {
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
			// }
			break;
		// if right click: don't handle. this is for menu selection
		case GLFW_MOUSE_BUTTON_RIGHT:
			//TODO: menu
			break;
		// if middle click: don't handle. doesn't work well on laptops
		case GLFW_MOUSE_BUTTON_MIDDLE:
			break;
		default:
			break;
	}
}
