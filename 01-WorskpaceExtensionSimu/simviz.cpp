// This example simulates the remote control of a Kuka iiwa interacting with different rigid
// objects (boxe, hole, edge, complex shape). The workspace extension algorithm is tested
// with this application in translation and rotation.
// The robot is equipped with a spherical or rectangular tool. The haptic interaction are
// estimated or sensed in the virtual task environment. This simulation runs with the
// application controller (state machine, haptic device's and remote robot's controllers).

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

#include "force_sensor/ForceSensorSim.h" // Add force sensor simulation and display classes
#include "force_sensor/ForceSensorDisplay.h"

#include <iostream>
#include <fstream>
#include <string>

#include <signal.h>
bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

using namespace std;
using namespace Eigen;
using namespace chai3d;

const string world_file = "../resources/01-WorskpaceExtensionSimu/world.urdf";
const string robot_file = "../resources/01-WorskpaceExtensionSimu/kuka_iiwa.urdf";
const string robot_name = "Kuka-IIWA";

//Introduce a massless/stiff virtual avatar
//const string proxy_file = "../resources/01-WorskpaceExtensionSimu/proxy.urdf";
//const string proxy_name = "proxy";
const string proxy_file = "../resources/01-WorskpaceExtensionSimu/proxy6d.urdf";
const string proxy_name = "proxy6d";

const string camera_name = "camera";
const string link_name = "link6"; //robot end-effector
const Vector3d pos_in_link = Vector3d(0.0,0.0,0.04); //Position of the control point in the end-effector frame
Affine3d transform_in_link = Affine3d::Identity();

VectorXd f_task = VectorXd::Zero(6);
Vector3d pos_rob_model = Vector3d::Zero();
cVector3d pos_rob_model_chai;

// redis keys:
// - write (from simulation or robot sensors):
string JOINT_ANGLES_KEY  = "sai2::Sigma7Applications::sensors::q";
string JOINT_VELOCITIES_KEY  = "sai2::Sigma7Applications::sensors::dq";
string FORCE_SENSED_KEY = "sai2::Sigma7Applications::sensors::force_task_sensed";

// - read (from haptic device command input):
string JOINT_TORQUES_COMMANDED_KEY = "sai2::Sigma7Applications::actuators::commanded_torque_robot";

//Proxy related keys (with massless/stiff virtual avatar)
string JOINT_TORQUES_PROXY_KEY = "sai2::Sigma7Applications::actuators::commanded_torque_proxy";
string JOINT_ANGLES_PROXY_KEY  = "sai2::Sigma7Applications::sensors::q_proxy";
string JOINT_VELOCITIES_PROXY_KEY  = "sai2::Sigma7Applications::sensors::dq_proxy";

//Proxy related keys (with Finger proxy algorithm)
// string TRANS_VELOCITY_KEY =  "sai2::Sigma7Applications::sensors::commanded_velocity";
// string POSITION_COMMANDED_KEY = "sai2::Sigma7Applications::sensors::commanded_position";
// string PROXY_FORCE_KEY = "sai2::Sigma7Applications::controller:Fproxy_simu";
// string PROXY_COMPUTATION_KEY = "sai2::Sigma7Applications::sensors::proxy_computation";

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


RedisClient redis_client;

// simulation function prototype
void simulation(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* avatar, Simulation::Sai2Simulation* sim, Sai2Graphics::Sai2Graphics* graphics);
void simulation_dummy(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* avatar, Simulation::Sai2Simulation* sim, Sai2Graphics::Sai2Graphics* graphics);

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;
// flag for enabling/disabling remote task
bool fOnOffRemote = false;
int k=0;

// const bool flag_simulation = false;
const bool flag_simulation = true;

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;
	cout << "Keyboard Options:" << endl << endl;
    cout << "[1] - Enable-Disable remote task" << endl;

	if(!flag_simulation)
	{
		// JOINT_ANGLES_KEY = "sai2::FrankaPanda::sensors::q";
		// JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::sensors::dq";
	}

	// start redis client
	redis_client = RedisClient();
	redis_client.connect();

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
	Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	VectorXd initial_q(robot->dof());
	initial_q << 0.0, -20.0, 0.0, 30.0, 0.0, -50.0, 0.0; 
	initial_q *= M_PI/180.0;

	auto avatar = new Sai2Model::Sai2Model(proxy_file, false);
	VectorXd initial_q_proxy(avatar->dof());
	initial_q_proxy << -0.565, 0.0, 0.954, 0.0, 0.0, 0.0;

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setCollisionRestitution(0);
	sim->setCoeffFrictionStatic(0.2);

	// read joint positions, velocities, update model robot
	sim->setJointPositions(robot_name, initial_q);
	sim->getJointPositions(robot_name, robot->_q);
	sim->getJointVelocities(robot_name, robot->_dq);

	// read joint positions, velocities, update model proxy
	sim->setJointPositions(proxy_name, initial_q_proxy);
	sim->getJointPositions(proxy_name, avatar->_q);
	sim->getJointVelocities(proxy_name, avatar->_dq);

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
	thread sim_thread(simulation_dummy, robot, avatar, sim, graphics);
	if(flag_simulation)
	{
		// start the simulation thread first
		thread sim_thread_bis(simulation, robot, avatar, sim, graphics);
		sim_thread.swap(sim_thread_bis);
		sim_thread_bis.join();
	}

	// while window is open:
	while (!glfwWindowShouldClose(window))
	{
		if(!flag_simulation)
		{
			// robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
			// robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
			// robot->updateKinematics();
		}

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		graphics->updateGraphics(proxy_name, avatar);
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
		cam_up_axis << 1.0, 0.0, 1.0; //TODO: there might be a better way to do this
		Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
		cam_roll_axis.normalize();
		Eigen::Vector3d cam_lookat_axis = camera_lookat;
		cam_lookat_axis.normalize();
		if (fTransXp) {
			camera_pos = camera_pos + 0.05*cam_roll_axis;
			camera_lookat = camera_lookat + 0.05*cam_roll_axis;
		}
		if (fTransXn) {
			camera_pos = camera_pos - 0.05*cam_roll_axis;
			camera_lookat = camera_lookat - 0.05*cam_roll_axis;
		}
		if (fTransYp) {
			// camera_pos = camera_pos + 0.05*cam_lookat_axis;
			camera_pos = camera_pos + 0.05*cam_up_axis;
			camera_lookat = camera_lookat + 0.05*cam_up_axis;
		}
		if (fTransYn) {
			// camera_pos = camera_pos - 0.05*cam_lookat_axis;
			camera_pos = camera_pos - 0.05*cam_up_axis;
			camera_lookat = camera_lookat - 0.05*cam_up_axis;
		}
		if (fTransZp) {
			camera_pos = camera_pos + 0.1*cam_depth_axis;
			camera_lookat = camera_lookat + 0.1*cam_depth_axis;
		}	    
		if (fTransZn) {
			camera_pos = camera_pos - 0.1*cam_depth_axis;
			camera_lookat = camera_lookat - 0.1*cam_depth_axis;
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

	// Enable/Disable the remote task
			// if (k==0 && fOnOffRemote)
			// {
			// 	if (0 == remote_enabled) { remote_enabled = 1; }
			// 	else if (1 == remote_enabled) { remote_enabled = 0; }
			// 	k++;
			// }
			// if(k!=0)
			// {
			// 	if (k<=20) {k++; }
			// 	else {k=0; }
			// }
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();

	// destroy context
	glfwDestroyWindow(window);

	// terminate
	glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------
void simulation_dummy(Sai2Model::Sai2Model* robot,Sai2Model::Sai2Model* avatar, Simulation::Sai2Simulation* sim, Sai2Graphics::Sai2Graphics* graphics) {}

//------------------------------------------------------------------------------
void simulation(Sai2Model::Sai2Model* robot,Sai2Model::Sai2Model* avatar, Simulation::Sai2Simulation* sim, Sai2Graphics::Sai2Graphics* graphics) {

	VectorXd command_torques = VectorXd::Zero(robot->dof());
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	VectorXd command_torques_proxy = VectorXd::Zero(avatar->dof());
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_PROXY_KEY, command_torques_proxy);

	// Vector3d command_trans_velocity = Vector3d::Zero();
	// Vector3d command_position = Vector3d::Zero();
	// cVector3d proxy_forces_chai;
	// Vector3d proxy_forces = Vector3d::Zero();
	// cVector3d command_pos_chai;
	// cVector3d command_trans_vel_chai;

	// redis_client.setEigenMatrixJSON(TRANS_VELOCITY_KEY, command_trans_velocity);

	// int proxy_computation=0;
	// redis_client.set(PROXY_COMPUTATION_KEY, to_string(proxy_computation));

	// Add force sensor to the end-effector
	auto force_sensor = new ForceSensorSim(robot_name, link_name, transform_in_link, robot);
	Vector3d sensed_force = Vector3d::Zero();
	Vector3d sensed_moment = Vector3d::Zero();
	// force_sensor->removeSpike(5.0);
	// force_sensor->enableFilter(0.02);

	// logging
	// fstream logger;
	// logger.open("log.csv", std::ios::out);
	// logger << "time, force, torque, tx, ty, tz" << endl;
	
	// Create proxy computation
	// auto proxy = new chai3d::cAlgorithmFingerProxy();
	// proxy->setProxyRadius(0.01);
	// robot->position(pos_rob_model, link_name, pos_in_link);
	// pos_rob_model_chai = convertEigenToChaiVector(pos_rob_model);
	// proxy->initialize(graphics->_world, pos_rob_model_chai);

	// Initialize Finger Proxy algorithm
	// robot->updateKinematics();
	// robot->position(pos_rob_model, link_name, pos_in_link);
	// pos_rob_model_chai = convertEigenToChaiVector(pos_rob_model);
	// proxy->initialize(graphics->_world, pos_rob_model_chai);
	// redis_client.setEigenMatrixJSON(POSITION_COMMANDED_KEY, pos_rob_model);

	// create a timer
	double sim_frequency = 1000.0;
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(sim_frequency); 
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	unsigned long long simulation_counter = 0;

	bool first_iteration=true;


	while (fSimulationRunning) {

		fTimerDidSleep = timer.waitForNextLoop();

		// read arm torques from redis
		command_torques = redis_client.getEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY);

		//read proxy torques from redis
		command_torques_proxy = redis_client.getEigenMatrixJSON(JOINT_TORQUES_PROXY_KEY);

 		// read set position/velocity
		// command_trans_velocity = redis_client.getEigenMatrixJSON(TRANS_VELOCITY_KEY);
		// command_position = redis_client.getEigenMatrixJSON(POSITION_COMMANDED_KEY);

		// read proxy computation
		// proxy_computation = stoi(redis_client.get(PROXY_COMPUTATION_KEY));

		// set torques to simulation
		sim->setJointTorques(robot_name, command_torques);
		sim->setJointTorques(proxy_name, command_torques_proxy);

		// integrate forward
		// double curr_time = timer.elapsedTime();
		// double loop_dt = curr_time - last_time; 
		sim->integrate(1/sim_frequency);

		// read joint positions, velocities, update model robot
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateKinematics();

		// read joint positions, velocities, update model proxy
		sim->getJointPositions(proxy_name, avatar->_q);
		sim->getJointVelocities(proxy_name, avatar->_dq);
		avatar->updateKinematics();

		// write new robot state to redis
		redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, robot->_q);
		redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, robot->_dq);

		// write new proxy state to redis
		redis_client.setEigenMatrixJSON(JOINT_ANGLES_PROXY_KEY, avatar->_q);
		redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_PROXY_KEY, avatar->_dq);

		////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// read end-effector task forces from the force sensor simulation
		force_sensor->update(sim);
		force_sensor->getForce(sensed_force);
		force_sensor->getMoment(sensed_moment);
		f_task << sensed_force, sensed_moment;
		// write task force in redis
		redis_client.setEigenMatrixJSON(FORCE_SENSED_KEY,f_task);


		// if (simulation_counter % 33 == 0) {
		// 	logger << sim->_world->m_time
		// 		<< ", " << sensed_force.norm()
		// 		<< ", " << sensed_moment.norm()
		// 		<< ", " << sensed_moment[0]
		// 		<< ", " << sensed_moment[1]
		// 		<< ", " << sensed_moment[2]
		// 		<< "\n";
		// }

		// // Compute haptic feedback from proxy
		// if (proxy_computation==1)
		// {
		// 	//Reset initial position of the proxy
		// 	if (first_iteration)
		// 	{
		// 	robot->position(pos_rob_model, link_name, pos_in_link);
		// 	pos_rob_model_chai = convertEigenToChaiVector(pos_rob_model);
		// 	proxy->setProxyGlobalPosition(pos_rob_model_chai);

		// 	first_iteration=false;
		// 	}

		// 	// Compute the force feedback from finger proxy 
		// 	command_pos_chai = convertEigenToChaiVector(command_position);
		// 	command_trans_vel_chai = convertEigenToChaiVector(command_trans_velocity);

		// 	proxy_forces_chai = proxy->computeForces(command_pos_chai, command_trans_vel_chai);
		// 	proxy_forces = convertChaiToEigenVector(proxy_forces_chai);
		// }
		// else
		// {
		// 	bool first_iteration=true;
		// 	proxy_forces.setZero();
		// }


		// cVector3d devicePos = proxy->getDeviceGlobalPosition();
		// cVector3d  proxyPos = proxy->getProxyGlobalPosition();

		//  std::cout << "commanded position" << command_position << endl;
		//  std::cout << "device position" << devicePos << endl;
		//  std::cout << "proxy position" << proxyPos << endl;
		// std::cout << "haptic velocity" << command_trans_velocity << endl;
		// std::cout << "proxy force" << proxy_forces << endl;

		// redis_client.setEigenMatrixJSON(PROXY_FORCE_KEY,proxy_forces);

		//update last time
		// last_time = curr_time;

		simulation_counter++;
	}

	// logger.close();
	
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

