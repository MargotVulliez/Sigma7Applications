// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

#include <iostream>
#include <string>

#include <signal.h>
bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

using namespace std;
using namespace Eigen;

const string world_file = "../resources/02-open_loop_force_control/world.urdf";
// const string robot_file = "../resources/02-open_loop_force_control/panda_arm.urdf";
const string robot_file = "../resources/02-open_loop_force_control/panda_arm_hand.urdf";
const string robot_name = "PANDA";
const string camera_name = "camera_fixed";

// redis keys:
// - write:
std::string JOINT_ANGLES_KEY  = "sai2::PandaApplication::sensors::q";
std::string JOINT_VELOCITIES_KEY = "sai2::PandaApplication::sensors::dq";
// - read
const std::string TORQUES_COMMANDED_KEY  = "sai2::PandaApplication::actuators::fgc";

// - gripper
const std::string GRIPPER_MODE_KEY  = "sai2::PandaApplication::gripper::mode"; // m for move and g for graps
const std::string GRIPPER_MAX_WIDTH_KEY  = "sai2::PandaApplication::gripper::max_width";
const std::string GRIPPER_CURRENT_WIDTH_KEY  = "sai2::PandaApplication::gripper::current_width";
const std::string GRIPPER_DESIRED_WIDTH_KEY  = "sai2::PandaApplication::gripper::desired_width";
const std::string GRIPPER_DESIRED_SPEED_KEY  = "sai2::PandaApplication::gripper::desired_speed";
const std::string GRIPPER_DESIRED_FORCE_KEY  = "sai2::PandaApplication::gripper::desired_force";


RedisClient redis_client;

// simulation function prototype
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);
void simulation_dummy(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);

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

// const bool flag_simulation = false;
const bool flag_simulation = true;

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	if(!flag_simulation)
	{
		JOINT_ANGLES_KEY = "sai2::FrankaPanda::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::sensors::dq";
	}

	// start redis client
	redis_client = RedisClient();
	redis_client.connect();

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
	Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->updateKinematics();

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setCollisionRestitution(0);
	sim->setCoeffFrictionStatic(0.6);

	// read joint positions, velocities, update model
	sim->getJointPositions(robot_name, robot->_q);
	sim->getJointVelocities(robot_name, robot->_dq);
	robot->updateKinematics();

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
	GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - PandaApplications", NULL, NULL);
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
	thread sim_thread(simulation_dummy, robot, sim);
	if(flag_simulation)
	{
		// start the simulation thread first
		thread sim_thread_bis(simulation, robot, sim);
		sim_thread.swap(sim_thread_bis);
		sim_thread_bis.join();
	}


	// if(!redis_client.exists(JOINT_ANGLES_KEY))
	// {
	// 	redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, VectorXd::Zero(robot->dof()));
	// }
	// if(!redis_client.exists(JOINT_VELOCITIES_KEY))
	// {
	// 	redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, VectorXd::Zero(robot->dof()));
	// }

	// while window is open:
	while (!glfwWindowShouldClose(window))
	{
		if(!flag_simulation)
		{
			robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
			robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
			robot->updateKinematics();
		}

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
void simulation_dummy(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim) {}

//------------------------------------------------------------------------------
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim) {

	VectorXd command_torques = VectorXd::Zero(robot->dof());
	redis_client.setEigenMatrixJSON(TORQUES_COMMANDED_KEY, command_torques.head<7>());

	double kp_gripper = 50.0;
	double kv_gripper = 14.0;
	double gripper_width = (robot->_q(7) - robot->_q(8));
	double gripper_opening_speed = 0;
	double gripper_center_point = (robot->_q(7) + robot->_q(8));
	double gripper_center_point_velocity = 0;
	double gripper_constraint_force, gripper_behavior_force;

	double gripper_desired_width, gripper_desired_speed, gripper_desired_force;
	string gripper_mode = "m";

	double gripper_max_width = 0.08;

	redis_client.set(GRIPPER_MAX_WIDTH_KEY, to_string(gripper_max_width));
	redis_client.set(GRIPPER_DESIRED_WIDTH_KEY, to_string(gripper_width));
	redis_client.set(GRIPPER_DESIRED_SPEED_KEY, to_string(gripper_desired_speed));
	redis_client.set(GRIPPER_DESIRED_FORCE_KEY, to_string(0));
	redis_client.set(GRIPPER_MODE_KEY, gripper_mode);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(2000); 
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	unsigned long long simulation_counter = 0;

	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// read arm torques from redis
		command_torques.head<7>() = redis_client.getEigenMatrixJSON(TORQUES_COMMANDED_KEY);

		// compute gripper torques
		gripper_desired_width = stod(redis_client.get(GRIPPER_DESIRED_WIDTH_KEY));
		gripper_desired_speed = stod(redis_client.get(GRIPPER_DESIRED_SPEED_KEY));
		gripper_desired_force = stod(redis_client.get(GRIPPER_DESIRED_FORCE_KEY));
		gripper_mode = redis_client.get(GRIPPER_MODE_KEY);
		if(gripper_desired_width > gripper_max_width)
		{
			gripper_desired_width = gripper_max_width;
			redis_client.setCommandIs(GRIPPER_DESIRED_WIDTH_KEY, std::to_string(gripper_max_width));
			std::cout << "WARNING : Desired gripper width higher than max width. saturating to max width\n" << std::endl;
		}
		if(gripper_desired_width < 0)
		{
			gripper_desired_width = 0;
			redis_client.setCommandIs(GRIPPER_DESIRED_WIDTH_KEY, std::to_string(0));
			std::cout << "WARNING : Desired gripper width lower than 0. saturating to max 0\n" << std::endl;
		}
		if(gripper_desired_speed < 0)
		{
			gripper_desired_speed = 0;
			redis_client.setCommandIs(GRIPPER_DESIRED_SPEED_KEY, std::to_string(0));
			std::cout << "WARNING : Desired gripper speed lower than 0. saturating to max 0\n" << std::endl;
		} 
		if(gripper_desired_force < 0)
		{
			gripper_desired_force = 0;
			redis_client.setCommandIs(GRIPPER_DESIRED_FORCE_KEY, std::to_string(0));
			std::cout << "WARNING : Desired gripper speed lower than 0. saturating to max 0\n" << std::endl;
		}

		gripper_constraint_force = -400.0*gripper_center_point - 40.0*gripper_center_point_velocity;
		if(gripper_mode == "m")
		{
			// cout << "switching to motion mode\n" << endl;
			gripper_behavior_force = -kp_gripper*(gripper_width - gripper_desired_width) - kv_gripper*(gripper_opening_speed - gripper_desired_speed);
			// cout << "gripper width : " << gripper_width << endl;
			// cout << "gripper desired width : " << gripper_desired_width << endl;
			// cout << "gripper closing force : " << gripper_behavior_force << endl;
		}
		else if(gripper_mode == "g")
		{
			gripper_behavior_force = -gripper_desired_force;
		}
		else
		{
			cout << "gripper mode not recognized\n" << endl;
		}

		command_torques(7) = gripper_constraint_force + gripper_behavior_force;
		command_torques(8) = gripper_constraint_force - gripper_behavior_force;

		// set torques to simulation
		sim->setJointTorques(robot_name, command_torques);

		// integrate forward
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time; 
		sim->integrate(loop_dt);

		// read joint positions, velocities, update model
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateKinematics();
		gripper_center_point = (robot->_q(7) + robot->_q(8));
		gripper_width = (robot->_q(7) - robot->_q(8));
		gripper_center_point_velocity = (robot->_dq(7) + robot->_dq(8));
		gripper_opening_speed = (robot->_dq(7) - robot->_dq(8));

		// cout << "joint 7 : " << robot->_q(7) << endl;
		// cout << "joint 8 : " << robot->_q(8) << endl;

		// write new robot state to redis
		redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, robot->_q.head<7>());
		redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, robot->_dq.head<7>());
		redis_client.set(GRIPPER_CURRENT_WIDTH_KEY, to_string(gripper_width));

		//update last time
		last_time = curr_time;

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

