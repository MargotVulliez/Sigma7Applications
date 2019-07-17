// This example tests the haptic device driver and the open-loop bilateral teleoperation controller.

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "haptic_tasks/OpenLoopTeleop.h"
#include "chai3d.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

#include "force_sensor/ForceSensorSim.h" // Add force sensor simulation and display classes
#include "force_sensor/ForceSensorDisplay.h"

#include <iostream>
#include <string>

#include <signal.h>
bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

using namespace std;
using namespace Eigen;
using namespace chai3d;

const string world_file = "../resources/06-HapticGuidanceFranka/world.urdf";
const string robot_file = "../resources/06-HapticGuidanceFranka/panda_arm.urdf";
const string robot_name = "panda";
const string camera_name = "camera";
const string link_name = "link7"; //robot end-effector
Affine3d transform_in_link = Affine3d::Identity();

cVector3d convertEigenToChaiVector( Eigen::Vector3d a_vec )
{
    double x = a_vec(0);
    double y = a_vec(1);
    double z = a_vec(2);
    return cVector3d(x,y,z);
}

cMatrix3d convertEigenToChaiRotation( Eigen::Matrix3d a_mat )
{
    return cMatrix3d( a_mat(0,0), a_mat(0,1), a_mat(0,2), a_mat(1,0), a_mat(1,1), a_mat(1,2), a_mat(2,0), a_mat(2,1), a_mat(2,2) );
}

VectorXd f_task = VectorXd::Zero(6);

// redis keys:
// - write (from simulation or robot sensors):
string JOINT_ANGLES_KEY  = "sai2::Sigma7Applications::sensors::q";
string JOINT_VELOCITIES_KEY  = "sai2::Sigma7Applications::sensors::dq";
string FORCE_SENSED_KEY = "sai2::Sigma7Applications::sensors::force_task_sensed";
string GUIDANCE_PLANE_NORMAL_KEY = "sai2::Sigma7Applications::simulation::guidance_plane_normal_vec";
string GUIDANCE_PLANE_POINT_KEY = "sai2::Sigma7Applications::simulation::guidance_plane_point";
string ROBOT_HAPTIC_ROTATION_MATRIX_KEY = "sai2::Sigma7Applications::simulation::robot_haptic_rotation_matrix_guidance";
string ROBOT_HAPTIC_FRAME_TRANSLATION_KEY = "sai2::Sigma7Applications::simulation::robot_haptic_frame_translation_guidance";
string GUIDANCE_LINE_FIRST_POINT = "sai2::Sigma7Applications::simulation::line_guidance_first_point";
string GUIDANCE_LINE_SECOND_POINT = "sai2::Sigma7Applications::simulation::line_guidance_second_point";
const string GUIDANCE_PLANE_ENABLE = "sai2::Sigma7Applications::simulation::_enable_plane_guidance_3D";
// const string GUIDANCE_LINE_ENABLE = "sai2::Sigma7Applications::simulation::_enable_line_guidance_3D";
string GUIDANCE_SCALE_FACTOR = "sai2::Sigma7Applications::simulation::_guidance_scale_factor";

// - read (from haptic device command input):
string JOINT_TORQUES_COMMANDED_KEY = "sai2::Sigma7Applications::actuators::torque_joint_robot";

// initialize variables that are redis keys
Eigen::Vector3d point_one = Eigen::Vector3d::Zero();							// first point defining the line
Eigen::Vector3d point_two = Eigen::Vector3d::Zero();							// second point defining the line
Eigen::Matrix3d _Rotation_Matrix_DeviceToRobot = Eigen::Matrix3d::Identity(); 	// rotation matrix between haptic global and robot global
Eigen::Vector3d _Translation_DeviceToRobot = Eigen::Vector3d::Zero(); 			// translation between device to robot home positions
Eigen::Vector3d normal_vec = Eigen::Vector3d::Zero();							// plane normal vector
Eigen::Vector3d plane_point = Eigen::Vector3d::Zero();							// plane origin (initial end-effector position) Eigen
int plane_enable = 0;															// controls plane visualization
int line_enable = 0;															// controls line visualization
int scale_factor = 1;															// scale factor from haptic to robot workspace

RedisClient redis_client;

// create graphics object
auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
// auto line_object = new chai3d::cShapeLine();
auto plane_object = new chai3d::cMesh();

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
	Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);

	// create a camera
	auto camera = new cCamera(graphics->_world);
	// add the camera to the world
	graphics->_world->addChild(camera);
	camera->setUseMultipassTransparency(true);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	
	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setCollisionRestitution(0);
	sim->setCoeffFrictionStatic(0.2);

	// read joint positions, velocities, update model
	// sim->setJointPositions(robot_name, initial_q);
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
	thread sim_thread(simulation_dummy, robot, sim);
	if(flag_simulation)
	{
		// start the simulation thread first
		thread sim_thread_bis(simulation, robot, sim);
		sim_thread.swap(sim_thread_bis);
		sim_thread_bis.join();
	}

	auto handler = new cHapticDeviceHandler();
	
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
		cam_up_axis << -1.0, 0.0, 1.0; //TODO: there might be a better way to do this
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

	// // Enable/Disable the remote task
	// 		if (k==0 && fOnOffRemote)
	// 		{
	// 			if (0 == remote_enabled) { remote_enabled = 1; }
	// 			else if (1 == remote_enabled) { remote_enabled = 0; }
	// 			k++;
	// 		}
	// 		if(k!=0)
	// 		{
	// 			if (k<=20) {k++; }
	// 			else {k=0; }
	// 		}
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
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	// Add force sensor to the end-effector
	auto force_sensor = new ForceSensorSim(robot_name, link_name, transform_in_link, robot);
	Vector3d sensed_force = Vector3d::Zero();
	Vector3d sensed_moment = Vector3d::Zero();
	force_sensor->enableFilter(0.016);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(2000); 
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	unsigned long long simulation_counter = 0;

	// variables used to make only one plane and one line at a time
	int plane_counter = 1;
	int line_counter = 1;
	plane_point.setZero();
	normal_vec.setZero();
	_Rotation_Matrix_DeviceToRobot.setIdentity();
	_Translation_DeviceToRobot.setZero();

	redis_client.set(GUIDANCE_PLANE_ENABLE, to_string(1));
	redis_client.set(GUIDANCE_SCALE_FACTOR, to_string(2.0));
	redis_client.setEigenMatrixJSON(GUIDANCE_PLANE_POINT_KEY, plane_point);
	redis_client.setEigenMatrixJSON(GUIDANCE_PLANE_NORMAL_KEY, normal_vec);
	redis_client.setEigenMatrixJSON(ROBOT_HAPTIC_ROTATION_MATRIX_KEY, _Rotation_Matrix_DeviceToRobot);
	redis_client.setEigenMatrixJSON(ROBOT_HAPTIC_FRAME_TRANSLATION_KEY, _Translation_DeviceToRobot);


	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// read arm torques from redis
		command_torques = redis_client.getEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY);

		/********************************** Plane and Line Visualization **********************************/

		plane_enable = stoi(redis_client.get(GUIDANCE_PLANE_ENABLE));
		// line_enable = stoi(redis_client.get(GUIDANCE_LINE_ENABLE));;

		// // if there's no plane or line (destructor was used) then make a new one
		// if (plane_object == NULL) {

		// 	plane_object = new cMesh();

		// } else if (line_object == NULL) {

		// 	line_object = new cShapeLine();

		// }

		// plane visualization
		if (plane_enable == 1 && plane_counter == 1) {
			
			// local variable initialization for the plane
			chai3d::cMatrix3d PlaneRotation;	// rotation matrix from world to guidance plane
			chai3d::cVector3d PlanePosition;	// plane origin (initial end-effector position) chai 
			Eigen::Vector3d FloorAxis;			// world plane normal vector

			// read scaling factor from redis
			scale_factor = stoi(redis_client.get(GUIDANCE_SCALE_FACTOR));

			// read plane normal and plane point from redis
			normal_vec = redis_client.getEigenMatrixJSON(GUIDANCE_PLANE_NORMAL_KEY);
			plane_point = redis_client.getEigenMatrixJSON(GUIDANCE_PLANE_POINT_KEY);


			// read haptic to robot translation vector and rotation matrix
			_Rotation_Matrix_DeviceToRobot = redis_client.getEigenMatrixJSON(ROBOT_HAPTIC_ROTATION_MATRIX_KEY);
			_Translation_DeviceToRobot = redis_client.getEigenMatrixJSON(ROBOT_HAPTIC_FRAME_TRANSLATION_KEY);

			// define this in the haptic space; normal vector of the floor plane for the virtual plane visualization
			FloorAxis << 0, 0, 1;

			// get plane and point vectors defined in the haptic space and convert them to the robot space	
			normal_vec = _Rotation_Matrix_DeviceToRobot * normal_vec;	// normal vec in robot space
			FloorAxis = _Rotation_Matrix_DeviceToRobot * FloorAxis;		// floor axis in robot space
			
			// Adjust plane point position from device workspace to the robot space (translate and rotate)
			plane_point = scale_factor * _Rotation_Matrix_DeviceToRobot * plane_point + _Translation_DeviceToRobot;

			// plane rotation matrix and position in chai3d vector/matrix form
			PlaneRotation = convertEigenToChaiRotation((Quaterniond().setFromTwoVectors(FloorAxis, normal_vec)).toRotationMatrix());
			PlanePosition = convertEigenToChaiVector(plane_point);

			// create the plane, set the color and transparency
			chai3d::cCreatePlane(plane_object, 3.0, 3.0, PlanePosition, PlaneRotation);
			plane_object->m_material->setColorf(0.69, 0.93, 0.93);
			plane_object->setTransparencyLevel(0.3, false, false, false);

			// add the plane to the world
			graphics->_world->addChild(plane_object);

			// increment the plane counter so we don't break into this statement continuously
			plane_counter++;

		}
		// // line visualization
		// else if (line_enable == 1 && plane_enable == 0 && line_counter == 1) {
			
		// 	// local variable initialization for the line
		// 	chai3d::cVector3d point_one_chai;	// chai vector for the first point
		// 	chai3d::cVector3d point_two_chai;	// chai vector for the second point
		// 	Eigen::Vector3d point_one_scaled;	// point one scaled for better visualization
		// 	Eigen::Vector3d point_two_scaled;	// point two scaled for better visualization
		// 	double line_scaling = 10;			// scaling factor for line visualization

		// 	// read scaling factor from redis
		// 	scale_factor = stoi(redis_client.get(GUIDANCE_SCALE_FACTOR));

		// 	// grab values from redis set in the controller cpp for the two points defining the line
		// 	point_one = redis_client.getEigenMatrixJSON(GUIDANCE_LINE_FIRST_POINT);
		// 	point_two = redis_client.getEigenMatrixJSON(GUIDANCE_LINE_SECOND_POINT);

		// 	// read haptic to robot translation vector and rotation matrix
		// 	_Rotation_Matrix_DeviceToRobot = redis_client.getEigenMatrixJSON(ROBOT_HAPTIC_ROTATION_MATRIX_KEY);
		// 	_Translation_DeviceToRobot = redis_client.getEigenMatrixJSON(ROBOT_HAPTIC_FRAME_TRANSLATION_KEY);

		// 	// define the points in the robot workspace from the haptic workspace
		// 	point_one = scale_factor * _Rotation_Matrix_DeviceToRobot * point_one + _Translation_DeviceToRobot;
		// 	point_two = scale_factor * _Rotation_Matrix_DeviceToRobot * point_two + _Translation_DeviceToRobot;

		// 	// scale these points in the robot workspace for visualization
		// 	point_one_scaled = line_scaling * (point_one - point_two) + point_one;
		// 	point_two_scaled = line_scaling * (point_two - point_one) + point_two; 

		// 	// turn the two points into chai vectors
		// 	point_one_chai = convertEigenToChaiVector(point_one_scaled);
		// 	point_two_chai = convertEigenToChaiVector(point_two_scaled);

		// 	// create the line using the two scaled chai vectors
		// 	*line_object = cShapeLine(point_one_chai, point_two_chai);

		// 	// add the line to the world
		// 	graphics->_world->addChild(line_object);

		// 	line_counter++;

		// } else {
			
		// 	if (plane_counter > 1 && line_enable == 1) {
		// 		// reset the counters
		// 		plane_counter = 1;
		// 		line_counter++;

		// 		// remove the plane from visualization and make the pointer null
		// 		plane_object->~cMesh();
		// 		plane_object = NULL;

		// 	} else if (line_counter > 1 && plane_enable == 1) {
		// 		// reset the counters
		// 		plane_counter++;
		// 		line_counter = 1;
		// 		// remove the line from visualization and make the pointer null
		// 		line_object->~cShapeLine();
		// 		line_object = NULL;
		// 	}

		// } 

		// } else if (line_counter > 1 && plane_enable == 1) {
			
		// 	cout << "reset line" << endl;
		// 	// reset the counters
		// 	plane_counter = 1;
		// 	line_counter = 1;

		// 	// remove the line from visualization and make the pointer null
		// 	line_object->~cShapeLine();
		// 	line_object = NULL;

		// }

		/******************************** End Plane and Line Visualization *********************************/

		sim->setJointTorques(robot_name, command_torques);

		// integrate forward
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time; 
		sim->integrate(loop_dt);

		// read joint positions, velocities, update model
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateKinematics();


		// write new robot state to redis
		redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, robot->_q);
		redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, robot->_dq);
		
		////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// read end-effector task forces from the force sensor simulation
		force_sensor->update(sim);
		force_sensor->getForce(sensed_force);
		force_sensor->getMoment(sensed_moment);
		f_task << sensed_force, sensed_moment;
		// write task force in redis
		redis_client.setEigenMatrixJSON(FORCE_SENSED_KEY,f_task);

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
